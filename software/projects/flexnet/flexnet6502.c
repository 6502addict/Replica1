#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdint.h>
#include <stdbool.h>
#include "ff.h"
#include "flexnet.h"
#include "flexnet_hw.h"

#define SECSIZE     256
#define LF          0x0a
#define CR          0x0d
#define ACK         0x06
#define NAK         0x15
#define ESC         0x1b

// timeout per state in ms
#define TO_RREAD    500
#define TO_RWRITE   500
#define TO_RDIR     5000
#define TO_RLIST    5000
#define TO_MOUNT    2000
#define TO_CD       2000
#define TO_SYNC     500
#define TO_PARAM    2000

// ── state machine ─────────────────────────────────────────────────────────────

typedef enum {
    FS_IDLE,
    FS_SYNC,
    FS_RREAD,
    FS_RWRITE,
    FS_RDIR,
    FS_RLIST,
    FS_MOUNT,
    FS_CD,
} FlexState;

static FlexState state     = FS_IDLE;
static bool      timed_out = false;

static const char *state_name(FlexState s) {
    switch (s) {
    case FS_IDLE:   return "IDLE";
    case FS_SYNC:   return "SYNC";
    case FS_RREAD:  return "RREAD";
    case FS_RWRITE: return "RWRITE";
    case FS_RDIR:   return "RDIR";
    case FS_RLIST:  return "RLIST";
    case FS_MOUNT:  return "MOUNT";
    case FS_CD:     return "CD";
    }
    return "?";
}

static FIL      dsk_fil;
static bool     dsk_open  = false;
static uint8_t  bloc[SECSIZE];
static uint8_t  nbtrk, nbsec;
static uint32_t track0l;
static bool     ready  = false;
static bool     rdonly = false;
static char     param[128];
static char     curdir[256];

// ── UART helpers ──────────────────────────────────────────────────────────────

// returns -1 on timeout, byte value on success
static int flex_getc_to(uint32_t timeout_ms) {
    uint32_t count = timeout_ms * FLEX_TIMEOUT_SCALE;
    while (!(MC6850_STAT & MC6850_RDRF)) {
        if (count-- == 0) {
            timed_out = true;
            printf("flexnet: timeout in state %s\n", state_name(state));
            return -1;
        }
    }
    return MC6850_DATA;
}

// blocking getc used only in IDLE (waiting for next command — no timeout needed)
static uint8_t flex_getc(void) {
    while (!(MC6850_STAT & MC6850_RDRF))
        ;
    return MC6850_DATA;
}

static void flex_putc(uint8_t c) {
    while (!(MC6850_STAT & MC6850_TDRE))
        ;
    MC6850_DATA = c;
}

static void flex_puts(const char *s) { while (*s) flex_putc((uint8_t)*s++); }

// ── misc helpers ──────────────────────────────────────────────────────────────

static int ts2blk(uint8_t ntrk, uint8_t nsec) {
    if (ntrk > nbtrk || nsec > nbsec || (nsec == 0 && ntrk != 0)) return -1;
    if (ntrk == 0) return nsec == 0 ? 0 : nsec - 1;
    return (int)(track0l + (ntrk - 1) * nbsec + nsec - 1);
}

static int checksum(uint8_t *data) {
    int chks = 0;
    for (int i = 0; i < 256; i++) chks += data[i];
    return chks & 0xFFFF;
}

static bool is_dsk(const char *name) {
    int len = strlen(name);
    return len > 4 && strcasecmp(name + len - 4, ".DSK") == 0;
}

// getparam with timeout — returns false on timeout
static bool getparam_to(uint32_t timeout_ms) {
    int c, i = 0;
    while (1) {
        c = flex_getc_to(timeout_ms);
        if (c < 0) { param[0] = 0; return false; }
        if (c == CR) break;
        if (i < 127) param[i++] = (char)c;
    }
    param[i] = 0;
    return true;
}

// ── disk image open/close ─────────────────────────────────────────────────────

static bool load_dsk(const char *name) {
    FILINFO  fno;
    UINT     br;
    uint32_t nb_sectors;

    if (dsk_open) { f_close(&dsk_fil); dsk_open = false; }

    if (f_stat(name, &fno) != FR_OK) {
        printf("flexnet: %s not found\n", name);
        return false;
    }
    nb_sectors = fno.fsize / SECSIZE;
    rdonly = (fno.fattrib & AM_RDO) != 0;

    BYTE mode = FA_READ | (rdonly ? 0 : FA_WRITE);
    if (f_open(&dsk_fil, name, mode) != FR_OK) {
        printf("flexnet: open failed\n");
        return false;
    }
    dsk_open = true;

    f_lseek(&dsk_fil, SECSIZE * 2);
    if (f_read(&dsk_fil, bloc, SECSIZE, &br) != FR_OK || br != SECSIZE) {
        f_close(&dsk_fil); dsk_open = false;
        return false;
    }
    if (bloc[0x26] == 0 || bloc[0x27] == 0) {
        printf("flexnet: not a valid Flex disk\n");
        f_close(&dsk_fil); dsk_open = false;
        return false;
    }
    nbtrk = bloc[0x26];
    nbsec = bloc[0x27];
    if ((uint32_t)(nbtrk + 1) * nbsec == nb_sectors)
        track0l = nbsec;
    else {
        track0l = nb_sectors - nbtrk * nbsec;
        if (track0l > nbsec) { track0l = nbsec; nbtrk++; }
    }
    printf("flexnet: %s %lu sectors %u trk %u sec%s\n",
        name, nb_sectors, nbtrk + 1, nbsec, rdonly ? " RO" : "");
    ready = true;
    return true;
}

// ── S command : send sector to FLEX ──────────────────────────────────────────

static void sndblk(void) {
    state = FS_RREAD;
    timed_out = false;

    int ntrk = flex_getc_to(TO_RREAD); if (ntrk < 0) goto abort_send;
    int nsec = flex_getc_to(TO_RREAD); if (nsec < 0) goto abort_send;

    {
        int  blk = ts2blk((uint8_t)ntrk, (uint8_t)nsec);
        int  pos = SECSIZE * blk;
        UINT br  = 0;

        if (!ready || pos < 0) {
            memset(bloc, 0, SECSIZE);
        } else {
            FRESULT fr = f_lseek(&dsk_fil, (FSIZE_t)pos);
            fr = f_read(&dsk_fil, bloc, SECSIZE, &br);
            if (fr != FR_OK || br != SECSIZE) memset(bloc, 0, SECSIZE);
        }
        int chks = checksum(bloc);
        for (int i = 0; i < 256; i++) flex_putc(bloc[i]);
        flex_putc((chks >> 8) & 0xFF);
        flex_putc(chks & 0xFF);
        // wait for ACK/NAK with timeout
        flex_getc_to(TO_RREAD);
    }
    state = FS_IDLE;
    return;

abort_send:
    memset(bloc, 0, SECSIZE);
    for (int i = 0; i < 256; i++) flex_putc(0);
    flex_putc(0xFF); flex_putc(0xFF);   // checksum mismatch guaranteed
    state = FS_IDLE;
}

// ── R command : receive sector from FLEX ─────────────────────────────────────

static void rcvblk(void) {
    state = FS_RWRITE;
    timed_out = false;

    int ntrk = flex_getc_to(TO_RWRITE); if (ntrk < 0) goto abort_recv;
    int nsec = flex_getc_to(TO_RWRITE); if (nsec < 0) goto abort_recv;

    {
        int  pos = SECSIZE * ts2blk((uint8_t)ntrk, (uint8_t)nsec);
        UINT bw;

        for (int i = 0; i < 256; i++) {
            int c = flex_getc_to(TO_RWRITE);
            if (c < 0) goto abort_recv;
            bloc[i] = (uint8_t)c;
        }
        int msb = flex_getc_to(TO_RWRITE); if (msb < 0) goto abort_recv;
        int lsb = flex_getc_to(TO_RWRITE); if (lsb < 0) goto abort_recv;

        bool ok = false;
        if (checksum(bloc) == msb * 256 + lsb && ready && !rdonly && pos >= 0 &&
            f_lseek(&dsk_fil, (FSIZE_t)pos) == FR_OK &&
            f_write(&dsk_fil, bloc, SECSIZE, &bw) == FR_OK && bw == SECSIZE)
            ok = true;

        flex_putc(ok ? ACK : NAK);
    }
    state = FS_IDLE;
    return;

abort_recv:
    flex_putc(NAK);
    state = FS_IDLE;
}

// ── A command : list .DSK files ───────────────────────────────────────────────

static void lstdsk(void) {
    DIR     dir;
    FILINFO fno;
    bool    aborted = false;

    state = FS_RDIR;
    if (!getparam_to(TO_RDIR)) goto abort_dir;

    flex_putc(CR); flex_putc(LF);

    if (f_opendir(&dir, ".") != FR_OK) { flex_putc(ACK); state = FS_IDLE; return; }
    while (f_readdir(&dir, &fno) == FR_OK && fno.fname[0]) {
        if (!is_dsk(fno.fname)) continue;
        if (param[0] && param[0] != '*' &&
            strncasecmp(fno.fname, param, strlen(param)) != 0) continue;
        int c = flex_getc_to(TO_RDIR);
        if (c < 0) { aborted = true; break; }
        if (c != ' ') { aborted = true; break; }
        flex_puts(fno.fname); flex_putc(CR); flex_putc(LF);
    }
    if (!aborted) flex_getc_to(TO_RDIR);
    f_closedir(&dir);
    flex_putc(ACK);
    state = FS_IDLE;
    return;

abort_dir:
    flex_putc(ACK);
    state = FS_IDLE;
}

// ── I command : list subdirectories ──────────────────────────────────────────

static void lstdir(void) {
    DIR     dir;
    FILINFO fno;
    bool    aborted = false;

    state = FS_RLIST;
    if (!getparam_to(TO_RLIST)) goto abort_list;

    {
        int c = flex_getc_to(TO_RLIST);
        if (c < 0) goto abort_list;
        if (c != 0x20) { flex_putc(ACK); state = FS_IDLE; return; }
    }
    flex_putc(CR); flex_putc(LF);

    if (f_opendir(&dir, ".") != FR_OK) { flex_putc(ACK); state = FS_IDLE; return; }
    while (f_readdir(&dir, &fno) == FR_OK && fno.fname[0]) {
        if (!(fno.fattrib & AM_DIR)) continue;
        if (strcmp(fno.fname, ".") == 0 || strcmp(fno.fname, "..") == 0) continue;
        int c = flex_getc_to(TO_RLIST);
        if (c < 0) { aborted = true; break; }
        if (c != 0x20) { aborted = true; break; }
        flex_puts(fno.fname); flex_putc(CR); flex_putc(LF);
    }
    if (!aborted) flex_getc_to(TO_RLIST);
    f_closedir(&dir);
    flex_putc(ACK);
    state = FS_IDLE;
    return;

abort_list:
    flex_putc(ACK);
    state = FS_IDLE;
}

// ── P command : change directory ─────────────────────────────────────────────

static void chngd(void) {
    state = FS_CD;
    bool ok = f_chdir(param) == FR_OK;
    if (ok) f_getcwd(curdir, sizeof(curdir));
    flex_putc(ok ? ACK : NAK);
    state = FS_IDLE;
}

// ── M command : mount new disk image ─────────────────────────────────────────

static void remount(void) {
    char path[132];
    bool ok;

    state = FS_MOUNT;
    snprintf(path, sizeof(path), "%s.DSK", param);
    ok = load_dsk(path);
    if (!ok) {
        snprintf(path, sizeof(path), "%s.dsk", param);
        ok = load_dsk(path);
    }
    if (ok) { flex_putc(ACK); flex_putc(rdonly ? 'R' : 'W'); }
    else      flex_putc(NAK);
    state = FS_IDLE;
}

// ── public API ────────────────────────────────────────────────────────────────

void flexnet_init(void) {
    MC6850_CTRL = MC6850_RESET;
    MC6850_CTRL = MC6850_CTRL_INIT;
    strncpy(curdir, "/", sizeof(curdir));
    state = FS_IDLE;
    printf("flexnet: mc6850 @ 0x%04X\n", MC6850_BASE);
}

bool flexnet_set_disk(const char *name) {
    bool ok = load_dsk(name);
    if (ok) f_getcwd(curdir, sizeof(curdir));
    return ok;
}

void flexnet_unmount(void) {
    if (dsk_open) { f_close(&dsk_fil); dsk_open = false; }
    ready = false;
    state = FS_IDLE;
}

void flexnet_task(void) {
    if (!(MC6850_STAT & MC6850_RDRF)) return;

    uint8_t cmd = flex_getc();
    *param = 0;
    timed_out = false;

    switch (cmd) {
    case 0x55:
    case 0xAA:
        state = FS_SYNC;
        flex_putc(cmd);
        printf("flexnet: sync %02X\n", cmd);
        state = FS_IDLE;
        break;
    case 's':
        flex_getc_to(TO_RREAD);  // skip drive byte
        if (timed_out) break;
        // fall through
    case 'S': sndblk(); break;
    case 'r':
        flex_getc_to(TO_RWRITE); // skip drive byte
        if (timed_out) break;
        // fall through
    case 'R': rcvblk(); break;
    case 'V':
        if (getparam_to(TO_PARAM)) flex_putc(ACK);
        break;
    case '?':
        flex_puts(curdir); flex_putc(CR); flex_putc(ACK);
        break;
    case 'Q':
        flex_putc(ACK);
        break;
    case 'A': lstdsk(); break;
    case 'I': lstdir(); break;
    case 'C':
        getparam_to(TO_PARAM); getparam_to(TO_PARAM);
        getparam_to(TO_PARAM); getparam_to(TO_PARAM);
        // fall through
    case 'D':
        if (getparam_to(TO_PARAM)) flex_putc(NAK);
        break;
    case 'E':
        flex_putc(ACK);
        printf("flexnet: exit\n");
        break;
    case 'P':
        if (getparam_to(TO_CD)) chngd();
        break;
    case 'M':
        if (getparam_to(TO_MOUNT)) remount();
        break;
    default:
        printf("flexnet: unknown cmd 0x%02X\n", cmd);
        break;
    }
}
