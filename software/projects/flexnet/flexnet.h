#pragma once
#include <stdbool.h>

#define FLEX_DSK_DEFAULT  "FLEX.DSK"   // change at compile time if needed

void flexnet_init(void);               // call once after tusb_init()
bool flexnet_set_disk(const char *name);
void flexnet_unmount(void);            // call from tuh_msc_umount_cb
void flexnet_task(void);               // call every main loop iteration
