# Top-level Makefile for AVR projects
LIBS = libraries
TESTS = monitor hello dskbrowser medieval test-timer spi-speed test-dir test-multi test-seek test-fatfs

.PHONY: all install install-all clean all-mcus $(LIBS) $(TESTS)

all: $(LIBS) $(TESTS)

# Build and install libraries first
$(LIBS):
	$(MAKE) -C $@ all
	$(MAKE) -C $@ install

# Tests depend on libraries being installed
$(TESTS): $(LIBS)
	$(MAKE) -C $@ all

install: all
	@for dir in $(TESTS); do \
		$(MAKE) -C $$dir install || exit 1; \
	done

install-all: all
	@for dir in $(TESTS); do \
		$(MAKE) -C $$dir install-all || exit 1; \
	done

all-mcus:
	$(MAKE) -C $(LIBS) all-mcus
	@for dir in $(TESTS); do \
		$(MAKE) -C $$dir all-mcus || exit 1; \
	done

clean:
	$(MAKE) -C $(LIBS) clean
	@for dir in $(TESTS); do \
		$(MAKE) -C $$dir clean || exit 1; \
	done
