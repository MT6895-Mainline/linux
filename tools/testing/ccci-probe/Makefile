# SPDX-License-Identifier: GPL-2.0
CFLAGS ?= -O1 -g
CFLAGS += -std=gnu11 -Wall -Wextra -Werror -fno-omit-frame-pointer
SANITIZERS ?= address,undefined
ifneq ($(SANITIZERS),)
CFLAGS += -fsanitize=$(SANITIZERS)
LDFLAGS += -fsanitize=$(SANITIZERS)
endif

PARSER := ../../../drivers/misc/mediatek/ccci_probe/ccci_probe_parser.h
LAYOUT := ../../../drivers/misc/mediatek/ccci_smem_dump/ccci_smem_layout.h
RINGBUF := ../../../drivers/misc/mediatek/ccci_ccif/ccci_ccif_ringbuf.h

.PHONY: all test clean
all: test_ccci_probe test_ccci_smem test_ccci_ringbuf

test_ccci_probe: test_ccci_probe.c $(PARSER)
	$(CC) $(CFLAGS) $< $(LDFLAGS) -o $@

test_ccci_smem: test_ccci_smem.c $(LAYOUT)
	$(CC) $(CFLAGS) $< $(LDFLAGS) -o $@

test_ccci_ringbuf: test_ccci_ringbuf.c $(RINGBUF)
	$(CC) $(CFLAGS) $< $(LDFLAGS) -o $@

test: test_ccci_probe test_ccci_smem test_ccci_ringbuf
	./test_ccci_probe
	./test_ccci_smem
	./test_ccci_ringbuf
	sh ./test_kconfig.sh

clean:
	$(RM) test_ccci_probe test_ccci_smem test_ccci_ringbuf
