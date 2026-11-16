# Firmware build: boot ROM, monitor, applications, CoreMark and the ISA tests.
#
#   make            everything except CoreMark and the ISA tests
#   make coremark   fetches the EEMBC sources on first use
#   make isa        fetches riscv-tests on first use
#
# Needs a bare-metal RISC-V GCC with an rv32im multilib, e.g. the xPack
# riscv-none-elf-gcc. Override RISCV= for a different prefix.

RISCV   ?= riscv-none-elf-
CC       = $(RISCV)gcc
OBJCOPY  = $(RISCV)objcopy
OBJDUMP  = $(RISCV)objdump
PYTHON  ?= python3

ARCH     = -march=rv32im_zicsr_zifencei -mabi=ilp32
CFLAGS   = $(ARCH) -O2 -ffreestanding -fno-builtin -ffunction-sections -fdata-sections \
           -Wall -Wextra -Wno-format -Wno-array-bounds -Icommon
LDFLAGS  = $(ARCH) -nostdlib -nostartfiles -Wl,--gc-sections -Wl,--no-warn-rwx-segments

B        = build
COMMON   = common/crt0.S common/trap.S common/lib.c
APPS     = hello irqtest memtest tasks

.PHONY: all clean coremark isa
.SECONDARY:
all: bootrom/bootrom.hex $(B)/monitor.img $(B)/monitor_ram.bin $(APPS:%=$(B)/%.bin)

$(B):
	@mkdir -p $(B)

# ---- Boot ROM (committed as bootrom.hex so the FPGA build needs no compiler) --
$(B)/bootrom.elf: bootrom/start.S bootrom/boot.c bootrom/bootrom.ld common/soc.h | $(B)
	$(CC) $(ARCH) -Os -ffreestanding -fno-builtin -Wall -Wextra $(LDFLAGS) \
	    -T bootrom/bootrom.ld -o $@ bootrom/start.S bootrom/boot.c

bootrom/bootrom.hex: $(B)/bootrom.elf
	$(OBJCOPY) -O binary $< $(B)/bootrom.bin
	$(PYTHON) ../scripts/mkimage.py hex $(B)/bootrom.bin $@ --words 512

# ---- Programs that run from RAM ------------------------------------------------
$(B)/%.elf: apps/%.c $(COMMON) common/lib.h common/soc.h common/ram.ld | $(B)
	$(CC) $(CFLAGS) $(LDFLAGS) -T common/ram.ld -o $@ $(COMMON) $< -lgcc

$(B)/%.bin: $(B)/%.elf
	$(OBJCOPY) -O binary $< $@

# ---- Monitor: executes in place from flash, or from RAM -------------------------
$(B)/monitor.elf: monitor/monitor.c $(COMMON) common/lib.h common/soc.h common/xip.ld | $(B)
	$(CC) $(CFLAGS) $(LDFLAGS) -T common/xip.ld -o $@ $(COMMON) $< -lgcc

$(B)/monitor.img: $(B)/monitor.elf
	$(OBJCOPY) -O binary $< $(B)/monitor.bin
	$(PYTHON) ../scripts/mkimage.py image $(B)/monitor.bin $@ --xip --entry 0x01100014

$(B)/monitor_ram.elf: monitor/monitor.c $(COMMON) common/lib.h common/soc.h common/ram.ld | $(B)
	$(CC) $(CFLAGS) $(LDFLAGS) -T common/ram.ld -o $@ $(COMMON) $< -lgcc

# ---- CoreMark --------------------------------------------------------------------
COREMARK_DIR ?= $(B)/coremark-src
COREMARK_SRC  = core_list_join.c core_main.c core_matrix.c core_state.c core_util.c
ITERATIONS   ?= 600           # at least 10 s of run time at 18 MHz

$(COREMARK_DIR)/core_main.c: | $(B)
	git clone --depth 1 https://github.com/eembc/coremark.git $(COREMARK_DIR)

coremark: $(B)/coremark.bin
$(B)/coremark.elf: $(COREMARK_DIR)/core_main.c coremark/core_portme.c coremark/core_portme.h $(COMMON)
	$(CC) $(CFLAGS) -O2 -funroll-loops -finline-functions -Wno-unused-parameter \
	    -Icoremark -I$(COREMARK_DIR) -DITERATIONS=$(ITERATIONS) \
	    -DFLAGS_STR="\"-O2 -funroll-loops -finline-functions\"" \
	    $(LDFLAGS) -T common/ram.ld -o $@ $(COMMON) coremark/core_portme.c \
	    $(COREMARK_SRC:%=$(COREMARK_DIR)/%) -lgcc

# ---- riscv-tests -------------------------------------------------------------------
ISA_DIR ?= $(B)/riscv-tests
isa: | $(B)
	@test -d $(ISA_DIR) || git clone --depth 1 https://github.com/riscv-software-src/riscv-tests.git $(ISA_DIR)
	$(PYTHON) ../scripts/build_isa.py --src $(ISA_DIR) --out $(B)/isa --cc $(CC) --objcopy $(OBJCOPY)

clean:
	rm -rf $(B)
