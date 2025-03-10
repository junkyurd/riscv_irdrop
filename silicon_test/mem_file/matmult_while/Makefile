TOP=../..
include $(TOP)/Make.rules

LDLIBS=-lc -lgcc
CFLAGS+=-O0

LDFLAGS+=-Wl,--no-relax
LDFLAGS+=-Wl,-Tdata=0x1000

PROGS=prog prog.bin prog.lst 

all:: $(PROGS)

prog: crt0.o main.o mult_mat.o
	$(LINK.c) -o $@ $^ $(LDLIBS)
	$(SIZE) -A -x $@
	rm -f riscv.inst
	./elf2hex/elf2hex --bit-width 32 --input prog --output riscv.inst
	
clean::
	rm -f $(PROGS) *.o *.s *.lst *.bin *.srec

.PHONY: run
run: prog.bin
	../../../src/rvddt -l0x2000 -f prog.bin
