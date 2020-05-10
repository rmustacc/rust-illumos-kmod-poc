#
# Do something
#

CC = toolchain/bin/rustc
CFLAGS = -C no-redzone -O -C relocation-model=static --emit=obj -C soft-float -C panic=abort
CFLAGS += -C default-linker-libraries=no -C code-model=kernel -g
OBJS = rmod.rs
BIN = $(OBJS:%.rs=%)

$(BIN): $(OBJS)
	$(CC) $(CFLAGS) -o $(BIN) $(OBJS)

example: example.rs
	$(CC) $(CFLAGS) -o $(BIN) example.rs

clean:
	rm -f $(BIN)
