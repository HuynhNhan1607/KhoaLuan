# Set dia chi VL53L0X - chay 1 lan khi boot
#   cd vl53l0x && make
#   sudo ./vl53l0x_addr_init

CC      = gcc
CFLAGS  = -Wall -std=c11 -D_GNU_SOURCE
LDFLAGS = -lgpiod -li2c

SRCS = vl53l0x_addr_init.c vl53l0x_c.c gpio_helper.c
OBJS = $(SRCS:.c=.o)
TARGET = vl53l0x_addr_init

.PHONY: all clean

all: $(TARGET)
	@echo "Done. Chay: sudo ./$(TARGET)"

$(TARGET): $(OBJS)
	$(CC) $^ -o $@ $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -I. -c $< -o $@

clean:
	rm -f $(OBJS) $(TARGET)
