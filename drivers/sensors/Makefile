# sololz
# super mountain callage

obj-$(CONFIG_IMAP_PRODUCTION_0702)  += sensor_4b.o
obj-$(CONFIG_IMAP_PRODUCTION_7901)  += sensor_4b.o
obj-$(CONFIG_IMAP_PRODUCTION_S01)  += sensor_7600.o

EXTRA_CFLAGS += -DLINUX
EXTRA_CFLAGS += -DDIVX_ENABLE

ifeq ($(CONFIG_IMAP_DECODE_DEBUG),y)
EXTRA_CFLAGS += -DDEBUG
endif

