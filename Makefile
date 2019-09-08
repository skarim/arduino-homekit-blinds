PROGRAM = blinds

EXTRA_COMPONENTS = \
	extras/http-parser \
	$(abspath $(LIB_PATH)/wolfssl) \
	$(abspath $(LIB_PATH)/cJSON) \
	$(abspath $(LIB_PATH)/homekit)

FLASH_SIZE ?= 32

EXTRA_CFLAGS += -I../.. -DHOMEKIT_SHORT_APPLE_UUIDS

include $(SDK_PATH)/common.mk

monitor:
	$(FILTEROUTPUT) --port $(ESPPORT) --baud 115200 --elf $(PROGRAM_OUT)
