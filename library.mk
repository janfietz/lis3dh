
LIS3DH_DIR := $(dir $(lastword $(MAKEFILE_LIST)))
CSRC += $(wildcard $(LIS3DH_DIR)/lis3dh.c)
EXTRAINCDIRS += $(LIS3DH_DIR)

CFLAGS += -DHAS_LIS3DH
