LOCAL_DIR := $(GET_LOCAL_DIR)

INCLUDES += -I$(LK_TOP_DIR)/platform/msm_shared/include \
						-I$(LK_TOP_DIR)/app/aboot/displayer

DEFINES += ASSERT_ON_TAMPER=1

OBJS += \
	$(LOCAL_DIR)/aboot.o \
	$(LOCAL_DIR)/fastboot.o \
	$(LOCAL_DIR)/recovery.o\
	$(LOCAL_DIR)/ven_crc.o \
	$(LOCAL_DIR)/displayer/palette.o \
	$(LOCAL_DIR)/displayer/displayer.o \
	$(LOCAL_DIR)/displayer/font.o \
	$(LOCAL_DIR)/displayer/sh1106.o  
