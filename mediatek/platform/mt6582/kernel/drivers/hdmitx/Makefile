# drivers/video/mtk/Makefile
#
# Makefile for MT6516 video driver.
#

ccflags-y += -I$(MTK_PATH_PLATFORM)/drivers/hdmitx/  \
             -I$(MTK_PATH_SOURCE)/drivers/hdmitx/    \
             -I$(MTK_PATH_PLATFORM)/drivers/video/   \
             -I$(MTK_PATH_SOURCE)/drivers/video/     \
             -I$(MTK_PATH_PLATFORM)/drivers/hdmitx/  \
             -I$(MTK_PATH_SOURCE)/drivers/hdmitx/    \
             -I$(MTK_PATH_PLATFORM)/drivers/dispsys/ \
             -I$(MTK_PATH_SOURCE)/drivers/sync/ \
             -I$(MTK_PATH_SOURCE)/../hardware/dpframework/inc \
             -I$(MTK_PATH_PLATFORM)/../../common/kernel/drivers/video
            
ifeq ($(MTK_INTERNAL_HDMI_SUPPORT), yes)
ccflags-y += -I$(MTK_PATH_PLATFORM)/drivers/hdmi/
endif  
ifeq ($(MTK_INTERNAL_MHL_SUPPORT), yes)
ccflags-y += -I$(MTK_PATH_PLATFORM)/drivers/mhltx/
endif  
ifeq ($(CUSTOM_KERNEL_HDMI),nxp_tda19989)
	ccflags-y += 	-DHDMI_TDA19989
endif
obj-y	:= hdmitx.o debug.o  hdmitx_test.o




