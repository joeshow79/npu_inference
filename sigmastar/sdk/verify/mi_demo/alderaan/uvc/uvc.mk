INC  += $(DB_BUILD_TOP)/../common/live555/UsageEnvironment/include
INC  += $(DB_BUILD_TOP)/../common/live555/groupsock/include
INC  += $(DB_BUILD_TOP)/../common/live555/liveMedia/include
INC  += $(DB_BUILD_TOP)/../common/live555/BasicUsageEnvironment/include
INC  += $(DB_BUILD_TOP)/../common/live555/mediaServer/include

ST_DEP := common vpe venc vif uvc
MODE := $(findstring fastboot, $(BOARD))

MODE:=fastboot

LIBS += -lmi_sensor -lmi_vif -lmi_vpe -lmi_venc -lmi_iqserver
LIBS += -lmi_isp -lcus3a -lispalgo -lmi_ipu -lcam_fs_wrapper -lmi_rgn -lmi_divp
ifeq ($(CHIP), i6e)
LIBS +=-lfbc_decode
endif
ifeq ($(MODE), fastboot)
LIBS += -lmi_vif -lmi_vpe -lmi_venc -lmi_iqserver -lfbc_decode
CODEDEFINE += -DFASTBOOT_MODE=1
else
ST_DEP += common vpe venc vif uvc rgn onvif live555
LIBS += -lmi_vif -lmi_vpe -lmi_venc -lmi_rgn -lmi_divp -lmi_iqserver -lmi_vdf \
		-lmi_shadow -lOD_LINUX -lMD_LINUX -lVG_LINUX -lmi_ive -lmi_ao
CODEDEFINE += -DFASTBOOT_MODE=0
endif
