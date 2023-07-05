#******************************************************************************
#**  CarMaker - Version 11.0
#**  Vehicle Dynamics Simulation Toolkit
#**
#**  Copyright (C)   IPG Automotive GmbH
#**                  Bannwaldallee 60             Phone  +49.721.98520.0
#**                  76185 Karlsruhe              Fax    +49.721.98520.99
#**                  Germany                      WWW    www.ipg-automotive.com
#******************************************************************************
#**
#**  Application Makefile
#**  Run with 'make V=1 ...' for a verbose build.
#**
#******************************************************************************

include /opt/ipg/carmaker/linux64-11.0/include/MakeDefs.linux64

APP_VER =		"Car_Generic <insert.your.version.no>"
APP_NAME =		CarMaker.$(ARCH)$(EXE_EXT)

#OPT_CFLAGS =		-g -O1

LD_LIBS_USR = bridge_utils.lib protobuf.lib
LD_LIBS =		$(CAR_LIB) \
			$(CARMAKER_LIB) $(DRIVER_LIB) $(ROAD_LIB) $(TIRE_LIB) -lprotobuf 
			
OBJ_PROTOBUF :=  ../proto/generated/modules/drivers/gnss/proto/gnss_best_pose.pb.o  \
            ../proto/generated/modules/common/proto/header.pb.o \
            ../proto/generated/modules/common/proto/error_code.pb.o \
            ../proto/generated/modules/common/proto/geometry.pb.o \
            ../proto/generated/modules/localization/proto/pose.pb.o \
			../proto/generated/modules/drivers/proto/sensor_image.pb.o \
            ../proto/generated/modules/common/proto/pnc_point.pb.o \
            ../proto/generated/modules/control/proto/pad_msg.pb.o \
            ../proto/generated/modules/common/proto/vehicle_signal.pb.o \
            ../proto/generated/modules/common/configs/proto/vehicle_config.pb.o \
            ../proto/generated/modules/canbus/proto/chassis.pb.o \
            ../proto/generated/modules/common/proto/drive_state.pb.o \
            ../proto/generated/modules/control/proto/control_cmd.pb.o \
            ../proto/generated/modules/drivers/proto/pointcloud.pb.o \
            ../proto/generated/modules/localization/proto/gps.pb.o \
            ../proto/generated/modules/localization/proto/imu.pb.o \
            ../proto/generated/modules/drivers/gnss/proto/imu.pb.o \
            ../proto/generated/modules/drivers/velodyne/proto/velodyne.pb.o \
            ../proto/generated/modules/drivers/gnss/proto/ins.pb.o \
			../proto/generated/modules/drivers/canbus/proto/can_card_parameter.pb.o \
			../proto/generated/modules/drivers/radar/conti_radar/proto/conti_radar_conf.pb.o \
			../proto/generated/modules/drivers/proto/conti_radar.pb.o 


OBJ_BRIDGE = bridge_tcp_client.o cyberbridge.o base64.o cyberbridge_C_interface.o\
             conversion.o cloud_doublebuffer.o
             
OBJS =			CM_Main.o CM_Vehicle.o User.o

OBJS += UTM_PositionData.o
OBJS += Apollo_CarMaker_IF.o
OBJS += rsds-client-camera.o

# Prepend local include/library directory to include path:
# PREINC_CFLAGS +=		-I../include -I../lib/$(ARCH) -I../lib
LDFLAGS += -L../lib/linux64
LD_LIBS_OS += -lGeographic 
# Append local include/library directory to include path:
# INC_CFLAGS +=		-I../include -I../lib/$(ARCH) -I../lib
INC_CFLAGS += -I../proto/generated -I../include

### Linking with RTW-built Simulink models

#MATSUPP_MATVER =	R2019a
#LD_LIBS +=		$(MATSUPP_LIB)

# C Compiler options
# zez: comment out to let the warnings of unused function parameters in User.c and CMMain.c not be shown
#CFLAGS += -Wall -Wextra -O2 -msse3 -mavx
#CFLAGS += -Wall -Wextra -g

#Modification for Localization, e,g. coord. system transformation and UTM conversion
DEF_CFLAGS += -DWITH_CMLOCAL
#Modification for using the SimTime with the start time of sys. WC on HostPC (cm) 
#DEF_CFLAGS += -DWITH_CMSIMTIME

# C++ Compiler Add support for C++17
CXXFLAGS += -std=c++17 -Wall -Wextra -O2 -msse3 -mavx
#CXXFLAGS += -std=c++17 -Wall -Wextra -g 


# @@PLUGIN-BEGIN-LIBS@@ - Automatically generated code - don't edit!
# @@PLUGIN-END@@

### END (Linking with RTW-built Simulink models)


default:	$(APP_NAME)


$(APP_NAME):	$(OBJS_$(ARCH)) $(OBJS) $(LD_LIBS_USR) $(LD_LIBS_MK) app_tmp.o
	$(QECHO) " LD     $@"
	
	$Q $(CXX) $(CXXFLAGS) $(LDFLAGS) -o $@ \
		$(OBJS_$(ARCH)) $(OBJS) $(LD_LIBS_USR) $(LD_LIBS) $(LD_LIBS_OS) \
		app_tmp.o -latomic 
	

	$(SET_EXE_PERMISSIONS) $@


install: $(APP_NAME)
	$(INSTALL_APP) $(APP_NAME) $(ARCH)

clean:
	-rm -f *~ *% *.o  bridge_utils.lib core CarMaker.linux64 

clean_all: clean
	-rm -fv $(OBJ_PROTOBUF) *.lib 

app_tmp.c: Makefile $(OBJS_$(ARCH)) $(OBJS) $(LD_LIBS_MK) $(LD_LIBS_USR)
	$(QECHO) " MK     $@"
	$Q $(CREATE_INFO_CMD)

protobuf.lib: $(OBJ_PROTOBUF)
	@echo Making library $@
	@ar crs $@ $+

bridge_utils.lib: $(OBJ_BRIDGE)
	@echo Making library $@
	@ar crs $@ $+
	
depend .depend: Makefile
	$(QECHO) " MK     $@"
	@echo -n "" >.depend
#ifneq ($(wildcard *.c),)
	$Q-$(CC)  $(CFLAGS)   $(DEPCFLAGS)   *.c   >>.depend 2>/dev/null
#endif
#ifneq ($(wildcard *.cpp),)
	$Q-$(CXX) $(CXXFLAGS) $(DEPCXXFLAGS) *.cpp >>.depend 2>/dev/null
#endif
include .depend
