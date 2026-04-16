TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt
DEFINES += NON_MATLAB_PARSING
DEFINES += MAX_EXT_API_CONNECTIONS=255
DEFINES += DO_NOT_USE_SHARED_MEMORY
DEFINES += _LARGEFILE64_SOURCE
XENO_DIR = /usr/xenomai

XENO_CONFIG = $$XENO_DIR/bin/xeno-config
XCFLAGS = $(shell $$XENO_CONFIG --skin=native  --cflags)
XLDFLAGS = $(shell $$XENO_CONFIG --skin=native --ldflags)
QMAKE_CXXFLAGS += $$XCFLAGS

#*-g++* { #includes MinGW
#    CONFIG(debug,debug|release) {
#        QMAKE_CXXFLAGS += -g -ggdb
#    } else {
#        QMAKE_CFLAGS += -O3
#        QMAKE_CXXFLAGS += -O3
#    }

#    QMAKE_CFLAGS_WARN_ON = -Wall
#    QMAKE_CFLAGS_WARN_ON += -Wno-strict-aliasing
#    QMAKE_CFLAGS_WARN_ON += -Wno-unused-parameter
#    QMAKE_CFLAGS_WARN_ON += -Wno-unused-but-set-variable
#    QMAKE_CFLAGS_WARN_ON += -Wno-unused-local-typedefs

#    QMAKE_CXXFLAGS_WARN_ON = -Wall
#    QMAKE_CXXFLAGS_WARN_ON += -Wno-unused-parameter
#    QMAKE_CXXFLAGS_WARN_ON += -Wno-strict-aliasing
#    QMAKE_CXXFLAGS_WARN_ON += -Wno-empty-body
#    QMAKE_CXXFLAGS_WARN_ON += -Wno-write-strings
#    QMAKE_CXXFLAGS_WARN_ON += -Wno-unused-but-set-variable
#    QMAKE_CXXFLAGS_WARN_ON += -Wno-unused-local-typedefs
#    QMAKE_CXXFLAGS_WARN_ON += -Wno-narrowing

#    DEFINES += SIM_COMPILER_STR=\\\"GCC\\\"
#}

SOURCES += \
#    ../../Github/rbdl-orb-3.0.0/src/Constraint_Contact.cc \
#    ../../Github/rbdl-orb-3.0.0/src/Constraint_Loop.cc \
#    ../../Github/rbdl-orb-3.0.0/src/Constraints.cc \
#    ../../Github/rbdl-orb-3.0.0/src/Dynamics.cc \
#    ../../Github/rbdl-orb-3.0.0/src/Joint.cc \
#    ../../Github/rbdl-orb-3.0.0/src/Kinematics.cc \
#    ../../Github/rbdl-orb-3.0.0/src/Logging.cc \
#    ../../Github/rbdl-orb-3.0.0/src/Model.cc \
#    ../../Github/rbdl-orb-3.0.0/src/rbdl_errors.cc \
#    ../../Github/rbdl-orb-3.0.0/src/rbdl_mathutils.cc \
#    ../../Github/rbdl-orb-3.0.0/src/rbdl_utils.cc \
#    ../../Github/rbdl-orb-3.0.0/src/rbdl_version.cc \
#    coppeliaKinematicsRoutines/dummy.cpp \
#    coppeliaKinematicsRoutines/environment.cpp \
#    coppeliaKinematicsRoutines/ik.cpp \
#    coppeliaKinematicsRoutines/ikElement.cpp \
#    coppeliaKinematicsRoutines/ikGroup.cpp \
#    coppeliaKinematicsRoutines/ikGroupContainer.cpp \
#    coppeliaKinematicsRoutines/ikRoutines.cpp \
#    coppeliaKinematicsRoutines/joint.cpp \
#    coppeliaKinematicsRoutines/objectContainer.cpp \
#    coppeliaKinematicsRoutines/sceneObject.cpp \
#    coppeliaKinematicsRoutines/serialization.cpp \
    dynamics.cpp \
#    kinematics.cpp \
    main.cpp \
    mitmotor.cpp \
    motionplanning.cpp \
    remoteApi/extApi.c \
    remoteApi/extApiPlatform.c \
#    robot.cpp \
#    simMath/3Vector.cpp \
#    simMath/3X3Matrix.cpp \
#    simMath/4Vector.cpp \
#    simMath/4X4FullMatrix.cpp \
#    simMath/4X4Matrix.cpp \
#    simMath/6Vector.cpp \
#    simMath/6X6Matrix.cpp \
#    simMath/7Vector.cpp \
#    simMath/MMatrix.cpp \
#    simMath/MyMath.cpp \
#    simMath/Vector.cpp \
    robot.cpp \
    rtcan.cpp \
    t265camera.cpp \
    vrepsim.cpp \
    magneticencoder.cpp \
    src/protocol2_packet_handler.cpp \
    src/protocol1_packet_handler.cpp \
    src/port_handler_windows.cpp \
    src/port_handler_mac.cpp \
    src/port_handler_linux.cpp \
    src/port_handler_arduino.cpp \
    src/port_handler.cpp \
    src/packet_handler.cpp \
    src/group_sync_write.cpp \
    src/group_sync_read.cpp \
    src/group_bulk_write.cpp \
    src/group_bulk_read.cpp \
    Dynamixel.cpp \
    rt485.cpp \
    Hololens.cpp \
    ftsensor.cpp \
    CPython.cpp \
    include/uitools.c \
    mujocosim.cpp
    #../B0Api/cpp/b0RemoteApi.cpp \
    #vrepsimb0.cpp

HEADERS += \
#    coppeliaKinematicsRoutines/dummy.h \
#    coppeliaKinematicsRoutines/environment.h \
#    coppeliaKinematicsRoutines/ik.h \
#    coppeliaKinematicsRoutines/ikElement.h \
#    coppeliaKinematicsRoutines/ikGroup.h \
#    coppeliaKinematicsRoutines/ikGroupContainer.h \
#    coppeliaKinematicsRoutines/ikRoutines.h \
#    coppeliaKinematicsRoutines/joint.h \
#    coppeliaKinematicsRoutines/objectContainer.h \
#    coppeliaKinematicsRoutines/sceneObject.h \
#    coppeliaKinematicsRoutines/serialization.h \
    dynamics.h \
    globalDef.h \
#    interp.h \
#    kinematics.h \
    mitmotor.h \
    motionplanning.h \
    remoteApi/extApi.h \
    remoteApi/extApiPlatform.h \
#    robot.h \
#    simMath/3Vector.h \
#    simMath/3X3Matrix.h \
#    simMath/4Vector.h \
#    simMath/4X4FullMatrix.h \
#    simMath/4X4Matrix.h \
#    simMath/6Vector.h \
#    simMath/6X6Matrix.h \
#    simMath/7Vector.h \
#    simMath/MMatrix.h \
#    simMath/MyMath.h \
#    simMath/VPoint.h \
#    simMath/Vector.h \
#    simMath/mathDefines.h \
    robot.h \
    rtcan.h \
    t265camera.h \
    vrepsim.h \
    magneticencoder.h \
    dynamixel_sdk/protocol2_packet_handler.h \
    dynamixel_sdk/protocol1_packet_handler.h \
    dynamixel_sdk/port_handler_windows.h \
    dynamixel_sdk/port_handler_mac.h \
    dynamixel_sdk/port_handler_linux.h \
    dynamixel_sdk/port_handler_arduino.h \
    dynamixel_sdk/port_handler.h \
    dynamixel_sdk/packet_handler.h \
    dynamixel_sdk/group_sync_write.h \
    dynamixel_sdk/group_sync_read.h \
    dynamixel_sdk/group_bulk_write.h \
    dynamixel_sdk/group_bulk_read.h \
    dynamixel_sdk/dynamixel_sdk.h \
    Dynamixel.h \
    rt485.h \
    Hololens.h \
    ftsensor.h \
    CPython.h \
#    include/glfw3.h \
#    include/mjdata.h \
#    include/mjmodel.h \
#    include/mjrender.h \
#    include/mjui.h \
#    include/mjvisualize.h \
#    include/mjxmacro.h \
#    include/mujoco.h \
#    include/uitools.h \
#    mujocosim.h
    #vrepsimb0.h


#INCLUDEPATH += \
#			/usr/local/qwt-6.1.3/include \

#INCLUDEPATH += \
#			./include \

#LIBS += \
#		-L/usr/local/qwt-6.1.3/lib -lqwt \

#LIBS += \
#		-L./lib -lqwt \

#mujuco 依赖
LIBS += -L$$PWD/lib -lglewegl
LIBS += -L$$PWD/lib/ -lglewosmesa
LIBS += -L$$PWD/lib/ -lmujoco210nogl
LIBS += -L$$PWD/lib/ -lmujoco210
LIBS += -L$$PWD/lib/ -lglfw3
PRE_TARGETDEPS += $$PWD/lib/libglfw3.a
LIBS += -L$$PWD/lib/ -lglfw
LIBS += -L$$PWD/lib/ -lglew
LIBS += -ldl
unix:!macx: LIBS += -lGL
unix|win32: LIBS += -lpthread
unix:!macx: LIBS += -lX11
unix:!macx: LIBS += -lrt
unix:!macx: LIBS += -lXrandr
unix:!macx: LIBS += -lXinerama
unix:!macx: LIBS += -lXxf86vm
unix:!macx: LIBS += -lXcursor
unix:!macx: LIBS += -lc
unix:!macx: LIBS += -lxcb
unix:!macx: LIBS += -lXext
unix:!macx: LIBS += -lXrender
unix:!macx: LIBS += -lXfixes
unix:!macx: LIBS += -lXau
unix:!macx: LIBS += -lXdmcp
#unix:!macx: LIBS += -lbsd
unix:!macx: LIBS += -lm





INCLUDEPATH += \
INCLUDEPATH += \
    ./  \
    #./simMath \
    #./coppeliaKinematicsRoutines \
    #./ReflexxesTypeII/include \
    ./remoteApi \
    ./remoteApi/include \
    #../B0Api/cpp/   \
    #../B0Api/cpp/msgpack-c/include/ \
    #../B0Api/cpp/bluezero/include/b0/bindings/  \
    ./rbdl-orb-3.0.0/    \
    ./rbdl-orb-3.0.0/include \
    ./rbdl-orb-3.0.0/build/include/  \
    ./eigen-3.3.9    \
    /usr/xenomai/include/cobalt \
    /usr/xenomai/include \
    /usr/xenomai/include/alchemy \

INCLUDEPATH += $$PWD/include
DEPENDPATH += $$PWD/include

LIBS +=\
    -lpthread \
    -L../ReflexxesTypeII/lib/ -lReflexxesTypeII  \
    #-L/home/yhzhu/Documents/myVrepWorks/B0Api/cpp/lib/ -lb0 \
    -L/usr/xenomai/lib/ -lalchemy \
    -L/usr/xenomai/lib/ -lcopperplate \
    -L/usr/xenomai/lib/ -lcobalt \
    -L/usr/xenomai/lib/ -lmodechk \
    -L/usr/xenomai/lib/ -lsmokey \
    -L/usr/xenomai/lib/ -ltrank \
    -L/usr/xenomai/lib/ -lanalogy \
    -lcopperplate \
    -lrealsense2 \
    -lrt \
    -lpcan  \
    $$XLDFLAGS
#LIBS += \
#		-L./lib -lqwt \

DISTFILES += \
    main_back1.txt

QT += core

#INCLUDEPATH += /home/mjq/01_software/03_SL_related_libs/rbdl-orb/
#INCLUDEPATH += /home/mjq/01_software/03_SL_related_libs/rbdl-orb/include/
#INCLUDEPATH += /home/mjq/01_software/03_SL_related_libs/rbdl-orb/build/include/
INCLUDEPATH += /home/mjq/01_software/03_SL_related_libs/06_peak-linux-driver-8.11.0/lib
INCLUDEPATH += /home/mjq/01_software/03_SL_related_libs/07_librealsense/include
INCLUDEPATH += /home/mjq/02_project/03_hit_superlimbs/SRL2/ReflexxesTypeII/include

LIBS += -L/home/mjq/01_software/03_SL_related_libs/rbdl-orb/build -lrbdl
#LIBS += -L/home/mjq/02_project/03_hit_superlimbs/SRL2/rbdl-orb-3.0.0/build -lrbdl
LIBS += -L/home/mjq/01_software/03_SL_related_libs/rbdl-orb/build_urdf/addons/urdfreader -lrbdl_urdfreader
LIBS += -L/home/mjq/01_software/03_SL_related_libs/06_peak-linux-driver-8.11.0/lib/lib -lpcan
LIBS += -L/home/mjq/01_software/03_SL_related_libs/07_librealsense/build/Release -lrealsense2
#LIBS += -L/home/mjq/01_software/03_SL_related_libs/ReflexxesTypeII/Linux/x64/release/lib/shared -lReflexxesTypeII
LIBS += -L/home/mjq/02_project/03_hit_superlimbs/SRL2/ReflexxesTypeII/lib -lReflexxesTypeII

