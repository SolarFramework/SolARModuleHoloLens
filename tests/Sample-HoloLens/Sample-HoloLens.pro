TARGET = Sample-HoloLens
VERSION=0.7.0

CONFIG += c++1z
CONFIG -= qt
CONFIG += console

DEFINES += MYVERSION=$${VERSION}

QT += opengl

CONFIG(debug,debug|release) {
    TARGETDEPLOYDIR = $${PWD}../../bin/Debug
    DEFINES += _DEBUG=1
    DEFINES += DEBUG=1
}

CONFIG(release,debug|release) {
    TARGETDEPLOYDIR = $${PWD}../../bin/Release
    DEFINES += _NDEBUG=1
    DEFINES += NDEBUG=1
}

win32::CONFIG -= static
win32::CONFIG += shared
QMAKE_TARGET.arch = x86_64 #must be defined prior to include

DEPENDENCIESCONFIG = sharedlib recursive install_recurse
PROJECTCONFIG = QTVS

#NOTE : CONFIG as staticlib or sharedlib, DEPENDENCIESCONFIG as staticlib or sharedlib and PROJECTDEPLOYDIR MUST BE DEFINED BEFORE templatelibbundle.pri inclusion
include ($$(REMAKEN_RULES_ROOT)/qmake/templateappconfig.pri)


DEFINES += BOOST_ALL_NO_LIB
DEFINES += BOOST_ALL_DYN_LINK

HEADERS += \

SOURCES += \
    main.cpp

unix {
    LIBS += -ldl
}

macx {
    QMAKE_MAC_SDK= macosx
    QMAKE_CXXFLAGS += -fasm-blocks -x objective-c++
}

win32 {
    QMAKE_LFLAGS += /MACHINE:X64
    DEFINES += WIN64 UNICODE _UNICODE
    QMAKE_COMPILER_DEFINES += _WIN64

    # Windows Kit (msvc2013 64)
    LIBS += -L$$(WINDOWSSDKDIR)lib/winv6.3/um/x64 -lshell32 -lgdi32 -lComdlg32
    INCLUDEPATH += $$(WINDOWSSDKDIR)lib/winv6.3/um/x64
 }

config_files.path = $${TARGETDEPLOYDIR}
config_files.files = $$files($${PWD}/../data/conf_Sample-HoloLens.xml) \
                     $$files($${PWD}/../data/hololens_sensors_calibration.yml)
INSTALLS += config_files

include ($$(REMAKEN_RULES_ROOT)/qmake/remaken_install_target.pri)
