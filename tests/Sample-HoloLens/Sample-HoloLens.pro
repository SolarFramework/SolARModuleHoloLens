QT     -= core gui
CONFIG -= app_bundle qt

INSTALLSUBDIR = SolARBuild
TARGET = Sample-HoloLens
VERSION=0.7.0
DEFINES +=  $${TARGET}VERSION=\"$${VERSION}\"

CONFIG += c++1z
CONFIG += console

include(findremakenrules.pri)

CONFIG += shared

DEPENDENCIESCONFIG = sharedlib install_recurse

## Configuration for Visual Studio to install binaries and dependencies. Work also for QT Creator by replacing QMAKE_INSTALL
PROJECTCONFIG = QTVS

#NOTE : CONFIG as staticlib or sharedlib, DEPENDENCIESCONFIG as staticlib or sharedlib and PROJECTDEPLOYDIR MUST BE DEFINED BEFORE templatelibbundle.pri inclusion
include ($$shell_quote($$shell_path($$(REMAKEN_RULES_ROOT)/qmake/templateappconfig.pri)))

DEFINES += BOOST_ALL_NO_LIB
DEFINES += BOOST_ALL_DYN_LINK

HEADERS +=
SOURCES +=     main.cpp

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
    QMAKE_CXXFLAGS += -wd4250 -wd4251 -wd4244 -wd4275

    # Windows Kit (msvc2013 64)
    LIBS += -L$$(WINDOWSSDKDIR)lib/winv6.3/um/x64 -lshell32 -lgdi32 -lComdlg32
    INCLUDEPATH += $$(WINDOWSSDKDIR)lib/winv6.3/um/x64
 }

config_files.path = $${TARGETDEPLOYDIR}
config_files.files = $$files($${PWD}/conf_Sample-HoloLens.xml)
config_files.files = $$files($${PWD}/hololens_sensors_calibration.yml)

INSTALLS += config_files

#NOTE : Must be placed at the end of the .pro
include ($$shell_quote($$shell_path($$(REMAKEN_RULES_ROOT)/qmake/remaken_install_target.pri)))) # Shell_quote & shell_path required for visual on windows
