HEADERS += interfaces/SolARModuleHoloLensAPI.h \
    interfaces/SolARModuleHoloLens_traits.h \
    interfaces/SolARBuiltInSLAM.h \
    interfaces/sensorStreaming/sensorStreaming.pb.h \
    interfaces/sensorStreaming/sensorStreaming.grpc.pb.h

SOURCES += src/SolARModuleHoloLens.cpp \
    src/SolARBuiltInSLAM.cpp \
    src/sensorStreaming/sensorStreaming.pb.cc \
    src/sensorStreaming/sensorStreaming.grpc.pb.cc
