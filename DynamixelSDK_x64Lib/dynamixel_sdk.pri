
SOURCES += \
    $$PWD/src/dynamixel_sdk/port_handler.cpp \
    $$PWD/src/dynamixel_sdk/port_handler_arduino.cpp \
    $$PWD/src/dynamixel_sdk/port_handler_linux.cpp \
    $$PWD/src/dynamixel_sdk/port_handler_mac.cpp \
    $$PWD/src/dynamixel_sdk/port_handler_windows.cpp 

INCLUDEPATH += $$PWD/include/dynamixel_sdk
INCLUDEPATH += $$PWD/include/

HEADERS += \
    $$PWD/include/dynamixel_sdk/port_handler.h \
    $$PWD/include/dynamixel_sdk/port_handler_arduino.h \
    $$PWD/include/dynamixel_sdk/port_handler_linux.h \
    $$PWD/include/dynamixel_sdk/port_handler_mac.h \
    $$PWD/include/dynamixel_sdk/port_handler_windows.h \
    $$PWD/src/DynamixelSDK.h
