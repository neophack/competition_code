    #-------------------------------------------------
    #
    # Project created by QtCreator 2014-11-03T21:27:45
    #
    #-------------------------------------------------

    QT       += core

    QT       -= gui

    TARGET = untitled1
    CONFIG   += console
    CONFIG   -= app_bundle

    TEMPLATE = app


    SOURCES += main.cpp \
    EDcircle_function.cpp


INCLUDEPATH += /usr/local/include \
              /usr/local/zed/include \
              /usr/local/cuda/include \
              /usr/local/include/libvibe++
LIBS += /usr/local/lib/libopencv_*.so \
        /usr/local/lib/libfreenect2.so \
        /usr/local/lib/libfreenect2.so.0.2 \
        /usr/local/lib/libfreenect2.so.0.2.0 \
        /usr/local/cuda/lib64/lib*.so.* \
        /usr/local/cuda/lib64/lib*.so \
        /usr/local/zed/lib/libsl*.so

HEADERS += \
    function.h



