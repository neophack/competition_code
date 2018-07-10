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
    CONFIG += c++11

    TEMPLATE = app


    SOURCES += \
    main.cpp


INCLUDEPATH += /usr/local/include \
              /usr/local/include/libvibe++
LIBS += /usr/local/lib/libopencv_*.so \
        /usr/local/lib/libvibe++.so






