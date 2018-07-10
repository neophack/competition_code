    #-------------------------------------------------
    #
    # Project created by QtCreator 2014-11-03T21:27:45
    #
    #-------------------------------------------------

    QT       += core

    QT       -= gui

    TARGET = learn_c
    CONFIG   += console
    CONFIG   -= app_bundle

    TEMPLATE = app


    SOURCES += \
    main.cpp \
    src/TagFamilies.cpp \
    src/TagFamily.cpp



INCLUDEPATH += /usr/local/include \
              /usr/include/cairo

LIBS += /usr/local/lib/libopencv_*.so \
        /usr/lib/x86_64-linux-gnu/libcairo.so*

HEADERS += \
    src/TagFamily.h







