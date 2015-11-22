TEMPLATE = app
TARGET = heartlistener

QT += quick bluetooth


# Input
HEADERS += deviceinfo.h \
           heartrate.h \
    processing.h
SOURCES += deviceinfo.cpp \
           heartrate.cpp \
           main.cpp \
    processing.cpp

OTHER_FILES += assets/*.qml \
               assets/*.js

RESOURCES += \
             resources.qrc

QML2_IMPORT_PATH += assets

target.path = $$[QT_INSTALL_EXAMPLES]/bluetooth/heartlistener
INSTALLS += target

DISTFILES += \
    assets/chart.js
