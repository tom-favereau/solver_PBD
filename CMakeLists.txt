cmake_minimum_required(VERSION 3.16)

project(TD9 VERSION 0.1 LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Qt 5/6 détection + modules nécessaires
find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets Core Concurrent)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets Core Concurrent)

set(PROJECT_SOURCES
        main.cpp
        mainwindow.cpp
        mainwindow.h
        mainwindow.ui
        drawarea.cpp
        drawarea.h
        constraints.cpp
        constraints.h
        physicalbody.h
        multithreading.cpp
        multithreading.h
        grid.h)

if(QT_VERSION_MAJOR GREATER_EQUAL 6)
    qt_add_executable(TD9
            MANUAL_FINALIZATION
            ${PROJECT_SOURCES}
            )
else()
    if(ANDROID)
        add_library(TD9 SHARED ${PROJECT_SOURCES})
    else()
        add_executable(TD9 ${PROJECT_SOURCES})
    endif()
endif()

target_link_libraries(TD9
        PRIVATE
        Qt${QT_VERSION_MAJOR}::Core
        Qt${QT_VERSION_MAJOR}::Concurrent
        Qt${QT_VERSION_MAJOR}::Widgets
        )

# Identifiant bundle (optionnel selon version de Qt)
if((QT_VERSION VERSION_LESS 6.1.0) AND APPLE)
    set(BUNDLE_ID_OPTION MACOSX_BUNDLE_GUI_IDENTIFIER com.example.TD9)
endif()

set_target_properties(TD9 PROPERTIES
        ${BUNDLE_ID_OPTION}
        MACOSX_BUNDLE TRUE
        WIN32_EXECUTABLE TRUE
        MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
        MACOSX_BUNDLE_SHORT_VERSION_STRING
        ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
        )

include(GNUInstallDirs)
install(TARGETS TD9
        BUNDLE DESTINATION .
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        )

if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(TD9)
endif()