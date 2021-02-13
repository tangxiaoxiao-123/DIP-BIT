cmake_minimum_required(VERSION 3.18)
set(OpenCV_DIR D:/OPENCV)
find_package(OpenCV REQUIRED)
project (DIPlanes)# 工程名

set(SOURCES
    src/main.cpp
    src/myGuass.cpp
    src/myCanny.cpp
    src/myHough.cpp
)#创建一个变量，名字叫SOURCE。它包含了所有的cpp文件。

add_executable(DIPlanes ${SOURCES})#用所有的源文件生成一个可执行文件，因为这里定义了SOURCE变量，所以就不需要罗列cpp文件了

include_directories( ${OpenCV_INCLUDE_DIRS} )
target_include_directories(DIPlanes
    PRIVATE 
    ${PROJECT_SOURCE_DIR}/include
)
target_link_libraries( DIPlanes ${OpenCV_LIBS} )
target_link_libraries( DIPlanes myJNI)
add_library( # Sets the name of the library.
        myJNI

        # Sets the library as a shared library.
        SHARED

        # Provides a relative path to your source file(s)
        jsoncpp/json_tool.h
        jsoncpp/json_reader.cpp
        jsoncpp/json_valueiterator.inl
        jsoncpp/json_value.cpp
        jsoncpp/json_writer.cpp
        jsoncpp/assertions.h
        jsoncpp/allocator.h
        jsoncpp/config.h
        jsoncpp/json_features.h
        jsoncpp/forwards.h
        jsoncpp/json.h
        jsoncpp/reader.h
        jsoncpp/value.h
        jsoncpp/version.h
        jsoncpp/writer.h
        )
