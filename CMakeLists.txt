CMAKE_MINIMUM_REQUIRED(VERSION 3.0)
SET(PROJECT_NAME "annotate")
SET(PROJECT_TOOL "extract")

PROJECT("${PROJECT_NAME}")

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# add opencv package to the project
FIND_PACKAGE( OpenCV REQUIRED )
INCLUDE_DIRECTORIES( ${OpenCV_INCLUDE_DIRS} ) 

SUBDIRS(vivalib skcf)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/vivalib) 
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/skcf)

FILE(GLOB files
  "*.h"
  "*.hpp"
  "*.cpp"
)
FILE(GLOB proj_files
	"*.hpp"
	"main.cpp"
	"aprocess.*"
)
FILE(GLOB tool_files
	"*.hpp"
	"aprocess.*"
	"extract.cpp"
)
FILE(GLOB hidden
	".*"
)
FILE(GLOB resources
  "*.*"
)

MESSAGE(STATUS "Project Name: ${PROJECT_NAME}")
MESSAGE(STATUS "    OpenCV version: ${OpenCV_VERSION} ")
MESSAGE(STATUS "    libraries: ${OpenCV_LIBS}")
MESSAGE(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

ADD_EXECUTABLE(${PROJECT_NAME} ${proj_files})
ADD_EXECUTABLE(${PROJECT_TOOL} ${tool_files})
 
TARGET_LINK_LIBRARIES( ${PROJECT_NAME} ${OpenCV_LIBS} vivalib skcf)
TARGET_LINK_LIBRARIES( ${PROJECT_TOOL} ${OpenCV_LIBS} vivalib skcf)

LIST(REMOVE_ITEM resources ${files} ${hidden} "${CMAKE_SOURCE_DIR}/CMakeLists.txt")
FILE(COPY ${resources} DESTINATION "Debug")
FILE(COPY ${resources} DESTINATION "Release")





