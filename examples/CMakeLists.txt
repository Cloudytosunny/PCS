cmake_minimum_required(VERSION 3.16)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)  
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(
	include
	${PCL_INCLUDE_DIRS})
	

set(TARGET pcs)

add_executable(${TARGET} src/main.cc)

if (UNIX)
  target_compile_options(${TARGET} PRIVATE -Wall -Wextra -Werror)
elseif (MSVC)
  target_compile_options(${TARGET} PRIVATE /W4 /WX)
endif ()

target_include_directories(${TARGET} PRIVATE
  ${PROJECT_SOURCE_DIR}
)

target_link_libraries(${TARGET} PRIVATE
  ${PCL_LIBRARIES} -lstdc++fs
)

