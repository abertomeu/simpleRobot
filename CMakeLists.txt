cmake_minimum_required(VERSION 3.10)

project(simpleRobot
	VERSION 1.0
	DESCRIPTION "Simple library for computing robot kinematics"
	LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

option(SIMPLEROBOT_STATIC_LIB "Build static lybrary" ON)

if (SIMPLEROBOT_STATIC_LIB)
	add_library(${PROJECT_NAME} STATIC
		src/simpleRobot.cpp
		src/matrix.cpp
		src/vector.cpp
		src/quaternion.cpp
		)
else()
	add_library(${PROJECT_NAME} SHARED
		src/simpleRobot.cpp
		src/matrix.cpp
		src/vector.cpp
		src/quaternion.cpp
		)
endif()

set_target_properties(${PROJECT_NAME} 
	PROPERTIES VERSION ${PROJECT_VERSION})
	
set_target_properties(${PROJECT_NAME}
	PROPERTIES PUBLIC_HEADER 
	include/simpleRobot.h
	)
	
target_include_directories(${PROJECT_NAME} PRIVATE include)
target_include_directories(${PROJECT_NAME} PRIVATE src)

install(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
	
configure_file(simpleRobot.pc.in simpleRobot.pc @ONLY)

install(FILES ${CMAKE_BINARY_DIR}/simpleRobot.pc 
	DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/pkgconfig)

# SAMPLES	
list(APPEND SIMPLEROBOT_LIBS simpleRobot)
list(APPEND SIMPLEROBOT_INCLUDES "${PROJECT_SOURCE_DIR}/include")

add_executable(sample samples/howto_simpleRobot.cpp)

target_link_libraries(sample PUBLIC ${SIMPLEROBOT_LIBS})
target_include_directories(sample PUBLIC
						   "${PROJECT_BINARY_DIR}"
						   ${SIMPLEROBOT_INCLUDES}
						   )
						   