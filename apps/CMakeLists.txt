include_directories(${CMAKE_SOURCE_DIR}/kfusion/include)
set (CMAKE_CXX_STANDARD 11)


#demo

find_package(PhoXi REQUIRED CONFIG PATHS "$ENV{PHOXI_CONTROL_PATH}")
file(GLOB srcs demo.cpp *.hpp *.c *.h)
include_directories(${PHOXI_INCLUDE_DIRS})

add_executable(demo ${srcs})
target_link_libraries(demo ${OpenCV_LIBS} kfusion ${PHOXI_LIBRARY})

set_target_properties(demo PROPERTIES
    DEBUG_POSTFIX "d"
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

install(TARGETS demo RUNTIME DESTINATION bin COMPONENT main)
install(FILES ${srcs} DESTINATION app COMPONENT main)


#demo2

find_package(PhoXi REQUIRED CONFIG PATHS "$ENV{PHOXI_CONTROL_PATH}")
file(GLOB srcs2 demo2.cpp *.hpp *.c *.h)
include_directories(${PHOXI_INCLUDE_DIRS})


add_executable(demo2 ${srcs2})
target_link_libraries(demo2 ${OpenCV_LIBS} kfusion ${PHOXI_LIBRARY})

set_target_properties(demo2 PROPERTIES
        DEBUG_POSTFIX "d"
        ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

install(TARGETS demo2 RUNTIME DESTINATION bin COMPONENT main)
install(FILES ${srcs2} DESTINATION app COMPONENT main)
