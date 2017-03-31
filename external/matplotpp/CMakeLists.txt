cmake_minimum_required ( VERSION 2.8.12.2 )
project ( matplotpp )

list (APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
find_package ( OpenGL REQUIRED )
find_package ( GLEW REQUIRED )
find_package ( GLUT REQUIRED )

set ( COMMON_INCLUDES ${PROJECT_SOURCE_DIR}/include )

include_directories ( 
    ${COMMON_INCLUDES}
    ${OPENGL_INCLUDE_DIRS} 
    ${GLEW_INCLUDE_DIRS} 
    ${GLUT_INCLUDE_DIRS} 
)

# Library

set ( LINK_LIBS ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} )

file ( GLOB SRCS ${PROJECT_SOURCE_DIR}/src/*.* )

add_library ( matplotpp STATIC ${SRCS} )

target_include_directories ( matplotpp PUBLIC ${COMMON_INCLUDES} )

target_link_libraries ( matplotpp LINK_PUBLIC ${LINK_LIBS} )

install ( DIRECTORY ${PROJECT_SOURCE_DIR}/include/ DESTINATION include )
install ( TARGETS matplotpp DESTINATION lib )

# Examples

function ( add_example EX_PATH )
    get_filename_component ( EX_NAME ${EX_PATH} NAME_WE )
    add_executable ( ${EX_NAME} ${EX_PATH} )
    target_link_libraries ( ${EX_NAME} matplotpp )
endfunction ( add_example )

file ( GLOB EXAMPLES ${PROJECT_SOURCE_DIR}/examples/*.cc )

foreach ( EXAMPLE ${EXAMPLES} )
    add_example ( ${EXAMPLE} )  
endforeach ( EXAMPLE )
