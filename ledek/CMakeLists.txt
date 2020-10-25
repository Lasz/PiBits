cmake_minimum_required( VERSION 3.12..3.16 )

set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib )
set( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib )
set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin )

project( ledek
         LANGUAGES C
         VERSION 0.1
         DESCRIPTION ""
)
set( CMAKE_C_STANDARD 11 )

set( EXEC_NAME ledek )
list( APPEND SOURCE_FILES
        clk.c
        dma.c
        gpio.c
        hardware.c
        mailbox.c
        main.c
        pwm.c
        servod.c
)
list( APPEND HEADER_FILES
        clk.h
        dma.h
        gpio.h
        hardware.h
        mailbox.h
        pwm.h
)

set( CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} -Wall -g -O2 )
add_executable( ${EXEC_NAME} ${SOURCE_FILES} ${HEADER_FILES} )
target_link_libraries( ${EXEC_NAME} PRIVATE m bcm_host )

