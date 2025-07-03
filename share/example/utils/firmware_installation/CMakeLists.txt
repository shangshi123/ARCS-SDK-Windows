cmake_minimum_required(VERSION 3.16)
project(firmware_install)
set(CMAKE_CXX_STANDARD 17)
add_compile_options("$<$<C_COMPILER_ID:MSVC>:/utf-8>")
add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/utf-8>")

macro(IMPORT_SDK SDK)
    if(DEFINED USING_SDK_SOURCE_CODE)
        message(STATUS "Use aubo_sdk source code.")
        set(SDK_LIB ${ARGV0})
        if("${ARGV1}" STREQUAL "robot_proxy")
            set(PROXY_LIB ${ARGV1})
        endif()
    else()
        string(REGEX REPLACE "(.+)\\share/example/utils.*" "\\1" AUBO_SDK_PATH ${PROJECT_SOURCE_DIR})
        message(STATUS "FOUND AUBO_SDK PATH ${AUBO_SDK_PATH}")
        set(CMAKE_PREFIX_PATH ${AUBO_SDK_PATH})
        find_package(aubo_sdk REQUIRED aubo_sdk)
        message(STATUS "Import aubo_sdk::${ARGV0}")
        set(SDK_LIB aubo_sdk::${ARGV0})
        if("${ARGV1}" STREQUAL "robot_proxy")
            message(STATUS "Import aubo_sdk::${ARGV1}")
            set(PROXY_LIB aubo_sdk::${ARGV1})
        endif()
    endif()
endmacro(IMPORT_SDK)


add_executable(firmware_install firmware_installation.cpp)
target_link_libraries(firmware_install aubo_sdk -lpthread)
set_target_properties(firmware_install PROPERTIES
    INSTALL_RPATH "\${ORIGIN}/../lib"
)

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION bin)
INSTALL_DEPS($<TARGET_FILE:firmware_install>)

