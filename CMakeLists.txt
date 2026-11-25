cmake_minimum_required(VERSION 3.16)

if(DEFINED ESP_PLATFORM)
    idf_component_register(
        SRCS
            "src/od_soc.cpp"
            "src/od_soc_util.cpp"
            "src/od_history.cpp"
            "src/ip_frame.cpp"
            "src/od_soc_c.cpp"
            "boards/m5stickc_plus/src/axp192_sensor.cpp"
            "boards/m5stickc_plus/src/fuel_gauge.cpp"
        INCLUDE_DIRS
            "include"
            "boards/m5stickc_plus/include"
    )
    return()
endif()

project(od_soc LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(od_soc STATIC
    src/od_soc.cpp
    src/od_soc_util.cpp
    src/od_history.cpp
    src/ip_frame.cpp
    src/od_soc_c.cpp
    boards/m5stickc_plus/src/axp192_sensor.cpp
    boards/m5stickc_plus/src/fuel_gauge.cpp
)
target_include_directories(od_soc PUBLIC
    include
    boards/m5stickc_plus/include
)
target_compile_options(od_soc PRIVATE -Wall -Wextra -Wpedantic -fno-exceptions -fno-rtti)

enable_testing()
add_executable(test_od_soc tests/test_od_soc.cpp)
target_link_libraries(test_od_soc PRIVATE od_soc)
target_include_directories(test_od_soc PRIVATE sim)
add_test(NAME test_od_soc COMMAND test_od_soc)
