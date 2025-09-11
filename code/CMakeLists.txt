cmake_minimum_required(VERSION 3.28)

project(
        pnp
        VERSION 0.0.1
        LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Werror -Wall -Wextra -Wpedantic)

find_package(Eigen3 REQUIRED)

set(SRC_FILES
        src/dlt.cpp
        src/matrix_utilities.cpp
        src/pnp.cpp
)
add_library(${PROJECT_NAME} SHARED
        ${SRC_FILES}
)
target_include_directories(${PROJECT_NAME} PRIVATE
        include
)
target_link_libraries(${PROJECT_NAME}
        Eigen3::Eigen
)

include(FetchContent)
FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
FetchContent_MakeAvailable(googletest)
enable_testing()
include(GoogleTest)

set(TESTS
        src/dlt.test.cpp
        src/matrix_utilities.test.cpp
        src/multiple_view_geometry_data_generator.test.cpp
        test/pnp.test.cpp
)
foreach(TEST IN LISTS TESTS)
    get_filename_component(TEST_NAME ${TEST} NAME)
    add_executable(${TEST_NAME} ${TEST})
    target_include_directories(${TEST_NAME} PRIVATE
            include
    )
    target_link_libraries(${TEST_NAME}
            GTest::gtest_main
            ${PROJECT_NAME}
    )
    gtest_discover_tests(${TEST_NAME})
endforeach()

