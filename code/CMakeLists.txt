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
        test_pnp
)
foreach(TEST IN LISTS TESTS)
    add_executable(${TEST} test/${TEST}.cpp)
    target_include_directories(${TEST} PRIVATE
            include
    )
    target_link_libraries(${TEST}
            GTest::gtest_main
            ${PROJECT_NAME}
    )
    gtest_discover_tests(${TEST})
endforeach()

