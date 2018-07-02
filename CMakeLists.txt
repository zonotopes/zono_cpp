cmake_minimum_required(VERSION 3.1)
project(zonotope)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/..")

if (${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU")
add_definitions(-Wno-ignored-attributes)
add_definitions(-Wno-write-strings)
add_definitions(-Wno-deprecated-declarations)
endif()

include_directories(Eigen)

add_library(libzonotope STATIC zonotope.cpp)
set_property(TARGET libzonotope PROPERTY POSITION_INDEPENDENT_CODE ON)

add_executable(zonotope main.cpp)
target_link_libraries(zonotope libzonotope)

add_library(zonostata2 MODULE statamain.cpp)
target_link_libraries(zonostata2 libzonotope)
set_target_properties(zonostata2 PROPERTIES PREFIX "" SUFFIX ".plugin")

add_library(zonostata3 MODULE statamain.cpp)
target_link_libraries(zonostata3 libzonotope)
target_compile_definitions(zonostata3 PRIVATE SD_PLUGINMAJ=3)
set_target_properties(zonostata3 PROPERTIES PREFIX "" SUFFIX ".plugin")

install(TARGETS zonotope zonostata2 zonostata3
        RUNTIME DESTINATION zono_standalone
        LIBRARY DESTINATION zono_stata/plugin)

enable_testing()

add_executable(test_zonotope test_zonotope.cpp)
target_link_libraries(test_zonotope libzonotope)

add_executable(test_zonotope_single test_zonotope_single.cpp)
target_link_libraries(test_zonotope_single libzonotope)

add_test(R3gen10.txt test_zonotope_single ${PROJECT_SOURCE_DIR}/zono_data/R3gen10.txt 20.19476270431274)
add_test(R3gen10_WP931_YEAR1.txt test_zonotope_single ${PROJECT_SOURCE_DIR}/zono_data/R3gen10_WP931_YEAR1.txt 8265)
add_test(R4gen20.txt test_zonotope_single ${PROJECT_SOURCE_DIR}/zono_data/R4gen20.txt 142.99687395286884)
add_test(R5gen10.txt test_zonotope_single ${PROJECT_SOURCE_DIR}/zono_data/R5gen10.txt 1.2721697593853392)
add_test(R5gen13_Le.txt test_zonotope_single ${PROJECT_SOURCE_DIR}/zono_data/R5gen13_Le.txt 523522786344730427392)
add_test(R5gen23_Le.txt test_zonotope_single ${PROJECT_SOURCE_DIR}/zono_data/R5gen23_Le.txt 1391102746013858529280)
add_test(R5gen30.txt test_zonotope_single ${PROJECT_SOURCE_DIR}/zono_data/R5gen30.txt 675.74349104380292)
add_test(R5gen43_Le.txt test_zonotope_single ${PROJECT_SOURCE_DIR}/zono_data/R5gen43_Le.txt 18023335216112354721792)
add_test(R5gen50_Le.txt test_zonotope_single ${PROJECT_SOURCE_DIR}/zono_data/R5gen50_Le.txt 51438444390.579117)
