cmake_minimum_required(VERSION 3.8)
project(gtsam_segfault)

# Find GTSAM components
find_package(GTSAM REQUIRED) # Uses installed package
include_directories(${GTSAM_INCLUDE_DIR})
set(GTSAM_LIBRARIES gtsam)   # TODO: automatic search libs

add_executable(gtsam_opt gtsam_opt.cpp GraphOpt.cpp)
target_link_libraries(gtsam_opt PUBLIC ${GTSAM_LIBRARIES})
target_include_directories(gtsam_opt PRIVATE ".")
target_compile_features(gtsam_opt PRIVATE cxx_std_17)
