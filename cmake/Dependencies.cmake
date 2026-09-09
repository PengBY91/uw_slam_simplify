option(UW_BUILD_TESTS "Build the contract, unit, and integration tests" ON)

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV 4 REQUIRED COMPONENTS core calib3d imgproc)

include(UwProtobuf)
include(UwMcap)

if(UW_BUILD_TESTS)
  enable_testing()
  find_package(GTest REQUIRED)
  find_package(Threads REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
endif()

