# Include the directory itself as a path to include directories
SET(CMAKE_INCLUDE_CURRENT_DIR ON)

# Create a variable containing all .cpp files:
FILE(GLOB files *.cpp)

# Create an executable file from sources:
ADD_LIBRARY(NormalEstimation SHARED ${files})

# Link external libraries
TARGET_LINK_LIBRARIES(NormalEstimation ${DisCODe_LIBRARIES})

INSTALL_COMPONENT(NormalEstimation)
