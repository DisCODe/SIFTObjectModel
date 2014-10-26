# Include the directory itself as a path to include directories
SET(CMAKE_INCLUDE_CURRENT_DIR ON)

# Create a variable containing all .cpp files:
FILE(GLOB files *.cpp)

# Find required packages


# Create an executable file from sources:
ADD_LIBRARY(PC2SOM SHARED ${files})

# Link external libraries
TARGET_LINK_LIBRARIES(PC2SOM ${DisCODe_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})

INSTALL_COMPONENT(PC2SOM)
