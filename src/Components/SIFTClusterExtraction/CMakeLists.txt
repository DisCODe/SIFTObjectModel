# Include the directory itself as a path to include directories
SET(CMAKE_INCLUDE_CURRENT_DIR ON)

# Create a variable containing all .cpp files:
FILE(GLOB files *.cpp)

# Find required packages


# Create an executable file from sources:
ADD_LIBRARY(SIFTClusterExtraction SHARED ${files})

# Link external libraries
TARGET_LINK_LIBRARIES(SIFTClusterExtraction ${DisCODe_LIBRARIES} ${PCL_LIBRARIES} ${PCL_SEGMENTATION_LIBRARIES} ${PCL_REGISTRATION_LIBRARIES} ${PCL_RECOGNITION_LIBRARIES})

INSTALL_COMPONENT(SIFTClusterExtraction)
