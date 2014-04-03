# Include the directory itself as a path to include directories
SET(CMAKE_INCLUDE_CURRENT_DIR ON)

# Create a variable containing all .cpp files:
FILE(GLOB files *.cpp)

# Find required packages
FIND_PACKAGE( OpenCV REQUIRED )

# Create an executable file from sources:
ADD_LIBRARY(CorrespondencesViewer SHARED ${files})

# Link external libraries
TARGET_LINK_LIBRARIES(CorrespondencesViewer ${OpenCV_LIBS} ${DisCODe_LIBRARIES} ${PCL_LIBRARIES} )

INSTALL_COMPONENT(CorrespondencesViewer)
