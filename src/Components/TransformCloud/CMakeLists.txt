# Include the directory itself as a path to include directories
SET(CMAKE_INCLUDE_CURRENT_DIR ON)

# Create a variable containing all .cpp files:
FILE(GLOB files *.cpp)

# Find required packages

# Create an executable file from sources:
ADD_LIBRARY(TransformCloud SHARED ${files})
FIND_PACKAGE( PCL 1.7.0 REQUIRED )
FIND_PACKAGE (OpenCV REQUIRED )

# Link external libraries
TARGET_LINK_LIBRARIES(TransformCloud ${DisCODe_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${OpenCV_LIBS})

INSTALL_COMPONENT(TransformCloud)
