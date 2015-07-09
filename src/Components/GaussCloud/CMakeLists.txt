# Include the directory itself as a path to include directories
SET(CMAKE_INCLUDE_CURRENT_DIR ON)

# Create a variable containing all .cpp files:
FILE(GLOB files *.cpp)

# Find required packages
FIND_PACKAGE( OpenCV REQUIRED )
FIND_PACKAGE( PCL 1.7.0  REQUIRED )


# Create an executable file from sources:
ADD_LIBRARY(GaussCloud SHARED ${files})

# Link external libraries
TARGET_LINK_LIBRARIES(GaussCloud ${DisCODe_LIBRARIES} 
	${OpenCV_LIBS} ${PCL_LIBRARIES} ${PCL_COMMON_LIBRARIES}
	)

INSTALL_COMPONENT(GaussCloud)
