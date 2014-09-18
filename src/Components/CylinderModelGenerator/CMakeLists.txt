# Include the directory itself as a path to include directories
SET(CMAKE_INCLUDE_CURRENT_DIR ON)

# Create a variable containing all .cpp files:
FILE(GLOB files *.cpp)

# Find required packages
FIND_PACKAGE( OpenCV REQUIRED )

FIND_FILE(NONFREE_FEATURES_HPP "opencv2/nonfree/features2d.hpp" PATHS ${OpenCV_INCLUDE_DIRS})

IF (NONFREE_FEATURES_HPP OR ${OpenCV_VERSION} VERSION_LESS 2.4)

# Create an executable file from sources:
ADD_LIBRARY(CylinderModelGenerator SHARED ${files})

# Link external libraries
TARGET_LINK_LIBRARIES(CylinderModelGenerator ${DisCODe_LIBRARIES} 
	${OpenCV_LIBS}
	${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} )

INSTALL_COMPONENT(CylinderModelGenerator)

ELSE()

MESSAGE(STATUS "Skipping CylinderModelGenerator - required nonfree module not found.")

ENDIF()
