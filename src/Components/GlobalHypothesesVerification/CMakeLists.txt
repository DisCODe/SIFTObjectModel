# Include the directory itself as a path to include directories
SET(CMAKE_INCLUDE_CURRENT_DIR ON)

# Create a variable containing all .cpp files:
FILE(GLOB files *.cpp)

# Create an executable file from sources:
ADD_LIBRARY(GlobalHypothesesVerification SHARED ${files})

# Link external libraries
TARGET_LINK_LIBRARIES(GlobalHypothesesVerification ${DisCODe_LIBRARIES})

INSTALL_COMPONENT(GlobalHypothesesVerification)
