# Include the directory itself as a path to include directories
SET(CMAKE_INCLUDE_CURRENT_DIR ON)

# Create a variable containing all .cpp files:
FILE(GLOB files *.cpp)

# Create an executable file from sources:
ADD_LIBRARY(GreedyVerification SHARED ${files})

# Link external libraries
TARGET_LINK_LIBRARIES(GreedyVerification ${DisCODe_LIBRARIES})

INSTALL_COMPONENT(GreedyVerification)
