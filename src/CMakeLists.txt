SET(CMAKE_INCLUDE_CURRENT_DIR ON)

include_directories(.)

# ##############################################################################
# Find required libraries
# ##############################################################################

# Find Boost, at least ver. 1.41
FIND_PACKAGE(Boost 1.41.0 REQUIRED COMPONENTS system thread filesystem date_time)
include_directories(SYSTEM ${Boost_INCLUDE_DIR})

# Find another necessary libraries
FIND_PACKAGE(PCL 1.7.2 REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Find another DCLs our one depends on
# Use macro: DISCODE_FIND_DCL(<DCL_NAME>)
DISCODE_FIND_DCL(PCLCoreTypes)
DISCODE_FIND_DCL(CvCoreTypes)
DISCODE_FIND_DCL(CvBasic)

#DISCODE_FIND_DCL(PCL)

# Add standard libraries for this DCL: Boost & PCL
MESSAGE(STATUS "${PCL_LIBRARIES}")
SET(DisCODe_LIBRARIES ${DisCODe_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

# Validate whether components are properly linked to utilized libraries.
SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-z,defs") 

# ##############################################################################
# Add source directories
# ##############################################################################

# CvBlobs components
ADD_SUBDIRECTORY(Components)

# CvBlobs types
ADD_SUBDIRECTORY(Types)

# Prepare config file to use from another DCLs
CONFIGURE_FILE(SIFTObjectModelConfig.cmake.in ${CMAKE_INSTALL_PREFIX}/SIFTObjectModelConfig.cmake @ONLY)
