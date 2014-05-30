SET(CMAKE_INCLUDE_CURRENT_DIR ON)

include_directories(.)

# ##############################################################################
# Find required libraries
# ##############################################################################

# Find Boost, at least ver. 1.41
FIND_PACKAGE(Boost 1.41.0 REQUIRED)
include_directories(SYSTEM ${Boost_INCLUDE_DIR})

# Find another necessary libraries

FIND_PACKAGE(PCL 1.3 REQUIRED COMPONENTS io visualization)
include_directories(${PCL_INCLUDE_DIRS})
MESSAGE("PCL include: ${PCL_INCLUDE_DIRS}")
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Find another DCLs our one depends on
# Use macro: DISCODE_FIND_DCL(<DCL_NAME>)

DISCODE_FIND_DCL(CvCoreTypes)
DISCODE_FIND_DCL(CvBasic)
DISCODE_FIND_DCL(PCL)

# ##############################################################################
# Add source directories
# ##############################################################################

# CvBlobs components
ADD_SUBDIRECTORY(Components)

# CvBlobs types
ADD_SUBDIRECTORY(Types)

# Prepare config file to use from another DCLs
CONFIGURE_FILE(SIFTObjectModelConfig.cmake.in ${CMAKE_INSTALL_PREFIX}/SIFTObjectModelConfig.cmake @ONLY)
