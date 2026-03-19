# - Find GEOS
# Find the native GEOS includes and library
#
#  GEOS_INCLUDE_DIRS    - where to find GEOS includes
#  GEOS_LIBRARIES       - List of libraries when using GEOS.
#  GEOS_FOUND           - True if GEOS found.

# Check if GEOS_INCLUDE_DIRS is already in cache
if (GEOS_INCLUDE_DIRS)
  # Already in cache, be silent
  set (GEOS_FIND_QUIETLY TRUE)
endif (GEOS_INCLUDE_DIRS)

# Find the path to the GEOS include directories
find_path (GEOS_INCLUDE_DIRS
    geos.h
    geos_c.h
    /usr/local/include/
)

# Find the GEOS libraries
find_library (GEOS_LIBRARIES 
    NAMES geos_c
    PATHS
    /usr/local/lib/
)

# Handle the QUIETLY and REQUIRED arguments and set GEOS_FOUND to TRUE if
# all listed variables are TRUE
include (FindPackageHandleStandardArgs)
find_package_handle_standard_args (GEOS DEFAULT_MSG GEOS_LIBRARIES GEOS_INCLUDE_DIRS)

# Mark as advanced
mark_as_advanced (GEOS_LIBRARIES GEOS_INCLUDE_DIRS)

# Create an imported target if not already defined
if(NOT TARGET GEOS)
  add_library(GEOS INTERFACE IMPORTED)
  set_target_properties(GEOS PROPERTIES 
    INTERFACE_LINK_LIBRARIES "${GEOS_LIBRARIES}" 
    INTERFACE_INCLUDE_DIRECTORIES "${GEOS_INCLUDE_DIRS}"
  )
  
  # Define an alias target for GEOS to create a namespaced target
#   add_library(GEOS::GEOS ALIAS GEOS)
endif()
