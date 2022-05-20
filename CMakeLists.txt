cmake_minimum_required(VERSION 2.8.4)
project(Ardupilot)
add_compile_options(-std=c++11)
#set(TARGET_PLATFORM sitl)
set(TARGET_PLATFORM chibios)
#set(TARGET_PLATFORM linux)

string(TOUPPER ${TARGET_PLATFORM} TARGET_PLATFORM_UPPER)
# To work with SITL
add_definitions(-DSKETCHBOOK="./")
add_definitions(-DCONFIG_HAL_BOARD=HAL_BOARD_${TARGET_PLATFORM_UPPER})
if(${TARGET_PLATFORM} MATCHES sitl)
  add_definitions(-DCONFIG_HAL_BOARD_SUBTYPE=HAL_BOARD_SUBTYPE_NONE)
  set(SUB_TARGET_PLATFORM sitl)
elseif(${TARGET_PLATFORM} MATCHES chibios)
  add_definitions(-DCONFIG_HAL_BOARD_SUBTYPE=HAL_BOARD_SUBTYPE_CHIBIOS_FMUV3)
  set(SUB_TARGET_PLATFORM fmuv3)
endif()
add_definitions(-DHAL_WITH_UAVCAN)

set(AP_VERSION_DIRECTORY_ROOT build/${SUB_TARGET_PLATFORM})
set(MAVLINK_HEADER_DIRECTORY_ROOT ${AP_VERSION_DIRECTORY_ROOT}/libraries/GCS_MAVLink)
set(MAVLINK_HEADER_DIRECTORY ${MAVLINK_HEADER_DIRECTORY_ROOT}/include/mavlink/v2.0)

# Function:                 EXCLUDE_FILES_FROM_DIR_IN_LIST
# Description:              Exclude all files from a list under a specific directory.
# Param _InFileList:        Input and returned List
# Param _excludeDirName:    Name of the directory, which shall be ignored.
# Param _verbose:           Print the names of the files handled

FUNCTION (EXCLUDE_FILES_FROM_DIR_IN_LIST _InFileList _excludeDirName _verbose)
  foreach (ITR ${_InFileList})
    if ("${_verbose}")
      message(STATUS "ITR=${ITR}")
    endif ("${_verbose}")

    if ("${ITR}" MATCHES "(.*)${_excludeDirName}(.*)")                   # Check if the item matches the directory name in _excludeDirName
      message(STATUS "Remove Item from List:${ITR}")
      list (REMOVE_ITEM _InFileList ${ITR})                              # Remove the item from the list
    endif ("${ITR}" MATCHES "(.*)${_excludeDirName}(.*)")

  endforeach(ITR)
  set(SOURCE_FILES ${_InFileList} PARENT_SCOPE)                          # Return the SOURCE_FILES variable to the calling parent
ENDFUNCTION (EXCLUDE_FILES_FROM_DIR_IN_LIST)

# we just grab all the cpp and h files. not ideal, but works well enough
# means a "reload CMake project" when you add new files
file(GLOB_RECURSE SOURCE_FILES APMrover2/*.cpp APMrover2/*.h)
file(GLOB_RECURSE SOURCE_FILES ArduCopter/*.cpp ArduCopter/*.h)
file(GLOB_RECURSE SOURCE_FILES ArduPlane/*.cpp ArduPlane/*.h)
file(GLOB_RECURSE SOURCE_FILES libraries/*.cpp libraries/*.h)
file(GLOB_RECURSE MAVLINK_HEADER_FILES ${MAVLINK_HEADER_DIRECTORY}/*.h)



EXCLUDE_FILES_FROM_DIR_IN_LIST("${SOURCE_FILES}" "/libraries/AP_HAL_F4Light/support/minimosd-extra/Character_Updater/" FALSE)
EXCLUDE_FILES_FROM_DIR_IN_LIST("${SOURCE_FILES}" "/libraries/AP_HAL_F4Light/hardware/osd/osd_core/" FALSE)


include_directories(APMrover2)
include_directories(ArduCopter)
include_directories(ArduPlane)
include_directories(libraries)
include_directories(modules)
include_directories(${MAVLINK_HEADER_DIRECTORY})
include_directories(build/${SUB_TARGET_PLATFORM})



add_executable(fake-target-for-clion ${SOURCE_FILES} ${MAVLINK_HEADER_FILES})
set_target_properties(fake-target-for-clion PROPERTIES LINKER_LANGUAGE CXX)
target_include_directories(fake-target-for-clion PUBLIC ${MAVLINK_HEADER_DIRECTORY} ${MAVLINK_HEADER_DIRECTORY_ROOT} ${AP_VERSION_DIRECTORY_ROOT})
