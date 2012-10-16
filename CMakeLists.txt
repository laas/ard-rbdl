# Requires at least CMake 2.6 to configure the package.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(CXX_DISABLE_WERROR TRUE)
INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/ros.cmake)

SET(PROJECT_NAME ard-rbdl)
SET(PROJECT_DESCRIPTION
  "Wrapping of RBDL dynamic library in abstract-robot-dynamics.")
SET(PROJECT_URL "")

# Where to compile shared objects
SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

SET(${PROJECT_NAME}_TOOLS_HEADERS
  include/ard/rbdl/tools/fwd.hh
  include/ard/rbdl/tools/types.hh
  include/ard/rbdl/tools/pointer-util.hh
  include/ard/rbdl/tools/math-util.hh
  )

SET(${PROJECT_NAME}_MODEL_HEADERS
  include/ard/rbdl/model/joint.hh
  include/ard/rbdl/model/body.hh
  include/ard/rbdl/model/dynamic-robot.hh
  include/ard/rbdl/model/hand.hh
  include/ard/rbdl/model/foot.hh
  include/ard/rbdl/model/humanoid-dynamic-robot.hh
  include/ard/rbdl/model/robot-dynamics-object-factory.hh
  )

INSTALL(FILES ${${PROJECT_NAME}_TOOLS_HEADERS}
  DESTINATION include/ard/rbdl/tools
  )

INSTALL(FILES ${${PROJECT_NAME}_MODEL_HEADERS}
  DESTINATION include/ard/rbdl/model
  )

SETUP_PROJECT()

# Declare dependencies
SEARCH_FOR_BOOST()
ADD_REQUIRED_DEPENDENCY("abstract-robot-dynamics")
ADD_REQUIRED_DEPENDENCY("jrl-mal")
ADD_ROSPACK_DEPENDENCY("rbdl")

PKG_CONFIG_APPEND_LIBS(${PROJECT_NAME})

ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(tests)

SETUP_PROJECT_FINALIZE()
