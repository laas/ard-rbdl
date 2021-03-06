# Add Boost path to include directories.
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})

# Make Boost.Test generates the main function in test cases.
ADD_DEFINITIONS(-DBOOST_TEST_DYN_LINK -DBOOST_TEST_MAIN)

# ADD_TESTCASE(NAME)
# ------------------------
#
# Define a test named `NAME'.
#
# This macro will create a binary from `NAME.cc', link it against
# Boost and add it to the test suite.
#
MACRO(ADD_TESTCASE NAME)
  ADD_EXECUTABLE(${NAME} ${CMAKE_CURRENT_SOURCE_DIR}/${NAME}.cc)
  ADD_TEST(${NAME} ${RUNTIME_OUTPUT_DIRECTORY}/${NAME})

  PKG_CONFIG_USE_DEPENDENCY(${NAME} abstract-robot-dynamics)
  PKG_CONFIG_USE_DEPENDENCY(${NAME} jrl-mal)
  ROSPACK_USE_DEPENDENCY(${NAME} rbdl)

  # Link against package library.
  TARGET_LINK_LIBRARIES(${NAME} 
    ${Boost_LIBRARIES}
    ${PROJECT_NAME})
ENDMACRO(ADD_TESTCASE)

# Generated tests.
ADD_TESTCASE(create-joint)
ADD_TESTCASE(create-body)
ADD_TESTCASE(create-hand)
ADD_TESTCASE(create-foot)
ADD_TESTCASE(create-dynamic-robot)
ADD_TESTCASE(create-humanoid-dynamic-robot)
