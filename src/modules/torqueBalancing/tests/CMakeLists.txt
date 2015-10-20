add_subdirectory(balancingTest)
add_subdirectory(matlabTorqueBalancingTest)

# copy all suites and fixture files in build for running the actual tests
file(GLOB xmlTests ${CMAKE_CURRENT_SOURCE_DIR}/*.xml)
file(GLOB xmlFixtures ${CMAKE_CURRENT_SOURCE_DIR}/fixtures/*.xml)

add_custom_target(copy_xml_tests_in_build ALL)
foreach(xmlTest ${xmlTests})
  add_custom_command(TARGET copy_xml_tests_in_build PRE_BUILD
                     COMMAND ${CMAKE_COMMAND} -E
                         copy ${xmlTest} ${CMAKE_CURRENT_BINARY_DIR})
endforeach()
foreach(xmlFixture ${xmlFixtures})
  add_custom_command(TARGET copy_xml_tests_in_build PRE_BUILD
                     COMMAND ${CMAKE_COMMAND} -E
                         copy ${xmlFixture} ${CMAKE_CURRENT_BINARY_DIR}/fixtures)
endforeach()