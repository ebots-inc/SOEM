
set(SOURCES simple_test.c)
add_executable(simple_test ${SOURCES})
target_link_libraries(simple_test soem)
install(TARGETS simple_test DESTINATION bin)

set(SOURCES custom_test.c)
add_executable(custom_test ${SOURCES})
target_link_libraries(custom_test soem)
install(TARGETS custom_test DESTINATION bin)
