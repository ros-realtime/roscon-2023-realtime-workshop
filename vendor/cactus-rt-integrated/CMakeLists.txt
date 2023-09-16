cmake_minimum_required(VERSION 3.18)
project(cactus-rt-integrated)

add_subdirectory(quill)
get_target_property(QUILL_INC quill INTERFACE_INCLUDE_DIRECTORIES)
set_target_properties(quill PROPERTIES INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${QUILL_INC}")

add_subdirectory(readerwriterqueue)
get_target_property(READERWRITERQUEUE_INC readerwriterqueue INTERFACE_INCLUDE_DIRECTORIES)
set_target_properties(readerwriterqueue PROPERTIES INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${READERWRITERQUEUE_INC}")

set(CACTUS_RT_ENABLE_FETCH_DEPENDENCIES OFF)
add_subdirectory(cactus-rt)
