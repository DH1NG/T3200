add_executable(Studienarbeit)

# by default the header is generated into the build dir
pico_generate_pio_header(Studienarbeit ${CMAKE_CURRENT_LIST_DIR}/Studienarbeit.pio)
# however, alternatively you can choose to generate it somewhere else (in this case in the source tree for check in)
#pico_generate_pio_header(pio_blink ${CMAKE_CURRENT_LIST_DIR}/blink.pio OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR})

target_sources(Studienarbeit PRIVATE Studienarbeit.c)

target_link_libraries(Studienarbeit PRIVATE pico_stdlib hardware_pio)
pico_add_extra_outputs(Studienarbeit)

# add url via pico_set_program_url
example_auto_set_url(Studienarbeit)