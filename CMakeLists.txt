cmake_minimum_required(VERSION 3.12) # Define which CMAKE version

include(pico_sdk_import.cmake) # Use pico sdk

project(Studienarbeit_Display C CXX ASM) # Define Project Name
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init() # init pico sdk


add_executable(Studienarbeit_Display) 
pico_generate_pio_header(Studienarbeit_Display ${CMAKE_CURRENT_LIST_DIR}/Studienarbeit.pio)

target_sources(Studienarbeit_Display PRIVATE Studienarbeit_Display.c)
target_link_libraries(Studienarbeit_Display PRIVATE pico_stdlib hardware_pio hardware_i2c)# Welche libs werden denn verwendet?



# by default the header is generated into the build dir
#pico_generate_pio_header(Studienarbeit ${CMAKE_CURRENT_LIST_DIR}/Studienarbeit.pio)
# however, alternatively you can choose to generate it somewhere else (in this case in the source tree for check in)
#pico_generate_pio_header(pio_blink ${CMAKE_CURRENT_LIST_DIR}/blink.pio OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR})

pico_add_extra_outputs(Studienarbeit_Display) # Gerneriere UF2 und solche Dateiformate

#add_subdirectory(lib)
#add_library(font font.h)
#add_library(ssd1306 ssd1306.h)
#add_library(twi twi.h)

#target_link_libraries(Studienarbeit_Display twi)

# add url via pico_set_program_url

