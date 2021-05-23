cmake_minimum_required(VERSION 3.12) # Define which CMAKE version
include(pico_sdk_import.cmake) # Use pico sdk
project(Studienarbeit_Display C CXX ASM) # Define Project Name
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init() # init pico sdk
add_executable(Studienarbeit_Display) 
pico_generate_pio_header(Studienarbeit_Display ${CMAKE_CURRENT_LIST_DIR}/Studienarbeit.pio)
add_library(kiss_fftr kiss_fftr.c)
add_library(kiss_fft kiss_fft.c)
target_link_libraries(kiss_fftr kiss_fft)
pico_enable_stdio_usb(Studienarbeit_Display 1)
pico_enable_stdio_uart(Studienarbeit_Display 1)
target_sources(Studienarbeit_Display PRIVATE Studienarbeit_Display.c)
target_link_libraries(Studienarbeit_Display PRIVATE pico_stdlib hardware_pio hardware_i2c hardware_adc hardware_dma kiss_fftr)# Welche libs werden denn verwendet?

pico_add_extra_outputs(Studienarbeit_Display) # Gerneriere UF2 und solche Dateiformate

add_custom_target(pio_squarewave_datasheet DEPENDS
		${CMAKE_CURRENT_LIST_DIR}/generated/Studienarbeit.hex
		${CMAKE_CURRENT_LIST_DIR}/generated/Studienarbeit.pio.h
		)
add_custom_command(OUTPUT ${CMAKE_CURRENT_LIST_DIR}/generated/Studienarbeit.hex
        DEPENDS ${CMAKE_CURRENT_LIST_DIR}/Studienarbeit.pio
        COMMAND Pioasm -o hex ${CMAKE_CURRENT_LIST_DIR}/Studienarbeit.pio ${CMAKE_CURRENT_LIST_DIR}/generated/Studienarbeit_Display.hex
        )
add_custom_command(OUTPUT ${CMAKE_CURRENT_LIST_DIR}/generated/Studienarbeit.pio.h
        DEPENDS ${CMAKE_CURRENT_LIST_DIR}/Studienarbeit.pio
        COMMAND Pioasm ${CMAKE_CURRENT_LIST_DIR}/Studienarbeit.pio ${CMAKE_CURRENT_LIST_DIR}/generated/Studienarbeit.pio.h
        )
add_dependencies(Studienarbeit_Display pio_squarewave_datasheet)