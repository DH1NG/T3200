# CMAKE generated file: DO NOT EDIT!
# Generated by "NMake Makefiles" Generator, CMake Version 3.19

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


.SUFFIXES: .hpux_make_needs_suffix_list


# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

!IF "$(OS)" == "Windows_NT"
NULL=
!ELSE
NULL=nul
!ENDIF
SHELL = cmd.exe

# The CMake executable.
CMAKE_COMMAND = "C:\Program Files\CMake\bin\cmake.exe"

# The command to remove a file.
RM = "C:\Program Files\CMake\bin\cmake.exe" -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = "D:\Pico\Studienarbeit - Display"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "D:\Pico\Studienarbeit - Display\build"

# Utility rule file for Studienarbeit_Display_Studienarbeit_pio_h.

# Include the progress variables for this target.
include CMakeFiles\Studienarbeit_Display_Studienarbeit_pio_h.dir\progress.make

CMakeFiles\Studienarbeit_Display_Studienarbeit_pio_h: Studienarbeit.pio.h


Studienarbeit.pio.h: ..\Studienarbeit.pio
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir="D:\Pico\Studienarbeit - Display\build\CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Generating Studienarbeit.pio.h"
	pioasm\pioasm.exe -o c-sdk "D:/Pico/Studienarbeit - Display/Studienarbeit.pio" "D:/Pico/Studienarbeit - Display/build/Studienarbeit.pio.h"

Studienarbeit_Display_Studienarbeit_pio_h: CMakeFiles\Studienarbeit_Display_Studienarbeit_pio_h
Studienarbeit_Display_Studienarbeit_pio_h: Studienarbeit.pio.h
Studienarbeit_Display_Studienarbeit_pio_h: CMakeFiles\Studienarbeit_Display_Studienarbeit_pio_h.dir\build.make

.PHONY : Studienarbeit_Display_Studienarbeit_pio_h

# Rule to build all files generated by this target.
CMakeFiles\Studienarbeit_Display_Studienarbeit_pio_h.dir\build: Studienarbeit_Display_Studienarbeit_pio_h

.PHONY : CMakeFiles\Studienarbeit_Display_Studienarbeit_pio_h.dir\build

CMakeFiles\Studienarbeit_Display_Studienarbeit_pio_h.dir\clean:
	$(CMAKE_COMMAND) -P CMakeFiles\Studienarbeit_Display_Studienarbeit_pio_h.dir\cmake_clean.cmake
.PHONY : CMakeFiles\Studienarbeit_Display_Studienarbeit_pio_h.dir\clean

CMakeFiles\Studienarbeit_Display_Studienarbeit_pio_h.dir\depend:
	$(CMAKE_COMMAND) -E cmake_depends "NMake Makefiles" "D:\Pico\Studienarbeit - Display" "D:\Pico\Studienarbeit - Display" "D:\Pico\Studienarbeit - Display\build" "D:\Pico\Studienarbeit - Display\build" "D:\Pico\Studienarbeit - Display\build\CMakeFiles\Studienarbeit_Display_Studienarbeit_pio_h.dir\DependInfo.cmake" --color=$(COLOR)
.PHONY : CMakeFiles\Studienarbeit_Display_Studienarbeit_pio_h.dir\depend
