# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/sireesha/cis563/cispba/Build/Release/partio-download

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/sireesha/cis563/cispba/Build/Release/partio-download

# Utility rule file for partio-download.

# Include the progress variables for this target.
include CMakeFiles/partio-download.dir/progress.make

CMakeFiles/partio-download: CMakeFiles/partio-download-complete


CMakeFiles/partio-download-complete: partio-download-prefix/src/partio-download-stamp/partio-download-install
CMakeFiles/partio-download-complete: partio-download-prefix/src/partio-download-stamp/partio-download-mkdir
CMakeFiles/partio-download-complete: partio-download-prefix/src/partio-download-stamp/partio-download-download
CMakeFiles/partio-download-complete: partio-download-prefix/src/partio-download-stamp/partio-download-patch
CMakeFiles/partio-download-complete: partio-download-prefix/src/partio-download-stamp/partio-download-configure
CMakeFiles/partio-download-complete: partio-download-prefix/src/partio-download-stamp/partio-download-build
CMakeFiles/partio-download-complete: partio-download-prefix/src/partio-download-stamp/partio-download-install
CMakeFiles/partio-download-complete: partio-download-prefix/src/partio-download-stamp/partio-download-test
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/sireesha/cis563/cispba/Build/Release/partio-download/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Completed 'partio-download'"
	/usr/bin/cmake -E make_directory /home/sireesha/cis563/cispba/Build/Release/partio-download/CMakeFiles
	/usr/bin/cmake -E touch /home/sireesha/cis563/cispba/Build/Release/partio-download/CMakeFiles/partio-download-complete
	/usr/bin/cmake -E touch /home/sireesha/cis563/cispba/Build/Release/partio-download/partio-download-prefix/src/partio-download-stamp/partio-download-done

partio-download-prefix/src/partio-download-stamp/partio-download-install: partio-download-prefix/src/partio-download-stamp/partio-download-build
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/sireesha/cis563/cispba/Build/Release/partio-download/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "No install step for 'partio-download'"
	cd /home/sireesha/cis563/cispba/Build/Release/partio-build && /usr/bin/cmake -E echo_append
	cd /home/sireesha/cis563/cispba/Build/Release/partio-build && /usr/bin/cmake -E touch /home/sireesha/cis563/cispba/Build/Release/partio-download/partio-download-prefix/src/partio-download-stamp/partio-download-install

partio-download-prefix/src/partio-download-stamp/partio-download-mkdir:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/sireesha/cis563/cispba/Build/Release/partio-download/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Creating directories for 'partio-download'"
	/usr/bin/cmake -E make_directory /home/sireesha/cis563/cispba/Build/Release/partio-src
	/usr/bin/cmake -E make_directory /home/sireesha/cis563/cispba/Build/Release/partio-build
	/usr/bin/cmake -E make_directory /home/sireesha/cis563/cispba/Build/Release/partio-download/partio-download-prefix
	/usr/bin/cmake -E make_directory /home/sireesha/cis563/cispba/Build/Release/partio-download/partio-download-prefix/tmp
	/usr/bin/cmake -E make_directory /home/sireesha/cis563/cispba/Build/Release/partio-download/partio-download-prefix/src/partio-download-stamp
	/usr/bin/cmake -E make_directory /home/sireesha/cis563/cispba/Build/Release/partio-download/partio-download-prefix/src
	/usr/bin/cmake -E make_directory /home/sireesha/cis563/cispba/Build/Release/partio-download/partio-download-prefix/src/partio-download-stamp
	/usr/bin/cmake -E touch /home/sireesha/cis563/cispba/Build/Release/partio-download/partio-download-prefix/src/partio-download-stamp/partio-download-mkdir

partio-download-prefix/src/partio-download-stamp/partio-download-download: partio-download-prefix/src/partio-download-stamp/partio-download-gitinfo.txt
partio-download-prefix/src/partio-download-stamp/partio-download-download: partio-download-prefix/src/partio-download-stamp/partio-download-mkdir
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/sireesha/cis563/cispba/Build/Release/partio-download/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Performing download step (git clone) for 'partio-download'"
	cd /home/sireesha/cis563/cispba/Build/Release && /usr/bin/cmake -P /home/sireesha/cis563/cispba/Build/Release/partio-download/partio-download-prefix/tmp/partio-download-gitclone.cmake
	cd /home/sireesha/cis563/cispba/Build/Release && /usr/bin/cmake -E touch /home/sireesha/cis563/cispba/Build/Release/partio-download/partio-download-prefix/src/partio-download-stamp/partio-download-download

partio-download-prefix/src/partio-download-stamp/partio-download-patch: partio-download-prefix/src/partio-download-stamp/partio-download-download
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/sireesha/cis563/cispba/Build/Release/partio-download/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Performing patch step for 'partio-download'"
	cd /home/sireesha/cis563/cispba/Build/Release/partio-src && /usr/bin/git apply /home/sireesha/cis563/cispba/Deps/partio.patch
	cd /home/sireesha/cis563/cispba/Build/Release/partio-src && /usr/bin/cmake -E touch /home/sireesha/cis563/cispba/Build/Release/partio-download/partio-download-prefix/src/partio-download-stamp/partio-download-patch

partio-download-prefix/src/partio-download-stamp/partio-download-configure: partio-download-prefix/tmp/partio-download-cfgcmd.txt
partio-download-prefix/src/partio-download-stamp/partio-download-configure: partio-download-prefix/src/partio-download-stamp/partio-download-skip-update
partio-download-prefix/src/partio-download-stamp/partio-download-configure: partio-download-prefix/src/partio-download-stamp/partio-download-patch
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/sireesha/cis563/cispba/Build/Release/partio-download/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "No configure step for 'partio-download'"
	cd /home/sireesha/cis563/cispba/Build/Release/partio-build && /usr/bin/cmake -E echo_append
	cd /home/sireesha/cis563/cispba/Build/Release/partio-build && /usr/bin/cmake -E touch /home/sireesha/cis563/cispba/Build/Release/partio-download/partio-download-prefix/src/partio-download-stamp/partio-download-configure

partio-download-prefix/src/partio-download-stamp/partio-download-build: partio-download-prefix/src/partio-download-stamp/partio-download-configure
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/sireesha/cis563/cispba/Build/Release/partio-download/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "No build step for 'partio-download'"
	cd /home/sireesha/cis563/cispba/Build/Release/partio-build && /usr/bin/cmake -E echo_append
	cd /home/sireesha/cis563/cispba/Build/Release/partio-build && /usr/bin/cmake -E touch /home/sireesha/cis563/cispba/Build/Release/partio-download/partio-download-prefix/src/partio-download-stamp/partio-download-build

partio-download-prefix/src/partio-download-stamp/partio-download-test: partio-download-prefix/src/partio-download-stamp/partio-download-install
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/sireesha/cis563/cispba/Build/Release/partio-download/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "No test step for 'partio-download'"
	cd /home/sireesha/cis563/cispba/Build/Release/partio-build && /usr/bin/cmake -E echo_append
	cd /home/sireesha/cis563/cispba/Build/Release/partio-build && /usr/bin/cmake -E touch /home/sireesha/cis563/cispba/Build/Release/partio-download/partio-download-prefix/src/partio-download-stamp/partio-download-test

partio-download-prefix/src/partio-download-stamp/partio-download-skip-update: partio-download-prefix/src/partio-download-stamp/partio-download-download
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/sireesha/cis563/cispba/Build/Release/partio-download/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Skipping update step for 'partio-download'"
	cd /home/sireesha/cis563/cispba/Build/Release/partio-src && /usr/bin/cmake -E echo_append

partio-download: CMakeFiles/partio-download
partio-download: CMakeFiles/partio-download-complete
partio-download: partio-download-prefix/src/partio-download-stamp/partio-download-install
partio-download: partio-download-prefix/src/partio-download-stamp/partio-download-mkdir
partio-download: partio-download-prefix/src/partio-download-stamp/partio-download-download
partio-download: partio-download-prefix/src/partio-download-stamp/partio-download-patch
partio-download: partio-download-prefix/src/partio-download-stamp/partio-download-configure
partio-download: partio-download-prefix/src/partio-download-stamp/partio-download-build
partio-download: partio-download-prefix/src/partio-download-stamp/partio-download-test
partio-download: partio-download-prefix/src/partio-download-stamp/partio-download-skip-update
partio-download: CMakeFiles/partio-download.dir/build.make

.PHONY : partio-download

# Rule to build all files generated by this target.
CMakeFiles/partio-download.dir/build: partio-download

.PHONY : CMakeFiles/partio-download.dir/build

CMakeFiles/partio-download.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/partio-download.dir/cmake_clean.cmake
.PHONY : CMakeFiles/partio-download.dir/clean

CMakeFiles/partio-download.dir/depend:
	cd /home/sireesha/cis563/cispba/Build/Release/partio-download && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/sireesha/cis563/cispba/Build/Release/partio-download /home/sireesha/cis563/cispba/Build/Release/partio-download /home/sireesha/cis563/cispba/Build/Release/partio-download /home/sireesha/cis563/cispba/Build/Release/partio-download /home/sireesha/cis563/cispba/Build/Release/partio-download/CMakeFiles/partio-download.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/partio-download.dir/depend

