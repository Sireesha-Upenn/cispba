cmake_minimum_required(VERSION 2.8.2)

project(partio-download NONE)

include(ExternalProject)
ExternalProject_Add(partio-download
                    GIT_REPOSITORY;https://github.com/wdas/partio.git;GIT_TAG;v1.1.0;UPDATE_DISCONNECTED;1;PATCH_COMMAND;C:/Program Files/Git/cmd/git.exe;apply;C:/Users/psire/Documents/UPenn/Fall 2020/CIS 563/cispba/Deps/partio.patch
                    SOURCE_DIR          "C:/Users/psire/Documents/UPenn/Fall 2020/CIS 563/cispba/build/partio-src"
                    BINARY_DIR          "C:/Users/psire/Documents/UPenn/Fall 2020/CIS 563/cispba/build/partio-build"
                    CONFIGURE_COMMAND   ""
                    BUILD_COMMAND       ""
                    INSTALL_COMMAND     ""
                    TEST_COMMAND        ""
)
