# SPDX-License-Identifier: Apache-2.0
idf_component_register(
    SRCS "src/bq27220.c"
    INCLUDE_DIRS "include"
    REQUIRES driver
)

include(package_manager)
cu_pkg_define_version(${CMAKE_CURRENT_LIST_DIR})    
