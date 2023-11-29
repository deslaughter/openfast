
include(ExternalProject)

list(APPEND CMAKE_MODULE_PATH 
	"${CMAKE_CURRENT_SOURCE_DIR}/cmake" 
	"${CMAKE_CURRENT_SOURCE_DIR}/paraconf/cmake")
include(GenerateExportHeader)

#-------------------------------------------------------------------------------
# libyaml
#-------------------------------------------------------------------------------

file(MD5 "${CMAKE_CURRENT_SOURCE_DIR}/vendor/libyaml-0.2.2.tar" HASH)
ExternalProject_Add(libyaml_ext
	URL "${CMAKE_CURRENT_SOURCE_DIR}/vendor/libyaml-0.2.2.tar"
	URL_HASH "MD5=${HASH}"
	PREFIX "${CMAKE_CURRENT_BINARY_DIR}/libyaml"
	CMAKE_CACHE_ARGS 
		"-DYAML_STATIC_LIB_NAME:STRING=yaml"
	    "-DBUILD_TESTING:BOOL=OFF" 
		"-DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_CURRENT_BINARY_DIR}/libyaml/install"
)

file (MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/libyaml/install/include)
add_library(libyaml STATIC IMPORTED)
add_dependencies(libyaml libyaml_ext)
set_target_properties(libyaml PROPERTIES 
  IMPORTED_LOCATION ${CMAKE_CURRENT_BINARY_DIR}/libyaml/install/lib/${CMAKE_STATIC_LIBRARY_PREFIX}yaml${CMAKE_STATIC_LIBRARY_SUFFIX}
  INTERFACE_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_BINARY_DIR}/libyaml/install/include
)

#-------------------------------------------------------------------------------
# paraconf libraries
#-------------------------------------------------------------------------------

add_library(paraconf
	paraconf/src/api.c
	paraconf/src/status.c
	paraconf/src/ypath.c
	paraconf/src/fortran/paraconf.f90
)
generate_export_header(paraconf)
target_link_libraries(paraconf Threads::Threads libyaml)
target_include_directories(paraconf PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/paraconf/include>
	$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
)
set_target_properties(paraconf PROPERTIES
	C_STANDARD 99
	C_STANDARD_REQUIRED TRUE
	PUBLIC_HEADER "paraconf/include/paraconf.F90;paraconf/include/paraconff.h;paraconf/include/paraconf_f90_types.h;paraconf/include/paraconf_f90.h;${CMAKE_CURRENT_BINARY_DIR}/paraconf/paraconf_export.h"
)

install(TARGETS paraconf
	EXPORT "${CMAKE_PROJECT_NAME}Libraries"
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	INCLUDES DESTINATION bin
	PUBLIC_HEADER DESTINATION include
)

