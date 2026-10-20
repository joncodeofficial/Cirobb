# Minimum required version to generate this script
cmake_minimum_required(VERSION 3.8)

# Main project
project(Cirobb VERSION 1.1.7)

# Enforce C++11 usage
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF) # avoid using non-standard extensions

# Compilation database for clangd and other tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Output directories for binaries and libraries
if (EMSCRIPTEN)
	# Keep the web build out of the desktop bin/ and lib/
	set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
	set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
	set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/web)
else()
	set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
	set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
	set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
endif()

# Dependencies (Emscripten provides GLFW and WebGL itself)
if (NOT EMSCRIPTEN)
	find_package(OpenGL REQUIRED)
	add_subdirectory(vendor/glad)
	add_subdirectory(vendor/glfw)
	add_subdirectory(vendor/imgui)
endif()
add_subdirectory(cirobb)
add_subdirectory(demo)

# Default startup project for Visual Studio
if (MSVC)
	set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT demo)
endif()
