# Variable INCLUDE_FILES
set(DEMO_INCLUDE_FILES

  Render.h
  ${CMAKE_SOURCE_DIR}/vendor/imgui/backends/imgui_impl_glfw.h
  ${CMAKE_SOURCE_DIR}/vendor/imgui/backends/imgui_impl_opengl2.h
)


# Variable SOURCE_FILES
set(DEMO_SOURCE_FILES

  ${CMAKE_SOURCE_DIR}/vendor/imgui/backends/imgui_impl_glfw.cpp
  ${CMAKE_SOURCE_DIR}/vendor/imgui/backends/imgui_impl_opengl2.cpp
  Render.cpp
  Main.cpp
)


# Esto es usado para crear carpetas de visual studio.
source_group(include FILES ${INCLUDE_FILES})
source_group(src FILES ${SOURCE_FILES})


# Proyecto ejecutable con el archivo fuente VAR INCLUDE_FILES Y VAR SOURCE_FILES
add_executable(demo ${DEMO_INCLUDE_FILES} ${DEMO_SOURCE_FILES})


target_include_directories(demo PRIVATE ${CMAKE_SOURCE_DIR}/vendor/imgui/backends)


target_link_libraries(

  demo
  glad
  glfw
  imgui
  cirobb
  ${OPENGL_LIBRARIES}
)