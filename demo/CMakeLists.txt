# Variable INCLUDE_FILES
set(DEMO_INCLUDE_FILES

  render.h
)


# Variable SOURCE_FILES
set(DEMO_SOURCE_FILES

  render.cpp
  main.cpp
)


# Desktop uses ImGui for the controls; the web build uses the HTML page in web/
if (NOT EMSCRIPTEN)
  list(APPEND DEMO_INCLUDE_FILES
    ${CMAKE_SOURCE_DIR}/vendor/imgui/backends/imgui_impl_glfw.h
    ${CMAKE_SOURCE_DIR}/vendor/imgui/backends/imgui_impl_opengl2.h
  )
  list(APPEND DEMO_SOURCE_FILES
    ${CMAKE_SOURCE_DIR}/vendor/imgui/backends/imgui_impl_glfw.cpp
    ${CMAKE_SOURCE_DIR}/vendor/imgui/backends/imgui_impl_opengl2.cpp
  )
endif()


# Esto es usado para crear carpetas de visual studio.
source_group(include FILES ${DEMO_INCLUDE_FILES})
source_group(src FILES ${DEMO_SOURCE_FILES})


# Proyecto ejecutable con el archivo fuente VAR INCLUDE_FILES Y VAR SOURCE_FILES
add_executable(demo ${DEMO_INCLUDE_FILES} ${DEMO_SOURCE_FILES})


if (EMSCRIPTEN)
  # Produces web/index.js + web/index.wasm, loaded by web/index.html
  set_target_properties(demo PROPERTIES OUTPUT_NAME index)

  target_link_options(demo PRIVATE
    -sUSE_GLFW=3
    -sUSE_WEBGL2=1
    -sLEGACY_GL_EMULATION=1
    -sEXPORTED_FUNCTIONS=_main,_setPause,_setCorrectionType,_setTouchDevice
    -sEXPORTED_RUNTIME_METHODS=ccall,cwrap
  )

  target_link_libraries(demo cirobb)

  # Copy the web page next to the generated files
  add_custom_command(TARGET demo POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
      ${CMAKE_CURRENT_SOURCE_DIR}/web/index.html
      $<TARGET_FILE_DIR:demo>/index.html
  )
else()
  target_include_directories(demo PRIVATE ${CMAKE_SOURCE_DIR}/vendor/imgui/backends)

  target_link_libraries(

    demo
    glad
    glfw
    imgui
    cirobb
    ${OPENGL_LIBRARIES}
  )
endif()
