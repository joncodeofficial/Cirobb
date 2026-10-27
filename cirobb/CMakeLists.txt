# Variable INCLUDE_FILES
set(CIROBB_INCLUDE_FILES

  cbMath.h
  collision.h 
  rigidBody.h 
  manifold.h
  scene.h
  shape.h
)


# Variable SOURCE_FILES
set(CIROBB_SOURCE_FILES

  collision.cpp 
  rigidBody.cpp
  manifold.cpp
  scene.cpp
  shape.cpp
)


add_library(cirobb STATIC

  ${CIROBB_INCLUDE_FILES} 
  ${CIROBB_SOURCE_FILES}
)


# Esto es usado para crear carpetas en visual studio.
source_group(include FILES ${CIROBB_INCLUDE_FILES})
source_group(src FILES ${CIROBB_SOURCE_FILES})

