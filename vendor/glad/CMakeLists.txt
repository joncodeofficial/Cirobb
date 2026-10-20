set(GLAD_HEADER_FILES
  glad.h
  khrplatform.h
)

set(GLAD_SOURCE_FILES
  glad.c
)

add_library(glad STATIC 
  ${GLAD_HEADER_FILES}
  ${GLAD_SOURCE_FILES} 
)

target_include_directories(glad PUBLIC ..)

# Esto es usado para crear carpetas en visual studio.
source_group(include FILES ${GLAD_INCLUDE_FILES})
source_group(src FILES ${GLAD_SOURCE_FILES})