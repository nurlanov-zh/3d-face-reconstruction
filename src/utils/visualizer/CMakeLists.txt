include (ACGCommon)

include_directories (
  ../../../thirdparty/build-openmesh/install
  ${CMAKE_CURRENT_SOURCE_DIR}
)

if (NOT QT5_FOUND)
  acg_qt5 ()
endif()

set (directories 
  .
)

set (sources
  MeshViewerWidget.cc
  QGLViewerWidget.cc
  main.cc
)

set (headers
  MeshViewerWidget.hh
  QGLViewerWidget.hh  
)

acg_add_executable (face_viewer ${sources} ${headers})

target_link_libraries (face_viewer
  common
  data_reader
  OpenMeshCore
  OpenMeshTools
  Qt5::OpenGL
  spdlog::spdlog
  ${OPENGL_LIBRARIES}
)

