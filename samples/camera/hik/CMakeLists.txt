if(NOT BUILD_rmvl_hik_camera)
  return()
endif()

rmvl_add_exe(
  rmvl_hik_mono
  SOURCES sample_hik_mono.cpp
  DEPENDS hik_camera
  EXTERNAL opencv_highgui opencv_calib3d
)

rmvl_add_exe(
  rmvl_hik_multi
  SOURCES sample_hik_multi.cpp
  DEPENDS hik_camera
  EXTERNAL opencv_highgui opencv_calib3d
)

rmvl_add_exe(
  rmvl_hik_writer
  SOURCES sample_hik_writer.cpp
  DEPENDS hik_camera
  EXTERNAL opencv_highgui opencv_calib3d
)

rmvl_add_exe(
  rmvl_hik_manual_calib
  SOURCES sample_hik_manual_calib.cpp
  DEPENDS hik_camera
  EXTERNAL opencv_highgui opencv_calib3d
)
