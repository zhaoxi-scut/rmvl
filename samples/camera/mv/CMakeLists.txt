if(NOT BUILD_rmvl_mv_camera)
  return()
endif()

rmvl_add_exe(
  rmvl_mv_mono
  SOURCES sample_mv_mono.cpp
  DEPENDS mv_camera
)

find_package(MvSDK REQUIRED)

rmvl_add_exe(
  rmvl_mv_multi
  SOURCES sample_mv_multi.cpp
  DEPENDS mv_camera
  EXTERNAL mvsdk
)

rmvl_add_exe(
  rmvl_mv_writer
  SOURCES sample_mv_writer.cpp
  DEPENDS mv_camera
)

rmvl_add_exe(
  rmvl_mv_auto_calib
  SOURCES sample_mv_auto_calib.cpp
  DEPENDS mv_camera
)

rmvl_add_exe(
  rmvl_mv_manual_calib
  SOURCES sample_mv_manual_calib.cpp
  DEPENDS mv_camera
)
