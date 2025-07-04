# --------------------------------------------------------------------------
#  Camera base module
# --------------------------------------------------------------------------
rmvl_update_build(camera WITH_OPENCV AND WITH_EIGEN3)

rmvl_generate_para(
  camera
  MODULE camera
)

rmvl_add_module(
  camera
  DEPENDS algorithm
)

# --------------------------------------------------------------------------
#  Search the specific SDK
# --------------------------------------------------------------------------
# mv
find_package(MvSDK QUIET)
rmvl_update_build(mv_camera MvSDK_FOUND AND BUILD_rmvl_camera)
# hik
find_package(HikSDK QUIET)
rmvl_update_build(hik_camera HikSDK_FOUND AND BUILD_rmvl_camera)
# opt
find_package(OPTCameraSDK QUIET)
rmvl_update_build(opt_camera OPTCameraSDK_FOUND AND BUILD_rmvl_camera)
# galaxy
find_package(GalaxySDK QUIET)
rmvl_update_build(galaxy_camera GalaxySDK_FOUND AND BUILD_rmvl_camera)

# --------------------------------------------------------------------------
#  Build camera sub-library
# --------------------------------------------------------------------------
# mv
rmvl_generate_para(
  mv_camera
  MODULE camera
)
rmvl_add_module(
  mv_camera
  DEPENDS camera
  EXTERNAL mvsdk
)

# hik
rmvl_generate_para(
  hik_camera
  MODULE camera
)
rmvl_add_module(
  hik_camera
  DEPENDS camera
  EXTERNAL hiksdk
)

# opt
rmvl_generate_para(
  opt_camera
  MODULE camera
)
rmvl_add_module(
  opt_camera
  DEPENDS camera
  EXTERNAL optcamsdk
)

# galaxy
rmvl_generate_para(
  galaxy_camera
  MODULE camera
)
rmvl_add_module(
  galaxy_camera
  DEPENDS camera
  EXTERNAL galaxysdk
)

rmvl_generate_module_para(camera)

# --------------------------------------------------------------------------
#  Install the necessary library for OS
# --------------------------------------------------------------------------
foreach(sdk_name MvSDK HikSDK OPTCameraSDK GalaxySDK)
  if(${sdk_name}_FOUND AND DEFINED ${sdk_name}_LIB)
    rmvl_install_sdk(${sdk_name})
  endif()
endforeach()

# --------------------------------------------------------------------------
#  Build the test program
# --------------------------------------------------------------------------
if(BUILD_TESTS)

endif(BUILD_TESTS)

# --------------------------------------------------------------------------
#  Build Python bindings
# --------------------------------------------------------------------------
if(BUILD_PYTHON)
  if(BUILD_rmvl_camera)
    list(APPEND cam_inc rmvl/camera/camutils.hpp rmvlpara/camera/camera.h) 
    list(APPEND cam_dep camera)
  endif()
  foreach(m mv hik opt galaxy)
    if(BUILD_rmvl_${m}_camera)
      list(APPEND cam_inc rmvl/camera/${m}_camera.h rmvlpara/camera/${m}_camera.h) 
      list(APPEND cam_dep ${m}_camera)
    endif()
  endforeach()
  
  rmvl_generate_python(camera
    FILES ${cam_inc}
    DEPENDS ${cam_dep}
    USE_PREBIND
  )
endif()