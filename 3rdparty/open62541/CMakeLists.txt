# ----------------------------------------------------------------------------
#   open62541
# ----------------------------------------------------------------------------

set(OPEN62541_PKG open62541)

project(
  ${OPEN62541_PKG}
  LANGUAGES C
)

option(UA_ENABLE_PUBSUB "Enable the PubSub protocol" ON)
rmvl_download(${OPEN62541_PKG} GIT "https://github.com/open62541/open62541.git : v1.3.8")
set(open62541_VERSION "1.3.8" CACHE INTERNAL "open62541 version")

if(NOT BUILD_SHARED_LIBS)
  install(
    TARGETS ${OPEN62541_PKG}
    EXPORT RMVLModules
    ARCHIVE DESTINATION ${RMVL_3P_LIB_INSTALL_PATH}
    OPTIONAL
  )
endif()
