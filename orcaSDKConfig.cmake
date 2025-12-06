include(CMakeFindDependencyMacro)

find_dependency(asio)
find_dependency(orcaAPI)

include("${CMAKE_CURRENT_LIST_DIR}/orcaSDKTargets.cmake")