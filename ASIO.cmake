include(FetchContent)
FetchContent_Declare(
	asio-repo
	GIT_REPOSITORY https://github.com/chriskohlhoff/asio.git
	GIT_TAG 03ae834edbace31a96157b89bf50e5ee464e5ef9 # release 1.32.0
	FIND_PACKAGE_ARGS
)

FetchContent_MakeAvailable(asio-repo)

file(GLOB_RECURSE asioHeaders_glob
		"${asio-repo_SOURCE_DIR}/asio/include/*.hpp"
		"${asio-repo_SOURCE_DIR}/asio/include/*.h"
		"${asio-repo_SOURCE_DIR}/asio/include/*.ipp")

target_sources(orcaSDK_core
	PUBLIC
		FILE_SET asioHeaders
		TYPE HEADERS
		BASE_DIRS "${asio-repo_SOURCE_DIR}/asio/include"
		FILES
			${asioHeaders_glob}
)