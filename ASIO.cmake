if(orcaSDK_VENDOR_ASIO)
	project(asio)

	include(GNUInstallDirs)

	include(FetchContent)
	FetchContent_Declare(
		asio
		GIT_REPOSITORY https://github.com/chriskohlhoff/asio.git
		GIT_TAG 03ae834edbace31a96157b89bf50e5ee464e5ef9 # release 1.32.0
	)

	FetchContent_MakeAvailable(asio)

	file(GLOB_RECURSE asioHeaders_glob
			"${asio_SOURCE_DIR}/asio/include/*.hpp"
			"${asio_SOURCE_DIR}/asio/include/*.h"
			"${asio_SOURCE_DIR}/asio/include/*.ipp")

	add_library(asio_headers INTERFACE)
	add_library(asio::asio ALIAS asio_headers)

	target_sources(asio_headers
		PUBLIC
			FILE_SET asioHeaders
			TYPE HEADERS
			BASE_DIRS "${asio_SOURCE_DIR}/asio/include"
			FILES
				${asioHeaders_glob}
	)

	set_target_properties(asio_headers
		PROPERTIES
			EXPORT_NAME asio
	)
	
	install(TARGETS asio_headers
		EXPORT asioConfig
		FILE_SET asioHeaders
	)
	
	install(EXPORT asioConfig
		DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/asio
		NAMESPACE asio::
	)

else()
	find_package(asio)
endif()
target_link_libraries(orcaSDK_core PUBLIC asio::asio)