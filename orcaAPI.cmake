if(orcaSDK_VENDOR_ORCAAPI)
	include(FetchContent)
	FetchContent_Declare(
		orcaAPI
		GIT_REPOSITORY https://github.com/IrisDynamics/orca-api.git
		GIT_TAG main
	)

	FetchContent_MakeAvailable(orcaAPI)

else()
	find_package(orcaAPI)
endif()

target_link_libraries(orcaSDK_core PUBLIC orcaAPI::orcaAPI)