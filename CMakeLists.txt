cmake_minimum_required(VERSION 3.23)

project(orcaSDK VERSION 1.0.0)

#TODO(Aiden Oct 31, 2024): 
#	Address when we've decided how to handle platform dependencies. Add conditional 
#	compilation of platform specific code. See Professional CMake section 16.2.6 for suggestions
add_library(orcaSDK_core 
	src/actuator.cpp
	src/orca_stream.cpp
	src/standard_modbus_functions.cpp	
	src/command_and_confirm.cpp
)
add_library(orcaSDK::core ALIAS orcaSDK_core)
	
target_sources(orcaSDK_core
	PUBLIC
		FILE_SET publicAPI
		TYPE HEADERS
		FILES
			actuator.h
			
			src/actuator_config.h
			src/orca_modes.h
			src/orca_stream_config.h
			src/serial_interface.h
			src/message_priority.h
			src/command_and_confirm.h
			src/clock.h
			src/error_types.h
			tools/log.h
			tools/log_interface.h
			tools/timer.h
			orca600_api/orca600.h
			orca600_api/orca600_memory_map.h
			orca600_api/orca600_registers.h
			
			src/chrono_clock.h
			src/diagnostics_tracker.h
			src/function_code_parameters.h
			src/mb_config.h
			src/mb_crc.h
			src/message_queue.h
			src/modbus_client.h
			src/standard_modbus_functions.h
			src/orca_stream.h
			src/transaction.h
			src/platform/windows_serial_interface.h #WINDOWS SPECIFIC
			
	# PRIVATE
		# FILE_SET implementationIncludes
		# TYPE HEADERS
		# FILES
			# src/chrono_clock.h
			# src/diagnostics_tracker.h
			# src/function_code_parameters.h
			# src/mb_config.h
			# src/mb_crc.h
			# src/message_queue.h
			# src/modbus_client.h
			# src/modbus_client
			# src/standard_modbus_functions.h
			# src/orca_stream.h
			# src/transaction.h
			# src/platform/windows_serial_interface.h #WINDOWS SPECIFIC
		
)

target_compile_features(orcaSDK_core PUBLIC cxx_std_11)

set_target_properties(orcaSDK_core
	PROPERTIES
		VERSION 1.0.0
		SOVERSION 1
	EXPORT_NAME core
)

# include(CMakePackageConfigHelpers)
# write_basic_package_version_file(
	# orcaSDKConfigVersion.cmake
	# VERSION 1.0.0
	# COMPATIBILITY SameMajorVersion
# )

include(GNUInstallDirs)
install(TARGETS orcaSDK_core
	EXPORT orcaSDKConfig
	FILE_SET publicAPI
	INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
	# PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
	# PRIVATE_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
export(TARGETS orcaSDK_core
	NAMESPACE orcaSDK::
	FILE "${CMAKE_CURRENT_BINARY_DIR}/orcaSDKConfig.cmake"
)
install(EXPORT orcaSDKConfig 
	DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/orcaSDK
	NAMESPACE orcaSDK::
)
	# RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
		# COMPONENT	orcaSDK_Runtime
	# LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
		# COMPONENT			orcaSDK_Runtime
		# NAMELINK_COMPONENT	orcaSDK_Development
	# ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
		# COMPONENT	orcaSDK_Development
# )

# #Packaging
# set(CPACK_PACKAGE_NAME orcaSDK)
# set(CPACK_PACKAGE_VENDOR IrisDynamicsLtd)
# set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "A C++ library for interacting with your Orca Series Linear Motor.")
# set(CPACK_PACKAGE_INSTALL_DIRECTORY ${CPACK_PACKAGE_NAME})
# set(CPACK_VERBATIM_VARIABLES TRUE)

# include(CPack)

# add_subdirectory(tests)