add_executable(orcaSDKUnitTests 
	IrisSDKTests/pch.cpp
	IrisSDKTests/modbus/ActuatorTests.cpp
	IrisSDKTests/modbus/MessageQueueTests.cpp
	IrisSDKTests/modbus/ModbusClientTests.cpp
	IrisSDKTests/modbus/TransactionConstructionTests.cpp
	IrisSDKTests/modbus/ActuatorIntegrationTests.cpp
)

target_include_directories(orcaSDKUnitTests 
	PRIVATE 
		IrisSDKTests)

target_link_libraries(orcaSDKUnitTests PRIVATE gtest_main gmock_main)
target_link_libraries(orcaSDKUnitTests PRIVATE orcaSDK::core)

gtest_discover_tests(orcaSDKUnitTests "" AUTO)