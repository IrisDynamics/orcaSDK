add_executable(orcaSDKUnitTests 
	EXCLUDE_FROM_ALL
	IrisSDKTests/pch.cpp
	IrisSDKTests/modbus/ActuatorTests.cpp
	IrisSDKTests/modbus/IrisStreamHandshakeTests.cpp
	IrisSDKTests/modbus/MessageQueueTests.cpp
	IrisSDKTests/modbus/ModbusClientTests.cpp
	IrisSDKTests/modbus/TransactionConstructionTests.cpp
)

target_include_directories(orcaSDKUnitTests 
	PRIVATE 
		IrisSDKTests)

target_link_libraries(orcaSDKUnitTests PRIVATE gtest_main)
target_link_libraries(orcaSDKUnitTests PRIVATE orcaSDK)

gtest_discover_tests(orcaSDKUnitTests "" AUTO)