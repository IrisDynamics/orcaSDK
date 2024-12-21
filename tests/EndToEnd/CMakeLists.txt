add_executable(orcaSDKEndToEndTests 
	orcaSDKEndToEndTests/BasicInteractions.cpp
)

target_link_libraries(orcaSDKEndToEndTests PRIVATE gtest_main)
target_link_libraries(orcaSDKEndToEndTests PRIVATE orcaSDK::core)

gtest_discover_tests(orcaSDKEndToEndTests "" AUTO)