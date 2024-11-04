add_executable(orcaSDKEndToEndTests 
	EXCLUDE_FROM_ALL
	orcaSDKEndToEndTests/BasicInteractions.cpp
)

target_link_libraries(orcaSDKEndToEndTests PRIVATE gtest_main)
target_link_libraries(orcaSDKEndToEndTests PRIVATE orcaSDK)

gtest_discover_tests(orcaSDKEndToEndTests "" AUTO)