#include "pch.h"
#include "actuator.h"

class BasicInteractionTests : public ::testing::Test
{
protected:
	BasicInteractionTests() :
		motor(4, "unimportant")
	{}

	void SetUp()
	{
		motor.init();
	}

	Actuator motor;
};

//TEST_F(BasicInteractionTests, ReadsToRegisterPositionGoThroughAsExpected) {
//	EXPECT_EQ(0, motor.get_position_um());
//	motor.read_registers(SHAFT_POS_UM, 2);
//	motor.flush();
//	EXPECT_NE(0, motor.get_position_um());
//}
//
////TODO[Aiden, Oct 23 2024]: This behaviour can be checked as a unit test and should be
//TEST_F(BasicInteractionTests, AfterInitRegistersAreSetToZero) {
//	motor.read_registers(SHAFT_POS_UM, 2);
//	motor.flush();
//	EXPECT_NE(0, motor.get_position_um());
//	motor.disable_comport();
//	motor.init();
//	EXPECT_EQ(0, motor.get_position_um());
//}
//
//TEST_F(BasicInteractionTests, MotorCanObtainRelinquishAndThenObtainAgainTheSameComport)
//{
//	motor.read_registers(SHAFT_POS_UM, 2);
//	motor.flush();
//	EXPECT_NE(0, motor.get_position_um());
//	motor.disable_comport();
//	motor.init();
//	motor.read_register(STATOR_TEMP);
//	motor.flush();
//	EXPECT_NE(0, motor.get_orca_reg_content(STATOR_TEMP));
//}

TEST_F(BasicInteractionTests, MotorGoesToSleepIfEnoughTimePassesBetweenForceStreamCommands)
{
	ConnectionConfig config;
	config.response_timeout_us = 100000;
	motor.set_connection_config(config);
	motor.enable();
	while (!motor.is_connected())
	{
		motor.run();
	}
	motor.set_mode(MotorMode::ForceMode);

	EXPECT_EQ(MotorMode::ForceMode, motor.get_mode());

	auto stream_start = std::chrono::steady_clock::now();
	while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()- stream_start) < std::chrono::milliseconds(105))
	{
		motor.run();
	}

	motor.read_register(MODE_OF_OPERATION);
	motor.flush();

	EXPECT_EQ(MotorMode::SleepMode, motor.get_mode());
}

//TEST_F(BasicInteractionTests, EnablingMotorStream) {
//	motor.enable();
//	while (1)
//	{
//		motor.run();
//	}
//}

TEST_F(BasicInteractionTests, CommandAndTestCompletesDeterministically)
{
	for (int i = 0; i < 25; i++)
	{
		EXPECT_TRUE(motor.command_and_confirm(CTRL_REG_3, MotorMode::ForceMode, MODE_OF_OPERATION, 
			[this]()->bool{ return (motor.get_orca_reg_content(MODE_OF_OPERATION) == MotorMode::ForceMode); }));
		EXPECT_TRUE(motor.command_and_confirm(CTRL_REG_3, MotorMode::SleepMode, MODE_OF_OPERATION, MotorMode::SleepMode));
	}
}
