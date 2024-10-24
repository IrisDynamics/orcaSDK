#include "pch.h"
#include "actuator.h"

class BasicInteractionTests : public ::testing::Test
{
protected:
	BasicInteractionTests() :
		motor(4, "unimportant"),
		motor1(7, "unimportant"),
		motor2(9, "unimportant")
	{}

	void SetUp()
	{
		motor.init();
		//motor1.init();
		//motor2.init();
	}

	Actuator motor;
	Actuator motor1;
	Actuator motor2;
};

TEST_F(BasicInteractionTests, ReadsToRegisterPositionGoThroughAsExpected) {
	EXPECT_EQ(0, motor.get_position_um());
	motor.read_registers(SHAFT_POS_UM, 2);
	motor.flush();
	EXPECT_NE(0, motor.get_position_um());
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
		EXPECT_TRUE(motor.command_and_confirm(CTRL_REG_3, Actuator::ForceMode, MODE_OF_OPERATION, 
			[this]()->bool{ return (motor.get_orca_reg_content(MODE_OF_OPERATION) == Actuator::ForceMode); }));
		EXPECT_TRUE(motor.command_and_confirm(CTRL_REG_3, Actuator::SleepMode, MODE_OF_OPERATION,
			[this]()->bool { return (motor.get_orca_reg_content(MODE_OF_OPERATION) == Actuator::SleepMode); }));
	}
}