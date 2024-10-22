#include "pch.h"
#include "actuator.h"

class BasicInteractionTests : public ::testing::Test
{
protected:
	BasicInteractionTests() :
		motor(4),
		motor1(7),
		motor2(9)
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

TEST_F(BasicInteractionTests, EnablingMotorStream) {
	motor.enable();
	while (1)
	{
		motor.run();
	}
}