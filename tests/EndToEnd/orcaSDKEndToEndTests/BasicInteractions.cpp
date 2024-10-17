#include "pch.h"
#include "actuator.h"

class BasicInteractionTests : public ::testing::Test
{
protected:
	BasicInteractionTests() :
		motor(3)
	{}

	void SetUp()
	{
		motor.set_new_comport(3);
		motor.init();
	}

	Actuator motor;
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