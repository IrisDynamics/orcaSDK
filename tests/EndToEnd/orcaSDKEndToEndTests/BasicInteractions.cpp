#include "pch.h"
#include "actuator.h"
#include "src/command_and_confirm.h"
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

TEST_F(BasicInteractionTests, MotorCanObtainRelinquishAndThenObtainAgainTheSameComport)
{
	motor.read_multiple_registers_blocking(SHAFT_POS_UM, 2);
	motor.flush();
	EXPECT_NE(0, motor.get_position_um());
	motor.disable_comport();
	motor.init();
	motor.read_register_blocking(STATOR_TEMP);
	motor.flush();
	EXPECT_NE(0, motor.get_orca_reg_content(STATOR_TEMP));
}

TEST_F(BasicInteractionTests, CommandAndTestCompletesDeterministically)
{
	motor.enable();
	while (!motor.is_connected())
	{
		motor.run();
	}
	for (int i = 0; i < 20; i++)
	{
		EXPECT_TRUE(command_and_confirm(motor, CTRL_REG_3, MotorMode::ForceMode, MODE_OF_OPERATION, 
			[this](uint16_t read_value)->bool{ return (read_value == MotorMode::ForceMode); }));
		EXPECT_TRUE(command_and_confirm(motor, CTRL_REG_3, MotorMode::SleepMode, MODE_OF_OPERATION, MotorMode::SleepMode));
	}
}

TEST_F(BasicInteractionTests, WhenEnabledAndConnectedActuatorObjectAutomaticallyEnqueuesStreamCommands)
{
	motor.enable();
	while (!motor.is_connected())
	{
		motor.run();
	}
	motor.set_mode(MotorMode::PositionMode);

	motor.set_position_um(2); // This sets the stream timeout timer

	motor.run(); //Inject position command
	motor.read_register_blocking(POS_CMD);

	motor.flush();

	EXPECT_EQ(2, motor.get_orca_reg_content(POS_CMD));
}

TEST_F(BasicInteractionTests, WhenStreamPauseIsCalledAutomaticStreamMessagesDoNotGetQueued)
{
	motor.enable();
	while (!motor.is_connected())
	{
		motor.run();
	}
	motor.set_mode(MotorMode::PositionMode);

	motor.write_register_blocking(POS_CMD, 0);
	motor.set_stream_paused(true); // Disable the queuing of new stream messages

	motor.set_position_um(2); // This sets the stream timeout timer

	motor.run(); // This sends the change mode command
	
	EXPECT_EQ(0, motor.read_register_blocking(POS_CMD));
}

TEST_F(BasicInteractionTests, HapticModeStreamingUpdatesTheHapticStatusRegisterDuringRun)
{
	motor.enable();
	while (!motor.is_connected())
	{
		motor.run();
	}

	motor.enable_haptic_effects(0);

	uint16_t haptic_effects = Actuator::HapticEffect::ConstF + Actuator::HapticEffect::Osc0;

	motor.set_mode(MotorMode::HapticMode);
	motor.update_haptic_stream_effects(haptic_effects);
	
	motor.run(); //Sends the motor command frame, updating the haptic status register

	EXPECT_EQ(haptic_effects, motor.read_register_blocking(HAPTIC_STATUS));
}