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
		motor.open_serial_port();
	}

	Actuator motor;
};

using namespace std::literals::chrono_literals;
using namespace std::chrono;

TEST_F(BasicInteractionTests, MotorCanObtainRelinquishAndThenObtainAgainTheSameComport)
{
	EXPECT_NE(0, motor.read_wide_register_blocking(SHAFT_POS_UM).value);
	motor.close_serial_port();
	motor.open_serial_port();
	EXPECT_NE(0, motor.read_register_blocking(STATOR_TEMP).value);
}

TEST_F(BasicInteractionTests, CommandAndTestCompletesDeterministically)
{
	motor.enable_stream();
	while (!motor.stream_is_established())
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
	motor.enable_stream();
	while (!motor.stream_is_established())
	{
		motor.run();
	}
	motor.set_mode(MotorMode::PositionMode);

	motor.set_position_um(2); // This sets the stream timeout timer

	motor.run(); //Inject position command
	auto [value, error] = motor.read_register_blocking(POS_CMD);

	EXPECT_EQ(2, value);
}

TEST_F(BasicInteractionTests, HapticModeStreamingUpdatesTheHapticStatusRegisterDuringRun)
{
	motor.enable_stream();
	while (!motor.stream_is_established())
	{
		motor.run();
	}

	motor.enable_haptic_effects(0);

	uint16_t haptic_effects = Actuator::HapticEffect::ConstF + Actuator::HapticEffect::Osc0;

	motor.set_mode(MotorMode::HapticMode);
	motor.update_haptic_stream_effects(haptic_effects);
	
	motor.run(); //Sends the motor command frame, updating the haptic status register

	EXPECT_EQ(haptic_effects, motor.read_register_blocking(HAPTIC_STATUS).value);
}

TEST_F(BasicInteractionTests, MotorCommandPopulatesAllStreamValues)
{
	motor.enable_stream();
	motor.set_stream_mode(OrcaStream::StreamMode::MotorRead);
	motor.update_read_stream(2, SHAFT_PIXEL);
	while (!motor.stream_is_established())
	{
		motor.run();
	}

	auto stream_start = steady_clock::now();
	while (duration_cast<milliseconds>(steady_clock::now() - stream_start) < 10ms)
	{
		motor.run();
	}

	EXPECT_NE(motor.stream_cache.read_stream_reg, 0);
	EXPECT_NE(motor.stream_cache.mode, 0);
	EXPECT_NE(motor.stream_cache.position, 0);
	EXPECT_NE(motor.stream_cache.temperature, 0);
	EXPECT_NE(motor.stream_cache.voltage, 0);
}