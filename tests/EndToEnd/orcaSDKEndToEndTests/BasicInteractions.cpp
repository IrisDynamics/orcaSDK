#include "pch.h"
#include "actuator.h"
#include "src/command_and_confirm.h"
#include <chrono>
using namespace orcaSDK;

class BasicInteractionTests : public ::testing::Test
{
protected:
	BasicInteractionTests() :
		motor("unimportant")
	{}

	void SetUp()
	{
		OrcaError error = motor.open_serial_port(serial_port_number, baud_rate, 0);
		if (error) std::cout << "Error opening serial port: " << error.what();
	}

	unsigned int baud_rate = 1000000;
	size_t serial_port_number = 7;
	Actuator motor;
};

using namespace std::literals::chrono_literals;
using namespace std::chrono;

//TEST_F(BasicInteractionTests, InfiniteLoopTest)
//{
//	while(true)
//	{
//		std::cout << motor.get_position_um().value << "           \r";
//	}
//}
TEST_F(BasicInteractionTests, ReadingWithoutOpeningDoesntThrowException)
{
	motor.close_serial_port();
	motor.get_position_um();
}

TEST_F(BasicInteractionTests, MotorCanObtainRelinquishAndThenObtainAgainTheSameComport)
{
	EXPECT_NE(0, motor.read_wide_register_blocking(SHAFT_POS_UM).value);
	motor.close_serial_port();
	OrcaError error = motor.open_serial_port(serial_port_number, baud_rate, 0);
	if (error) std::cout << "Error opening serial port: " << error.what();
	EXPECT_NE(0, motor.read_register_blocking(BOARD_TEMP).value);
}

TEST_F(BasicInteractionTests, CommandAndTestCompletesDeterministically)
{
	motor.enable_stream();

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

	motor.set_mode(MotorMode::ForceMode);

	motor.set_streamed_force_mN(2); // This sets the stream timeout timer

	motor.run(); //Inject position command
	OrcaResult<uint16_t> result = motor.read_register_blocking(FORCE_CMD);

	EXPECT_EQ(2, result.value);
}

TEST_F(BasicInteractionTests, HapticModeStreamingUpdatesTheHapticStatusRegisterDuringRun)
{
	motor.enable_stream();

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

	auto stream_start = steady_clock::now();
	while (duration_cast<milliseconds>(steady_clock::now() - stream_start) < 50ms)
	{
		motor.run();
	}

	EXPECT_NE(motor.stream_cache.position, 0);
	EXPECT_NE(motor.stream_cache.temperature, 0);
	EXPECT_NE(motor.stream_cache.voltage, 0);
}

TEST_F(BasicInteractionTests, DestroyingAnActuatorWithoutOpeningASerialPortDoesntThrow)
{
	EXPECT_NO_THROW(
		{
			Actuator unopened_motor{"hello"};
		}
	);
}