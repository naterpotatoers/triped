#include <libhal-lpc40/constants.hpp>
#include <libhal-lpc40/pwm.hpp>
#include <libhal-pca/pca9685.hpp>
#include <libhal-util/map.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>

#include "application.hpp"

float convert_to_duty_cycle(int angle)
{
  std::pair<float, float> from;
  std::pair<float, float> to;
  from.first = 0.0f;
  from.second = 180.0f;
  to.first = 0.025f;
  to.second = 0.125f;
  return hal::map(static_cast<float>(angle), from, to);
}

hal::status application(application_framework& p_framework)
{
  using namespace std::chrono_literals;
  using namespace hal::literals;

  auto& i2c = *p_framework.i2c;
  auto& led = *p_framework.led;
  auto& clock = *p_framework.clock;
  auto& console = *p_framework.console;

  hal::print(console, "Starting the triped\n");
  auto pca9685 = HAL_CHECK(hal::pca::pca9685::create(i2c, 0b100'0000));
  auto pwm0 = pca9685.get_pwm_channel<0>();
  auto pwm = HAL_CHECK((hal::lpc40::pwm::get(1, 1)));  // P2.0
  HAL_CHECK(pwm.frequency(50.0_Hz));
  HAL_CHECK(pwm0.frequency(50.0_Hz));

  while (true) {
    HAL_CHECK(pwm0.duty_cycle(convert_to_duty_cycle(180)));
    hal::print(console, "180\n");
    hal::delay(clock, 1s);
    HAL_CHECK(pwm0.duty_cycle(convert_to_duty_cycle(90)));
    hal::print(console, "90\n");
    hal::delay(clock, 1s);
    HAL_CHECK(pwm0.duty_cycle(convert_to_duty_cycle(0)));
    hal::print(console, "0\n");
    hal::delay(clock, 1s);
  }
  return hal::success();
}