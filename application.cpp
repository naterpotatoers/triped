#include <libhal-lpc40/constants.hpp>
#include <libhal-lpc40/pwm.hpp>
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

  auto& led = *p_framework.led;
  auto& clock = *p_framework.clock;
  auto& console = *p_framework.console;

  hal::print(console, "Starting the triped\n");
  auto pwm = HAL_CHECK((hal::lpc40::pwm::get(1, 1))); // P2.0
  HAL_CHECK(pwm.frequency(50.0_Hz));

  while (true) {
    HAL_CHECK(pwm.duty_cycle(convert_to_duty_cycle(180)));
    hal::print(console, "180\n");
    hal::delay(clock, 1s);
    HAL_CHECK(pwm.duty_cycle(convert_to_duty_cycle(90)));
    hal::print(console, "90\n");
    hal::delay(clock, 1s);
    HAL_CHECK(pwm.duty_cycle(convert_to_duty_cycle(0)));
    hal::print(console, "0\n");
    hal::delay(clock, 1s);
  }
}