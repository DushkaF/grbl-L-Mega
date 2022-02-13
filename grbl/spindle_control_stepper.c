/*
  spindle_control.c - spindle control methods
  Part of Grbl

  Copyright (c) 2012-2017 Sungeun K. Jeon for Gnea Research LLC
  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "grbl.h"

static float freq_gradient; // Precalulated value to speed up rpm to PWM conversions.

// Set period on CTC timer mode
uint32_t set_period(uint32_t _timer_period)
{
  uint32_t _timer_cycles = F_CPU / 1000000 * _timer_period; // Calculation of the number of timer cycles per period
  uint8_t _timer_prescaler = 0x00;
  uint16_t _timer_divider = 0x00;

  if (_timer_cycles < 65536UL)
  { // Сhoose optimal divider for the timer
    _timer_prescaler = 0x01;
    _timer_divider = 1UL;
  }
  else if (_timer_cycles < 65536UL * 8)
  {
    _timer_prescaler = 0x02;
    _timer_divider = 8UL;
  }
  else if (_timer_cycles < 65536UL * 64)
  {
    _timer_prescaler = 0x03;
    _timer_divider = 64UL;
  }
  else if (_timer_cycles < 65536UL * 256)
  {
    _timer_prescaler = 0x04;
    _timer_divider = 256UL;
  }
  else
  {
    _timer_prescaler = 0x05;
    _timer_divider = 1024UL;
  }

  uint16_t _timer_top = (_timer_cycles < 65536UL * 1024 ? (_timer_cycles / _timer_divider) : 65536UL);

  //  SPINDLE_TCCRA_REGISTER = (TCCR4A & 0xFC);
  SPINDLE_TCCRB_REGISTER = SPINDLE_TCCRB_INIT_MASK;
  SPINDLE_TCCRB_REGISTER |= _timer_prescaler; // CTC mode + set prescaler
  SPINDLE_ICR_REGISTER = _timer_top - 1;      // Set timer top

  return (1000000UL / ((F_CPU / _timer_divider) / _timer_top)); // Return real timer period
}

uint32_t set_frequency(uint32_t _timer_frequency)
{
  return 1000000UL / (set_period(1000000UL / _timer_frequency));
}

ISR(SPINDLE_TIMER)
{
  SPINDLE_CONTROL_PORT ^= (1 << SPINDLE_CONTROL_BIT);
}

void spindle_init()
{
  // Configure variable spindle PWM and enable pin, if required.
  SPINDLE_CONTROL_DDR |= (1 << SPINDLE_CONTROL_BIT); // Configure as PWM output pin.
  SPINDLE_TCCRA_REGISTER = SPINDLE_TCCRA_INIT_MASK;  // Configure PWM output compare timer
  SPINDLE_TCCRB_REGISTER = SPINDLE_TCCRB_INIT_MASK;
  SPINDLE_ENABLE_DDR |= (1 << SPINDLE_ENABLE_BIT);       // Configure as output pin.
  SPINDLE_DIRECTION_DDR |= (1 << SPINDLE_DIRECTION_BIT); // Configure as output pin.

  freq_gradient = (1.0 / 60.0) * (360.0 * SPINDLE_MICROSTEP_DIVIDER / 1.8);
  spindle_stop();
}

uint8_t spindle_get_state()
{
#ifdef INVERT_SPINDLE_ENABLE_PIN
  if (bit_isfalse(SPINDLE_ENABLE_PORT, (1 << SPINDLE_ENABLE_BIT)) && (SPINDLE_TIMSK_REGISTER & (1 << SPINDLE_OCIE_BIT)))
  {
#else
  if (bit_istrue(SPINDLE_ENABLE_PORT, (1 << SPINDLE_ENABLE_BIT)) && (SPINDLE_TIMSK_REGISTER & (1 << SPINDLE_OCIE_BIT)))
  {
#endif
    if (SPINDLE_DIRECTION_PORT & (1 << SPINDLE_DIRECTION_BIT))
    {
      return (SPINDLE_STATE_CCW);
    }
    else
    {
      return (SPINDLE_STATE_CW);
    }
  }
  return (SPINDLE_STATE_DISABLE);
}

// Disables the spindle and sets control interruptus output to zero when interruptus variable spindle speed is enabled.
// Called by various main program and ISR routines. Keep routine small, fast, and efficient.
// Called by spindle_init(), spindle_set_speed(), spindle_set_state(), and mc_reset().
void spindle_stop()
{
  SPINDLE_TIMSK_REGISTER &= ~(1 << SPINDLE_OCIE_BIT);  // turn off SPINDLE_TIMER
  SPINDLE_CONTROL_PORT &= ~(1 << SPINDLE_CONTROL_BIT); // set step pin low
  sei();
#ifdef INVERT_SPINDLE_ENABLE_PIN
  SPINDLE_ENABLE_PORT |= (1 << SPINDLE_ENABLE_BIT); // Set pin to high
#else
  SPINDLE_ENABLE_PORT &= ~(1 << SPINDLE_ENABLE_BIT); // Set pin to low
#endif
}

// Sets spindle speed PWM output and enable pin, if configured. Called by spindle_set_state()
// and stepper ISR. Keep routine small and efficient.
void spindle_set_speed(uint32_t freq_value, uint8_t state)
{
  if (state == SPINDLE_ENABLE_CW) // Set direction
  {
    SPINDLE_DIRECTION_PORT &= ~(1 << SPINDLE_DIRECTION_BIT);
  }
  else if (state == SPINDLE_ENABLE_CCW)
  {
    SPINDLE_DIRECTION_PORT |= (1 << SPINDLE_DIRECTION_BIT);
  }

  set_frequency(2 * freq_value);

#ifdef SPINDLE_ENABLE_OFF_WITH_ZERO_SPEED
  if (freq_value == SPINDLE_RPM_OFF_VALUE)
  {
    spindle_stop();
  }
  else
  {
    SPINDLE_TIMSK_REGISTER |= (1 << SPINDLE_OCIE_BIT); // Ensure PWM output is enabled.
#ifdef INVERT_SPINDLE_ENABLE_PIN
    SPINDLE_ENABLE_PORT &= ~(1 << SPINDLE_ENABLE_BIT);
#else
    SPINDLE_ENABLE_PORT |= (1 << SPINDLE_ENABLE_BIT);
#endif
  }
#else
  if (freq_value == SPINDLE_RPM_OFF_VALUE)
  {
    SPINDLE_TIMSK_REGISTER &= ~(1 << SPINDLE_OCIE_BIT);  // turn off SPINDLE_TIMER
    SPINDLE_CONTROL_PORT &= ~(1 << SPINDLE_CONTROL_BIT); // set step pin low
  }
  else
  {
    SPINDLE_TIMSK_REGISTER |= (1 << SPINDLE_OCIE_BIT); // Ensure PWM output is enabled.
#ifdef INVERT_SPINDLE_ENABLE_PIN
    SPINDLE_ENABLE_PORT &= ~(1 << SPINDLE_ENABLE_BIT);
#else
    SPINDLE_ENABLE_PORT |= (1 << SPINDLE_ENABLE_BIT);
#endif
  }
#endif
  sei();
}

#ifdef ENABLE_PIECEWISE_LINEAR_SPINDLE
// Be removed for step motor on spindle
#else
#endif

// TODO This function must makes gradient of RPM for smooth acceleration and deceleration
// Called by spindle_set_state() and step segment generator. Keep routine small and efficient.
uint32_t spindle_compute_freq_value(float rpm) // Mega2560 PWM register is 16-bit.
{
  uint32_t freq_value;
  // Compute intermediate frequence value with linear spindle speed model.
  // NOTE: A nonlinear model could be installed here, if required, but keep it VERY light-weight.
  freq_value = (uint32_t)(rpm * freq_gradient);
  return (freq_value);
}

// Immediately sets spindle running state with direction and spindle rpm via PWM, if enabled.
// Called by g-code parser spindle_sync(), parking retract and restore, g-code program end,
// sleep, and spindle stop override.
void spindle_set_state(uint8_t state, float rpm)
{
  if (sys.abort)
  {
    return;
  } // Block during abort.
  if (state == SPINDLE_DISABLE)
  { // Halt or set spindle direction and rpm.
    // sys.spindle_speed = 0.0;
    //    spindle_stop();
    rpm = 0.0;
  }
  else
  {
    // NOTE: Assumes all calls to this function is when Grbl is not moving or must remain off.
    if (settings.flags & BITFLAG_LASER_MODE)
    {
      if (state == SPINDLE_ENABLE_CCW)
      {
        rpm = 0.0;
      } // TODO: May need to be rpm_min*(100/MAX_SPINDLE_SPEED_OVERRIDE);
    }
  }

  if (sys.spindle_speed != rpm || sys_rt_exec_spindel_state != state)
  {

#ifndef SPINDLE_ENABLE_OFF_WITH_ZERO_SPEED
#ifdef INVERT_SPINDLE_ENABLE_PIN
    SPINDLE_ENABLE_PORT &= ~(1 << SPINDLE_ENABLE_BIT);
#else
    SPINDLE_ENABLE_PORT |= (1 << SPINDLE_ENABLE_BIT);
#endif
#endif
    // correcting RPM for output
    rpm *= (0.010 * sys.spindle_speed_ovr); // Scale by spindle speed override value.
    // Calculate PWM register value based on rpm max/min settings and programmed rpm.
    if ((settings.rpm_min >= settings.rpm_max) || (rpm >= settings.rpm_max))
    {
      // No range possible. Set simple on/off spindle control pin state.
      rpm = settings.rpm_max;
    }
    else if (rpm <= settings.rpm_min)
    {
      rpm = settings.rpm_min;
    }

    sys_rt_exec_spindel_state |= EXEC_SPINDLE_CHANGING_SPEED; // Set changing speed flag
    spindle_speed_changing(rpm, state);
  }

  sys.report_ovr_counter = 0; // Set to report change immediately
}

// This function make gradient of PRM for smooth acceleration and deceleration
void spindle_speed_changing(float end_rpm, uint8_t end_state)
{
  float now_speed_signed = sys.spindle_speed * ((sys_rt_exec_spindel_state & EXEC_SPINDLE_SYNCHRONIZED_CW) ? 1.0 : -1.0); // RPM with sign: "+" if CW, "-" if CCW
  float end_speed_signed = end_rpm * ((end_state & EXEC_SPINDLE_SYNCHRONIZED_CW) ? 1.0 : -1.0);
  float increment = (float)(SPINDLE_ROTATION_ACCELERATION * 0.00001);
  increment *= (end_speed_signed - now_speed_signed) >= 0 ? 1.0 : -1.0;
  while ((((end_speed_signed - now_speed_signed) >= 0 ? 1.0 : -1.0) == ((end_speed_signed - (now_speed_signed + increment)) >= 0 ? 1.0 : -1.0)))
  {
    now_speed_signed += increment;
    spindle_set_speed(spindle_compute_freq_value(abs(now_speed_signed)), (now_speed_signed > 0 ? EXEC_SPINDLE_SYNCHRONIZED_CW : EXEC_SPINDLE_SYNCHRONIZED_CCW));
    delay_us(10);
  }
  sys.spindle_speed = end_rpm;
  spindle_set_speed(spindle_compute_freq_value(abs(end_speed_signed)), (end_speed_signed > 0 ? EXEC_SPINDLE_SYNCHRONIZED_CW : EXEC_SPINDLE_SYNCHRONIZED_CCW));
  if (end_state == SPINDLE_DISABLE)
  {
    spindle_stop();
  }
  sys_rt_exec_spindel_state = end_state;
}

// G-code parser entry-point for setting spindle state. Forces a planner buffer sync and bails
// if an abort or check-mode is active.
void spindle_sync(uint8_t state, float rpm)
{
  if (sys.state == STATE_CHECK_MODE)
  {
    return;
  }
  protocol_buffer_synchronize(); // Empty planner buffer to ensure spindle is set when programmed.
  spindle_set_state(state, rpm);
}
