// Define standard libraries used by Grbl.
#include <util/atomic.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <math.h>
#include <inttypes.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#define DEFAULTS_RAMPS_BOARD

#define N_AXIS 3 // Number of axes
#define X_AXIS 0 // Axis indexing value.
#define Y_AXIS 1
#define Z_AXIS 2

// Define ports and pins
#define DDR(port) DDR##port
#define _DDR(port) DDR(port)
#define PORT(port) PORT##port
#define _PORT(port) PORT(port)
#define PIN(pin) PIN##pin
#define _PIN(pin) PIN(pin)

#define STEP_PORT_1 F
#define STEP_BIT_1 6 // Y Step - Pin A6
#define _STEP_BIT(i) STEP_BIT_##i
#define STEP_BIT(i) _STEP_BIT(i)
#define STEP_DDR(i) _DDR(STEP_PORT_##i)
#define _STEP_PORT(i) _PORT(STEP_PORT_##i)
#define STEP_PORT(i) _STEP_PORT(i)
#define STEP_PIN(i) _PIN(STEP_PORT_##i)

#define DIRECTION_PORT_1 F
#define DIRECTION_BIT_1 7 // Y Dir - Pin A7
#define _DIRECTION_BIT(i) DIRECTION_BIT_##i
#define DIRECTION_BIT(i) _DIRECTION_BIT(i)
#define DIRECTION_DDR(i) _DDR(DIRECTION_PORT_##i)
#define _DIRECTION_PORT(i) _PORT(DIRECTION_PORT_##i)
#define DIRECTION_PORT(i) _DIRECTION_PORT(i)
#define DIRECTION_PIN(i) _PIN(DIRECTION_PORT_##i)

#define STEPPER_DISABLE_PORT_1 F
#define STEPPER_DISABLE_BIT_1 2 // Y Enable - Pin A2
#define STEPPER_DISABLE_BIT(i) STEPPER_DISABLE_BIT_##i
#define STEPPER_DISABLE_DDR(i) _DDR(STEPPER_DISABLE_PORT_##i)
#define STEPPER_DISABLE_PORT(i) _PORT(STEPPER_DISABLE_PORT_##i)
#define STEPPER_DISABLE_PIN(i) _PIN(STEPPER_DISABLE_PORT_##i)

// Advanced Configuration Below You should not need to touch these variables
// Set Timer up to use TIMER4B which is attached to Digital Pin 8 - Ramps 1.4 12v output with heat sink

#define SPINDLE_TIMER          TIMER4_COMPA_vect
#define SPINDLE_TCCRA_REGISTER TCCR4A
#define SPINDLE_TCCRB_REGISTER TCCR4B
#define SPINDLE_TIMSK_REGISTER TIMSK4
#define SPINDLE_ICR_REGISTER   ICR4
#define SPINDLE_OCIE_BIT       OCIE4A

#define SPINDLE_TCCRA_INIT_MASK 0
#define SPINDLE_TCCRB_INIT_MASK ((1 << WGM42) | (1 << WGM43))

// Define spindle output pins.
#define SPINDLE_CONTROL_DDR STEP_DDR(1)
#define SPINDLE_CONTROL_PORT STEP_PORT(1)
#define SPINDLE_CONTROL_BIT STEP_BIT(1)

// Define spindle enable and spindle direction output pins.
#define SPINDLE_ENABLE_DDR STEPPER_DISABLE_DDR(1)
#define SPINDLE_ENABLE_PORT STEPPER_DISABLE_PORT(1)
#define SPINDLE_ENABLE_BIT STEPPER_DISABLE_BIT(1)
#define INVERT_SPINDLE_ENABLE_PIN

#define SPINDLE_DIRECTION_DDR DIRECTION_DDR(1)
#define SPINDLE_DIRECTION_PORT DIRECTION_PORT(1)
#define SPINDLE_DIRECTION_BIT DIRECTION_BIT(1)

#define SPINDLE_STEPS_PER_REVOLUTION  200           // for stepper with 1.8 grad step
#define SPINDLE_MICROSTEP_DIVIDER     32
#define SPINDLE_RPM_OFF_VALUE         0

#define SPINDLE_ROTATION_ACCELERATION 50000 // For soft start/stop spindel. In RPM/s


// Modal Group M7: Spindle control
#define SPINDLE_DISABLE 0 // M5 (Default: Must be zero)
#define SPINDLE_ENABLE_CW    bit(4) // M3 (NOTE: Uses planner condition bit flag)
#define SPINDLE_ENABLE_CCW   bit(5) // M4 (NOTE: Uses planner condition bit flag)

#define EXEC_SPINDLE_DISABLED          bit(0)   //REVIEW spindel speed states
#define EXEC_SPINDLE_CHANGING_SPEED    bit(1)
#define EXEC_SPINDLE_SYNCHRONIZED_CW   SPINDLE_ENABLE_CW
#define EXEC_SPINDLE_SYNCHRONIZED_CCW   SPINDLE_ENABLE_CCW


#define DEFAULT_SPINDLE_SPEED_OVERRIDE    100 // 100%. Don't change this value.

#define BITFLAG_LASER_MODE         bit(1)

// Define global system variables
typedef struct {
  uint8_t state;               // Tracks the current system state of Grbl.
  uint8_t abort;               // System abort flag. Forces exit back to main loop for reset.
  uint8_t suspend;             // System suspend bitflag variable that manages holds, cancels, and safety door.
  uint8_t soft_limit;          // Tracks soft limit errors for the state machine. (boolean)
  uint8_t step_control;        // Governs the step segment generator depending on system state.
  uint8_t probe_succeeded;     // Tracks if last probing cycle was successful.
#ifdef DEFAULTS_RAMPS_BOARD
  uint8_t homing_axis_lock[N_AXIS];    // Locks axes when limits engage. Used as an axis motion mask in the stepper ISR.
#else
  uint8_t homing_axis_lock;    // Locks axes when limits engage. Used as an axis motion mask in the stepper ISR.
#endif
  uint8_t f_override;          // Feed rate override value in percent
  uint8_t r_override;          // Rapids override value in percent
  uint8_t spindle_speed_ovr;   // Spindle speed value in percent
  uint8_t spindle_stop_ovr;    // Tracks spindle stop override states
  uint8_t report_ovr_counter;  // Tracks when to add override data to status reports.
  uint8_t report_wco_counter;  // Tracks when to add work coordinate offset data to status reports.
#ifdef ENABLE_PARKING_OVERRIDE_CONTROL
  uint8_t override_ctrl;     // Tracks override control states.
#endif
  float spindle_speed;
} system_t;
system_t sys;

typedef struct {
  // Axis settings
  float steps_per_mm[N_AXIS];
  float max_rate[N_AXIS];
  float acceleration[N_AXIS];
  float max_travel[N_AXIS];

  // Remaining Grbl settings
  uint8_t pulse_microseconds;
  uint8_t step_invert_mask;
  uint8_t dir_invert_mask;
  uint8_t stepper_idle_lock_time; // If max value 255, steppers do not disable.
  uint8_t status_report_mask; // Mask to indicate desired report data.
  float junction_deviation;
  float arc_tolerance;
  float rpm_max;
  float rpm_min;
  uint8_t flags;  // Contains default boolean settings

  uint8_t homing_dir_mask;
  float homing_feed_rate;
  float homing_seek_rate;
  uint16_t homing_debounce_delay;
  float homing_pulloff;

  uint16_t sync_pulses_per_revolution;  // the number of synchronization pulses per spindle revolution
  uint16_t debounce_tics;               // the number of 4 us debounce tics for index and sync pulses
} settings_t;
settings_t settings;

volatile uint8_t sys_rt_exec_spindel_speed_change = 0;


static float freq_gradient; // Precalulated value to speed up rpm to PWM conversions.



// ========================== SET PERIOD ==========================
uint32_t set_period(uint32_t _timer_period)
{
  //TODO  make safety for max and min period
  // _timer4_period = constrain(_timer4_period, 1, MAX_PERIOD_16);

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
  SPINDLE_ICR_REGISTER = _timer_top - 1;                                 // Set timer top

  return (1000000UL / ((F_CPU / _timer_divider) / _timer_top)); // Return real timer period
}

uint32_t set_frequency(uint32_t _timer_frequency) {
  return 1000000UL / (set_period(1000000UL / _timer_frequency));
}

// Delays variable defined microseconds. Compiler compatibility fix for _delay_us(),
// which only accepts constants in future compiler releases. Written to perform more
// efficiently with larger delays, as the counter adds parasitic time in each iteration.
void delay_us(uint32_t us)
{
  while (us) {
    if (us < 10) {
      _delay_us(1);
      us--;
    } else if (us < 100) {
      _delay_us(10);
      us -= 10;
    } else if (us < 1000) {
      _delay_us(100);
      us -= 100;
    } else {
      _delay_ms(1);
      us -= 1000;
    }
  }
}

ISR(SPINDLE_TIMER) {
  SPINDLE_CONTROL_PORT ^= (1 << SPINDLE_CONTROL_BIT);
}

void spindle_init()
{
  // Configure variable spindle PWM and enable pin, if required.
  SPINDLE_CONTROL_DDR |= (1 << SPINDLE_CONTROL_BIT);        // Configure as PWM output pin.
  SPINDLE_TCCRA_REGISTER = SPINDLE_TCCRA_INIT_MASK; // Configure PWM output compare timer
  SPINDLE_TCCRB_REGISTER = SPINDLE_TCCRB_INIT_MASK;
  SPINDLE_ENABLE_DDR |= (1 << SPINDLE_ENABLE_BIT);       // Configure as output pin.
  SPINDLE_DIRECTION_DDR |= (1 << SPINDLE_DIRECTION_BIT); // Configure as output pin.

  freq_gradient = (1.0/ 60.0) * (360.0 * SPINDLE_MICROSTEP_DIVIDER / 1.8);
  spindle_stop();
}

void spindle_stop()
{
  SPINDLE_TIMSK_REGISTER &= ~(1 << SPINDLE_OCIE_BIT); // turn off SPINDLE_TIMER
  SPINDLE_CONTROL_PORT &= ~(1 << SPINDLE_CONTROL_BIT);        // set step pin low
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
  if (state == SPINDLE_ENABLE_CW) //Set direction
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
    SPINDLE_TIMSK_REGISTER &= ~(1 << SPINDLE_OCIE_BIT); // turn off SPINDLE_TIMER
    SPINDLE_CONTROL_PORT &= ~(1 << SPINDLE_CONTROL_BIT);        // set step pin low
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

// TODO This function must makes gradient of RPM for smooth acceleration and deceleration
// Called by spindle_set_state() and step segment generator. Keep routine small and efficient.
uint32_t spindle_compute_freq_value(float rpm) // Mega2560 PWM register is 16-bit.
{
  uint32_t freq_value;
  // Compute intermediate frequence value with linear spindle speed model.
  // NOTE: A nonlinear model could be installed here, if required, but keep it VERY light-weight.
  Serial.print("RPM ");
  Serial.println(rpm);
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

  if (sys.spindle_speed != rpm || sys_rt_exec_spindel_speed_change != state)
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
    
    sys_rt_exec_spindel_speed_change |= EXEC_SPINDLE_CHANGING_SPEED;  // Set changing speed flag
    spindle_speed_changing(rpm, state);
  }

  sys.report_ovr_counter = 0; // Set to report change immediately
}

// This function realise soft changing speed
void spindle_speed_changing(float end_rpm, uint8_t end_state)
{
  float now_speed_signed = sys.spindle_speed * ((sys_rt_exec_spindel_speed_change & EXEC_SPINDLE_SYNCHRONIZED_CW) ? 1.0 : -1.0); // RPM with sign: "+" if CW, "-" if CCW
  float end_speed_signed = end_rpm * ((end_state & EXEC_SPINDLE_SYNCHRONIZED_CW) ? 1.0 : -1.0);
  float increment = (float)(SPINDLE_ROTATION_ACCELERATION * 0.00001);
  increment *= (end_speed_signed - now_speed_signed) >= 0 ? 1.0 : -1.0;
  while ((((end_speed_signed - now_speed_signed) >= 0 ? 1.0 : -1.0) == ((end_speed_signed - (now_speed_signed + increment)) >= 0 ? 1.0 : -1.0)))
  {
    now_speed_signed += increment;
    spindle_set_speed(spindle_compute_freq_value(abs(now_speed_signed)), (now_speed_signed > 0 ? EXEC_SPINDLE_SYNCHRONIZED_CW : EXEC_SPINDLE_SYNCHRONIZED_CCW));
    delayMicroseconds(10);
  }
  sys.spindle_speed = end_rpm;
  spindle_set_speed(spindle_compute_freq_value(abs(end_speed_signed)), (end_speed_signed > 0 ? EXEC_SPINDLE_SYNCHRONIZED_CW : EXEC_SPINDLE_SYNCHRONIZED_CCW));
  if (end_state == SPINDLE_DISABLE)
  {
    spindle_stop();
  }
  sys_rt_exec_spindel_speed_change = end_state;
}

void setup() {
  spindle_init();
  sys.spindle_speed_ovr = DEFAULT_SPINDLE_SPEED_OVERRIDE;    // Set to 100%
  settings.rpm_max = 300;
  settings.rpm_min = 0;
  Serial.begin(115200);
}

void loop()
{

  spindle_set_state(SPINDLE_ENABLE_CCW, 200);
  Serial.println("NEXT");
  delay(1000);

//  spindle_set_state(SPINDLE_DISABLE, 0);
  spindle_set_state(SPINDLE_ENABLE_CW, 200);
  Serial.println("NEXT");
  delay(1000);

  spindle_set_state(SPINDLE_ENABLE_CW, 50);
//  spindle_set_state(SPINDLE_ENABLE_CW, 2000);
  Serial.println("NEXT");
  delay(1000);

  spindle_set_state(SPINDLE_DISABLE, 0);
  Serial.println("NEXT");
  delay(1000);

    spindle_set_state(SPINDLE_ENABLE_CW, 0);
//  spindle_set_state(SPINDLE_ENABLE_CW, 2000);
  Serial.println("NEXT");
  delay(1000);
}
