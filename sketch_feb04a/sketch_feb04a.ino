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

// Define spindle output pins.
#define SPINDLE_PWM_DDR STEP_DDR(1)
#define SPINDLE_PWM_PORT STEP_PORT(1)
#define SPINDLE_PWM_BIT STEP_BIT(1)

#define SPINDLE_TIMER          TIMER4_COMPA_vect
#define SPINDLE_TCCRA_REGISTER TCCR4A
#define SPINDLE_TCCRB_REGISTER TCCR4B
#define SPINDLE_TIMSK_REGISTER TIMSK4
#define SPINDLE_ICR_REGISTER   ICR4
#define SPINDLE_OCIE_BIT       OCIE4A
// #define SPINDLE_COMB_BIT COM4C1

#define SPINDLE_TCCRA_INIT_MASK 0
#define SPINDLE_TCCRB_INIT_MASK ((1 << WGM42) | (1 << WGM43))
#define SPINDLE_OCRA_TOP_VALUE 0x0400 // PWM counter reset value. Should be the same as PWM_MAX_VALUE in hex.



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
  //  _timer_clock = (SPINDLE_TCCRB_REGISTER & 0x07);                        // Save timer clock settings

  return (1000000UL / ((F_CPU / _timer_divider) / _timer_top)); // Return real timer period
}

uint32_t set_frequency(uint32_t _timer_frequency) {
  return 1000000UL / (set_period(1000000UL / _timer_frequency));
}

void spindle_init()
{
  // Configure variable spindle PWM and enable pin, if required.
  SPINDLE_TCCRA_REGISTER = SPINDLE_TCCRA_INIT_MASK; // Configure PWM output compare timer
  SPINDLE_TCCRB_REGISTER = SPINDLE_TCCRB_INIT_MASK;
  SPINDLE_PWM_DDR |= (1 << SPINDLE_PWM_BIT);        // Configure as PWM output pin.
}

ISR(SPINDLE_TIMER) {
  SPINDLE_PWM_PORT = (~SPINDLE_PWM_PORT & (1 << SPINDLE_PWM_BIT));
}

void setup() {
  Serial.begin(115200);
  // put your setup code here, to run once:
  spindle_init();
  //  SPINDLE_TCCRB_REGISTER |= ((1 << CS40) | (1 << CS42));
  set_frequency(30);
  //  SPINDLE_ICR_REGISTER = 30000;
  SPINDLE_TIMSK_REGISTER |= (1 << SPINDLE_OCIE_BIT);
  sei();
}

void loop() {
  set_frequency(30);
  //  sei();
  delay(1000);
  set_frequency(10);
  //  sei();
  delay(1000);
}
