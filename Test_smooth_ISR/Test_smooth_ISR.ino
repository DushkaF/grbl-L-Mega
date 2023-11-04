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

// dynamic frequency generator
// for timer2
#include <avr/io.h>
#include <avr/interrupt.h>
unsigned long frequency = 300000;
void setup()
{
    DDRH |= (1 << 4);
    // DFG(frequency);
    //   Serial.begin(57600);
}

void loop()
{
    for (uint32_t i = 0; i < 400000; i += 1000)
    {
        DFG(i);
        delay(10);
    }
}

void DFG(unsigned long tempfreq)
{
    cli();      // disable interupts
    TCCR4A = 0; // registers for timer
    TCCR4B = 0;
    TCNT4 = 0;
    //   TCCR4A |= _BV(COM4A0) + _BV(COM4B0);
    TCCR4A |= (1 << COM4B0); // wavegeneratir on pin 7
    TCCR4B |= (1 << WGM42); // CTC with top in OCRn reg
    // TCCR4C = (1 << FOC4A);
    if (tempfreq > 122 && tempfreq < 1000001){
        OCR4A = (8000000 / tempfreq) - 1; // #TIMER COUNTS
        TCCR4B |= (1 << CS40);
    } else if (tempfreq <= 122 && tempfreq > 15) {
        OCR4A = (1000000 / tempfreq) - 1;
        TCCR4B |= (1 << CS41);
    } else if (tempfreq <= 15 && tempfreq > 4){
        OCR4A = (125000 / tempfreq) - 1;
        TCCR4B |= (1 << CS40) + (1 << CS41);
    }
    // TIMSK1 = _BV(OCIE1A);//TIMER1 COMPARE INTERUPT
    sei(); // enable interupts
}