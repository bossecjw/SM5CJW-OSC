//CONTROL PROGRAM FOR ARDF RECEIVER 80m19 VFO   2019-02-13

#include "sysconf.h"

#include <avr/io.h>
#include <util/delay.h>

#include "si5351-avr-tiny-minimal.h"

extern struct Si5351Status dev_status;
extern struct Si5351IntStatus dev_int_status;

#define ADCBITS 10
#define ADC2 2
#define POT_PIN PB4
#define DIG_PIN PB3
#define LED_PIN PB1

#define SETBIT(port, x) (port |= (1<<x))
#define CLRBIT(port, x) (port &= ~(1<<x))
#define GETBIT(pin, x) (pin & (1<<x))

//int analogRead(uint8_t pin)
int analogRead()
{
    uint8_t low, high;

    //ADMUX = (pin & 0x07);
    ADMUX = (ADC2 & 0x07);

    // start the conversion
    SETBIT(ADCSRA, ADSC);

    // ADSC is cleared when the conversion finishes
    while (GETBIT(ADCSRA, ADSC));

    // we have to read ADCL first; doing so locks both ADCL
    // and ADCH until ADCH is read.  reading ADCL second would
    // cause the results of each conversion to be discarded,
    // as ADCL and ADCH would be locked when it completed.
    low  = ADCL;
    high = ADCH;

    // combine the two bytes
    return (high << 8) | low;
}

inline int digitalRead()
{
    return GETBIT(PINB, DIG_PIN);
}

void sleep(uint16_t millisec)
{
    while(millisec)
    {
        _delay_ms(1);/* 1 ms delay */
        millisec--;
    }
}

// 3 separate oscillators in the frequency range 14 kHz - 114 MHz
//
// If a divider must be used (f < 1800 kHz) - set the frequency DIV times higher
// (DIV = 1, 2, 4, 8, 16, 32, 64 or 128)

/* Valid defines for divider:
 *
 * SI5351_OUTPUT_CLK_DIV_1
 * SI5351_OUTPUT_CLK_DIV_2
 * SI5351_OUTPUT_CLK_DIV_4
 * SI5351_OUTPUT_CLK_DIV_8
 * SI5351_OUTPUT_CLK_DIV_16
 * SI5351_OUTPUT_CLK_DIV_32
 * SI5351_OUTPUT_CLK_DIV_64
 * SI5351_OUTPUT_CLK_DIV_128
 */

// Defines for frequency ranges 1800 kHz to 114 MHz

// VFO 7000 - 7204,8 kHz for 80m19 Switched mixer (on CLK0)
// BASE = lowest frequency (f1), Hz   here: 6996 kHz
// COARSE = tuning range (f2 - f1), Hz   here: 204800 Hz
// FINE = fine tuning range +/-FINE/2 Hz in 1024 steps  here: 20480 Hz

// Output 0   here VFO output

#define CLK0_BASE       7000000UL
#define CLK0_COARSE      204800UL

#define CLK0_FINE        20480UL
#define CLK0_DIV     SI5351_OUTPUT_CLK_DIV_1
// Output 1   here: disabled
#define CLK1_FREQ      1000000UL
#define CLK1_DIV     SI5351_OUTPUT_CLK_DIV_1
// Output 2   here: disabled
#define CLK2_FREQ      1820000UL
#define CLK2_DIV     SI5351_OUTPUT_CLK_DIV_4

int main()
{
    int hysteresDir = 0; // 1=up, 0=down
    int newButtonVal, oldButtonVal = 0;
    int64_t range0 = 0, range1 = 0;
    int oldPotVal = 0;
    int potMode = 0;
    int calActive = 0;
    // Initialize pins
    CLRBIT(DDRB, POT_PIN); // Pot = input
    CLRBIT(DDRB, DIG_PIN); // Dig = input
    SETBIT(DDRB, LED_PIN); // Led = output

    SETBIT(PORTB, DIG_PIN); // Dig = pull-up

    // Enable ADC
    SETBIT(ADCSRA, ADEN);

    // TEMP SETUP FAST PWM
    SETBIT(TCCR0A, WGM00);
    SETBIT(TCCR0A, WGM01);

    // Initialize si5351a library
    si5351_init(SI5351_CRYSTAL_LOAD_6PF, 0);

    si5351_output_enable(SI5351_CLK0, 1);
    si5351_set_freq(CLK0_BASE, SI5351_CLK0);
    si5351_drive_strength(SI5351_CLK0, SI5351_DRIVE_2MA);
    //Set calibrator
    si5351_output_enable(SI5351_CLK1, 0);
    si5351_set_freq(CLK1_FREQ, SI5351_CLK1);
    si5351_set_ms_div(SI5351_CLK1, CLK1_DIV, (CLK1_DIV == SI5351_OUTPUT_CLK_DIV_4) ? 1 : 0 );
    si5351_drive_strength(SI5351_CLK1, SI5351_DRIVE_2MA);
    //Set BFO
    si5351_output_enable(SI5351_CLK2, 0);
    si5351_set_freq(CLK2_FREQ, SI5351_CLK2);
    si5351_set_ms_div(SI5351_CLK2, CLK2_DIV, (CLK2_DIV == SI5351_OUTPUT_CLK_DIV_4) ? 1 : 0 );
    si5351_drive_strength(SI5351_CLK2, SI5351_DRIVE_2MA);

    // There will be some inherent error in the reference crystal's actual frequency, so we can measure the difference (f - fnom ) between the actual and nominal output frequency in Hz, enter this figure into the library.
    si5351_set_correction(40);

    while(1)
    {
        /**
          HANDLE BUTTON PRESSES
          */
        // Handle button presses. Switch between coarse and fine mode (potMode 0=Coarse, 1=Fine).
        newButtonVal = digitalRead();
        if(oldButtonVal != newButtonVal && !newButtonVal) {
            int cnt = 10000;
            while(!newButtonVal && cnt > 0) {
                cnt--;
                _delay_ms(1);
                newButtonVal = digitalRead();
            }

            // Long press
            if(cnt <= 0) {
                //Toggle calibrator
                calActive = !calActive;
                si5351_output_enable(SI5351_CLK1, calActive);
            }
            // Short press
            else
            {
                //Toggle coarse and fine pot mode
                potMode = !potMode;
                if(potMode)
                    SETBIT(PORTB, LED_PIN);
                else
                    CLRBIT(PORTB, LED_PIN);
            }
        }
        oldButtonVal = newButtonVal;

        /**
          HANDLE POTENTIOMETER
          */
        // Read pot
        uint16_t newPotVal = analogRead();
        // Detect hysteresis, only change if same direction, or 2 steps
        int shouldSetNew = 0;
        int diff = newPotVal - oldPotVal;
        if(diff > 1) shouldSetNew = 1;
        if(diff < -1) shouldSetNew = 1;
        if(hysteresDir == 1) {
            if(diff > 0) shouldSetNew = 1;
        } else {
            if(diff < 0) shouldSetNew = 1;
        }

        if(shouldSetNew) {
            if(diff > 0)
                hysteresDir = 1; //up
            else
                hysteresDir = 0; //down

            // Set fine
            if(potMode) {
                range1 = ((uint64_t)newPotVal * CLK0_FINE) >> ADCBITS;
                range1 = range1 - (CLK0_FINE/2);
            }
            // Set coarse
            else {
                range0 = ((uint64_t)newPotVal * CLK0_COARSE) >> ADCBITS; range1 = 0;
            }
            // Set frequency
            uint64_t newFreq = CLK0_BASE + range0 + range1;

            si5351_set_freq(newFreq, SI5351_CLK0);
            si5351_set_ms_div(SI5351_CLK0, CLK0_DIV, (CLK0_DIV == SI5351_OUTPUT_CLK_DIV_4) ? 1 : 0 );
            oldPotVal = newPotVal;
        }
    }

    // We will never get here
    return 0;
}

