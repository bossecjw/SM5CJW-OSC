//CONTROL PROGRAM FOR ARDF RECEIVER 80m20 VFO   2019-11-08

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

// VFO 12486 - 12600 kHz for 80m20 mixer SA612AD (on CLK0)
// BASE = lowest frequency (f1), Hz   here: 12486 kHz
// COARSE = tuning range (f2 - f1), Hz   here: 102400 Hz
// FINE = fine tuning range +/- 5120 Hz in 1024 10 Hz steps  here: 10240 Hz

// Output 0   here VFO output

#define CLK0_BASE      12486000UL
#define CLK0_COARSE      102400UL

#define CLK0_FINE         10240UL
#define CLK0_DIV     SI5351_OUTPUT_CLK_DIV_1
// Output 1   here: Calibrator (after 10 seconds on pushbutton)
#define CLK1_FREQ      3579545UL
#define CLK1_DIV     SI5351_OUTPUT_CLK_DIV_1
// Output 2   here: disabled
#define CLK2_FREQ      1820000UL
#define CLK2_DIV     SI5351_OUTPUT_CLK_DIV_4

/* ======================================= */
/* FUNCTION THAT INITIALIZES Si5351 DEVICE */
/* ======================================= */

void initialize() {
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
    si5351_drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);
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
}

/* ============================================= */
/* FINITE STATE MACHINE DEFINITION AND FUNCTIONS */
/* ============================================= */

// Describe Finite State Machine states
#define S_STEP_COARSE   0
#define S_STEP_FINE     1
#define S_CONT_COARSE   2
#define S_CONT_FINE     3

int isFine(int state) {
    return (state == S_STEP_FINE) || (state == S_CONT_FINE);
}

int isCont(int state) {
    return (state == S_CONT_COARSE) || (state == S_CONT_FINE);
}

#define LONG_PRESS_TIMER 10000

// Traverse FSM with button presses, returns new state
int handleFSM(int state) {
    static int oldButtonVal = 0;

    // Handle button presses. Switch between coarse and fine mode
    int newButtonVal = digitalRead();
    if(oldButtonVal != newButtonVal && !newButtonVal) {
        int cnt = LONG_PRESS_TIMER;
        while(!newButtonVal && cnt > 0) {
            cnt--;
            _delay_ms(1);
            newButtonVal = digitalRead();
        }

        // Long press (> 10 seconds)
        if(cnt <= 0) {
            if(!isFine(state)) {
                state = (state == S_STEP_COARSE) ? S_CONT_COARSE : S_STEP_COARSE; 
            }
        }
        // Short press, Toggle coarse and fine state
        else if(isFine(state)) {
            state = (state == S_STEP_FINE) ? S_STEP_COARSE : S_CONT_COARSE;
        } else {
            state = (state == S_STEP_COARSE) ? S_STEP_FINE : S_CONT_FINE;
        }
    }
    oldButtonVal = newButtonVal;

    return state;
}

/* ==================== */
/* LED CONTROL FUNCTION */
/* ==================== */

#define LED_BLINK_TIMER 200

void handleLed(int state) {
    static int ledTimer = 0;
    static int ledState = 0;

    if(isFine(state)) {
        if(isCont(state)) {
            SETBIT(PORTB, LED_PIN);
        } else {
            _delay_ms(1);
            ledTimer++;
            if(ledTimer >= LED_BLINK_TIMER) {
                ledTimer = 0;
                ledState = !ledState;
                if(ledState) {
                    SETBIT(PORTB, LED_PIN);
                } else {
                    CLRBIT(PORTB, LED_PIN);
                }
            }
        }
    } else {
        CLRBIT(PORTB, LED_PIN);
    }
}

/* ====================== */
/* POTENTIOMETER FUNCTION */
/* ====================== */

#define POT_MAX     1034
#define POT_STEPS   11
#define STEP_STRIDE (POT_MAX / POT_STEPS)

int64_t step_array[POT_STEPS] = {
    /*  0 */ 12486000UL,
    /*  1 */ 12496000UL,
    /*  2 */ 12506000UL,
    /*  3 */ 12516000UL,
    /*  4 */ 12526000UL,
    /*  5 */ 12536000UL,
    /*  6 */ 12546000UL,
    /*  7 */ 12556000UL,
    /*  8 */ 12566000UL,
    /*  9 */ 12576000UL,
    /* 10 */ 12586000UL,
};

// Check if we want to change or not according to how much pot has moved, and what dir
int potShouldSetNew(int diff, int hysteresDir) {
    if(diff > 1) return 1;
    if(diff < -1) return 1;
    if(hysteresDir == 1) {
        if(diff > 0) return 1;
    } else {
        if(diff < 0) return 1;
    }
    return 0;
}

void handlePot(int state) {
    static int hysteresDir = 0; // 1=up, 0=down
    static int64_t range_coarse = 0;
    static int64_t range_fine = 0;
    static int oldPotVal = 0;

    // Read pot
    uint16_t newPotVal = analogRead();
    // Detect hysteresis, only change if same direction, or 2 steps
    int diff = newPotVal - oldPotVal;

    if(potShouldSetNew(diff, hysteresDir)) {
        if(diff > 0)
            hysteresDir = 1; //up
        else
            hysteresDir = 0; //down

        /* State specific behavior */

        // Set fine
        if(isFine(state)) {
            range_fine = ((uint64_t)newPotVal * CLK0_FINE) >> ADCBITS;
            range_fine = range_fine - (CLK0_FINE/2);
        } else { // Set coarse
            range_fine = 0;
            if(isCont(state)) {
                int64_t coarse_freq = ((uint64_t)newPotVal * CLK0_COARSE) >> ADCBITS;
                range_coarse = CLK0_BASE + coarse_freq;
            } else {
                int step = newPotVal / STEP_STRIDE;
                range_coarse = step_array[step];
            }
        }

        /* Set Si5351 CLK0 frequency */
        uint64_t newFreq = range_coarse + range_fine;

        si5351_set_freq(newFreq, SI5351_CLK0);
        si5351_set_ms_div(SI5351_CLK0, CLK0_DIV, (CLK0_DIV == SI5351_OUTPUT_CLK_DIV_4) ? 1 : 0 );
        oldPotVal = newPotVal;
    }
}

/* ========= */
/* MAIN LOOP */
/* ========= */
int main()
{
    int state = S_STEP_COARSE;

    initialize();

    while(1)
    {
        // HANDLE BUTTON PRESSES
        state = handleFSM(state);

        // HANDLE LED
        handleLed(state);

        // HANDLE POTENTIOMETER
        handlePot(state);
    }

    // We will never get here
    return 0;
}
