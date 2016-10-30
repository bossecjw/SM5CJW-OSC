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


// Right now, PWM output only works on the pins with
// hardware support.  These are defined in the appropriate
// pins_*.c file.  For the rest of the pins, we default
// to digital output.

//TODO DOESN*T WORK!!!
void analogWrite(uint8_t val)
{
    SETBIT(TCCR0A, COM0B1);
    OCR0B = val; // set pwm duty
}



void sleep(uint16_t millisec)
{
    while(millisec)
    {
        _delay_ms(1);/* 1 ms delay */
        millisec--;
    }
}

#define CLK0_BASE    3500000UL
#define CLK0_COARSE   100000UL
#define CLK0_FINE      10000UL

int main()
{
    int newVal, oldVal = 0;
    int32_t range0 = 0, range1 = 0;
    int potMode = 0;
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

    // There will be some inherent error in the reference crystal's actual frequency, so we can measure the difference between the actual and nominal output frequency in Hz, multiply by 10, make it an integer, and enter this correction factor into the library.
    //si5351_set_correction(-900);

    while(1)
    {
	/*
        si5351_update_status();

        if(dev_int_status.SYS_INIT_STKY)
            SETBIT(PORTB, LED_PIN);
        else
            CLRBIT(PORTB, LED_PIN);
	*/
	newVal = digitalRead();
	if(oldVal != newVal && !newVal) {
		potMode = !potMode;
		if(potMode)
			SETBIT(PORTB, LED_PIN);
		else
			CLRBIT(PORTB, LED_PIN);
	}
	oldVal = newVal;

	uint16_t val = analogRead();
        if(potMode) {
		range1 = (val * CLK0_FINE) >> ADCBITS;
		range1 = range1 - (CLK0_FINE/2);
	} else {
		range0 = (val * CLK0_COARSE) >> ADCBITS;
	}
	si5351_set_freq(CLK0_BASE + range0 + range1, SI5351_CLK0);
        //analogWrite(128);

/*
        SETBIT(PORTB, LED_PIN);
        sleep(val);
        CLRBIT(PORTB, LED_PIN);
        sleep(val);
*/        
        /*
        if(digitalRead())
            CLRBIT(PORTB, LED_PIN);
        else
            SETBIT(PORTB, LED_PIN);
        */
    }
    return 0;
}

