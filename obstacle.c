#include <avr/io.h>
#include <util/delay.h>

// Define pin connections
#define SERVO_PIN1 PB1 // Pin 9 on Arduino Uno for Servo 1
#define SERVO_PIN2 PB2 // Pin 10 on Arduino Uno for Servo 2
#define BUTTON_CW_PIN PD3 // Pin 3 on Arduino Uno for CW rotation
#define BUTTON_CCW_PIN PD2 // Pin 2 on Arduino Uno for CCW rotation
#define BUTTON_LEFT_PIN PD4 // Pin 4 on Arduino Uno for left rotation
#define BUTTON_RIGHT_PIN PD5 // Pin 5 on Arduino Uno for right rotation

// Initialize servo positions at 90 degrees
int current1 = 90;
int current2 = 90;

void init() {
    // Set servo pins as outputs
    DDRB |= (1 << SERVO_PIN1) | (1 << SERVO_PIN2);

    // Set button pins as inputs
    DDRD &= ~((1 << BUTTON_CW_PIN) | (1 << BUTTON_CCW_PIN) | (1 << BUTTON_LEFT_PIN) | (1 << BUTTON_RIGHT_PIN));

    // Timer 1 configuration for servo control (Fast PWM, ICR1 as top)
    ICR1 = 39999; // Top value for 20ms period, using prescaler 8
    TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11); // Enable non-inverting mode on Channel A and B
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11); // Set prescaler to 8 and use ICR1 as top

    // Set initial PWM values (1500 corresponds to 1.5ms pulse width, neutral position)
    OCR1A = 3000; // 1500us
    OCR1B = 3000; // 1500us
}

void updateServo(int *current, uint8_t servoPin) {
    // Ensure the servo angle is within the allowed range
    if (*current > 180) *current = 180;
    else if (*current < 0) *current = 0;

    // Convert angle to PWM value (between 2000 for 0 degrees and 4000 for 180 degrees)
    uint16_t pwmValue = 1000 + (int)(22.22 * (*current));

    // Update the appropriate servo
    if (servoPin == SERVO_PIN1) {
        OCR1A = pwmValue;
    } else {
        OCR1B = pwmValue;
    }
}

int main(void) {
    init();

    while (1) {
        if (PIND & (1 << BUTTON_CW_PIN)) {
            if (current1 < 180) {
                current1 += 1; // Increase angle by 1 degree
                updateServo(&current1, SERVO_PIN1);
            }
            _delay_ms(20); // Add a delay for button press debouncing
        } else if (PIND & (1 << BUTTON_CCW_PIN)) {
            if (current1 > 0) {
                current1 -= 1; // Decrease angle by 1 degree
                updateServo(&current1, SERVO_PIN1);
            }
            _delay_ms(20);
        }

        if (PIND & (1 << BUTTON_LEFT_PIN)) {
            if (current2 < 180) {
                current2 += 1;
                updateServo(&current2, SERVO_PIN2);
            }
            _delay_ms(20);
        } else if (PIND & (1 << BUTTON_RIGHT_PIN)) {
            if (current2 > 0) {
                current2 -= 1;
                updateServo(&current2, SERVO_PIN2);
            }
            _delay_ms(20);
        }
    }
}


