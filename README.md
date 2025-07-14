    AT90 Microcontroller Lab Frequency Measurement

    Lab Week 12 Frequency Display

The frequency generator on our lab board is connected to IC1 (=PD4) of the AT90. The frequency

generator produces a PWM signal of a certain frequency and duty cycle.

Design and implement a program that measures the frequency of the signal on IC1 (PD4) and sends

it through the USART. It is not necessary to measure the duty cycle.

Button Functions:

1. PA0: triggers frequency calculation when pressed.

Display & Behaviour:

1. Output: Display frequency result via USART (e.g., PuTTY terminal).

2. Input Capture Setup:

a) Trigger on rising edge

b) Prescaler: 8

3. When PA0 is pressed, the current frequency should be measured and displayed correctly on

PuTTY.

Hints:

1. If the frequency displayed on the onboard display is "5", your program should output "1000 Hz"

or "1 kHz”.

2. A sample USART function for sending data to Putty is listed as blow.

void uart_send_char(char c) {

while (!(UCSR1A & (1 << UDRE1)));

UDR1 = c;

}

void uart_send_string(const char *str) {

while (*str) {

uart_send_char(*str++);

}

}

void uart_send_uint32(uint32_t value) {

char buffer[10];

char *ptr = buffer + sizeof(buffer) - 1;

*ptr = '\0';

if (value == 0) {

uart_send_char('0');

return;

}

while (value != 0) {

ptr--;

*ptr = '0' + (value % 10);

value /= 10;

}

uart_send_string(ptr);

}



/*

*	GccApplication4.c
*	
 *

*	Created: 4/06/2025 5:49:23 pm
*	
*	Author : 
*	
 */ 

#define F_CPU 8000000UL  // System clock frequency: 8MHz

#include <avr/io.h>        

#include <avr/interrupt.h> 

#include <util/delay.h>    

#include <stdint.h>        

/*

 

*	GLOBAL VARIABLES FOR FREQUENCY MEASUREMENT
*	
 

*	These variables are used by the Input Capture interrupt to store timing data
*	
 */

volatile uint16_t first_capture = 0;   // Timestamp of first rising edge volatile uint16_t second_capture = 0;  // Timestamp of second rising edge volatile uint8_t measurement_ready = 0; // Flag: 1 when measurement is complete volatile uint8_t capture_count = 0;    // Counter for number of captures (0-1)

/*

 

*	FUNCTION PROTOTYPES
*	
 

 */

void init_system(void);                    // Initialize all system components void init_uart(void);                      // Configure UART for 9600 baud void init_timer1_input_capture(void);      // Setup Timer1 for input capture void init_button(void);                    // Configure button with pull-up void uart_send_char(char c);              // Send single character via UART void uart_send_string(const char *str);   // Send string via UART void uart_send_uint32(uint32_t value);    // Send 32-bit number via UART

uint32_t calculate_frequency(void);       // Calculate frequency from captures

void display_frequency(uint32_t freq);    // Format and display frequency void start_frequency_measurement(void);   // Initialize new measurement uint8_t is_button_pressed(void);         // Check button state

void enable_frequency_generator(void);    // Configure frequency generator routing

/*

 * MAIN PROGRAM

 */

int main(void) {

    // Initialize all hardware systems     init_system();

    

    // Configure the frequency generator routing (tries different PE1:PE0 combinations)     enable_frequency_generator();

    

    // Send startup messages to UART

    uart_send_string("=== AT90USB1287 Frequency Measurement Lab ===\r\n");     uart_send_string("Week 12 - Frequency Display\r\n");

    uart_send_string("Press PA0 button to measure frequency on PD4\r\n");     uart_send_string("Ready...\r\n");

    

    /*

*	INITIAL DIAGNOSTICS SECTION
*	
*	Displays current pin states and Timer1 configuration for debugging
*	
     */

    uart_send_string("\r\n=== Initial Diagnostics ===\r\n");

    

    // Check PD4 (Input Capture Pin) initial state     uart_send_string("PD4 (ICP1) initial state: ");     if (PIND & (1 << PD4)) {         uart_send_string("HIGH\r\n");

    } else {

        uart_send_string("LOW\r\n");

    }

    

    // Check button state

    uart_send_string("PA0 (button) state: ");     if (PINA & (1 << PA0)) {

        uart_send_string("HIGH (not pressed)\r\n");

    } else {

        uart_send_string("LOW (pressed)\r\n");

    }

    

    // Verify Timer1 prescaler configuration     uart_send_string("Timer1 prescaler check: ");

    if (TCCR1B & (1 << CS11) && !(TCCR1B & (1 << CS10)) && !(TCCR1B & (1 << CS12))) {         uart_send_string("OK (prescaler = 8)\r\n");

    } else {

        uart_send_string("ERROR - Wrong prescaler!\r\n");

    }

    

    // Check input capture edge configuration     uart_send_string("Input capture edge: ");     if (TCCR1B & (1 << ICES1)) {         uart_send_string("Rising edge\r\n");

    } else {

        uart_send_string("Falling edge\r\n");

    }

    uart_send_string("===========================\r\n\r\n");

    

    // Button state tracking for edge detection     uint8_t button_was_pressed = 0;

    

    /*

*	MAIN PROGRAM LOOP
*	
*	Continuously monitors button presses and performs frequency measurements
*	
     */     while (1) {

        uint8_t button_current = is_button_pressed();

        

        // Detect button press event (rising edge: not pressed -> pressed)         if (button_current && !button_was_pressed) {

            uart_send_string("Button pressed - Starting measurement...\r\n");

            

            // Reset measurement system and start new measurement             start_frequency_measurement();

            

            /*

*	MEASUREMENT TIMEOUT LOOP
*	
*	Wait for Input Capture interrupt to complete measurement
*	
*	Includes 2-second timeout to prevent infinite waiting
*	
             */

            uint16_t timeout_ms = 0;

            const uint16_t MAX_TIMEOUT_MS = 2000; // 2 second timeout

            

            while (!measurement_ready && timeout_ms < MAX_TIMEOUT_MS) {

                _delay_ms(50);        // Wait 50ms                 timeout_ms += 50;     // Increment timeout counter

            }

            

            // Process measurement results             if (measurement_ready) {

                // Measurement successful - calculate and display frequency                 uint32_t frequency = calculate_frequency();                 display_frequency(frequency);

                measurement_ready = 0; // Reset flag for next measurement

            } else {

                // Timeout occurred - no signal detected

                uart_send_string("Timeout - No signal detected on PD4\r\n");                 uart_send_string("Check frequency generator connection\r\n");                 uart_send_string("Current PD4 state: ");                 if (PIND & (1 << PD4)) {                     uart_send_string("HIGH\r\n");

                } else {

                    uart_send_string("LOW\r\n");

                }

            }

            

            uart_send_string("Press PA0 again to measure...\r\n\r\n");

            

            /*

*	BUTTON DEBOUNCING
*	
*	Wait for button release to prevent multiple triggers
*	
             */

            while (is_button_pressed()) {

                _delay_ms(10);

            }

            _delay_ms(100); // Additional debounce delay

        }

        

        button_was_pressed = button_current;

        _delay_ms(10); // Reduce CPU usage in main loop

    }

    

    return 0;

}

/*

*	SYSTEM INITIALIZATION FUNCTIONS
*	
 */

/*

*	Initialize all system components in proper order
*	
 */

void init_system(void) {

    init_uart();                    // Setup UART communication

    init_timer1_input_capture();    // Setup Timer1 for frequency measurement     init_button();                  // Setup button input

    sei();                         // Enable global interrupts (required for Timer1 ISR)

}

/*

*	Configure UART1 for 9600 baud, 8N1 format
*	
*	UART is used for displaying measurement results and debug information
*	
 */

void init_uart(void) {

    // Calculate baud rate register value

    // Formula: UBRR = F_CPU/(16*BAUD) - 1

    // For 8MHz clock and 9600 baud: UBRR = 8000000/(16*9600) - 1 = 51.08 ≈ 51     uint16_t ubrr = 51;

    UBRR1H = (uint8_t)(ubrr >> 8);  // High byte of baud rate

    UBRR1L = (uint8_t)ubrr;         // Low byte of baud rate

    

    // Enable UART transmitter only (we don't need to receive data)

    UCSR1B = (1 << TXEN1);

    

    // Set frame format: 8 data bits, 1 stop bit, no parity (8N1)

    UCSR1C = (1 << UCSZ11) | (1 << UCSZ10);

}

/*

*	Configure Timer1 for Input Capture mode to measure frequency
*	
*	Timer1 is a 16-bit timer with Input Capture capability on PD4 (ICP1)
*	
 */

void init_timer1_input_capture(void) {

    // Configure PD4 (ICP1 pin) as input for frequency signal

    DDRD &= ~(1 << PD4);    // Set PD4 as input (clear bit in Data Direction Register)

    PORTD &= ~(1 << PD4);   // Disable pull-up resistor (signal should be driven externally)

    

    /*

*	Configure Timer1 Control Registers:
*	
*	- TCCR1A = 0: Normal mode (no PWM, just counting)
*	
*	- TCCR1B configures input capture and prescaler
*	
     */

    TCCR1A = 0;  // Normal mode, no output compare pins used

    

    /*

*	TCCR1B configuration:
*	
*	- ICES1 = 1: Input Capture on rising edge
*	
*	- CS11 = 1, CS10 = 0, CS12 = 0: Prescaler = 8 (Timer frequency = 8MHz/8 = 1MHz)     * - ICNC1 = 1: Input Capture Noise Canceler enabled (reduces false triggers)
*	
     */

    TCCR1B = (1 << ICES1) | (1 << CS11) | (1 << ICNC1);

    

    // Enable Input Capture Interrupt (triggers ISR when signal edge detected)

    TIMSK1 = (1 << ICIE1);

    

    // Clear any pending Input Capture interrupt flag

    TIFR1 = (1 << ICF1);

}

/*

*	Configure PA0 as button input with internal pull-up resistor
*	
*	Button is active-low (pressed = 0V, not pressed = 5V via pull-up)
*	
 */

void init_button(void) {

    DDRA &= ~(1 << PA0);    // Set PA0 as input

    PORTA |= (1 << PA0);    // Enable internal pull-up resistor

}

/*

 

*	BUTTON HANDLING FUNCTIONS
*	
 */

/*

*	Check if button is currently pressed
*	
*	Returns: 1 if pressed, 0 if not pressed
*	
*	Note: Button is active-low, so we invert the reading
*	
 */

uint8_t is_button_pressed(void) {

    return !(PINA & (1 << PA0));  // Invert bit: LOW = pressed, HIGH = not pressed

}

/*

*	FREQUENCY MEASUREMENT FUNCTIONS
*	
 

 */

/*

*	Initialize a new frequency measurement cycle
*	
*	Resets all measurement variables and clears timer
*	
 */

void start_frequency_measurement(void) {

    // Disable interrupts during variable reset to prevent race conditions     cli();

    

    // Reset measurement state variables     capture_count = 0;          // No captures yet     measurement_ready = 0;      // Measurement not complete     first_capture = 0;          // Clear first timestamp     second_capture = 0;         // Clear second timestamp

    

    // Reset Timer1 counter and clear interrupt flag

    TCNT1 = 0;                  // Reset timer count to 0

    TIFR1 = (1 << ICF1);       // Clear Input Capture Flag

    

    // Re-enable interrupts     sei();

    

    uart_send_string("Waiting for signal...\r\n"); }

/*

*	Input Capture Interrupt Service Routine
*	
*	This function is automatically called when a rising edge is detected on PD4
*	
*	It captures two consecutive rising edges to measure one complete period
*	
 */

ISR(TIMER1_CAPT_vect) {

    if (capture_count == 0) {

        /*

*	FIRST RISING EDGE
*	
*	Record the timestamp and wait for second edge
*	
         */

        first_capture = ICR1;       // ICR1 contains the timer value when edge occurred         capture_count = 1;          // Indicate we've captured the first edge

    } else if (capture_count == 1) {

        /*

*	SECOND RISING EDGE
*	
*	Record timestamp and signal that measurement is complete
*	
         */

        second_capture = ICR1;      // Capture second timestamp

        measurement_ready = 1;      // Signal main loop that calculation can proceed         capture_count = 0;          // Reset for next measurement cycle

    } }

/*

*	Calculate frequency from captured timestamps
*	
*	Uses the time difference between two rising edges to determine period
*	
*	Returns: Frequency in Hz
*	
 */

uint32_t calculate_frequency(void) {

    uint32_t period_ticks;

    

    /*

*	Calculate period in timer ticks between two rising edges
*	
*	Handle potential timer overflow (16-bit timer wraps from 65535 to 0)
*	
     */

    if (second_capture >= first_capture) {         // Normal case: no timer overflow         period_ticks = second_capture - first_capture;

    } else {

        // Timer overflow occurred between captures         // Calculate: (max_value - first) + second + 1

        period_ticks = (0xFFFFUL - first_capture) + second_capture + 1;

    }

    

    // Prevent division by zero

    if (period_ticks == 0) return 0;

    

    /*

*	Calculate frequency using timer characteristics:
*	
*	- Timer clock frequency = F_CPU / prescaler = 8MHz / 8 = 1MHz
*	
*	- Each tick = 1/1,000,000 seconds = 1 microsecond
*	
*	- Period in seconds = period_ticks / 1,000,000
*	
*	- Frequency = 1 / period_in_seconds = 1,000,000 / period_ticks
*	
     */

    uint32_t frequency = 1000000UL / period_ticks;

    

    return frequency; }

/*

*	Format and display frequency measurement results
*	
*	Automatically chooses Hz or kHz units based on frequency magnitude
*	
*	Also displays raw capture data for debugging purposes
*	
 */

void display_frequency(uint32_t freq) {

    uart_send_string("=== MEASUREMENT RESULT ===\r\n");     uart_send_string("Frequency: ");

    

    if (freq == 0) {

        // Invalid measurement (division by zero or other error)         uart_send_string("0 Hz (Invalid measurement)\r\n");

    } else if (freq >= 1000) {

        /*

*	HIGH FREQUENCY DISPLAY (≥1000 Hz)
*	
*	Show in kHz with decimal places for better readability
*	
         */

        uint32_t khz_whole = freq / 1000;        // Whole kHz part         uint32_t hz_remainder = freq % 1000;     // Remaining Hz part

        

        uart_send_uint32(khz_whole);

        

        if (hz_remainder > 0) {             uart_send_string(".");

            

            // First decimal place (tenths of kHz)             uint32_t decimal_part = hz_remainder / 100;             uart_send_uint32(decimal_part);

            

            // Second decimal place (hundredths of kHz) - only if significant             uint32_t second_decimal = (hz_remainder % 100) / 10;             if (second_decimal > 0 || (hz_remainder % 10) >= 5) {                 uart_send_uint32(second_decimal);

            }

        }

        

        // Show both kHz and exact Hz values         uart_send_string(" kHz (");         uart_send_uint32(freq);         uart_send_string(" Hz)\r\n");

    } else {

        /*

*	LOW FREQUENCY DISPLAY (<1000 Hz)
*	
*	Show in Hz only
*	
         */

        uart_send_uint32(freq);         uart_send_string(" Hz\r\n");

    }

    

    /*

*	DEBUG INFORMATION
*	
*	Display raw capture values and calculated period for troubleshooting
*	
     */

    uart_send_string("Debug - First: ");     uart_send_uint32(first_capture);     uart_send_string(", Second: ");     uart_send_uint32(second_capture);     uart_send_string(", Period: ");

    

    // Recalculate period for display (same logic as in calculate_frequency)     if (second_capture >= first_capture) {         uart_send_uint32(second_capture - first_capture);

    } else {

        uart_send_uint32((0xFFFFUL - first_capture) + second_capture + 1);

    }

    uart_send_string(" ticks\r\n");

    uart_send_string("===========================\r\n");

}

/*

 

*	UART COMMUNICATION FUNCTIONS
*	
 */

/*

*	Send a single character via UART
*	
*	Waits for transmit buffer to be empty before sending
*	
 */

void uart_send_char(char c) {

    // Wait for UART Data Register Empty flag     while (!(UCSR1A & (1 << UDRE1)));

    UDR1 = c;  // Write character to UART data register

}

/*

*	Send a null-terminated string via UART
*	
*	Calls uart_send_char for each character in the string
*	
 */

void uart_send_string(const char *str) {

    while (*str) {                  // Continue until null terminator         uart_send_char(*str++);     // Send character and advance pointer

    } }

/*

*	Convert and send a 32-bit unsigned integer via UART
*	
*	Converts number to ASCII decimal representation
*	
 */

void uart_send_uint32(uint32_t value) {

    char buffer[11]; // Buffer for digits + null terminator (max 10 digits for 32-bit)     char *ptr = buffer + sizeof(buffer) - 1;  // Start at end of buffer

    *ptr = '\0';  // Null terminator

    

    // Handle special case of zero     if (value == 0) {         uart_send_char('0');         return;

    }

    

    /*

*	Convert number to string by extracting digits from right to left
*	
*	This builds the string backwards in the buffer
*	
     */

    while (value != 0) {

        ptr--;                          // Move pointer left

        *ptr = '0' + (value % 10);     // Convert digit to ASCII         value /= 10;                   // Remove processed digit

    }

    

    // Send the resulting string     uart_send_string(ptr); }

/*

*	FREQUENCY GENERATOR CONFIGURATION
*	
 

 */

/*

*	Configure the frequency generator routing using PE1:PE0 device selectors
*	
*	This function tries different combinations to connect the frequency generator
*	
*	to the PD4 input capture pin. The exact combination depends on the lab board design.
*	
 */

void enable_frequency_generator(void) {

    uart_send_string("Configuring frequency generator...\r\n");

    

    /*

*	Configure PE1:PE0 as outputs for device selection
*	
*	These pins control multiplexers or switches that route signals on the lab board
*	
     */

    DDRE |= (1 << PE0) | (1 << PE1);  // Set PE1 and PE0 as outputs

    

    /*

*	TRY CONFIGURATION 1: PE1=0, PE0=1 (binary 01)
*	
*	This is often the configuration for routing frequency generator to ICP1
*	
     */

    PORTE &= ~(1 << PE1);  // Clear PE1 (set to 0)

    PORTE |= (1 << PE0);   // Set PE0 (set to 1)

    

    uart_send_string("Device selector: PE1=0, PE0=1\r\n");

    _delay_ms(100);  // Allow time for signal routing to stabilize

    

    /*

*	TEST FOR SIGNAL ACTIVITY
*	
*	Check if the frequency generator is now connected by looking for signal transitions
*	
     */

    uart_send_string("Checking for signal on PD4...\r\n");     uint8_t last_state = PIND & (1 << PD4);  // Read initial pin state     uint8_t changes = 0;                      // Count signal transitions

    

    // Monitor PD4 for 500ms looking for signal changes     for (uint16_t i = 0; i < 50; i++) {         uint8_t current_state = PIND & (1 << PD4);         if (current_state != last_state) {             changes++;             if (changes == 1) {

                uart_send_string("Signal detected!\r\n");                 break;  // Found activity, configuration is correct

            }

        }

        last_state = current_state;

        _delay_ms(10);

    }

    

    /*

*	TRY CONFIGURATION 2: PE1=1, PE0=0 (binary 10)
*	
*	If first configuration didn't work, try this alternative
*	
     */

    if (changes == 0) {

        uart_send_string("No signal detected, trying PE1=1, PE0=0...\r\n");

        PORTE |= (1 << PE1);   // Set PE1 (set to 1)

        PORTE &= ~(1 << PE0);  // Clear PE0 (set to 0)

        _delay_ms(100);

        

        // Test again for signal activity         for (uint16_t i = 0; i < 50; i++) {             uint8_t current_state = PIND & (1 << PD4);             if (current_state != last_state) {                 changes++;                 if (changes == 1) {

                    uart_send_string("Signal detected with PE1=1, PE0=0!\r\n");                     break;

                }

            }

            last_state = current_state;

            _delay_ms(10);

        }

    }

    

    /*

*	TRY CONFIGURATION 3: PE1=1, PE0=1 (binary 11)
*	
*	Final attempt if previous configurations didn't work
*	
     */

    if (changes == 0) {

        uart_send_string("Trying PE1=1, PE0=1...\r\n");

        PORTE |= (1 << PE1) | (1 << PE0);  // Set both pins high

        _delay_ms(100);

    }

    

    uart_send_string("Frequency generator setup complete\r\n");

}


