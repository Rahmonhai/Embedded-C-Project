/* Libraries */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <math.h>

/* Macros */
#define bitSet(reg, n)    (reg |=  (1 << n))
#define bitClear(reg, n)  (reg &= ~(1 << n))
#define bitCheck(reg, n)  (reg &   (1 << n))

#define F_CPU 16000000UL
#define Baud 9600

#define MAP_LINEAR(x, in_min, in_max, out_min, out_max) \
    (((x) - (in_min)) * ((out_max) - (out_min)) / ((in_max) - (in_min)) + (out_min))

#define pin_trigger PB--
#define pin_echo PB--



/* ADC chanels*/
#define CH_SOIL  0   //A0 - pointimeter (simulated soil)
#define CH_LIGHT 1   // A1 - LDR (photoresistor) 
#define CH_TEMP  2   // A2 - thermistor (temperature sensor)

/* Thresholds - tune these on real hardware*/
#define SOIL_DRY 300
#define SOIL_HOT 700






/* ── Function prototypes ── */
void  adc_init(void);
uint16_t adc_read(uint8_t ch);
void  usart_init(unsigned long baud);
void  usart_send_byte(unsigned char data);
void  usart_send_string(const char *pstr);
void  usart_read(char *pbuf_usart);
void usart_flush();
uint16_t GetWaterLevel(void);
uint16_t GeTemperatureLevel(void);

void  timer0_pwm_init(void);   //fast PWM for motor control
void  timer1_ctc_init(void);   //CTC mode for ultrasonic sensor timing (1ms timebase)
void  timer2_pwm_init(void);  //fast PWM for servo control phase-correct-servo mode
void  capture_init(void);     // Input Capture for ultrasonic sensor echo timing
void  interrupt_init(void);   // Enable global interrupts and specific interrupt sources INT0 + INT1
void  comparator_init(void);    // Analog Comparator for overheat detection (comparing thermistor voltage to reference)
void  motor_set(uint8_t duty);
void  servo_set_angle(uint8_t deg);
void  hcsr04_trigger(void);


/* Global variables shared with ISRs */
volatile uint32_t g_millis       = 0; // Milliseconds since start (updated by Timer1 ISR)
volatile uint16_t g_echo_start   = 0; // Timer value at echo start (Input Capture)
volatile uint16_t g_echo_width   = 0;   // Echo pulse width in timer ticks (calculated in Input Capture ISR)
volatile uint8_t  g_echo_raising = 0; // Flag to indicate if we're waiting for rising edge (1) or falling edge (0) in ultrasonic sensor timing
volatile uint8_t  g_soil_done    = 0; // Flag to indicate soil moisture reading is ready
volatile uint8_t  flag_water     = 0;// Flag to indicate water pump should be turned on (INT0 pressed)
volatile uint8_t  flag_manual    = 0; // Flag to indicate manual mode is active (INT1 toggled mode)
volatile uint8_t  flag_low       = 0; //comparator low-water
volatile uint8_t  rx_flag        = 0; // Flag to indicate new USART data received
volatile char     rx_data        = 0; // Variable to store received USART data (e.g., for manual control commands)

/* ADC initialisation */
void adc_init(void)
{
    ADMUX  = (1 << REFS0); // AVCC with external capacitor at AREF pin
    ADCSRA = (1 << ADEN) | //Writing this bit to one enables the ADC. By writing it to zero, the ADC is turned off. 
    (1 << ADPS2) | (1 << ADPS1);  //Prescaler Select Bits (64)
}

/* ADC Read function 

Settings for
    - Reference voltage
    - ADC channel select
    - ADC conversion begin

*/
uint16_t adc_read(uint8_t ch)
{
    ADMUX = (ADMUX & 0xF0) //Persevers Voltage Reference
            | (ch & 0x0F); // Enables selected ADC Channel (ch must be between 0-5)
    bitSet(ADCSRA, ADSC); // ADC Start Conversion
    while (bitCheck(ADCSRA, ADSC)) // Checks for ADC conversion to begin
        ;
    return ADC; // Returns ADC value
}

/* USART Initalization

Settings for
    - UBRR
    - RX & TX Enable
    - Size of data bits
    
*/
void usart_init(unsigned long baud)
{
    uint16_t ubrr = (F_CPU / (16 * baud)) - 1; //Asynchronous normal mode // ubrr = 103
    UBRR0H = (unsigned char)(ubrr >> 8); // The UBRRnH contains the four most significant bits
    UBRR0L = (unsigned char) ubrr; //UBRRnL contains the eight least significant bits
    UCSR0B = (1 << RXEN0) 
           | (1 << TXEN0) // Receiver Enable & Transmitter Enable
           | (1 << RXCIE0); // RX Complete Interrupt Enable
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // number of data bits (8-bit)
}


/* USART SendByte | SendString | ReadString

Settings for
    - Serial monitor --> MCU
    - MCU --> Serial monitor
    
*/
void usart_send_byte(unsigned char data)
{
  while (!(UCSR0A & (1 << UDRE0))); // wait until the transmit buffer is empty (UDRE0 is set)
  
  UDR0 = data; // Store string inside UDR0
}

void usart_send_string(const char *pstr)
{
  while (*pstr != '\0'){
    usart_send_byte(*pstr);
    pstr++; // Increment by 1 to read whole string
  }
}

/* send an interger as text, using itoa form <stdlib.h> */
void usart_send_int(int num)
{
  char buffer[10]; // Buffer to hold the string representation of the integer
  itoa(num, buffer, 10); // Convert integer to string (base 10)
  usart_send_string(buffer); // Send the string over USART
}

/* USART recieve-complete interupt: store the byte, set a flag*/
ISR(USART_RX_vect)
{
  rx_data = UDR0; // Store received byte in global variable
  rx_flag = 1; // Set flag to indicate new data received
}


void usart_flush()
{
  char dummy;
  while (bitCheck(UCSR0A, RXC0)){ // Check for data
    dummy = UDR0; // Clearing
  }
}




/////////////////* TIMERS */////////////////

/*Timer0 Fast PWM (TOP = 0xFF) DC Motor on D5 (OC0B)*/

/* Settings for
    - 
    - 
    
*/
void timer0_pwm_init(void)
{
    bitSet(DDRD, PD5); // Set OC0B (PD5) as output for motor control
    TCCR0A = (1 << WGM00) | (1 << WGM01) | // Fast PWM mode with TOP = 0xFF
             (1 << COM0B1); // Clear OC0B on compare match, set at BOTTOM (non-inverting mode)
    TCCR0B = (1 << CS01) | (1 << CS00); // Prescaler 64 for ~976.56 Hz PWM frequency
    OCR0B = 0; // Start with 0% duty cycle (motor off)
}

/* Settings for
    - 
    - 
    
*/
void motor_set(uint8_t duty)
{
    OCR0B = duty; // Set duty cycle (0-255)
}



/*Timer1 PWM*/
/* Settings for
    - PWM Clear timer on compare match (CTC)
    - Top Value
    - Mode
    - Prescalar
    
*/
void timer1_ctc_init(void)
{
bitSet(DDRB, PB1); // OC1A

TCCR1B |= (1 << WGM12); // PWM CTC, mode 4, with TOP = OCR1A
TCCR1A |= (1 << COM1A0); // Toggle OC1A on compare match

OCR1A = 65535; // 2^16 bits

TCCR1B |= (1 << CS11) | (1 << CS10); // Prescalar 64 for 250khz
}


/////////////////* Ultrasonic water-level sensor */////////////////
#define DRY 200
#define WET 700
uint16_t GetWaterLevel(uint8_t ch){
    
    adc_init();

    uint16_t adc_value = adc_read(ch); 
       
    if (adc_value <= DRY) adc_value = DRY; //Clamping
    if (adc_value >= WET) adc_value = WET; //Clamping

    return (uint16_t)((DRY - adc_value) * 100 / (DRY - WET)); //Map to percentage
}
/////////////////* Themistor reading */////////////////

#define V_REF 5.0
#define R_FIXED 10000.0
#define R0 10000.0
#define T0_KELVIN 298.15
#define B_COEFF 3950.0


#define T_MIN -10.0
#define T_MAX 60.0

uint16_t GetTemperatureLevel(uint8_t ch){

    uint16_t adc_value = adc_read(ch);
    float v_adc = (float)adc_value * V_REF / 1023.0;
    if (v_adc <= 0.0 || v_adc >= V_REF) return T_MIN;
    float r_th = R_FIXED * v_adc / (V_REF - v_adc);
    float t_k  = 1.0 / (1.0 / T0_KELVIN + log(r_th / R0) / B_COEFF);
    return t_k - 273.15;
}




int main(void)
{
    uint16_t soil, light, temp, tank;
    uint32_t last = 0;
 
    /* initialise everything */
    adc_init();
    usart_init(9600);
    timer0_pwm_init();
    timer1_ctc_init();
    timer2_pwm_init();
    capture_init();
    interrupts_init();
    comparator_init();
    bitSet(DDRB, PB1);                           /* D9 trigger out  */
 
    usart_flush();
    sei();                                       /* enable interrupts */
    usart_send_string("Greenhouse online\r\n");
}