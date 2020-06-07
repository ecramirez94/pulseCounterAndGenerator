//#define BOS_INTERRUPT_PIN      2  // INT0 on PD2, physical pin 4
//#define EOS_INTERRUPT_PIN      3  // INT1 on PD3, physical pin 5
//#define COUNTER_PIN            5  // T1 on PD5,   physical pin 11
//#define TWO_HUNDRED_HZ_PIN     6  // OC0A on PD6, physical pin 12
//#define RAND_PULSER_PIN        11 // PB3, physical pin 17

// ===== Global Variables =====
uint8_t pulseWindowTime = 0;
uint8_t variableTime = 0;
volatile uint8_t prePulseTime = 0;
uint8_t postPulseTime = 0;
// ----- Random Pulse Characteristics -----
/*
 * Use: (4 * pulse variable) to determine time of event in microseconds.
 */
uint8_t prePulseMin = 1;  // 4uS
uint8_t pulseTime = 2; // 8uS
uint8_t postPulseMin = 1; // 4uS

// ===== State Machine =====
// ----- Constant Enumerations -----
#define IDL 0 // The word IDLE is reserved by some other library so IDL is used instead
#define BOS 1
#define EOS 2
// ----- Variables -----
uint8_t whichSignal = 0;
// ----- Booleans -----
bool firstCue = true;
volatile bool prePulse = true;
volatile bool genNewPulseValues = false;


void setup() 
{
  PRR = (1<<PRTWI) | (1<<PRSPI); // Disable some unused peripherals to reduce power consumption.
  MCUCR = (1<<PUD);

  DDRD = (0<<DDD2) | (0<<DDD3) | (0<<DDD5) | (1<<DDD6); // Set BOS_INTERRUPT_PIN (PD2) , EOS_INTERRUPT_PIN (PD3), and COUNTER_PIN (PD5) as an INPUT; TWO_HUNDRED_HZ_PIN (PD6) as an OUTPUT
  
  DDRB = (1<<DDB3) | (1<<DDB4); // Set RAND_PULSER_REF_PIN (PB3) and RAND_PULSER_PIN (PB4) as an OUTPUT
  
  EICRA = (1<<ISC11) | (1<<ISC10) | (1<<ISC01) | (1<<ISC00); // Rising edge of INT1/0 generates the interrupts
  EIMSK = (1<<INT1) | (1<<INT0);  // Enable external interrupt request 1 and 0.

  counter_INIT();
  twoHundredHzWaveGenerator_INIT();

  Serial.begin(9600);
} 

void loop() 
{
  switch (whichSignal)
  {
    case BOS:
      turnOff200HzGenerator();
      startCounting();
      startRandomPulser();

      firstCue = false; // After first BOS cue, device is synchronized
      whichSignal = IDL;
    break;

    case EOS:
      if(!firstCue) // Semaphore to properly synchonize device with BOS/EOS pattern
      {
        turnOn200HzGenerator(); // Generate 200Hz signal in between EOS and next BOS
        stopCounting(); 
        stopRandomPulser();
        reportNumberOfPulses(); // Comment out and uncomment 'SerialEvent' function to switch to query based data transfer
        
        whichSignal = IDL;
      }
    break;

    case IDL:
      if (genNewPulseValues)
      {
        cli();  // Disable global interrupts to not interrupt the calculation of new values
        calculatePrePulseTime();
        calculatePostPulseTime();
        genNewPulseValues = false;
        sei();  // Reenable global interrupts 
      }
    break;
  }
}


// ################################### Interrupt Service Routines ##################################################################################

ISR(INT0_vect)  // BOS Interrupt Service Routine (ISR)
{
  whichSignal = BOS;
}

ISR(INT1_vect)  // EOS Interrupt Service Routine (ISR)
{
  whichSignal = EOS;
}

// ################################### End Interrupt Service Routines #################################################################################


// ####################################################### Random Pulse Generator #####################################################################

ISR(TIMER2_COMPA_vect)  // Pulse frequency generator
{
  PORTB = (0<<PB3); // Drive output LOW to end pulse

  OCR2B = prePulseTime;
  prePulse = true;
}

ISR(TIMER2_COMPB_vect)  // Pulse shaper
{
  if (prePulse) // OCR2B interrupt #1
  {
    PORTB = (1<<PB3); // Drive output HIGH to begin pulse
    OCR2B = postPulseTime;
    prePulse = false; 
  } 
  else  // OCR2B interrupt #2
  {
    PORTB = (0<<PB3); // Drive output LOW to end pulse
    genNewPulseValues = true; // Generate new values while waiting for next event
    prePulse = true;
  }
}

void startRandomPulser(void)
{
  TCCR2A = (1<<WGM21); // Enable Compare Match (CTC) mode with OCR2A
  PORTB = (0<<PB3); // Drive output pin LOW so first pulse is guaranteed to be a rising edge.

  calculatePulseWindowTime();  // Determine how many pulses will be generated this spill
  calculatePrePulseTime();  // Fill repective global variables with initial values.
  calculatePostPulseTime();

  TCNT2 = 0;  // Reset counter register
  OCR2B = prePulseTime; // Load prePulseTime value into OCR2B
  TIMSK2 = (1<<OCIE2A) | (1<<OCIE2B); // Enable interrupts
  TCCR2B = (1<<CS22); // Pre-scale clock by 64 and start the counter. ie. start generating random pulses.
}

void stopRandomPulser(void)
{ 
  TCCR2B = (0<<CS22); // Stop the counter. ie. stop generating random pulses.
  PORTB = (0<<PB3); // Drive output pin LOW between EOS and next BOS.
}

void calculatePulseWindowTime(void)
{
  /*  
   * First, calculate total 'Random' number of pulses to generate this run. 
   * Want between 1kHz and 5kHz worth of pulses randomly spaced apart for 4 seconds,
   * thus between 4k and 20k pulses per spill.
   * 
   * Use the waveform equation below to determine the OCR2A values that will generate the frequency:
   *                                        f_OCR2A = f_clk / (2 * N * (1 + OCR2A))
   * 
   * Where: 
   *        - f_OC2A is the frequency of a compare match  
   *        - f_clk is the CPU clock frequency (16MHz)                                
   *        - N is the CPU clock prescale factor (1, 8, 32, 64, 128, 256, 1024)
   *        - OCR2A is an 8 bit value in the Compare Match register.
   *        
   * We find that when the clock pre-scaler is N = 32, we can generate frequencies between 1KHz and 125KHz, thus
   * it can easily be used to generate the 1KHz and 5KHz frequencies of interest. Since there are two pulse windows per cycle, 
   * N must equal 64 to compensate for the factor of two.
   * 
   * The OCR2A values are:
   * 1KHz = 249
   * 5KHz = 49
   */
   
  pulseWindowTime = uint8_t(random(49, 249)); // Randomly select OCR2A value to generate frequencies between 5KHz and 1KHz.
  OCR2A = pulseWindowTime; // Put new single pulse time into OCR2A to set frequency. ie. number of pulses generated this spill
 
  // Since pulseWindowTime = prePulseTime + pulseTime + postPulseTime
  variableTime = pulseWindowTime - pulseTime; // The remaining time is available to vary.
}

void calculatePrePulseTime(void)
{  
  prePulseTime = random(prePulseMin, (variableTime - postPulseMin));
}

void calculatePostPulseTime(void)
{
  /*
   * postPulseTime is actually pulseWindowTime - pulseTime - prePulseTime
   * Since the counter is always incrementing, gets you the OCR2B value required to mark the beginning of
   * the postPulseTime (the OCR2A CTC match handles the end). ie. The relative time since the beginning
   * of the pulse window.
   */
  postPulseTime = prePulseTime + pulseTime;
}
// ############################################# End Random Pulse Generator ##############################################################################


// ############################################## Counter ################################################################################################

void counter_INIT(void)
{
  TCCR1A = (0<<WGM11) | (0<<WGM10); // Set all WGM1x modes to 0 to ensure Normal mode. 
  // TCCR1B handles clock selection.
  TCCR1B = (0<<WGM12) | (0<<WGM13) | (1<<CS12) | (1<<CS11) | (1<<CS10); // External clk source, rising edge, on T1/PD5/Physical pin 11/Arduino pin 5
}

void startCounting(void)
{ 
  // ** First Reset Counter **
  TCNT1 = 0;  // Reset TCNT1 to zero.

  // ** Then Start Counting **
  TCCR1B = (1<<CS12) | (1<<CS11) | (1<<CS10);  // Connect external clock source from counter, effectively turning on Timer/Counter1.
}

void stopCounting(void)
{
  /* 
   * Disconnect clock source from counter, effectively turning off Timer/Counter1. 
   * Cannot simply disable counter from Power Reduction Register (PRR) since that would
   * make the data register inaccessable and thus wouldn't be able to collect the data. 
   */
  TCCR1B = (0<<CS12) | (0<<CS11) | (0<<CS10); 
}

// ############################################## End Counter #################################################################################################


// #################################################### Start 200 Hz Generator ###################################################################################
void twoHundredHzWaveGenerator_INIT(void)
{
  /*
   * See pages 93 - 109 in the ATMega328P datasheet for more details on configuration and control of the timer/counter.
   * http://ww1.microchip.com/downloads/en/DeviceDoc/ATmega48A-PA-88A-PA-168A-PA-328-P-DS-DS40002061A.pdf
   * 
   * This counter is setup to generate a 200.3Hz signal but can be configured to generate any frequency up to f_clk/2. Which in this case is 16MHz/2 = 8MHz.
   * The waveform frequency is defined by:
   *                                        f_OC0A = f_clk / (2 * N * (1 + OCR0A))
   * 
   * Where: 
   *        - f_OC0A is the frequency output on PD6/Arduino pin 6/Physical pin 12. 
   *        - f_clk is the CPU clock frequency (16MHz)                                
   *        - N is the CPU clock prescale factor (1, 8, 64, 256, 1024)
   *        - OCR0A is an 8 bit value in the Compare Match register.
   */
  
  // Set up timer to Compare Match Mode (CTC) and prescale clock
  TCCR0A = (1<<COM0A0) | (1<<WGM01); // Toggle OC0A on Compare Match. Enable Compare Match (CTC) mode.

  TCCR0B = (1<<CS02) | (1<<CS00); // Prescale clock (16MHz) by 1024.

  // Set OCR0A to 38 to generate 200.3Hz signal.
  OCR0A = 38; // Write Compare Match value to Compare Match A register. Use formula above to calculate.
}

void turnOn200HzGenerator(void)
{
  TCCR0A = (1<<COM0A0) | (1<<WGM01); // Reconnect OC0A and enable CTC mode.
  TCCR0B = (1<<CS02) | (1<<CS00); // Start the 200Hz generator
}

void turnOff200HzGenerator(void)
{
  TCCR0B = (0<<CS02) | (0<<CS00);  // Turn off the 200Hz generator during spill
  TCCR0A = (0<<COM0A0); // Disconnect OC0A in order to drive PD6 low. That way, it'll always be a rising edge when restarting after EOS.
  PORTD = (0<<PD6); // Drive pin LOW 
}

// ##################################################### End 200 Hz Generator ######################################################################################


// ##################################################### Serial Communication ######################################################################################
void reportNumberOfPulses(void) // Asynchronous data transfer
{ 
  // Read pulses from Timer/Counter1
  uint16_t data = 0;
  data = TCNT1;  // Read data from TCNT1 registers.

  Serial.print(data);
  Serial.print('\n');
}

void serialEvent(void) // Synchronous/Query based data transfer. Automatically called from loop() in series with the Switch statement
{
  String command = Serial.readStringUntil('\n');  // Read until newline char. The newline char is truncated from String.

  if (command.equals(F("GET_DATA"))) // Test received command
  {
    // Read pulses from Timer/Counter1
    uint16_t data = 0;
    data = TCNT1;  // Read data from TCNT1 registers.
  
    Serial.print(data);
    Serial.print('\n');
  }
}

// ################################################### End Serial Communication #######################################################################################
