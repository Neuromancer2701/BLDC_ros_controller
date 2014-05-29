// This file has been prepared for Doxygen automatic documentation generation.
/*! \file ********************************************************************
*
* Atmel Corporation
*
* - File              : main.c
* - Compiler          : IAR EWAAVR 2.28a/3.10c
*
* - Support mail      : avr@atmel.com
*
* - Supported devices : ATmega48/88/168
*
* - AppNote           : AVR444 - Sensorless control of three-phase brushless
*                       DC motors with ATmega48.
*
* - Description       : Example of how to use the ATmega48 for sensorless
*                       control of a three phase brushless DC motor.
*
* $Revision: 1.1 $
* $Date: Monday, October 10, 2005 11:15:46 UTC $
*****************************************************************************/
#include "Arduino.h"
#include "BLDC.h"
#include <avr/wdt.h>

#define SYSTEM_FREQUENCY        16000000	//! System clock frequecy. Used to calculate PWM TOP value.
#define PWM_BASE_FREQUENCY      20000	//! PWM base frequency. Used to calculate PWM TOP value.
#define PWM_TOP_VALUE           (SYSTEM_FREQUENCY / PWM_BASE_FREQUENCY / 2)	 //! PWM TOP value. Automatically calculated to give desired \ref PWM_BASE_FREQUENCY.


#define FALSE     0 //! Boolean FALSE value.


#define TRUE      (!FALSE)	//! Boolean TRUE expression. Can be used both for test and assignment.


#define UL    PINB5	//! Port pin connected to phase U, low side enable switch.  Arduino Pin 13
#define UH    PINB4	//! Port pin connected to phase U, high side enable switch. Arduino Pin 12 

#define VL    PINB3	//! Port pin connected to phase V, high side enable switch. Arduino Pin 11
#define VH    PINB2	//! Port pin connected to phase V, high side enable switch. Arduino Pin 10

#define WL    PINB1	//! Port pin connected to phase W, low side enable switch.	Arduino Pin 9
#define WH    PINB0	//! Port pin connected to phase W, high side enable switch. Arduino Pin 8

#define CW    0	//! Clockwise rotation flag. Used only in macros.
#define CCW   1	//! Counterclockwise rotation flag. Used only in macros.

/*! Direction of rotation. Set to either CW or CCW for
 * clockwise and counterclockwise respectively.
 */
#define DIRECTION_OF_ROTATION     CCW


#define DRIVE_PATTERN_STEP1_CCW      ((1 << UL) | (1 << VH))	//! Drive pattern for commutation step 1, CCW rotation.
#define DRIVE_PATTERN_STEP2_CCW      ((1 << UL) | (1 << WH))	//! Drive pattern for commutation step 2, CCW rotation.
#define DRIVE_PATTERN_STEP3_CCW      ((1 << VL) | (1 << WH))	//! Drive pattern for commutation step 3, CCW rotation.
#define DRIVE_PATTERN_STEP4_CCW      ((1 << VL) | (1 << UH))	//! Drive pattern for commutation step 4, CCW rotation.
#define DRIVE_PATTERN_STEP5_CCW      ((1 << WL) | (1 << UH))	//! Drive pattern for commutation step 5, CCW rotation.
#define DRIVE_PATTERN_STEP6_CCW      ((1 << WL) | (1 << VH))	//! Drive pattern for commutation step 6, CCW rotation.



#define DRIVE_PATTERN_STEP1_CW      ((1 << VH) | (1 << WL))		//! Drive pattern for commutation step 1, CW rotation.
#define DRIVE_PATTERN_STEP2_CW      ((1 << UH) | (1 << WL))		//! Drive pattern for commutation step 2, CW rotation.
#define DRIVE_PATTERN_STEP3_CW      ((1 << UH) | (1 << VL))		//! Drive pattern for commutation step 3, CW rotation.
#define DRIVE_PATTERN_STEP4_CW      ((1 << WH) | (1 << VL))		//! Drive pattern for commutation step 4, CW rotation.
#define DRIVE_PATTERN_STEP5_CW      ((1 << WH) | (1 << UL))		//! Drive pattern for commutation step 5, CW rotation.
#define DRIVE_PATTERN_STEP6_CW      ((1 << VH) | (1 << UL))		//! Drive pattern for commutation step 6, CW rotation.


#define EDGE_FALLING          1	//! Zero crossing polarity flag value for falling zero crossing.
#define EDGE_RISING           0	//! Zero crossing polarity flag value for rinsing zero crossing.


#define DRIVE_PORT  PORTB	//! PORT register for drive pattern output.


#define DRIVE_DDR   DDRB	//! Data direction register for drive pattern output.


#define ADC_MUX_U           0x3	//! ADC multiplexer selection for channel U sampling.
#define ADC_MUX_V           0x2	//! ADC multiplexer selection for channel V sampling.
#define ADC_MUX_W           0x1	//! ADC multiplexer selection for channel W sampling.
#define ADC_MUX_CURRENT     0x0	//! ADC multiplexer selection for current sampling.
#define ADC_MUX_REF_VOLTAGE 0x4	//! ADC multiplexer selection for reference voltage sampling.


#define ADC_REF_CHANNEL                 ((0 << REFS1) | (0 << REFS0))	//! ADC reference channel selection.
#define ADC_RES_ALIGNMENT_BEMF          (1 << ADLAR)	//! ADC result alignment for BEMF measurement.
#define ADC_RES_ALIGNMENT_SPEED_REF     (1 << ADLAR)	//! ADC result alignment for speed reference measurement.


#define ADC_RES_ALIGNMENT_CURRENT       (1 << ADLAR)	//! ADC result alignment for CURRENT measurement.
#define ADC_RES_ALIGNMENT_REF_VOLTAGE   (1 << ADLAR)	//! ADC result alignment for reference voltage measurement.


#define ADMUX_U             (ADC_REF_CHANNEL | ADC_RES_ALIGNMENT_BEMF | ADC_MUX_U)	//! ADMUX register value for channel U sampling.
#define ADMUX_V             (ADC_REF_CHANNEL | ADC_RES_ALIGNMENT_BEMF | ADC_MUX_V)	//! ADMUX register value for channel V sampling.
#define ADMUX_W             (ADC_REF_CHANNEL | ADC_RES_ALIGNMENT_BEMF | ADC_MUX_W)	//! ADMUX register value for channel W sampling.
#define ADMUX_CURRENT       (ADC_REF_CHANNEL | ADC_RES_ALIGNMENT_CURRENT | ADC_MUX_CURRENT)	//! ADMUX register value for current sampling.
#define ADMUX_REF_VOLTAGE   (ADC_REF_CHANNEL | ADC_RES_ALIGNMENT_REF_VOLTAGE | ADC_MUX_REF_VOLTAGE)	//! ADMUX register value for reference voltage sampling.

#define ADC_PRESCALER_8     ((0 << ADPS2) | (1 << ADPS1) | (1 << ADPS0))	//! ADC clock prescaled by 8 value.
#define ADC_PRESCALER_16     ((1 << ADPS2) | (0 << ADPS1) | (0 << ADPS0))	//! ADC clock prescaled by 8 value.
#define ADC_PRESCALER       ADC_PRESCALER_8	//! ADC prescaler used.
#define ADC_TRIGGER_SOURCE  ((1 << ADTS2) | (0 << ADTS1) | (0 << ADTS0))	//! ADC trigger source.

/*! Number of commutations performed during startup. Also specifies
 *  size of \ref startupDelays. Do not change without also changing
 *  \ref MakeTables().
 */
#define STARTUP_NUM_COMMUTATIONS  8
#define STARTUP_DELAY_MULTIPLIER  100	//! Startup delays are given in milliseconds times STARTUP_DELAY_MULTIPLIER.
#define STARTUP_LOCK_DELAY        10000 //Number of milliseconds to lock rotor in first commutation step before the timed startup sequence is initiated.

#define MAX_RESTART_ATTEMPTS    10	//! The maximum number of restart attempts without external action when stall is detected.
#define ZC_DETECTION_HOLDOFF_TIME_US (filteredTimeSinceCommutation / 2)	//! Holdoff time where zero-cross detection is disabled after commutation.
#define SET_PWM_COMPARE_VALUE(compareValue)          (OCR0B = compareValue)	//! Macro that sets a new duty cycle by changing the PWM compare value.

#define CLEAR_ALL_TIMER2_INT_FLAGS    	(TIFR2 = TIFR2)	//! Macro that clears all Timer/counter2 interrupt flags.
#define DISABLE_ALL_TIMER2_INTS       	(TIMSK2 = 0)	//! Macro that disables all Timer/Counter2 interrupts.
#define SET_TIMER2_INT_ZC_DETECTION		(TIMSK2 = (1 << TOIE2))	//! Macro that enables Timer/Counter2 interrupt where zero crossings are detected.

#define CLEAR_ALL_TIMER1_INT_FLAGS    	(TIFR1 = TIFR1)	//! Macro that clears all Timer/Counter1 interrupt flags.
#define DISABLE_ALL_TIMER1_INTS       	(TIMSK1 = 0)	//! Macro that disables all Timer/Counter1 interrupts.
#define SET_TIMER1_INT_COMMUTATION    	(TIMSK1 = (1 << OCIE1A))	//! Macro that enable Timer/Counter1 interrupt responsible for commutation.
#define SET_TIMER1_INT_HOLDOFF        	(TIMSK1 = (1 << OCIE1B))	//! Macro that enables Timer/Counter1 interrupt responsible for enabling ADC sampling after ADC holdoff period.

#define EXTERNAL_REF_VOLTAGE      	  	((4930UL * 10) / 43)	//! External reference voltage in milliVolts.
#define SHUNT_RESISTANCE          		1	//! Current measurement shunt value in milliOhm.
#define ADC_RESOLUTION   				256	//! The ADC resolution used.


#define TICKS_PER_SECOND    1000000UL //The number of Timer/Counter1 ticks per second. Equals MCU clock frequency / Timer/counter1 prescaler.


#define TICKS_PER_MINUTE    (TICKS_PER_SECOND * 60)	//! The number of Timer/Counter1 ticks per minute.

/*! \brief Input value relative gain in IIR filter.
 *
 *  This value specifies the relative gain given to the input value in the IIR filter.
 *  The value should be an integer. Make sure that COEFF_A + COEFF_B is a power of 2,
 *  to avoid instability.
 */
#define COMMUTATION_TIMING_IIR_COEFF_A      1

/*! \brief Relative feedback gain in IIR filter.
 *
 *  This value specifies the relative feedback gain given to the input value in the IIR filter.
 *  The value should be an integer. Make sure that COEFF_A + COEFF_B is a power of 2,
 *  to avoid instability.
 */
#define COMMUTATION_TIMING_IIR_COEFF_B      3


#define CURRENT_LIMITER_START         2500	//! The current value in milliAmpere where the current limiter starts to kick in.
#define CURRENT_LIMITER_CRITICAL      3000	//! The current value in milliAmpere where the motor should be shut down.

/*!
 *  Uncomment the following line if the analog comparator should be used to
 *  detect overcurrent.
 */
//#define ANALOG_COMPARATOR_ENABLE

/*!
 *  The maximum duty cycles is decreased by this number of PWM duty cycle steps
 *  per milliAmpere current consumption over \ref CURRENT_LIMITER_START.
 */
#define CURRENT_LIMITER_FACTOR   (1 / 5)
#define DISABLE_DRIVING          (DRIVE_PORT = 0x00)	//! Macro that cuts all power to the motor.
#define STARTUP_PWM_COMPARE_VALUE  130	//! PWM compare value used during startup.
#define ADC_ZC_THRESHOLD 0x98	//! Zero-cross threshold.

/*!
 *  The number of milliseconds to subtract from the zero-crossing to commutation time.
 *  Used to compensate for zero-cross sample frequency.
 */
#define COMMUTATION_CORRECTION 50
#define MIN_PWM_COMPARE_VALUE    90	//! The minimum allowed PWM compare value.
#define MAX_PWM_COMPARE_VALUE    200	//! The maximum allowed PWM compare value.
#define MIN_SPEED 3000UL	//! The minimum allowed speed. (Only has effect when closed loop speed control is used)
#define MAX_SPEED 8000UL	//! The maximum allowed speed. (Only has effect when closed loop speed control is used)
#define P_REG_K_P 64	//! P-regulator proportional gain.
#define P_REG_SCALING 65536	//! P-regulator scaling factor. The result is divided by this number.
#define MAX_COMMUTATION_STEPS 6
#define MIN_COMMUTATION_STEPS 0

#define MAX_ADC_DATA 5
#define MIN_ADC_DATA 0

static void ResetHandler(void);
static void InitPorts(void);
static void InitTimers(void);
static void InitADC(void);
static void InitAnalogComparator(void);
static void WatchdogTimerEnable(void);
static void MakeTables(void);
static void StartMotor(void);
static void PWMControl(void);
static void StartupDelay(unsigned int delay);
static unsigned long CalculateSpeed();
static unsigned int CalculateCurrent();
static signed int SpeedControl(void);
static unsigned char CurrentControl(void);


volatile unsigned long speedSetpoint = 0;
unsigned char driveTable[MAX_COMMUTATION_STEPS];			//! Array of power stage enable signals for each commutation step.
unsigned char AdcData[MAX_ADC_DATA];


BLDC::BLDC()
{
	speed = 0;
}

BLDC::~BLDC()
{

}

void BLDC::init()
{
  ResetHandler();
  InitPorts();
  InitTimers();
  InitADC();
  MakeTables();
  InitAnalogComparator();
  
  setSpeed( 50 );
  // Run startup procedure.
  StartMotor();

  // Turn on watchdog for stall-detection.
  WatchdogTimerEnable();
  sei();

}
void BLDC::setSpeed(unsigned long _speed)
{
 speedSetpoint = speed = _speed;
}

void BLDC::Control()
{
    PWMControl();
}

void BLDC::SetCommutationState(unsigned char state)
{
	if( (state >= MIN_COMMUTATION_STEPS) && (state < MAX_COMMUTATION_STEPS)
	{
		DRIVE_PORT = driveTable[state];
	}
}
unsigned char BLDC::AnalogData(unsigned char mux)
{
	unsigned char data = 0x00;
	if( (mux >= MIN_ADC_DATA) && (mux < MAX_ADC_DATA)
	{
		data = AdcData[mux];
	}
	
	return data;
}




//! Array of ADC channel selections for each commutation step.
unsigned char ADMUXTable[6];

//! Array holding the intercommutation delays used during startup.
unsigned int startupDelays[STARTUP_NUM_COMMUTATIONS];

/*! \brief Filtered commutation timer variable and speed indicator.
 *  This value equals half the time of one commutation step. It is filtered
 *  through an IIR filter, so the value stored is not the most recent measuremnt.
 *  The variable is stored in registers R14-R15 for quicker access.
 */
volatile unsigned int filteredTimeSinceCommutation;

/*! \brief The power stage enable signals that will be output to the motor drivers
 *  at next commutation.
 *
 *  This variable holds the pattern of enable signals that will be output to the
 *  power stage at next commutation. It is stored in register R13 for quick access.
 */
volatile unsigned char nextDrivePattern;

/*! \brief Polarity of the expected zero crossing.
 *
 *  The polarity of the expected zero crossing.
 *  Could be eiter \ref EDGE_FALLING or \ref EDGE_RISING.
 */
volatile unsigned char zcPolarity;

/*! \brief The commutation step that starts at next commutation.
 *
 *  The commutation step that starts at next commutation. This is used to keep
 *  track on where in the commutation cycle we are. Stored in register R11 for
 *  quick access
 */
volatile unsigned char nextCommutationStep;

//! ADC reading of shunt voltage.
volatile unsigned char shuntVoltageADC = 0;

//! ADC reading of the known external reference voltage.
volatile unsigned char referenceVoltageADC;

//! Flag that specifies whether a new external speed reference and a motor speed measurement is available.
volatile unsigned char speedUpdated = FALSE;

//! Flag that specifies whether a new current measurement is available.
volatile unsigned char currentUpdated = FALSE;

/*! \brief Examines the reset source and acts accordingly.
 *
 *  This function is called early in the program to disable watchdog timer and
 *  determine the source of reset.
 *
 *  Actions can be taken, based on the reset source. When the watchdog is used as
 *  stall protection, a stall can be detected here. It is possible to e.g. store
 *  a variable in EEPROM that counts the number of failed restart attempts. After a
 *  certain number of attempts, the motor could simply refuse to continue until
 *  an external action happens, indicating that the rotor lock situation could be
 *  fixed.
 */
static void ResetHandler(void)
{
  unsigned static int restartAttempts;
  // Temporary variable to avoid unnecessary reading of volatile register MCUSR.
  unsigned char tempMCUSR;

  tempMCUSR = MCUSR;
  MCUSR = tempMCUSR & ~((1 << WDRF) | (1 << BORF) | (1 << EXTRF) | (1 << PORF));

  // Reset watchdog to avoid false stall detection before the motor has started.
  cli();
  wdt_reset();
  WDTCSR |= (1 << WDCE) | (1 << WDE);
  WDTCSR = 0x00;

  // Examine the reset source and take action.
  switch (tempMCUSR & ((1 << WDRF) | (1 << BORF) | (1 << EXTRF) | (1 << PORF)))
  {
  case (1 << WDRF):
    restartAttempts++;
    if (restartAttempts >= MAX_RESTART_ATTEMPTS)
    {
      // Do something here. E.g. wait for a button to be pressed.
      for (;;)
      {

      }
    }

    // Put watchdog reset handler here.
    break;
  case (1 << BORF):
    //Put brownout reset handler here.
    break;
  case (1 << EXTRF):
    restartAttempts = 0;
    // Put external reset handler here.
    break;
  case (1 << PORF):
    restartAttempts = 0;
    // Put power-on reset handler here.
    break;
  }
}


/*! \brief Initializes I/O ports.
 *
 *  Initializes I/O ports.
 */
static void InitPorts(void)
{
  // Init DRIVE_DDR for motor driving.
  DRIVE_DDR = (1 << UL) | (1 << UH) | (1 << VL) | (1 << VH) | (1 << WL) | (1 << WH);

  // Init PORTD for PWM on PD3.
  DDRD = (1 << PIND3);

  // Disable digital input buffers on ADC channels.
  DIDR0 = (1 << ADC4D) | (1 << ADC3D) | (1 << ADC2D) | (1 << ADC1D) | (1 << ADC0D);
}


/*! \brief Initializes timers (for commutation timing and PWM).
 *
 *  This function initializes Timer/counter0 for PWM operation for motor speed control
 *  and Timer/counter1 for commutation timing.
 */
static void InitTimers(void)
{
  // Set up Timer/counter0 for PWM, output on OCR0B, OCR0A as TOP value, prescaler = 1.
  TCCR0A = (0 << COM0A1) | (0 << COM0A0) | (1 << COM0B1) | (0 << COM0B0) | (0 << WGM01) | (1 << WGM00);
  TCCR0B = (1 << WGM02) | (0 << CS02) | (0 << CS01) | (1 << CS00);
  OCR0A = PWM_TOP_VALUE;
  CLEAR_ALL_TIMER2_INT_FLAGS;
  SET_TIMER2_INT_ZC_DETECTION;

  // Set up Timer/counter1 for commutation timing, prescaler = 8.
  TCCR1B = (1 << CS11) | (0 << CS10);
}


/*! \brief Initializes the AD-converter.
 *
 *  This function initializes the AD-converter and makes a reading of the external
 *  reference voltage.
 */
static void InitADC(void)
{
  // First make a measurement of the external reference voltage.
  ADMUX = ADMUX_REF_VOLTAGE;
  ADCSRA = (1 << ADEN) | (1 << ADSC) | (1 << ADIF) | (ADC_PRESCALER_16);
  while (ADCSRA & (1 << ADSC))
  {

  }
  referenceVoltageADC = ADCH;
  AdcData[0] = referenceVoltageADC;

  // Initialize the ADC for autotriggered operation on PWM timer overflow.
  ADCSRA = (1 << ADEN) | (0 << ADSC) | (1 << ADATE) | (1 << ADIF) | (0 << ADIE) | ADC_PRESCALER_8;
  ADCSRB = ADC_TRIGGER_SOURCE;
}


/*! \brief Initializes the analog comparator.
 *
 *  This function initializes the analog comparator for overcurrent detection.
 */
static void InitAnalogComparator(void)
{
#ifdef ANALOG_COMPARATOR_ENABLE
  // Enable analog comparator interrupt on rising edge.
  ACSR = (0 << ACBG) | (1 << ACI) | (1 << ACIE) | (1 << ACIS1) | (1 << ACIS0);
#endif
}


/*! \brief Initializes the watchdog timer
 *
 *  This function initializes the watchdog timer for stall restart.
 */
static void WatchdogTimerEnable(void)
{
  cli();
  wdt_reset();

  WDTCSR |= (1 << WDCE) | (1 << WDE);

  WDTCSR = (1 << WDIF) | (1 << WDIE) | (1 << WDE) | (1 << WDP2);
  sei();
}


/*! \brief Initializes arrays for motor driving and AD channel selection.
 *
 *  This function initializes the arrays used for motor driving and AD channel
 *  selection that changes for each commutation step.
 */
static void MakeTables(void)
{
#if DIRECTION_OF_ROTATION == CCW
  driveTable[0] = DRIVE_PATTERN_STEP1_CCW;
  driveTable[1] = DRIVE_PATTERN_STEP2_CCW;
  driveTable[2] = DRIVE_PATTERN_STEP3_CCW;
  driveTable[3] = DRIVE_PATTERN_STEP4_CCW;
  driveTable[4] = DRIVE_PATTERN_STEP5_CCW;
  driveTable[5] = DRIVE_PATTERN_STEP6_CCW;

  ADMUXTable[0] = ADMUX_W;
  ADMUXTable[1] = ADMUX_V;
  ADMUXTable[2] = ADMUX_U;
  ADMUXTable[3] = ADMUX_W;
  ADMUXTable[4] = ADMUX_V;
  ADMUXTable[5] = ADMUX_U;
#else
  driveTable[0] = DRIVE_PATTERN_STEP1_CW;
  driveTable[1] = DRIVE_PATTERN_STEP2_CW;
  driveTable[2] = DRIVE_PATTERN_STEP3_CW;
  driveTable[3] = DRIVE_PATTERN_STEP4_CW;
  driveTable[4] = DRIVE_PATTERN_STEP5_CW;
  driveTable[5] = DRIVE_PATTERN_STEP6_CW;

  ADMUXTable[0] = ADMUX_U;
  ADMUXTable[1] = ADMUX_V;
  ADMUXTable[2] = ADMUX_W;
  ADMUXTable[3] = ADMUX_U;
  ADMUXTable[4] = ADMUX_V;
  ADMUXTable[5] = ADMUX_W;

#endif

  startupDelays[0] = 200;
  startupDelays[1] = 150;
  startupDelays[2] = 100;
  startupDelays[3] = 80;
  startupDelays[4] = 70;
  startupDelays[5] = 65;
  startupDelays[6] = 60;
  startupDelays[7] = 55;
}


/*! \brief Executes the motor startup sequence.
 *
 *  This function locks the motor into a known position and fires off a
 *  commutation sequence controlled by the Timer/counter1 overflow interrupt.
 */
static void StartMotor(void)
{
  unsigned char i;

  SET_PWM_COMPARE_VALUE(STARTUP_PWM_COMPARE_VALUE);

  nextCommutationStep = 0;

  //Preposition.
  DRIVE_PORT = driveTable[nextCommutationStep];
  StartupDelay(STARTUP_LOCK_DELAY);
  nextCommutationStep++;
  nextDrivePattern = driveTable[nextCommutationStep];

  for (i = 0; i < STARTUP_NUM_COMMUTATIONS; i++)
  {
    DRIVE_PORT = nextDrivePattern;
    StartupDelay(startupDelays[i]);

    ADMUX = ADMUXTable[nextCommutationStep];

    // Use LSB of nextCommutationStep to determine zero crossing polarity.
    zcPolarity = nextCommutationStep & 0x01;

    nextCommutationStep++;
    if (nextCommutationStep >= 6)
    {
      nextCommutationStep = 0;
    }
    nextDrivePattern = driveTable[nextCommutationStep];
  }

  // Switch to sensorless commutation.
  TCNT1 = 0;
  TIMSK1 = (1 << OCIE1A);

  // Set filteredTimeSinceCommutation to the time to the next commutation.
  filteredTimeSinceCommutation = startupDelays[STARTUP_NUM_COMMUTATIONS - 1] * (STARTUP_DELAY_MULTIPLIER  / 2);
}


/*! \brief Timer/counter2 bottom overflow. Used for zero-cross detection.
 *
 *  This interrupt service routine is called every time the up/down counting
 *  PWM counter reaches bottom. An ADC reading on the active channel is
 *  automatically triggered at the same time as this interrupt is triggered.
 *  This is used to detect a zero crossing.
 *
 *  In the event of a zero crossing, the time since last commutation is stored
 *  and Timer/counter1 compare A is set up to trigger at the next commutation
 *  instant.
 */
 ISR( TIMER2_OVF_vect ) //MotorPWMBottom()
{
  unsigned char temp;
  unsigned char mux = 0;

  // Disable ADC auto-triggering. This must be done here to avoid wrong channel being sampled on manual samples later.
  ADCSRA &= ~((1 << ADATE) | (1 << ADIE));

  // Wait for auto-triggered ADC sample to complete.
  while (!(ADCSRA & (1 << ADIF)))
  {

  }
  temp = ADCH;
  
  mux = nextCommutationStep - 1;
  if(mux == 0 || mux == 3)
	AdcData[1] = temp;
  else if(mux == 1 || mux == 4)
	AdcData[2] = temp;
  else if(mux == 2 || mux == 5)
	AdcData[3] = temp;
	
	
  if (((zcPolarity == EDGE_RISING) && (temp > ADC_ZC_THRESHOLD)) || ((zcPolarity == EDGE_FALLING) && (temp < ADC_ZC_THRESHOLD)))
  {
    unsigned int timeSinceCommutation;

    // Find time since last commutation
    timeSinceCommutation = TCNT1;
    TCNT1 = COMMUTATION_CORRECTION;

    // Filter the current ZC detection with earlier measurements through an IIR filter.
    filteredTimeSinceCommutation = (COMMUTATION_TIMING_IIR_COEFF_A * timeSinceCommutation
                                + COMMUTATION_TIMING_IIR_COEFF_B * filteredTimeSinceCommutation)
                                / (COMMUTATION_TIMING_IIR_COEFF_A + COMMUTATION_TIMING_IIR_COEFF_B);
    OCR1A = filteredTimeSinceCommutation;

    speedUpdated = TRUE;

    SET_TIMER1_INT_COMMUTATION;
    CLEAR_ALL_TIMER1_INT_FLAGS;

    // Disable Timer/Counter0 overflow ISR.
    DISABLE_ALL_TIMER2_INTS;

    // Read voltage reference.
    // Change ADC channel.
    ADMUX = ADMUX_REF_VOLTAGE;
    // Start conversion manually.
    ADCSRA |= (1 << ADSC);
    // Wait for conversion to complete.
    while((ADCSRA & (1 << ADSC)))
    {

    }
    referenceVoltageADC = ADCH;
	AdcData[0] = referenceVoltageADC;
	
    // Enable current measurements in ADC ISR.
    ADMUX = ADMUX_CURRENT;
    ADCSRA |= (1 << ADATE) | (1 << ADIE) | ADC_PRESCALER;
  }
  else
  {
    unsigned char tempADMUX;

    tempADMUX = ADMUX;
    // Read current

    // Make sure that a sample is not in progress
    while (ADCSRA & (1 << ADSC))
    {

    }

    // Change channel
    ADMUX = ADMUX_CURRENT;

    // Start conversion manually.
    ADCSRA |= (1 << ADSC);
    // Wait for conversion to complete.
    while((ADCSRA & (1 << ADSC)))
    {

    }

    shuntVoltageADC = ADCH;
	AdcData[4] = shuntVoltageADC;
    currentUpdated = TRUE;

    // Restore ADC channel.
    ADMUX = tempADMUX;
    ADCSRA |= (1 << ADATE) | (1 << ADIE) | ADC_PRESCALER;
  }
}


/*! \brief Commutates and prepares for new zero-cross detection.
 *
 *  This interrupt service routine is triggered exactly when a commutation
 *  is scheduled. The commutation is performed instantly and Timer/counter0
 *  is reset to measure the delay between commutation and zero-cross detection.
 *
 *  Commutation causes large transients on all phases for a short while that could
 *  cause false zero-cross detections. A zero cross detection hold-off period is
 *  therefore used to avoid any false readings. This is performed by using Timer/counter1
 *  compare B. The compare is set to happen after the specified hold-off period.
 *  Timer/counter1 compare B interrupt handler then enables the zero-cross detection.
 */
ISR( TIMER1_COMPA_vect) //Commutate()
{
  // Commutate and clear commutation timer.
  DRIVE_PORT = nextDrivePattern;
  TCNT1 = 0;

  zcPolarity = nextCommutationStep & 0x01;

  // Set zero-cross detection holdoff time.
  CLEAR_ALL_TIMER1_INT_FLAGS;
  OCR1B = ZC_DETECTION_HOLDOFF_TIME_US;
  SET_TIMER1_INT_HOLDOFF;

  wdt_reset();
}


/*! \brief Enables zero-cross detection.
 *
 *  This interrupt service routine is triggered when the zero cross detection
 *  hold-off time after commutation is over. All Timer/counter1 interrupts are
 *  disabled and Timer/counter0 (PWM) overflow interrupt is enabled to allow
 *  the ADC readings to be used for zero-cross detection.
 */
ISR( TIMER1_COMPB_vect ) //EnableZCDetection()
{
  CLEAR_ALL_TIMER1_INT_FLAGS;
  CLEAR_ALL_TIMER2_INT_FLAGS;
  SET_TIMER2_INT_ZC_DETECTION;
  DISABLE_ALL_TIMER1_INTS;

  // Set up ADC for zero-cross detection
  ADMUX = ADMUXTable[nextCommutationStep];

  // Wait for ADC to complete
  while (!(ADCSRA & (1 << ADIF)))
  {

  }
  ADCSRA &= ~(1 << ADIE);
  ADCSRA |= (1 << ADSC) | (1 << ADATE);

  // Rotate commutation step counter.
  nextCommutationStep++;
  if (nextCommutationStep >= 6)
  {
    nextCommutationStep = 0;
  }
  nextDrivePattern = driveTable[nextCommutationStep];
}


/* \brief ADC complete interrupt service routine, used for current measurements.
 *
 *  This interrupt service routine is only enabled when current measurements are
 *  auto-triggered by the PWM counter overflow. The measured value is simply
 *  copied to \ref shuntVoltageADC, the \ref currentUpdated flag is set and
 *  Timer0 (PWM timer) interrupt flags are cleared.
 */
ISR( ADC_vect ) //void CurrentMeasurementComplete()
{
  shuntVoltageADC = ADCH;
  AdcData[4] = shuntVoltageADC;
  currentUpdated = TRUE;
  CLEAR_ALL_TIMER2_INT_FLAGS;
}


/*! \brief Watchdog interrupt
 *
 *  This ISR is called before the watchdog timer resets the device because of a stall.
 *  It simply disables driving, but other tasks that must be done before a watchdog reset,
 *  such as storage of variables in non-volatile memory can be done here.
 */
ISR( WDT_vect) //void WatchdogISR()
{
  DISABLE_DRIVING;
  for(;;)
  {
    ;
  }
}

/*! \brief Overcurrent interrupt
 *
 *  This interrupt service routine cuts power to the motor when an overcurrent situation
 *  is detected.
 */
#ifdef ANALOG_COMPARATOR_ENABLE
ISR( ANA_COMP_vect) //void OverCurrentISR()
{
  DISABLE_DRIVING;
  for(;;)
  {
    ;
  }
}
#endif


/*! \brief Generates a delay used during startup
 *
 *  This functions is used to generate a delay during the startup procedure.
 *  The length of the delay equals delay * STARTUP_DELAY_MULTIPLIER microseconds.
 *  Since Timer/Counter1 is used in this function, it must never be called when
 *  sensorless operation is running.
 */
void StartupDelay(unsigned int delay)
{
  CLEAR_ALL_TIMER1_INT_FLAGS;
  do
  {
    TCNT1 = 0xffff - STARTUP_DELAY_MULTIPLIER;
    // Wait for timer to overflow.
    while (!(TIFR1 & (1 << TOV1)))
    {

    }

    CLEAR_ALL_TIMER1_INT_FLAGS;
    delay--;
  } while (delay);
}



/*! \brief Controls the PWM duty cycle based on speed set-point and current consumption.
 *
 *  This function controls the PWM duty cycle by calling a speed controller and a
 *  current controller. The speed controller signal is directly applied to the duty
 *  cycle. The current controller signal is used to limit the maximum duty cycle.
 */
static void PWMControl(void)
{
  signed int speedCompensation;
  static unsigned char currentCompensation = 0;
  static signed int duty = STARTUP_PWM_COMPARE_VALUE;

  // Run speed control only if a new speed measurement is available.
 if (speedUpdated)
  {
    speedCompensation = SpeedControl();
    speedUpdated = FALSE;
    duty += speedCompensation;
  }

  // Run current control only if a new current measurement is available.
  if (currentUpdated)
  {
     currentCompensation = CurrentControl();
     currentUpdated = FALSE;
  }

 // Keep duty cycle within limits.
  if (duty < MIN_PWM_COMPARE_VALUE)
  {
    duty = MIN_PWM_COMPARE_VALUE;
  }
  if (duty > (MAX_PWM_COMPARE_VALUE - currentCompensation))
  {
    duty = MAX_PWM_COMPARE_VALUE - currentCompensation;
  }

  SET_PWM_COMPARE_VALUE((unsigned char)duty);
}



/*! \brief Calculates the current speed in electrical RPM.
 *
 *  This function calculates the current speed in electrical rotations per
 *  minute from the global variable \ref filteredTimeSinceCommutation.
 */
static unsigned long CalculateSpeed()
{
  // Copy used to minimize period where interrupts are disabled.
  unsigned int filteredTimeSinceCommutationCopy;
  unsigned long rotationPeriod;
  unsigned long speed;

  /*
  Disable interrupts to ensure that \ref filteredTimeSinceCommutation is accessed in
  an atomic operation.
  */
  cli();
  filteredTimeSinceCommutationCopy = filteredTimeSinceCommutation;
  sei();

  /*
  filteredTimeSinceCommutation is one half commutation time. Must be multiplied by 12 to get
  one full rotation.
  */
  rotationPeriod = (unsigned long)filteredTimeSinceCommutationCopy * 12;
  speed = (TICKS_PER_MINUTE / rotationPeriod);

  return speed;
}


/*! \brief Calculates current consumption.
 *
 *  This function calculates the current consumption in milliAmperes from the
 *  global variable \ref shuntVoltageADC. The external know reference voltage
 *  is used to compensate for varying AREF voltage.
 */
static unsigned int CalculateCurrent()
{
  unsigned long ADCref;
  unsigned int current;

  // Calculate the voltage at AREF pin (scaled down motor supply voltage),
  // using the known reference voltage. (In milliVolts)
  ADCref = EXTERNAL_REF_VOLTAGE * 256UL / referenceVoltageADC;

  // Calculate the current through the shunt. (In milliAmperes)
  current = (unsigned int)((shuntVoltageADC * ADCref * 1000UL / 256UL) / SHUNT_RESISTANCE);

  return current;
}


/*! \brief Speed control loop
 *
 *  This function runs a simple P-regulator speed control loop. The duty cycle
 *  is only updated each time a new speed measurement is ready (on each zero-crossing).
 *  The time step is thus variable, so the P-gain of the P-regulator corresponds to
 *  a speed-varying P-gain for the continous system.
 */
static signed int SpeedControl(void)
{
  unsigned long currentSpeed;
  signed long speedError;
  signed long dutyChange;

  currentSpeed = CalculateSpeed();
  speedError = (speedSetpoint - currentSpeed);
  dutyChange = speedError * P_REG_K_P / P_REG_SCALING;

  return dutyChange;
}

/*! \brief Current control loop
 *
 *  This function is called after the speed control loop. The desired duty cycle
 *  calculated by the speed control loop is available, and this function is
 *  responsible for adjusting the duty cycle to ensure that the current stays
 *  within limits.
 */
static unsigned char CurrentControl(void)
{
  unsigned int current;
  unsigned int overCurrentCorrection = 0;

  current = CalculateCurrent();

  // Cut power to motor if current is critically high.
  if (current > CURRENT_LIMITER_CRITICAL)
  {
    DISABLE_DRIVING;
    for (;;)
    {
      // Stop and let watchdog timer reset part.
    }
  }

  if (current > CURRENT_LIMITER_START)
  {
    overCurrentCorrection = (current - CURRENT_LIMITER_START) * CURRENT_LIMITER_FACTOR;
  }

  if (overCurrentCorrection > 255)
  {
    return 255;
  }

  return overCurrentCorrection;
}


/*! \mainpage
 * \section Intro Introduction
 * This documents data structures, functions, variables, defines, enums, and
 * typedefs in the software for application note AVR444.
 *
 * \section CI Compilation Info
 * This software was written for the IAR Embedded Workbench 4.11A.
 *
 * To make project:
 * <ol>
 * <li> Add the file main.c to project.
 * <li> Under processor configuration, select device ATmega48, ATmega88
 * or ATmega168
 * <li> Enable bit definitions in I/O include files
 * <li> Under "C/C++ compiler->Code->Register utilization", select 5
 * registers (R11..R15) locked for global variables.
 * <li> High optimization on speed is recommended for best performance
 * </ol>
 *
 * \section DI Device Info
 * The included source code is written for ATmega48/88/168.
 */


