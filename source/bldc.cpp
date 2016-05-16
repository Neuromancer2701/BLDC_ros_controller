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
#include "bldc.h"
#include "SoftPWM.h"
#include <avr/wdt.h>

#define SYSTEM_FREQUENCY        16000000	//! System clock frequecy. Used to calculate PWM TOP value.
#define PWM_BASE_FREQUENCY      20000	//! PWM base frequency. Used to calculate PWM TOP value.
#define PWM_TOP_VALUE           (SYSTEM_FREQUENCY / PWM_BASE_FREQUENCY / 2)	 //! PWM TOP value. Automatically calculated to give desired \ref PWM_BASE_FREQUENCY.


#define FALSE     0 //! Boolean FALSE value.


#define TRUE      (!FALSE)	//! Boolean TRUE expression. Can be used both for test and assignment.

#if defined (__AVR_ATmega2560__)

	#define UL    PINB7	//! Port pin connected to phase U, low side enable switch.  Arduino Pin 13
	#define UH    PINB6	//! Port pin connected to phase U, high side enable switch. Arduino Pin 12 

	#define VL    PINB5	//! Port pin connected to phase V, high side enable switch. Arduino Pin 11
	#define VH    PINB4	//! Port pin connected to phase V, high side enable switch. Arduino Pin 10

	#define WL    PINH6	//! Port pin connected to phase W, low side enable switch.	Arduino Pin 9
	#define WH    PINH5	//! Port pin connected to phase W, high side enable switch. Arduino Pin 8
	
	#define DRIVE_PORT( io )  PORTB = (io & 0xF0) \	//! PORT register for drive pattern output.
							  PORTH = (io & 0x6)
	#define DRIVE_DDR( io )   DDRB	//! Data direction register for drive pattern output.
#else
	#define UL    	PINB5	//! Port pin connected to phase U, low side enable switch.  Arduino Pin 13
	#define UL_Pin  13
	#define UH      PINB4	//! Port pin connected to phase U, high side enable switch. Arduino Pin 12 
	#define UH_Pin  12
	
	#define VL      PINB3	//! Port pin connected to phase V, high side enable switch. Arduino Pin 11
	#define VL_Pin  11
	#define VH      PINB2	//! Port pin connected to phase V, high side enable switch. Arduino Pin 10
	#define VH_Pin  10
	
	#define WL    PINB1	//! Port pin connected to phase W, low side enable switch.	Arduino Pin 9
	#define WL_Pin  9
	#define WH    PINB0	//! Port pin connected to phase W, high side enable switch. Arduino Pin 8
	#define WH_Pin  8
	
	#define DRIVE_PORT	PORTB	//! PORT register for drive pattern output.
	#define DRIVE_DDR 	DDRB	//! Data direction register for drive pattern output.	
	
#endif

#define CW    0	//! Clockwise rotation flag. Used only in macros.
#define CCW   1	//! Counterclockwise rotation flag. Used only in macros.


/*! Direction of rotation. Set to either CW or CCW for
 * clockwise and counterclockwise respectively.
 */
#define DIRECTION_OF_ROTATION     CCW
#define useIO 0

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



#define ADC_MUX_U           0x3	//! ADC multiplexer selection for channel U sampling.
#define ADC_MUX_V           0x2	//! ADC multiplexer selection for channel V sampling.
#define ADC_MUX_W           0x1	//! ADC multiplexer selection for channel W sampling.
#define ADC_MUX_CURRENT     0x0	//! ADC multiplexer selection for current sampling.
#define ADC_MUX_REF_VOLTAGE 0x4	//! ADC multiplexer selection for reference voltage sampling.
#define ADC_HALL_1           0x1
#define ADC_HALL_2           0x2
#define ADC_HALL_3           0x3

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
#define MAX_COMMUTATION_STEPS 7
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
    forward = true;
    HallIndex = 0;
    CommunationState = 0;
}

BLDC::~BLDC()
{

}

void BLDC::init()
{
  InitPorts();
  MakeTables();

}


void BLDC::ReadADCHalls()
{
    RawHallData[0] = analogRead(ADC_HALL_1);
    RawHallData[1] = analogRead(ADC_HALL_2);
    RawHallData[2] = analogRead(ADC_HALL_3);
    for(int i = 0; i < NUMBER_HALLS; i++) {
        HallStates[i] = (RawHallData[i] > HALL_THRESHOLD);
    }
}

unsigned short BLDC::LookupIndex()
{
    for(int i = 0; i < NUMBER_HALLS; i++)
    {
        HallIndex |= ((unsigned short)HallStates[i]) << i;
    }

    return LookupTable[HallIndex][(unsigned)forward];
}

void BLDC::setSpeed(unsigned long _speed)
{
 speedSetpoint = speed = _speed;
}

void BLDC::Control()
{
    ReadADCHalls();
    SetCommutationState(LookupIndex());
}

void BLDC::SetCommutationState(unsigned short state)
{
	#if useIO
	if( (state >= MIN_COMMUTATION_STEPS) && (state < MAX_COMMUTATION_STEPS))
	{
		DRIVE_PORT = driveTable[state];
	}
	#else
	char UL_duty = 0;
	char UH_duty = 0;
	char VL_duty = 0;
	char VH_duty = 0;
	char WL_duty = 0;
	char WH_duty = 0;


    CommunationState = state;
	if( (state >= MIN_COMMUTATION_STEPS) && (state < MAX_COMMUTATION_STEPS))
	{
		switch(driveTable[state])
		{
            case 0:
                 UL_duty = 0;
                 UH_duty = 0;
                 VL_duty = 0;
                 VH_duty = 0;
                 WL_duty = 0;
                 WH_duty = 0;
                 break;

			case DRIVE_PATTERN_STEP1_CCW:
				 UL_duty = 75;
				 VH_duty = 75;
				 break;
			case DRIVE_PATTERN_STEP2_CCW:
				 UL_duty = 75;
				 WH_duty = 75;
				 break;
			case DRIVE_PATTERN_STEP3_CCW:
				 VL_duty = 75;
				 WH_duty = 75;
				 break;
			case DRIVE_PATTERN_STEP4_CCW:
				 VL_duty = 75;
				 UH_duty = 75;
				 break;
			case DRIVE_PATTERN_STEP5_CCW:
				 WL_duty = 75;
				 UH_duty = 75;
				 break;
			case DRIVE_PATTERN_STEP6_CCW:
				 WL_duty = 75;
				 VH_duty = 75;
				 break;
		
		}
	}

	SoftPWMSetPercent(UL_Pin, UL_duty);
	SoftPWMSetPercent(UH_Pin, UH_duty);
	SoftPWMSetPercent(VL_Pin, VL_duty);
	SoftPWMSetPercent(VH_Pin, VH_duty);
	SoftPWMSetPercent(WL_Pin, WL_duty);
	SoftPWMSetPercent(WH_Pin, WH_duty);


	#endif
}



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

/*! \brief Initializes I/O ports.
 *
 *  Initializes I/O ports.
 */
static void InitPorts(void)
{
  	#if useIO
  // Init DRIVE_DDR for motor driving.
  DRIVE_DDR = (1 << UL) | (1 << UH) | (1 << VL) | (1 << VH) | (1 << WL) | (1 << WH);

  // Init PORTD for PWM on PD3.
  DDRD = (1 << PIND3);
	#else
	SoftPWMBegin();
	SoftPWMSetPercent(UL_Pin,0);
	SoftPWMSetPercent(UH_Pin,0);
	SoftPWMSetPercent(VL_Pin,0);
	SoftPWMSetPercent(VH_Pin,0);
	SoftPWMSetPercent(WL_Pin,0);
	SoftPWMSetPercent(WH_Pin,0);
	#endif
  

}

/*! \brief Initializes arrays for motor driving and AD channel selection.
 *
 *  This function initializes the arrays used for motor driving and AD channel
 *  selection that changes for each commutation step.
 */
static void MakeTables(void)
{
#if DIRECTION_OF_ROTATION == CCW
    driveTable[0] = 0x00;
    driveTable[1] = DRIVE_PATTERN_STEP1_CCW;
    driveTable[2] = DRIVE_PATTERN_STEP2_CCW;
    driveTable[3] = DRIVE_PATTERN_STEP3_CCW;
    driveTable[4] = DRIVE_PATTERN_STEP4_CCW;
    driveTable[5] = DRIVE_PATTERN_STEP5_CCW;
    driveTable[6] = DRIVE_PATTERN_STEP6_CCW;
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
