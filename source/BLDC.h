// This file has been prepared for Doxygen automatic documentation generation.
/*! \file ********************************************************************
*
* Atmel Corporation
*
* - File              : BLDC.h
* - Compiler          : IAR EWAAVR 2.28a/3.10c
*
* - Support mail      : avr@atmel.com
*
* - Supported devices : ATmega48/88/168
*
* - AppNote           : AVR444 - Sensorless control of three-phase brushless
'                       DC motors with ATmega48.
*
* - Description       : Example of how to use the ATmega48 for sensorless
*                       control of a three phase brushless DC motor.
*
* $Revision: 1.1 $
* $Date: Monday, October 10, 2005 11:15:46 UTC $
*****************************************************************************/

#ifndef __BLDC_H__
#define __BLDC_H__

//! System clock frequecy. Used to calculate PWM TOP value.
#define SYSTEM_FREQUENCY        8000000

//! PWM base frequency. Used to calculate PWM TOP value.
#define PWM_BASE_FREQUENCY      20000

//! PWM TOP value. Automatically calculated to give desired \ref PWM_BASE_FREQUENCY.
#define PWM_TOP_VALUE           (SYSTEM_FREQUENCY / PWM_BASE_FREQUENCY / 2)

//! Boolean FALSE value.
#define FALSE     0

//! Boolean TRUE expression. Can be used both for test and assignment.
#define TRUE      (!FALSE)

//! Port pin connected to phase U, low side enable switch.
#define UL    PB5

//! Port pin connected to phase U, high side enable switch.
#define UH    PB4

//! Port pin connected to phase V, low side enable switch.
#define VL    PB3

//! Port pin connected to phase V, high side enable switch.
#define VH    PB2

//! Port pin connected to phase W, low side enable switch.
#define WL    PB1

//! Port pin connected to phase W, high side enable switch.
#define WH    PB0

//! Clockwise rotation flag. Used only in macros.
#define CW    0

//! Counterclockwise rotation flag. Used only in macros.
#define CCW   1

/*! Direction of rotation. Set to either CW or CCW for
 * clockwise and counterclockwise respectively.
 */
#define DIRECTION_OF_ROTATION     CCW

//! Drive pattern for commutation step 1, CCW rotation.
#define DRIVE_PATTERN_STEP1_CCW      ((1 << UL) | (1 << VH))

//! Drive pattern for commutation step 2, CCW rotation.
#define DRIVE_PATTERN_STEP2_CCW      ((1 << UL) | (1 << WH))

//! Drive pattern for commutation step 3, CCW rotation.
#define DRIVE_PATTERN_STEP3_CCW      ((1 << VL) | (1 << WH))

//! Drive pattern for commutation step 4, CCW rotation.
#define DRIVE_PATTERN_STEP4_CCW      ((1 << VL) | (1 << UH))

//! Drive pattern for commutation step 5, CCW rotation.
#define DRIVE_PATTERN_STEP5_CCW      ((1 << WL) | (1 << UH))

//! Drive pattern for commutation step 6, CCW rotation.
#define DRIVE_PATTERN_STEP6_CCW      ((1 << WL) | (1 << VH))


//! Drive pattern for commutation step 1, CW rotation.
#define DRIVE_PATTERN_STEP1_CW      ((1 << VH) | (1 << WL))

//! Drive pattern for commutation step 2, CW rotation.
#define DRIVE_PATTERN_STEP2_CW      ((1 << UH) | (1 << WL))

//! Drive pattern for commutation step 3, CW rotation.
#define DRIVE_PATTERN_STEP3_CW      ((1 << UH) | (1 << VL))

//! Drive pattern for commutation step 4, CW rotation.
#define DRIVE_PATTERN_STEP4_CW      ((1 << WH) | (1 << VL))

//! Drive pattern for commutation step 5, CW rotation.
#define DRIVE_PATTERN_STEP5_CW      ((1 << WH) | (1 << UL))

//! Drive pattern for commutation step 6, CW rotation.
#define DRIVE_PATTERN_STEP6_CW      ((1 << VH) | (1 << UL))

//! Zero crossing polarity flag value for falling zero crossing.
#define EDGE_FALLING          1

//! Zero crossing polarity flag value for rinsing zero crossing.
#define EDGE_RISING           0

//! PORT register for drive pattern output.
#define DRIVE_PORT  PORTB

//! Data direction register for drive pattern output.
#define DRIVE_DDR   DDRB

//! ADC multiplexer selection for channel U sampling.
#define ADC_MUX_U           0x0

//! ADC multiplexer selection for channel V sampling.
#define ADC_MUX_V           0x1

//! ADC multiplexer selection for channel W sampling.
#define ADC_MUX_W           0x2

//! ADC multiplexer selection for speed reference sampling.
#define ADC_MUX_SPEED_REF   0x4

//! ADC multiplexer selection for current sampling.
#define ADC_MUX_CURRENT     0x3

//! ADC multiplexer selection for reference voltage sampling.
#define ADC_MUX_REF_VOLTAGE 0x5

//! ADC reference channel selection.
#define ADC_REF_CHANNEL                 ((0 << REFS1) | (0 << REFS0))

//! ADC result alignment for BEMF measurement.
#define ADC_RES_ALIGNMENT_BEMF          (1 << ADLAR)

//! ADC result alignment for speed reference measurement.
#define ADC_RES_ALIGNMENT_SPEED_REF     (1 << ADLAR)

//! ADC result alignment for CURRENT measurement.
#define ADC_RES_ALIGNMENT_CURRENT       (1 << ADLAR)

//! ADC result alignment for reference voltage measurement.
#define ADC_RES_ALIGNMENT_REF_VOLTAGE   (1 << ADLAR)

//! ADMUX register value for channel U sampling.
#define ADMUX_U             (ADC_REF_CHANNEL | ADC_RES_ALIGNMENT_BEMF | ADC_MUX_U)

//! ADMUX register value for channel V sampling.
#define ADMUX_V             (ADC_REF_CHANNEL | ADC_RES_ALIGNMENT_BEMF | ADC_MUX_V)

//! ADMUX register value for channel W sampling.
#define ADMUX_W             (ADC_REF_CHANNEL | ADC_RES_ALIGNMENT_BEMF | ADC_MUX_W)

//! ADMUX register value for speed reference sampling.
#define ADMUX_SPEED_REF     (ADC_REF_CHANNEL | ADC_RES_ALIGNMENT_SPEED_REF | ADC_MUX_SPEED_REF)

//! ADMUX register value for current sampling.
#define ADMUX_CURRENT       (ADC_REF_CHANNEL | ADC_RES_ALIGNMENT_CURRENT | ADC_MUX_CURRENT)

//! ADMUX register value for reference voltage sampling.
#define ADMUX_REF_VOLTAGE   (ADC_REF_CHANNEL | ADC_RES_ALIGNMENT_REF_VOLTAGE | ADC_MUX_REF_VOLTAGE)

//! ADC clock prescaled by 8 value.
#define ADC_PRESCALER_8     ((0 << ADPS2) | (1 << ADPS1) | (1 << ADPS0))

//! ADC clock prescaled by 8 value.
#define ADC_PRESCALER_16     ((1 << ADPS2) | (0 << ADPS1) | (0 << ADPS0))

//! ADC prescaler used.
#define ADC_PRESCALER       ADC_PRESCALER_8

//! ADC trigger source.
#define ADC_TRIGGER_SOURCE  ((1 << ADTS2) | (0 << ADTS1) | (0 << ADTS0))

/*! Number of commutations performed during startup. Also specifies
 *  size of \ref startupDelays. Do not change without also changing
 *  \ref MakeTables().
 */
#define STARTUP_NUM_COMMUTATIONS  8

//! Startup delays are given in milliseconds times STARTUP_DELAY_MULTIPLIER.
#define STARTUP_DELAY_MULTIPLIER  100

/*!
 *  Number of milliseconds to lock rotor in first commutation step before
 *  the timed startup sequence is initiated.
 */
#define STARTUP_LOCK_DELAY        10000

//! The maximum number of restart attempts without external action when stall is detected.
#define MAX_RESTART_ATTEMPTS    10

//! Holdoff time where zero-cross detection is disabled after commutation.
#define ZC_DETECTION_HOLDOFF_TIME_US (filteredTimeSinceCommutation / 2)

//! Macro that sets a new duty cycle by changing the PWM compare value.
#define SET_PWM_COMPARE_VALUE(compareValue)          (OCR0B = compareValue)

//! Macro that clears all Timer/counter0 interrupt flags.
#define CLEAR_ALL_TIMER0_INT_FLAGS    (TIFR0 = TIFR0)

//! Macro that disables all Timer/Counter0 interrupts.
#define DISABLE_ALL_TIMER0_INTS       (TIMSK0 = 0)

//! Macro that enables Timer/Counter0 interrupt where zero crossings are detected.
#define SET_TIMER0_INT_ZC_DETECTION   (TIMSK0 = (1 << TOIE0))

//! Macro that clears all Timer/Counter1 interrupt flags.
#define CLEAR_ALL_TIMER1_INT_FLAGS    (TIFR1 = TIFR1)

//! Macro that disables all Timer/Counter1 interrupts.
#define DISABLE_ALL_TIMER1_INTS       (TIMSK1 = 0)

//! Macro that enable Timer/Counter1 interrupt responsible for commutation.
#define SET_TIMER1_INT_COMMUTATION    (TIMSK1 = (1 << OCIE1A))

//! Macro that enables Timer/Counter1 interrupt responsible for enabling ADC sampling after ADC holdoff period.
#define SET_TIMER1_INT_HOLDOFF        (TIMSK1 = (1 << OCIE1B))

//! External reference voltage in milliVolts.
#define EXTERNAL_REF_VOLTAGE      ((4930UL * 10) / 43)

//! Current measurement shunt value in milliOhm.
#define SHUNT_RESISTANCE          220

//! The ADC resolution used.
#define ADC_RESOLUTION   256

/*!
 *  The number of Timer/Counter1 ticks per second. Equals
 *  MCU clock frequency / Timer/counter1 prescaler.
 */
#define TICKS_PER_SECOND    1000000UL

//! The number of Timer/Counter1 ticks per minute.
#define TICKS_PER_MINUTE    (TICKS_PER_SECOND * 60)

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

//! Uncomment one of the following lines to choose open or closed loop speed control.
#define SPEED_CONTROL_OPEN_LOOP
//#define SPEED_CONTROL_CLOSED_LOOP

//! The current value in milliAmpere where the current limiter starts to kick in.
#define CURRENT_LIMITER_START         2500

//! The current value in milliAmpere where the motor should be shut down.
#define CURRENT_LIMITER_CRITICAL      3000

/*!
 *  Uncomment the following line if the analog comparator should be used to
 *  detect overcurrent.
 */
#define ANALOG_COMPARATOR_ENABLE

/*!
 *  The maximum duty cycles is decreased by this number of PWM duty cycle steps
 *  per milliAmpere current consumption over \ref CURRENT_LIMITER_START.
 */
#define CURRENT_LIMITER_FACTOR   (1 / 5)

//! Macro that cuts all power to the motor.
#define DISABLE_DRIVING               (DRIVE_PORT = 0x00)

//! PWM compare value used during startup.
#define STARTUP_PWM_COMPARE_VALUE  130

//! Zero-cross threshold.
#define ADC_ZC_THRESHOLD 0x98

/*!
 *  The number of milliseconds to subtract from the zero-crossing to commutation time.
 *  Used to compensate for zero-cross sample frequency.
 */
#define COMMUTATION_CORRECTION 50

//! The minimum allowed PWM compare value.
#define MIN_PWM_COMPARE_VALUE    90

//! The maximum allowed PWM compare value.
#define MAX_PWM_COMPARE_VALUE    200

//! The minimum allowed speed. (Only has effect when closed loop speed control is used)
#define MIN_SPEED 3000UL

//! The maximum allowed speed. (Only has effect when closed loop speed control is used)
#define MAX_SPEED 8000UL

//! P-regulator proportional gain.
#define P_REG_K_P 64

//! P-regulator scaling factor. The result is divided by this number.
#define P_REG_SCALING 65536

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
static unsigned long CalculateSpeedSetpoint();
static unsigned int CalculateCurrent();
static signed int SpeedControl(void);
static unsigned char CurrentControl(void);


#endif  // File guard
