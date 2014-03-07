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

#include "BLDC.h"

#include <ioavr.h>
#include <inavr.h>

//! Array of power stage enable signals for each commutation step.
unsigned char driveTable[6];

//! Array of ADC channel selections for each commutation step.
unsigned char ADMUXTable[6];

//! Array holding the intercommutation delays used during startup.
unsigned int startupDelays[STARTUP_NUM_COMMUTATIONS];

/*! \brief Filtered commutation timer variable and speed indicator.
 *  This value equals half the time of one commutation step. It is filtered
 *  through an IIR filter, so the value stored is not the most recent measuremnt.
 *  The variable is stored in registers R14-R15 for quicker access.
 */
__regvar __no_init volatile unsigned int filteredTimeSinceCommutation @14;

/*! \brief The power stage enable signals that will be output to the motor drivers
 *  at next commutation.
 *
 *  This variable holds the pattern of enable signals that will be output to the
 *  power stage at next commutation. It is stored in register R13 for quick access.
 */
__regvar __no_init volatile unsigned char nextDrivePattern @13;

/*! \brief Polarity of the expected zero crossing.
 *
 *  The polarity of the expected zero crossing.
 *  Could be eiter \ref EDGE_FALLING or \ref EDGE_RISING.
 */
__regvar __no_init volatile unsigned char zcPolarity @ 12;

/*! \brief The commutation step that starts at next commutation.
 *
 *  The commutation step that starts at next commutation. This is used to keep
 *  track on where in the commutation cycle we are. Stored in register R11 for
 *  quick access
 */
__regvar __no_init volatile unsigned char nextCommutationStep @11;

//! ADC reading of external analog speed reference.
volatile unsigned char speedReferenceADC;

//! ADC reading of shunt voltage.
volatile unsigned char shuntVoltageADC = 0;

//! ADC reading of the known external reference voltage.
volatile unsigned char referenceVoltageADC;

//! Flag that specifies whether a new external speed reference and a motor speed measurement is available.
volatile unsigned char speedUpdated = FALSE;

//! Flag that specifies whether a new current measurement is available.
volatile unsigned char currentUpdated = FALSE;


/*! \brief Program entry point.
 *
 *  Main initializes all peripheral units used and calls the startup procedure.
 *  All commutation control from that point is done in interrupt routines.
 */
void main(void)
{
  // Initialize all sub-systems.
  ResetHandler();
  InitPorts();
  InitTimers();
  InitADC();
  MakeTables();
  InitAnalogComparator();

  // Run startup procedure.
  StartMotor();

  // Turn on watchdog for stall-detection.
  WatchdogTimerEnable();
  __enable_interrupt();

  for(;;)
  {
    PWMControl();
  }
}


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
  __eeprom unsigned static int restartAttempts;
  // Temporary variable to avoid unnecessary reading of volatile register MCUSR.
  unsigned char tempMCUSR;

  tempMCUSR = MCUSR;
  MCUSR = tempMCUSR & ~((1 << WDRF) | (1 << BORF) | (1 << EXTRF) | (1 << PORF));

  // Reset watchdog to avoid false stall detection before the motor has started.
  __disable_interrupt();
  __watchdog_reset();
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

  // Init PORTD for PWM on PD5.
  DDRD = (1 << PD5);

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
  TIFR0 = TIFR0;
  TIMSK0 = (0 << TOIE0);

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
  __disable_interrupt();
  __watchdog_reset();

  WDTCSR |= (1 << WDCE) | (1 << WDE);

  WDTCSR = (1 << WDIF) | (1 << WDIE) | (1 << WDE) | (1 << WDP2);
  __enable_interrupt();
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


/*! \brief Timer/counter0 bottom overflow. Used for zero-cross detection.
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
#pragma vector=TIMER0_OVF_vect
__interrupt void MotorPWMBottom()
{
  unsigned char temp;

  // Disable ADC auto-triggering. This must be done here to avoid wrong channel being sampled on manual samples later.
  ADCSRA &= ~((1 << ADATE) | (1 << ADIE));

  // Wait for auto-triggered ADC sample to complete.
  while (!(ADCSRA & (1 << ADIF)))
  {

  }
  temp = ADCH;
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
    DISABLE_ALL_TIMER0_INTS;

    // Read speed reference.

    // Make sure that a sample is not in progress.
    while (ADCSRA & (1 << ADSC))
    {

    }
    // Change channel
    ADMUX = ADMUX_SPEED_REF;

    // Start conversion manually.
    ADCSRA |= (1 << ADSC);

    // Wait for conversion to complete.
    while((ADCSRA & (1 << ADSC)))
    {

    }
    speedReferenceADC = ADCH;

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
#pragma vector=TIMER1_COMPA_vect
__interrupt void Commutate()
{
  // Commutate and clear commutation timer.
  DRIVE_PORT = nextDrivePattern;
  TCNT1 = 0;

  zcPolarity = nextCommutationStep & 0x01;

  // Set zero-cross detection holdoff time.
  CLEAR_ALL_TIMER1_INT_FLAGS;
  OCR1B = ZC_DETECTION_HOLDOFF_TIME_US;
  SET_TIMER1_INT_HOLDOFF;

  __watchdog_reset();
}


/*! \brief Enables zero-cross detection.
 *
 *  This interrupt service routine is triggered when the zero cross detection
 *  hold-off time after commutation is over. All Timer/counter1 interrupts are
 *  disabled and Timer/counter0 (PWM) overflow interrupt is enabled to allow
 *  the ADC readings to be used for zero-cross detection.
 */
#pragma vector=TIMER1_COMPB_vect
__interrupt void EnableZCDetection()
{
  // Enable TCNT0 overflow ISR.
  CLEAR_ALL_TIMER0_INT_FLAGS;
  CLEAR_ALL_TIMER1_INT_FLAGS;
  SET_TIMER0_INT_ZC_DETECTION;
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
#pragma vector=ADC_vect
__interrupt void CurrentMeasurementComplete()
{
  shuntVoltageADC = ADCH;
  currentUpdated = TRUE;
  CLEAR_ALL_TIMER0_INT_FLAGS;
}


/*! \brief Watchdog interrupt
 *
 *  This ISR is called before the watchdog timer resets the device because of a stall.
 *  It simply disables driving, but other tasks that must be done before a watchdog reset,
 *  such as storage of variables in non-volatile memory can be done here.
 */
#pragma vector=WDT_vect
__interrupt void WatchdogISR()
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
#pragma vector=ANA_COMP_vect
__interrupt void OverCurrentISR()
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



#ifdef SPEED_CONTROL_CLOSED_LOOP
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
#endif

#ifdef SPEED_CONTROL_OPEN_LOOP
static void PWMControl(void)
{
  // Only update duty cycle if a new speed reference measurement has been made. (Done right after speed measurement is ready)
  if (speedUpdated)
  {
    // Calculate duty cycle from speed reference value.
    SET_PWM_COMPARE_VALUE(MIN_PWM_COMPARE_VALUE + speedReferenceADC * (MAX_PWM_COMPARE_VALUE - MIN_PWM_COMPARE_VALUE) / ADC_RESOLUTION);
  }
}
#endif


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
  __disable_interrupt();
  filteredTimeSinceCommutationCopy = filteredTimeSinceCommutation;
  __enable_interrupt();

  /*
  filteredTimeSinceCommutation is one half commutation time. Must be multiplied by 12 to get
  one full rotation.
  */
  rotationPeriod = (unsigned long)filteredTimeSinceCommutationCopy * 12;
  speed = (TICKS_PER_MINUTE / rotationPeriod);

  return speed;
}


/*! \brief Calculates the speed set-point in electrical RPM.
 *
 *  This function calculates the speed set-point from the global variable
 *  speedReferenceADC.
 *
 *  In this implementation, the speed reference values from 0x00 to 0xff are
 *  linearly mapped into the allowable speed range, set by \ref MIN_SPEED and
 *  \ref MAX_SPEED.
 */
static unsigned long CalculateSpeedSetpoint()
{
  return (MIN_SPEED + ((MAX_SPEED - MIN_SPEED) * (unsigned int)speedReferenceADC) / ADC_RESOLUTION);
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
  unsigned long speedSetpoint;
  unsigned long currentSpeed;
  signed long speedError;
  signed long dutyChange;



  speedSetpoint = CalculateSpeedSetpoint();
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
    DRIVE_PORT = 0x00;
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


