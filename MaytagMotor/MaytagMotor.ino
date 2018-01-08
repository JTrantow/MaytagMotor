/*!
 * Variable speed three phase induction motor controlled by torque pwm using tach feedback signal from Maytag motor controller.
 * See https://hackaday.io/project/28630-variable-speed-washer-motor-and-controller-reuse
 * 
 * Input: MTRY_CTRL_TACH 4 pulses per revolution. This signal is optocoupled and requires pull up (and maybe some filtering).
 * Output: TORQUE_PWM This signal probably drives the diode of an optocoupler. 5V seems to work nicely. (measured 7 volts) 130 hz frequency. Low duty corresponds to low torgue applied. Must be >3% to start spinning. Direction of rotation depends on whether this signals starts from high or low state.

 This is just the beginning of the motor control.
 The plan is to connect the MTR_CTRL_TACH to a PID loop to drive TORQUE_PWM. 
 I will probably have an encoder to select the direction and RPM and an LCD to display desired and measured RPM and duty cycle. 

 The longer term goal is to replace Arduino by integrating into LinuxCNC/Machinekit.

 This currently is open loop control that changes direction on my system. (nothing connected to my motor)
 If your motor is connected to a load you will need to change test_percentage and the delays.
 */

const int TORQUE_PWM_pin = 11; //!< Pin connected to the motor controller TORQUE_PWM input.
const int MTR_CTRL_TACH_pin = 2; //!< Pin connected to the motor controller MTR_CTRL_TACH input.  Pin 2 specifically chosen for interrupt use.

const unsigned int pulses_per_revolution = 4; //!< Number of tach pulses per motor shaft rotation.
/*
 * Use interrupts to measure the tach period.
 */
volatile unsigned long rise_time = 0UL; // Measures the period from rising edge until rising edge.
volatile bool rise_data_updated = false;

/*! 
 * \brief Interrupt handler for rising edge of MTR_CTRL_TACH_pin.
 * 
 * Updates rise_time and rise_data_updated variables.
 * These values only update on the rising edge of the pulse.
 * Check the rise_data_update status to make sure there is new data available.
 */
void rising() {
  static unsigned long last_rise = 0UL;
  
  rise_time = micros() - last_rise;
  last_rise += rise_time;
  rise_data_updated = true;
}
 

/*!
 * \brief Put your setup code , to run once here.
 */
void setup() {

  Serial.begin(115200); 
  Serial.println("setup().");

  pinMode(TORQUE_PWM_pin, OUTPUT);   
  pinMode(MTR_CTRL_TACH_pin, INPUT_PULLUP);   

  // Use interrupts pin D2 to measure the tach pulse.
  attachInterrupt(0, rising, RISING);   // When pin D2 goes high, the rising function interrupt will run.
 
  /* 
   *  Change the PWM frequency for the PWMpin close to the 130 hz frequency used by the original Maytag control circuit.
   *  The PWM needs to be set depending on the board and pin.
   *  See https://arduino-info.wikispaces.com/Arduino-PWM-Frequency for details.
   */
  #if (1)
    //For Arduino Mega1280, Mega2560, MegaADK, Spider or any other board using ATmega1280 or ATmega2560**
     
    //---------------------------------------------- Set PWM frequency for D11 & D12 -----------------------------
    //TCCR1B = TCCR1B & B11111000 | B00000011;    // set timer 1 divisor to    64 for PWM frequency of   490.20 Hz
    TCCR1B = TCCR1B & B11111000 | B00000100;    // set timer 1 divisor to   256 for PWM frequency of   122.55 Hz
  #endif  
}


/*!
 *  \brief Arduino Processing main loop.
 *  
 *  This function spins the motor and displays the RPM of the motor.
 */
void loop() {
  static uint8_t pwm_out = 0U;
  const float test_percent = 5.0; // Test percentage we ramp up to.
  unsigned int i;

  unsigned long high_time;
  
  const uint16_t direction_hold_msec = 200U; //!< Number of milliseconds to hold TORQUE_PWM to set the rotation direction. 
  const uint16_t spin_msec = 1200U;          //!< Time to apply test_percent to torque signal.
  const uint16_t wait_for_spin_stop_msec = 10000U; //!<  Make sure this is long enough to stop spinning.
  

#if (1)
  // This is enough to spin the motor in each direction.
  
  analogWrite(TORQUE_PWM_pin, 0);   // Set direction
  delay(direction_hold_msec);
  analogWrite(TORQUE_PWM_pin, (uint8_t)(test_percent * 2.56)); // Enough torque duty to get it spinning.
  delay( spin_msec);
  analogWrite(TORQUE_PWM_pin, 0); // Turn it off.
  
  /*
   * The MTR_CTRL_TACH_pin signal is noisy (16khz) while the motor is being driven.
   * Values don't settle down until 140-150 msec after I turn off the PWM.
   * Signal needs some conditioning???
   */
  // Read after pulses stop.
  
  delay(100);
  for (i=0;i<20;i++) {
  if (rise_data_updated) {
    Serial.print("CW rise ");
    Serial.print(rise_time);
    Serial.print(" frequency ");
    Serial.println((1.0E6/pulses_per_revolution)/rise_time);
    
    rise_data_updated = false;
  } else {
    Serial.println("No new pulses.");    
  }
    delay(5);
  }
 
  delay(wait_for_spin_stop_msec); // Make sure this is long enough to stop spinning.

  // Spin in opposite direction.
  analogWrite(TORQUE_PWM_pin, 255);   // Set direction
  delay(direction_hold_msec);
  analogWrite(TORQUE_PWM_pin, (uint8_t)(test_percent * 2.56)); // Enough torque duty to get it spinning.
  delay( spin_msec);
  analogWrite(TORQUE_PWM_pin, 0); // Turn it off.
  high_time = pulseIn(MTR_CTRL_TACH_pin, HIGH, 1000U);  
  delay(100);
  for (i=0;i<20;i++) {    
    if (rise_data_updated) {
      Serial.print("CCW rise ");
      Serial.print(rise_time);
      Serial.print(" frequency ");
      Serial.println((1.0E6/pulses_per_revolution)/rise_time);
      
      rise_data_updated = false;
    } else {
      Serial.println("No new pulses.");    
    }
    delay(5);
}
  delay(wait_for_spin_stop_msec); // Make sure this is long enough to stop spinning.
#else
  /*
   * Just display the tach values. (useful when function generator is connected to tach input.
   */
  if (rise_data_updated) {
    Serial.print("rise ");
    Serial.print(rise_time);
    Serial.print(" frequency ");
    Serial.println((1.0E6/pulses_per_revolution)/rise_time);
    
    rise_data_updated = false;
  } else {
    Serial.println("No new pulses.");    
  }
  
  delay(1000);
#endif
}
