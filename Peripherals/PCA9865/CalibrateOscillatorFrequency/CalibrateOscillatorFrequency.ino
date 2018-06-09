/*************************************************** 
  This is an example for our Adafruit 16-channel PWM & Servo driver
  Servo test to test the real frequency of the oscillator of the PCA9685.

  CAUTION: DOES NOT USE ANY VOLTAGE HIGHER THAN THE BOARD LIMITS.

  So for ESP8266 remove any 5V input as this setup will feed the Voltage back
  into the board. KABOEM, SMOKE if you use too much VOLTAGE.
  
  Just supply the PCA9685 with 3.3V on V+ and VVC and setup all other
  I2C cables
  Use an extra cable to feedback the signal of the PCA9685 to your board
  For example connect the yellow signal pin 3 (last of first block) to pin 14 of your ESP8266

  Formula:
  prescale = round ( osc_clock / 4096 * update_rate) - 1
  this can be written to osc_clock = (prescale + 1) * 4096 * update_rate
  We will measure the real update_rate to assert the real osc_clock
  
  ***************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
// you can also call it with a different address and I2C interface
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(&Wire, 0x40);

#define FREQUENCY             50
// CURRENT_OSCIFREQ is not used in any calculations. Just for information
//#define CURRENT_OSCIFREQ      25000000  // original
//#define CURRENT_OSCIFREQ      27500000  // 10% issue 11 correction
#define CURRENT_OSCIFREQ      26075000  // 4.3% Bolukan's correction

// CAUTION: ONLY CONNECT server and ESP WITHOUT 5V ON V+ or green breakout supply pins. Use 3.3V on V+
#define PIN_SERVO_FEEDBACK     3 // Connect 4th (no 3) Yellow pin (0 reserved for servo, 3 for feedback-test)
#define PIN_BOARD_FEEDBACK    14 // 14 => D5 on NodeMCU

uint8_t prescale = 0;
// loop
#define INTERVAL   1000  // 1 sec
int32_t lastEvaluation = 0;
uint16_t frozenCounter = 0;
uint16_t countDeviations = 0;

uint32_t totalCounter = 0;
uint32_t totalTime = 0;   // in millis
uint32_t realOsciFreq = 0;
uint32_t multiplier = 4096;

// interrupt
volatile uint16_t interruptCounter = 0;

void handleInterrupt() {
  interruptCounter++;
}

void setup() {
  Serial.begin(115200);
  Serial.println("PCA9685 Oscillator test");

  // disable LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // set PCA9685
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);             // Set some frequency
  pwm.setPWM(PIN_SERVO_FEEDBACK,0,2048); // half of time high, half of time low

  prescale = pwm.readPrescale();         // read prescale
  Serial.printf("Target frequency: %u\n", FREQUENCY);
  Serial.printf("Applied prescale: %u\n", prescale);
  Serial.printf("Current Oscillator frequency in Library: %u\n", CURRENT_OSCIFREQ);
  
  // prepare interrupt on ESP pin
  pinMode(PIN_BOARD_FEEDBACK, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_BOARD_FEEDBACK), handleInterrupt, RISING);

  // take a breath and reset to zero
  delay(10);
  interruptCounter = 0;
  lastEvaluation = millis();
}

void loop() {
  if (millis() - lastEvaluation > INTERVAL)
  { 
    // first freeze counters and adjust for new round
    frozenCounter = interruptCounter; // first freeze counter
    interruptCounter -= frozenCounter; 
    lastEvaluation += INTERVAL;

    totalCounter += frozenCounter;
    totalTime += 1;
    
    // only print deviations from targetted frequency
    //if (frozenCounter != FREQUENCY)
    {
       // this is an ugly hack to have maximum precision in 32 bits before doing the division
       multiplier = 4096;
       realOsciFreq = (prescale + 1) * totalCounter;
       while (((realOsciFreq & 0x80000000) == 0) && (multiplier != 1))
       {
          realOsciFreq <<= 1;
          multiplier >>= 1;
       }
       realOsciFreq /= totalTime;
       if (multiplier) realOsciFreq *= multiplier;
       
       countDeviations++;
       Serial.printf("%4u", countDeviations);
       Serial.printf(" Timestamp: %4u ", totalTime);
       Serial.printf(" Freq: %4u ", frozenCounter);
       Serial.printf(" Counter: %6u ", totalCounter);
       Serial.printf(" calc.osci.freq: %9u\n",realOsciFreq);
    }
  }
 
}
