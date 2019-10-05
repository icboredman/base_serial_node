/**********************************************************
 * This sketch runs on Arduino Leonardo embedded within LattePanda board.
 * It receives a one byte temperature over serial connection and
 * uses it to control a cooling fan through a PWM on pin 9.
 * 
 * LED blinking rate serves as a visual indicator of current temperature.
 * 
 * Fast PWM code is based on work of Vicente Jiménez:
 * (http://r6500.blogspot.com/2014/12/fast-pwm-on-arduino-leonardo.html)
 * uses TIMER 1 to generate high speed PWM on pin 9 of Leonardo.
 * 
 * (c) boredman@BoredomProjects.net
 ***********************************************************/

int LED = 13;


// Frequency modes for TIMER1
#define PWM62k   1   //62500 Hz
#define PWM8k    2   // 7812 Hz
#define PWM1k    3   //  976 Hz
#define PWM244   4   //  244 Hz
#define PWM61    5   //   61 Hz

// Direct PWM change variables
#define PWM9   OCR1A

// Configure the PWM clock
// The argument is one of the 5 previously defined modes
void pwm91011configure(int mode)
{
  // TCCR1A configuration
  //  01 : Fast PWM 8 bit
  TCCR1A=1;

  // TCCR1B configuration
  // Clock mode and Fast PWM 8 bit
  TCCR1B=mode|0x08;  

  // TCCR1C configuration
  TCCR1C=0;
}

// Set PWM to D9
// Argument is PWM between 0 and 255
void pwmSet9(int value)
{
  OCR1A=value;   // Set PWM value
  DDRB|=1<<5;    // Set Output Mode B5
  TCCR1A|=0x80;  // Activate channel
}

// Macro to converts from duty (0..100) to PWM (0..255)
#define DUTY2PWM(x)  ((255*(x))/100)


//*****************************************************
// Arduino SETUP function
//*****************************************************
void setup()
{
  // Configure Timer 1 (Pins 9, 10 and 11)
  // Valid options are: 
  //      PWM62k, PWM8k, PWM1k, PWM244 and PWM61
  pwm91011configure(PWM61);

  // Prepare pin 9 to use PWM
  // We need to call pwm91011configure before
  // For now, we set it at 0%
  pwmSet9(0);

  pinMode(LED, OUTPUT);

  delay(5000);

  Serial.begin(115200);
  // serial connection will be established after
  // the calling program opens the port.
  while (!Serial)
  {
    digitalWrite(LED, HIGH);
    delay(50);
    digitalWrite(LED, LOW);
    delay(50);
  }

}


//*****************************************************
// Calculates led blinking period in [ms]
//  based on temperature in [°C]
//*****************************************************
int Temp2period(int temp)
{
  float perd_max = 2000.0;  // 0.5 Hz
  float perd_min = 200.0;   // 5 Hz
  int temp_min = 30;
  int temp_max = 80;

  if (temp >= temp_max)
    return (int)perd_min;
  else if (temp <= temp_min)
    return (int)perd_max;
  else
  {
    float perd = perd_max - (temp - temp_min) * (perd_max - perd_min) / (temp_max - temp_min);
    return (int)perd;
  }
}


//*****************************************************
// Calculates Fan pwm value
//  based on temperature in [°C].
// Applies a small hysteresis on the lower threshold
//*****************************************************
int Temp2pwm(int temp)
{
  float pwm_min = 150.0;
  float pwm_max = 255.0;
  int temp_min = 55;
  int temp_max = 70;
  int temp_hyst = 3;
  static int hyst_state = 1;

  if (temp <= (temp_min + hyst_state * temp_hyst))
  {
    hyst_state = 1;
    return 0;
  }
  else
  {
    hyst_state = -1;
    float pwm = pwm_min + (temp - temp_min) * (pwm_max - pwm_min) / (temp_max - temp_min);
    if (pwm >= pwm_max)
      return (int)pwm_max;
    else
      return (int)pwm;
  }
}


//*****************************************************
// Arduino LOOP function
//*****************************************************
void loop()
{
  static int blink_period_ms = 0;
  static unsigned long time_now = 0;

  if (Serial.available())
  {
    int t = Serial.read();

    // calculate blinking rate
    blink_period_ms = Temp2period(t);

    // Set PWM on pin 9
    PWM9 = Temp2pwm(t);
  }

  if (millis() > time_now + blink_period_ms)
  {
    time_now = millis();
    digitalWrite(LED, HIGH);
    delay(25);
    digitalWrite(LED, LOW);
  }
}

