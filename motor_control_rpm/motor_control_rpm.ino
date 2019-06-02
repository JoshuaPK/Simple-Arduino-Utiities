
/*  Motor Control Program- a simple motor control program for Arduino, intended
 *  to test motors with tachometers.
 *  
 *  Helpful Links:
 * 
 *  https://gonium.net/md/2006/12/20/handling-external-interrupts-with-arduino/ 
 *  https://playground.arduino.cc/Main/ReadingRPM
 *  https://dzone.com/articles/arduino-and-raspberry-pi-working-together-part-2-now-with-i2
 *  
 *  Using the SainSmart 1602 LCD/Keypad shield.  Note that this has slightly
 *  different values than other LCD keypads of similar design.
 *  
 *  LCD Pins: 8, 13, 9, 4, 5, 6, 7 for the SainSmart 1602
 *  
 */

#include <avr/io.h>
#include <avr/interrupt.h>

#include <LiquidCrystal.h>

LiquidCrystal lcd(8, 13, 9, 4, 5, 6, 7);

/* Tachometer variables: */

volatile unsigned int hal_pulses = 0;
unsigned long timeold = 0;
unsigned long ms_since_last = 0;
unsigned int rpm = 0;
int motor_ppr = 6;  // Pulses Per Revolution from HAL sensor

/* Keypad and control variables: */

int control_key = 0;
int target_rpm = 0;
int prev_target = 0;
int motor_rev = 0;
int motor_enable_pin = 10;
int motor_pwm_pin = 11;
int motor_direction_pin = 0;
int sense_pin = 1;  //INT2
int adc_key_pin = 0;
int adc_key_in = 0;
int lcd_key = 0;

char rpm_msg[8];

#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNONE   5


int read_LCD_buttons()
{
 adc_key_in = analogRead(adc_key_pin);
 if (adc_key_in > 1500) return btnNONE; 
 if (adc_key_in < 50)   return btnRIGHT;  
 if (adc_key_in < 195)  return btnUP; 
 if (adc_key_in < 380)  return btnDOWN; 
 if (adc_key_in < 510)  return btnLEFT; 
 if (adc_key_in < 750)  return btnSELECT;   
 return btnNONE;
}

void setup() {

  pinMode (motor_enable_pin, OUTPUT);
  pinMode (motor_pwm_pin, OUTPUT);
  pinMode (motor_direction_pin, OUTPUT);
  pinMode (sense_pin, INPUT_PULLUP);
  pinMode (adc_key_pin, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(sense_pin), cnt_rev, FALLING);

  lcd.begin(16, 2);
  lcd.setCursor(0,0);
  lcd.print("Motor Speed:");

  lcd.setCursor(6, 1);
  lcd.print("Tgt:");

  lcd.setCursor(15, 0);
  lcd.print("F");

  digitalWrite(motor_enable_pin, 1);
 
}

void loop() {

  // Display RPM and zero out counter:

  delay(500);
  noInterrupts();
  
  if (hal_pulses >= 24)
  {
    ms_since_last = ((millis() - timeold));
    timeold = millis();
    rpm = 60000 / ms_since_last * (hal_pulses / motor_ppr);
    hal_pulses = 0;

    sprintf(rpm_msg, "%5d", rpm);
    
    lcd.setCursor(0, 1);
    lcd.print (rpm_msg);
  }

  interrupts();

  // See if we need to adjust any controls:

  control_key = read_LCD_buttons();
  if (control_key == btnUP) inc_rpm();
  if (control_key == btnDOWN) dec_rpm();
  if (control_key == btnLEFT) reverse_mtr();

  // Send out the control signals:

  if (target_rpm != prev_target) {
    lcd.setCursor(11, 1);
    lcd.print("    ");
    lcd.setCursor(11, 1);
    lcd.print(target_rpm);
    prev_target=target_rpm;
  }

  analogWrite(motor_pwm_pin, target_rpm);
  digitalWrite(motor_direction_pin, motor_rev);
  
}

void cnt_rev()
{
  hal_pulses++;
}

void inc_rpm()
{
  if (target_rpm < 240)
    target_rpm += 5;
  delay(500);
}

void dec_rpm()
{
  if (target_rpm > 0)
    target_rpm -= 5;
  delay(500);
}

void reverse_mtr()
{
  if (motor_rev == 0)
  {
    lcd.setCursor(15, 0);
    lcd.print("R");
    motor_rev = 1;
    delay(500);
    return;
  }
  else
  {
    lcd.setCursor(15, 0);
    lcd.print("F");
    motor_rev = 0;
    delay(500);
    return;
  }
}
