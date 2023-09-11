#include <MsTimer2.h>

const byte outPin = 13; // Output pin: digital pin 13(D13)
//const byte interruptPin1 = 2; // Interrupt pin: D2
//const byte interruptPin2 = 3; // Interrupt pin: D2
const byte encoder1_A = 2; // Interrupt pin: D2
const byte encoder1_B = 3; // Interrupt pin: D2
const byte encoder2_A = 18; // Interrupt pin: D2
const byte encoder2_B = 19; // Interrupt pin: D2
const byte resetPin = 5;
volatile byte state = 0;
unsigned long cnt1 = 0; 
unsigned long cnt2 = 0; 
unsigned long cnt1_old = 0; 
unsigned long cnt2_old = 0; 
//long cnt1 = 0; 
//long cnt2 = 0; 
double pulse_to_distance_left = 0.2/515; 
double pulse_to_distance_right = 0.2/935;
const double odom_left = cnt2 * pulse_to_distance_left;
const double odom_right = cnt1 * pulse_to_distance_right;
double heading(double x, double y)
{
  double head = atan2(y,x);
  return head;
}
void msTimer2_ISR()
{
  double odom_left_delta;
  double odom_right_delta;
  double theta_delta= 0.0;
  double wheel_track = 0.2;
  odom_left_delta = (cnt1 - cnt1_old) * pulse_to_distance_left;
  odom_right_delta = (cnt2 - cnt2_old) * pulse_to_distance_right;

  //theta_delta = (odom_right_delta - odom_left_delta)/wheel_track;
  theta_delta = heading(wheel_track,(odom_right_delta - odom_left_delta));
  Serial.print("odom_left:   ");
  Serial.print(odom_left);
  Serial.print("odom_right: ");
  Serial.println(odom_right);
  theta_delta = theta_delta*180.0/3.14150;
  Serial.print("theta_delta: ");
  Serial.print(theta_delta);
  Serial.println("");
  
  cnt1_old = cnt1;
  cnt2_old = cnt2;
}
void setup()
{
  pinMode(outPin, OUTPUT); // Output mode
  //  pinMode(interruptPin1, INPUT_PULLUP); // Input mode, pull-up
  //  pinMode(interruptPin2, INPUT_PULLUP); // Input mode, pull-up
  pinMode(encoder1_A, INPUT_PULLUP);
  pinMode(encoder1_B, INPUT_PULLUP);
  pinMode(encoder2_A, INPUT_PULLUP);
  pinMode(encoder2_B, INPUT_PULLUP);
  pinMode(resetPin, INPUT);
  //  attachInterrupt(digitalPinToInterrupt(interruptPin1), intfunc1, RISING); // Enable interrupt
  //  attachInterrupt(digitalPinToInterrupt(interruptPin2), intfunc2, RISING); // Enable interrupt
  attachInterrupt(digitalPinToInterrupt(encoder1_A), intfunc1, RISING); // Enable interrupt
  attachInterrupt(digitalPinToInterrupt(encoder2_A), intfunc2, RISING); // Enable interrupt

  Serial.begin(115200);
  MsTimer2::set(100,msTimer2_ISR);
  MsTimer2::start();
}

/*
  void intfunc1() // Interrupt function
  {
  cnt1++;
  if (state == 0) // If D4 output is low
  {
    digitalWrite(outPin, HIGH);
    state = 1;
  }
  else
  {
    digitalWrite(outPin, LOW);
    state = 0;
  }
  }

  void intfunc2() // Interrupt function
  {
  cnt2++;
  if (state == 0) // If D4 output is low
  {
    digitalWrite(outPin, HIGH);
    state = 1;
  }
  else
  {
    digitalWrite(outPin, LOW);
    state = 0;
  }
  }

*/
void intfunc1()
{
  if (digitalRead(encoder1_A) == digitalRead(encoder1_B))
  {
    cnt1++;
  }
  else
  {
    cnt1--;
  }
}

void intfunc2()
{
  if (digitalRead(encoder2_B) == LOW)
  {
    cnt2--;
  }
  else
  {
    cnt2++;
  }
}

void loop()
{
  const double odom_left = cnt2 * pulse_to_distance_left;
  const double odom_right = cnt1 * pulse_to_distance_right;
  /*
    Serial.print("cnt1: ");
    Serial.print(cnt1);
    if (digitalRead (resetPin) == HIGH)
    {
     cnt1 = 0;
    }

    Serial.print("   cnt2: ");
    Serial.println(cnt2);
    if (digitalRead (resetPin) == HIGH)
    {
     cnt2 = 0;
    }
    delay(1000);
  
  Serial.print("cnt1: ");
  Serial.print(cnt1);
  Serial.print("   cnt2: ");
  Serial.println(cnt2);
  */
}
