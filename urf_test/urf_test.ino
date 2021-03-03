/*
 * Rosserial HC-SR04 Test
 * Demonstrates the rosserial arduino pipeline with a URF
 * Author: Trevor Slack
 * Date: 3/3/2021
 */

#include <ros.h>
#include <std_msgs/Float32MultiArray.h>

// ros node
ros::NodeHandle  urf_node;

std_msgs::Float32MultiArray urf_msg;
ros::Publisher urf_pub("/argos/urf",&urf_msg);
const int array_length = 2;

// Pins
const int TRIG_PIN = 7;
const int ECHO_PIN = 8;
// Max dustance = 400cm
const unsigned int MAX_DIST = 23200;

void setup() {
  // ros node
  urf_node.initNode();
  urf_node.advertise(urf_pub);
  
  // The Trigger pin will tell the sensor to range find
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);

  //Set Echo pin as input to measure the duration of 
  //pulses coming back from the distance sensor
  pinMode(ECHO_PIN, INPUT);

  // We'll use the serial monitor to view the sensor output
//  Serial.begin(9600);
}

void loop() {
  
  unsigned long t1;
  unsigned long t2;
  unsigned long pulse_width;
  float m;

  // Hold the trigger pin high for at least 10 us
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Wait for pulse on echo pin
  while ( digitalRead(ECHO_PIN) == 0 );

  // Measure how long the echo pin was held high (pulse width)
  // Note: the micros() counter will overflow after ~70 min
  t1 = micros();
  while ( digitalRead(ECHO_PIN) == 1);
  t2 = micros();
  pulse_width = t2 - t1;

  // Calculate distance in centimeters and inches. The constants
  // are found in the datasheet, and calculated from the assumed speed
  //of sound in air at sea level (~340 m/s).
  m = pulse_width / 58.0/100.0;
  if ( m > 4 || m<0.02 ) {
    m = -1;
  }

  // add to ros message
  float values[array_length] = {m,-1};
  urf_msg.data_length = array_length;
  urf_msg.data = values;
  urf_pub.publish(&urf_msg);

  delay(60);

  urf_node.spinOnce();
  

}
