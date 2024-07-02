#include <ros.h>
#include <std_msgs/Float32.h>
#include <PID_v1.h>
#include <Wire.h>
#include <AS5600.h>
#include <util/atomic.h> // For the ATOMIC_BLOCK macro

#define encoderPinA 2  // Encoder A pin
#define encoderPinB 3  // Encoder B pin
#define BAUD_RATE 115200

double kp = 1, ki = 20, kd = 0;  // PID constants, modify for optimal performance
double input = 0, output = 0, setpoint = 0;
unsigned long lastTime = 0, now = 0;
volatile long pulseCount = 0, lastPulseCount = 0, lastPos = 0;
PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);

ros::NodeHandle nh;
std_msgs::Float32 throttle_wheel_msg;
ros::Publisher throttle_wheel_pub("throttle_wheel_position", &throttle_wheel_msg);

std_msgs::Float32 steer_wheel_msg;
ros::Publisher steer_wheel_pub("steer_wheel_position", &steer_wheel_msg);

AMS_5600 ams5600;

void setup() {
  pinMode(encoderPinA, INPUT);  // Encoder A input
  pinMode(encoderPinB, INPUT);  // Encoder B input
  attachInterrupt(digitalPinToInterrupt(encoderPinA), encoderISR, RISING);  // Update pulse count on change
  
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1);
  myPID.SetOutputLimits(-255, 255);

  nh.initNode();
  nh.advertise(throttle_wheel_pub);
  nh.advertise(steer_wheel_pub);

  Serial.begin(BAUD_RATE);
  Wire.begin();  // Initialize I2C
  // ams5600.begin();  // Initialize AS5600 sensor
}

float mapDegreesToRange(float degrees) {
  /* Mapping 0-360 degrees to -0.4 to 0.4 */
  return ((degrees / 360.0) * 0.8) - 0.4;
}

float convertRawAngleToDegrees(word newAngle) {
  /* Raw data reports 0 - 4095 segments, which is 0.087890625 of a degree */
  float retVal = newAngle * 0.087890625;
  return retVal;
}

void loop() {
  now = millis();
  int timeChange = (now - lastTime);
  if (timeChange >= 500) {
    // Calculate speed in degrees per second
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      input = (360.0 * (pulseCount - lastPos)) /(185.0*(now - lastTime));
      lastTime = now;
      lastPulseCount = pulseCount;
    }
  }

  myPID.Compute();  // Calculate new PID output
  delay(10);  // Small delay for stability

  // Populate the message with data
  throttle_wheel_msg.data = input;
  
  // Read the position from the AS5600 sensor
  float steer_wheel_position = convertRawAngleToDegrees(ams5600.getRawAngle());
  float mappedValue = mapDegreesToRange(steer_wheel_position);
  steer_wheel_msg.data = mappedValue;

  // Publish the messages
  throttle_wheel_pub.publish(&throttle_wheel_msg);
  steer_wheel_pub.publish(&steer_wheel_msg);
  Serial.print("throttle Wheel Output: ");
  Serial.println(input);
  Serial.print("steer Wheel Position: ");
  Serial.println(mappedValue);

  nh.spinOnce();
}

void encoderISR() {  // ISR for Encoder
  int b = digitalRead(encoderPinB);
  if (b > 0) {
    pulseCount++;
  } else {
    pulseCount--;
  }
}
