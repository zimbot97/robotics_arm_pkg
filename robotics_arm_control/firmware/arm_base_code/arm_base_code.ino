#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// gripper 180 close, 120 open
const int NUM_SERVOS = 6;
int servoChannels[NUM_SERVOS] = {0, 1, 2, 3, 4, 5}; // PCA9685 channels

// Servo calibration (adjust for your servos)
#define SERVOMIN  120  // pulse length for 0 degrees
#define SERVOMAX  720  // pulse length for 180 degrees

String inputString = "";   // buffer for incoming serial data

void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setPWMFreq(60);  // typical servos run at 50 Hz

  // Initialize all servos at 90°
  for (int i = 0; i < NUM_SERVOS; i++) {
    if(i == 5)
    {
      setServoAngle(servoChannels[i], 180);
    }
    else
    {
      setServoAngle(servoChannels[i], 90);
    }
  }
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.println("Ready: expecting [v1,v2,v3,v4,v5,v6]");
  // Read serial input as it arrives
  while (!Serial.available()) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(800);
    digitalWrite(LED_BUILTIN, LOW);
    delay(800);
  }
}

void loop() {

  // Once here, data is available → LED solid ON
  digitalWrite(LED_BUILTIN, HIGH);

  while (Serial.available()) {
    char inChar = (char)Serial.read();

    if (inChar == '\n') {  
      // End of message → process array
      parseAndUpdate(inputString);
      inputString = ""; // clear buffer
    } else {
      inputString += inChar;
    }
  }
}

void parseAndUpdate(String data) {
  data.trim();

  // Ensure data is wrapped with [ ]
  if (data.startsWith("[") && data.endsWith("]")) {
    data.remove(0, 1);                // remove '['
    data.remove(data.length() - 1);   // remove ']'
  }

  int values[NUM_SERVOS];
  int idx = 0;

  // Split string into tokens
  char buf[100];
  data.toCharArray(buf, sizeof(buf));
  char *token = strtok(buf, ",");
  while (token != NULL && idx < NUM_SERVOS) {
    values[idx] = constrain(atoi(token), 0, 180); // limit 0–180
    idx++;
    token = strtok(NULL, ",");
  }

  // If we got exactly NUM_SERVOS values → update servos
  if (idx == NUM_SERVOS) {
    for (int i = 0; i < NUM_SERVOS; i++) {
      setServoAngle(servoChannels[i], values[i]);
    }

    // Debug feedback
    Serial.print("Updated: [");
    for (int i = 0; i < NUM_SERVOS; i++) {
      Serial.print(values[i]);
      if (i < NUM_SERVOS - 1) Serial.print(", ");
    }
    Serial.println("]");
  } else {
    Serial.println("⚠️ Invalid data received");
  }
}

void setServoAngle(int channel, int angle) {
  int pulselen;
  if(channel == 5)
  {
    if(angle >= 100)
    {
      pulselen = map(angle, 0, 180, SERVOMIN, SERVOMAX);
      pwm.setPWM(channel, 0, pulselen);
    }
    else
    {
      Serial.println("⚠️ Invalid gripper angle received");
    }
  }
  else
  {
    pulselen = map(angle, 0, 180, SERVOMIN, SERVOMAX);
    pwm.setPWM(channel, 0, pulselen);
  }
  
}
