#include <ESP32Servo.h>

// Motor pins for Shoulder, Base, Elbow
#define SHOULDER_R_PWM 25  // Forward PWM pin
#define SHOULDER_L_PWM 26  // Reverse PWM pin

#define BASE_R_PWM 33      // Forward PWM pin
#define BASE_L_PWM 32      // Reverse PWM pin

#define ELBOW_R_PWM 13     // Forward PWM pin
#define ELBOW_L_PWM 12     // Reverse PWM pin

// GPIO pins for Roller and Gripper
#define ROLL_MOTOR1 2  
#define ROLL_MOTOR2 4

#define GRIPPER_MOTOR1 19  
#define GRIPPER_MOTOR2 21  

// Servo pin
#define SERVO_PIN 18
Servo servo;

int speed=255;



void setup() {
  Serial.begin(115200);
  
  // Setup roller and gripper pins
  pinMode(SHOULDER_R_PWM, OUTPUT);
  pinMode(SHOULDER_L_PWM, OUTPUT);

  pinMode(BASE_R_PWM, OUTPUT);
  pinMode(BASE_L_PWM, OUTPUT);

  pinMode(ELBOW_R_PWM, OUTPUT);
  pinMode(ELBOW_L_PWM, OUTPUT);


  pinMode(ROLL_MOTOR1, OUTPUT);
  pinMode(ROLL_MOTOR2, OUTPUT);
  pinMode(GRIPPER_MOTOR1, OUTPUT);
  pinMode(GRIPPER_MOTOR2, OUTPUT);
  
  // Setup servo
  servo.attach(SERVO_PIN);
  
  // Initialize all motors to off
  allOff();
  
  Serial.println("Robotic Arm Controller Ready");
}

void loop() {
  if (Serial.available()) {
    char command = Serial.read();
    
    switch(command) {
      case 'a':  // Shoulder forward
        shoulderForward();
        break;
      case 'b':  // Shoulder backward
        shoulderBackward();
        break;
      case 's':  // Stop all
        allOff();
        break;
      case 'c':  // Elbow forward
        elbowForward();
        break;
      case 'd':  // Elbow backward
        elbowBackward();
        break;
      case 'f':  // Base forward
        baseForward();
        break;
      case 'g':  // Base backward
        baseBackward();
        break;
      case 'i':  // Roller forward
        rollerForward();
        break;
      case 'j':  // Roller backward
        rollerBackward();
        break;
      case 'l':  // Gripper forward
        gripperForward();
        break;
      case 'm':  // Gripper backward
        gripperBackward();
        break;
      case 'x':  // Move servo to 0 degrees
        moveServo(0);
        break;
      case 'y':  // Move servo to 90 degrees
        moveServo(90);
        break;
      case 'z':  // Move servo to 180 degrees
        moveServo(180);
        break;
      default:
        Serial.println("Invalid command!");
    }
  }
  delay(100);  // Small delay to avoid flooding
}

// Motor control functions
void shoulderForward() {
  analogWrite(SHOULDER_R_PWM, speed); // Drive motor forward
  analogWrite(SHOULDER_L_PWM, 0);     // Ensure LPWM is off
  Serial.println("Shoulder Forward");
}

void shoulderBackward() {
  analogWrite(SHOULDER_R_PWM, 0);     // Ensure RPWM is off
  analogWrite(SHOULDER_L_PWM, speed); // Drive motor backward
  Serial.println("Shoulder Backward");
}

void shoulderOff() {
  analogWrite(SHOULDER_R_PWM, 0);    
  analogWrite(SHOULDER_L_PWM, 0);
  Serial.println("Shoulder Stopped");
}

void elbowForward() {
  analogWrite(ELBOW_R_PWM, speed);  // Max PWM value
  analogWrite(ELBOW_L_PWM , 0);
  Serial.println("Elbow Forward");
}

void elbowBackward() {
  analogWrite(ELBOW_R_PWM, 0);
  analogWrite(ELBOW_L_PWM , speed);  // Max PWM value
  Serial.println("Elbow Backward");
}

void elbowOff() {
  analogWrite(ELBOW_R_PWM , 0);
  analogWrite(ELBOW_L_PWM , 0);
  Serial.println("Elbow Stopped");
}

void baseForward() {
  analogWrite(BASE_R_PWM, speed);  // Max PWM value
  analogWrite(BASE_L_PWM, 0);
  Serial.println("Base Forward");
}

void baseBackward() {
  analogWrite(BASE_R_PWM, 0);
  analogWrite(BASE_L_PWM, speed);  // Max PWM value
  Serial.println("Base Backward");
}

void baseOff() {
  analogWrite(BASE_R_PWM, 0);
  analogWrite(BASE_L_PWM, 0);
  Serial.println("Base Stopped");
}

void rollerForward() {
  digitalWrite(ROLL_MOTOR1, LOW);
  digitalWrite(ROLL_MOTOR2, HIGH);
  Serial.println("Roller Forward");
}

void rollerBackward() {
  digitalWrite(ROLL_MOTOR1, HIGH);
  digitalWrite(ROLL_MOTOR2, LOW);
  Serial.println("Roller Backward");
}

void rollerOff() {
  digitalWrite(ROLL_MOTOR1, LOW);
  digitalWrite(ROLL_MOTOR2, LOW);
  Serial.println("Roller Off");
}

void gripperForward() {
  digitalWrite(GRIPPER_MOTOR1, LOW);
  digitalWrite(GRIPPER_MOTOR2, HIGH);
  Serial.println("Gripper Forward");
}

void gripperBackward() {
  digitalWrite(GRIPPER_MOTOR1, HIGH);
  digitalWrite(GRIPPER_MOTOR2, LOW);
  Serial.println("Gripper Backward");
}

void gripperOff() {
  digitalWrite(GRIPPER_MOTOR1, LOW);
  digitalWrite(GRIPPER_MOTOR2, LOW);
  Serial.println("Gripper Off");
}

void moveServo(int angle) {
  if (angle < 0 || angle > 180) {
    Serial.println("Invalid angle! Please provide an angle between 0 and 180.");
    return;
  }

  
  servo.write(angle);
  Serial.print("Servo moved to ");
  Serial.print(angle);
  Serial.println(" degrees");
}

void allOff() {
  shoulderOff();
  elbowOff();
  baseOff();
  rollerOff();
  gripperOff();
  Serial.println("All motors stopped");
}
