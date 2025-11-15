//1- first working 
// #include <Servo.h>

// Servo myServo;  // Create servo object

// void setup() {
//   myServo.attach(3);  // Attach the servo signal pin to digital pin 3
//   myServo.write(30);  // Move servo to 30 degrees
// }

// void loop() {
//   // Nothing here — servo stays at 30 degrees
// }

// 2 - second working code better #include <Servo.h>

// Servo myServo;  // Create a servo object
// int angle;      // Variable to store the input angle

// void setup() {
//   Serial.begin(9600);     // Start serial communication
//   myServo.attach(3);      // Attach servo signal to pin 3
//   Serial.println("Enter angle (0–180):");
// }

// void loop() {
//   // Check if data is available from Serial Monitor
//   if (Serial.available() > 0) {
//     angle = Serial.parseInt();  // Read integer input
//     if (angle >= 0 && angle <= 180) {
//       myServo.write(angle);     // Move servo to input angle
//       Serial.print("Servo moved to: ");
//       Serial.print(angle);
//       Serial.println(" degrees");
//     } else {
//       Serial.println("Invalid angle! Enter a value between 0 and 180.");
//     }
//     delay(500);
//     Serial.println("Enter next angle (0–180):");
//   }
// }




//3-Best working code 1 motor

// #include <Servo.h>

// Servo myServo;
// int angle = 0;

// void setup() {
//   Serial.begin(9600);
//   myServo.attach(3);
//   Serial.println("Enter angle (0–180):");
// }

// void loop() {
//   if (Serial.available() > 0) {
//     int input = Serial.parseInt();
//     if (Serial.read() == '\n' || Serial.read() == '\r') { } // clear newline
//     if (input >= 0 && input <= 180) {
//       angle = input;
//       myServo.write(angle);
//       Serial.print("Moved to: ");
//       Serial.println(angle);
//     } else {
//       Serial.println("Invalid angle (0–180)");
//     }
//   }
// }



//4-Best working code 2 motor
// #include <Servo.h>

// Servo servo1;  // First servo
// Servo servo2;  // Second servo

// int servoNum = 0;
// int angle = 0;

// void setup() {
//   Serial.begin(9600);
//   servo1.attach(3);  // Servo 1 control pin
//   servo2.attach(5);  // Servo 2 control pin
//   Serial.println("Enter: <servo_number> <angle>");
//   Serial.println("Example: '1 90' or '2 45'");
// }

// void loop() {
//   if (Serial.available() > 0) {
//     servoNum = Serial.parseInt();  // First number = servo number
//     angle = Serial.parseInt();     // Second number = angle
//     if (Serial.read() == '\n' || Serial.read() == '\r') { } // clear newline

//     if ((servoNum == 1 || servoNum == 2) && (angle >= 0 && angle <= 180)) {
//       if (servoNum == 1) {
//         servo1.write(angle);
//       } else {
//         servo2.write(angle);
//       }
//       Serial.print("Servo ");
//       Serial.print(servoNum);
//       Serial.print(" moved to ");
//       Serial.print(angle);
//       Serial.println("°");
//     } else {
//       Serial.println("Invalid input! Use: <1|2> <0–180>");
//     }
//   }
// }



#include <Servo.h>

Servo s1;
Servo s2;
Servo s3;

String inputString = "";

void setup() {
  Serial.begin(9600);

  s1.attach(3);
  s2.attach(5);
  s3.attach(6);

  Serial.println("Enter command as: servo:angle");
  Serial.println("Example: 1:120");
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (inputString.length() > 0) {
        processCommand(inputString);
        inputString = "";
      }
    } else {
      inputString += c;
    }
  }
}

void processCommand(String cmd) {
  int colonPos = cmd.indexOf(':');
  if (colonPos == -1) {
    Serial.println("Invalid format. Use servo:angle");
    return;
  }

  int servoNum = cmd.substring(0, colonPos).toInt();
  int angle = cmd.substring(colonPos + 1).toInt();

  angle = constrain(angle, 0, 180);

  switch (servoNum) {
    case 1:
      s1.write(angle);
      Serial.print("Servo 1 → ");
      Serial.println(angle);
      break;

    case 2:
      s2.write(angle);
      Serial.print("Servo 2 → ");
      Serial.println(angle);
      break;

    case 3:
      s3.write(angle);
      Serial.print("Servo 3 → ");
      Serial.println(angle);
      break;

    default:
      Serial.println("Servo number must be 1, 2, or 3");
  }
}

