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
//   myServo.attach(9);
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


// #include <Servo.h>

// Servo myServo;
// int angle = 0;

// void setup() {
//   Serial.begin(9600);
//   myServo.attach(3);
//   Serial.println("Enter angle (-180 to 180):");
// }

// void loop() {
//   if (Serial.available() > 0) {

//     int input = Serial.parseInt();   // read signed integer

//     // Clear any newline characters
//     while (Serial.peek() == '\n' || Serial.peek() == '\r') {
//       Serial.read();
//     }

//     // Check range
//     if (input >= -180 && input <= 180) {

//       // Map -180..180 to 0..180
//       int servoAngle = map(input, -180, 180, 0, 180);

//       myServo.write(servoAngle);

//       Serial.print("Input: ");
//       Serial.print(input);
//       Serial.print("  → Servo angle: ");
//       Serial.println(servoAngle);

//     } else {
//       Serial.println("Invalid angle (-180 to 180)");
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


// 5 -best working code for 3 servo motors 
// #include <Servo.h>

// Servo s1;
// Servo s2;
// Servo s3;

// String inputString = "";

// void setup() {
//   Serial.begin(9600);

//   s1.attach(3);
//   s2.attach(5);
//   s3.attach(6);

//   Serial.println("Enter command as: servo:angle");
//   Serial.println("Example: 1:120");
// }

// void loop() {
//   while (Serial.available()) {
//     char c = Serial.read();

//     if (c == '\n' || c == '\r') {
//       if (inputString.length() > 0) {
//         processCommand(inputString);
//         inputString = "";
//       }
//     } else {
//       inputString += c;
//     }
//   }
// }

// void processCommand(String cmd) {
//   int colonPos = cmd.indexOf(':');
//   if (colonPos == -1) {
//     Serial.println("Invalid format. Use servo:angle");
//     return;
//   }

//   int servoNum = cmd.substring(0, colonPos).toInt();
//   int angle = cmd.substring(colonPos + 1).toInt();

//   angle = constrain(angle, 0, 180);

//   switch (servoNum) {
//     case 1:
//       s1.write(angle);
//       Serial.print("Servo 1 → ");
//       Serial.println(angle);
//       break;

//     case 2:
//       s2.write(angle);
//       Serial.print("Servo 2 → ");
//       Serial.println(angle);
//       break;

//     case 3:
//       s3.write(angle);
//       Serial.print("Servo 3 → ");
//       Serial.println(angle);
//       break;

//     default:
//       Serial.println("Servo number must be 1, 2, or 3");
//   }
// }

//6-all motors move toghther
// #include <Servo.h>

// Servo s1;
// Servo s2;
// Servo s3;
// Servo s4;

// String inputString = "";

// void setup() {
//   Serial.begin(9600);

//   s1.attach(3);
//   s2.attach(5);
//   s3.attach(6);
//   s4.attach(9);   // <--- Added fourth servo

//   Serial.println("Enter command as: servo:angle");
//   Serial.println("Example: 1:120");
// }

// void loop() {
//   while (Serial.available()) {
//     char c = Serial.read();

//     if (c == '\n' || c == '\r') {
//       if (inputString.length() > 0) {
//         processCommand(inputString);
//         inputString = "";
//       }
//     } else {
//       inputString += c;
//     }
//   }
// }

// void processCommand(String cmd) {
//   int colonPos = cmd.indexOf(':');
//   if (colonPos == -1) {
//     Serial.println("Invalid format. Use servo:angle");
//     return;
//   }

//   int servoNum = cmd.substring(0, colonPos).toInt();
//   int angle = cmd.substring(colonPos + 1).toInt();

//   angle = constrain(angle, 0, 180);

//   switch (servoNum) {
//     case 1:
//       s1.write(angle);
//       Serial.print("Servo 1 → ");
//       Serial.println(angle);
//       break;

//     case 2:
//       s2.write(angle);
//       Serial.print("Servo 2 → ");
//       Serial.println(angle);
//       break;

//     case 3:
//       s3.write(angle);
//       Serial.print("Servo 3 → ");
//       Serial.println(angle);
//       break;

//     case 4:
//       s4.write(angle);
//       Serial.print("Servo 4 → ");
//       Serial.println(angle);
//       break;

//     default:
//       Serial.println("Servo number must be 1, 2, 3, or 4");
//   }
// }

// #include <Servo.h>

// Servo s1;
// Servo s2;
// Servo s3;
// Servo s4;

// String inputString = "";

// void setup() {
//   Serial.begin(9600);

//   s1.attach(3);
//   s2.attach(5);
//   s3.attach(6);
//   s4.attach(9);   // Attach fourth servo

//   Serial.println("Enter commands as: servo:angle,servo:angle,...");
//   Serial.println("Example: 1:120,2:90,3:45,4:180");
// }

// void loop() {
//   while (Serial.available()) {
//     char c = Serial.read();

//     if (c == '\n' || c == '\r') {
//       if (inputString.length() > 0) {
//         processCommands(inputString);  // Process commands in batch
//         inputString = "";  // Clear the string
//       }
//     } else {
//       inputString += c;  // Append characters to inputString
//     }
//   }
// }

// void processCommands(String cmds) {
//   // Split the input into individual servo commands
//   int startIndex = 0;
//   int commaPos = cmds.indexOf(',');

//   while (commaPos != -1) {
//     String cmd = cmds.substring(startIndex, commaPos);
//     processCommand(cmd);  // Process each individual command
//     startIndex = commaPos + 1;
//     commaPos = cmds.indexOf(',', startIndex);
//   }
  
//   // Process the last command (if any)
//   String lastCmd = cmds.substring(startIndex);
//   if (lastCmd.length() > 0) {
//     processCommand(lastCmd);
//   }
// }

// void processCommand(String cmd) {
//   int colonPos = cmd.indexOf(':');
//   if (colonPos == -1) {
//     Serial.println("Invalid format. Use servo:angle");
//     return;
//   }

//   int servoNum = cmd.substring(0, colonPos).toInt();
//   int angle = cmd.substring(colonPos + 1).toInt();

//   angle = constrain(angle, 0, 180);  // Ensure angle is within valid range

//   switch (servoNum) {
//     case 1:
//       s1.write(angle);
//       Serial.print("Servo 1 → ");
//       Serial.println(angle);
//       break;

//     case 2:
//       s2.write(angle);
//       Serial.print("Servo 2 → ");
//       Serial.println(angle);
//       break;

//     case 3:
//       s3.write(angle);
//       Serial.print("Servo 3 → ");
//       Serial.println(angle);
//       break;

//     case 4:
//       s4.write(angle);
//       Serial.print("Servo 4 → ");
//       Serial.println(angle);
//       break;

//     default:
//       Serial.println("Servo number must be 1, 2, 3, or 4");
//   }
// }

// #include <Servo.h>

// Servo s1;
// Servo s2;
// Servo s3;
// Servo s4;
// Servo s5;   // New servo

// String inputString = "";

// void setup() {
//   Serial.begin(9600);

//   s1.attach(3);
//   s2.attach(5);
//   s3.attach(6);
//   s4.attach(9);
//   s5.attach(10);   // Attach fifth servo (choose any free PWM pin)

//   Serial.println("Enter commands as: servo:angle,servo:angle,...");
//   Serial.println("Example: 1:120,2:90,3:45,4:180,5:30");
// }

// void loop() {
//   while (Serial.available()) {
//     char c = Serial.read();

//     if (c == '\n' || c == '\r') {
//       if (inputString.length() > 0) {
//         processCommands(inputString);
//         inputString = "";
//       }
//     } else {
//       inputString += c;
//     }
//   }
// }

// void processCommands(String cmds) {
//   int startIndex = 0;
//   int commaPos = cmds.indexOf(',');

//   while (commaPos != -1) {
//     String cmd = cmds.substring(startIndex, commaPos);
//     processCommand(cmd);
//     startIndex = commaPos + 1;
//     commaPos = cmds.indexOf(',', startIndex);
//   }

//   String lastCmd = cmds.substring(startIndex);
//   if (lastCmd.length() > 0) {
//     processCommand(lastCmd);
//   }
// }

// void processCommand(String cmd) {
//   int colonPos = cmd.indexOf(':');
//   if (colonPos == -1) {
//     Serial.println("Invalid format. Use servo:angle");
//     return;
//   }

//   int servoNum = cmd.substring(0, colonPos).toInt();
//   int angle = cmd.substring(colonPos + 1).toInt();
//   angle = constrain(angle, 0, 180);

//   switch (servoNum) {
//     case 1:
//       s1.write(angle);
//       Serial.print("Servo 1 → ");
//       Serial.println(angle);
//       break;

//     case 2:
//       s2.write(angle);
//       Serial.print("Servo 2 → ");
//       Serial.println(angle);
//       break;

//     case 3:
//       s3.write(angle);
//       Serial.print("Servo 3 → ");
//       Serial.println(angle);
//       break;

//     case 4:
//       s4.write(angle);
//       Serial.print("Servo 4 → ");
//       Serial.println(angle);
//       break;

//     case 5:
//       s5.write(angle);
//       Serial.print("Servo 5 → ");
//       Serial.println(angle);
//       break;

//     default:
//       Serial.println("Servo number must be 1, 2, 3, 4, or 5");
//   }
// }

// #include <Servo.h>

// Servo s1;
// Servo s2;
// Servo s3;
// Servo s4;
// Servo s5;

// String inputString = "";

// void setup() {
//   Serial.begin(9600);

//   s1.attach(3);
//   s2.attach(5);
//   s3.attach(6);
//   s4.attach(9);
//   s5.attach(10);

//   Serial.println("Enter commands as: servo:angle,servo:angle,...");
//   Serial.println("Angle range = -90 to +90");
//   Serial.println("Example: 1:-45,2:60,3:0,4:90,5:-30");
// }

// void loop() {
//   while (Serial.available()) {
//     char c = Serial.read();

//     if (c == '\n' || c == '\r') {
//       if (inputString.length() > 0) {
//         processCommands(inputString);
//         inputString = "";
//       }
//     } else {
//       inputString += c;
//     }
//   }
// }

// void processCommands(String cmds) {
//   int startIndex = 0;
//   int commaPos = cmds.indexOf(',');

//   while (commaPos != -1) {
//     String cmd = cmds.substring(startIndex, commaPos);
//     processCommand(cmd);
//     startIndex = commaPos + 1;
//     commaPos = cmds.indexOf(',', startIndex);
//   }

//   String lastCmd = cmds.substring(startIndex);
//   if (lastCmd.length() > 0) {
//     processCommand(lastCmd);
//   }
// }

// void processCommand(String cmd) {
//   int colonPos = cmd.indexOf(':');
//   if (colonPos == -1) {
//     Serial.println("Invalid format. Use servo:angle");
//     return;
//   }

//   int servoNum = cmd.substring(0, colonPos).toInt();
//   int userAngle = cmd.substring(colonPos + 1).toInt();

//   // Limit input angle to -90..90
//   userAngle = constrain(userAngle, -90, 90);

//   // Convert -90..+90 → 0..180 for servo
//   int servoAngle = map(userAngle, -90, 90, 0, 180);
//   Serial.println(servoAngle);

//   switch (servoNum) {
//     case 1:
//       s1.write(servoAngle);
//       Serial.print("Servo 1 → ");
//       Serial.println(userAngle);
//       break;

//     case 2:
//       s2.write(servoAngle);
//       Serial.print("Servo 2 → ");
//       Serial.println(userAngle);
//       break;

//     case 3:
//       s3.write(servoAngle);
//       Serial.print("Servo 3 → ");
//       Serial.println(userAngle);
//       break;

//     case 4:
//       s4.write(servoAngle);
//       Serial.print("Servo 4 → ");
//       Serial.println(userAngle);
//       break;

//     case 5:
//       s5.write(servoAngle);
//       Serial.print("Servo 5 → ");
//       Serial.println(userAngle);
//       break;

//     default:
//       Serial.println("Servo number must be 1–5");
//   }
// }

// #include <Servo.h>

// // =================== SERVO OBJECTS ===================
// Servo s1;  // MG996R
// Servo s2;  // MG996R
// Servo s3;  // MG996R
// Servo s4;  // SG90
// Servo s5;  // SG90

// // =================== TRAJECTORY TABLE ===================
// // Each row = one time step
// // Each column = angles for motors 1..5
// // ADD YOUR FULL TRAJECTORY HERE:

// float traj[][5] = {
//   {45.000076, 42.273839, 39.805707, 37.568715, 35.537790},
//   {33.690170, 32.005477, 30.465631, 29.054683, 27.758613},

//   // <<< ADD MORE ROWS HERE >>>
// };

// int trajLength = sizeof(traj) / sizeof(traj[0]);

// // Playback speed (milliseconds between samples)
// const int dt = 50;   // 50ms = 20 Hz

// // ========================================================

// void setup() {
//   Serial.begin(9600);

//   // Attach ALL servos
//   s1.attach(3);
//   s2.attach(5);
//   s3.attach(6);
//   s4.attach(9);
//   s5.attach(10);

//   Serial.println("Trajectory playback started...");
// }

// void loop() {

//   // === Play trajectory once ===
//   for (int i = 0; i < trajLength; i++) {

//     float a1 = traj[i][0];
//     float a2 = traj[i][1];
//     float a3 = traj[i][2];
//     float a4 = traj[i][3];
//     float a5 = traj[i][4];

//     // Clamp user angles safely (-90..90)
//     a1 = constrain(a1, -90, 90);
//     a2 = constrain(a2, -90, 90);
//     a3 = constrain(a3, -90, 90);
//     a4 = constrain(a4, -90, 90);
//     a5 = constrain(a5, -90, 90);

//     // MAP angles:
//     // MG996R = full safe range 0..180
//     int s1angle = map(a1, -90, 90, 0, 180);
//     int s2angle = map(a2, -90, 90, 0, 180);
//     int s3angle = map(a3, -90, 90, 0, 180);

//     // SG90 safe range = 10..170 only
//     int s4angle = constrain(map(a4, -90, 90, 0, 180), 10, 170);
//     int s5angle = constrain(map(a5, -90, 90, 0, 180), 10, 170);

//     // WRITE SERVOS
//     s1.write(s1angle);
//     s2.write(s2angle);
//     s3.write(s3angle);
//     s4.write(s4angle);
//     s5.write(s5angle);

//     // Debug output
//     Serial.print("Step "); Serial.print(i);
//     Serial.print(": "); 
//     Serial.print(a1); Serial.print("  ");
//     Serial.print(a2); Serial.print("  ");
//     Serial.print(a3); Serial.print("  ");
//     Serial.print(a4); Serial.print("  ");
//     Serial.print(a5); Serial.println();

//     delay(dt);
//   }

//   // Stop program after finishing trajectory
//   Serial.println("Trajectory complete.");
//   while (1);
// }
#include <Servo.h>

// =================== SERVO OBJECTS ===================
Servo s1;  
Servo s2;  
Servo s3;  
Servo s4;  
Servo s5;  

// =================== TRAJECTORY TABLE ===================
float traj[][5] = {
  {45.000076, 42.273839, 39.805707, 37.568715, 35.537790},
  {33.690170, 32.005477, 30.465631, 29.054683, 27.758613},
  // Add more rows...
};

int trajLength = sizeof(traj) / sizeof(traj[0]);

// Smooth movement settings
const float stepSize = 1.0;      // degrees per step
const int stepDelay = 20;        // ms delay between micro-steps

// Store current servo positions
float currA[5] = {0, 0, 0, 0, 0};

void setup() {
  Serial.begin(9600);

  s1.attach(3);
  s2.attach(5);
  s3.attach(6);
  s4.attach(9);
  s5.attach(10);

  // Initialize starting angles to first row
  for (int i = 0; i < 5; i++) currA[i] = traj[0][i];

  Serial.println("Smooth trajectory playback...");
}

void smoothMove(float target[5]) {
  
  bool done = false;

  while (!done) {
    done = true;

    for (int m = 0; m < 5; m++) {
      float diff = target[m] - currA[m];

      if (abs(diff) > 0.5) {
        done = false;

        // move slowly toward target
        currA[m] += (diff > 0 ? stepSize : -stepSize);

        // clamp to safe range
        currA[m] = constrain(currA[m], -90, 90);
      }
    }

    // Map to servo angles
    int a1 = map(currA[0], -90, 90, 0, 180);
    int a2 = map(currA[1], -90, 90, 0, 180);
    int a3 = map(currA[2], -90, 90, 0, 180);
    int a4 = constrain(map(currA[3], -90, 90, 0, 180), 10, 170);
    int a5 = constrain(map(currA[4], -90, 90, 0, 180), 10, 170);

    // write servo positions
    s1.write(a1);
    s2.write(a2);
    s3.write(a3);
    s4.write(a4);
    s5.write(a5);

    delay(stepDelay);
  }
}

void loop() {
  for (int i = 0; i < trajLength; i++) {
    smoothMove(traj[i]);
  }

  Serial.println("Trajectory finished.");
  while (1);
}

