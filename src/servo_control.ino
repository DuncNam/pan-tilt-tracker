#include <Servo.h>

// Create two servo objects
Servo panServo;
Servo tiltServo;

// Pin assignments
const int PAN_PIN  = 9;
const int TILT_PIN = 10;
const int LASER_PIN = 8;

// Servo center positions in degrees
int panAngle  = 90;
int tiltAngle = 90;

void setup() {
    // Attach servos to their pins
    panServo.attach(PAN_PIN);
    tiltServo.attach(TILT_PIN);

    // Set laser pin as output
    pinMode(LASER_PIN, OUTPUT);
    digitalWrite(LASER_PIN, HIGH); // Turn laser on

    // Move servos to center position on startup
    panServo.write(panAngle);
    tiltServo.write(tiltAngle);

    // Start serial communication at 9600 baud (bit/s)
    Serial.begin(9600);
    Serial.println("Arduino ready");
}

void loop() {
    // Check if data is available from RPi
    if (Serial.available() > 0) {
        // Read incoming string until newline
        String incoming = Serial.readStringUntil('\n');

        // Parse pan and tilt angles from "pan,tilt" format
        int commaIndex = incoming.indexOf(',');
        if (commaIndex > 0) {
            int newPan  = incoming.substring(0, commaIndex).toInt();
            int newTilt = incoming.substring(commaIndex + 1).toInt();

            // Constrain angles to safe servo range
            panAngle  = constrain(newPan,  0, 180);
            tiltAngle = constrain(newTilt, 0, 180);

            // Command servos
            panServo.write(panAngle);
            tiltServo.write(tiltAngle);

            // Echo back confirmation
            Serial.print("Pan: ");
            Serial.print(panAngle);
            Serial.print(" Tilt: ");
            Serial.println(tiltAngle);
        }
    }
}