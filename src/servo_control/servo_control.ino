#include <Servo.h>

// Create two servo objects
Servo panServo;
Servo tiltServo;

// Pin assignments
const int PAN_PIN  = 9;
const int TILT_PIN = 10;
const int LASER_PIN = 8;

// Servo center positions in microseconds
int panMicros  = 1500;
int tiltMicros = 1500;

void setup() {
    // Attach servos to their pins
    panServo.attach(PAN_PIN);
    tiltServo.attach(TILT_PIN);

    // Set laser pin as output
    pinMode(LASER_PIN, OUTPUT);
    digitalWrite(LASER_PIN, HIGH); // Turn laser on

    // Move servos to center position on startup
    panServo.writeMicroseconds(panMicros);
    tiltServo.writeMicroseconds(tiltMicros);

    // Start serial communication at 115200 baud (bit/s)
    Serial.begin(115200);
}

void loop() {
    static String latest = ""; // Command string
    bool gotCommand = false; // Verifies receipt of a complete command

    // Serial drain loop
    while (Serial.available() > 0) {
        char c = Serial.read(); // Read next character
        if (c == '\n') {
            // End of a command — mark it ready to process
            gotCommand = true;
            break;
        } else {
            latest += c;
        }
    }   

    if (gotCommand) {
        // Parse "pan,tilt" from the assembled string
        int commaIndex = latest.indexOf(',');
        if (commaIndex > 0) { // Verify commma was found
            // Extract angles from string to integer
            int newPan  = latest.substring(0, commaIndex).toInt();
            int newTilt = latest.substring(commaIndex + 1).toInt();

            // Constrain angles to safe PW range
            panMicros  = constrain(newPan,  500, 2500);
            tiltMicros = constrain(newTilt, 500, 2500);

            // Command servos
            panServo.writeMicroseconds(panMicros);
            tiltServo.writeMicroseconds(tiltMicros);
        }
        // Reset for the next command
        latest = "";
    }
}