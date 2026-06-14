#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

// Global HSV trackbar values
int hLow = 45, hHigh = 85;
int sLow = 50, sHigh = 255;
int vLow = 30, vHigh = 255;

// Servo angles — start centered
float panAngleF  = 90.0f;
float tiltAngleF = 90.0f;
int panAngle  = 90;
int tiltAngle = 90;

// Proportional gain - adjustment rate (degrees/pixel)
const float GAIN = 0.005f;

// Deadband — jitter threshold (pixels)
const int DEADBAND = 15;

// Servo safety angle limits
const int SERVO_MIN = 10;
const int SERVO_MAX = 170;

// Open serial port to Arduino
int openSerial(const char* port) {
    // Open for reading & writing, not the controlling terminal, finish each write before continuing
    int fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);

    // If serial port open fails
    if (fd < 0) {
        std::cerr << "Error opening serial port: " << port << std::endl;
        return -1;
    }

    // Initialize data structure for port attributes
    struct termios tty;
    tcgetattr(fd, &tty);    // Read open settings

    // Set baud rate to 9600 — must match Arduino
    cfsetospeed(&tty, B9600);
    cfsetispeed(&tty, B9600);

    // 8N1 — 8 data bits, no parity, 1 stop bit
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag |= CLOCAL | CREAD;  // Disable modem control and enable receiver
    tty.c_lflag = 0;                // Disable local modes processing
    tty.c_oflag = 0;                // Disable output processing
    tty.c_cc[VMIN]  = 0;            // No minimum bytes for return
    tty.c_cc[VTIME] = 1;            // Time out set to 100 ms

    // Apply attributes
    tcsetattr(fd, TCSANOW, &tty);
    return fd;
}

// Send pan and tilt angles to Arduino over serial
void sendServoCommand(int fd, int pan, int tilt) {
    // Ensure serial port connection
    if (fd < 0) return;

    // Convert ints to string command
    std::string cmd = std::to_string(pan) + "," + std::to_string(tilt) + "\n";

    // Send to Arduino through serial port
    write(fd, cmd.c_str(), cmd.length());
}

int main() {
    // Open serial port to Arduino
    int serialFd = openSerial("/dev/ttyACM0");
    // Ensure serial port connection
    if (serialFd < 0) {
        std::cerr << "Warning: Could not open serial port. Running vision only." << std::endl;
    }

    // Small delay to let Arduino initialize after serial connection
    sleep(2);

    // Open camera
    cv::VideoCapture cap(0);
    // Ensure camera connection
    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open camera" << std::endl;
        return -1;
    }

    // Generate matrices for raw feed, HSV, and masked feed
    cv::Mat frame, hsv, mask;

    // Initialize program loop
    while (true) {
        // Send current frame to frame matric
        cap >> frame;
        // Verify camera feed
        if (frame.empty()) break;

        // Convert BGR to HSV
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

        // Build color bounds from trackbar values
        cv::Scalar lowerBound(hLow, sLow, vLow);
        cv::Scalar upperBound(hHigh, sHigh, vHigh);

        // Create and clean mask
        cv::inRange(hsv, lowerBound, upperBound, mask);
        cv::erode(mask, mask, cv::Mat(), cv::Point(-1,-1), 2);      // Erode twice to remove noise
        cv::dilate(mask, mask, cv::Mat(), cv::Point(-1,-1), 2);     // Dilate twice to restore blob sizes

        // Create array contours and store blob outlines
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // Ensure there is a blob in  mask
        if (!contours.empty()) {
            // Find largest blob
            int largest = 0;
            for (int i = 1; i < (int)contours.size(); i++) {
                if (cv::contourArea(contours[i]) > cv::contourArea(contours[largest]))
                    largest = i;
            }

            // Threshold for minimum trackable blob size is 500 pixels
            if (cv::contourArea(contours[largest]) > 500) {
                // Compute centroid
                cv::Moments m = cv::moments(contours[largest]); // Compute moments
                // m.m00 is zeroth moment (area of blob)
                int cx = (int)(m.m10 / m.m00);      // m.m10 first moment X (intensity weighted X width)
                int cy = (int)(m.m01 / m.m00);      // m.m01 first moment Y (intensity weighted Y width)

                // Calculate error from frame center
                int frameW = frame.cols;
                int frameH = frame.rows;
                int errorX = cx - frameW / 2;
                int errorY = cy - frameH / 2;

                // Incremental proportional control
                // Only move if error exceeds deadband
                if (abs(errorX) > DEADBAND) {
                    panAngleF -= errorX * GAIN;
                }
                if (abs(errorY) > DEADBAND) {
                     tiltAngleF += errorY * GAIN;
                }

                // Clamp angles to safe servo range and convert to INT for PWM
                panAngleF  = std::max((float)SERVO_MIN, std::min((float)SERVO_MAX, panAngleF));
                tiltAngleF = std::max((float)SERVO_MIN, std::min((float)SERVO_MAX, tiltAngleF));
                panAngle  = (int)panAngleF;
                tiltAngle = (int)tiltAngleF;

                // Call functiont to send angles to Arduino
                sendServoCommand(serialFd, panAngle, tiltAngle);

                // Draw centroid marker
                cv::circle(frame, cv::Point(cx, cy), 10, cv::Scalar(0, 255, 0), -1);
                cv::line(frame, cv::Point(cx-20, cy), cv::Point(cx+20, cy), cv::Scalar(0,255,0), 2);
                cv::line(frame, cv::Point(cx, cy-20), cv::Point(cx, cy+20), cv::Scalar(0,255,0), 2);

                // Draw frame center crosshair
                cv::drawMarker(frame, cv::Point(frameW/2, frameH/2),
                    cv::Scalar(0,0,255), cv::MARKER_CROSS, 20, 2);

                // Display error and servo angles on screen
                cv::putText(frame, "Error X: " + std::to_string(errorX),
                    cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0,255,0), 2);
                cv::putText(frame, "Error Y: " + std::to_string(errorY),
                    cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0,255,0), 2);
                cv::putText(frame, "Pan: " + std::to_string(panAngle),
                    cv::Point(10, 90), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0,255,255), 2);
                cv::putText(frame, "Tilt: " + std::to_string(tiltAngle),
                    cv::Point(10, 120), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0,255,255), 2);

                // Print to terminal
                std::cout << "Error: (" << errorX << ", " << errorY << ")"
                          << "  Pan: " << panAngle
                          << "  Tilt: " << tiltAngle << std::endl;
            }
        }

        if (cv::waitKey(1) == '0') break;
    }

    // Cleanup
    if (serialFd >= 0) close(serialFd);
    cap.release();
    cv::destroyAllWindows();
    return 0;
}