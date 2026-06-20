#include <opencv2/opencv.hpp>   // Include OpenCV for image processing
#include <iostream>             // Terminal ouput/debugging
#include <string>               // String for serial commands
#include <fcntl.h>              // Open serial port
#include <termios.h>            // Serial port configuration
#include <unistd.h>             // Serial port writing, sleeping, and closing
#include <chrono>               // Clock for serial throttle

// HSV tuned values for green ping pong ball target
int hLow = 45, hHigh = 85;
int sLow = 50, sHigh = 255;
int vLow = 30, vHigh = 255;

// Servo angles float, int, and previous. Servos begin at 90º
float panAngleF  = 90.0f;
float tiltAngleF = 90.0f;
int panAngle  = 90;
int tiltAngle = 90;
int lastPanSent  = -1;   // last pan value sent to Arduino
int lastTiltSent = -1;   // last tilt value sent to Arduino

// Rate limiting for serial sends
auto lastSendTime = std::chrono::steady_clock::now();
const int MIN_SEND_INTERVAL_MS = 10;  // minimum ms between serial commands

// Implausible position rejection
int lastCx = -1;   // last valid centroid X
int lastCy = -1;   // last valid centroid Y
const int MAX_JUMP = 150;  // reject centroid jumps larger than this many pixels

// Gain - adjustment rate (degrees/pixel)
const float GAIN = 0.01f; // Proportional gain

// Deadband — jitter threshold (pixels)
const int DEADBAND = 15;

// Servo safety angle limits
const int SERVO_MIN = 10;
const int SERVO_MAX = 170;

// Open serial port to Arduino
int openSerial(const char* port) {
    // Open for reading & writing, not the controlling terminal, non-blocking writes
    int fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);

    // If serial port open fails
    if (fd < 0) {
        std::cerr << "Error opening serial port: " << port << std::endl;
        return -1;
    }

    // Initialize data structure for port attributes
    struct termios tty;
    tcgetattr(fd, &tty);    // Read open settings

    // Set baud rate to 115200 — must match Arduino
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

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
    cv::VideoCapture cap(0, cv::CAP_V4L2);
    // Ensure camera connection
    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open camera" << std::endl;
        return -1;
    }

    // Request MJPEG — YUYV at 720p is bandwidth-capped to 5fps; MJPEG does 30fps
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));

    // Set camera resolution
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    std::cout << "Capture: " << cap.get(cv::CAP_PROP_FRAME_WIDTH)
     << "x" << cap.get(cv::CAP_PROP_FRAME_HEIGHT) << std::endl; // Verify resolution
    
    // Generate matrices for raw feed, HSV, and masked feed
    cv::Mat frame, hsv, mask;

    // Frame rate 
    int frameCount = 0;
    auto fpsClock = std::chrono::steady_clock::now();

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
        cv::erode(mask, mask, cv::Mat(), cv::Point(-1,-1), 1);      // Erode to remove noise
        cv::dilate(mask, mask, cv::Mat(), cv::Point(-1,-1), 1);     // Dilate to restore blob sizes

        // Create array contours and store blob outlines
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // Ensure there is a blob in  mask
        if (!contours.empty()) {

            // Find largest blob
            int largest = 0;
            double area = cv::contourArea(contours[largest]);
            for (int i = 1; i < (int)contours.size(); i++) {
                double a = cv::contourArea(contours[i]);
                if (a > area) {
                    largest = i;
                    area = a;          // <-- keep area in sync with largest
                }
            }

            // Threshold for minimum trackable blob size is 500 pixels
            if (area > 500) {
                // Compute centroid
                cv::Moments m = cv::moments(contours[largest]); // Compute moments
                // m.m00 is zeroth moment (area of blob)
                int cx = (int)(m.m10 / m.m00);      // m.m10 first moment X (intensity weighted X width)
                int cy = (int)(m.m01 / m.m00);      // m.m01 first moment Y (intensity weighted Y width)

                // Reject implausible jumps — if the centroid teleports too far from last frame
                if (lastCx >= 0) {
                    int jumpX = abs(cx - lastCx);
                    int jumpY = abs(cy - lastCy);
                    if (jumpX > MAX_JUMP || jumpY > MAX_JUMP) {
                        continue;
                    }
                }

                // Accept this centroid as valid
                lastCx = cx;
                lastCy = cy;

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

                // Throttle serial commands and call functiont to send angles to Arduino
                auto now = std::chrono::steady_clock::now();
                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastSendTime).count();
                if ((panAngle != lastPanSent || tiltAngle != lastTiltSent) && elapsed >= MIN_SEND_INTERVAL_MS) {
                    sendServoCommand(serialFd, panAngle, tiltAngle);
                    lastPanSent  = panAngle;
                    lastTiltSent = tiltAngle;
                    lastSendTime = now;
                }
            }
        }

        // Frame rate update and print
        frameCount++;
        auto fpsNow = std::chrono::steady_clock::now();
        auto fpsElapsed = std::chrono::duration_cast<std::chrono::milliseconds>(fpsNow - fpsClock).count();
        if (fpsElapsed >= 1000) {
            std::cout << "FPS: " << frameCount << std::endl;
            frameCount = 0;
            fpsClock = fpsNow;
        }
    }

    // Cleanup
    if (serialFd >= 0) close(serialFd);
    cap.release();
    cv::destroyAllWindows();
    return 0;
}