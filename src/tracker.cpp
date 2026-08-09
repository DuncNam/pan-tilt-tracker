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

// Previous error for proportional term
float lastErrorX = 0.0f;
float lastErrorY = 0.0f;

// Error from two frames ago, for the derivative term
float lastLastErrorX = 0.0f;
float lastLastErrorY = 0.0f;

// Previous command to prevent redundant sends
int lastPanSent  = -1;
int lastTiltSent = -1;

// Serial throttle to avoid overload
auto lastSendTime = std::chrono::steady_clock::now();
const int MIN_SEND_INTERVAL_MS = 10;    // minimum ms between serial commands

// Implausible position rejection
int lastCx = -1;                // last valid centroid X
int lastCy = -1;                // last valid centroid Y
const int MAX_JUMP = 500;       // reject centroid jumps larger than this many pixels

// Gain - adjustment rate (degrees/pixel)
const float KP = 0.04f;        // Proportional
const float KI = 0.012f;         // Integral
const float KD = 0.015f;          // Derivative

// Deadband — jitter threshold (pixels)
const int DEADBAND = 15;

// Servo safety angle limits
const int SERVO_MIN = 10;
const int SERVO_MAX = 170;

// Servo safety PW limits for finest command resolution. MG996R servos still have 5us deadband
const int US_MIN = 500;   // pulse width at 0 degrees
const int US_MAX = 2500;   // pulse width at 180 degrees

// Boresigh adjustment
const int BORESIGHT_X = 60;   // laser-to-camera offset, horizontal (pixels)
const int BORESIGHT_Y = 50;   // laser-to-camera offset, vertical (pixels)

// Open and configure serial port to Arduino
int openSerial(const char* port) {
    // Open for reading & writing, not controlling, non-blocking writes
    int fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);

    // If serial port open fails
    if (fd < 0) {
        std::cerr << "Error opening serial port: " << port << std::endl;
        return -1;
    }

    struct termios tty;             // Initialize tty for port attributes
    tcgetattr(fd, &tty);            // Read open settings

    cfsetospeed(&tty, B115200);     // Set baud rate out to 115200
    cfsetispeed(&tty, B115200);     // Set baud rate in to 115200

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

// Convert angle commands to PW
int angleToMicros(float angle) {
    return (int)(US_MIN + (angle / 180.0f) * (US_MAX - US_MIN));
}

// Send pan and tilt angles to Arduino over serial
void sendServoCommand(int fd, int pan, int tilt) {
    if (fd < 0) return;                      // Ensure serial port connection

    // Convert ints to string
    std::string cmd = std::to_string(pan) + "," + std::to_string(tilt) + "\n";

    write(fd, cmd.c_str(), cmd.length());    // Send to Arduino through serial port
}

int main() {
    int serialFd = openSerial("/dev/ttyACM0");  // Open serial port to Arduino

    // Ensure serial port connection
    if (serialFd < 0) {
        std::cerr << "Warning: Could not open serial port. Running vision only." << std::endl;
    }

    sleep(2);                                   // Small delay to let Arduino initialize after serial connection

    cv::VideoCapture cap(0, cv::CAP_V4L2);      // Open camera

    // Ensure camera connection
    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open camera" << std::endl;
        return -1;
    }

    // Request MJPEG (30fps) instead of YUYV (5fps) at 720p
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));

    // Set camera resolution
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);

    // Lock camera exposure. Auto-exposure drifts with scene brightness, which
    // shifts HSV values under fixed thresholds and varies motion blur.
    // Measured 2026-08-08: auto mode selected 33 ms exposure (a full frame
    // period). Locking to 10 ms cut motion-blur area variance from 4.9x to 1.8x.
    // These are V4L2 device settings and reset when the camera re-enumerates,
    // so they must be reapplied every run.
    system("v4l2-ctl -d /dev/video0 --set-ctrl=exposure_dynamic_framerate=0");
    system("v4l2-ctl -d /dev/video0 --set-ctrl=auto_exposure=1");        // 1 = Manual
    system("v4l2-ctl -d /dev/video0 --set-ctrl=exposure_time_absolute=100");  // 10 ms
    system("v4l2-ctl -d /dev/video0 --set-ctrl=gain=147");

    // Verify resolution
    std::cout << "Capture: " << cap.get(cv::CAP_PROP_FRAME_WIDTH)
        << "x" << cap.get(cv::CAP_PROP_FRAME_HEIGHT) << std::endl;
    
    // Generate matrices for raw feed, HSV, and masked feed
    cv::Mat frame, hsv, mask;

    // Frame rate counter
    int frameCount = 0;
    int sendCount = 0;
    auto fpsClock = std::chrono::steady_clock::now();

    // Initialize program loop
    while (true) {
        cap >> frame;                                               // Send current frame to frame matrix
        if (frame.empty()) break;                                   // Verify camera feed

        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);                // Convert BGR to HSV

        // Build color bounds from trackbar values
        cv::Scalar lowerBound(hLow, sLow, vLow);
        cv::Scalar upperBound(hHigh, sHigh, vHigh);

        // Create mask
        cv::inRange(hsv, lowerBound, upperBound, mask);

        cv::erode(mask, mask, cv::Mat(), cv::Point(-1,-1), 1);      // Erode to remove noise
        cv::dilate(mask, mask, cv::Mat(), cv::Point(-1,-1), 1);     // Dilate to restore blob sizes
        
        // Fill the hole punched by the laser dot. 
        // Measured 2026-08-08: laser centered collapsed blob area from ~880 to ~150 px
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(15, 15)));

        // Create array contours and store blob outlines
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // Ensure there is a blob in  mask
        bool targetFound = false;
        double area = 0.0f;
        if (!contours.empty()) {
            // Find largest blob in contours
            int largest = 0;
            area = cv::contourArea(contours[largest]);
            for (int i = 1; i < (int)contours.size(); i++) {
                double a = cv::contourArea(contours[i]);
                if (a > area) {
                    largest = i;
                    area = a;
                }
            }

            // Threshold for minimum trackable blob size is 300 pixels
            if (area > 300) {
                // Compute centroid
                cv::Moments m = cv::moments(contours[largest]);     // Compute moments

                // m.m00 is zeroth moment (area of blob)
                int cx = (int)(m.m10 / m.m00);                      // m.m10 first moment X (intensity weighted X width)
                int cy = (int)(m.m01 / m.m00);                      // m.m01 first moment Y (intensity weighted Y width)

                // Reject implausible jumps — if the centroid teleports too far from last frame
                bool jumpRejected = false;
                if (lastCx >= 0) {                                  // Skips check on first detection
                    int jumpX = abs(cx - lastCx);
                    int jumpY = abs(cy - lastCy);
                    if (jumpX > MAX_JUMP || jumpY > MAX_JUMP) {
                        jumpRejected = true;                        // Discard this frame without moving servos
                    }
                }

                // Update centroid based on valid target
                if (!jumpRejected) {
                    targetFound = true;      // valid target locked this frame
                    lastCx = cx;
                    lastCy = cy;

                    // Calculate error from frame center
                    int frameW = frame.cols;        // (1280)
                    int frameH = frame.rows;        // (720)
                    int errorX = cx - ((frameW / 2) + BORESIGHT_X);   // pos (+) -> target right of center
                    int errorY = cy - ((frameH / 2) + BORESIGHT_Y);   // pos (+) -> target below center

                    // Velocity-form PID control
                    if (abs(errorX) > DEADBAND) {
                        float dErrorX = errorX - lastErrorX;                                // change in error since last frame
                        float d2ErrorX = errorX - (2.0f * lastErrorX) + lastLastErrorX;     // change of the change in error since last frame
                        panAngleF -= (dErrorX * KP) + (errorX * KI) + (d2ErrorX * KD);      // P on error' + I on error + D on error''
                    }
                    if (abs(errorY) > DEADBAND) {
                        float dErrorY = errorY - lastErrorY;
                        float d2ErrorY = errorY - (2.0f * lastErrorY) + lastLastErrorY;
                        tiltAngleF += (dErrorY * KP) + (errorY * KI) + (d2ErrorY * KD);
                    }

                    // Store error for next frame's change calculation
                    lastLastErrorX = lastErrorX;
                    lastLastErrorY = lastErrorY;
                    lastErrorX = errorX;
                    lastErrorY = errorY;

                    // Clamp angles to safe servo range in degrees
                    panAngleF  = std::max((float)SERVO_MIN, std::min((float)SERVO_MAX, panAngleF));
                    tiltAngleF = std::max((float)SERVO_MIN, std::min((float)SERVO_MAX, tiltAngleF));

                    // Convert fractional degrees to microseconds for finer command
                    int panMicros  = angleToMicros(panAngleF);
                    int tiltMicros = angleToMicros(tiltAngleF);

                    // Send if microsecond command changed and enough time has passed
                    auto now = std::chrono::steady_clock::now();
                    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastSendTime).count();
                    if ((panMicros != lastPanSent || tiltMicros != lastTiltSent) && elapsed >= MIN_SEND_INTERVAL_MS) {
                        sendServoCommand(serialFd, panMicros, tiltMicros);
                        lastPanSent  = panMicros;
                        lastTiltSent = tiltMicros;
                        lastSendTime = now;
                        sendCount++;
                    }
                }
            }
        }

        // If no valid target this frame, invalidate jump history so re-entry is accepted fresh
        if (!targetFound) {
            lastCx = -1;
            lastCy = -1;
            lastErrorX = 0.0f;    // reset so reacquisition doesn't compute a huge spurious error-change
            lastErrorY = 0.0f;
            lastLastErrorX = 0.0f;
            lastLastErrorY = 0.0f;
        }

        // Frame rate update and print
        frameCount++;
        auto fpsNow = std::chrono::steady_clock::now();
        auto fpsElapsed = std::chrono::duration_cast<std::chrono::milliseconds>(fpsNow - fpsClock).count();
        if (fpsElapsed >= 1000) {
            std::cout << "FPS: " << frameCount << "  Sends/s: " << sendCount << "  Area: " << area << std::endl;
            frameCount = 0;
            sendCount = 0;
            fpsClock = fpsNow;
        }
    }

    // Cleanup
    if (serialFd >= 0) close(serialFd);
    cap.release();
    return 0;
}