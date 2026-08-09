// measure_delay.cpp
//
// Measures total closed-loop delay: the time from issuing a servo command to
// the first camera frame in which that motion is visible.
//
// Method: point the camera at a stationary, well-lit target. Command a step in
// pan. The target does not move, but the camera does, so the centroid sweeps
// across the frame. Timestamp the command; timestamp the first frame whose
// centroid has moved more than MOVE_THRESHOLD_PX. The difference is the sum of
// every delay in the loop:
//
//   serial TX -> Arduino parse -> servo drive -> mechanical response
//   -> camera exposure -> USB transfer -> MJPEG decode -> HSV -> contour
//
// This is the number the Kalman feedforward must predict ahead by.
//
// Build:
//   g++ src/measure_delay.cpp -o measure_delay $(pkg-config --cflags --libs opencv4)
//
// Run (tracker must NOT be running - it holds the camera and serial port):
//   ./measure_delay

#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <chrono>

// --- Match tracker.cpp ---
int hLow = 45, hHigh = 85;
int sLow = 50, sHigh = 255;
int vLow = 30, vHigh = 255;

const int US_MIN = 500;
const int US_MAX = 2500;

// --- Measurement parameters ---
const int   TRIALS            = 10;    // repetitions; median is reported
const float REST_ANGLE        = 90.0f; // starting pan angle each trial
const float STEP_DEGREES      = 15.0f; // step size - large enough to be unambiguous
const int   SETTLE_MS         = 2000;  // wait for servo + target to come to rest
const int   MOVE_THRESHOLD_PX = 10;    // centroid displacement counted as "responded"
const int   TIMEOUT_MS        = 1000;  // abandon a trial after this long
const double MIN_AREA         = 300.0;

int openSerial(const char* port) {
    int fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) {
        std::cerr << "Error opening serial port: " << port << std::endl;
        return -1;
    }
    struct termios tty;
    tcgetattr(fd, &tty);
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 1;
    tcsetattr(fd, TCSANOW, &tty);
    return fd;
}

int angleToMicros(float angle) {
    return (int)(US_MIN + (angle / 180.0f) * (US_MAX - US_MIN));
}

void sendServoCommand(int fd, int pan, int tilt) {
    if (fd < 0) return;
    std::string cmd = std::to_string(pan) + "," + std::to_string(tilt) + "\n";
    ssize_t n = write(fd, cmd.c_str(), cmd.length());
    // The tracker ignores this return value on a non-blocking fd. Here we check,
    // because a short write would silently corrupt the command and invalidate
    // the measurement.
    if (n != (ssize_t)cmd.length()) {
        std::cerr << "WARNING: short serial write (" << n << " of "
                  << cmd.length() << " bytes)" << std::endl;
    }
}

// Returns true and sets cx/cy if a valid target is found in this frame.
bool findCentroid(const cv::Mat& frame, cv::Mat& hsv, cv::Mat& mask,
                  int& cx, int& cy, double& areaOut) {
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(hLow, sLow, vLow), cv::Scalar(hHigh, sHigh, vHigh), mask);
    cv::erode(mask, mask, cv::Mat(), cv::Point(-1,-1), 1);
    cv::dilate(mask, mask, cv::Mat(), cv::Point(-1,-1), 1);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty()) return false;

    int largest = 0;
    double area = cv::contourArea(contours[0]);
    for (int i = 1; i < (int)contours.size(); i++) {
        double a = cv::contourArea(contours[i]);
        if (a > area) { largest = i; area = a; }
    }
    if (area < MIN_AREA) return false;

    cv::Moments m = cv::moments(contours[largest]);
    if (m.m00 == 0) return false;

    cx = (int)(m.m10 / m.m00);
    cy = (int)(m.m01 / m.m00);
    areaOut = area;
    return true;
}

int main() {
    int fd = openSerial("/dev/ttyACM0");
    if (fd < 0) return 1;
    sleep(2);  // Arduino resets on serial connect

    cv::VideoCapture cap(0, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        std::cerr << "Error: could not open camera" << std::endl;
        return 1;
    }
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);   // minimise queued frames

    std::cout << "Capture: " << cap.get(cv::CAP_PROP_FRAME_WIDTH) << "x"
              << cap.get(cv::CAP_PROP_FRAME_HEIGHT) << "\n\n";
    std::cout << "Place a stationary target in view. It must remain visible\n"
              << "after a " << STEP_DEGREES << " deg pan step.\n"
              << "Press Enter to begin." << std::endl;
    std::cin.get();

    cv::Mat frame, hsv, mask;
    std::vector<double> results;

    for (int trial = 1; trial <= TRIALS; trial++) {
        // Alternate step direction so the servo returns to rest each time and
        // both directions of gear engagement are sampled.
        float target = (trial % 2 == 1) ? REST_ANGLE + STEP_DEGREES : REST_ANGLE;
        float from   = (trial % 2 == 1) ? REST_ANGLE : REST_ANGLE + STEP_DEGREES;

        // Move to the starting position and let everything come to rest.
        sendServoCommand(fd, angleToMicros(from), angleToMicros(90.0f));
        usleep(SETTLE_MS * 1000);

        // Flush stale frames, then establish the pre-step centroid.
        for (int i = 0; i < 5; i++) cap >> frame;

        int baseCx = -1, baseCy = -1;
        double area = 0;
        bool haveBase = false;
        for (int i = 0; i < 10 && !haveBase; i++) {
            cap >> frame;
            haveBase = findCentroid(frame, hsv, mask, baseCx, baseCy, area);
        }
        if (!haveBase) {
            std::cout << "Trial " << trial << ": no target before step - skipped\n";
            continue;
        }

        // Issue the step and start the clock.
        auto t0 = std::chrono::steady_clock::now();
        sendServoCommand(fd, angleToMicros(target), angleToMicros(90.0f));

        double elapsedMs = -1.0;
        int frames = 0;
        while (true) {
            cap >> frame;
            if (frame.empty()) break;
            frames++;

            auto now = std::chrono::steady_clock::now();
            double ms = std::chrono::duration<double, std::milli>(now - t0).count();

            int cx, cy;
            if (findCentroid(frame, hsv, mask, cx, cy, area)) {
                int dx = abs(cx - baseCx);
                int dy = abs(cy - baseCy);
                if (dx > MOVE_THRESHOLD_PX || dy > MOVE_THRESHOLD_PX) {
                    elapsedMs = ms;
                    break;
                }
            }
            if (ms > TIMEOUT_MS) break;
        }

        if (elapsedMs < 0) {
            std::cout << "Trial " << trial << ": timeout - no response detected\n";
        } else {
            std::cout << "Trial " << trial << ": " << elapsedMs
                      << " ms  (" << frames << " frames)\n";
            results.push_back(elapsedMs);
        }
    }

    // Return to rest.
    sendServoCommand(fd, angleToMicros(REST_ANGLE), angleToMicros(90.0f));

    if (results.empty()) {
        std::cout << "\nNo valid trials." << std::endl;
    } else {
        std::sort(results.begin(), results.end());
        double median = results[results.size() / 2];
        double sum = 0;
        for (double r : results) sum += r;

        std::cout << "\n--- Results (" << results.size() << " valid trials) ---\n";
        std::cout << "Median: " << median << " ms\n";
        std::cout << "Mean:   " << sum / results.size() << " ms\n";
        std::cout << "Min:    " << results.front() << " ms\n";
        std::cout << "Max:    " << results.back() << " ms\n";
        std::cout << "\nThis is an UPPER bound. See notes.\n";
    }

    close(fd);
    cap.release();
    return 0;
}
