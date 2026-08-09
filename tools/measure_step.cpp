// measure_step.cpp
//
// Measures the pan axis step response by logging the FULL trajectory, not a
// single threshold crossing.
//
// Why not threshold crossing: the V4L2 driver queues frames. A "first frame
// where the centroid moved" test measures how long it takes to drain that
// queue, not how long the servo took to respond. The previous version of this
// tool returned 195 ms with sub-millisecond repeatability across trials, which
// is impossible for a mechanical response and was a dead giveaway that the
// number was set by software, not physics.
//
// Instead: command a step, then log (timestamp, centroid) for every frame for
// a fixed window. Analyse the curve afterwards. Buffer latency appears as a
// constant offset on the leading edge, which you can see and subtract, rather
// than as a hidden term you cannot separate.
//
// Two numbers come out of the trajectory, and feedforward needs them separately:
//
//   DEAD TIME  - command issued until motion begins. This is the horizon the
//                Kalman feedforward must predict ahead by.
//   RISE TIME  - motion begins until the servo settles. This is the slew rate
//                limit and it bounds how fast a target you can follow at all.
//
// Setup: stationary, well-lit target in view. The target does NOT move. The
// camera swings, so the centroid sweeps across the frame. Position the ball so
// it stays in view through a STEP_DEGREES pan move -- offset it to one side so
// the step sweeps it toward centre rather than out of frame.
//
// Build:
//   g++ src/measure_step.cpp -o measure_step $(pkg-config --cflags --libs opencv4)
//
// Run (stop ./tracker first -- it holds the camera and the serial port):
//   ./measure_step
//
// Output: step_response.csv  (trial, t_ms, cx, cy, area)

#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <chrono>

// --- Must match tracker.cpp ---
int hLow = 45, hHigh = 85;
int sLow = 50, sHigh = 255;
int vLow = 30, vHigh = 255;

const int US_MIN = 500;
const int US_MAX = 2500;

// --- Measurement parameters ---
const int   TRIALS        = 10;
const float REST_ANGLE    = 100.0f;
const float STEP_DEGREES  = 15.0f;
const int   SETTLE_MS     = 2500;   // let servo and target come fully to rest
const int   CAPTURE_MS    = 600;    // log window after the step
const double MIN_AREA     = 300.0;

// Fraction of total travel used to define motion onset and settle.
const double ONSET_FRAC   = 0.05;   // 5% of final displacement
const double SETTLE_FRAC  = 0.90;   // 90% of final displacement

struct Sample {
    double t_ms;
    int    cx, cy;
    double area;
    bool   valid;
};

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
    if (n != (ssize_t)cmd.length()) {
        std::cerr << "WARNING: short serial write (" << n << " of "
                  << cmd.length() << ")" << std::endl;
    }
}

// Detection pipeline identical to tracker.cpp, including the closing step.
bool findCentroid(const cv::Mat& frame, cv::Mat& hsv, cv::Mat& mask,
                  const cv::Mat& closeKernel, int& cx, int& cy, double& areaOut) {
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(hLow, sLow, vLow), cv::Scalar(hHigh, sHigh, vHigh), mask);
    cv::erode(mask, mask, cv::Mat(), cv::Point(-1,-1), 1);
    cv::dilate(mask, mask, cv::Mat(), cv::Point(-1,-1), 1);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, closeKernel);

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

// Given one trial's trajectory, find dead time and rise time.
// Returns false if the trial is unusable.
bool analyseTrial(const std::vector<Sample>& traj, double& deadMs,
                  double& riseMs, double& travelPx) {
    // Baseline: median of the first few valid samples.
    std::vector<int> head;
    for (const auto& s : traj) {
        if (s.valid) head.push_back(s.cx);
        if (head.size() >= 3) break;
    }
    if (head.size() < 2) return false;
    std::sort(head.begin(), head.end());
    double x0 = head[head.size() / 2];

    // Final: median of the last few valid samples.
    std::vector<int> tail;
    for (auto it = traj.rbegin(); it != traj.rend(); ++it) {
        if (it->valid) tail.push_back(it->cx);
        if (tail.size() >= 3) break;
    }
    if (tail.size() < 2) return false;
    std::sort(tail.begin(), tail.end());
    double x1 = tail[tail.size() / 2];

    travelPx = std::fabs(x1 - x0);
    if (travelPx < 50.0) return false;   // step did not register

    double onsetLevel  = ONSET_FRAC  * travelPx;
    double settleLevel = SETTLE_FRAC * travelPx;

    double tOnset = -1, tSettle = -1;
    for (const auto& s : traj) {
        if (!s.valid) continue;
        double d = std::fabs(s.cx - x0);
        if (tOnset  < 0 && d >= onsetLevel)  tOnset  = s.t_ms;
        if (tSettle < 0 && d >= settleLevel) tSettle = s.t_ms;
    }
    if (tOnset < 0 || tSettle < 0) return false;

    deadMs = tOnset;
    riseMs = tSettle - tOnset;
    return true;
}

void report(const char* label, std::vector<double> v, const char* unit) {
    if (v.empty()) { std::cout << label << ": no data\n"; return; }
    std::sort(v.begin(), v.end());
    double sum = 0; for (double x : v) sum += x;
    std::cout << label << ": median " << v[v.size()/2]
              << ", mean " << sum / v.size()
              << ", range " << v.front() << "-" << v.back()
              << " " << unit << "\n";
}

int main() {
    int fd = openSerial("/dev/ttyACM0");
    if (fd < 0) return 1;
    sleep(2);   // Arduino resets when the serial port opens

    cv::VideoCapture cap(0, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        std::cerr << "Error: could not open camera" << std::endl;
        return 1;
    }
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);

    // Same exposure lock as tracker.cpp -- auto exposure would vary frame
    // timing and invalidate the measurement.
    system("v4l2-ctl -d /dev/video0 --set-ctrl=exposure_dynamic_framerate=0");
    system("v4l2-ctl -d /dev/video0 --set-ctrl=auto_exposure=1");
    system("v4l2-ctl -d /dev/video0 --set-ctrl=exposure_time_absolute=100");
    system("v4l2-ctl -d /dev/video0 --set-ctrl=gain=147");

    cv::Mat closeKernel =
        cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(15, 15));

    std::cout << "Capture: " << cap.get(cv::CAP_PROP_FRAME_WIDTH) << "x"
              << cap.get(cv::CAP_PROP_FRAME_HEIGHT) << "\n\n"
              << "Stationary target in view. It must remain visible through a "
              << STEP_DEGREES << " deg pan step.\n"
              << "Press Enter to begin." << std::endl;
    std::cin.get();

    std::ofstream csv("step_response.csv");
    csv << "trial,t_ms,cx,cy,area,valid\n";

    cv::Mat frame, hsv, mask;
    std::vector<double> deadTimes, riseTimes, travels;

    for (int trial = 1; trial <= TRIALS; trial++) {
        // Alternate direction so the servo returns to rest each trial and both
        // directions of gear engagement are sampled. A systematic difference
        // between odd and even trials is backlash.
        bool forward = (trial % 2 == 1);
        float from = forward ? REST_ANGLE : REST_ANGLE + STEP_DEGREES;
        float to   = forward ? REST_ANGLE + STEP_DEGREES : REST_ANGLE;

        sendServoCommand(fd, angleToMicros(from), angleToMicros(100.0f));
        usleep(SETTLE_MS * 1000);

        // Drain queued frames so the log starts with fresh ones. grab() decodes
        // nothing, so this is cheap.
        for (int i = 0; i < 10; i++) cap.grab();

        std::vector<Sample> traj;
        traj.reserve(32);

        auto t0 = std::chrono::steady_clock::now();
        sendServoCommand(fd, angleToMicros(to), angleToMicros(100.0f));

        while (true) {
            cap >> frame;
            auto now = std::chrono::steady_clock::now();
            double ms = std::chrono::duration<double, std::milli>(now - t0).count();
            if (ms > CAPTURE_MS) break;
            if (frame.empty()) continue;

            Sample s{ms, -1, -1, 0.0, false};
            s.valid = findCentroid(frame, hsv, mask, closeKernel, s.cx, s.cy, s.area);
            traj.push_back(s);

            csv << trial << "," << s.t_ms << "," << s.cx << "," << s.cy << ","
                << s.area << "," << (s.valid ? 1 : 0) << "\n";
        }

        double dead, rise, travel;
        if (analyseTrial(traj, dead, rise, travel)) {
            std::cout << "Trial " << trial << " (" << (forward ? "fwd" : "rev")
                      << "): dead " << dead << " ms, rise " << rise
                      << " ms, travel " << travel << " px, "
                      << traj.size() << " frames\n";
            deadTimes.push_back(dead);
            riseTimes.push_back(rise);
            travels.push_back(travel);
        } else {
            std::cout << "Trial " << trial << ": unusable ("
                      << traj.size() << " frames)\n";
        }
    }

    sendServoCommand(fd, angleToMicros(REST_ANGLE), angleToMicros(100.0f));
    csv.close();

    std::cout << "\n--- Summary ---\n";
    report("Dead time", deadTimes, "ms");
    report("Rise time", riseTimes, "ms");
    report("Travel   ", travels,   "px");

    if (!travels.empty() && !riseTimes.empty()) {
        std::sort(travels.begin(), travels.end());
        std::sort(riseTimes.begin(), riseTimes.end());
        double px  = travels[travels.size()/2];
        double rms = riseTimes[riseTimes.size()/2];
        double deg = px * 0.0433;                 // 0.0433 deg/px at 1280x720
        if (rms > 0) {
            std::cout << "\nImplied slew rate: " << (deg / (rms / 1000.0))
                      << " deg/s over a " << STEP_DEGREES << " deg step\n";
        }
    }

    std::cout << "\nTrajectories written to step_response.csv\n"
              << "Dead time INCLUDES camera exposure and pipeline latency.\n"
              << "Frame period is ~33 ms, so dead time is quantised to that.\n";

    close(fd);
    cap.release();
    return 0;
}