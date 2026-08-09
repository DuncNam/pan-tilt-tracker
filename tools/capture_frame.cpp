// capture_frame.cpp
//
// Interactive aiming plus single-frame capture, for measuring the camera's
// true focal length in pixels.
//
// Purpose: everything in this project that converts between pixels and angles
// depends on f_px, which is currently assumed from a published 58 deg diagonal
// FOV spec rather than measured. The step-response test implied a 9%
// discrepancy between commanded and observed rotation, which is either a servo
// scale error or an f_px error. This measurement separates them.
//
// Procedure:
//   1. Tape two marks a known distance apart on a flat wall. Wider is better --
//      a yard/metre stick, or two pieces of tape 24 in apart. Place them at
//      roughly the height of the camera lens.
//   2. Run this tool and use the servo commands below to aim. Get both marks
//      near the vertical centre and roughly symmetric about the red centre
//      line -- lens distortion is worst at the edges, and a square-on view
//      keeps both marks at the same range.
//   3. Measure the perpendicular distance from the camera LENS to the wall.
//      This is the dominant error term -- an error here maps 1:1 into f_px.
//   4. Capture, then read the x pixel coordinate of each mark.
//   5. f_px = (pixel_separation * distance) / real_separation
//      Distance and real_separation must be in the same units.
//
// Reference values to compare against:
//   f_px = 1325  -> 58.0 deg diagonal FOV (the published spec)
//   f_px = 1207  -> 62.6 deg diagonal FOV (what the step test implies)
//
// If f_px lands near 1325, the camera model is right and angleToMicros is
// optimistic -- the servo moves less per microsecond than assumed.
// If f_px lands near 1207, the FOV spec is wrong and the servo scale is right.
//
// Build:
//   g++ src/capture_frame.cpp -o capture_frame $(pkg-config --cflags --libs opencv4)
//
// Run (stop ./tracker first -- it holds the camera and the serial port):
//   ./capture_frame
//
// Commands (type at the prompt, then Enter):
//   <pan> <tilt>   absolute angles in degrees, e.g.  90 85
//   a / d          nudge pan  -1 / +1 deg
//   w / s          nudge tilt +1 / -1 deg
//   A / D          nudge pan  -5 / +5 deg
//   W / S          nudge tilt +5 / -5 deg
//   p              preview: capture and overwrite preview.png without saving
//   c              capture and write calib_frame.png + calib_frame_grid.png
//   q              quit (returns servos to 90, 90)
//
// Output: calib_frame.png, calib_frame_grid.png, preview.png

#include <opencv2/opencv.hpp>
#include <iostream>
#include <sstream>
#include <string>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

// Must match tracker.cpp
const int US_MIN     = 500;
const int US_MAX     = 2500;
const int SERVO_MIN  = 10;
const int SERVO_MAX  = 170;

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

float clampAngle(float a) {
    if (a < SERVO_MIN) return SERVO_MIN;
    if (a > SERVO_MAX) return SERVO_MAX;
    return a;
}

// Overlay a 100 px reference grid with labelled columns, plus brighter centre
// lines, so the image can be measured by eye without a coordinate readout.
cv::Mat addGrid(const cv::Mat& src) {
    cv::Mat out = src.clone();
    int w = out.cols, h = out.rows;

    for (int x = 0; x < w; x += 100) {
        cv::line(out, cv::Point(x, 0), cv::Point(x, h), cv::Scalar(0,255,0), 1);
        cv::putText(out, std::to_string(x), cv::Point(x + 3, 18),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0,255,0), 1);
    }
    for (int y = 0; y < h; y += 100) {
        cv::line(out, cv::Point(0, y), cv::Point(w, y), cv::Scalar(0,255,0), 1);
        cv::putText(out, std::to_string(y), cv::Point(3, y - 4),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0,255,0), 1);
    }
    cv::line(out, cv::Point(w/2, 0), cv::Point(w/2, h), cv::Scalar(0,0,255), 1);
    cv::line(out, cv::Point(0, h/2), cv::Point(w, h/2), cv::Scalar(0,0,255), 1);
    return out;
}

int main() {
    int fd = openSerial("/dev/ttyACM0");
    if (fd < 0) {
        std::cerr << "Continuing without servo control." << std::endl;
    } else {
        sleep(2);   // Arduino resets when the serial port opens
    }

    cv::VideoCapture cap(0, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        std::cerr << "Error: could not open camera" << std::endl;
        return 1;
    }
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);

    // Match tracker.cpp. The measurement should be taken under the same optical
    // conditions the tracker runs in.
    system("v4l2-ctl -d /dev/video0 --set-ctrl=exposure_dynamic_framerate=0");
    system("v4l2-ctl -d /dev/video0 --set-ctrl=auto_exposure=1");
    system("v4l2-ctl -d /dev/video0 --set-ctrl=exposure_time_absolute=100");
    system("v4l2-ctl -d /dev/video0 --set-ctrl=gain=147");

    int w = (int)cap.get(cv::CAP_PROP_FRAME_WIDTH);
    int h = (int)cap.get(cv::CAP_PROP_FRAME_HEIGHT);

    float pan = 90.0f, tilt = 90.0f;
    sendServoCommand(fd, angleToMicros(pan), angleToMicros(tilt));

    std::cout << "Capture: " << w << "x" << h << "\n"
              << "Frame centre is (" << w/2 << ", " << h/2 << ")\n\n"
              << "Commands:\n"
              << "  <pan> <tilt>   absolute degrees, e.g.  90 85\n"
              << "  a/d  w/s       nudge 1 deg   (pan left/right, tilt up/down)\n"
              << "  A/D  W/S       nudge 5 deg\n"
              << "  p              preview -> preview.png\n"
              << "  c              capture -> calib_frame.png + _grid.png\n"
              << "  q              quit\n\n";

    cv::Mat frame;
    std::string line;

    while (true) {
        std::cout << "[pan " << pan << "  tilt " << tilt << "] > " << std::flush;
        if (!std::getline(std::cin, line)) break;

        bool moved = false;
        bool doCapture = false;
        bool doPreview = false;

        if (line == "q") {
            break;
        } else if (line == "c") {
            doCapture = true;
        } else if (line == "p") {
            doPreview = true;
        } else if (line == "a") { pan  = clampAngle(pan  - 1); moved = true; }
        else if   (line == "d") { pan  = clampAngle(pan  + 1); moved = true; }
        else if   (line == "w") { tilt = clampAngle(tilt + 1); moved = true; }
        else if   (line == "s") { tilt = clampAngle(tilt - 1); moved = true; }
        else if   (line == "A") { pan  = clampAngle(pan  - 5); moved = true; }
        else if   (line == "D") { pan  = clampAngle(pan  + 5); moved = true; }
        else if   (line == "W") { tilt = clampAngle(tilt + 5); moved = true; }
        else if   (line == "S") { tilt = clampAngle(tilt - 5); moved = true; }
        else {
            std::istringstream iss(line);
            float p, t;
            if (iss >> p >> t) {
                pan  = clampAngle(p);
                tilt = clampAngle(t);
                moved = true;
            } else if (!line.empty()) {
                std::cout << "  unrecognised\n";
                continue;
            } else {
                continue;
            }
        }

        if (moved) {
            int pu = angleToMicros(pan), tu = angleToMicros(tilt);
            sendServoCommand(fd, pu, tu);
            std::cout << "  -> " << pu << ", " << tu << " us\n";
            usleep(400000);   // let the servo settle before the next capture
            continue;
        }

        // Discard stale frames. The V4L2 driver queues, and after sitting at a
        // prompt the queue holds images from before the last servo move.
        for (int i = 0; i < 15; i++) cap >> frame;
        if (frame.empty()) {
            std::cerr << "  empty frame\n";
            continue;
        }

        if (doPreview) {
            cv::imwrite("preview.png", addGrid(frame));
            std::cout << "  wrote preview.png\n";
        }
        if (doCapture) {
            cv::imwrite("calib_frame.png", frame);
            cv::imwrite("calib_frame_grid.png", addGrid(frame));
            std::cout << "  wrote calib_frame.png and calib_frame_grid.png\n"
                      << "  pan " << pan << " deg (" << angleToMicros(pan)
                      << " us), tilt " << tilt << " deg ("
                      << angleToMicros(tilt) << " us)\n\n"
                      << "  f_px = (pixel_separation * distance) / real_separation\n"
                      << "  Compare: 1325 = 58.0 deg dFOV (spec), "
                         "1207 = 62.6 deg dFOV (step test)\n\n";
        }
    }

    if (fd >= 0) {
        sendServoCommand(fd, angleToMicros(90.0f), angleToMicros(90.0f));
        close(fd);
    }
    cap.release();
    return 0;
}