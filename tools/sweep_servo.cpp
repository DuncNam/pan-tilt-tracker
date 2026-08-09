// sweep_servo.cpp
//
// Measures the pan servo's transfer function across the rated pulse-width
// range: pulse width -> observed angle, plus deadband and backlash.
//
// WHY THIS AND NOT THE STEP TEST
//
// measure_step.cpp derives deg/us by differencing two settled positions. Both
// endpoints carry the servo's own positioning error, so that error enters the
// result at full weight. It gave 0.086 deg/us +/- 4-5%, which straddles the
// 0.090 assumed in tracker.cpp -- not tight enough to say the code is wrong.
//
// This tool instead samples many points along the range and fits a line. The
// per-point positioning noise averages down as sqrt(N) rather than
// accumulating into a single difference. Residual uncertainty is then
// dominated by the focal length (+/-2%), which is as good as this rig gets
// without a proper angular reference.
//
// WHAT COMES OUT
//
//   SLOPE      deg/us over the rated range, from a least-squares fit.
//              Compare against the 0.090 implied by US_MIN/US_MAX in
//              tracker.cpp (500-2500 us = 180 deg).
//   LINEARITY  residuals of that fit. Hobby servos often go nonlinear near
//              the ends of travel; if so, the usable range is narrower than
//              the rated one.
//   DEADBAND   from the fine sweep: the smallest pulse-width change that
//              produces detectable motion. Datasheet nominal for MG996R is
//              5 us. Unverified on these units.
//   BACKLASH   hysteresis between the ascending and descending sweeps at the
//              same commanded pulse width. This is lost motion in the gear
//              train and it is the term that bites hardest on a swinging
//              target, which reverses direction twice per cycle.
//
// SETUP
//
// A stationary target that stays in frame across the whole sweep. The frame
// spans ~51.7 deg, so at ~0.086 deg/us a full 1000-2000 us sweep moves the
// centroid roughly 86 deg -- FAR wider than the frame. Two options:
//
//   1. COARSE mode sweeps a window centred on SWEEP_CENTRE_US, sized to keep
//      the target visible. Use this for slope and linearity.
//   2. To characterise the full rated range, run COARSE several times at
//      different SWEEP_CENTRE_US values, re-aiming the target between runs,
//      and stitch the segments. Slope should be consistent across segments if
//      the servo is linear.
//
// Mount the target rigidly. A ball on a string drifts between samples and
// that drift lands directly in the residuals. Tape it to a wall or a stand.
//
// BUILD
//   g++ src/sweep_servo.cpp -o sweep_servo $(pkg-config --cflags --libs opencv4)
//
// RUN (stop ./tracker first -- it holds the camera and the serial port)
//   ./sweep_servo
//
// OUTPUT
//   sweep_coarse.csv   pass,direction,pulse_us,cx,cy,area
//   sweep_fine.csv     pass,direction,pulse_us,cx,cy,area

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

// --- Detection: must match tracker.cpp ---
int hLow = 45, hHigh = 85;
int sLow = 50, sHigh = 255;
int vLow = 30, vHigh = 255;
const double MIN_AREA = 300.0;

// --- Measured camera constant ---
// f_px = 1321 from the 2026-08-09 wall calibration: 670 px separation for
// 36 in of real separation at 71 in range. +/-2%. Implies 58.1 deg diagonal
// FOV, confirming the published 58 deg spec.
const double F_PX = 1321.0;

// --- Sweep parameters ---
const int COARSE_CENTRE_US = 1500;  // centre of the coarse window
const int COARSE_HALFSPAN  = 250;   // +/- this many us. 500 us total.
const int COARSE_STEP_US   = 10;    // 51 points per direction
const int COARSE_PASSES    = 3;     // up-down cycles, for hysteresis stats

const int FINE_CENTRE_US   = 1500;  // centre of the fine deadband probe
const int FINE_HALFSPAN    = 25;    // +/- this many us
const int FINE_STEP_US     = 1;     // single-microsecond resolution
const int FINE_PASSES      = 3;

const int SETTLE_MS        = 350;   // dwell after each command before sampling
const int SAMPLES_PER_POINT = 5;    // median of this many frames per point
const int TILT_US          = 1833;  // hold tilt fixed throughout

struct Point {
    int    pass;
    int    dir;        // +1 ascending, -1 descending
    int    us;
    double cx, cy, area;
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

void sendUs(int fd, int pan_us, int tilt_us) {
    if (fd < 0) return;
    std::string cmd = std::to_string(pan_us) + "," + std::to_string(tilt_us) + "\n";
    ssize_t n = write(fd, cmd.c_str(), cmd.length());
    if (n != (ssize_t)cmd.length()) {
        std::cerr << "WARNING: short serial write (" << n << " of "
                  << cmd.length() << ")" << std::endl;
    }
}

bool findCentroid(const cv::Mat& frame, cv::Mat& hsv, cv::Mat& mask,
                  const cv::Mat& closeKernel,
                  double& cx, double& cy, double& areaOut) {
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(hLow, sLow, vLow),
                     cv::Scalar(hHigh, sHigh, vHigh), mask);
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

    // Sub-pixel: moments give a real-valued centroid. tracker.cpp truncates
    // this to int, discarding up to 1 px. Keep the fraction here -- it is free
    // precision and this is a measurement, not the control loop.
    cx = m.m10 / m.m00;
    cy = m.m01 / m.m00;
    areaOut = area;
    return true;
}

// Sample one commanded pulse width. Returns the median centroid over several
// frames to suppress per-frame detection noise.
bool samplePoint(cv::VideoCapture& cap, cv::Mat& frame, cv::Mat& hsv,
                 cv::Mat& mask, const cv::Mat& kernel,
                 double& cx, double& cy, double& area) {
    // Drain queued frames. The V4L2 driver buffers, so the first frames after
    // a dwell are from before the servo moved.
    for (int i = 0; i < 6; i++) cap.grab();

    std::vector<double> xs, ys, as;
    for (int i = 0; i < SAMPLES_PER_POINT; i++) {
        cap >> frame;
        if (frame.empty()) continue;
        double x, y, a;
        if (findCentroid(frame, hsv, mask, kernel, x, y, a)) {
            xs.push_back(x); ys.push_back(y); as.push_back(a);
        }
    }
    if (xs.size() < 2) return false;

    std::sort(xs.begin(), xs.end());
    std::sort(ys.begin(), ys.end());
    std::sort(as.begin(), as.end());
    cx   = xs[xs.size()/2];
    cy   = ys[ys.size()/2];
    area = as[as.size()/2];
    return true;
}

// Pixel x -> angle off the optical axis, in degrees. The pinhole projection is
// x = f*tan(alpha), so alpha = atan(x/f). Linear only near centre; this matters
// because the sweep deliberately runs the target out toward the frame edges.
double pxToDeg(double cx, double frameW) {
    return std::atan((cx - frameW / 2.0) / F_PX) * 180.0 / CV_PI;
}

// Least-squares fit of angle vs pulse width. Returns slope in deg/us.
void fitLine(const std::vector<double>& x, const std::vector<double>& y,
             double& slope, double& intercept, double& rmsResidual) {
    int n = (int)x.size();
    double sx = 0, sy = 0, sxx = 0, sxy = 0;
    for (int i = 0; i < n; i++) {
        sx += x[i]; sy += y[i]; sxx += x[i]*x[i]; sxy += x[i]*y[i];
    }
    double denom = n * sxx - sx * sx;
    slope     = (n * sxy - sx * sy) / denom;
    intercept = (sy - slope * sx) / n;

    double ss = 0;
    for (int i = 0; i < n; i++) {
        double r = y[i] - (slope * x[i] + intercept);
        ss += r * r;
    }
    rmsResidual = std::sqrt(ss / n);
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

    system("v4l2-ctl -d /dev/video0 --set-ctrl=exposure_dynamic_framerate=0");
    system("v4l2-ctl -d /dev/video0 --set-ctrl=auto_exposure=1");
    system("v4l2-ctl -d /dev/video0 --set-ctrl=exposure_time_absolute=100");
    system("v4l2-ctl -d /dev/video0 --set-ctrl=gain=147");

    double frameW = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(15,15));
    cv::Mat frame, hsv, mask;

    int coarseLo = COARSE_CENTRE_US - COARSE_HALFSPAN;
    int coarseHi = COARSE_CENTRE_US + COARSE_HALFSPAN;

    std::cout << "Capture: " << frameW << "x"
              << cap.get(cv::CAP_PROP_FRAME_HEIGHT) << "\n"
              << "f_px = " << F_PX << "\n\n"
              << "COARSE sweep: " << coarseLo << "-" << coarseHi << " us, step "
              << COARSE_STEP_US << ", " << COARSE_PASSES << " passes\n"
              << "FINE   sweep: " << (FINE_CENTRE_US - FINE_HALFSPAN) << "-"
              << (FINE_CENTRE_US + FINE_HALFSPAN) << " us, step "
              << FINE_STEP_US << ", " << FINE_PASSES << " passes\n\n"
              << "Mount the target RIGIDLY -- a swinging target puts its own\n"
              << "drift into the residuals. It must stay in frame across the\n"
              << "full coarse span.\n\n"
              << "Moving to " << coarseLo << " us. Check the target is visible.\n";

    sendUs(fd, coarseLo, TILT_US);
    sleep(2);

    double cx, cy, area;
    if (samplePoint(cap, frame, hsv, mask, kernel, cx, cy, area)) {
        std::cout << "  target at cx=" << cx << " (" << pxToDeg(cx, frameW)
                  << " deg off axis), area=" << area << "\n";
    } else {
        std::cout << "  WARNING: no target detected at the low end\n";
    }

    sendUs(fd, coarseHi, TILT_US);
    sleep(2);
    if (samplePoint(cap, frame, hsv, mask, kernel, cx, cy, area)) {
        std::cout << "  target at cx=" << cx << " (" << pxToDeg(cx, frameW)
                  << " deg off axis), area=" << area << "\n";
    } else {
        std::cout << "  WARNING: no target detected at the high end\n";
    }

    std::cout << "\nIf either end failed, quit (Ctrl-C), re-aim, and adjust\n"
              << "COARSE_CENTRE_US or COARSE_HALFSPAN.\n"
              << "Press Enter to run the sweep." << std::endl;
    std::cin.get();

    // ---------------- COARSE ----------------
    std::ofstream fc("sweep_coarse.csv");
    fc << "pass,dir,pulse_us,cx,cy,area,valid,deg\n";
    std::vector<Point> coarse;

    for (int pass = 1; pass <= COARSE_PASSES; pass++) {
        for (int dir = +1; dir >= -1; dir -= 2) {
            int from = (dir > 0) ? coarseLo : coarseHi;
            int to   = (dir > 0) ? coarseHi : coarseLo;

            // Approach the start from beyond it, so the gear train is already
            // loaded in the sweep direction and the first point is not a
            // backlash artefact.
            sendUs(fd, from - dir * 60, TILT_US);
            usleep(600000);
            sendUs(fd, from, TILT_US);
            usleep(600000);

            for (int us = from; (dir > 0) ? (us <= to) : (us >= to);
                 us += dir * COARSE_STEP_US) {
                sendUs(fd, us, TILT_US);
                usleep(SETTLE_MS * 1000);

                Point p{pass, dir, us, 0, 0, 0, false};
                p.valid = samplePoint(cap, frame, hsv, mask, kernel,
                                      p.cx, p.cy, p.area);
                coarse.push_back(p);

                fc << pass << "," << dir << "," << us << "," << p.cx << ","
                   << p.cy << "," << p.area << "," << (p.valid ? 1 : 0) << ","
                   << (p.valid ? pxToDeg(p.cx, frameW) : 0.0) << "\n";
            }
            std::cout << "  pass " << pass << " "
                      << (dir > 0 ? "up" : "down") << " done\n";
        }
    }
    fc.close();

    // ---------------- FINE ----------------
    int fineLo = FINE_CENTRE_US - FINE_HALFSPAN;
    int fineHi = FINE_CENTRE_US + FINE_HALFSPAN;

    std::cout << "\nFine sweep (deadband probe)...\n";
    std::ofstream ff("sweep_fine.csv");
    ff << "pass,dir,pulse_us,cx,cy,area,valid,deg\n";
    std::vector<Point> fine;

    for (int pass = 1; pass <= FINE_PASSES; pass++) {
        for (int dir = +1; dir >= -1; dir -= 2) {
            int from = (dir > 0) ? fineLo : fineHi;
            int to   = (dir > 0) ? fineHi : fineLo;

            sendUs(fd, from - dir * 60, TILT_US);
            usleep(600000);
            sendUs(fd, from, TILT_US);
            usleep(600000);

            for (int us = from; (dir > 0) ? (us <= to) : (us >= to);
                 us += dir * FINE_STEP_US) {
                sendUs(fd, us, TILT_US);
                usleep(SETTLE_MS * 1000);

                Point p{pass, dir, us, 0, 0, 0, false};
                p.valid = samplePoint(cap, frame, hsv, mask, kernel,
                                      p.cx, p.cy, p.area);
                fine.push_back(p);

                ff << pass << "," << dir << "," << us << "," << p.cx << ","
                   << p.cy << "," << p.area << "," << (p.valid ? 1 : 0) << ","
                   << (p.valid ? pxToDeg(p.cx, frameW) : 0.0) << "\n";
            }
            std::cout << "  pass " << pass << " "
                      << (dir > 0 ? "up" : "down") << " done\n";
        }
    }
    ff.close();

    sendUs(fd, 1500, TILT_US);

    // ---------------- ANALYSIS ----------------
    std::cout << "\n===== COARSE: transfer function =====\n";

    for (int dir = +1; dir >= -1; dir -= 2) {
        std::vector<double> xs, ys;
        for (const auto& p : coarse) {
            if (p.dir != dir || !p.valid) continue;
            xs.push_back(p.us);
            ys.push_back(pxToDeg(p.cx, frameW));
        }
        if (xs.size() < 10) {
            std::cout << (dir > 0 ? "Ascending" : "Descending")
                      << ": too few valid points\n";
            continue;
        }
        double slope, icept, rms;
        fitLine(xs, ys, slope, icept, rms);
        std::cout << (dir > 0 ? "Ascending " : "Descending")
                  << ": slope " << slope << " deg/us"
                  << ", RMS residual " << rms << " deg"
                  << ", n=" << xs.size() << "\n";
    }

    {
        std::vector<double> xs, ys;
        for (const auto& p : coarse) {
            if (!p.valid) continue;
            xs.push_back(p.us);
            ys.push_back(pxToDeg(p.cx, frameW));
        }
        if (xs.size() >= 10) {
            double slope, icept, rms;
            fitLine(xs, ys, slope, icept, rms);
            std::cout << "\nCombined  : slope " << slope << " deg/us"
                      << ", RMS residual " << rms << " deg\n";
            std::cout << "tracker.cpp assumes 0.09 deg/us "
                      << "(US_MIN/MAX 500-2500 over 180 deg)\n";
            std::cout << "Ratio measured/assumed: " << (slope / 0.09) << "\n";
            std::cout << "Implied span for 1000-2000 us: "
                      << (slope * 1000.0) << " deg\n";
        }
    }

    // Backlash: mean up-minus-down angle at matching pulse widths.
    {
        double sum = 0; int n = 0;
        for (int us = coarseLo; us <= coarseHi; us += COARSE_STEP_US) {
            std::vector<double> up, dn;
            for (const auto& p : coarse) {
                if (p.us != us || !p.valid) continue;
                (p.dir > 0 ? up : dn).push_back(pxToDeg(p.cx, frameW));
            }
            if (up.empty() || dn.empty()) continue;
            double mu = 0, md = 0;
            for (double v : up) mu += v; mu /= up.size();
            for (double v : dn) md += v; md /= dn.size();
            sum += (mu - md);
            n++;
        }
        if (n > 0) {
            double mean = sum / n;
            std::cout << "\n===== BACKLASH =====\n"
                      << "Mean hysteresis (up - down): " << mean << " deg"
                      << "  = " << std::fabs(mean) / 0.086 << " us"
                      << "  over " << n << " matched pulse widths\n"
                      << "This is lost motion on direction reversal. It is the\n"
                      << "term that bites at the ends of a pendulum swing.\n";
        }
    }

    // Deadband: consecutive fine-sweep points whose angle did not change.
    {
        std::cout << "\n===== FINE: deadband probe =====\n";
        for (int dir = +1; dir >= -1; dir -= 2) {
            std::vector<std::pair<int,double>> pts;
            for (const auto& p : fine) {
                if (p.dir != dir || !p.valid || p.pass != 1) continue;
                pts.push_back({p.us, pxToDeg(p.cx, frameW)});
            }
            if (pts.size() < 5) continue;

            std::cout << (dir > 0 ? "Ascending" : "Descending") << " (pass 1):\n";
            int stall = 0, maxStall = 0;
            for (size_t i = 1; i < pts.size(); i++) {
                double d = std::fabs(pts[i].second - pts[i-1].second);
                // 0.02 deg is under half a pixel -- below detection noise.
                if (d < 0.02) { stall++; maxStall = std::max(maxStall, stall); }
                else          { stall = 0; }
            }
            std::cout << "  longest run with no detectable motion: "
                      << maxStall << " us\n";
        }
        std::cout << "MG996R datasheet nominal deadband: 5 us.\n"
                  << "Detection floor here is ~0.5 px = 0.022 deg = ~0.25 us,\n"
                  << "so runs of 1-2 us may be noise rather than deadband.\n";
    }

    std::cout << "\nWrote sweep_coarse.csv and sweep_fine.csv\n"
              << "Plot deg vs pulse_us, coloured by dir, to see the hysteresis\n"
              << "loop and any curvature directly.\n";

    close(fd);
    cap.release();
    return 0;
}