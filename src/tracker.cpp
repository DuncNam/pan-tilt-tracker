#include <opencv2/opencv.hpp>
#include <iostream>

// Global HSV trackbar values
int hLow = 0, hHigh = 10;
int sLow = 120, sHigh = 255;
int vLow = 70, vHigh = 255;

int main() {
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open camera" << std::endl;
        return -1;
    }

    // Create a window with trackbars for tuning HSV range live
    cv::namedWindow("HSV Tuner");
    cv::waitKey(1);
    cv::createTrackbar("H Low",  "HSV Tuner", &hLow,  179);
    cv::createTrackbar("H High", "HSV Tuner", &hHigh, 179);
    cv::createTrackbar("S Low",  "HSV Tuner", &sLow,  255);
    cv::createTrackbar("S High", "HSV Tuner", &sHigh, 255);
    cv::createTrackbar("V Low",  "HSV Tuner", &vLow,  255);
    cv::createTrackbar("V High", "HSV Tuner", &vHigh, 255);

    cv::Mat frame, hsv, mask;

    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        // Convert BGR to HSV
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

        // Build the bounds from trackbar values
        cv::Scalar lowerBound(hLow, sLow, vLow);
        cv::Scalar upperBound(hHigh, sHigh, vHigh);

        // Create the mask
        cv::inRange(hsv, lowerBound, upperBound, mask);

        // Clean up noise
        cv::erode(mask, mask, cv::Mat(), cv::Point(-1,-1), 2);
        cv::dilate(mask, mask, cv::Mat(), cv::Point(-1,-1), 2);

        // Find contours
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        if (!contours.empty()) {
            // Find largest contour
            int largest = 0;
            for (int i = 1; i < (int)contours.size(); i++) {
                if (cv::contourArea(contours[i]) > cv::contourArea(contours[largest]))
                    largest = i;
            }

            if (cv::contourArea(contours[largest]) > 500) {
                // Compute centroid
                cv::Moments m = cv::moments(contours[largest]);
                int cx = (int)(m.m10 / m.m00);
                int cy = (int)(m.m01 / m.m00);

                // Draw centroid marker
                cv::circle(frame, cv::Point(cx, cy), 10, cv::Scalar(0, 255, 0), -1);
                cv::line(frame, cv::Point(cx-20, cy), cv::Point(cx+20, cy), cv::Scalar(0,255,0), 2);
                cv::line(frame, cv::Point(cx, cy-20), cv::Point(cx, cy+20), cv::Scalar(0,255,0), 2);

                // Draw error from frame center — this is what the PID will use later
                int frameW = frame.cols;
                int frameH = frame.rows;
                int errorX = cx - frameW / 2;
                int errorY = cy - frameH / 2;

                cv::putText(frame, "Error X: " + std::to_string(errorX),
                    cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0,255,0), 2);
                cv::putText(frame, "Error Y: " + std::to_string(errorY),
                    cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0,255,0), 2);

                // Draw frame center crosshair
                cv::drawMarker(frame, cv::Point(frameW/2, frameH/2),
                    cv::Scalar(0,0,255), cv::MARKER_CROSS, 20, 2);

                std::cout << "Centroid: (" << cx << ", " << cy << ")"
                          << "  Error: (" << errorX << ", " << errorY << ")" << std::endl;
            }
        }

        cv::imshow("Tracker", frame);
        cv::imshow("Mask", mask);

        if (cv::waitKey(1) == 'q') break;
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}