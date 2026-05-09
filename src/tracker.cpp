#include <opencv2/opencv.hpp>
#include <iostream>

// Global HSV trackbar values
int hLow = 0, hHigh = 10;
int sLow = 120, sHigh = 255;
int vLow = 70, vHigh = 255;

int main() {
    // Open the camera and initialize capture ("cap")
    cv::VideoCapture cap(0);
    // Verify camera opened
    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open camera" << std::endl;
        return -1;
    }

    // Create a window with trackbars for tuning HSV range live
    cv::namedWindow("HSV Tuner");
    cv::waitKey(1); // Ensure the window is generated before adding sliders
    cv::createTrackbar("H Low",  "HSV Tuner", &hLow,  179);
    cv::createTrackbar("H High", "HSV Tuner", &hHigh, 179);
    cv::createTrackbar("S Low",  "HSV Tuner", &sLow,  255);
    cv::createTrackbar("S High", "HSV Tuner", &sHigh, 255);
    cv::createTrackbar("V Low",  "HSV Tuner", &vLow,  255);
    cv::createTrackbar("V High", "HSV Tuner", &vHigh, 255);

    // Generate matrices for raw feed, HSV, and masked feed
    cv::Mat frame, hsv, mask;

    // Initialize program loop
    while (true) {
        // Send current frame to frame matrix
        cap >> frame;
        // Verify camera feed
        if (frame.empty()) break;

        // Convert BGR to HSV
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

        // Build the bounds from live trackbar values
        cv::Scalar lowerBound(hLow, sLow, vLow);
        cv::Scalar upperBound(hHigh, sHigh, vHigh);

        // Create the mask
        cv::inRange(hsv, lowerBound, upperBound, mask);

        // Erode twice to remove noise
        cv::erode(mask, mask, cv::Mat(), cv::Point(-1,-1), 2);
        // Dilate blob(s) twice to approximately restore mask
        cv::dilate(mask, mask, cv::Mat(), cv::Point(-1,-1), 2);

        // Create array contours and store blob outline(s)
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // Ensure there is a blob(s) in mask
        if (!contours.empty()) {
            // Find largest blob in mask
            int largest = 0;
            for (int i = 1; i < (int)contours.size(); i++) {
                if (cv::contourArea(contours[i]) > cv::contourArea(contours[largest]))
                    largest = i;
            }

            // Threshold for minimum trackable blob size is set at 500
            if (cv::contourArea(contours[largest]) > 500) {
                // Compute centroid
                cv::Moments m = cv::moments(contours[largest]); // compute moments
                // m.m00 zeroth moment (area of blob)
                int cx = (int)(m.m10 / m.m00); //m.m10 first moment X (intensity weighted X width)
                int cy = (int)(m.m01 / m.m00); //m.m01 first moment Y (intensity weighted Y width)

                // Draw centroid marker
                cv::circle(frame, cv::Point(cx, cy), 10, cv::Scalar(0, 255, 0), -1);
                cv::line(frame, cv::Point(cx-20, cy), cv::Point(cx+20, cy), cv::Scalar(0,255,0), 2);
                cv::line(frame, cv::Point(cx, cy-20), cv::Point(cx, cy+20), cv::Scalar(0,255,0), 2);

                // Calculate error from frame center
                int frameW = frame.cols;
                int frameH = frame.rows;
                int errorX = cx - frameW / 2;
                int errorY = cy - frameH / 2;
                
                // Draw error values on  frame
                cv::putText(frame, "Error X: " + std::to_string(errorX),
                    cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0,255,0), 2);
                cv::putText(frame, "Error Y: " + std::to_string(errorY),
                    cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0,255,0), 2);

                // Draw frame center crosshair
                cv::drawMarker(frame, cv::Point(frameW/2, frameH/2),
                    cv::Scalar(0,0,255), cv::MARKER_CROSS, 20, 2);

                // Print centroid and error coordinates to console for reference
                std::cout << "Centroid: (" << cx << ", " << cy << ")"
                          << "  Error: (" << errorX << ", " << errorY << ")" << std::endl;
            }
        }

        // Display frame and mask in named windows
        cv::imshow("Tracker", frame);
        cv::imshow("Mask", mask);

        // If user presses "q", quit
        if (cv::waitKey(1) == 'q') break;
    }

    cap.release(); // close camera feed
    cv::destroyAllWindows(); // close all windows
    return 0;
}