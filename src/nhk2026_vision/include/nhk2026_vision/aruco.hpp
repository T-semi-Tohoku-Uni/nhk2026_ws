#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <iostream>

class ArucoDetect
{
public:
    ArucoDetect();
    std::vector<std::vector<cv::Mat>> detect(cv::Mat input_img, double marker_size, cv::Mat camera_matrix, cv::Mat dist_coeffs);

private:
    std::vector<cv::Mat> measure(double marker_size, std::vector<cv::Point2f> image_points, cv::Mat cameraMatrix, cv::Mat distCoeffs);

    cv::Ptr<cv::aruco::DetectorParameters> parameters;
    cv::Ptr<cv::aruco::Dictionary> dictionary;
};