#include "aruco.hpp"

ArucoDetect::ArucoDetect()
{
    this->dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
    this->parameters = cv::aruco::DetectorParameters::create();
}

std::vector<std::vector<cv::Mat>> ArucoDetect::detect(cv::Mat input_img, double marker_size, cv::Mat camera_matrix, cv::Mat dist_coeffs)
{
    std::vector<std::vector<cv::Point2f>> corners, rejectedCandidates;
    std::vector<int> ids;
    cv::aruco::detectMarkers(input_img, this->dictionary, corners, ids, this->parameters, rejectedCandidates);
    
    std::vector<std::vector<cv::Mat>> return_values;
    if (ids.size() > 0)
    {
        for (size_t i = 0; i < ids.size(); i++)
        {
            std::vector<cv::Mat> kyori = this->measure(marker_size, corners[i], camera_matrix, dist_coeffs);
            if (kyori.empty())
            {
                continue;
            }
            
            std::vector<cv::Mat> return_value;
            return_value.push_back(cv::Mat({ids[i]}));
            return_value.insert(return_value.end(), kyori.begin(), kyori.end());
            return_values.push_back(return_value);
        }
    }
    return return_values;
}

std::vector<cv::Mat> ArucoDetect::measure(double marker_size, std::vector<cv::Point2f> image_points, cv::Mat cameraMatrix, cv::Mat distCoeffs)
{
    double half_size = marker_size / 2.0;
    std::vector<cv::Point3f> objectPoints;
    objectPoints.push_back(cv::Point3f(-half_size,  half_size, 0));
    objectPoints.push_back(cv::Point3f( half_size,  half_size, 0));
    objectPoints.push_back(cv::Point3f( half_size, -half_size, 0));
    objectPoints.push_back(cv::Point3f(-half_size, -half_size, 0));
    cv::Mat rvec, tvec;
    cv::solvePnP(
        objectPoints, image_points, cameraMatrix, distCoeffs,
        rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE
    );
    std::vector<cv::Point2f> reprojected;
    cv::projectPoints(objectPoints, rvec, tvec, cameraMatrix, distCoeffs, reprojected);
    
    double err = 0.0;
    for (size_t i = 0; i < image_points.size(); ++i) {
        cv::Point2f d = image_points[i] - reprojected[i];
        err += cv::norm(d);
    }
    err /= static_cast<double>(image_points.size());

    if (err > 2.0)
    {
        return {};
    }

    std::vector<cv::Mat> return_value;
    return_value.push_back(rvec);
    return_value.push_back(tvec);
    return return_value;
}