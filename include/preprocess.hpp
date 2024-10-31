#ifndef PREPROCESS
#define PREPROCESS

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/viz.hpp>
#include "correspondences.hpp"

void getHomogneousPoints(const cv::Mat& world_points, const std::vector<Correspondence>& Corr, const cv::Mat& img, std::vector<cv::Mat>& homogeneous_points, std::vector<cv::Point3f>& disp_points, std::vector<cv::Vec3b>& colors);
void getLSPoints(const cv::Mat& x, std::vector<cv::Point3f>& disp_points);
void visualise(const std::vector<cv::Point3f>& disp_points, const std::vector<cv::Vec3b>& colors, const cv::Mat& transformation_matrix_1, const cv::Mat& transformation_matrix_2, const cv::Mat& K);



#endif