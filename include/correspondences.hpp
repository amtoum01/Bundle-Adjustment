#ifndef CORRESPONDENCES
#define CORRESPONDENCES

#include <iostream>
#include <opencv2/opencv.hpp>
#include <fstream>

struct Correspondence{
    cv::Point2d p1;
    cv::Point2d p2;
};

class GetCorrespondence
{
private:
    static void onMouse(int event, int x, int y, int flag, void* userdata);
    cv::Mat img1, img2;
public:
    void getCorrespondence(std::vector<Correspondence>& Corr);
    void writeCorrToFile(const std::vector<Correspondence>& Corr);
    void readCorrFromFile(std::vector<Correspondence>& Corr);
    GetCorrespondence(const cv::Mat& img1,const cv::Mat& img2);
};

#endif
