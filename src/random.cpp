#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "correspondences.hpp"

using namespace cv;
using namespace cv::xfeatures2d;

int main() {
    // Load the two images
    Mat img1 = imread("/Users/amtouti/Documents/IMG_1037.jpeg");
    Mat img2 = imread("/Users/amtouti/Documents/IMG_1038.jpeg");
    if (img1.empty() || img2.empty()) {
        std::cerr << "Error loading images!" << std::endl;
        return -1;
    }

    // ORB or AKAZE feature detector
    Ptr<Feature2D> detector = SURF::create(); // or AKAZE::create();
    Ptr<Feature2D> descriptor = SIFT::create();

    std::vector<KeyPoint> keypoints1, keypoints2;

    int maxCorners = 500;
    double qualityLevel = 0.01;
    double minDistance = 10;
    int blockSize = 3;
    bool useHarrisDetector = false;
    double k = 0.04;

    // std::vector<cv::Point2f> corners1, corners2;
    // goodFeaturesToTrack(img1, corners1, maxCorners, qualityLevel, minDistance,
    //                     Mat(), blockSize, useHarrisDetector, k);
    // goodFeaturesToTrack(img2, corners2, maxCorners, qualityLevel, minDistance,
    //                     Mat(), blockSize, useHarrisDetector, k);

    // for(const auto& corner: corners1){
    //     cv::KeyPoint kpt;
    //     kpt.pt = corner;
    //     kpt.size = blockSize;
    //     keypoints1.push_back(kpt);
    // }

    // for(const auto& corner: corners2){
    //     cv::KeyPoint kpt;
    //     kpt.pt = corner;
    //     kpt.size = blockSize;
    //     keypoints2.push_back(kpt);
    // }

    // Detect keypoints and compute descriptors in the first image
    
    Mat descriptors_1, descriptors_2;
    detector->detect(img1, keypoints1);
    detector->detect(img2, keypoints2);

    std::cout << keypoints1.size() << std::endl;
    std::cout << keypoints2.size() << std::endl;

    descriptor->compute(img1, keypoints1, descriptors_1);
    descriptor->compute(img2, keypoints2, descriptors_2);

    // Calculate dense optical flow using Lucas-Kanade method
    std::vector<Point2f> prevPoints, nextPoints;
    // for (const auto& kp : keypoints1) {
    //     prevPoints.push_back(kp.pt);
    // }

    

    cv::Ptr<cv::DescriptorMatcher> matcher  = cv::DescriptorMatcher::create("BruteForce");
    std::vector<cv::DMatch> matches;
    matcher->match(descriptors_1, descriptors_2, matches);
    std::vector<DMatch> good_matches;
    // for (size_t i = 0; i < knn_matches.size(); ++i) {
    //     if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance) {
    //         good_matches.push_back(knn_matches[i][0]);
    //     }
    // }

    Mat img_matches_;
    drawMatches(img1, keypoints1, img2, keypoints2, matches, img_matches_);
    imshow("Matches", img_matches_);
    waitKey(0);
    cv::destroyAllWindows();

    std::vector<Correspondence> Corr;
    GetCorrespondence getCorr(img1, img2);
    getCorr.readCorrFromFile(Corr);

    std::vector<Point2f> points1, points2;
    std::vector<int> matches_ind;

    // for(const auto& corr: Corr){
    //     points1.push_back(corr.p1);
    //     points2.push_back(corr.p2);
        
    // }

    int match_count=0;
    
    for (const auto& match : matches) {
        if(match.distance < 100.0f){
            // std::cout << match.distance << std::endl;
            matches_ind.push_back(match_count); 
        }
        points1.push_back(keypoints1[match.queryIdx].pt);
        points2.push_back(keypoints2[match.trainIdx].pt);
        match_count++;
    }


    // START OF EPIPOLAR SECTION
    
    // std::vector<Point2f> points1_, points2_;
    // for(int i=0; i<20; i++){
    //     points1_.push_back(Corr[i].p1);
    //     points2_.push_back(Corr[i].p2);
    // }

    cv::Mat mask;

    Mat fundamental_matrix = findFundamentalMat(points1, points2, FM_RANSAC, 100, 0.99, mask);

    std::vector<Vec3f> lines1, lines2;
    computeCorrespondEpilines(points1, 1, fundamental_matrix, lines2);
    computeCorrespondEpilines(points2, 2, fundamental_matrix, lines1);
    

    std::cout << lines1.size() << " " << lines2.size() << std::endl;
    double inliers = 0;

    for(int i=0; i<mask.size[0]; i++){
        inliers = inliers + mask.at<uchar>(i);
    }
    std::cout << "NUMBER OF INLIERS IS: " << cv::countNonZero(mask) << " OUT OF " << mask.size[0] << std::endl;


    cv::Mat img1_cp;
    img1.copyTo(img1_cp);

    cv::Mat img2_cp;
    img2.copyTo(img2_cp);
    
    // Draw lines on the images
    RNG rng(0); // Random color generator
    // cv::Mat img1_cp;
    // cv::cvtColor(img1, img1_cp, cv::COLOR_GRAY2BGR);
    img1.copyTo(img1_cp);
    // cv::Mat img2_cp;
    // cv::cvtColor(img2, img2_cp, cv::COLOR_GRAY2BGR);
    img2.copyTo(img2_cp);
    cv::Mat img_epi_conc;

    for (size_t i = 0; i < points1.size(); ++i) {
        
        Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        if(mask.at<uchar>(i) == 1){
            img1.copyTo(img1_cp);
            img2.copyTo(img2_cp);
            circle(img1_cp, points1[i], 25, Scalar(0,255,0),15);
            circle(img2_cp, points2[i], 25, Scalar(255,0,0),15);
            cv::hconcat(img1_cp, img2_cp, img_epi_conc);
            imshow("Image 1 with Epipolar Lines", img_epi_conc);
            waitKey(0);
        }
        // line(img1_cp, Point(0, -lines1[i][2] / lines1[i][1]), Point(img1.cols, -(lines1[i][2] + lines1[i][0] * img1.cols) / lines1[i][1]), color, 10);
        // line(img2_cp, Point(0, -lines2[i][2] / lines2[i][1]), Point(img2.cols, -(lines2[i][2] + lines2[i][0] * img2.cols) / lines2[i][1]), color, 10);
    }

    cv::destroyAllWindows();
    cv::hconcat(img1_cp, img2_cp, img_epi_conc);
    // std::cout << "The distance of this match is: " << matches[i].distance << std::endl;
    imshow("Image 1 with Epipolar Lines", img_epi_conc);
    waitKey(0);

    // Display the images with epipolar lines
    // imshow("Image 1 with Epipolar Lines", img1_cp);
    // waitKey(0);
    // imshow("Image 2 with Epipolar Lines", img2_cp);
    // waitKey(0);
    

    // END OF EPIPOLAR SECTION

   
   
   
   
    // START OF OPTICAL FLOW SECTION
   
    for (int i=0; i<100; i++) {
        prevPoints.push_back(keypoints1[i].pt);
    }


    std::vector<uchar> status;
    std::vector<float> error;
    cv::Mat flow_img;
    calcOpticalFlowPyrLK(img1, img2, prevPoints, nextPoints, status, error, Size(11,11), 5);

    // Draw correspondences between the keypoints in the two frames
    Mat img_matches;
    Mat img_concat;
    hconcat(img1, img2, img_concat);
    cvtColor(img_concat, img_matches, COLOR_GRAY2BGR);

    int cols = img2.size[1];
    // for (size_t i = 0; i < prevPoints.size(); i++) {
    //     cvtColor(img_concat, img_matches, COLOR_GRAY2BGR);
    //     if (status[i]) {
    //         cv::Point2f nxtpt(nextPoints[i].x + cols, nextPoints[i].y);
    //         // line(img_matches, nxtpt, prevPoints[i], Scalar(0, 255, 0), 2);
    //         circle(img_matches,  prevPoints[i], 20, Scalar(0, 255, 255), -1);
    //         circle(img_matches,  nxtpt, 20, Scalar(0, 0, 255), -1);
    //         imshow("Matches", img_matches);
    //         waitKey(0);
    //     }
    // }

    double pyr_scale = 0.5;   // Scale factor for image pyramid
    int levels = 3;           // Number of pyramid levels
    int winsize = 15;         // Window size for optical flow calculation
    int iterations = 3;       // Number of iterations at each pyramid level
    int poly_n = 5;           // Size of the pixel neighborhood
    double poly_sigma = 1.2;  // Standard deviation for Gaussian filter
    int flags = 0;            // Additional flags (not used in this example)

    // Calculate dense optical flow using Farneback method
    cv::Mat flow;
    cv::calcOpticalFlowFarneback(img1, img2, flow, pyr_scale, levels, winsize, iterations, poly_n, poly_sigma, flags);

    // Extract corresponding points from the optical flow
    std::vector<cv::Point2f> points1__, points2__;
    for (int y = 0; y < img1.rows; y++) {
        for (int x = 0; x < img1.cols; x++) {
            // Get the flow vector for the current point
            cv::Point2f flow_at_point = flow.at<cv::Point2f>(y, x);

            // std::cout << "FLOW AT POINT: (" << y << ", " << x << ") IS " << flow_at_point << std::endl;
            
            // Calculate the corresponding point in the second image
            cv::Point2f point2(x + flow_at_point.x, y + flow_at_point.y);
            
            // Store the corresponding points
            points1__.push_back(cv::Point2f(x, y));
            points2__.push_back(point2);
        }
    }

    // for(int i=0; i<points1__.size(); i+=1000){
    //     std::cout << points1__[i] << " " << points2__[i] << std::endl;
    // }

    // for(int i=0; i<points1.size(); i+=1000){
    //      cv::Mat img1_cp;
    //     cv::cvtColor(img1, img1_cp, cv::COLOR_GRAY2BGR);
    //     // img1.copyTo(img1_cp);
    //     cv::Mat img2_cp;
    //     cv::cvtColor(img2, img2_cp, cv::COLOR_GRAY2BGR);
    //     // img2.copyTo(img2_cp);
    //     cv::Mat img_epi_conc;

    //     Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
    //     // line(img1_cp, Point(0, -lines1[i][2] / lines1[i][1]), Point(img1.cols, -(lines1[i][2] + lines1[i][0] * img1.cols) / lines1[i][1]), color, 10);
    //     circle(img1_cp, points1__[i], 25, Scalar(0,255,0),15);
    //     circle(img2_cp, points2__[i], 25, Scalar(255,0,0),15);
    //     // line(img2_cp, Point(0, -lines2[i][2] / lines2[i][1]), Point(img2.cols, -(lines2[i][2] + lines2[i][0] * img2.cols) / lines2[i][1]), color, 10);
    //     cv::hconcat(img1_cp, img2_cp, img_epi_conc);
    //     imshow("Image 1 with Epipolar Lines", img_epi_conc);
    //     waitKey(0);

    // }

    // cv::Mat flow_vis;
    // cv::cvtColor(img1, flow_vis, cv::COLOR_GRAY2BGR);
    // for (size_t i = 0; i < points1__.size(); i++) {
    //     cv::line(flow_vis, points1__[i], points2__[i], cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    //     cv::circle(flow_vis, points2__[i], 15, cv::Scalar(0, 0, 255), -1);
    // }

    // cv::imshow("Optical Flow", flow_vis);
    // cv::waitKey(0);
    // cv::destroyAllWindows();

    // Display the matches
    

    return 0;
}
