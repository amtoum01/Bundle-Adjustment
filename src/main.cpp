#include "control_points.hpp"
#include "correspondences.hpp"
#include "ls_solver.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/xfeatures2d.hpp"
#include "preprocess.hpp"

int main() {
    // Load the image 
    cv::Mat img1 = cv::imread("/Users/amtouti/Documents/IMG_1034.jpeg", cv::IMREAD_COLOR);
    cv::Mat img2 = cv::imread("/Users/amtouti/Documents/IMG_1035.jpeg", cv::IMREAD_COLOR);
    cv::String folder = "/Users/amtouti/Documents/SFM/2ViewBAGivenCorresspondence/3d_sample_data_set_sofa/3D_sofas";
    std::vector<MouseCallbackParams*> control_points;
    std::vector<cv::Point3f> XYZ_control_points;
    ControlPoints processor;

    bool data_ready = 1;
    bool debug = 0;
    bool get_more_correspondences = 0;

    // AKAZE DETECTOR START

    std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
    cv::Mat descriptors_1, descriptors_2;
    cv::Ptr<cv::FeatureDetector> detector = cv::xfeatures2d::SURF::create();
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::SIFT::create();
    
    cv::Ptr<cv::DescriptorMatcher> matcher  = cv::DescriptorMatcher::create("BruteForce");

    detector->detect(img1, keypoints_1);
    detector->detect(img2, keypoints_2);

    descriptor->compute(img1, keypoints_1, descriptors_1);
    descriptor->compute(img2, keypoints_2, descriptors_2);

    std::vector<cv::DMatch> matches;
    matcher->match(descriptors_1, descriptors_2, matches);


    cv::Mat img_match;
    std::cout << "MATCHES SIZE IS: " <<  matches.size() << std::endl;

    std::vector<cv::Point2f> points1, points2;
    
    for (const auto& match : matches) {
        points1.push_back(keypoints_1[match.queryIdx].pt);
        points2.push_back(keypoints_2[match.trainIdx].pt);
    }

    cv::Mat mask;
    cv::Mat fundamental_matrix = findFundamentalMat(points1, points2, cv::FM_RANSAC, 15, 0.99, mask);

    std::vector<Correspondence> Corr_auto_matches;
    // Correspondence mtch
    for(int i=0; i<points1.size(); i++){
        if(mask.at<uchar>(i) == 1){
            Correspondence mtch;
            cv::Point2d m1 = keypoints_1[matches[i].queryIdx].pt;
            cv::Point2d m2 = keypoints_2[matches[i].trainIdx].pt;
            mtch.p1 = m1;
            mtch.p2 = m2;
            Corr_auto_matches.push_back(mtch);
        } 
    }

    //AKAZE DETECTOR END
    
    processor.processControlPoints(folder, control_points, XYZ_control_points, data_ready); 

    if(debug){
        processor.printControlPixels(control_points);
        processor.printControlXYZ(XYZ_control_points);
    }
    

    std::vector<Correspondence> Corr;
    GetCorrespondence Correspond(img1, img2);
    Correspond.readCorrFromFile(Corr);
    
    if(get_more_correspondences){
        Correspond.getCorrespondence(Corr);
            std::cout << "Values After getting correspondences" << std::endl;
        for(const auto& corr: Corr){
            std::cout << corr.p1 << " " << corr.p2 << std::endl;
        }
        Correspond.writeCorrToFile(Corr);

    }
     

    MouseCallbackParams* last_image = new MouseCallbackParams;
    std::vector<cv::Point2i> control_points_pixel;

    last_image = control_points.back();
    for(const auto& control_pixel: last_image->dataPoints){
        control_points_pixel.push_back(control_pixel.pixelLocation);
    }

    std::vector<cv::Point2d> correspondence_p1;
    std::vector<cv::Point2d> correspondence_p2;

    for(const auto& corr: Corr_auto_matches){
        correspondence_p1.push_back(corr.p1);
        correspondence_p2.push_back(corr.p2);
    }

    cv::Mat corr_p1 = cv::Mat(correspondence_p1);
    cv::Mat corr_p2 = cv::Mat(correspondence_p2);
    
    double fx = 1617.0; 
    double fy = 1617.0; 
    double cx = 1510.0; 
    double cy = 2016.0; 
    double skew = 0.0; 

    cv::Mat K = (cv::Mat_<double>(3, 3) <<
                 fx, skew, cx,
                 0, fy, cy,
                 0, 0, 1);

    
    cv::Mat essential_matrix =  cv::findEssentialMat(corr_p1, corr_p2, K, 
                                                     cv::RANSAC, 0.999, 3.0, 1000);	

    cv::Mat R, t;
    cv::Mat mask_;
    int inliers = cv::recoverPose(essential_matrix, corr_p1, corr_p2, K ,R, t, mask_);

    std::cout << "NUMBER OF INLIERS IN RECIVER POSE IS: " << inliers << std::endl;

    cv::Mat transformationMatrix_2 = cv::Mat::zeros(3, 4, CV_64F);
    
    R.copyTo(transformationMatrix_2(cv::Rect(0, 0, 3, 3)));
    t.copyTo(transformationMatrix_2(cv::Rect(3, 0, 1, 3)));
    
    cv::Mat transformationMatrix_1 = (cv::Mat_<double>(3,4) <<
                                        1, 0, 0, 0,
                                        0, 1, 0, 0,
                                        0, 0, 1, 0);


    cv::Mat projecMatrix_2 = K * transformationMatrix_2;

    cv::Mat projecMatrix_1 =  K * transformationMatrix_1;
    cv::Mat world_points;

    cv::triangulatePoints(projecMatrix_1, projecMatrix_2, corr_p1, corr_p2, world_points);

    std::vector<cv::Mat> homogeneous_points;
    std::vector<cv::Point3f> disp_points;
    std::vector<cv::Vec3b> colors;

    getHomogneousPoints(world_points, Corr_auto_matches, img1, homogeneous_points, disp_points, colors);
    
    LS_SOLVER ls_solver(Corr_auto_matches, homogeneous_points, transformationMatrix_1, transformationMatrix_2, K);
    ls_solver.solveLS();

    std::vector<cv::Point3f> disp_points_;

    getLSPoints(ls_solver.x, disp_points_);
   
    // visualise(disp_points_, colors, transformationMatrix_1, ls_solver.transformation_matrix_2, ls_solver.K);
    
    cv::waitKey(0);
    cv::destroyAllWindows();
    visualise(disp_points, colors, transformationMatrix_1, transformationMatrix_2, K);

    int x = 98;
    
    std::vector<int> l{x,x,x,x};
    
    return 0;
}