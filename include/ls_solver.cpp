#include "ls_solver.hpp"


cv::Point2d LS_SOLVER::get_err(const cv::Point2d& p,const cv::Mat& transformation_matrix,const cv::Mat& K,const cv::Mat& hom_XYZ){


    cv::Mat p_est = K * transformation_matrix * hom_XYZ;
    p_est /= p_est.at<double>(2);
    cv::Point2d p_est_;
    p_est_.x = p_est.at<double>(0);
    p_est_.y = p_est.at<double>(1);
    cv::Point2d diff = p - p_est_;
    return diff;


}

double LS_SOLVER::total_error_func(cv::Mat& residual){
    double err = 0;
    cv::Mat p1;
    cv::Mat p2;
    double err_1;
    double err_2;
    
    int split = 2*num_points;

    cv::Point2d p1_err;
    cv::Point2d p2_err;
    for(int i=0; i<num_points; i++){

        p1_err = get_err(Corr[i].p1, transformation_matrix_1, K, hom_XYZ[i]);
        p2_err = get_err(Corr[i].p2, transformation_matrix_2, K, hom_XYZ[i]);

        residual.at<double>(2*i) = p1_err.x;
        residual.at<double>(2*i+1) = p1_err.y;

        residual.at<double>(2*i + split) = p2_err.x;
        residual.at<double>(2*i + split + 1) = p2_err.y;

        err_1 = (std::pow(p1_err.x, 2) + std::pow(p1_err.y,2));
        err_2 = (std::pow(p2_err.x,2) + std::pow(p2_err.y,2));

        err += std::sqrt(err_1) + std::sqrt(err_2);
    }

    std::cout << "TOTAL ERROR IS: " << err << std::endl;
    std::cout << std::endl;

    return err;
}

void LS_SOLVER::rotationToRodriguez(const cv::Mat& rotation_mat, cv::Mat& rodriguez_vec){
    cv::Rodrigues(rotation_mat, rodriguez_vec);

}

void LS_SOLVER::rodriguezToRotation(const cv::Mat& rodriguez_vec, cv::Mat& rotation_mat){
    cv::Rodrigues(rodriguez_vec, rotation_mat);
}

void LS_SOLVER::exportToCSV(const cv::Mat& matrix,const std::string& filename) {
    std::ofstream outputFile(filename);
    if (!outputFile.is_open()) {
        std::cerr << "Error: Unable to open file " << filename << " for writing." << std::endl;
        return;
    }

    for (int i = 0; i < matrix.rows; ++i) {
        for (int j = 0; j < matrix.cols; ++j) {
            outputFile << matrix.at<double>(i, j);
            if (j < matrix.cols - 1) {
                outputFile << ","; // Separate values with comma
            }
        }
        outputFile << std::endl; // Move to the next line after each row
    }

    outputFile.close();
}

void LS_SOLVER::resetJacob(){
    jacob = cv::Mat::zeros(4*num_points, no_params, CV_64F);
}

void LS_SOLVER::computeJacob(){

    cv::Mat K_cp;
    cv::Mat hom_XYZ_cp;
    cv::Point2d err_1_mod;
    cv::Point2d err_2_mod;
    cv::Point2d deriv_1;
    cv::Point2d deriv_2;
    cv::Mat rot_2;
    cv::Mat rod_2;
    cv::Mat rod_2_cp;
    cv::Mat rot_2_cp;
    cv::Rect rot_roi(0, 0, 3, 3);
    transformation_matrix_2(rot_roi).copyTo(rot_2);
    rotationToRodriguez(rot_2, rod_2);
    cv::Mat transformation_matrix_1_cp;
    cv::Mat transformation_matrix_2_cp;
    transformation_matrix_1.copyTo(transformation_matrix_1_cp);
    transformation_matrix_2.copyTo(transformation_matrix_2_cp);
    cv::Mat rot_cp;
    cv::Mat jacob_rod;

    double delta = 0.0001;
    int jacop_split = 2 * num_points;

    for(int i=0; i<Corr.size(); i++){
        cv::Point2d err_1 = get_err(Corr[i].p1, transformation_matrix_1, K, hom_XYZ[i]);
        cv::Point2d err_2 = get_err(Corr[i].p2, transformation_matrix_2, K, hom_XYZ[i]);
        
        for(int k=0; k<3; k++){
            transformation_matrix_2.copyTo(transformation_matrix_2_cp);
            rod_2.copyTo(rod_2_cp);
            rod_2_cp.at<double>(k) += delta;
            cv::Rodrigues(rod_2_cp, rot_2_cp);
            rot_2_cp.copyTo(transformation_matrix_2_cp(rot_roi));
            err_2_mod = get_err(Corr[i].p2, transformation_matrix_2_cp, K, hom_XYZ[i]);
            deriv_2 = (err_2_mod - err_2) / delta;
            jacob.at<double>(2*i + jacop_split, ext_2_startidx + k) = deriv_2.x;
            jacob.at<double>(2*i + + jacop_split + 1, ext_2_startidx + k) = deriv_2.y;
        }

        for(int p=1; p<3; p++){
            transformation_matrix_2.copyTo(transformation_matrix_2_cp);
            transformation_matrix_2_cp.at<double>(p,3) += delta;
            err_2_mod = get_err(Corr[i].p2, transformation_matrix_2_cp, K, hom_XYZ[i]);
            deriv_2 = (err_2_mod - err_2) / delta;
            jacob.at<double>(2*i + jacop_split, ext_2_startidx  + 3 + p - 1) = deriv_2.x;
            jacob.at<double>(2*i + jacop_split + 1, ext_2_startidx + 3 + p - 1) = deriv_2.y;

        }

        for(int j=0; j<3; j++){
            hom_XYZ[i].copyTo(hom_XYZ_cp);
            hom_XYZ_cp.at<double>(j) += delta;
            err_1_mod = get_err(Corr[i].p1, transformation_matrix_1, K, hom_XYZ_cp);
            err_2_mod = get_err(Corr[i].p2, transformation_matrix_2, K, hom_XYZ_cp);
            deriv_1 = (err_1_mod - err_1) / delta;
            deriv_2 = (err_2_mod - err_2) / delta;

            
            jacob.at<double>(2*i, hom_pts_startidx + 3*i + j) = deriv_1.x;
            jacob.at<double>(2*i + 1, hom_pts_startidx + 3*i + j) = deriv_1.y;

            jacob.at<double>(2*i + jacop_split, hom_pts_startidx + 3*i + j) = deriv_2.x;
            jacob.at<double>(2*i + jacop_split + 1, hom_pts_startidx + 3*i + j) = deriv_2.y;
        }
    }

    cv::SVD svd(jacob);
    int rank = 0;
    for(int i=0; i<svd.w.rows; i++){
        if(svd.w.at<double>(i) > 1e-8){
            rank++;
        }
    }
    std::cout << "RANK OF JACOBIAN IS: " << rank << std::endl;
    std::string filename = "jacob.csv";
    exportToCSV(jacob, filename);
}

void LS_SOLVER::getStateVec(){

    cv::Rect rot_roi(0, 0, 3, 3);
    cv::Mat rot_2 = transformation_matrix_2(rot_roi);
    cv::Mat rod_2;
    cv::Rodrigues(rot_2, rod_2);

    for(int j=0; j<3; j++){
        x.at<double>(ext_2_startidx + j) = rod_2.at<double>(j);
    }

    for(int k=1; k<3; k++){
        x.at<double>(ext_2_startidx + 3 + k - 1) = transformation_matrix_2.at<double>(k, 3);
    }
    
    for(int u=0; u<hom_XYZ.size(); u++){
        x.at<double>(hom_pts_startidx + 3*u) = hom_XYZ[u].at<double>(0);
        x.at<double>(hom_pts_startidx + 3*u+1) = hom_XYZ[u].at<double>(1);
        x.at<double>(hom_pts_startidx + 3*u+2) = hom_XYZ[u].at<double>(2);
    }
}

void LS_SOLVER::stateVecToMat(){
    

    cv::Rect rot_roi(0, 0, 3, 3);
    cv::Mat rot_2;
    cv::Mat rod_2 = x.rowRange(0,3);

    cv::Rodrigues(rod_2, rot_2);

    rot_2.copyTo(transformation_matrix_2(rot_roi));


    for(int i=1; i<3; i++){
        transformation_matrix_2.at<double>(i,3) = x.at<double>(ext_2_startidx + 3 + i - 1);
    }

    for(int i=0; i<hom_XYZ.size(); i++){
        hom_XYZ[i].at<double>(0) = x.at<double>(hom_pts_startidx + 3*i);
        hom_XYZ[i].at<double>(1) = x.at<double>(hom_pts_startidx + 3*i + 1);
        hom_XYZ[i].at<double>(2) = x.at<double>(hom_pts_startidx + 3*i + 2);
    }

}

void LS_SOLVER::solveLS(){

    x = cv::Mat(no_params,1,CV_64F);
    cv::Mat Hess;
    cv::Mat residuals = cv::Mat::zeros(4*num_points, 1, CV_64F);
    double err;
    std::string x_file = "x.csv";

    for(int i=0; i<no_iter; i++){
        std::cout << " Iter: " << i << std::endl;
        err = total_error_func(residuals);
        resetJacob();
        computeJacob();
        std::cout << "Jacob rows " << jacob.rows << " Jacob cols " << jacob.cols << std::endl;
        Hess = jacob.t() * jacob;
        cv::solve(Hess, -jacob.t() * residuals, delta, cv::DECOMP_SVD);
        getStateVec();
        exportToCSV(x, x_file);
        x += delta;
        stateVecToMat();
        // std::cout << "INTRINSIC MATRIX K = " << K << std::endl;

    }
    cv::Mat Hess_inv;
    cv::invert(Hess, Hess_inv, cv::DECOMP_SVD);

    std::string delta_file = "delta.csv";
    std::string Hess_file = "Hess.csv";
    std::string Hess_inv_file = "Hess_inv.csv";
    exportToCSV(delta, delta_file);
    exportToCSV(Hess, Hess_file);
    exportToCSV(Hess_inv, Hess_inv_file);

    
}

LS_SOLVER::LS_SOLVER(std::vector<Correspondence> Corr, std::vector<cv::Mat> hom_XYZ, cv::Mat transformation_matrix_1, cv::Mat transformation_matrix_2, cv::Mat K):
    Corr(Corr), hom_XYZ(hom_XYZ), transformation_matrix_1(transformation_matrix_1), transformation_matrix_2(transformation_matrix_2), K(K){
        num_points = Corr.size();
        no_params = 5 + 3*num_points;
    } 