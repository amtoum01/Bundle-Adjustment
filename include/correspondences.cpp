#include "correspondences.hpp"

int count = 0;
int col_size;
bool mouse_clicked = false;

void GetCorrespondence::onMouse(int event, int x, int y, int flag, void* userdata){ 
    if(event == cv::EVENT_LBUTTONDOWN){
        Correspondence* Corr = static_cast<Correspondence*>(userdata);
        if(count == 0){
            std::cout << "PIXEL LOCATION P1 IS: " << x << ", " << y << std::endl;
            Corr->p1 = cv::Point2d(x, y);
        }
        else{
            std::cout << "PIXEL LOCATION P2 IS: " << (x - col_size) << ", " << y << std::endl;
            Corr->p2 = cv::Point2d(x - col_size, y);
            mouse_clicked = true;
            std::cout << "PREESS ANY KEY EXCEPT ESC TO INPUT NEXT SET OF MATCHES" << std::endl;
        }
        count = (count + 1) % 2;

    }
}

void GetCorrespondence::getCorrespondence(std::vector<Correspondence>& Corr){

    cv::namedWindow("Image Correspondences", cv::WINDOW_NORMAL);
    Correspondence* Corr_ptr = new Correspondence;
    cv::Mat display;
    cv::hconcat(img1, img2, display);
    
    int corr = 4;
    col_size = img1.size[1];
    int corr_counter = 0;
    int val = 0;
    cv::setMouseCallback("Image Correspondences", onMouse, Corr_ptr);

    while(val!=27){
        
        cv::imshow("Image Correspondences", display);
        std::cout << "Corespondence Number " << corr_counter << std::endl;
        
        if (mouse_clicked) {
            Corr.push_back(*Corr_ptr);
            std::cout << "P1 is: " << Corr.back().p1 << " P2 is : " << Corr.back().p2 << std::endl; 
            mouse_clicked = false;

            for(const auto& corr: Corr){
                cv::Point2i p2_disp(corr.p2.x + col_size, corr.p2.y);
                cv::line(display, corr.p1, p2_disp, cv::Scalar(0, 255, 0), 2);
            }
        }
        cv::imshow("Image Correspondences", display);
        val = cv::waitKey(0);
        corr_counter++;

    }
}

void GetCorrespondence::writeCorrToFile(const std::vector<Correspondence>& Corr){
    std::ofstream input;
    // input.open("correspondesnces.txt");
    input.open("correspondesnces_bat.txt");
    for(const auto& corr : Corr){
        input << corr.p1.x << " " << corr.p1.y << " " << corr.p2.x << " " << corr.p2.y <<std::endl;
    }
    input.close();

}

void GetCorrespondence::readCorrFromFile(std::vector<Correspondence>& Corr){
    std::ifstream input;
    Corr.clear();
    // input.open("/Users/amtouti/Documents/SFM/2ViewBAGivenCorresspondence/build/correspondesnces.txt");
    input.open("/Users/amtouti/Documents/SFM/2ViewBAGivenCorresspondence/build/correspondesnces_bat.txt");
    cv::String line;
    while(std::getline(input, line) && !line.empty()){
        std::istringstream ss(line);
        int p1_x, p1_y, p2_x, p2_y;
        ss >> p1_x >> p1_y >> p2_x >> p2_y;

        cv::Point2i p1(p1_x, p1_y);
        cv::Point2i p2(p2_x, p2_y);
        Correspondence corr{p1, p2};
        Corr.push_back(corr);
    }
}

GetCorrespondence::GetCorrespondence(const cv::Mat& image1,const cv::Mat& image2)
{
    img1 = image1;
    img2 = image2;

}
