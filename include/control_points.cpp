#include "control_points.hpp"

void ControlPoints::writeToFile(const std::vector<MouseCallbackParams*>& control_points){
    std::ofstream input;
    input.open("control_points.txt");
    for(const auto& control_point : control_points){
        input << control_point->img_num << std::endl;
        for(const auto& point : control_point->dataPoints){
            input << point.pixelLocation << std::endl;
            input << point.number << std::endl;
        } 
        input << std::endl;

    }
    input.close();

}

void ControlPoints::readFromFile(std::vector<MouseCallbackParams*>& control_points){

    std::ifstream input;
    
    input.open("/Users/amtouti/Documents/SFM/2ViewBAGivenCorresspondence/3d_sample_data_set_sofa/gcp calc sofa.txt");
    
    cv::String line;
    cv::String line_copy;
    MouseCallbackParams* image_data = new MouseCallbackParams;
    DataPoint point;

    int len = 0;
    int start = 1;

    

    while(input.peek() != EOF){
         
        std::getline(input, line);
        if(len == 0){
            if(start == 0){
                control_points.push_back(image_data);
                image_data = new MouseCallbackParams;
            }
            image_data->img_num = line;
            start = 0;
            
        }
        else if(len > 1 && len < 70){
            
            point.number = std::stoi(line);
            image_data->dataPoints.push_back(point);
        }
        else{
            
            line_copy = line;
            line_copy.erase(std::remove(line_copy.begin(), line_copy.end(), '['), line_copy.end());
            line_copy.erase(std::remove(line_copy.begin(), line_copy.end(), ']'), line_copy.end());
            line_copy.erase(std::remove(line_copy.begin(), line_copy.end(), ','), line_copy.end());

            std::istringstream ss(line_copy);
            
            int x, y;
            ss >> x >> y;
            
            point.pixelLocation = cv::Point(x,y);
        }
        len = line.size();
    }
    control_points.push_back(image_data);

}

void ControlPoints::readXYZFromFile(std::vector<cv::Point3f>& XYZ_control_points){

    cv::String XYZ_file = "/Users/amtouti/Documents/SFM/2ViewBAGivenCorresspondence/3d_sample_data_set_sofa/gcp calc sofa.txt";
    std::ifstream input;
    input.open(XYZ_file);
    cv::String line;
    std::getline(input, line);
    for(int i = 0; i < 3; i++){
        std::getline(input, line);
        line.erase(0,1);
        // std::cout << line << std::endl;

        std::istringstream ss(line);
        float x, y, z;
        ss >> x >> y >> z;
        XYZ_control_points.push_back(cv::Point3f(x, y, z));
    }

}

void ControlPoints::onMouse(int event, int x, int y, int flags, void* userdata) {

    MouseCallbackParams* params = static_cast<MouseCallbackParams*>(userdata);
    cv::Mat image = params->image;
    // vector<DataPoint> dataPoints = params->dataPoints;

    if (event == cv::EVENT_LBUTTONDOWN) {
        int number;
        std::cout << "Enter a number (1, 2, or 3) for location (" << x << ", " << y << "): ";
        std::cin >> number;

        // Ensure the input is valid
        if (number >= 1 && number <= 3) {
            DataPoint data;
            data.pixelLocation = cv::Point(x, y);
            data.number = number;
            params->dataPoints.push_back(data);

            // Display the number on the image
            putText(image, std::to_string(number), cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
            imshow("Image", image);
        } else {
            std::cout << "Invalid input. Please enter a number between 1 and 3." << std::endl;
        }
    }
}

void ControlPoints::printControlPixels(const std::vector<MouseCallbackParams*>& control_points){
    std::cout << std::endl;
    for (const auto& params : control_points) {
        std::cout << params->img_num << std::endl;
        for(const auto& data: params->dataPoints)
        std::cout << "Location: (" << data.pixelLocation.x << ", " << data.pixelLocation.y << "), Number: " << data.number << std::endl;
    }
    std::cout << std::endl;
}

void ControlPoints::printControlXYZ(const std::vector<cv::Point3f>& XYZ_control_points){
    std::cout << "X Y Z values of control points 1 2 3 in World frame" << std::endl;
    for(const auto& point: XYZ_control_points){
        std::cout << point << std::endl;
    }
    std::cout << std::endl;
}

void ControlPoints::processControlPoints(const cv::String& folder, std::vector<MouseCallbackParams*>& control_points, 
                                        std::vector<cv::Point3f>& XYZ_control_points, int data_ready){
    
    std::vector<cv::String> filenames; 
    cv::glob(folder, filenames);
    

    readXYZFromFile(XYZ_control_points);
    

    

    if(!data_ready){ 
        
        for(const auto& file: filenames){
            cv::Mat image = cv::imread(file);

            

            cv::namedWindow("Image");
            cv::imshow("Image", image);

            

            std::vector<DataPoint> dataPoints;

            MouseCallbackParams* args = new MouseCallbackParams;
            args->image = image;
            args->dataPoints = dataPoints;
            args->img_num = file;
            
                
            
            cv::setMouseCallback("Image", onMouse, args);

            std::cout << "Click on locations and specify numbers (1, 2, or 3)." << std::endl;
            std::cout << "Press any key when done." << std::endl;

            cv::waitKey(0);

            std::cout << "Data points:" << std::endl;

            control_points.push_back(args);
        }

        

        for (const auto& params : control_points) {
            std::cout << params->img_num << std::endl;
            for(const auto& data: params->dataPoints){
                std::cout << "Location: (" << data.pixelLocation.x << ", " << data.pixelLocation.y << "), Number: " << data.number << std::endl;
            }

        writeToFile(control_points);
        }
    }

    else{
        
        readFromFile(control_points);
        
    }
}
