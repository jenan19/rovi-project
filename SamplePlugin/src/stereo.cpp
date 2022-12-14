#include "stereo.hpp"

Stereo::Stereo(){}
Stereo::~Stereo(){}


void Stereo::loadCamFromStream(std::istream & input, Camera &cam) {
    cv::Mat intrinsic = cv::Mat::zeros(3, 3, CV_64F);
    cv::Mat distortion = cv::Mat::zeros(4, 1, CV_64F);
    cv::Mat projection = cv::Mat::zeros(4, 4, CV_64F);
    cv::Mat transformation = cv::Mat::eye(4, 4, CV_64F);
    cv::Mat translation = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat rotation = cv::Mat::zeros(3, 3, CV_64F);
    double image_width, image_height;
    input.precision(20);
    input >> image_width >> image_height;

    input >> intrinsic.at<double>(0, 0) >> intrinsic.at<double>(0, 1)
            >> intrinsic.at<double>(0, 2);
    input >> intrinsic.at<double>(1, 0) >> intrinsic.at<double>(1, 1)
            >> intrinsic.at<double>(1, 2);
    input >> intrinsic.at<double>(2, 0) >> intrinsic.at<double>(2, 1)
            >> intrinsic.at<double>(2, 2);

    input >> distortion.at<double>(0, 0) >> distortion.at<double>(1, 0)
            >> distortion.at<double>(2, 0) >> distortion.at<double>(3, 0);

    input >> rotation.at<double>(0, 0) >> rotation.at<double>(0, 1)
            >> rotation.at<double>(0, 2);
    input >> rotation.at<double>(1, 0) >> rotation.at<double>(1, 1)
            >> rotation.at<double>(1, 2);
    input >> rotation.at<double>(2, 0) >> rotation.at<double>(2, 1)
            >> rotation.at<double>(2, 2);
    input >> translation.at<double>(0) >> translation.at<double>(1)
            >> translation.at<double>(2);

    cv::hconcat(rotation, translation, transformation);
    cv::Mat row = cv::Mat::zeros(1, 4, CV_64F);
    row.at<double>(0, 3) = 1;
    transformation.push_back(row);

    cv::Mat tmp = intrinsic;
    cv::Mat tmp1 = cv::Mat::zeros(3, 1, CV_64F);
    cv::hconcat(tmp, tmp1, tmp);
    projection = tmp * transformation;

    cam.distortion = distortion;
    cam.intrinsic = intrinsic;
    cam.projection = projection;
    cam.transformation = transformation;
    cam.image_height = image_height;
    cam.image_width = image_width;
    cam.translation = translation;
    cam.rotation = rotation;
}

bool Stereo::readStereoCameraFile(const std::string & fileNameP,
        StereoPair &stereoPair) {
    int number_of_cameras;
    Camera cam1, cam2;
    ifstream ifs(fileNameP.c_str());
    if (ifs) {
        ifs >> number_of_cameras;
        if (number_of_cameras == 2) {
            loadCamFromStream(ifs, cam1);
            loadCamFromStream(ifs, cam2);
            stereoPair.cam1 = cam1;
            stereoPair.cam2 = cam2;
            return true;
        }
    }
    return false;
}


int Stereo::loadImagesAndCalibration(string calibrationFile, cv::Mat &img_l, cv::Mat &img_r, StereoPair &stereoPair)
{
    if (img_l.empty() || img_r.empty()) {
        std::cout << "Error loading the images" << std::endl;
        return -1;
    }

    std::ifstream ifs(calibrationFile.c_str());
    if (ifs) {
        //Load calibration file
        readStereoCameraFile(calibrationFile, stereoPair);
    } else {
        std::cout << "Error opening calibration file. Calibration file must be in old OpenCV format. Calibration file must be in old OpenCV format.." << std::endl;
        return -1;
    }

    return 0;

}

cv::Mat Stereo::constructProjectionMat(Camera cam)
{
    cv::Mat KA = cam.intrinsic;
    cv::Mat H = cam.transformation;

    // Remember to add a row of zeros so the KA matrix becomes 3x4
    cv::Mat zeros = cv::Mat::zeros(3, 1, CV_64F);
    cv::hconcat(KA, zeros, KA);
    return KA * H;
}






Mat Stereo::everythingVision(Mat pictureLeft, Mat pictureRight, string calibrationFile)
{


    StereoPair stereoPair;

    Mat img_l = pictureLeft, img_r = pictureRight;


    if(loadImagesAndCalibration(calibrationFile, img_l, img_r, stereoPair)){
    std::cout << "Input error" << std::endl;

    }

    Mat houghR = img_r.clone();
    Mat houghL = img_l.clone();
    Mat hsv_thresholded_imageL, hsvL, hsv_thresholded_imageR, hsvR;

    //Box and cylinder HSV values
    int lowH, highH, lowS, highS, lowV, highV;
    lowH = 0; highH = 200; lowS = 0; highS = 200; lowV = 30; highV = 200;

    //Left
    cvtColor(houghL, hsvL, COLOR_BGR2HSV);
    // Detect the object based on HSV Range Values
    inRange(hsvL, Scalar(lowH, lowS, lowV), Scalar(highH, highS, highV), hsv_thresholded_imageL);

    imshow("test", hsv_thresholded_imageL);
    //waitKey(0);

    //Right
    cvtColor(houghR, hsvR, COLOR_BGR2HSV);
    // Detect the object based on HSV Range Values
    inRange(hsvR, Scalar(lowH, lowS, lowV), Scalar(highH, highS, highV), hsv_thresholded_imageR);

    //bottlecap
    //int lowH, highH, lowS, highS, lowV, highV;
    lowH = 0; highH = 180; lowS = 64; highS = 255; lowV = 224; highV = 237;

    medianBlur(hsv_thresholded_imageL, hsv_thresholded_imageL, 5);
    medianBlur(hsv_thresholded_imageR, hsv_thresholded_imageR, 5);

    //Left camera
    vector<Vec3f> circlesL;
    HoughCircles(hsv_thresholded_imageL, circlesL, HOUGH_GRADIENT, 1,
                 hsv_thresholded_imageL.rows/32,  
                 50, 13, 2, 30 

    );

    
    Point centerL;
    for( size_t i = 0; i < circlesL.size(); i++ )
    {
        Vec3i c = circlesL[i];
        if (c[1] > 200 && c[1] < 304)
        {
        centerL = Point(c[0], c[1]);
        //cout << "centerL : " << centerL.x << " , " << centerL.y << endl; 
        // circle center

            circle( houghL, centerL, 1, Scalar(0,100,100), 3, LINE_AA);
            // circle outline
            int radius = c[2];
            circle( houghL, centerL, radius, Scalar(255,0,255), 3, LINE_AA);
        }

    }
    //imshow("left", houghL);

    //waitKey(1);

    //Right camera
    vector<Vec3f> circlesR;
    HoughCircles(hsv_thresholded_imageR, circlesR, HOUGH_GRADIENT, 1,
                 hsv_thresholded_imageR.rows/32,  
                 50, 13, 2, 30 
    );

    
    Point centerR;
    for( size_t i = 0; i < circlesR.size(); i++ )
    {
        Vec3i c = circlesR[i];
        if (c[1] > 200 && c[1] < 304)
        {
            centerR = Point(c[0], c[1]);
            circle( houghR, centerR, 1, Scalar(0,100,100), 3, LINE_AA);
            // circle outline
            int radius = c[2];
            circle( houghR, centerR, radius, Scalar(255,0,255), 3, LINE_AA);
        }

    }
    //imshow("Right", houghR);

    


    auto proj_l = constructProjectionMat(stereoPair.cam1);
    auto proj_r = constructProjectionMat(stereoPair.cam2);


    cv::Mat m_l(3, 1, CV_64F);
    m_l.at<double>(0, 0) = centerL.x;
    m_l.at<double>(1, 0) = centerL.y;
    m_l.at<double>(2, 0) = 1;


    cv::Mat m_r(3, 1, CV_64F);
    m_r.at<double>(0, 0) = centerR.x;
    m_r.at<double>(1, 0) = centerR.y;
    m_r.at<double>(2, 0) = 1;


    // Compare with OpenCV triangulation
    cv::Mat pnts3D(1, 1, CV_64FC4);
    cv::Mat cam0pnts(1, 1, CV_64FC2);
    cv::Mat cam1pnts(1, 1, CV_64FC2);
    cam0pnts.at<cv::Vec2d>(0)[0] = m_l.at<double>(0, 0);
    cam0pnts.at<cv::Vec2d>(0)[1] = m_l.at<double>(1, 0);
    cam1pnts.at<cv::Vec2d>(0)[0] = m_r.at<double>(0, 0);
    cam1pnts.at<cv::Vec2d>(0)[1] = m_r.at<double>(1, 0);
    triangulatePoints(proj_l, proj_r, cam0pnts, cam1pnts, pnts3D);
    std::cout << "Image points: " << cam0pnts << "\t" << cam1pnts << std::endl << std::endl;
    std::cout << "Triangulated point (normalized): " << std::endl << pnts3D / pnts3D.at<double>(3, 0) << std::endl << std::endl;
    //waitKey(1);
    return (pnts3D / pnts3D.at<double>(3, 0)); 


}


/*
int main(int argc, const char** argv) 
{


    Stereo stereo;
    Mat img_l = imread("../pictures/-0204Camera_Left.png");
    Mat img_r = imread("../pictures/-0204Camera_Right.png");
    string calibrationFile = "../calibration.txt";


    Mat imgL = img_l.clone(), imgR = img_r.clone();

    imgL = img_l.clone(), imgR = img_r.clone();
    vector<double> sigma = {50, 50, 50};

    vector<double>  mean = {0, 0, 0};
    Mat noisel(imgL.size(),imgL.type());

    randn(noisel, mean, sigma);
    imgL += noisel;

    imshow("test",imgL);
    waitKey(0);

    Mat noiser(imgR.size(),imgR.type());

    randn(noiser, mean, sigma);
    imgR += noiser;


    Mat pnts3D(1, 1, CV_64FC4);
    pnts3D = stereo.everythingVision(imgL, imgR, calibrationFile);


    cout << pnts3D.at<double>(0) << " " << pnts3D.at<double>(1) << "  " << pnts3D.at<double>(2) << endl;

    

    return 0;
}
*/