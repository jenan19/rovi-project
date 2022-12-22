#include <iostream>
#include <fstream>

#include <eigen3/Eigen/Eigen>

// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/rgbd/linemod.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>


using namespace cv;
const int max_value_H = 360/2;
const int max_value = 255;

int low_H = 0, low_S = 0, low_V = 30;
int high_H = 200, high_S = 0, high_V = 200;



int main(int argc, char* argv[])
{
    Mat img = imread("../pictures/020474Camera_Right.png");
    imshow("image", img);
    imshow("Trackbars", 0);
    // Trackbars to set thresholds for HSV values
    createTrackbar("Low H", "Trackbars", nullptr, 200, 0);
    setTrackbarPos("Low H", "Trackbars", low_H);       

    createTrackbar("High H","Trackbars", nullptr, 200, 0);
    setTrackbarPos("High H", "Trackbars", high_H);  

    createTrackbar("Low S", "Trackbars", nullptr, 200, 0);
    setTrackbarPos("Low H", "Trackbars", low_S);  

    createTrackbar("High S", "Trackbars", nullptr, 200, 0);
    setTrackbarPos("High S", "Trackbars", high_S); 

    createTrackbar("Low V", "Trackbars", nullptr, 200, 0);
    setTrackbarPos("Low V", "Trackbars", low_V); 

    createTrackbar("High V", "Trackbars", nullptr, 200, 0);
    setTrackbarPos("High V", "Trackbars", high_V); 

    Mat frame, frame_HSV, frame_threshold;
    while (true) {
        frame = img.clone();
        if(frame.empty())
        {
            break;
        }
        low_H = cv::getTrackbarPos("Low H", "Trackbars");
        high_H = cv::getTrackbarPos("High H", "Trackbars");
        low_S = cv::getTrackbarPos("Low S", "Trackbars");
        high_S = cv::getTrackbarPos("High S", "Trackbars");
        low_V = cv::getTrackbarPos("Low V", "Trackbars");
        high_V = cv::getTrackbarPos("High V", "Trackbars");
        // Convert from BGR to HSV colorspace
        cvtColor(frame, frame_HSV, COLOR_BGR2HSV);
        // Detect the object based on HSV Range Values
        inRange(frame_HSV, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), frame_threshold);
        // Show the frames
        imshow("image", frame_threshold);
        imshow("Trackbars", frame_threshold);
        char key = (char) waitKey(30);
        if (key == 'q' || key == 27)
        {
            break;
        }
    }
    return 0;
}


