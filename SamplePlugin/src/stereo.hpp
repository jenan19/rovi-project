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
using namespace std;

struct Camera {
    cv::Mat intrinsic;
    cv::Mat transformation;
    cv::Mat distortion;
    cv::Mat projection;
    cv::Mat translation;
    cv::Mat rotation;
    double image_width;
    double image_height;

};


struct StereoPair {
    Camera cam1;
    Camera cam2;
};

class Stereo
{
    private:

    //Varialbles:

        double tmp_x=-1;
        double tmp_y=-1;



    //Functions

        void loadCamFromStream(std::istream & input, Camera &cam);

        bool readStereoCameraFile(const std::string & fileNameP, StereoPair &stereoPair);

        int loadImagesAndCalibration(string calibrationFile, cv::Mat &img_l, cv::Mat &img_r, StereoPair &stereoPair);

        cv::Mat constructProjectionMat(Camera cam);



    public:
        Stereo();
        ~Stereo();

        Mat everythingVision(Mat pictureLeft, Mat pictureRight, string calibrationFile);

};