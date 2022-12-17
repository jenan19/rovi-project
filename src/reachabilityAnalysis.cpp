#include <rw/invkin.hpp>
#include <rw/rw.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

#include <iostream>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include "reachabilityAnalyzer.hpp"



using namespace rw::kinematics;
using namespace rw::math;


/*

std::vector< Q > getConfigurations (const std::string nameGoal, const std::string nameTcp,
                                    rw::models::SerialDevice::Ptr robot,
                                    rw::models::WorkCell::Ptr wc, State& state)
{
    // Get, make and print name of frames
    const std::string robotName     = robot->getName ();
    const std::string nameRobotBase = robotName + "." + "Base";
    const std::string nameRobotTcp  = robotName + "." + "TCP";

    // Find frames and check for existence
    Frame::Ptr goal_f      = wc->findFrame (nameGoal);
    Frame::Ptr tcp_f       = wc->findFrame (nameTcp);
    Frame::Ptr robotBase_f = wc->findFrame (nameRobotBase);
    Frame::Ptr robotTcp_f  = wc->findFrame (nameRobotTcp);
    if (goal_f.isNull () || tcp_f.isNull () || robotBase_f.isNull () || robotTcp_f.isNull ()) {
        std::cout << " ALL FRAMES NOT FOUND:" << std::endl;
        std::cout << " Found \"" << nameGoal << "\": " << (goal_f.isNull () ? "NO!" : "YES!")
                  << std::endl;
        std::cout << " Found \"" << nameTcp << "\": " << (tcp_f.isNull () ? "NO!" : "YES!")
                  << std::endl;
        std::cout << " Found \"" << nameRobotBase
                  << "\": " << (robotBase_f.isNull () ? "NO!" : "YES!") << std::endl;
        std::cout << " Found \"" << nameRobotTcp
                  << "\": " << (robotTcp_f.isNull () ? "NO!" : "YES!") << std::endl;
    }

    // Make "helper" transformations
    Transform3D<> baseTGoal    = Kinematics::frameTframe (robotBase_f, goal_f, state);
    Transform3D<> tcpTRobotTcp = Kinematics::frameTframe (tcp_f, robotTcp_f, state);

    // get grasp frame in robot tool frame
    Transform3D<> targetAt = baseTGoal * tcpTRobotTcp;

    rw::invkin::ClosedFormIKSolverUR::Ptr closedFormSovler =
        rw::core::ownedPtr (new rw::invkin::ClosedFormIKSolverUR (robot, state));

    return closedFormSovler->solve (targetAt, state);
}





std::vector< Q > reachabilityAnalysis(rw::models::WorkCell::Ptr wc, rw::models::SerialDevice::Ptr robot, MovableFrame::Ptr obj)
{

    


    if (wc.isNull ()) {
        RW_THROW ("COULD NOT LOAD scene... check path!");
    }

    if (obj.isNull ()) {
        RW_THROW ("COULD not find movable frame Cylinder ... check model");
    }

    if (robot.isNull ()) {
        RW_THROW ("COULD not find device UR5 ... check model");
    }


    std::cout << "Looking for solutions.. \n";
    // create Collision Detector
    rw::proximity::CollisionDetector detector (
        wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy ());

    // get the default state
    State state = wc->getDefaultState ();
    std::vector< Q > collisionFreeSolutions;
    rw::models::Device::Ptr gripper = wc->findDevice("WSG50");
    if (gripper.isNull ()) {
        RW_THROW ("COULD not find device gripper ... check model");
    }
    

    for (double rollAngle = 0; rollAngle < 360.0;
         rollAngle += 1.0) {    // for every degree around the roll axis

        obj->setTransform (Transform3D<> (Vector3D<> (obj->getTransform (state).P ()),
                                              RPY<> (rollAngle * Deg2Rad, 0, 0)),
                               state);

        std::vector< Q > solutions =
            getConfigurations ("GraspTargetCylinder", "WSG50.TCP", robot, wc, state);
        for (unsigned int i = 0; i < solutions.size (); i++) {
            // set the robot in that configuration and check if it is in collision
            robot->setQ (solutions[i], state);
            //collisionFreeSolutions.push_back (solutions[i]);

            if (!detector.inCollision (state)) {
                collisionFreeSolutions.push_back (solutions[i]);    // save all reachable 
            }
             
        }
    }

    // visualize them
    rw::trajectory::TimedStatePath tStatePath;
    double time = 0;
    for (unsigned int i = 0; i < collisionFreeSolutions.size (); i++) {
        robot->setQ (collisionFreeSolutions[i], state);
        tStatePath.push_back (rw::trajectory::TimedState (time, state));
        time += 0.01;
    }

    rw::loaders::PathLoader::storeTimedStatePath (*wc, tStatePath, "../WorkCell/visu.rwplay");

    return collisionFreeSolutions;
}


void progressbar(float progress) // dies with OpenCV for some reason...
{

    int barWidth = 70;

    std::cout << "[";
    int pos = barWidth * progress;
    for (int i = 0; i < barWidth; ++i) {
        if (i < pos) std::cout << "=";
        else if (i == pos) std::cout << ">";
        else std::cout << " ";
    }
    std::cout << "] " << int(progress * 100.0) << " %\r";
    std::cout.flush();

}

*/



int main (int argc, char** argv)
{
    
    std::cout << "init... \n";
     // load workcell
    rw::models::WorkCell::Ptr wc =
    rw::loaders::WorkCellLoader::Factory::load ("../WorkCell/Scene.wc.xml");
    

    // find relevant frames
    MovableFrame::Ptr cylinderFrame = wc->findFrame< MovableFrame > ("Cylinder");
    

    rw::models::SerialDevice::Ptr robotUR5 = wc->findDevice< rw::models::SerialDevice > ("UR-6-85-5-A");
    


    MovableFrame::Ptr URReferenceFrame = wc->findFrame< MovableFrame > ("URReference");


    //rw::models::Device::Ptr gripper = wc->findDevice("WSG50");

    State state = wc->getDefaultState();


    //Q gripperQ(1);
    //gripperQ[0] = 0.055;
    //gripper->setQ(gripperQ, state);

    
    int resolution = 8;
    cv::Mat img (cv::Size(resolution,resolution),CV_8UC1);
    
    int tableWidth_mm = 800;
    int tableDebth_mm = 800;

    Reachability reachabilityTester(robotUR5, wc, cylinderFrame, URReferenceFrame);

    std::vector< std::vector <int> > results = reachabilityTester.analyzeWorkcSpace(tableWidth_mm, tableDebth_mm, resolution, 360);



    cv::namedWindow("heatmap",cv::WINDOW_NORMAL);
    cv::Mat colorImg;

    
    for(int i = 0; i < resolution; i++)
    {
        for(int j = 0; j < resolution; j++)
        {
            img.at<uchar>(cv::Point(j, i)) = results[i][j];//results[i][j];
            
        }
        std::cout << '\n';
    }
    
    cv::applyColorMap(img,colorImg,cv::COLORMAP_JET);
    cv::imshow("heatmap",colorImg);
    cv::imwrite("Hearmap.png",colorImg);
    cv::waitKey(0);



};