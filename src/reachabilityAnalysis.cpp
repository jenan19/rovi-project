
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

    rw::models::Device::Ptr gripper = wc->findDevice("WSG50");

    State state = wc->getDefaultState();

    Q gripperQ(1);
    gripperQ[0] = 0.055;
    gripper->setQ(gripperQ, state);

    
    int resX = 4;
    int resY = 4;

    
    int tableWidth_mm = 800;
    int tableDebth_mm = 800;


    int stepSizeX = tableWidth_mm / resX;
    int stepSizeY = tableDebth_mm / resY;

    cv::Mat img (cv::Size(resX,resY),CV_8UC1);

    rw::trajectory::TimedStatePath tStatePath;
    
    float score = 0;
    float maxScore = -1;
    float reachAnlalysis [resX][resY];


    float progress = 0;
    double time = 0;
    Reachability reacher = Reachability(robotUR5,wc,cylinderFrame);

    float percentageChange = 1/(tableDebth_mm * tableWidth_mm);

    for(int i = 0; i < resY + 1; i++)
    {
        float y = (i * stepSizeY - tableDebth_mm / 2.0f ) / 1000.0f; // calculate y and convert to meters
        for(int j = 0; j < resX + 1; j++)
        {
            float x = (j * stepSizeX - tableWidth_mm / 2.0f) / 1000.0f; // calculate x and convert to meters
            std::cout << x << "   " << y << '\n';
            //progressbar(progress);
            //state = wc->getDefaultState();

            Vector3D<> pos(x, y, 0.11);
            
            Transform3D<> transform(pos);

            URReferenceFrame->setTransform(transform, state);

            std::vector< Q > collisionFreeSolutions = reacher.reachabilityAnalysis(state);

            score = collisionFreeSolutions.size();

            //std::vector< Q > collisionFreeSolutions = reachabilityAnalysis(wc,robotUR5,cylinderFrame);


            if(score > maxScore)
            {
                maxScore = score;
            }
            reachAnlalysis[i][j] = score;
            
            progress += percentageChange;
            for (unsigned int i = 0; i < collisionFreeSolutions.size (); i++) 
            {
                robotUR5->setQ (collisionFreeSolutions[i], state);
                tStatePath.push_back (rw::trajectory::TimedState (time, state));
                time += 0.01;
            }

        }
    }




      for(int i = 0; i < resY; i++)
    {
        for(int j = 0; j < resX; j++)
        {
            int col = ((float)(reachAnlalysis[i][j] / maxScore) * 255);

            std::cout << "(" << reachAnlalysis[i][j] << ", " << col << ")  ";
            img.at<uchar>(cv::Point(j,i)) = col;

        }
        std::cout << '\n';
    }

    if(tStatePath.size())
        rw::loaders::PathLoader::storeTimedStatePath (*wc, tStatePath, "../WorkCell/visu.rwplay");
    else
        std::cout << "no solutions";

    std::cout << "\n DONE \n";

    
    cv::namedWindow("heatmap",cv::WINDOW_NORMAL);
    cv::Mat colorImg;
    
    cv::applyColorMap(img,colorImg,cv::COLORMAP_JET);
    cv::imshow("heatmap",colorImg);
    cv::waitKey(0);

















/*

    Reachability reacher = Reachability(robotUR5,wc,cylinderFrame);
    
    
    
    std::cout << "testing... \n";

    std::vector< Q > collisionFreeSolutions = reacher.reachabilityAnalysis();

    std::cout << "Current position of the robot vs object to be grasped has: "
              << collisionFreeSolutions.size () << " collision-free inverse kinematics solutions!"
              << std::endl;*/

    



    /*
    wc = rw::loaders::WorkCellLoader::Factory::load ("../WorkCell/Scene.wc.xml");

    State state = wc->getDefaultState();
    // visualize them
    rw::trajectory::TimedStatePath tStatePath;
    double time = 0;
    for (unsigned int i = 0; i < collisionFreeSolutions.size (); i++) {
        robotUR5->setQ (collisionFreeSolutions[i], state);
        tStatePath.push_back (rw::trajectory::TimedState (time, state));
        time += 0.01;
    }

    if(tStatePath.size())
        rw::loaders::PathLoader::storeTimedStatePath (*wc, tStatePath, "../WorkCell/visu.rwplay");
    



    collisionFreeSolutions = reachabilityAnalysis(wc,robotUR5,cylinderFrame);



    std::cout << "Current position of the robot vs object to be grasped has: "
              << collisionFreeSolutions.size () << " collision-free inverse kinematics solutions!"
              << std::endl;


    state = wc->getDefaultState();
    // visualize them
    tStatePath.clear();
    time = 0;
    for (unsigned int i = 0; i < collisionFreeSolutions.size (); i++) {
        robotUR5->setQ (collisionFreeSolutions[i], state);
        tStatePath.push_back (rw::trajectory::TimedState (time, state));
        time += 0.01;
    }

    if(tStatePath.size())
        rw::loaders::PathLoader::storeTimedStatePath (*wc, tStatePath, "../WorkCell/visuOld.rwplay");


*/




/*
    // load workcell
    rw::models::WorkCell::Ptr wc =
        rw::loaders::WorkCellLoader::Factory::load ("../WorkCell/Scene.wc.xml");
    if (wc.isNull ()) {
        RW_THROW ("COULD NOT LOAD scene... check path!");
    }

    // find relevant frames
    MovableFrame::Ptr cylinderFrame = wc->findFrame< MovableFrame > ("Cylinder");
    if (cylinderFrame.isNull ()) {
        RW_THROW ("COULD not find movable frame Cylinder ... check model");
    }

    rw::models::SerialDevice::Ptr robotUR5 = wc->findDevice< rw::models::SerialDevice > ("UR-6-85-5-A");
    if (robotUR5.isNull ()) {
        RW_THROW ("COULD not find device UR5 ... check model");
    }

    rw::models::Device::Ptr gripper = wc->findDevice("WSG50");
    if (gripper.isNull ()) {
        RW_THROW ("COULD not find device gripper ... check model");
    }



    State state = wc->getDefaultState ();
    Q defaultQ = gripper->getQ(state);
    Q openQ = defaultQ;
    openQ[0] = 5.5;
    gripper->setQ(openQ, state);
    std::cout << openQ << '\n';


std::vector< Q > collisionFreeSolutions = reachabilityAnalysis(wc,robotUR5,cylinderFrame);



std::cout << "Current position of the robot vs object to be grasped has: "
              << collisionFreeSolutions.size () << " collision-free inverse kinematics solutions!"
              << std::endl;*/
    

    /*
    // create Collision Detector
    rw::proximity::CollisionDetector detector (
        wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy ());

    
    
    // get the default state
    State state = wc->getDefaultState ();
    std::vector< Q > collisionFreeSolutions;

    for (double rollAngle = 0; rollAngle < 360.0;
         rollAngle += 1.0) {    // for every degree around the roll axis

        cylinderFrame->setTransform (Transform3D<> (Vector3D<> (cylinderFrame->getTransform (state).P ()),
                                              RPY<> (rollAngle * Deg2Rad, 0, 0)),
                               state);

        std::vector< Q > solutions =
            getConfigurations ("GraspTarget", "GraspTCP", robotUR5, wc, state);

        for (unsigned int i = 0; i < solutions.size (); i++) {
            // set the robot in that configuration and check if it is in collision
            robotUR5->setQ (solutions[i], state);

            if (!detector.inCollision (state)) {
                collisionFreeSolutions.push_back (solutions[i]);    // save it
                break;                                              // we only need one
            }
        }
    }

    std::cout << "Current position of the robot vs object to be grasped has: "
              << collisionFreeSolutions.size () << " collision-free inverse kinematics solutions!"
              << std::endl;

    // visualize them
    rw::trajectory::TimedStatePath tStatePath;
    double time = 0;
    for (unsigned int i = 0; i < collisionFreeSolutions.size (); i++) {
        robotUR5->setQ (collisionFreeSolutions[i], state);
        tStatePath.push_back (rw::trajectory::TimedState (time, state));
        time += 0.01;
    }

    rw::loaders::PathLoader::storeTimedStatePath (*wc, tStatePath, "../scene/visu.rwplay");*/

    return 0;
}
