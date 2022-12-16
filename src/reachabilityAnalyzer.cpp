#include "reachabilityAnalyzer.hpp"

#define OPENGRIPPERDIS 0.055

#define CYLINDERGRIP 0.045



Reachability::Reachability()
{

};


Reachability::Reachability(rw::models::SerialDevice::Ptr robot_,
                        rw::models::WorkCell::Ptr wc_,
                        MovableFrame::Ptr obj_)
{

    bool failedInit = false;

    if(checkWorkCell(wc_))
    { 
        wc = wc_;
    }
    else
    {
        failedInit = true;
    }
    if(checkRobot(robot_))
    {
        robot = robot_;
    }
    else
    {
        failedInit = true;
    }
    if(checkObj(obj_))
    {
        obj = obj_;
    }    
    else
    {
        failedInit = true;
    }

    
    rw::models::Device::Ptr gripper_ = wc->findDevice("WSG50");
    
    if(checkGripper(gripper_))
    {
        gripper = gripper_;
    }    
    else
    {
        failedInit = true;
    }




    if(failedInit)
    {
        RW_THROW ("COULD NOT LOAD scene...");
    }


};



std::vector< Q > Reachability::getConfigurations (const std::string nameGoal, 
                                    const std::string nameTcp
                                    ){
    
    
    State state = wc->getDefaultState();
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

std::vector< Q > Reachability::reachabilityAnalysis(State& state){

    
    // create Collision Detector

    wc->setDefaultState(state);


    rw::proximity::CollisionDetector detector (
        wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy ());
    std::vector< Q > collisionFreeSolutions;
    
    for (double rollAngle = 0; rollAngle < 360.0;
         rollAngle += 1.0) {    // for every degree around the roll axis

        obj->setTransform (Transform3D<> (Vector3D<> (obj->getTransform (state).P ()),
                                              RPY<> (rollAngle * Deg2Rad, 0, 0)),
                               state);
       
        std::vector< Q > solutions =
            getConfigurations ("GraspTargetCylinder", "GraspTCP");
        for (unsigned int i = 0; i < solutions.size (); i++) {
            // set the robot in that configuration and check if it is in collision
            robot->setQ (solutions[i], state);
            //collisionFreeSolutions.push_back (solutions[i]);

            if (!detector.inCollision (state)) {
                collisionFreeSolutions.push_back (solutions[i]);    // save all reachable 
            }
             
        }
    }


    // sort for unique solutions
    //std::sort(collisionFreeSolutions.begin(), collisionFreeSolutions.end());
    //collisionFreeSolutions.erase(std::unique(collisionFreeSolutions.begin(), collisionFreeSolutions.end()), collisionFreeSolutions.end());
    return collisionFreeSolutions;
}

bool Reachability::checkWorkCell(rw::models::WorkCell::Ptr wc_)
{
    bool test = false;
    
    if (!wc_.isNull ()) 
    {
        test = true;
    }
    else
    {
        RW_THROW ("COULD NOT LOAD scene... check path!");
    }

    return test;
}

bool Reachability::checkRobot(rw::models::SerialDevice::Ptr robot_)
{
    bool test = false;
    
    if (!robot_.isNull ()) 
    {
        test = true;
    }
    else
    {
        RW_THROW ("COULD NOT LOAD scene... check path!");
    }

    return test;
}

bool Reachability::checkGripper(rw::models::Device::Ptr gripper_)
{
    bool test = false;
    
    if (!gripper_.isNull ()) 
    {
        test = true;
    }
    else
    {
        RW_THROW ("COULD NOT LOAD scene... check path!");
    }

    return test;
}

bool Reachability::checkObj(MovableFrame::Ptr obj_)
{
    bool test = false;
    
    if (!obj_.isNull ()) 
    {
        test = true;
    }
    else
    {
        RW_THROW ("COULD NOT LOAD scene... check path!");
    }

    return test;
}
