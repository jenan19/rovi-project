#include "reachabilityAnalyzer.hpp"

#define OPENGRIPPERDIS 0.055

#define CYLINDERGRIP 0.045



Reachability::Reachability()
{

};


Reachability::Reachability(rw::models::SerialDevice::Ptr robot_,
                        rw::models::WorkCell::Ptr wc_,
                        MovableFrame::Ptr obj_,
                        MovableFrame::Ptr robotRef_)
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
    if(checkRobotRef(robotRef_))
    {
        robotRef = robotRef_;
    }    
    else
    {
        failedInit = true;
    }




    if(failedInit)
    {
        RW_THROW ("COULD NOT LOAD scene...");
    }

    state.upgradeTo(wc->getDefaultState());


};



std::vector< Q > Reachability::getConfigurations (const std::string nameGoal, 
                                    const std::string nameTcp
                                    ){
    
    
    //State state = wc->getDefaultState();
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

std::vector< Q > Reachability::reachabilityAnalysis(int maxRotAng, int approachRPY[3]){

    
    // create Collision Detector


    MovableFrame::Ptr test =  wc->findFrame< MovableFrame > ("GraspTargetCylinder");
    rw::proximity::CollisionDetector detector (
        wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy ());
    std::vector< Q > collisionFreeSolutions;
    
    for (double rollAngle = 0; rollAngle < maxRotAng;
         rollAngle += 1.0) {    // for every degree around the roll axis

        
        test->setTransform(Transform3D<>(Transform3D<>(
                                                    Vector3D<>(0, 0, 0),  
                                                    RPY<> ((rollAngle + approachRPY[0]) * Deg2Rad, 
                                                            approachRPY[1] * Deg2Rad, 
                                                            approachRPY[2] * Deg2Rad)
                                                        )
                                        ), 
                            state);
                                             
        std::cout << rollAngle << "\r";
        std::cout.flush();


        // can it reach the goal at all?
        bool reachableGoal = false;
        std::vector< Q > goalPoses =
            getConfigurations ("PlacePoint", "GraspTCP");
        for (unsigned int i = 0; i < goalPoses.size (); i++) {
            // set the robot in that configuration and check if it is in collision
            robot->setQ (goalPoses[i], state);
            //collisionFreeSolutions.push_back (solutions[i]);

            if (!detector.inCollision (state)) {
                reachableGoal = true;    // only need to reach the goal at one angle
                break;
            }
             
        }

        if(reachableGoal)
        {
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
    }
    std::cout <<'\n';


    // sort for unique solutions
    //std::sort(collisionFreeSolutions.begin(), collisionFreeSolutions.end());
    //collisionFreeSolutions.erase(std::unique(collisionFreeSolutions.begin(), collisionFreeSolutions.end()), collisionFreeSolutions.end());
    return collisionFreeSolutions;
}


std::vector<std::vector<int>> Reachability::analyzeWorkcSpace(int dimX, int dimY, int resolution, int maxRotAng)
{
    
    rw::trajectory::TimedStatePath tStatePath;

    int resX = dimX / resolution;
    int resY = dimY / resolution;
    
    double score = 0;
    double maxScore = -1;
    std::vector<std::vector<int>> results;

    //MovableFrame::Ptr test =  wc->findFrame< MovableFrame > ("URReference");
    MovableFrame::Ptr refValueTest = robotRef; // DO NOT DELETE!!! 

    double time = 0;


    for(int i = 0; i < resolution; i++)
    {

        std::vector <int> temp;


        float y = (i * resY - dimY / 2.0f ) / 1000.0f; // calculate y and convert to meters

        for(int j = 0; j < resolution; j++)
        {
            
            
            
            float x = (j * resX - dimX/ 2.0f) / 1000.0f; // calculate x and convert to meters
            std::cout << x << "   " << y << '\n';
            //progressbar(progress);
            //state = wc->getDefaultState();
            //test->moveTo(Transform3D<>(Vector3D<>(x, y, 0), RPY<>(0, 0, 0)), state);

            // move robot reference fram
            robotRef->moveTo(Transform3D<>(Vector3D<>(x, y, 0), RPY<>(0, 0, 0)), state);

            // get collision free configurations
            
            int approachTop[3] = {0, -180, 0};
            std::vector< Q > collisionFreeSolutionsTop = reachabilityAnalysis(maxRotAng, approachTop);
            savePath(&tStatePath, &collisionFreeSolutionsTop, &time);


            int approachSide[3] = {0, -180, 90};
            std::vector< Q > collisionFreeSolutionsSide = reachabilityAnalysis(maxRotAng, approachSide);
            savePath(&tStatePath, &collisionFreeSolutionsSide, &time);

            // set score to number of solutions
            score = collisionFreeSolutionsTop.size() + collisionFreeSolutionsSide.size();

            if(score > maxScore)
            {
                maxScore = score;
            }
            temp.push_back(score);
            

        }

        
        results.push_back(temp);

    }




      for(int i = 0; i < resolution; i++)
    {
        for(int j = 0; j < resolution; j++)
        {
            int color = results.at(i).at(j) / maxScore * 255;

            std::cout << "(" << results[i][j] << ", " << color << ")  ";
            results[i][j] = color;

        }
        std::cout << '\n';
    }

    if(tStatePath.size())
        rw::loaders::PathLoader::storeTimedStatePath (*wc, tStatePath, "../WorkCell/visu.rwplay");
    else
        std::cout << "no solutions";

    std::cout << "\n DONE \n";
    
    
    return results;
}


void Reachability::savePath(rw::trajectory::TimedStatePath *path, std::vector< Q > *collisionFreeSolutions, double *time)
{
    for (unsigned int i = 0; i < collisionFreeSolutions->size (); i++) 
            {
                robot->setQ (collisionFreeSolutions->at(i), state);
                path->push_back(rw::trajectory::TimedState (*time, state));
                *time += 0.01;
            }
};






bool Reachability::checkRobotRef(MovableFrame::Ptr robotRef_)
{
    bool test = false;
    
    if (!robotRef_.isNull ()) 
    {
        test = true;
    }
    else
    {
        RW_THROW ("COULD NOT LOAD scene... check path!");
    }

    return test;
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