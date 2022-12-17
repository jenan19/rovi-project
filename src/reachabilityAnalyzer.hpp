#include <rw/invkin.hpp>
#include <rw/rw.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

#include <iostream>
#include <string>
#include <vector>

using namespace rw::kinematics;
using namespace rw::math;

class Reachability {
    
    
    public:
        Reachability();

        Reachability(
                    rw::models::SerialDevice::Ptr robot_,
                    rw::models::WorkCell::Ptr wc_,
                    MovableFrame::Ptr obj_,
                    MovableFrame::Ptr robotRef_
                    );

        std::vector< Q > getConfigurations (
                                        const std::string nameGoal, 
                                        const std::string nameTcp
                                        );
        /*std::vector< Q > getConfigurations (const std::string nameGoal, const std::string nameTcp,
                                        rw::models::SerialDevice::Ptr robot,
                                        rw::models::WorkCell::Ptr wc, State& state);*/

        std::vector< Q > reachabilityAnalysis(int maxRotAng, int approachRPY[3]);

        std::vector < std::vector < int > > analyzeWorkcSpace(int dimX, int dimY, int resolution, int maxRotAng);

        void savePath(rw::trajectory::TimedStatePath *path, std::vector< Q > *collisionFreeSolutions, double *time);
   
    private:
        bool checkWorkCell(rw::models::WorkCell::Ptr wc_);
        bool checkRobot(rw::models::SerialDevice::Ptr robot_);
        bool checkGripper(rw::models::Device::Ptr gripper_);
        bool checkObj(MovableFrame::Ptr obj_);
        bool checkRobotRef(MovableFrame::Ptr robotRef_);




        rw::models::SerialDevice::Ptr robot;
        rw::models::WorkCell::Ptr wc;
        State state;
        MovableFrame::Ptr obj;
        MovableFrame::Ptr robotRef;


};