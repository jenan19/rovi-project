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
                    MovableFrame::Ptr obj_
                    );

        std::vector< Q > getConfigurations (
                                        const std::string nameGoal, 
                                        const std::string nameTcp
                                        );
        /*std::vector< Q > getConfigurations (const std::string nameGoal, const std::string nameTcp,
                                        rw::models::SerialDevice::Ptr robot,
                                        rw::models::WorkCell::Ptr wc, State& state);*/

        std::vector< Q > reachabilityAnalysis(State& state);
   
    private:
        bool checkWorkCell(rw::models::WorkCell::Ptr wc_);
        bool checkRobot(rw::models::SerialDevice::Ptr robot_);
        bool checkGripper(rw::models::Device::Ptr gripper_);
        bool checkObj(MovableFrame::Ptr obj_);
        



        rw::models::SerialDevice::Ptr robot;
        rw::models::WorkCell::Ptr wc;
        MovableFrame::Ptr obj;
        rw::models::Device::Ptr gripper;


};