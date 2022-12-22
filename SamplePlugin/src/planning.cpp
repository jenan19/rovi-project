
#include <functional>
#include <filesystem>
#include <vector>
#include <iostream>
#include <fstream>
#include <cstdio>
#include <cstdlib>

// RobWork includes
#include <rw/kinematics/State.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/rw.hpp>
#include <rw/trajectory/LinearInterpolator.hpp>
#include <rwlibs/opengl/RenderImage.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rwlibs/simulation/GLFrameGrabber.hpp>
#include <rwlibs/simulation/GLFrameGrabber25D.hpp>
#include <rwlibs/simulation/SimulatedCamera.hpp>
#include <rw/invkin/ClosedFormIKSolverUR.hpp>
#include <rwslibs/rwstudioapp/RobWorkStudioApp.hpp>
#include <rw/core/macros.hpp>


// RobWorkStudio includes
#include <RobWorkStudioConfig.hpp>    // For RWS_USE_QT5 definition
#include <rws/RobWorkStudioPlugin.hpp>

// OpenCV 3
#include <opencv2/opencv.hpp>

// Qt
#include <rw/loaders/ImageLoader.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rws/RobWorkStudio.hpp>

#include <QPushButton>
#include <QTimer>

#include "stereo.cpp"



//Namespaces
using namespace rw::common;
using namespace rw::graphics;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::sensor;
using namespace rw::invkin;

using namespace rwlibs::opengl;
using namespace rwlibs::simulation;

using namespace std;
using namespace rw::math;
using namespace rw::pathplanning;
using namespace rw::proximity;
using namespace rw::trajectory;
using namespace rwlibs::pathplanners;
using namespace rwlibs::proximitystrategies;

using namespace rws;

using namespace cv;

using namespace std::placeholders;

using rw::graphics::SceneViewer;
using rw::loaders::WorkCellLoader;
using rw::models::WorkCell;
using rw::sensor::Image;

//Defines
#define MAXTIME 60.
#define ESTEPSIZE 0.05
#define EXTEND 0.1
#define TRIALS 3



bool checkCollisions(Device::Ptr device, const State &state, const CollisionDetector &detector, const Q &q, size_t frame) {
	State testState;
	CollisionDetector::QueryResult data;
	bool colFrom;

	testState = state;
	device->setQ(q,testState);
	colFrom = detector.inCollision(testState,&data);
	if (colFrom) {
		cerr << "Configuration in collision: Frame" << frame << endl;
		cerr << "Colliding frames: " << endl;
		FramePairSet fps = data.collidingFrames;
		for (FramePairSet::iterator it = fps.begin(); it != fps.end(); it++) {
			cerr << (*it).first->getName() << " " << (*it).second->getName() << endl;
		}
		return false;
	}
    return true;
}



TimedStatePath linInterp (SerialDevice::Ptr device, State state, vector<Transform3D<> > points, double duration, WorkCell::Ptr wc)
{
    TimedStatePath res;
    rw::invkin::ClosedFormIKSolverUR::Ptr closedFormSovler = rw::core::ownedPtr (new rw::invkin::ClosedFormIKSolverUR (device, state));
    std::vector< Q > solutions;
    // create Collision Detector
    rw::proximity::CollisionDetector detector ( wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy ());

    for (size_t i = 0; i < points.size() - 1; i++)
    {
        CartesianLinearInterpolator interp (points[i], points[i+1], duration);
        
        
        for (double j = 0; j < duration; j += 0.05) {
            

            solutions = closedFormSovler->solve(interp.x (j), state );
            for (unsigned int k = 0; k < solutions.size (); k++) {
                // set the robot in that configuration and check if it is in collision
                device->setQ ( solutions[k], state);

                if (!detector.inCollision (state)) {
                    res.push_back (TimedState (j, state));    // save it
                    break;                        // we only need one
                }
            }

            
        }
    }

    return res;
}

TimedStatePath paraInterp (SerialDevice::Ptr device, State state, vector<Transform3D<> > points, double duration, WorkCell::Ptr wc)
{
    TimedStatePath res;
    rw::invkin::ClosedFormIKSolverUR::Ptr closedFormSovler = rw::core::ownedPtr (new rw::invkin::ClosedFormIKSolverUR (device, state));
    std::vector< Q > solutions;
    // create Collision Detector
    rw::proximity::CollisionDetector detector ( wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy ());
    if (points.size() > 1)
    {
            InterpolatorTrajectory<Transform3D<> > traj;
        for (size_t i = 0; i < points.size() - 2; i++)
        {
            CartesianLinearInterpolator interp1 (points[i], points[i+1], duration);
            CartesianLinearInterpolator interp2 (points[i+1], points[i+2], duration);
            ParabolicBlend blend(interp1, interp2, duration/2);
            if (i == 0)
                traj.add(interp1);
            
            traj.add(blend,interp2);
        }    
        for (double j = 0; j <= traj.duration(); j += 0.05) {
            

            solutions = closedFormSovler->solve(traj.x(j), state );
            for (unsigned int k = 0; k < solutions.size (); k++) {
                // set the robot in that configuration and check if it is in collision
                device->setQ ( solutions[k], state);

                if (!detector.inCollision (state)) {
                    res.push_back (TimedState (j, state));    // save it
                    break;                        // we only need one
                }
            }

            
        }
        
    }
    else
        cout << "Error: Not enough points to interpolate between" << endl;


    return res;
}
CollisionDetector::Ptr detector;
void RRTconnect(string deviceName , State state, vector<Transform3D<> > points3D, WorkCell::Ptr wc)
{



    int index = 0;
        for (double extend = 0.02; extend <= 1.0; extend += EXTEND) 
        {
            
            for (int trial = 0; trial < TRIALS; trial++) 
            {
                
                std::cout << "Progress: " << (extend + trial * EXTEND / TRIALS) * 100 << " %          \r" << std::flush;

                rw::math::Math::seed ();

                Device::Ptr device = wc->findDevice (deviceName);
                if (device == NULL) {
                    cerr << "Device: " << deviceName << " not found!" << endl;
                    break;
                }
                SerialDevice::Ptr deviceSer = wc->findDevice<SerialDevice>(deviceName);
                if (deviceSer == NULL) {
                    cerr << "Device: " << deviceName << " not found!" << endl;
                    break;
                }
                Frame* WORLD = wc->findFrame("WORLD");
                if (WORLD == NULL)
                {
                    cerr << "Device: " << "WORLD" << " not found!" << endl;
                    break;
                }
                Frame* URFrame = wc->findFrame("URReference");
                if (URFrame == NULL)
                {
                    cerr << "Device: " << "URReference" << " not found!" << endl;
                    break;
                }
                Frame* toolFrame = wc->findFrame("GraspTCP");
                if (toolFrame == NULL)
                {
                    cerr << "Device: " << "GraspTCP" << " not found!" << endl;
                    break;
                }

                Transform3D<>  transformWU = Kinematics::frameTframe (URFrame, WORLD, state);

                std::vector<Q> to;
                detector = ownedPtr (new CollisionDetector (wc, ProximityStrategyFactory::makeDefaultCollisionStrategy ()));

                rw::invkin::ClosedFormIKSolverUR::Ptr closedFormSovler = rw::core::ownedPtr (new rw::invkin::ClosedFormIKSolverUR (deviceSer, state));
                for (size_t j = 0; j < points3D.size(); j++)
                {
                    std::vector< Q > solutions = closedFormSovler->solve(points3D[j] , state);
                    
                    for (size_t i = 0; i < solutions.size (); i++) {
                        // set the robot in that configuration and check if it is in collision
                        device->setQ (solutions[i], state);
                        if (!detector->inCollision (state)) {
                            to.push_back(solutions[i]);    // save it
                            break;                // we only need one
                        }
                    }
                    if (to.size() == 0)
                        cout << "Frame " << j << " in collision" << endl;
                }

                //cout << to.size() << endl;
                
                device->setQ (to[0], state);

                //Kinematics::gripFrame (bottle_frame, tool_frame, state);
                CollisionDetector detector (wc,
                                            ProximityStrategyFactory::makeDefaultCollisionStrategy ());
                PlannerConstraint constraint = PlannerConstraint::make (&detector, device, state);

                QSampler::Ptr sampler    = QSampler::makeConstrained (QSampler::makeUniform (device), constraint.getQConstraintPtr ());
                QMetric::Ptr metric      = MetricFactory::makeEuclidean< Q > ();
                QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner ( constraint, sampler, metric, extend, RRTPlanner::RRTConnect);

                for (size_t i = 0; i < to.size(); i++)
                {
                    if (!checkCollisions (device, state, detector, to[i], i))
                        break;
                }



                TimedStatePath tStatePath;
                // cout << "Planning from " << from << " to " << to << endl;
                QPath path;
                Timer t;
                t.resetAndResume ();
                vector <Vector3D<>> logging;

                for (size_t i = 0; i < to.size()-1; i++)
                {
                    path.clear();
                    planner->query (to[i], to[i+1], path, MAXTIME);
                

                    t.pause ();

                    // cout << "Path of length " << path.size() << " found in " << t.getTime() << "
                    // seconds." << endl;
                    if (t.getTime () >= MAXTIME) 
                    {
                        cout << "\nNotice: max time of " << MAXTIME << " seconds reached." << endl;
                    }
                    // visualize them
                    if (path.size () >= 2) 
                    {
                        double distance = 0;
                        Q last (path.front ());
                        for (Q q : path) 
                        {
                            distance += (q-last).norm2();
                        }

                        //mydata << t.getTime () << "\t" << distance << "\t" << extend << "\t" << path.size () << "\n";

                        double time = 0.0;
                        
                        for (size_t i = 0; i < path.size (); i += 0.05) 
                        {
                            device->setQ (path.at (i), state);
                            
                            logging.push_back((device->baseTframe(toolFrame, state) * transformWU).P());
                            tStatePath.push_back (TimedState (time, state));
                            time += i;
                        }

                    }
                }
                cout << "x: " << logging.at(0)[0] << " y: " << logging.at(0)[1] << " z: " <<  logging.at(0)[2] << endl;
                ofstream coordOut("coords"+std::to_string(index)+".txt");
                for (size_t g = 0; g < logging.size(); g++)
                {
                    coordOut << logging.at(g)[0] << " , " << logging.at(g)[1] << " , " << logging.at(g)[2] << endl;
                }
                rw::loaders::PathLoader::storeTimedStatePath (*wc, tStatePath, "visuRRTConnect"+std::to_string(index++)+".rwplay");
            }

            
        }
        std::cout << "Progress: " << 100 << " %          " << std::endl;

}

Mat convertToOpenCV(const Image* img)
{
    cv::Mat image = cv::Mat (img->getHeight (),
                                img->getWidth (),
                                CV_8UC3,
                                (rw::sensor::Image*) img->getImageData ());
    Mat imflip, imflip_mat;
    cv::flip (image, imflip, 1);
    cvtColor (imflip, imflip_mat, COLOR_RGB2BGR);
    return imflip_mat;
}





int main(int argc, char** argv) 
{

    

    string deviceName = "UR-6-85-5-A";

    static const std::string WC_FILE = std::string ("../../WorkCell/Scene.wc.xml");
    // load workcell
    WorkCell::Ptr wc = WorkCellLoader::Factory::load (WC_FILE);
    if (wc.isNull ()) {
        RW_THROW ("COULD NOT LOAD scene... check path!");
    }

    // load workcell
    WorkCell::Ptr wcObstacle = WorkCellLoader::Factory::load ("../../WorkCellObstacle/Scene.wc.xml");
    if (wcObstacle.isNull ()) {
        RW_THROW ("COULD NOT LOAD scene... check path!");
    }
    // find Device
    SerialDevice::Ptr robotUR5 = wc->findDevice<SerialDevice>(deviceName);
    if (robotUR5.isNull ()) {
        RW_THROW ("COULD not find device UR5 ... check model");
    }
    Device::Ptr device = wc->findDevice (deviceName);
    if (device == NULL) 
    {
        cerr << "Device: " << deviceName << " not found!" << endl;
        return 0;
    }
    Frame* WORLD = wc->findFrame("WORLD");
    if (WORLD == NULL)
    {
        cerr << "Device: " << "WORLD" << " not found!" << endl;
        return 0;
    }
    Frame* URFrame = wc->findFrame("URReference");
    if (URFrame == NULL)
    {
        cerr << "Device: " << "URFrame" << " not found!" << endl;
        return 0;
    }
    Frame* URTCPFrame = wc->findFrame("UR-6-85-5-A.TCP");
    if (URTCPFrame == NULL)
    {
        cerr << "Device: " << "UR-6-85-5-A.TCP" << " not found!" << endl;
        return 0;
    }
    Frame* toolFrame = wc->findFrame("GraspTCP");
    if (toolFrame == NULL)
    {
        cerr << "Device: " << "GraspTCP" << " not found!" << endl;
        return 0;
    }


    

    Frame* cameraL = wc->findFrame("Camera_Left");
    if (cameraL == NULL)
    {
        cerr << "Device: " << "Camera_Left" << " not found!" << endl;
        return 0;
    }
    Frame* cameraR = wc->findFrame("Camera_Right");
    if (cameraR == NULL)
    {
        cerr << "Device: " << "Camera_Right" << " not found!" << endl;
        return 0;
    }
    const PropertyMap& properties = cameraL->getPropertyMap ();
    if (!properties.has ("Camera"))
        RW_THROW ("Camera frame does not have Camera property.");

    const std::string parameters = properties.get< std::string > ("Camera");
    std::istringstream iss (parameters, std::istringstream::in);
    double fovy;
    int width;
    int height;
    iss >> fovy >> width >> height;
    
    Mat imgLeft;
    Mat imgRight;


    RobWorkStudioApp app ("");
    RWS_START (app)
    {
        RobWorkStudio* const rwstudio = app.getRobWorkStudio ();
        rwstudio->postOpenWorkCell (WC_FILE);
        TimerUtil::sleepMs (1000);

        const SceneViewer::Ptr gldrawer = rwstudio->getView ()->getSceneViewer ();
        const GLFrameGrabber::Ptr framegrabber = ownedPtr (new GLFrameGrabber (width, height, fovy));
        framegrabber->init (gldrawer);
        SimulatedCamera::Ptr simcamL = ownedPtr (new SimulatedCamera ("SimulatedCamera", fovy, cameraL, framegrabber));
        SimulatedCamera::Ptr simcamR = ownedPtr (new SimulatedCamera ("SimulatedCamera", fovy, cameraR, framegrabber));
        simcamL->setFrameRate (100);
        simcamL->initialize ();
        simcamL->start ();
        simcamL->acquire ();
        simcamR->setFrameRate (100);
        simcamR->initialize ();
        simcamR->start ();
        simcamR->acquire ();

        static const double DT = 0.001;
        const Simulator::UpdateInfo info (DT);
        State state = wc->getDefaultState ();
        int cnt     = 0;
        const Image* img;
        while (!simcamL->isImageReady ()) 
        {
            simcamL->update (info, state);
            cnt++;
        }
        img = simcamL->getImage ();
        simcamL->stop ();
        imgLeft = convertToOpenCV(img);

        simcamL->acquire ();
        while (!simcamR->isImageReady ()) 
        {

            simcamR->update (info, state);
            cnt++;
        }
        img = simcamR->getImage ();
        imgRight = convertToOpenCV(img);

        simcamR->stop ();
        app.close ();
    }
    RWS_END ()


    Stereo stereo;

    string calibrationFile = "../calibration.txt";

    Mat coord = stereo.everythingVision(imgLeft, imgRight, calibrationFile);


    /*

    State stateInv = wcObstacle->getDefaultState ();
    rw::invkin::ClosedFormIKSolverUR::Ptr closedFormSovlerRRT = rw::core::ownedPtr (new rw::invkin::ClosedFormIKSolverUR (robotUR5, stateInv));
    rw::proximity::CollisionDetector detector ( wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy ());
    std::vector< Q > solutions;
    std::vector< Q > pointsQ;


    for (size_t i = 0; i < points3d.size(); i++)
    {
        solutions = closedFormSovlerRRT->solve(points3d[i], stateInv );
        for (unsigned int k = 0; k < solutions.size (); k++) {
            robotUR5->setQ ( solutions[k], stateInv);
            if (!detector.inCollision (stateInv)) 
            {
                pointsQ.push_back(solutions[k]);    // save it
                break;                        // we only need one
            }
        }
    }
    */

    vector<Transform3D<> > points3d;
    vector< Q > pointsQ;


    Transform3D<>  transformUW = Kinematics::frameTframe (URFrame, WORLD, wc->getDefaultState());
    Transform3D<>  transformTT = Kinematics::frameTframe (toolFrame, URTCPFrame, wc->getDefaultState());


    //points3d.push_back( transformUW * Transform3D(Vector3Dd(0.250, 0.474, 0.200),RPY( 0.01, 0.01, 3.14).toRotation3D())  ); //Square pick
    //points3d.push_back( transformUW * Transform3D(Vector3Dd(0.250, 0.474, 0.350),RPY( 0.01, 0.01, 3.14).toRotation3D()) ); //Square above

    points3d.push_back( transformUW * Transform3D(Vector3Dd(-0.1, 0.4, 0.3),RPY( 0.01, 0.01, 3.14).toRotation3D()) * transformTT); //Home


    //points3d.push_back( transformUW * Transform3D(Vector3Dd(coord.at<double>(0), coord.at<double>(1), coord.at<double>(2) + 0.20),RPY( 0.01, 0.01, 3.14)) * transformTT ); //detected object above
    points3d.push_back( transformUW * Transform3D(Vector3Dd(coord.at<double>(0), coord.at<double>(1) , coord.at<double>(2) ), RPY( 0.01, 0.01, 3.14))  * transformTT); //detected object pick
    //points3d.push_back( transformUW * Transform3D(Vector3Dd(coord.at<double>(0), coord.at<double>(1), coord.at<double>(2) + 0.20),RPY( 0.01, 0.01, 3.14))  * transformTT  ); //detected object above

    cout << coord.at<double>(0) << " " <<  coord.at<double>(1) << " " <<  coord.at<double>(2) << endl;
/*
    points3d.push_back(Transform3D(Vector3Dd(0.00, 0.474, 0.430),RPY( 0.01, 0.01, 3.14).toRotation3D()) ); //Bottle above
    points3d.push_back(Transform3D(Vector3Dd(0.00, 0.474, 0.300),RPY( 0.01, 0.01, 3.14).toRotation3D()) ); //Bottle pick
*/
    //points3d.push_back(transformUW * Transform3D(Vector3Dd(-0.250, 0.474, 0.300),RPY( 0.01, 0.01, 3.14))    ); //Cylinder above
    //points3d.push_back(transformUW * Transform3D(Vector3Dd(-0.250, 0.474, 0.225),RPY( 0.01, 0.01, 3.14))   ); //Cylinder pick

    points3d.push_back( transformUW * Transform3D(Vector3Dd(0.290, -0.5, 0.2),RPY( 0.01, 0.01, 3.14))  * transformTT  ); //Place - goal


    

    pointsQ.push_back( Q(1.289, -1.837, -1.92, -0.959, 1.58, -0.292));//Cylinder pick
    pointsQ.push_back( Q(1.289, -1.689, -1.681, -1.347, 1.58, -0.292)); //Cylinder above

    pointsQ.push_back( Q(1.801, -1.516, -1.654, -1.547, 1.582, 0.22)); //Bottle above
    pointsQ.push_back( Q(1.801, -1.578, -1.934, -1.205, 1.582, 0.22)); //Bottle pick

    pointsQ.push_back( Q(2.26, -1.722, -1.779, -1.207, 1.58, 0.679)); //Cylinder above
    pointsQ.push_back( Q(2.26, -1.803, -1.893, -1.012, 1.58, 0.679)); //Cylinder pick

    pointsQ.push_back( Q(-0.837, -1.791, -1.694, -1.234, 1.562, -2.417)); //Place



    TimedStatePath linearMotion = linInterp (robotUR5, wc->getDefaultState (), points3d, 5, wc);
    PathLoader::storeTimedStatePath (*wc, linearMotion, "./visu.rwplay");

    TimedStatePath linearParabolicMotion = paraInterp (robotUR5, wc->getDefaultState (), points3d, 5, wc);
    PathLoader::storeTimedStatePath (*wc, linearParabolicMotion, "./visuPB.rwplay");



    //cout << img << endl;

    RRTconnect("UR-6-85-5-A",  wcObstacle->getDefaultState (),  points3d, wcObstacle);
   


}