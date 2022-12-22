#ifndef SAMPLEPLUGIN_HPP
#define SAMPLEPLUGIN_HPP

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

// RobWorkStudio includes
#include <RobWorkStudioConfig.hpp>    // For RWS_USE_QT5 definition
#include <rws/RobWorkStudioPlugin.hpp>

// OpenCV 3
#include <opencv2/opencv.hpp>

// Qt
#include "ui_SamplePlugin.h"

#include <rw/loaders/ImageLoader.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rws/RobWorkStudio.hpp>

#include <QPushButton>
#include <QTimer>
#include <functional>

using namespace rw::common;
using namespace rw::graphics;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::sensor;
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

class SamplePlugin : public rws::RobWorkStudioPlugin, private Ui::SamplePlugin
{
    Q_OBJECT
    Q_INTERFACES (rws::RobWorkStudioPlugin)
    Q_PLUGIN_METADATA (IID "dk.sdu.mip.Robwork.RobWorkStudioPlugin/0.1" FILE "plugin.json")
  public:
    SamplePlugin ();
    virtual ~SamplePlugin ();

    /**
     * @brief called when a workcell is opened
     * @param workcell [in] that has been loaded
     */
    virtual void open (rw::models::WorkCell* workcell);

    /**
     * @brief called when a workcell is being closed.
     */
    virtual void close ();

    /**
     * @brief is called when RobWorkStudio instance is valid. Can be used
     * to initialize values in the plugin that depend on RobWorkStudio
     *
     * @note DO NOT fire any events under initialization since the order of
     * which plugins are initialized is unknown. Therefore undefined behavior
     * might occour. Instead wait until open is called for the first time.
     */
    virtual void initialize ();

  private slots:

    void on_btnCalculatePath ();
    void on_btnRunPath ();
    void on_btnGetScan ();
    void on_btnGetImage ();

    void on_spinBoxChanged (int);

    void on_timerTick ();

  private:
    /**
     * @brief Get image from Cameras
     */
    void getImage ();

    /**
     * @brief Get 25Dimage from Cameras
     */
    void get25DImage ();

    /**
     * @brief Function is called whenever RobWorkStudio state is changed
     * @param state 
     */
    void stateChangedListener (const rw::kinematics::State& state);


    /**
     * @brief Check for collisions if the robot is at a given Q and print which frames are in collision
     * 
     * @param device [in] the robot to move
     * @param state [in] initial state
     * @param detector [in] the collision detector to use
     * @param q [in] the Configuration of the robot that should be checked for collisions
     * @return true if in collision
     * @return false if not in collision
     */
    bool checkCollisions (Device::Ptr device, const State& state, const CollisionDetector& detector,
                          const Q& q);

    /**
     * @brief Create a Path using RRTConnect from \b from to \b to, using step size \b extend
     * 
     * @param from [in] where to start the path
     * @param to [in] where to end the path
     * @param extend [in] maximum move distance per step
     * @param maxTime [in] maxtime befor planning is abandoned
     */
    void createPathRRTConnect (Q from, Q to, double extend, double maxTime);

    /**
     * @brief print camera information for camera at a specific frame
     * 
     * @param frameName [in] camera name
     */
    void printProjectionMatrix (std::string frameName);

    /**
     * @brief Convert RobWork Image to OpenCV
     * 
     * @param img 
     * @return cv::Mat 
     */
    static cv::Mat toOpenCVImage (const rw::sensor::Image& img);

    QTimer* _timer;
    QTimer* _timer25D;

    rw::models::WorkCell::Ptr _wc;
    rw::kinematics::State _state;
    rwlibs::opengl::RenderImage *_textureRender, *_bgRender;
    rwlibs::simulation::GLFrameGrabber* _framegrabber;
    rwlibs::simulation::GLFrameGrabber25D* _framegrabber25D;
    std::vector< std::string > _cameras;
    std::vector< std::string > _cameras25D;
    Device::Ptr _device;
    QPath _path;
    int _step;
};

#endif /*RINGONHOOKPLUGIN_HPP_*/
