#!/usr/bin/python3

import sdurws as rws
import sdurw as rw
import sdurw_simulation as rw_simulation 
import sdurw_proximity as rw_prox
from sdurw_proximitystrategies import ProximityStrategyFactory as PSF

import numpy as np
import math, time

import cv2 #This might cause segfault unless you have use the "opencv-contrib-python-headless" pkg from pip


class RWSHandler():
    def __init__(self,rwstudio,wc_file) -> None:
        self.rwstudio = rwstudio
        self.wc = rw.WorkCellLoaderFactory.load(wc_file)

        if self.wc.isNull():
            raise RuntimeError("WorkCell Not Fount")

        self.rwstudio.setWorkCell(self.wc)

        self.framegrapper = None
        self.cameras = ("Camera_Right", "Camera_Left")
        self.cameras25D = ("Scanner25D",)  

        self.state = self.wc.getDefaultState()

       

        if (not self.wc == None) and (not self.wc.isNull()):
            
            # Create a GLFrameGrabber if there is a camera frame with a Camera property set
            cameraFrame = self.wc.findFrame(self.cameras[0])
            if not cameraFrame == None:
                if cameraFrame.getPropertyMap().has("Camera"):
                    cameraParam = cameraFrame.getPropertyMap().getString("Camera")
                    fovy, width, height = cameraParam.split(' ')
                    self.framegrapper = rw_simulation.GLFrameGrabber(int(width),int(height),float(fovy))
                    gldrawer = self.rwstudio.getView().getSceneViewer()
                    self.framegrapper.init(gldrawer)
            
            cameraFrame25D = self.wc.findFrame(self.cameras25D[0])
            if not cameraFrame25D == None:
                if cameraFrame25D.getPropertyMap().has("Scanner25D"):
                    cameraParam = cameraFrame25D.getPropertyMap().getString("Scanner25D")
                    fovy, width, height = cameraParam.split(' ')
                    self.framegrapper25D = rw_simulation.GLFrameGrabber25D(int(width),int(height),float(fovy))
                    gldrawer = self.rwstudio.getView().getSceneViewer()
                    self.framegrapper25D.init(gldrawer)

            self.device = self.wc.findDevice("UR-6-85-5-A")
            self.step=-1

    def toOpenCVImage(self, rw_img):
        img = np.zeros((rw_img.getWidth(),rw_img.getHeight(),rw_img.getNrOfChannels()),dtype=np.uint8)

        for x in range(0,rw_img.getWidth()):
            for y in range(0,rw_img.getHeight()):
                for c in range(0, rw_img.getNrOfChannels()):
                    img[x,y,c]=rw_img.getPixelValuei(x,y,c)

        return img

    def getImage(self):
        img = []
        if not self.framegrapper == None:
            for i in range(0,len(self.cameras)):
                cameraFrame = self.wc.findFrame(self.cameras[i])
                self.framegrapper.grab(cameraFrame,self.state)

                rw_img = self.framegrapper.getImage()

                cvImgTmp = self.toOpenCVImage(rw_img)
                img.append(cv2.transpose(cvImgTmp))

        print("Camera_Right")
        self.printProjectionMatrix("Camera_Right")
        print("Camera_Left")
        self.printProjectionMatrix("Camera_Left")
        return img

    def get25DImage(self):

        img25 = []
        print("print get 25DImg", self.framegrapper25D)        
        if not self.framegrapper25D == None:
            for cam25d in self.cameras25D:
                print("cam cam25d")

                cameraFrame25D = self.wc.findFrame(cam25d)
                self.framegrapper25D.grab(cameraFrame25D, self.state)

                img25.append(self.framegrapper25D.getImage())                
                rw.PointCloud.savePCD(img25[-1],cam25d+".pcd")
        
        return img25

    def playTimedStatePath(self, path:rw.PathTimedState):
        t_last = 0
        for ts in path:
            time.sleep(ts[0]-t_last)
            t_last = ts[0]
            self.rwstudio.postState(ts[1])

    def checkCollisions(self, device, testState, detector, q):
        data = rw.FramePairVector()

        device.setQ(q,testState)
        colFrom = detector.inCollision(testState,data)

        if colFrom:
            print("Configuration in collision: "+ str(q) + "\n")
            print("Colliding Frames: \n")
            
            for fPair in data:
                print( fPair.first.getName() + " " + fPair.second.getName())

        return not colFrom

    def createPathRRTConnect(self, From, To , extend, maxTime):
        self.device.setQ(From, self.state)
        CDevice = self.device.cptr()
        super().getRobWorkStudio().setState(self.state)
        detector    = rw_prox.ownedPtr(rw_prox.CollisionDetector(self.wc, PSF.makeDefaultCollisionStrategy()))
        constraint  = rw.PlannerConstraint.make(detector,CDevice,self.state)
        sampler     = rw.QSampler.makeConstrained(rw.QSampler.makeUniform(CDevice),constraint.getQConstraintPtr().asCPtr())
        metric      = rw.MetricFactory.makeEuclideanQ()
        planner     = rw.RRTPlanner.makeQToQPlanner(constraint,sampler,metric,extend,rw.RRTPlanner.RRTConnect)

        self.path.clear()
        if not self.checkCollisions(self.device,self.state,detector,From):
            print(From, "is in colission")
        if not self.checkCollisions(self.device,self.state,detector,To):
            print(To, "is in colission")

        t = rw.Timer()
        t.resetAndResume()
        planner.query(From,To,self.path,maxTime)
        t.pause()

        if t.getTime() >= maxTime:
            print("Notice: max time of ", maxTime ," seconds reached.")
        
        duration = 10

        if self.path.size() == 2:
            linInt = rw.LinearInterpolatorQ(From, To, duration)
            tempQ = rw.PathQ()
            for i in range(0,duration+1):
                tempQ.push_back(linInt.x(i))
            self.path=tempQ

    def printProjectionMatrix(self, frameName):
        cameraFrame = self.wc.findFrame(frameName)
        
        if (cameraFrame is not None):
            if (cameraFrame.getPropertyMap().has("Camera")):
                # Read the dimensions and field of view
                cameraParam = cameraFrame.getPropertyMap().getString("Camera")
                fovy, width, height = cameraParam.split(' ')
                fovy = float(fovy)
                width = int(width)
                height = int(height)

                fovy_pixel = height / 2 / math.tan(fovy*(2*math.pi)/360/2.0)
                print("Intrinsic parameters:")
                KA = np.array([fovy_pixel, 0, width/2.0, 0, 0, fovy_pixel, height/2.0, 0, 0, 0, 1, 0]).reshape((3,4))
                print(KA)

                camPosOGL = cameraFrame.wTf(self.state)
                openGLToVis = rw.Transform3D(rw.RPY(-math.pi,0,math.pi).toRotation3D())
                H = rw.inverse(camPosOGL*rw.inverse(openGLToVis))

                print("Extrinsic parameters:")
                # for i in range(3):
                #     print(H[i,0],H[i,1],H[i,2],H[i,3])
                # print(0,0,0,1)

                H_1 = []
                for i in range(3):
                    H_1.append([H[i,0],H[i,1],H[i,2],H[i,3]])
                row = np.array([0, 0, 0, 1])
                H_Convert = np.append(H_1,[row], axis=0)
                
                print(H_Convert)

            

if __name__ == '__main__':
    app = rws.RobWorkStudioApp("")

    app.start()
    rwstudio = app.getRobWorkStudio()

    handel = RWSHandler(rwstudio,"/home/kalor/Workspace/roviexercises/RoViProject/WorkCell/Scene.wc.xml")

    handel.getImage()

    while app.isRunning():
        pass 
    
