// test the relative cam/robot position
//;exec('testPositionCamRobot.sce')


// Init Pose of the CoM expressed in the world frame
posewMr        = [3 0 0 -%pi 0 0 ];


//------compute the homogeneous matrix related to the
// init and final camera position
wMr            = homogeneousMatrixFromPos(posewMr);

//------link camera/CoM : the camera is linked to the HRP2
// with two rotations of pi/2 
// comMc is the fixed position of the camera in the com frame
// comVc is the fixed twist matrix that allows to change the frame of
// expression of the velocity 
[comMc,comVc]  = loadHRP2camCom();

// build the homogeneous matrix related to the camera pose
wMc            = wMr*comMc ;

// create the window
hf = createFigure3D(2,"Camera Position",2);
Camera3DDrawColor(1,wMr,1)
show_pixmap()
halt();
Camera3DDrawColor(1,wMc,5)
show_pixmap()






