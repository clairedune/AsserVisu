// this code test the asser visu 
// given as an input to Andreis function
// for step computing


//------------- LOAD The Files ---------------------//
path=get_absolute_file_path("scilab-src");
disp('HOME:'+path),
getd(path + "src/graphisme"); // pour charger un repertoire en entier
getd(path + "src/transformation");  
getd(path + 'src/projectionPers');
getd(path + 'src/asserVisu');
getd(path + 'src/hrp2')
//Andrei
exec('src/prwControl/prwControl.sci');


exec('../HuMAnS/KickStart.sci');
execstr(LoadModule('../HuMAnS/Kernel'));
execstr(LoadModule('../HuMAnS/LagrangianModel/HRP2'));
execstr(LoadModule('../HuMAnS/LagrangianDynamics/Complete'));
execstr(LoadModule('../HuMAnS/LagrangianModel/HRP2/TaskFunctionDefinition'));
execstr(LoadModule('../HuMAnS/LagrangianModel/HRP2/JacadiModel'));
execstr(LoadModule('../HuMAnS/ActuationModel/DelayedSampling/ComputedTorqueControl'));
execstr(LoadModule('../HuMAnS/ActuationModel/DelayedSampling'));

if(CPPVERSION) then
  execstr(LoadModule2('../HuMAnS/Kernel'));
  execstr(LoadModule('../HuMAnS/ActuationModel/BipActuators/ActuatorsGeometry'));
  //LoadJacadi();
end

setActuationSamplingPeriod(2.5e-3);
setActuationSamplingDelay(0.1e-3);
setNbActuators(30);
//Andrei/////

//-------------------------------------------------//


//---------  USER CHOICES --------------------------//
// choose the number of ddl of the robot
// 3 for HRP2 Andrei Com Control
// 6 for free Camera
NB_DDL = 3;

//choose the test case relative to the paper of Mombaur
// test case=0, let the user set its own param
TEST_CASE = 0;
if (TEST_CASE ~= 0)
 [ poseComInit poseComFinal pose_cMo] = testCase(TEST_CASE);
else 
// Init Pose of the CoM expressed in the robot frame
poseComInit = [ 0 0 0 0 0 90*%pi/180 ];
// Final Pose of the CoM
poseComFinal = [0 1 0 0 0 90*%pi/180 ];
// target pose in the init camera frame
pose_cMo = [0 0 3 0 0 0]; 
end;


// -- For Simulation
dt = 1/10;// time delay between two command
cote = 0.05; // size of the target dot

// -- For Visual Servoing
lambda = 8;//aservissement gain
threshold = 1e-3 ;// threshold on the error to stop



//------------ SETTING UP -------------------------//




//------compute the homogeneous matrix related to the
// init and final camera position
wMcomInit = homogeneousMatrixFromPos(poseComInit);
wMr = wMcomInit;
wMcomFinal = homogeneousMatrixFromPos(poseComFinal);

//------link camera/CoM : the camera is linked to the HRP2
// with two rotations of pi/2 
// comMc is the fixed position of the camera in the com frame
// comVc is the fixed twist matrix that allows to change the frame of
// expression of the velocity 
[comMc,comVc]=loadHRP2camCom();

// build the homogeneous matrix related to the camera pose
wMc = wMcomInit*comMc ;

//------Init set up
// its position in the camera frame is 
cMo = homogeneousMatrixFromPos(pose_cMo); 
// its position in the world frame is
wMo = wMc*cMo; 
// create the target
L = 0.5; // related to the target size 
oP = mire5points (L); 
wP = changeFrameMire(oP,wMo); 
// compute the first projection on the view
p = projectMire(oP,inv(cMo));

//------Final set up 
// create the desired pose
wMcdes = wMcomFinal*comMc ;
cdesMo = inv(wMcdes)*wMo;
pdes = projectMire(oP,inv(cdesMo));

//------create the figures
hf2d = createPlanImage(1,"Point 2D");

mire2DDraw(p,cote,3);
show_pixmap()
mire2DDraw(pdes,cote,5);
show_pixmap()
disp('This is the first point projection')
disp('Green : current one, Red Desired one')
//disp('Click to display the servoing')
//xclick;

// create the window
hf = createFigure3D(2,"Camera Motion",2);
Camera3DDrawColor(0.1,wMc,3);// display the first camera
Camera3DDrawColor(0.1,wMcdes,5);// display the first camera
Mire3DDraw(wP);
show_pixmap()

disp('Red : desired pose, Green : init pose and target');
halt();

//create a windows to display the top view
hf2d_2 = createFigure2D(4,"TopView");

//----------------- VISUAL SERVOING LOOP -----------------//

// Visual servoing param
Zint = 1; // approximation of the object depth


// vcam is the camera velocity
// vrobot is the robot velocity
// test on DDL
if(NB_DDL==3)
  L = matIntMire3ddl(p,Zint); 
  vcam = [0 0 0];
elseif(NB_DDL==6)
  L = matIntMire6ddl(p,Zint); 
  vcam = [0 0 0 0 0 0];
else
  disp('NB_DDL should be 6 or 3... kill the appli here');
  halt();
end;

//---
  vrobot = zeros(1,3); // desired robot velocity
  vcamReal= vcam; // real camera montion
  vrobotReal = zeros(1,3); // real robot displacement

// create a window to display the error
norme=[];
hf2d_2 = createFigure2D(3,"Error");

//create a windows to display the top view
velocity=[];
hf2d_3 = createFigure2D(5,"Velocity");

// launch the servo
er = 10; // error value init

//Andrei
//state of the robot [x;dx;ddx;y;dy;ddy;theta;dtheta;ddtheta]
RobotReal = zeros(9,1);
//we must enter in robot init the value of pose init
//poseComInit = [ 0 0 0 0 0 45*%pi/180 ];
RobotReal(1) = poseComInit(1) ;  
RobotReal(4) = poseComInit(2) ;
RobotReal(7) = poseComInit(6) ;
k = 1;
//Andrei
while(er>threshold)

  // --- System Simulation Using velocity
  // motion performed
  //c1Mc2 = computeMotionCam(vcamReal,dt);
  //vrobot6ddl = [vrobotReal(1) vrobotReal(2) 0 0 0 vrobotReal(3)];
  //r1Mr2 = expMapDirectRxRyRz(vrobot6ddl,dt);
  //wMr = wMr*r1Mr2;
  //pRobotInWorld = pFromHomogeneousMatrix(wMr);

  
  // new camera position
  //wMc=wMc*c1Mc2; 
  wMc = wMr * comMc;
  pCamInWorld = pFromHomogeneousMatrix(wMc);
  
   
  
  
  // targetprojection
  p = projectMire(wP,wMc);

  // uncomment the two lines to compute Z at each step
  //cMo = inv(wMc)*wMo;
  //Zint = cMo(3,4);
       
  // --- Classical visual servoing     
  // compute the error
  E = p-pdes;
  e = [E(:,1)'  E(:,2)'  E(:,3)'  E(:,4)'  E(:,5)'];
  norme = [norme norm(e)];
  er = norm(e);
  // compute interaction matrix
  if(NB_DDL==3)
    L = matIntMire3ddl(p,Zint); 
  elseif(NB_DDL==6)
    L = matIntMire6ddl(p,Zint); 
  end;
  // desired velocity of the camera
  vcam = computeVelocity(lambda, L,e');
  // desired corresponding robot motion
  vrobot = convertVelocityCamRobot(vcam,comVc); 

  //Andrei
  //printf('Andrei');
  //vrobot = [1/8,0,0];
  [RobotReal, RobotPrw] = WalkingTrajectory_withFP(vrobot,RobotReal,k);
  //printf('Andrei finished');
  k = k + 1;
  
 // pause;
  vrobotReal = RobotReal([2;5;8]); // attention vRobot dans le repere monde.
  disp(vrobot);
  disp(vrobotReal);
  probotReal = [RobotReal(1) RobotReal(4) 0 0 0 RobotReal(7)];
  wMr = homogeneousMatrixFromPos(probotReal);
  //Andrei
  //vcamReal = convertVelocityRobotCam(vrobotReal,comVc); 
  
  
  // --- Displays
  xset("window",1); 
  mire2DDraw(p,cote,3);
  show_pixmap();
  
  xset("window",2);
  Camera3DDraw(0.005,wMc);
  show_pixmap()
  
  xset("window",3);
  plot(threshold*ones(1,length(norme)))
  her=gce();
  her.foreground=5;
  plot(norme);
  her=gce();
  her.foreground=3;
  show_pixmap();
  
  xset("window",4);
  AxeZ2DDraw(0.02,wMc);
  show_pixmap();
  
  xset("window",5);
  velocity =[velocity;vcam ];
  plot2d(velocity);
  show_pixmap()
  

 
end;

xset("pixmap",0);




