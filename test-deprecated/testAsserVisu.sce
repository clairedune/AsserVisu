// main program

//------------- LOAD The Files ---------------------//
path=get_absolute_file_path("scilab-src");
disp('HOME:'+path),
getd(path + "src/graphisme"); // pour charger un repertoire en entier
getd(path + "src/transformation");  
getd(path + 'src/projectionPers');
getd(path + 'src/asserVisu');
getd(path + 'src/hrp2')
//-------------------------------------------------//


//---------  USER CHOICES --------------------------//
// choose the number of ddl of the robot
// 3 for HRP2 Andrei Com Control
// 6 for free Camera
NB_DDL = 6;

//choose the test case relative to the paper of Mombaur
// test case=0, let the user set its own param
TEST_CASE = 0;
if (TEST_CASE ~= 0)
 [ poseComInit poseComFinal pose_cMo] = testCase(TEST_CASE);
else 
// Init Pose of the CoM
poseComInit = [ 0 0 0 0 0 0 ];
// Final Pose of the CoM
poseComFinal = [1 1 0 30*%pi/180 0 0 ];
// target pose in the init camera frame
pose_cMo = [0 0 2 0 0 0]; 
end;


// -- For Simulation
dt = 1/30;// time delay between two command
cote = 0.05; // size of the target dot

// -- For Visual Servoing
lambda = 0.9;//aservissement gain
threshold = 1e-3 ;// threshold on the error to stop



//------------ SETTING UP -------------------------//




//------compute the homogeneous matrix related to the
// init and final camera position
wMcomInit = homogeneousMatrixFromPos(poseComInit);
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

// create a window to display the error
norme=[];
hf2d_2 = createFigure2D(3,"Error");

//create a windows to display the top view
velocity=[];
hf2d_3 = createFigure2D(5,"Velocity");

// launch the servo
er = 10; // error value init
while(er>threshold)

  // --- System Simulation
  // motion performed
  c1Mc2 = computeMotion(vcam,dt);
  // new camera position
  wMc=wMc*c1Mc2; 
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
  vcam = computeVelocity(lambda, L,e');

  // --- Displays
  xset("window",1); 
  mire2DDraw(p,cote,3);
  show_pixmap();
  
  xset("window",2);
  Camera3DDraw(cote,wMc);
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



