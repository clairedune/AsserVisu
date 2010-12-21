// main program

clear
//--------------------------------------------------//
//              LOAD The Files ---------------------//
//--------------------------------------------------//
path=get_absolute_file_path("scilab-src");
disp('HOME:'+path),
getd(path + "src/graphisme"); // pour charger un repertoire en entier
getd(path + "src/transformation");  
getd(path + 'src/projectionPers');
getd(path + 'src/asserVisu');
getd(path + 'src/hrp2')

// load optimisation
exec('../HuMAnS/KickStart.sci');
execstr(LoadModule('../HuMAnS/Kernel'));
if ~c_link('libcfsqp') then exec('../HuMAnS/Kernel/OptimizationTools/fsqp-1.2/loader.sce') ; end 



//--------------------------------------------------//
//                   USER CHOICES                   //
//--------------------------------------------------//
// choose the number of ddl of the robot
// 3 for HRP2 Andrei Com Control x y and rotation in z
// 6 for free Camera
NB_DDL = 6;

//choose the test case relative to the paper of Mombaur
// test case=0, let the user set its own param
TEST_CASE = 0;
//---------------------
if (TEST_CASE ~= 0)
 [ poseComInit poseComFinal pose_cMo] = testCase(TEST_CASE);
else 
// Init Pose of the CoM
poseComInit = [ 0 0 0 0 0 0 ];
// Final Pose of the CoM
poseComFinal = [0 0 0.2 0 0 0 ];
// target pose in the init camera frame
pose_cMo = [0.1 0.1 1 0 0 0]; 
end;


// -- For Simulation
Te = 1/30;// time delay between two command
cote = 0.01; // size of the target dot

// -- For Visual Servoing
//lambda = 0.9;//aservissement gain
threshold = 1e-5 ;// threshold on the error to stop



//--------------------------------------------------//
//------------ SETTING UP -------------------------//
//--------------------------------------------------//

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
L = 0.05; // related to the target size 
oP = mire5points (L); 
wP = changeFrameMire(oP,wMo); 
cP = changeFrameMire(oP,cMo); 
// compute the first projection on the view
p = projectMireDirect(cP)

//------Final set up 
// create the desired pose
wMcdes = wMcomFinal*comMc ;
cdesMo = inv(wMcdes)*wMo;
pdes = projectMire(oP,inv(cdesMo));

//------Create the figures
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
//hf = createFigure3D(2,"Camera Motion",2);
//Camera3DDrawColor(0.1,wMc,3);// display the first camera
//Camera3DDrawColor(0.1,wMcdes,5);// display the first camera
//Mire3DDraw(wP);
//show_pixmap()
//disp('Red : desired pose, Green : init pose and target');
//create a windows to display the top view
//hf2d_2 = createFigure2D(4,"TopView");


//--------------------------------------------------//
//----------- VISUAL SERVOING LOOP -----------------//
//--------------------------------------------------//
// Visual servoing param
Z       = cP(3:3:15) ;        // init depth for the step 
vcam    = [0 0 0 0 0 0];// init velocity
Np      = 10;                   // horizon lenght
U0      = [];                  // create the first control horizon
for i = 1:Np
  U0    = [U0 ; vcam'];
end ;


Nbts    = 5 ;                  // number of points of the target 5
xu      = [0.3;0.2];           // position max of the a 2D point in the image plane 
xl      = [-0.3;-0.2];         // position min of the a 2D point in the image plane 
sm0     = p;                   // the first projection is p
e0      = zeros(Nbts*2,1);     // the first error is e0
Q       = eye(Nbts*2,Nbts*2);  // the weight matrix 
sStar   = pdes;                  // desired features

// control limit
bu1 = [1,1,1,10*%pi/180,10*%pi/180,10*%pi/180]';
bu=[];
for i=1:Np
  bu=[bu;bu1];
end
bl = -1*bu;


// create a window to display the error
norme    =[];
//hf2d_2   = createFigure2D(3,"Error");

//create a windows to display the top view
velocity =[];
//hf2d_3   = createFigure2D(5,"Velocity");


//create a windows for the prediction
//hf2d_4 = createPlanImage(7,"Prediction Mire");


halt()
// launch the servo
er = 10; // error value init
iter=0;
while(er>threshold)

  iter=iter+1;
  disp('---------------------------------')
  disp(iter)
  // --- System Simulation
  // motion performed
  c1Mc2  = computeMotion(vcam,Te);
  // new camera position in teh world frame
  wMc    = wMc*c1Mc2; 
  cMo    = inv(wMc)*wMo;
  cP     = changeFrameMire(oP,cMo); 
  Z      = cP(3:3:15) ;
  // targetprojection
  p      = projectMire(wP,wMc);

  // uncomment the two lines to compute Z at each step
  //cMo = inv(wMc)*wMo;
  //Zint = cMo(3,4);
       
  // --- Visual servoing     
  // compute the error
  e = p-pdes;
  //e = [E(:,1)'  E(:,2)'  E(:,3)'  E(:,4)'  E(:,5)'];
  norme = [norme norm(e)];
  er = norm(e)
 
  // compute the camera velocity
  [x,sm]=predControlLocalMire(Np,Nbts,xl,xu,bl,bu,U0,sm0,sStar,e0,Q,Te,Z);
  sm = [p; sm];
  vcam = x(1:6)

  // --- Displays
  //xset("window",1); 
  //mire2DDraw(p,cote,3);
  //show_pixmap();
  
  //xset("window",2);
  //Camera3DDraw(cote,wMc);
  //how_pixmap()
  
  //xset("window",3);
  //plot(threshold*ones(1,length(norme)))
  //her=gce();
  //her.foreground=5;
  //plot(norme);
  //her=gce();
  //her.foreground=3;
  //show_pixmap();
  
  //xset("window",4);
  //AxeZ2DDraw(0.02,wMc);
  //show_pixmap();
  
  //xset("window",5);
  //velocity =[velocity;vcam ];
  //plot2d(velocity);
  //show_pixmap()
  
  //xset("windows",7)
  //mireEvolutionDraw(Np+1,sm,2);
  //show_pixmap()
    
end;

xset("pixmap",0);



