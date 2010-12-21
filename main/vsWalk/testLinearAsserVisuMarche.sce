// this code test the asser visu 
// given as an input to Andreis function
// for step computing
// Andrei
exec('LoadMarche.sce');
setActuationSamplingPeriod(2.5e-3);
setActuationSamplingDelay(0.1e-3);
setNbActuators(30);




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
poseComInit = [ 0 0.1 0 0 0 0 ];
// Final Pose of the CoM
poseComFinal = [0 0.5 0 0 0 0 ];
// target pose in the init camera frame
pose_cMo = [0 0 2 0 0 0]; 
end;


// -- For Simulation
dt = 1/10;// time delay between two command
cote = 0.05; // size of the target dot

// -- For Visual Servoing
lambda = 0.4;//aservissement gain
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

disp('camera courante')
disp(wMc)


//------Init set up
// its position in the camera frame is 
cMo   = homogeneousMatrixFromPos(pose_cMo); 
// its position in the world frame is
wMo   = wMc*cMo; 
// create the target
L     = 0.1; // related to the target size 
oP    = mire5points (L); 
Nbpts = 5;
wP    = changeFrameMire(oP,wMo); 
// compute the first projection on the view
p     = projectMire(oP,inv(cMo));

//------Final set up 
// create the desired pose

wMcdes         = wMcomFinal*comMc ;
cdesMo         = inv(wMcdes)*wMo;
cdesP          = changeFrameMire(oP,cdesMo);
pdes           = projectMire(oP,inv(cdesMo));
Zdes           = cdesP(3:3:length(cdesP));




//------create the figures
xl = [-0.5;-0.5]
xu = [0.5; 0.5]
hf2d = createPlanImage(1,xl,xu,"Point 2D");

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
Mire3DDraw5pts(wP);
show_pixmap()

disp('Red : desired pose, Green : init pose and target');
halt();

//create a windows to display the top view
hf2d_2 = createFigure2D(4,"TopView");



//----------------- VISUAL SERVOING LOOP -----------------//

//Anrei Np est la taille de l'horizon
Np = 32 ; 
[Mxx,Myy,Mxy,Myx,R,L] = matLinearAsserVisu(pdes,Zdes,Np,Nbpts);
Sdes = [];  //S*
Sk   = [];  //Sk
for i=1:Np
    
    Sdes = [Sdes;pdes];
    Sk   = [Sk;p];
    
end


