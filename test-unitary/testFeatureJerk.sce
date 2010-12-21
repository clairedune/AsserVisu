// Test the feature prediction over a time horizon
// for a current state and a given jerk on the horizon
//
//
// remark : all the variables defined here are 
//followed by "_m" not to interfer with function variables
//
// ;exec('testFeatureJerk.sce');
//
//---------------------------------------------------------
clear
//load and check
;exec('Load.sce');
isMirePointsLoaded()

//----------------------------------------------//
//       1. INITIALISE                          //
//----------------------------------------------//

// ----- Settings
Te_m           = 1/10; 	// time delay between two command
Np_m           = 3; 	// length of the time horizon

// ----- Build the P S and U matrices for robot behavior
[Sp_m, Sv_m, Sa_m, Up_m, Uv_m, Ua_m] = buildC(Np_m,Te_m);

// ----- Init Pose of the CoM expressed in the world frame
// w stands for world and
// c stands for CoM
poseC_m        = [0 0 0 -%pi 0 0 ]'; 
wMc_m          = homogeneousMatrixFromPos(poseC_m');

// ----- Init Velocity of the CoM
velocityC_m    = zeros(6,1);

// ----- Init Acceleration
accelerationC_m = zeros(6,1);

// ----- Build State
state = [poseC_m;velocityC_m;accelerationC_m]; 

// ----- Init Jerk of the CoM
jerkC_m        = zeros(6*Np_m,1);

// ----- Link camera/CoM : the camera is linked to the HRP2
// k stands for camera
[cMk_m,cVk_m]  = loadHRP2camCom();
wMk_m          = wMc_m*cMk_m ;

// ----- Init Camera/object pose
posekMo_m      = [0 0 1 0 0 0]; // init pose in meter and rad trans and rot
kMo_m          = homogeneousMatrixFromPos(posekMo_m);
wMo_m          = wMk_m*kMo_m;


// ----- Create the target 
a_m            = 0.2; // related to the target size
Nbpts_m        = 5;
oP_m   	       = mirePoints(Nbpts_m,a_m);
cP_m           = changeFrameMire(oP_m,kMo_m); 
p_m            = projectMireDirect(cP_m);
pinit_m        = p_m;
Z_m            = cP_m(3:3:$); 

// ----- Create the figures
xl             = [-0.6;-0.6];
xu             = [0.6; 0.6];
dot_size       = 0.05;
hf_1           = createPlanImage(1,xl,xu,"Point 2D");
mire2DDraw(p_m,dot_size,3);
show_pixmap()


