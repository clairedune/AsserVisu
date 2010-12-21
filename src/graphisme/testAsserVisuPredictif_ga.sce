//--------------------------------------------------//
// main program
// test the functions defined in 
// src/asserVisu/predictiveControl.sci
//
// author Claire Dune
// date 04/01/2010
// ;exec('testAsserVisuPredictif.sce');
//--------------------------------------------------//

clear

DEBUG_VERBOSE = %F;


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
getd(path + 'src/optimisation')
// load optimisation
exec('../HuMAnS/KickStart.sci');
execstr(LoadModule('../HuMAnS/Kernel'));
if ~c_link('libcfsqp') then exec('../HuMAnS/Kernel/OptimizationTools/fsqp-1.2/loader.sce') ; end 

disp('')
disp('------ Test Predictive Control -------')
disp('')

//----------------------------------------------------------------//
//  problem Statement 
// A fixed Object and a mobile camera
//----------------------------------------------------------------//

Te            = 0.4; // to be consistant with the image frame rate

//------- Create the object
// create the target
L             = 0.20; // related to the target size 
oP            = mire4points (L); 

//------- First Camera pose
pose_wMc_init = [0 0 0 90*%pi/180 0 0 ];
wMcinit       = homogeneousMatrixFromPos(pose_wMc_init); 
// ------ First Camera Object Position
pose_cMo_init = [0 0 1 0 0 0 ];
cinitMo       = homogeneousMatrixFromPos(pose_cMo_init);

// the object is fixed, we can compute its position in the world frame
wMo           = wMcinit*cinitMo;
wP            = changeFrameMire(oP,wMo); // the dots in the fixed frame

// compute the first projection on the view
cinitP        = changeFrameMire(oP,cinitMo);
p             = projectMireDirect(cinitP);


//------- Desired Camera Object Position 
pose_cMo_des  = [0.1 0.1 0.8 0 0 0 ];
cdesiredMo    = homogeneousMatrixFromPos(pose_cMo_des);
wMcdes        = wMo*inv(cdesiredMo);
// compute the desired projection on the view
cdesiredP     = changeFrameMire(oP,cdesiredMo); 
pdes          = projectMireDirect(cdesiredP);



//---------------------------------------------------------------//
//             Optimisation method:
//         Mire Local with arbitrary Z   
//---------------------------------------------------------------//

Z       = cinitP(3:3:15)       // init depth for the step 
v0      = [0 0 0 0 0 0]';      // init velocity
Np      = 2;                   // horizon lenght
U0      = [];                  // create the first control horizon
for i = 1:Np, U0 = [U0 ; v0];,end ;
Nbts    = 5 ;                  // number of points of the target 5
xu      = [  0.3 ;  0.2 ];     // position max of the a 2D point in the image plane 
xl      = [ -0.3 ; -0.2 ];     // position min of the a 2D point in the image plane 
sm0     = p;                   // modelised pose
// the first projection is p
e0      = sm0 - p              // the first error is e0
Q       = eye(Nbts*2,Nbts*2);  // the weight matrix 
sStar   = pdes;                // desired features
// control limit
bu1     = [1,1,1,10*%pi/180,10*%pi/180,10*%pi/180]'
bu      = [];
for i=1:Np,  bu = [bu;bu1];,end
bl      = -bu;
// first camera position
wMc     = wMcinit;


//---------------------Displays -----------------------//

// create the image plane
cote          = 0.01; // display size of the dots
hf2d_1        = createPlanImage(1,xl,xu,"Point 2D");
mire2DDraw(p,cote,3);
show_pixmap()
mire2DDraw(pdes,cote,5);
show_pixmap()

// create the 3D view
hf            = createFigure3D(2,"Camera Motion",1);
Camera3DDrawColor(0.1,wMcinit,3);// display the first camera
Camera3DDrawColor(0.1,wMcdes ,5);// display the first camera
Mire3DDraw(wP);
show_pixmap()
disp('Red : desired pose, Green : init pose and target');

//create a windows to display the top view
hf2d_2   = createFigure2D(3,"TopView");

// create a window to display the error
norme    =[];
hf2d_3   = createFigure2D(4,"Error");

//create a windows to display the top view
velocity =[];
hf2d_4   = createFigure2D(5,"Velocity");

//create a windows for the prediction
hf2d_5   = createPlanImage(6,xl,xu,"Prediction Mire");



disp('-------------------- servo loop------------')
halt
//-----------------------------------------------------//
//                   launch the servo
//------------------------------------------------------//

er     = 10;   // error value init
iter   = 0;    // number of the iteration
thres  = 1e-3; //  
while(er>thres)

  iter      = iter+1;
  disp('---------------------------------')
  disp(iter)
  disp('---------------------------------')
  
  
  // compute the best control on the Time horizon
  // vcam is a vector of Np velocities
  // sm is a vector of Np features / Np*Nbpts 2d points
  [vcam,sm] = predControlLocalMire(Np,Nbts,xl,xu,bl,bu,U0,sm0,sStar,e0,Q,Te,Z);
  
  // ----- Figure -------------//
  //hf2d_5   = createPlanImage(6,xl,xu,"Prediction Mire");
  xset("window",6);
  smvisu=[sm0;sm];
  mireEvolutionDraw(Np+1,smvisu,1);
  show_pixmap()

  //------ Update data for next run
  v       = vcam(1:6);              // apply only the first velocity
  velocity =[velocity ;v'];
  c1Mc2   = computeMotion(v',Te); // resulting motion
  // new camera position in the world frame
  wMc     = wMc*c1Mc2; 
  cMo     = inv(wMc)*wMo;
  cP      = changeFrameMire(oP,cMo); 
  Z       = cP(3:3:15) ;
  // targetprojection
  p       = projectMireDirect(cP);
  e0      = sm(1:10)-p ;      // the first error is e0
  sm0     = p;
  U0      = vcam;
  
  //------ termination test----//
  er     = norm(p-pdes);
  norme  = [norme er];
  
  // ----- Figure -------------//
  xset("window",1);           // image plane
  mire2DDraw(p,cote,3);       // current projection
  show_pixmap()
  mire2DDraw(sStar,cote,5);   // desired projection 
  show_pixmap()
  mire2DDraw(sm(1:10),cote,4);//model projection
  show_pixmap()
  
  xset("window",2);          // 3D view
  Camera3DDraw(0.1,wMc);
  show_pixmap()  
  
  xset("window",3);
  AxeZ2DDraw(1,wMc);
  show_pixmap();
  
  xset("window",4);
  plot(thres*ones(1,length(norme)))
  her=gce();
  her.foreground=5;
  plot(norme);
  her=gce();
  her.foreground=3;
  show_pixmap();
 
  if(size(velocity,1)>1)
    xset("window",5);
    plot2d(velocity);
    show_pixmap()
  end

end

disp('------ the end -------')
disp('Enter to exit')
halt
xset("pixmap",0);
delete(hf);
delete(hf2d_1);
delete(hf2d_2);
delete(hf2d_3);
delete(hf2d_4);
delete(hf2d_5);











