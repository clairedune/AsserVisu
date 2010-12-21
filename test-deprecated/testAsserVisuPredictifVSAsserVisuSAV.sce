//--------------------------------------------------//
// main program
// test the functions defined in 
// src/asserVisu/predictiveControl.sci
//
// author Claire Dune
// date 20/01/2010
// ;exec('testAsserVisuPredictifVSAsserVisu.sce');
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

//------------------------------------------------------------------//
// ASSER VISU L=Lcurrent(s_m(t),Z_m(t))
//-------------------------------------------------------------------//
posecDesMo_m  = [0 0 0.5 0 0 0 ];
posecMoInit_m = [0 0 0.5 0 0 -%pi/2 ];
//-------------------------------//
Np_m          = 20;              // horizon lenght
Nc_m          = 1;               // command horizon length  
v0_m          = [0 0 0 0 0 0]';  // init velocity
//------ OPT ------//
OPTZVARYING_m     = %T;          // if true then Z = Z* else Z=Z(t)
OPTPREDICTIF_m    = %T;          // if true then control predictif, else asservisu classique 
OPTNORMVELOCITY_m = %T;          // normalise velocity or not    
OPTLCASE_m     = 'CURRENT';      // current interaction matrix Lc = L(s_m(t),Z_m(t)) or Lp = L(s_m(t),Z*)
//OPTLCASE_m  = 'DESIRED';       // desired                    Ld = L* = L(s*,z*)
//OPTLCASE_m  = 'MEAN';          // mean                       Lm = 1/2(Ld+Lc)

path_exp = '/home/dune/Documents/Resultats/100120-TestAsserVisu/16h00';
name_exp = 'PIsur2_pred_Np_20Nc_1_Lc';
save_path = path_exp+'/'+name_exp;
disp('SAVE IMAGE IN------>')
disp (save_path);
halt
//------------------------------------------------------------------//


// ------ Constraints definition
xu_m      = [  1.22 ;  1.22 ];                     // position max of the a 2D point in the image plane 
xl_m      = [ -1.22 ; -1.22 ];                     // position min of the a 2D point in the image plane 
bu_m      = 1e3*[0.25,0.25,0.25,0.25,0.25,0.25]';                // command bounds
bl_m      = -1e3*[0.25,0.25,0.25,0.25,0.25,0.25]';// command bounds on the horizon
//
lambda    = 0.6;

////--------------------------------------------//



// all the variable defined in the main are followed by main
Te_m       = 0.4; // to be consistant with the image frame rate

//------- Create the object
// create the target
size_m        = 0.20; // related to the target size 
oP_m       = mire4points (size_m); 
posewMo_m = [0 0 0 0 %pi 0 ];
wMo_m      = homogeneousMatrixFromPos(posewMo_m);
wP_m       = changeFrameMire(oP_m,wMo_m); // the dots in the fixed frame

// ------ First Camera Object Position
cInitMo_m  = (homogeneousMatrixFromPos(posecMoInit_m));
//------- First Camera pose
wMcInit_m  = wMo_m*inv(cInitMo_m); 

// compute the first projection on the view
cInitP_m   = changeFrameMire(oP_m,cInitMo_m);
s_m        = projectMireDirect(cInitP_m);

//------- Desired Camera Object Position 

cDesMo_m     = (homogeneousMatrixFromPos(posecDesMo_m));
wMcDes_m     = wMo_m*inv(cDesMo_m);
// compute the desired projection on the view
cDesP_m      = changeFrameMire(oP_m,cDesMo_m); 
sDes_m       = projectMireDirect(cDesP_m);



//---------------------------------------------------------------//
//             Optimisation method:
//         Mire Local with arbitrary Z   
//---------------------------------------------------------------//
// ----- Target param
Nbpts_m   = length(s_m)/2;                       // number of points of the target 5
ZDes_m    = cDesP_m(3:3:length(cDesP_m)) ;
LDes_m    = matIntMire6ddl(sDes_m,ZDes_m); 


if(OPTZVARYING_m)
  Z_m       = cInitP_m(3:3:length(cInitP_m)) ;      // init depth for the step 
else
  Z_m       = ZDes_m;
end  

// ----- Control param
U0_m      = [];                                  // create the first control horizon
for i = 1:Nc_m
  U0_m    = [U0_m ; v0_m];
end ;
sm0_m     = s_m;                                 // modelised pose the first projection is p
e0_m      = sm0_m - s_m;                         // the first error is e0
Q_m       = eye(Nbpts_m*2,Nbpts_m*2);              // the weight matrix 
sStar_m   = sDes_m;                              // desired features

// ----- Init camera position for the simulation
wMc_m     = wMcInit_m;                           // first camera position

defineGlobalVariable(Nc_m,Np_m,Nbpts_m,Te_m,Z_m,Q_m,e0_m,sm0_m,sStar_m,xl_m,xu_m,bl_m,bu_m,OPTLCASE_m,LDes_m );

//---------------------Displays -----------------------//

// create the image plane
cote_m    = 0.01;                                // display size of the dots
hf2d1_m   = createPlanImage(1,xl_m,xu_m,"Point 2D");
mire2DDraw(s_m,cote_m,3);                        // display the current points                     
show_pixmap()
mire2DDraw(sDes_m,cote_m,5);                     // display the desired points
show_pixmap()

// create the 3D view
hf3d1_m   = createFigure3D(2,"Camera Motion",0.5);
Camera3DDrawColor(0.1,wMcInit_m,3);              // display the current camera
Camera3DDrawColor(0.1,wMcDes_m ,5);              // display the desired camera
Mire3DDraw4pts(wP_m);                            // display the target
show_pixmap()
disp('Red : desired pose, Green : init pose and target');

//create a windows to display the top view
hf2d2_m       = createFigure2D(3,"TopView");

// create a window to display the error
norme_m       =[];
hf2d3_m       = createFigure2D(4,"Error");

//create a windows to display the top view
velocity_m    =[];
hf2d4_m       = createFigure2D(5,"Velocity");

//create a windows for the prediction
hf2d5_m       = createPlanImage(6,xl_m,xu_m,"Prediction Mire");

//create a windows to display the top view
hf2d6_m       = createFigure2D(7,"Prediction Velocity");


//-----------------------------------------------------//
//                   launch the servo
//------------------------------------------------------//
disp('-------------------- servo loop------------')
halt

er_m          = 10;       // error value init
iter_m        = 0;        // number of the iteration
thres_m       = 1e-2;     // error threshold  

pt2D_m        = s_m';     // store the real points for display
pt2Dm_m       = sm0_m';   // store the the modelised points for the display
sm_m          = sm0_m;

error_m       = s_m-sDes_m;

while(er_m > thres_m & iter_m < 40 )
  iter_m      = iter_m+1;
  disp('---------------------------------')
  disp(iter_m)
 

  if(OPTPREDICTIF_m)
    tic;
    // compute the best control on the Time horizon
    // vcam is a vector of Np velocities
    // sm is a vector of Np features / Np*Nbpts 2d points
    [U_m,sm_m,Uhor_m] = predControlLocalMire(U0_m,sm0_m,e0_m,sStar_m,Z_m,Q_m);
     U0_m             = U_m; 
    disp('time')
    toc
    v_m               = U_m(1:6);              // apply only the first velocity
  else
    L_m = matIntMire6ddlCase(s_m,Z_m,OPTLCASE_m,LDes_m);
    // desired velocity of the camera
    v_m = computeVelocity(lambda, L_m,error_m);
    v_m = v_m';
  end;
  
  if(OPTNORMVELOCITY_m)
     v_m = normalizeU(v_m,0.25);
  end
  
  // ----- Figure -------------//
  hf2d5_m   = createPlanImage(6,xl_m,xu_m,"Prediction Mire");
  //xset("window",6);
  mire2DDraw(s_m,cote_m,3);       // current projection
  show_pixmap()
  mire2DDraw(sStar_m-e0_m,cote_m,5);   // desired projection 
  show_pixmap()
  mire2DDraw(sm_m(1:Nbpts_m*2),cote_m,4);//model projection
  show_pixmap()
  smvisu_m=[sm0_m;sm_m];
  mireEvolutionDraw(Np_m+1,smvisu_m,1);
  show_pixmap()

  //------ Update data for next run
  velocity_m =[velocity_m ;v_m'];
 
  c1Mc2_m    = computeMotion(v_m',Te_m); // resulting motion
  wMc_m      = wMc_m*c1Mc2_m; 
  cMo_m      = inv(wMc_m)*wMo_m;
  cP_m       = changeFrameMire(oP_m,cMo_m); 
  if(OPTZVARYING_m)
      Z_m       = cP_m(3:3:length(cP_m)) ;      // init depth for the step 
  else
      Z_m        = ZDes_m;
  end  
  s_m        = projectMireDirect(cP_m);
  e0_m       = sm_m(1:Nbpts_m*2)-s_m ;      // the first error is e0
  sm0_m      = s_m;
 
  
  
  //------ termination test----//
  error_m    = s_m-sDes_m;
  er_m       = norm(error_m);
  norme_m    = [norme_m er_m];
  
  // ----- Figure -------------//
  xset("window",1);           // image plane
  mire2DDraw(s_m,cote_m,3);       // current projection
  show_pixmap()
  mire2DDraw(sStar_m,cote_m,5);   // desired projection 
  show_pixmap()
  mire2DDraw(sm_m(1:Nbpts_m*2),cote_m,4);//model projection
  show_pixmap()
  pt2D_m  = [pt2D_m  ; s_m'];
  //pt2Dm_m = [pt2Dm_m ; sm_m(1:Nbpts_m*2)'];
  if(size(pt2D_m,1)>1)
   for l=1:Nbpts_m
     xset("color",l)
    // xpoly(pt2D_m(:,(l-1)*2+1),pt2D_m(:,(l-1)*2+2),"lines",0)
     plot2d(pt2D_m(:,(l-1)*2+1),pt2D_m(:,(l-1)*2+2))
    // xset("color",2)
    // xpoly(point2Dmodel(:,(l-1)*2+1),point2Dmodel(:,(l-1)*2+2),"lines",0)
     show_pixmap()
   
   end
  end

  xset("window",2);          // 3D view
  Camera3DDraw(0.1,wMc_m);
  show_pixmap()  
  
  xset("window",3);
  AxeZ2DDraw(1,wMc_m);
  show_pixmap();
  
  xset("window",4);
  plot(thres_m*ones(1,length(norme_m)))
  her=gce();
  her.foreground=5;
  plot(norme_m);
  her=gce();
  her.foreground=3;
  show_pixmap();
 
  if(size(velocity_m,1)>1)
    xset("window",5);
    plot2d(velocity_m);
    show_pixmap()
  end

  if(OPTPREDICTIF_m)
    hf2d6_m   = createFigure2D(7,"Prediction Velocity");
    previewVelocity_m = [];
    for index=1:Np_m
      previewVelocity_m = [previewVelocity_m ;...
                  Uhor_m((index-1)*6+1:(index-1)*6+6)' ];
    end
    disp(previewVelocity_m)
    plot2d(previewVelocity_m);
    show_pixmap()
  end

 disp('---------------------------------')
end

disp('------ the end -------')
disp('Enter to exit')
halt


xs2fig(1,save_path+'-point2D.fig')
xs2fig(2,save_path+'-cameraPosition.fig')
xs2fig(3,save_path+'-topview.fig')
xs2fig(4,save_path+'-error.fig')
xs2fig(5,save_path+'-velocity.fig')



disp('SAVED IMAGE IN')

xset("pixmap",0);
delete(hf3d1_m);
delete(hf2d1_m);
delete(hf2d2_m);
delete(hf2d3_m);
delete(hf2d4_m);
delete(hf2d5_m);











