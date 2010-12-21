//--------------------------------------------------//
// main program
// test the functions defined in 
// src/asserVisu/predictiveControl.sci
//
// author Claire Dune
// date 04/01/2010
// ;exec('testAsserVisuPredictif_ga.sce');
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

////-------------case 1 Allibert-------------------------// //
posecDesMo_m= [0 0 0.5 0 0 0 ];                  // pose target/object desired
posecMo_m   = [0 0 1   0 0 0 ];                  // pose target/object init  
posewMo_m   = [0 0 0 0 %pi 0 ];                  // pose of the target in the World Frame
Np_m        = 1;                                 // horizon lenght
Nc_m        = 1;                                 // command horizon length  
v_m         = [0 0 0 0 0 0]';                    // init velocity

// ------ Constraints definition
xu_m        = [  0.22 ;  0.22 ];                 // position max of the a 2D point in the image plane 
xl_m        = [ -0.22 ;  -0.22 ];                // position min of the a 2D point in the image plane 
bu_m        = 1e3*[0.25,0.25,0.25,0.25,0.25,0.25]';  // command bounds
bl_m        = -bu_m;                             // command bounds on the horizon
// -------- Sampling time
Te_m        = 1/25;                              // to be consistant with the image frame rate
a_m         = 0.20;                              // dimension of the target  
oP_m        = mire4points (a_m);                 // create the Npbts Points Target
Nbpts_m     = length(oP_m)/3 ;
thres_m     = 1e-2;     // error threshold  
lambda      = 1; 
//----------LFunction and Q definition
Lfunction    = matIntMireC;                       // Lc(t) classical visual servo s(t) Z(t)
//Lfunction    = matIntMireP;                     // Lp(t) classical visual servo s(t) Z*
//Lfunction    = matIntMireM;                     // Lm(t) mixte (L*+Lc(t))
//Lfunction    = matIntMireD;                     // Ld    classical interaction matrix desired
Q_m          = matWeightIdentity(Np_m,Nbpts_m);
//Q_m          = matWeightIdentityZero(Np_m,Nbpts_m,1);
//Q_m          = matWeightTV(Np,Nbpts);

// -------- OPT
OPT_DISPLAY = %T;
OPT_CONTROL = 'AV';
//OPT_CONTROL = 'PRED';

////--------------------------------------------//

//---------------------------------------------------------------------//
//         Create the object
//---------------------------------------------------------------------//
  
wMo_m       = homogeneousMatrixFromPos(posewMo_m);
wP_m        = changeFrameMire(oP_m,wMo_m);       // the Points in the World frame

//---------------------------------------------------------------------//
//         Create the cameras
//---------------------------------------------------------------------//
// ------ First Camera Object Position
cMo_m       = (homogeneousMatrixFromPos(posecMo_m));// pose target/object init   
wMc_m       = wMo_m*inv(cMo_m);                  // pose of the camera in the world frame
// compute the init projection on the view
cP_m        = changeFrameMire(oP_m,cMo_m);       // target Points in the camera frame
s_m         = projectMireDirect(cP_m);           // projection of the target points in the image plane 
Z_m         = cP_m(3:3:length(cP_m)) ;           // depth of the target points in the camera frame

//------- Desired Camera Object Position 
cDesMo_m     = (homogeneousMatrixFromPos(posecDesMo_m));
wMcDes_m     = wMo_m*inv(cDesMo_m);
// compute the desired projection on the view
cDesP_m      = changeFrameMire(oP_m,cDesMo_m);   // desired target Points in the camera frame
sDes_m       = projectMireDirect(cDesP_m);       // desired target Points projection
ZDes_m       = cDesP_m(3:3:length(cDesP_m)) ;    // desired depth

//---------------------------------------------------------------------//
//          Global Variable Control settings
//----------------------------------------------------------------------//
e0_m         = zeros(Nbpts_m*2,1);
defineGlobalVariable(Nc_m,Np_m,Nbpts_m,Te_m,sDes_m,ZDes_m,s_m,Z_m,Q_m,e0_m,xl_m,xu_m,bl_m,bu_m,Lfunction);

// ----- Control param
U0_m         = [];                                  // create the first control horizon
for i = 1:Nc_m
  U0_m       = [U0_m ; v_m];
end ;

//----------------------------------------------------------------------//
//                         Displays                                      //
//----------------------------------------------------------------------//
// create the image plane
if(OPT_DISPLAY) 
  cote_m      = 0.01;                                // display size of the dots
  hf2d1_m   = createPlanImage(1,xl_m,xu_m,"Point 2D");
  mire2DDraw(s_m,cote_m,3);                        // display the current points                     
  show_pixmap()
  mire2DDraw(sDes_m,cote_m,5);                     // display the desired points
  show_pixmap()

  // create the 3D view
  hf3d1_m   = createFigure3D(2,"Camera Motion",1);
  Camera3DDrawColor(0.1,wMc_m,3);                  // display the current camera  
  Camera3DDrawColor(0.1,wMcDes_m ,5);              // display the desired camera
  Mire3DDraw4pts(wP_m);                            // display the target
  show_pixmap()
  disp('Red : desired pose, Green : init pose and target');
  //create a windows to display the top view
  hf2d2_m   = createFigure2D(3,"TopView");
  // create a window to display the error
  hf2d3_m   = createFigure2D(4,"Error");
  //create a windows to display the top view
  hf2d4_m        = createFigure2D(5,"Velocity");
  //create a windows for the prediction
  hf2d5_m        = createPlanImage(6,xl_m,xu_m,"Prediction Mire");
  //create a windows to display the top view
  hf2d6_m        = createFigure2D(7,"Prediction Velocity");
end

//-----------------------------------------------------//
//                   launch the servo
//------------------------------------------------------//
disp('-------------------- servo loop------------')
halt

er_m          = 10;                                // error value init
iter_m        = 0;                                 // number of the iteration
pt2D_m        = s_m';                              // store the real points for display
norme_m       = [];
velocity_m    = [];

while(er_m > thres_m & iter_m < 100 )
  iter_m      = iter_m+1;
  disp('---------------------------------')
  disp(iter_m)
  
  if(OPT_CONTROL == 'PRED')
    tic;
    // compute the best control on the Time horizon
    // vcam is a vector of Np velocities
    // sm is a vector of Np features / Np*Nbpts 2d points
    [U_m,smhor_m,Uhor_m] =predControlLocalMire(U0_m,s_m,Z_m,sDes_m,ZDes_m,e0_m);
    disp('time')
    toc
    v_m        = U_m(1:6);                        // apply only the first velocity
    
    U0_m       = U_m; 
  elseif(OPT_CONTROL == 'AV')
    L_m        = Lfunction (s_m,Z_m);
    v_m        = computeVelocity(lambda, L_m,s_m-sDes_m);
    v_m        = v_m';
    sm_m       = s_m;
  end
  velocity_m =[velocity_m ;v_m'];

  // ----- Figure -------------//
  if(OPT_DISPLAY) 

 
    xset("window",1);           // image plane
    mire2DDraw(s_m,cote_m,3);       // current projection
    show_pixmap()
    mire2DDraw(sDes_m,cote_m,5);   // desired projection 
    show_pixmap()
    mire2DDraw(sm_m,cote_m,4);//model projection
    show_pixmap()
    pt2D_m     = [pt2D_m  ; s_m'];
    if(size(pt2D_m,1)>1)
      for l=1:Nbpts_m
        xset("color",l)
        xpoly(pt2D_m(:,(l-1)*2+1),pt2D_m(:,(l-1)*2+2),"lines",0)
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

    if(OPT_CONTROL == 'PRED')
      hf2d5_m  = createPlanImage(6,xl_m,xu_m,"Prediction Mire");
      //xset("window",6);
      mire2DDraw(s_m,cote_m,3);                     // current projection
      show_pixmap()
      mire2DDraw(sDes_m-e0_m,cote_m,5);             // desired projection 
      show_pixmap()
      mire2DDraw(sm_m(1:Nbpts_m*2),cote_m,4);       //model projection
      show_pixmap()
      smvisu_m =[s_m;smhor_m];
      mireEvolutionDraw(Np_m+1,smvisu_m,1);
      show_pixmap()
      
      hf2d6_m   = createFigure2D(7,"Prediction Velocity");
      previewVelocity_m = [];
      for index=1:Np_m
        previewVelocity_m = [previewVelocity_m ;...
                  Uhor_m((index-1)*6+1:(index-1)*6+6)' ];
      end
      plot2d(previewVelocity_m);
      show_pixmap()
    end
   end

  //------ Update data for next run
  c1Mc2_m    = computeMotion(v_m',Te_m);           // resulting motion
  cMo_m      = inv(c1Mc2_m)*cMo_m;
  wMc_m      = wMc_m* c1Mc2_m;  
  cP_m       = changeFrameMire(oP_m,cMo_m); 
  sm_m       = ga_predLoc2dMire(s_m,v_m,Te_m,Z_m,Lfunction);
  Z_m        = cP_m(3:3:length(cP_m)) ;            // init depth for the step 
  s_m        = projectMireDirect(cP_m);
  e0_m       = sm_m-s_m               // model error
    
  //------ termination test----//
  er_m       = norm(s_m-sDes_m);
  norme_m    = [norme_m er_m];



   disp('---------------------------------')
end

disp('------ the end -------')
disp('Enter to exit')
halt
if(OPT_DISPLAY) 
  xset("pixmap",0);
  delete(hf3d1_m);  
  delete(hf2d1_m);
  delete(hf2d2_m);
  delete(hf2d3_m);  
  delete(hf2d4_m);
  delete(hf2d5_m);
  delete(hf2d6_m);
end










