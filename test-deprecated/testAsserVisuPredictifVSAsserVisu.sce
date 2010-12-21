//--------------------------------------------------//
// main program
// test the functions defined in 
// src/asserVisu/predictiveControl.sci
//
// author Claire Dune
// date 04/01/2010
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

////-------------case 1 Allibert-------------------------// //
posecDesMo_m= [0   0   0.5 0 0 0 ];              // pose target/object desired
posecMo_m   = [0.1   0.1   1 0 0 0 ];              // pose target/object init  
posewMo_m   = [0   0   0 0 %pi 0 ];              // pose of the target in the World Frame
Np_m        = 1;                                // horizon lenght
Nc_m        = 1;                                 // command horizon length  
v_m         = [0 0 0 0 0 0]';                    // init velocity

// ------ Constraints definition
xu_m        = [  0.22 ;  0.22 ];                 // position max of the a 2D point in the image plane 
xl_m        = [ -0.22 ;  -0.22 ];                // position min of the a 2D point in the image plane 
bu_m        = 1e3*0.25*ones(6,1);  // command bounds
bl_m        = -bu_m;                             // command bounds on the horizon
// -------- Sampling time
Te_m        = 1/25;                              // to be consistant with the image frame rate
a_m         = 0.20;                              // dimension of the target  
oP_m        = mire4points (a_m);                 // create the Npbts Points Target
Nbpts_m     = length(oP_m)/3 ;
thres_m     = 1e-3;     // error threshold  
lambda      = 0.1; 
//----------LFunction and Q definition
Lfunction    = matIntMireC;                       // Lc(t) classical visual servo s(t) Z(t)
//Lfunction    = matIntMireP;                     // Lp(t) classical visual servo s(t) Z*
//Lfunction    = matIntMireM;                     // Lm(t) mixte (L*+Lc(t))
//Lfunction    = matIntMireD;                     // Ld    classical interaction matrix desired

Q_m          = matWeightIdentity(Np_m,Nbpts_m);
//Q_m          = matWeightIdentityZero(Np_m,Nbpts_m,1);
//Q_m          = matWeightTV(Np_m,Nbpts_m);

//funcost_m    = cld_costLocalMire;
funcost_m    = ga_costLocalMire;
funcst_m     = ga_constraintsLocalMire;
jaccost_m    = "grobfd";
jaccst_m     = "grcnfd";
   
// -------- OP
OPT_DISPLAY = %T;
OPT_SAVE    = %F;
OPT_CONTROL = 'AV';
//OPT_CONTROL = 'PRED';
path_exp = '/home/dune/Documents/Resultats/100125-TestAsserVisu/';
name_exp = 'PRED-sansCsrt';
save_path = path_exp+'/'+name_exp;



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
wMc_m       = wMo_m*inv(cMo_m) ;                 // pose of the camera in the world frame
wMc_m       = wMc_m.*(abs(wMc_m)>1e-10);
// compute the init projection on the view
cP_m        = changeFrameMire(oP_m,cMo_m);       // target Points in the camera frame
s_m         = projectMireDirect(cP_m);           // projection of the target points in the image plane 
Z_m         = cP_m(3:3:length(cP_m)) ;           // depth of the target points in the camera frame

disp(s_m)
halt

//------- Desired Camera Object Position 
cDesMo_m     = (homogeneousMatrixFromPos(posecDesMo_m));
wMcDes_m     = wMo_m*inv(cDesMo_m);
wMcDes_m       = wMcDes_m.*(abs(wMcDes_m)>1e-10);
// compute the desired projection on the view
cDesP_m      = changeFrameMire(oP_m,cDesMo_m);   // desired target Points in the camera frame
sDes_m       = projectMireDirect(cDesP_m);       // desired target Points projection
ZDes_m       = cDesP_m(3:3:length(cDesP_m)) ;    // desired depth

//---------------------------------------------------------------------//
//          Global Variable Control settings
//----------------------------------------------------------------------//
e0_m         = zeros(Nbpts_m*2,1);
defineGlobalVariable(s_m,Z_m,Nc_m,Np_m,Nbpts_m,Te_m,sDes_m,ZDes_m,Q_m,e0_m,xl_m,xu_m,bl_m,bu_m,Lfunction);
// ----- Control param
U0_m         = [];                              // create the first control horizon
for i = 1:Nc_m
  U0_m       = [U0_m ; v_m];
end ;

//----------------------------------------------------------------------//
//                         Displays                                      //
//----------------------------------------------------------------------//
// create the image plane
if(OPT_DISPLAY) 
  cote_m      = 0.01;                                // display size of the dots
  hf2d1_m     = createPlanImage(1,xl_m,xu_m,"Point 2D");
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
  hf2d4_m   = createFigure2D(5,"Velocity");
  //create a windows for the prediction
  hf2d5_m   = createPlanImage(6,xl_m,xu_m,"Prediction Mire");
  //create a windows to display the top view
  hf2d6_m   = createFigure2D(7,"Prediction Velocity");
  //create a windows to display the top view
  hf2d7_m   = createFigure2D(8,"Erreur en Position ");
end

//-----------------------------------------------------//
//                   launch the servo
//------------------------------------------------------//
disp('-------------------- servo loop------------')
halt
err          = 10;                                // error value init
iter_m        = 0;                                 // number of the iteration
pt2D_m        = s_m';                              // store the real points for display
norme_m       = [];
velocity_m    = [];
sm_m          = s_m;
position_m    = [];
while(err > thres_m & iter_m < 1000 )
  iter_m      = iter_m+1;
  disp('---------------------------------')
  disp(iter_m)
  //-------------------------------------------------------//
  //  Compute the velocity using predictive control        //
  //-------------------------------------------------------//
  if(OPT_CONTROL == 'PRED')
    tic;
    // compute the best control on the Time horizon
    // vcam is a vector of Np velocities
    // sm is a vector of Np features / Np*Nbpts 2d points
    [U_m,smhor_m,Uhor_m] =predControlLocalMire(s_m,Z_m,U0_m,sDes_m,ZDes_m,e0_m,funcost_m,funcst_m,jaccost_m,jaccst_m);
    
    disp('time')
    toc
    v_m        = U_m(1:6);                        // apply only the first velocity
    U0_m       = U_m; 
    cost = ga_costHorLoc2dMire(s_m,Z_m,Uhor_m,Te_m,Lfunction,Np_m,Q_m,e0_m,sDes_m);
  //--------------------------------------------------------//
  //  Compute the velocity using classical visual servoing  //
  //--------------------------------------------------------//
  elseif(OPT_CONTROL == 'AV')
    L_m        = Lfunction (s_m,Z_m);
    v_m        = computeVelocity(lambda, L_m,s_m-sDes_m);
    v_m        = v_m';
    sm_m       = s_m;
    cost       = norm(s_m-sDes_m);
  end
  v_m = normalizeU(v_m,0.25);
  //v_m = v_m.*(abs(v_m)>1e-08);
  velocity_m =[velocity_m ;v_m'];
  

  //--------------------------------------------------------//
  //                    Displays                            //
  //--------------------------------------------------------//
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
    plot(thres_m*ones(1,iter_m),'r')
    show_pixmap();
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
    if(size(position_m,1)>1)
      xset("window",8);
      plot2d(position_m);
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
  
  //--------------------------------------------------------//
  //         Update data for next iter                      //
  //--------------------------------------------------------//
  c1Mc2_m    = computeMotion(v_m',Te_m)  ;         // resulting motion
  cMo_m      = inv(c1Mc2_m)*cMo_m;
  wMc_m      = wMc_m* c1Mc2_m  ;
  cP_m       = changeFrameMire(oP_m,cMo_m); 
  sm_m       = ga_predLoc2dMire(s_m,Z_m,v_m,Te_m,Lfunction);
  Z_m        = cP_m(3:3:length(cP_m)) ;            // init depth for the step 
  s_m        = projectMireDirect(cP_m);
  e0_m       = sm_m-s_m    ;                       // model error
  err        = norm(s_m-sDes_m);
  
  cDesMc_m   = inv(wMcDes_m)*wMc_m;
  p = pFromHomogeneousMatrix(cDesMc_m);
  position_m = [position_m;p'];
  //------ termination test----//
  norme_m    = [norme_m err];
  //halt
  disp('---------------------------------')
end

disp('------ the end -------')
disp('Enter to exit')
halt

if(OPT_SAVE & OPT_DISPLAY)
xs2fig(1,save_path+'-point2D.fig')
xs2fig(2,save_path+'-cameraPosition.fig')
xs2fig(3,save_path+'-topview.fig')
xs2fig(4,save_path+'-error.fig')
xs2fig(5,save_path+'-velocity.fig')
xs2fig(8,save_path+'-errPosision.fig')
disp('IMAGES SAVED')
end
if(OPT_DISPLAY) 
  xset("pixmap",0);
  delete(hf3d1_m);  
  delete(hf2d1_m);
  delete(hf2d2_m);
  delete(hf2d3_m);  
  delete(hf2d4_m);
  delete(hf2d5_m);
  delete(hf2d6_m);
  delete(hf2d7_m);
end













