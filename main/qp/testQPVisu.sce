// Test QP visu without walking
//--------------------------------------------------//

//clear;
path=get_absolute_file_path("scilab-src");
  disp('HOME:'+path),
  getd(path + "src/graphisme"); // pour charger un repertoire en entier
  getd(path + "src/transformation");  
  getd(path + "src/projectionPers");
  getd(path + "src/asserVisu");
  getd(path + "src/hrp2");
  getd(path + "src/optimisation");
  getd(path + "src/tools");
  getd(path);

  disp('')
  disp('------ Test Linear vision solving -------')
  disp('')

  Te_m          = 0.1  ;                       // to be consistant with the image frame rate
  Te_simu       = 1/25;
 
  funcost_in    = costGlobalMireJerk;          // use the global cost computation  
  Lfunction     = matIntMireC;;                //-------- LFunction 
  funcost_m     = funcost_in;                  // ----- Cost and function
  

  ndof_m        = 2;                           // X and Y are free theta constrained
  Np_m          = 5;                           // Horizon length
  Nc_m          = Np_m;
  name_in       ='test';

  //----------------------------------------------------------------//
  //  problem Statement 
  // A fixed Object and a mobile camera
  //----------------------------------------------------------------//
 
 // ------ init pose
  pose          = [ 0 0 0 -%pi 0 0]';  // CoM position in the world frame
  posecMo_m     = [ 0 0 1 0 0 0 ];    // current object position in the camera frame
  posecDesMo_m  = [ 0 0 0.9 0 0 0 ];    // desired object position in the camera frame
  // init state
  stateCoM_m   = zeros(3*2,1);                
  stateCoM_m(1)= pose(1);
  stateCoM_m(2)= pose(2);
  //stateCoM_m(3)= pose(6);
  // ------ init jerk
  jerk_m        = 0.0*ones(Np_m,1);
  for i=1:ndof_m-1
      jerk_m    = [jerk_m; zeros(Np_m,1)];
  end   
  // ------ Constraints definition
  xu_m          = [0.4 ; 0.4 ];                  // position max of the a 2D point in the image plane 
  xl_m          = [-0.4 ; -0.4 ];                // position min of the a 2D point in the image plane 
  bu_m          = 1e9*ones(Np_m*ndof_m,1);        // command bounds, jerk bounds
  bl_m          = -bu_m;                         // command bounds on the horizon, jerk bounds
  // ------- Target building
  a_m           = 0.1;                           // dimension of the target  
  oP_m          = mire5points (a_m);             // create the Npbts Points Target
  Nbpts_m       = length(oP_m)/3 ;               // number of points of the target
  // ------ Homogeneous Poses
  wMr_m         = homogeneousMatrixFromPos(pose);
  [rMc_m,rVc_m] = loadHRP2camCom();
  wMc_m         = wMr_m*rMc_m;
  cMo_m         = homogeneousMatrixFromPos(posecMo_m); // pose target/object init   
  wMo_m         = wMc_m*cMo_m;
  //------ compute the init projection on the view
  wP_m          = changeFrameMire(oP_m,wMo_m); 
  cP_m          = changeFrameMire(oP_m,cMo_m);   // target Points in the camera frame
  s_m           = projectMireDirect(cP_m);       // projection of the target points  
  Z_m           = cP_m(3:3:$) ;                  // depth of the target points in the camera frame
  //------- Desired Camera Object Position 
  cDesMo_m     = (homogeneousMatrixFromPos(posecDesMo_m));
  wMcDes_m     = wMo_m*inv(cDesMo_m);
  wMcDes_m     = wMcDes_m.*(abs(wMcDes_m)>1e-10);
  //------- compute the desired projection on the view
  cDesP_m      = changeFrameMire(oP_m,cDesMo_m); // desired target Points in the camera frame
  sDes_m       = projectMireDirect(cDesP_m);     // desired target Points projection
  ZDes_m       = cDesP_m(3:3:length(cDesP_m)) ;  // desired depth

  halt

  cote_m=0.1;
 //-----------------
 // QP construction
 //------------------

 defineGlobalVariableJerk(...
  s_m,...
  Z_m,...
  stateCoM_m(1:6),...
  rMc_m,...
  Np_m,...
  Nbpts_m,...
  ndof_m,...
  Te_m,...
  sDes_m,...
  ZDes_m,...
  Lfunction);

 global Sp_global;
 global Up_global;
 global Sv_global;
 global Uv_global;  
 global Sa_global; 
 global Ua_global;
 global computeL_global ;
 global jerkPrec;
 global  Sdes_global ; 

 maxiter  = 150;

 wVr_m    = twistMatrix(wMr_m);
 wMcfirst = wMc_m;
 posec    = pFromHomogeneousMatrix(wMc_m);
 poser    = pFromHomogeneousMatrix(wMr_m);
 Xc       = [posec(1)];
 Yc       = [posec(2)]; 
 Zc       = [posec(3)];
 Xr       = [poser(1)];
 Yr       = [poser(2)];
 Zr       = [poser(3)]; 


 k=0;
 maxiter  = 150;

 h2d        = createFigure2D(3,'prediction');
 while(k<maxiter)
  
  pause
  [Sk, Z_out ] = predHorGlobalMireJerk(s_m, ...
                                       Z_m, ...
                                       stateCoM_m(1:6),...                
                                       rMc_m,...
                                       jerk_m,..
                                       Sp_global,...
                                       Up_global,...
                                       Sv_global,... 
                                       Uv_global,...
                                       Sa_global,...
                                       Ua_global,...
                                       Te_m,...
                                       Np_m,...
                                       ndof_m);
   L_out          = predIntMatGlobalJerk(computeL_global,Sk,Z_out,Np_m,Nbpts_m);
   Lbig           = bigPredBigMatrixGlobal(L_out,Np_m, Nbpts_m);
   me = 0;
  
   [Qvisu pTvisu]= computeQPvisuDelta(Sk,Sdes_global, Lbig);
   
   Qvisu=Qvisu+1e-2*eye(Np_m*ndof_m,Np_m*ndof_m);
   [deltaJerk, lagr, info] = qld(Qvisu, pTvisu, [], [], bl_m, bu_m, me, 1e-13);
    
   disp('deltajerk')
   deltaJerk
   jerk_m = jerk_m+deltaJerk

   costQP = (Sk-Sdes_global)'*(Sk-Sdes_global)+pTvisu*deltaJerk+deltaJerk'*Qvisu*deltaJerk
 

   [P,Pstack,V,Vstack,A,Astack] = stateFromJerkHorizon(stateCoM_m(1:6),jerk_m,Sp_global,Up_global,Sv_global,Uv_global,Sa_global,Ua_global);
   [p, v] = stateCamFromComHor(rMc_m,P,V,ndof_m,Np_m);
   stateCoM_m = [P(1:ndof_m);V(1:ndof_m);A(1:ndof_m)]  
   //update the value for the servoing
   k = k + 1;
   //vrobotRealW = RobotReal([2;5;8]);
   vrobotRealW = [V(1:2);0];
   vrobotReal  = (inv(wVr_m)*[vrobotRealW(1) vrobotRealW(2) 0 0 0 vrobotRealW(3)]')'; 
   vrobotReal  = vrobotReal.*(abs(vrobotReal) > 1e-6);
   //r1Mr2       = expMapDirectRxRyRz(vrobotReal,Te_m);
   poseRobot   = [P(1), P(2), 0 ,-%pi ,0,P(3)];  
   wMr_m       = homogeneousMatrixFromPos(poseRobot)//wMr_m*r1Mr2;
   wMr_m       = wMr_m.*(abs(wMr_m)>1e-10);
   disp('Vrobot vrobotMonde/vrobotreal')
   disp(vrobotRealW');
   disp(vrobotReal);
   // deduce vcamReal   
   vcamReal   = convertVelocityRobotCam(vrobotReal,rVc_m) ;
   vcamReal   = [vcamReal(1),0,vcamReal(2),0,vcamReal(3),0 ]; 
   vcamReal   = vcamReal.*(abs(vcamReal) > 1e-6);
   disp(vcamReal([1,3,5]));

   // the resulting camera motion is then  
   wMc_m = wMr_m*rMc_m;
   //c1Mc2      = expMapDirectRxRyRz(vcamReal,Te_m);
   //cMo_m      = inv(c1Mc2)*cMo_m;
   cMo_m = inv(wMc_m)*wMo_m;
   //cMo_m      = cMo_m.*(abs(cMo_m)>1e-10);
   //wMc_m      = wMo_m*inv(cMo_m);
   //wMc_m      = wMc_m.*(abs(wMc_m)>1e-10);
  
   cP_m       = changeFrameMire(oP_m,cMo_m);                    // compute the 3D points
   s_m        = projectMireDirect(cP_m);                      // compute the 2D points
   Z_m        = cP_m(3:3:$);                         // compute Z
  
   updateVarJerk(s_m,Z_m,stateCoM_m(1:6),sDes_m,ZDes_m);
  

  // --- store and display ---//
  posec      = pFromHomogeneousMatrix(wMc_m);
  poser      = pFromHomogeneousMatrix(wMr_m);
  Xc         = [Xc;posec(1)];
  Yc         = [Yc;posec(2)]; 
  Zc         = [Zc;posec(3)];
  Xr         = [Xr;poser(1)];
  Yr         = [Yr;poser(2)];
  Zr         = [Zr;poser(3)]; 
  hf         = createFigure3D(2,"Camera Motion",2);
  Camera3DDrawColor(1,wMcfirst,3);
  Camera3DDrawColor(1,wMcDes_m,5);
  Mire3DDraw5pts(wP_m);
  if (k>=2)
    plot3d(Xc,Yc,Zc);
    plot3d(Xr,Yr,Zr);
  end
  Camera3DDraw(cote_m,wMc_m);
  show_pixmap()
  
    
  
  xset("window",3);
  mireEvolutionDraw(Np_m,Sk,1);
  mire2DDraw(s_m,0.01,3);       // current projection
  show_pixmap()
  mire2DDraw(sDes_m,0.01,5);   // desired projection 
  show_pixmap()
  h2d2       = createFigure2D(4,'topView');
  plot(p(1:6:$),p(2:6:$))
  
 
   
end


