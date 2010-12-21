//--------------------------------------------------//
// test the cost computed with different QP
//
//--------------------------------------------------//

  clear;
  exec('Load.sce') 
  disp('')
  disp('------ Test Predictive Control -------')
  disp('')

  // Variables global pour le calcul du cout
  global Q_global ; 
  global Udiag_global; 
  global Vdiag_global;  
  global Eframe_global;  
  global Sdes_global ; 
  global Ewalk_global  ; 
  global Np_global;
  
  ndof_in     = 2;
  Te_in       = 0.1;
  L_in        = matIntMireC;
  stateCoM_in = zeros(3*ndof_in,1);
  funcost_in  = costGlobalMireJerk;
  Np_in       = 4;

  name_in     ='test';
 

  //----------------------------------------------------------------//
  //  problem Statement 
  // A fixed Object and a mobile camera
  //----------------------------------------------------------------//
 
  // ------ Camera Pose
  pose        = [ stateCoM_in(1) stateCoM_in(2) 0 -%pi 0 stateCoM_in(3)]';
  stateCoM_m  = stateCoM_in;
  Np_m        = Np_in;                              // horizon lenght
  Nc_m        = Np_m;
  jerk_m      = [0*ones(Np_m,1);0.6*ones(Np_m,1)];
  ndof_m      = ndof_in;
  for i=1:ndof_m-2
      jerk_m  = [jerk_m; zeros(Np_m,1)];
  end   

  // ------ Constraints definition
  xu_m        = [0.4 ; 0.4 ];                       // position max of the a 2D point in the image plane 
  xl_m        = [-0.4 ; -0.4 ];                     // position min of the a 2D point in the image plane 
  bu_m        = 1e4*0.25*ones(ndof_in,1);           // command bounds
  bl_m        = -bu_m;                              // command bounds on the horizon

  // -------- Sampling time
  Te_m        = Te_in;   // to be consistant with the image frame rate
  Te_simu     = 1/25;
  a_m         = 0.10;                              // dimension of the target  
  oP_m        = mire5points (a_m);                 // create the Npbts Points Target
  Nbpts_m     = length(oP_m)/3 ;
  
  //-------- LFunction 
  Lfunction = L_in;

  // ----- Q definction
  Q_m          = matWeightIdentity(Np_m,Nbpts_m);
  //Q_m          = matWeightIdentityZero(Np_m,Nbpts_m,1);
  //Q_m          = matWeightTV(Np_m,Nbpts_m);

  // ----- Cost and function
  funcost_m   = funcost_in;
  
  // ------ Homogeneous Poses
  wMr_m       = homogeneousMatrixFromPos(pose);
  [rMc_m,rVc_m]= loadHRP2camCom();
  wMc_m       = wMr_m*rMc_m;
  posecMo_m   = [ 0 0 1 0 0 0 ];
  posecDesMo_m= [ 0 0 1 0 0 0 ];
  cMo_m       = homogeneousMatrixFromPos(posecMo_m); // pose target/object init   
  wMo_m       = wMc_m*cMo_m;
  // compute the init projection on the view
  cP_m        = changeFrameMire(oP_m,cMo_m);       // target Points in the camera frame
  s_m         = projectMireDirect(cP_m);           // projection of the target points in the image plane 
  Z_m         = cP_m(3:3:length(cP_m)) ;           // depth of the target points in the camera frame


  //------- Desired Camera Object Position 
  cDesMo_m     = (homogeneousMatrixFromPos(posecDesMo_m));
  wMcDes_m     = wMo_m*inv(cDesMo_m);
  wMcDes_m     = wMcDes_m.*(abs(wMcDes_m)>1e-10);
  // compute the desired projection on the view
  cDesP_m      = changeFrameMire(oP_m,cDesMo_m);   // desired target Points in the camera frame
  sDes_m       = projectMireDirect(cDesP_m);       // desired target Points projection
  ZDes_m       = cDesP_m(3:3:length(cDesP_m)) ;    // desired depth

  halt;


 //-----------------
 //
 //------------------
 [Sp_m, Sv_m, Sa_m, Up_m, Uv_m, Ua_m] = buildC(Np_m,Te_m);
 [sm_out, Z_out ]= predHorGlobalMireJerk(s_m,Z_m,stateCoM_m,rMc_m,jerk_m,Sp_m,Up_m,Sv_m,Uv_m,Sa_m,Ua_m,Te_m,Np_m,ndof_m);
 
 global computeL_global ;
 computeL_global= matIntMireC;
 L_out = predIntMatGlobalJerk(computeL_global,sm_out,Z_out,Np_m,Nbpts_m);
 Lbig= bigPredBigMatrixGlobal(L_out,Np_m, Nbpts_m);
 StepNumber=4;
 defineGlobalVariableJerk(s_m,Z_m,stateCoM_m,rMc_m,Np_m,Nbpts_m,ndof_m,Te_m,sDes_m,ZDes_m,Lfunction);

Ewalk_global= [[eye(Np_global,Np_global), zeros(Np_global,StepNumber*2+Np_global)];[zeros(Np_global,StepNumber+Np_global),eye(Np_global,Np_global), zeros(Np_global,StepNumber)]] ;
Q       = Ewalk_global'*Udiag_global'*Eframe_global'*Vdiag_global'*Lbig'*Lbig*Vdiag_global*Eframe_global*Udiag_global*Ewalk_global;
pT      = (sm_out-Sdes_global)'*Lbig*Vdiag_global*Eframe_global*Udiag_global*Ewalk_global;

  djerk_m      = 0*ones(Np_m,1);
  ndof_m      = ndof_in;
  for i=1:ndof_m-1
      djerk_m  = [djerk_m; zeros(Np_m,1)];
  end 

djerkAndrei = [djerk_m(1:Np_m);zeros(StepNumber,1);0.0*ones(Np_m,1);
zeros(StepNumber,1)];

costQP = (sm_out-Sdes_global)'*(sm_out-Sdes_global)+pT*djerkAndrei+djerkAndrei'*Q*djerkAndrei

Q2     = Udiag_global'*Eframe_global'*Vdiag_global'*Lbig'*Lbig*Vdiag_global*Eframe_global*Udiag_global;
pT2    = (sm_out-Sdes_global)'*Lbig*Vdiag_global*Eframe_global*Udiag_global;

cost2 =  djerk_m'*Q2* djerk_m+pT2* djerk_m+(sm_out-Sdes_global)'*(sm_out-Sdes_global)

[Qvisu, pTvisu]= computeQPvisuDelta(sm_out,Sdes_global, Lbig)
cost3 = djerk_m'*Qvisu* djerk_m+pTvisu* djerk_m+(sm_out-Sdes_global)'*(sm_out-Sdes_global)

 
  halt;
  index=1;

  costGlobal = costGlobalMireJerk(index,jerk_m+djerk_m);
  disp('costGlobal')
  disp(costGlobal)
  costLocal = costLocalMireJerk(index,jerk_m+djerk_m) ;
  disp('costLocal')
  disp(costLocal)

 









