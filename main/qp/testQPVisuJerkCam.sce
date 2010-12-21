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
  Lfunction     = matIntMireC;                 //-------- LFunction 
  funcost_m     = funcost_in;                  // ----- Cost and function
  

  ndof_m        = 2;                           // X and Y are free theta constrained
  Np_m          = 5;                           // Horizon length
  Nc_m          = Np_m;
  name_in       ='test';

  [Sp_m, Sv_m, Sa_m, Up_m, Uv_m, Ua_m] = buildC(Np_m,Te_m);







  //----------------------------------------------------------------//
  //  problem Statement 
  // A fixed Object and a mobile camera
  //----------------------------------------------------------------//
 
  posecMo_m     = [ 0 0 1 0 0 0 ];    // current object position in the camera frame
  posecDesMo_m  = [ 0 0 1 0 0 0 ];    // desired object position in the camera frame

  
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
  cMo_m         = homogeneousMatrixFromPos(posecMo_m); // pose target/object init   
  // init state
  stateCam_m    = zeros(3*ndof_m,1);                
  oMc_m         = inv(cMo_m);
  stateCam_m(1) = oMc_m(1,4);
  stateCam_m(2) = oMc_m(3,4);

  //------ compute the init projection on the view
  cP_m          = changeFrameMire(oP_m,cMo_m);   // target Points in the camera frame
  s_m           = projectMireDirect(cP_m);       // projection of the target points  
  Z_m           = cP_m(3:3:$) ;                  // depth of the target points in the camera frame
  //------- Desired Camera Object Position 
  cDesMo_m     = (homogeneousMatrixFromPos(posecDesMo_m));
  cDesP_m      = changeFrameMire(oP_m,cDesMo_m); // desired target Points in the camera frame
  sDes_m       = projectMireDirect(cDesP_m);     // desired target Points projection
  ZDes_m       = cDesP_m(3:3:length(cDesP_m)) ;  // desired depth

  halt

  cote_m=0.1;
  maxiter  = 150;
  oMcfirst = oMc_m;
  k=0;
  maxiter  = 150;
  h2d        = createFigure2D(3,'prediction');

  // position vitesse acceleration
  [Pcam,Pstack,Vcam,Vstack,A,Astack] = stateFromJerkHorizon(stateCam_m,jerk_m,Sp_m,Up_m,Sv_m,Uv_m,Sa_m,Ua_m);
  disp('Pcam')
  disp(Pcam)
  disp('Vcam')
  disp(Vcam)  
 


  Udiag_m = bigDiag(Up_m,2);
  Sdes_m =[];
  for i=1:Np_m
     Sdes_m =[Sdes_m ; sDes_m];
  end
  Eframe_m  = selectJerk(2,Np_m);


while(k<maxiter)
   pause
 
   Sk           = [];
   Z_out        = [];
   oMcPrec      = oMc_m;
   cPrecP       = cP_m;
   [Pcam,Pstack,Vcam,Vstack,A,Astack] = stateFromJerkHorizon(stateCam_m(1:ndof_m*3),jerk_m,Sp_global,Up_global,Sv_global,Uv_global,Sa_global,Ua_global);
   for i=1:Np_m
 
        poseNext = [Pcam((i-1)*2+1) 0 Pcam((i-1)*2+2) 0 0 0]  ;//Pcam((i-1)*6+1:(i-1)*6+6)'; 
        oMcNext = homogeneousMatrixFromPos(poseNext)
        cPrecMcNext  = inv(oMcPrec)*oMcNext  ;
        
        // new feature  
        cNextP       = changeFrameMire(cPrecP,inv(cPrecMcNext)); 
        Z_out        = [Z_out; cNextP(3:3:$)] ;            
        Sk           = [Sk;projectMireDirect(cNextP)];
        // update
        oMcPrec      = oMcNext;   
        cPrecP       = cNextP;
  end
 
  L_out          = predIntMatGlobalJerk(computeL_global,Sk,Z_out,Np_m,Nbpts_m);
  Lbig           = bigPredBigMatrixGlobal(L_out,Np_m, Nbpts_m);
  me = 0;
  

  //------------QP
 
  Qvisu   = Udiag_m' * Eframe_m*Lbig(:,[1,3])'*Lbig(:,[1,3])*Eframe_m*Udiag_m;
  pTvisu  = (Sk-Sdes_m)'*Lbig*Eframe_m*Udiag_m;
  //Qvisu=Qvisu+1e-2*eye(Np_m*ndof_m,Np_m*ndof_m);
 // [deltaJerk, lagr, info] = qld(Qvisu, pTvisu, [], [], bl_m, bu_m, me, 1e-13);
    
   //disp('deltajerk')
   //deltaJerk
  
 //  jerk_m = jerk_m+deltaJerk
 //  costQP = (Sk-Sdes_global)'*(Sk-Sdes_global)+pTvisu*deltaJerk+deltaJerk'*Qvisu*deltaJerk
 
//   [Pcam,Pstack,Vcam,Vstack,A,Astack] = stateFromJerkHorizon(stateCam_m(1:ndof_m*3),jerk_m,Sp_global,Up_global,Sv_global,Uv_global,Sa_global,Ua_global);
 
   //update the value for the servoing
  // k = k + 1;
   
   // -----------------------------------------------------
   //vcamReal   = [Vcam(1),0,Vcam(2),0,0,0 ]; 
   //vcamReal   = vcamReal.*(abs(vcamReal) > 1e-6);
  // disp(vcamReal([1,3,5]));

   // the resulting camera motion is then  
   //wMc_m      = homogeneouMatrixFromPos([Pcam(1),0,Pcam(2),0 ,0 ,0]);
   //cMo_m      = inv(wMc_m)*wMo_m;
   //cP_m       = changeFrameMire(oP_m,cMo_m);                    // compute the 3D points
   //s_m        = projectMireDirect(cP_m);                      // compute the 2D points
   //Z_m        = cP_m(3:3:$);                         // compute Z
  
//   updateVarJerk(s_m,Z_m,stateCam_m(1:6),sDes_m,ZDes_m);
  

  // --- store and display ---//
  //posec      = pFromHomogeneousMatrix(wMc_m);
  //Xc         = [Xc;posec(1)];
  //Yc         = [Yc;posec(2)]; 
  //Zc         = [Zc;posec(3)];
  // hf         = createFigure3D(2,"Camera Motion",2);
  //Camera3DDrawColor(1,wMcfirst,3);
  //Camera3DDrawColor(1,wMcDes_m,5);
  //Mire3DDraw5pts(wP_m);
  //if (k>=2)
  //  plot3d(Xc,Yc,Zc);
  //end
  //Camera3DDraw(cote_m,wMc_m);
  //show_pixmap()
  
    
  
//  xset("window",3);
 // mireEvolutionDraw(Np_m,Sk,1);
 // mire2DDraw(s_m,0.01,3);       // current projection
 // show_pixmap()
 // mire2DDraw(sDes_m,0.01,5);   // desired projection 
 // show_pixmap()
  //h2d2       = createFigure2D(4,'topView');
 // plot(p(1:6:$),p(2:6:$))

   
end


