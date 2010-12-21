//--------------------------------------------------//
// main program
// test the functions defined in 
// src/asserVisu/predictiveControl.sci
//
// author Claire Dune
// date 04/01/2010
//--------------------------------------------------//

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





//--------------------------------------------------//
//              Tests unitaires                    //
//--------------------------------------------------//


//----- test ga_cost
sm = [ 1 1 ; 1 1 ];
sd = [ 0 0 ; 0 0 ];
Q = [1 0 ; 0 1];
//disp('Test ga_cost');
//ga_cost(sm,Q,sd);


//---------------------------------------------------------------//
//              1 Point Local with arbitrary Z   
//---------------------------------------------------------------//
if(0)
  
  //----- test ga_predLocal2dPoint
  Te = 1/30; // to be consistant with the image frame rate
  v = [0 0.1 0.01 0 0 2*%pi/180]';
  smCurrent = [0.1;0.01];
  Zmenu =1;
  disp('Test ga_predLocal2dPoint');
  smNext = ga_predLocal2dPoint(smCurrent,v,Te,Zmenu);

  //----- test ga_predHorLoc2dPoint
  Nhor = 5;
  U=[];
  for i=1:Nhor
    U = [ U ;v];
  end 
  
  disp('Test ga_predHorLoc2dPoint')
  sm = ga_predHorLoc2dPoint(smCurrent,U,Te,Zmenu)

  //----- deroule et verifie (equivaut a L constante sur l'horizon)
  sm = ga_predVerifLoc2dPoint(smCurrent,U,Te,Zmenu)

  //----- test cost = ga_costHorizon
  smPredict = [0.099;0.012];
  e0 = smCurrent - smPredict;
  sStar = [0;0];
  disp('Test ga_costHorizon')
  cost = ga_costHorLoc2dPoint(U,Te,Zmenu,smCurrent,Q,e0,sStar)
end

//---------------------------------------------------------------//
//              Mire Local with arbitrary Z   
//---------------------------------------------------------------//
if(1)
  Te    = 1/30; // to be consistant with the image frame rate
  Zmenu = 0.7;
  v     = [0 1 1 0 0 20*%pi/180]';
  Np    = 4;
  U     =[];
  
  for i=1:Np
    U   = [U;v];
  end
  xu      = [  1 ;  1 ];     // position max of the a 2D point in the image plane 
  xl      = [ -1 ; -1 ];     // position min of the a 2D point in the image plane 

  //----- test ga_predLoc2dMire
  // its position in the camera frame is 
  pose_cMo = [0.1 0.1 1 0 0 0 ];
  cMo = homogeneousMatrixFromPos(pose_cMo);
  [oP cP p] = mire4pointsInCam(0.05,cMo);
  disp('Test ga_predLoc2dMire')
  sm0 = ga_predLoc2dMire(p,v,Te,Zmenu);
 
  ////----- test ga_predHorLoc2dMire(sm0,U,Te,Z)
  disp('Test ga_predHorLoc2dMire')
  sm = ga_predHorLoc2dMire(Np,sm0,U,Te,Zmenu);
  hf2d = createPlanImage(1,xl,xu,"Local2d");
  mireEvolutionDraw(Np,sm,2);
  show_pixmap()
   
disp('Test predictif optim')
  posewMo       = [0 0 0 0 %pi 0 ];                     // object pose
  Q             = eye (8,8);
  wMo           = homogeneousMatrixFromPos(posewMo); // object pose
  wP            = changeFrameMire(oP,wMo);         // the dots in the fixed frame
  posecDesMo    = [0 0 1 0 0 0 ]; 
  cDesMo        = homogeneousMatrixFromPos(posecDesMo);
  wMcDes        = wMo*inv(cDesMo);
  cDesP         = changeFrameMire(oP,cDesMo); 
  sDes          = projectMireDirect(cDesP);
  Nc            = 1;
  Np            = 1;
  OPTL          = 'CURRENT';
  bu            = 1e4*[0.25,0.25,0.25,0.25,0.25,0.25]';                // command bounds
  bl            = -bu;
  Ldes          = zeros(8,6);
  Nbpts         = 4 ;
  Z             = cP(3:3:length(cP));
  e0            = zeros(8,1);
  sm0           = p;
  sStar         = sDes;
  defineGlobalVariable(Nc,Np,Nbpts,Te,Z,Q,e0,sm0,sStar,xl,xu,bl,bu,OPTL,Ldes);
  U0      = [];                                  // create the first control horizon
  for i = 1:Nc
    U0    = [U0 ; [0.1 0.1 0 0 0 0]'];
  end ;
  U0;
  halt
  [U,sm,Uhor] = predControlLocalMire(U0,sm0,e0,sStar,Z,Q)
  lambda =1;
  test = sStar - sm
  
  error_m = p-sStar
  L_m = matIntMire6ddlCase(p,Z,OPTL,Ldes);
  // desired velocity of the camera
  v_m = computeVelocity(lambda, L_m,error_m)


end


//---------------------------------------------------------------//
//              Mire Global    
//---------------------------------------------------------------//
if(0)
  
  Te = 1/30; // to be consistant with the image frame rate
  v = [0 1 1 0 0 20*%pi/180]';
  pose_cMo = [0.1 0.1 1 0 0 0 ];
  cMo = homogeneousMatrixFromPos(pose_cMo);
  [oP cP p] = mire5pointsInCam(0.05,cMo);
  //----- test ga_predGlobal3dMire
  disp('Test ga_predGlobal3dMire')
  [c2P, p2] = ga_predGlobal3dMire(cP,v',Te);

  //-----test ga_predHorGlobal3dMire
  Nhor = 5;
  U    = [];
  for i=1:Nhor
    U = [ U v];
  end ;
  cP0= cP;
  disp('Test ga_predHorGlobal3dMire')
  [cP_store , sm ]= ga_predHorGlobal3dMire(cP0,U,Te);
  hf2d = createPlanImage(0.5,"Point 2D");
  
  mireEvolutionDraw(sm,2);
  show_pixmap()
  
  //-----test ga_costHorGlobal3dMire
  smPredictMire = sm(:,Nhor )+ rand(10,1);
  e0Mire = sm(:,Nhor) - smPredictMire;
  sStarMire = zeros(10,1);
  QMire = eye(10,10);
  cost = ga_costHorGlobal3dMire(U,Te,sm(:,Nhor),QMire,e0Mire,sStarMire)
  //for iter=1:4 
  //  pause;
  //  //last cP modelised
  //  N = size(cP_store,2);
  //  cPLast = cP_store (:,N-4:N);
  //  [cP_store , sm_store ]= ga_predHorGlobal3dMire(cPLast,U,Te);
  //  mireEvolutionDraw(sm_store,iter);
  //  show_pixmap()
  //end

end ;

