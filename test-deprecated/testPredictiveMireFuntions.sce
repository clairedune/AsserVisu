//--------------------------------------------------//
// main program
// test the functions defined in 
// src/asserVisu/predictiveControl.sci
//
// author Claire Dune
// date 04/01/2010
//;exec('testPredictiveMireFuntions.sce');
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

// load optimisation
exec('../HuMAnS/KickStart.sci');
execstr(LoadModule('../HuMAnS/Kernel'));
if ~c_link('libcfsqp') then exec('../HuMAnS/Kernel/OptimizationTools/fsqp-1.2/loader.sce') ; end 

//---------------------------------------------------------------//
//              Mire Local with arbitrary Z   
//---------------------------------------------------------------//
  a             = 0.20; // related to the target size   
  oP            = mire4points (a); 
  posewMo       = [0 0 0 0 0 0 ]; 
  wMo           = homogeneousMatrixFromPos(posewMo); 
  wP            = changeFrameMire(oP,wMo);         
 
  posecMo       = [0 0 1 0 0 %pi/2 ]; 
  cMo           = homogeneousMatrixFromPos(posecMo);
  wMc           = wMo*inv(cMo);
  cP            = changeFrameMire(oP,cMo); 
  s             = projectMireDirect(cP);
  Z             = cP(3:3:length(cP));
    
  posecDesMo    = [0 0 1 0 0 0 ]; 
  cDesMo        = homogeneousMatrixFromPos(posecDesMo);
  wMcDes        = wMo*inv(cDesMo);
  cDesP         = changeFrameMire(oP,cDesMo) ;
  sDes          = projectMireDirect(cDesP);
  ZDes          = cDesP(3:3:length(cDesP));
  
  v             = [0 0 0 0 0 1]';
  Uc            = [v;v;v;v;v];
  
disp('Test predictif optim')


  Te            = 1/30; // to be consistant with the image frame rate
  Nc            = 5;
  Np            = 20;
  xu            = [  1 ;  1 ];     // position max of the a 2D point in the image plane 
  xl            = [ -1 ; -1 ];     // position min of the a 2D point in the image plane 
  bu            = 10*[0.25,0.25,0.25,0.25,0.25,0.25]';                // command bounds
  bl            = -bu;
  Nbpts         = 4 ;
  e0            = zeros(8,1);
 
  Lfunction  = matIntMireC; // Lc(t) classical visual servo s(t) Z(t)
  //Lfunction  = matIntMireP; // Lp(t) classical visual servo s(t) Z*
  //Lfunction  = matIntMireM;   // Lm(t) mixte (L*+Lc(t))
  //Lfunction  = matIntMireD; // Ld    classical interaction matrix desired
  Q = matWeightIdentity(Np,Nbpts);
  //Q = matWeightIdentityZero(Np,Nbpts,1);
  //Q = matWeightTV(Np,Nbpts);
  
  defineGlobalVariable(Nc,Np,Nbpts,Te,sDes,ZDes,s,Z,Q,e0,xl,xu,bl,bu,Lfunction);
  
  U = computeControlOnHorizon(Nc,Np,Uc)
  sm = ga_predHorLoc2dMire(Np,s,U,Te,Z);
 
  
  
  disp('-----------------sm')
  disp(sm')
 
 disp('----------------stest')
  stest = s;
  st =[];
  for i=1:Np
     L = Lfunction(stest,Z)
     val =Te*L*U((i-1)*6+1:(i-1)*6+6);
     disp(val')
     stest = stest + Te*L*U((i-1)*6+1:(i-1)*6+6);
     st=[st; stest];
  end
  disp(st')
  
  disp((st-sm)')
  
  

  
  cost = ga_costHorizon(Np,sm,Q,e0,sDes)
  cost = ga_costLocalMire(1,Uc)

//  
  lambda=0.4;
  L = Lfunction(s,Z);
  e = s-sDes;
  vc = computeVelocity(lambda, L,e)
  cost = ga_costLocalMire(1,Uc)
  vcc = computeControlOnHorizon(Nc,Np,Uc);

  c1Mc2    = computeMotion(vc,Te) // resulting motion
  cMoAV    = inv(c1Mc2)*cMo;
  cP       = changeFrameMire(oP,cMoAV); 
  sAV      = projectMireDirect(cP)'
  sA2      = (s + Te*L*vc');
  disp(sA2')
  cMoA     = c1Mc2*cMo;
  cP       = changeFrameMire(oP,cMoA); 
  sA3      = projectMireDirect(cP)'
  
  
  coutA=sAV-sDes'   ;
  costA=norm(coutA)
   
   halt
  [x,sm,res]=predControlLocalMire(Uc,s,Z,sDes,ZDes,e0);
    hf2d5 = createPlanImage(6,xl,xu,"Prediction Mire");
  mire2DDraw(s,0.01,3);                     // current projection
  show_pixmap()
  mire2DDraw(sDes-e0,0.01,5);             // desired projection 
  show_pixmap()
  mire2DDraw(sm(1:Nbpts*2),0.01,4);       //model projection
  show_pixmap()
  smvisu =[s;sm];
  mireEvolutionDraw(Np+1,smvisu,1);
  show_pixmap()
  
  
  x'
  cost = ga_costLocalMire(1,x)
 
  c1Mc2    = computeMotion(x(1:6)',Te) // resulting motion
  cMoB     = inv(c1Mc2)*cMo;
  cP       = changeFrameMire(oP,cMoB); 
  sB       = projectMireDirect(cP)'
  coutB=sB-sDes'   ;
  costB=norm(coutB) 
