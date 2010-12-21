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


getd(path + 'src/optimisation')
// load optimisation
exec('../HuMAnS/KickStart.sci');
execstr(LoadModule('../HuMAnS/Kernel'));
if ~c_link('libcfsqp') then exec('../HuMAnS/Kernel/OptimizationTools/fsqp-1.2/loader.sce') ; end 



//-------------- POSITION DU PROBLEME---------------------------//
sm = [ 1 1 ; 1 1 ];
sd = [ 0 0 ; 0 0 ];
Q = [1 0 ; 0 1];
Te            = 1/30; 			// to be consistant with the image frame rate
v             = [0.2 -0.4 1 0 0 0]'; 	// init velocity
posecDesMo    = [0 0.1 2 0 0 0 ]; 	// desired pose 
posecMo       = [0 0 2 0.1 0 0.01 ]; 	// current pose
cMo           = homogeneousMatrixFromPos(posecMo);
[oP cP s]     = mire4pointsInCam(0.20,cMo);
cDesMo        = homogeneousMatrixFromPos(posecDesMo);
cDesP         = changeFrameMire(oP,cDesMo); 
sDes          = projectMireDirect(cDesP);
ZDes          = cDesP(3:3:length(cDesP));
Nbpts         = 4 ;
  
Nc            = 1;
Np            = 10;
bu            = 1e4*[0.25,0.25,0.25,0.25,0.25,0.25]';                // command bounds
bl            = -bu;
Z             = cP(3:3:length(cP));
e0            = zeros(8,1);
Q             = matWeightIdentity(Np,Nbpts);
xu   = [  0.2 ;  0.2 ];                 // position max of the a 2D point in the image plane 
xl   = [ -0.2 ;  -0.2 ];                // position min of the a 2D point in the image plane 
U    =     [];
for i=1:Nc
  U  = [U;v]
end


computeL= matIntMireC;



defineGlobalVariable(s,...
  Z,Nc,Np,Nbpts,Te,sDes,ZDes,Q,e0,xl,xu,bl,bu,computeL,1e-8,%F);
 
disp('test ga_predGlobalMire & test ga_predLoc2DMire') 
[smG,ZG]    = ga_predGlobalMire(s,Z,v,Te);
 smL        = ga_predLoc2dMire(s,Z,v,Te);
 
 disp('[smG smL]')
 disp([smG smL])

Up = computeControlOnHorizon(U,Nc,Np);
smHorL     = ga_predHorLoc2dMire(s,Z,Up,Te,Np);
smHorG     = ga_predHorGlobalMire(s,Z,Up,Te,Np);
disp('[smHorG smHorL]')
disp([smHorG smHorL])

halt

lambda =1/Te
disp('Asservissement visuel classique')
L        = computeL(s,Z);
v        = computeVelocity(lambda, L,s-sDes);
v        = v';
disp(v')


ga_costSQPMire(1,v)
ga_costLocalMire(1,v)


funcost     = ga_costGlobalMire;
funpred     = ga_predHorGlobalMire;

//funcost     = ga_costLocalMire;
//funpred     = ga_predHorLoc2dMire;

//funcost     = ga_costSQPMire;
//funpred     = ga_predHorLoc2dMire;

funcstr     = ga_constraintsLocalMire;
jaccost     = "grobfd";
jaccst      = "grcnfd";
[U,sm,Uhor] = predControl(U,funpred,funcost,funcstr,jaccost,jaccst );
