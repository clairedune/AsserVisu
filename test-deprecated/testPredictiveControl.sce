//--------------------------------------------------//
// main program
// test the functions defined in 
// src/asserVisu/predictiveControl.sci
//
// author Claire Dune
// date 04/01/2010
//;exec('testPredictiveControl.sce');
//--------------------------------------------------//

clear

DEBUG_VERBOSE = %F;
stacksize(4e7)

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

Te = 0.4; // to be consistant with the image frame rate

//------- Create the object
// create the target
L = 0.05; // related to the target size 
oP = mire5points (L); 
//oP = mire4points (L);
// ------ First Camera Object Position
pose_cMo_init = [0.1 0.1 1 0 0 %pi/6 ];
cinitMo = homogeneousMatrixFromPos(pose_cMo_init);

// compute the first projection on the view
cinitP = changeFrameMire(oP,cinitMo)
p = projectMireDirect(cinitP);
halt
//------- Desired Camera Object Position 
pose_cMo_desired = [0 0 1 0 0 0 ];
cdesiredMo = homogeneousMatrixFromPos(pose_cMo_desired);
// compute the desired projection on the view
cdesiredP = changeFrameMire(oP,cdesiredMo); 
pd = projectMireDirect(cdesiredP);
Zdes  = cdesiredP(3:3:length(cdesiredP))  


////---------------------------------------------------------------//
////              Mire Local with arbitrary Z   
////---------------------------------------------------------------//

Z       = cinitP(3:3:length(cinitP))      ;   // init depth for the step 
v0      = [0 0 0 0 0 0]';// init velocity
Np      = 20;                   // horizon lenght
Nc      = 1;
U0      = [];                  // create the first control horizon
for i = 1:Nc
  U0    = [U0 ; v0];
end ;

Nbts    = length(cinitP)/3    // number of points of the target 5
xu      = [0.2;0.2];           // position max of the a 2D point in the image plane 
xl      = [-0.2;-0.2];         // position min of the a 2D point in the image plane 
sm0     = p;                   // the first projection is p
e0      = zeros(Nbts*2,1);     // the first error is e0
Q       = matWeightIdentity(Np,Nbts);


sStar   = pd;                  // desired features

// control limit
bu1 = [10,10,10,100*%pi/180,100*%pi/180,100*%pi/180]';

bu=[];
for i=1:Nc
  bu=[bu;bu1];
end
bl = -bu;

//------- Create figure
cote =0.01;
hf2d = createPlanImage(1,xl,xu,"Point 2D");
mire2DDraw(p,cote,3);
show_pixmap()
mire2DDraw(pd,cote,5);
show_pixmap()

global Nc_global ;   Nc_global   = Nc                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               ;
global Np_global ;   Np_global   = Np;        // the sampling time
global Te_global ;   Te_global   = Te;        // the sampling time
global Z_global;     Z_global    = Z;         // the init depth
global Q_global;     Q_global    = Q;         // weighted matrix
global s_global;     s_global    = sm0; 
global e0_global;    e0_global   = e0;        // error between model and truth 
global sdes_global;  sdes_global = sStar;     // desired features   
global Nbpts_global; Nbpts_global= Nbts ;
global Zdes_global;  Zdes_global = Zdes;    

xmax    = xu(1);               // position max of the a 2D point in the image plane x axis
xmin    = xl(1);               // position min of the a 2D point in the image plane x axis
ymax    = xu(2);               // position max of the a 2D point in the image plane y axis
ymin    = xl(2);               // position min of the a 2D point in the image plane y axis 

global constraints_global;        // constraint limit vector needed in the contraint function eval
constraints_global = ga_constraintsLimits(Np,Nbts,xmax,ymax,xmin,ymin);

cost = ga_costLocalMire(1,U0)
halt
constraint = ga_constraintsLocalMire(1,U0)
halt

global computeL_global;computeL_global = matIntMireM;  

global ipar_global;
global rpar_global;
nf        = 1;         // nombre de fonction de cout
nineqn    = 0;         // nombre de contraintes d'inegalite nl
nineq     = Np*Nbts*4; // nombre de contraintes d'inegalites l
neqn      = 0;         // nombre de contraintes d'egalite nl
neq       = 0;         // nombre de contraintes d'egalite l
modefsqp  = 100; 
miter     = 200;       // maximum number of iteration allowed by the user 
iprint    = 2;         // displayed parameter objective and constraint is displayed
ipar_global=[nf,nineqn,nineq,neqn,neq,modefsqp,miter,iprint];

bigbnd    = 1e4;       // infinity value
eps       = 1e-4;      // final norm requirement for d0k
epsneq    = 0.e0;      // maximum violation of linear equalite contraints
udelta    = 0.e0;      // the perturbation sixze the user suggest to compute the gradients
               // 0 if the user has no idea of what to suggest  
rpar_global=[bigbnd,eps,epsneq,udelta];

x=fsqp(U0,ipar_global,rpar_global,[bl,bu],ga_costLocalMire,ga_constraintsLocalMire,"grobfd","grcnfd");

//deduce the evolution of sm
res = computeControlOnHorizon(x,Nc_global,Np_global);


//Np=100;
//res =[];
//vres = [0 0 0 0 0 %pi/2]'
//for i = 1:Np
//  res    = [res ; vres];
//end ;
sm = ga_predHorLoc2dMire(p,Z,res,Te,computeL_global,Np);


sm = [p; sm];
mireEvolutionDraw(Np+1,sm,2);
show_pixmap()


disp('------ the end -------')
disp('')









