//--------------------------------------------------//
// main program
// test the functions defined in 
// src/asserVisu/predictiveControl.sci
//
// author Claire Dune
// date 28/01/2010
// Pour tester les fonctions de cout
// ;exec('testAsserVisuTous.sce');
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

funcost_m     = testSQP;
funcst_m     = ga_constraintsLocalMire;
jaccost_m    = "grobfd";
jaccst_m     = "grcnfd";

bu_m        = 1e3*0.25*ones(6,1);  // command bounds
bl_m        = -bu_m;                             // command bounds on the horizon

v_m = [0 0 0 0 0 0 ]';
    disp('SQP')
    nf          = 1;         // nombre de fonction de cout
    nineqn      = 0;         // nombre de contraintes d'inegalite nl
    nineq  = 0;
    neqn        = 0;         // nombre de contraintes d'egalite nl
    neq         = 0;         // nombre de contraintes d'egalite l
    modefsqp    = 100; 
    miter       = 500;    // maximum number of iteration allowed by the user 
    iprint      = 0;        // displayed parameter objective and constraint is displayed
    ipar        =[nf,nineqn,nineq,neqn,neq,modefsqp,miter,iprint];
  
    bigbnd      = 1e4;       // infinity value
    eps         = 1e-5;      // final norm requirement for d0k
    epsneq      = 0.e0;      // maximum violation of linear equalite contraints
    udelta      = 0.e0;      // the perturbation sixze the user suggest to compute the gradients
               // 0 if the user has no idea of what to suggest  
    rpar        =[bigbnd,eps,epsneq,udelta];

    v_m=fsqp(v_m,ipar,rpar,[bl_m,bu_m],funcost_m,funcst_m,jaccost_m,jaccst_m);
    cost = funcost_m(1,v_m);












