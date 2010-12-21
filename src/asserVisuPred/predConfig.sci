function confPredVisualServo(pose_cMo_init, pose_cMo_des)
//  
//posewMo_m   = [0   0   0 0 %pi 0 ];              // pose of the target in the World Frame
//
//
//Te_m        = 1;   // to be consistant with the image frame rate
//a_m         = 0.20;                              // dimension of the target  
//oP_m        = mire4points (a_m);                 // create the Npbts Points Target
//Nbpts_m     = length(oP_m)/3 ;
//
//
//
//Np_m        = 1;                                // horizon lenght
//Nc_m        = 1;                                 // command horizon length  
//v_m         = [0 0 0 0 0 0]';                    // init velocity
defineGlobalVariable(s_in,Z_in,Nc_in,Np_in,Nbpts_in,Te_in,sdes_in,Zdes_in,Q_in,e0_in,xl_in,xu_in,bl_in,bu_in,computeL_in,option_ineq)
endfunction

//-----------------------------------------------//
//
//-----------------------------------------------//
function defineGlobalVariable(...
s_in,...
Z_in,...
stateCoM_in,...
rMc_in,...
Nc_in,...
Np_in,...
Nbpts_in,...
ndof_in,...
Te_in,...
sdes_in,...
Zdes_in,...
Q_in,...
e0_in,...
xl_in,...
xu_in,...
bl_in,...
bu_in,...
computeL_in,...
tol_in,...
option_ineq)

// We need global variable to make the cost fonction only depedent on U
global Nc_global ;      Nc_global       = Nc_in;      // control horizon
global Np_global ;      Np_global       = Np_in;      // prediction horizon
global Nbpts_global;    Nbpts_global    = Nbpts_in;   // number of points of the target   
global Te_global ;      Te_global       = Te_in       // the sampling time
global Zdes_global;     Zdes_global     = Zdes_in;    // the des depth
global sdes_global;     sdes_global     = sdes_in;    // the des features
global Z_global;        Z_global        = Z_in;       // the des depth
global s_global;        s_global        = s_in;       // the des features
global Q_global;        Q_global        = Q_in;       // weighted matrix
global e0_global;       e0_global       = e0_in;      // error between model and truth 
global computeL_global; computeL_global = computeL_in;// name of the function to compute the interaction matrix 
global STORE;           STORE           = [];
global ndof_global;     ndof_global     = ndof_in;    // degrees of freedom of the CoM
global rMc_global;      [rMc_global,rVc]= rMc_in;          
global stateCoM_global; stateCoM_global = stateCoM_in; 

global Sp_global;
global Up_global;
global Sv_global;
global Uv_global;  
global Sa_global; 
global Ua_global;
[Sp_global, Sv_global, Sa_global, Up_global, Uv_global, Ua_global] = buildC(Np_m,Te_m);


xmax    = xu_in(1);                    // position max of the a 2D point in the image plane x axis
xmin    = xl_in(1);                    // position min of the a 2D point in the image plane x axis
ymax    = xu_in(2);                    // position max of the a 2D point in the image plane y axis
ymin    = xl_in(2);                    // position min of the a 2D point in the image plane y axis 

global constraints_global;           // constraint limit vector needed in the contraint function eval
constraints_global = ga_constraintsLimits(Np_global,Nbpts_global,xmax,ymax,xmin,ymin);

global bl_global;
   bl_global = [];
global bu_global;
   bu_global = [];
  for i=1:Nc_global
    bu_global = [bu_global;bu_in];                           // upper bounds
    bl_global = [bl_global;bl_in];                           // upper bounds
  end

global ipar_global;
nf        = 1;         // nombre de fonction de cout
nineqn    = 0;         // nombre de contraintes d'inegalite nl
if(option_ineq)
   nineq  = Np_in*Nbpts_in*4; // nombre de contraintes d'inegalites l
else 
   nineq  = 0;
 end
 
neqn      = 0;         // nombre de contraintes d'egalite nl
neq       = 0;         // nombre de contraintes d'egalite l
modefsqp  = 100; 
miter     = 100;    // maximum number of iteration allowed by the user 
iprint    = 2;        // displayed parameter objective and constraint is displayed
ipar_global=[nf,nineqn,nineq,neqn,neq,modefsqp,miter,iprint];

global rpar_global;
bigbnd    = 1e4;       // infinity value
eps       = tol_in;      // final norm requirement for d0k
epsneq    = 0.e0;      // maximum violation of linear equalite contraints
udelta    = 0.e0;      // the perturbation sixze the user suggest to compute the gradients
               // 0 if the user has no idea of what to suggest  
rpar_global=[bigbnd,eps,epsneq,udelta];
endfunction



function defineGlobalVariableJerk(...
s_in,...
Z_in,...
stateCoM_in,...
rMc_in,...
Np_in,...
Nbpts_in,...
ndof_in,...
Te_in,...
sdes_in,...
Zdes_in,...
computeL_in)

// We need global variable to make the cost fonction only depedent on U
global Np_global ;      Np_global       = Np_in;      // prediction horizon
global Nbpts_global;    Nbpts_global    = Nbpts_in;   // number of points of the target   
global Te_global ;      Te_global       = Te_in       // the sampling time
global Zdes_global;     Zdes_global     = Zdes_in;    // the des depth
global sdes_global;     sdes_global     = sdes_in;    // the des features
global Z_global;        Z_global        = Z_in;       // the des depth
global s_global;        s_global        = s_in;       // the des features
global computeL_global; computeL_global = computeL_in;// name of the function to compute the interaction matrix 
global ndof_global;     ndof_global     = ndof_in;    // degrees of freedom of the CoM
global rMc_global;      rMc_global      = rMc_in;          
global stateCoM_global; stateCoM_global = stateCoM_in; 
global e0_global;       e0_global       = zeros(Nbpts_global*2,1)
global Sp_global;
global Up_global;
global Sv_global;
global Uv_global;  
global Sa_global; 
global Ua_global;
[Sp_global, Sv_global, Sa_global, Up_global, Uv_global, Ua_global] = buildC(Np_global,Te_global);

// pour le QP
global Udiag_global; Udiag_global = bigDiag(Up_global,ndof_global);

//pose    = [ 0 0 0 -%pi 0 0]';
//wMr     = homogeneousMatrixFromPos(pose);
//wVc     = twistMatrix(wMr*rMc_global);
//cVw     = inv(wVc);
cVw       = eye(6,6);// cVw(:,[1,2]);

global Vdiag_global;  Vdiag_global    = bigDiag(cVw,Np_global);
global Eframe_global;  Eframe_global  = selectJerk(ndof_global,Np_global);  
global Sdes_global ; 
Sdes_global  =[];
for i=1:Np_m
     Sdes_global =[Sdes_global ; sdes_global];
end

global Ewalk_global  ; 
//Ewalk_global= [[eye(Np_global,Np_global), zeros(Np_global,stepNumber_in*2+Np_global)];[zeros(Np_global,stepNumber_in+Np_global),eye(Np_globa//l,Np_global), zeros(Np_global,stepNumber_in)]] ;


endfunction





//-----------------------------------------------------//
//             predControlLocalMire
//-----------------------------------------------------//
function updateVar(s_in,Z_in,stateCoM_in,sdes_in,Zdes_in,e0_in)
global Z_global;           Z_global        = Z_in;
global s_global;           s_global        = s_in;
global e0_global;          e0_global       = e0_in;
global sdes_global;        sdes_global     = sdes_in;
global Zdes_global;        Zdes_global     = Zdes_in;
global stateCoM_global;    stateCoM_global =stateCoM_in
endfunction

function updateVarJerk(s_in,Z_in,stateCoM_in,sdes_in,Zdes_in)
global Z_global;           Z_global        = Z_in;
global s_global;           s_global        = s_in;
global sdes_global;        sdes_global     = sdes_in;
global Zdes_global;        Zdes_global     = Zdes_in;
global stateCoM_global;    stateCoM_global =stateCoM_in
endfunction


function isConfigLoaded()
  disp('yes')
endfunction

 
