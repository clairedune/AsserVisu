//;exec('testStateJerk.sce');
// this exemple test all the functions of
// src/asserVisuPred/predPredictionMire.sci

clear;
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

Np_m           = 3;
Te_m           = 0.1;
// build the amtrices used to compute the robot state from the current state and jerk
[Sp_m, Sv_m, Sa_m, Up_m, Uv_m, Ua_m] = buildC(Np_m,Te_m);

if(0)
  jerkx_m        = 1*ones(Np_m,1); 
  statex_m       = [0,1,-1]';
  dxHor          = horizonFromJerk(statex_m,jerkx_m,Sv_m,Uv_m);


  jerky_m        = -0.5*ones(Np_m,1);
  statey_m       = [1,0,1]';
  dyHor          = horizonFromJerk(statey_m,jerky_m,Sv_m,Uv_m);


  jerktheta_m    = 0*ones(Np_m,1);
  statetheta_m   = [0,0,0]';
  statetheta_m   = horizonFromJerk(statetheta_m,jerktheta_m,Sv_m,Uv_m);

  state          = [statex_m(1)
                  statey_m(1)
                  statetheta_m(1) 
                  statex_m(2)
                  statey_m(2)
                  statetheta_m(2)
                  statex_m(3)
                  statey_m(3)
                  statetheta_m(3)  ];


  jerk_m        = [jerkx_m;jerky_m;jerktheta_m];
  [P,Pstack,V,Vstack,A,Astack] = stateFromJerkHorizon(state,jerk_m,Sp_m,Up_m,Sv_m,Uv_m,Sa_m,Ua_m);
  [v, vstack] = velocityFromJerkHorizon(state,Sv_m,Uv_m,jerk_m);
end

halt();
//-----------------------Test StateFromJerk------------------------------------------//
if(1)
// construction de l'etat
pose        = [ 0 0 0 -%pi 0 0]';
vitesse     = [ 0 0 0]';// 0 0 0]';
acceleration= [ 0 0 0]';// 0 0 0]';

stateCoM    = [pose([1,2,6]);vitesse;acceleration];
//stateCoM    = [pose;vitesse;acceleration];

// construction du jerk
jerk_m      = [];
ndof_m      = length(vitesse);

jerk_m      = [1*[ones(Np_m,1)];0.3*[ones(Np_m,1)]];
for i=1:ndof_m-2
    jerk_m  = [jerk_m; zeros(Np_m,1)];
end   



[rMc,rVc]   = loadHRP2camCom();

//----------------------- predHorGlobalMireJerk ------------------------------------------//
posecMoInit    = [ 0 0 5 0 0*%pi/180 0 ];
wMr            = homogeneousMatrixFromPos(pose);
cMo            = homogeneousMatrixFromPos(posecMoInit);
wMc            = wMr*rMc; 
wMo            = wMr*rMc*cMo;         

//-------- Create the target -----------//
a              = 0.2; // related to the target size
Nbpts_m        = 5;
oP             = mire5points (a); 
cP             = changeFrameMire(oP,cMo); 
wP             = changeFrameMire(oP,wMo); 
s              = projectMireDirect(cP);
Z              = cP(3:3:length(cP)); 

[P,Pstack,V,Vstack,A,Astack] = stateFromJerkHorizon(stateCoM,jerk_m,Sp_m,Up_m,Sv_m,Uv_m,Sa_m,Ua_m);
[p, v] = stateCamFromComHor(rMc,P,V,ndof_m,Np_m);

pause
//halt()

hf = createFigure3D(2,"Camera Motion",2);
for i=1:Np_m
  disp(i)
  poseNext =p((i-1)*6+1:(i-1)*6+6)'; 
  poser = [P((i-1)*ndof_m+1);P((i-1)*ndof_m+2);0;-%pi;0;P((i-1)*ndof_m+3)];


  wMc = homogeneousMatrixFromPos(poseNext);
  wMr = homogeneousMatrixFromPos(poser);


  Camera3DDrawColor(0.1,wMc,10);// display the first camera
  Camera3DDrawColor(0.1,wMr,5);// display the first camera
  Mire3DDraw5pts(wP); 
  show_pixmap()
end
halt()


[sm_out, Z_out ]= predHorGlobalMireJerk(...
s,Z,stateCoM,rMc,jerk_m,Sp_m,Up_m,Sv_m,Uv_m,Sa_m,Ua_m,Te_m,Np_m,ndof_m);
sm_out4 = ga_predHorGlobalMire(s,Z,v,Te_m,Np_m);

global computeL_global ;
computeL_global= matIntMireC;
L_out = predIntMatGlobalJerk(computeL_global,sm_out,Z_out,Np_m,Nbpts_m);
L_outbig= bigPredBigMatrixGlobal(L_out,Np_m, Nbpts_m);



sm_out2  = predHorLocalMireJerk(s,Z,stateCoM,rMc,jerk_m,Sp_m,Up_m,Sv_m,Uv_m,Sa_m,Ua_m,Te_m,Np_m,ndof_m);
hf3 = createFigure2D(3,'evolutionMire');
mireEvolutionDraw(Np_m,sm_out,1);
show_pixmap();
hf4 = createFigure2D(4,'evolutionMire2');
mireEvolutionDraw(Np_m,sm_out2,2);
show_pixmap();
sm_out3 = ga_predHorLoc2dMire(s,Z,v,Te_m,Np_m);





//halt
//mireEvolutionDraw(Np_m,sm_out3,1);
//show_pixmap();


