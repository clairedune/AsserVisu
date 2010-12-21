// this code test the asser visu 
// given as an input to Andreis function
// for step computing
// 
//
// test another correction
//;exec('main/testAVrobotErreurMarche.sce');
function testSwayCorrection(OPT_CORR,OPT_DISTURB,pathExp)
disp("les fichier seront enreigstres sous path Exp:")
disp(pathExp)
// -- Pose init and final of the robot in the world frame
h              = 0.8;               // CoM height
g              = 9.81;              // gravity
maxiter        = 200;               //
poserInit      = [0 0 h 0 0 0]; // CoM init pose in the world reference frame
poserFinal     = [1 0.5 h 0 0 0]; // CoM final pose in the world reference frame
posecMoInit    = [0 0 1.5 0 0 0 ]; // Object position in the init camera frame
poser          = poserInit;

// ----- For saturations

OPT_SAT        = %T;               // velocity saturation
OPT_WARMING    = %T;               // transient affine increase of the velocity during the 

// ----- For Walking

OPT_WALK_SIMU  = %F;               // sinusoidal addition
OPT_WALK_HRP2  = %T;               // walk simulation 

// ----- For Debug and display

OPT_DEBUG      = %T; 
OPT_DISPLAY    = %T;
OPT_SAVE       = %F;

//-- Values of the saturation in the robot frame

satX           = 0.2;             // x translation
satY           = 0.2;             // y translation
satZ           = 1e3;             
satThetaX      = 1e3;              
satThetaY      = 1e3;              
satThetaZ      = 0.1;              // Z rotation
BIAIS          = 0 ;

// -- For Simulation

dt             = .1;
cote           = .01;              // size of the target dot

// -- For Visual Servoing

lambda         = 0.4;               // aservissement gain
threshold      = .001 ;             // threshold on the error to stop

betaGA         = 0.3;
lambdaI        = 0.7;
lambdaF        = 0;


// -- Perturbation 

disturbA=[-3,-2,-1];
disturbV=dt*disturbA;
DISTURB_ITER=40;
DISTURB_LENGTH=2;


//---------AUTO-SET -----//

if(OPT_WALK_HRP2)
  setActuationSamplingPeriod(2.5e-3);
  setActuationSamplingDelay(0.1e-3);
  setNbActuators(30);
  DOUBLE_SUPPORT = 2; 
  PERIOD=16;
else
  DOUBLE_SUPPORT = 0;
  PERIOD=16;
end

//-------- Matrix Position -------------//

wMrinit        = homogeneousMatrixFromPos(poserInit);
wMr            = wMrinit;
wMrdes         = homogeneousMatrixFromPos(poserFinal);
cMo            = homogeneousMatrixFromPos(posecMoInit);
[rMc,rVc]      = loadHRP2camCom();
wMc            = wMr*rMc;
wMcdes         = wMrdes*rMc
cdesMc         = inv(wMcdes)*wMc;
cdesMo         = cdesMc*cMo;
wMo            = wMc*cMo;         


hf_0           = createFigure3D(20,"Camera Motion",1);
Camera3DDrawColor(0.1,wMc,3);// display the first camera
Camera3DDrawColor(0.1,wMr,5);// display the first camera
show_pixmap()


// set to 0 the values that are < small
tooSmall       = 1e-10;
wMo            = wMo.*(abs(wMo)>tooSmall);
wMc            = wMc.*(abs(wMc)>tooSmall);
wMcdes         = wMcdes.*(abs(wMcdes)>tooSmall);
wMcfirst       = wMc;
wMrfirst       = wMr;

//-------- Create the target -----------//
a              = .1;               // related to the target size
Nbpts          = 5;                // nb points 
oP             = mire5points (a);  // target
cP             = changeFrameMire(oP,cMo); 
wP             = changeFrameMire(oP,wMo); 
p              = projectMireDirect(cP);
pinit          = p;               
Z              = cP(3:3:length(cP)); 
cdesP          = changeFrameMire(oP,cdesMo); 
pdes           = projectMireDirect(cdesP);
Zdes           = cdesP(3:3:length(cP)); 


if(OPT_DEBUG)
  disp('points 2d courant')
  disp(p')
  disp('points 2d desires')
  disp(pdes')
end

//----------------- VISUAL SERVOING LOOP -----------------//
Lfunction   = matIntMireC;
er          = 10; // error value init
iter        = 0;
vcam        = zeros(1,6);
vcamReal    = zeros(1,6);
psway       = zeros(length(p),1);
balancement = zeros(1,6); // difference entre vitesse souhaitee et vitesse realisee
sway        = zeros(1,6); // to test the sway motion
mu          = 0.8;
intsway     = 0;

Xc          = [ wMc(1,4)];
Yc          = [ wMc(2,4)];
Zc          = [ wMc(3,4)];
Xr          = [ wMr(1,4)];
Yr          = [ wMr(2,4)];
Zr          = [ wMr(3,4)];
zmpx        = [];
zmpy        = [];
XPGr        = [ wMc(1,4)];
YPGr        = [ wMc(2,4)];
ZPGr        = [ wMc(3,4)];

// velocity
VDes        = [];
VReal       = [];
VRobotReal  = [];
VRobotDes   = [];
// features
P           = [];
Pcorr       = [];
SWAY        = []; // to store the sway
E           = [];
Ecorr       = [];
normE       = [];
normEcorr   = [];
PSWAY       = [];
LB          = [];
INTEGRALE   = [];



// write in the files
if(OPT_SAVE)
  fError      = mopen(pathExp+'error','a');
  mfprintf(fError,'#testAVrobotErreurMarche.sce >> error real and corrected\n')
  fCamData    = mopen(pathExp+'camData','a');
  mfprintf(fCamData,'#3dof position and 3dof velocity and 3dof corrected velocity\n')
  fComData    = mopen(pathExp+'comData','a');
  mfprintf(fComData,'#testAVrobotErreurMarche.sce >> position real and corrected\n')
  fZmp        = mopen(pathExp+'zmp','a');
end

//Andrei
//state of the robot [x;dx;ddx;y;dy;ddy;theta;dtheta;ddtheta]
RobotReal      = zeros(9,1);
//we must enter in robot init the value of pose init
RobotReal(1)   = poserInit(1) ;  
RobotReal(4)   = poserInit(2) ;
RobotReal(7)   = poserInit(6) ;
k = 0;
//Andrei
warming        = PERIOD+DOUBLE_SUPPORT;
L              = Lfunction(p,Z);
Lcourant       = L;
STOP_CRITERION = %F;

ecorr =p-pdes;
vrobotprevious = zeros(6,1);
vrobotReal = zeros(6,1);
wVr            = twistMatrix([wMr(:,1:3) [zeros(3,1);1]]);
//----------------------------------------------------------------------//
// Servo LOOP
while( iter < maxiter & STOP_CRITERION==%F)

  iter = iter+1;
  printf('------------------------%d\n',iter)
  e          = p-pdes;               // error
  L          = Lfunction(p,Z);       // compute the interaction matrix
  Lcourant   = L;         // interaction matrix
  psway      = dt*Lcourant*balancement';// instant velocity of points
  intsway    = intsway+psway;
 
  //-----------------------------------------------------------///
  // Save data 
  E            = [E e];  
  normE        = [normE;norm(e)];
  SWAY         = [SWAY psway];
  INTEGRALE    = [INTEGRALE intsway];
    
  //------------------------------------------------------------//
  // compute the camera velocity
  //compute lambda
  lambda = computeGA(lambdaI,lambdaF,betaGA,ecorr);
  
  if(OPT_CORR) 
    if (iter>PERIOD+1+DOUBLE_SUPPORT)
        meanint = mean(INTEGRALE(:,iter-PERIOD:iter-1),2);
        
       
    else
       meanint =  mean(INTEGRALE,2);
  end
     ecorr = e-intsway +meanint;
     
     Ecorr=[Ecorr ecorr];
     normEcorr  = [normEcorr;norm(ecorr)];
     vcam = (-lambda *pinv( Lcourant)*ecorr)';
  else
     vcam = (-lambda *pinv( Lcourant)*e)';
  end
  vcam    = vcam.*(abs(vcam) > 1e-6);
  if(OPT_DEBUG)
    disp('vitesse calculee')
    disp(vcam)
  end
    
    
  //------------------------------------------------------------//
  // change velocity frame
  vrobot       = rVc*vcam';
  vrobot(3)    = 0; // no vertical translation
  vrobot(4)    = 0; // no rotation in x
  vrobot(5)    = 0; // no rotation in y
  
  //------------------------------------------------------------//
  // saturate the velocity
  dv     = 0.02;
  vmax   = [satX,satY,satZ,satThetaX,satThetaY,satThetaZ];
  if(OPT_SAT)
    vrobot = satVelo(vrobot,iter,dv,vmax,warming,OPT_WARMING);
  end
  vcam   = inv(rVc)*vrobot;
 
  //---------------------------------------------------------//
  // Approximation of what andrei code should do
  // input is the robot relative velocity
  // output is the robot velocity in the world frame
  //vrobotReal = [vrobot(1) vrobot(2) 0 0 0 vrobot(3)];
  // the real velocity is not vcam but something based on vDes
  
  //---------- Add sinus -------------------------------------//
  if (OPT_WALK_SIMU)
    vrobotReal   = vrobot;
    vrobotReal(2)= vrobot(2)+0.1*sin(%pi/(PERIOD/2)*(iter-1)) ;
    vrobotRealW  = wVr*vrobotReal;
      
  // ------------Andrei Code -------------------------------------//
  elseif (OPT_WALK_HRP2)
    
    
    [RobotReal,wMr,vrobotReal]=runPG(vrobot,RobotReal,k,wMrinit);
     k = k + 1;
     wVr          = twistMatrix([wMr(:,1:3) [zeros(3,1);1]]);
     vrobotRealW  = wVr*vrobotReal;
    if(OPT_DEBUG)
  
     disp('vitesse repere monde')
     disp(vrobotRealW')
     disp('vitesse donnee par le PG ROBOT REAL')
     disp(RobotReal([2;5;8])')
     disp('vitesse dans repererobot donne par pg')
     disp(vrobotReal')

    end

  else
     vrobotRealW = vrobotW;
  end
  

  //--------------------------------------------------------------//
  if(OPT_DEBUG)
    disp('vitesse robot repere robot real')
    disp(vrobotReal)
  end
  
  if(OPT_DISTURB&iter>DISTURB_ITER&iter<DISTURB_ITER+DISTURB_LENGTH)
      if(OPT_DEBUG)
      disp('robot real avant disturb')
      disp(RobotReal')
     end
     
     RobotReal(3)   = RobotReal(3)+disturbA(1) ;
     RobotReal(6)   = RobotReal(6)+disturbA(2);
     RobotReal(9)   = RobotReal(9)+disturbA(3);
     
     if(OPT_DEBUG)
      disp('robot real')
      disp(RobotReal')
     end
     
  end

  
  
  // deduce vcamReal 
  vcamReal   = inv(rVc)*vrobotReal;
  vcamReal   = vcamReal.*(abs(vcamReal) > 1e-6);
  if(OPT_DEBUG)
    disp('vitesse cam repere cam real')
    disp(vcamReal')
  end


  // position
  if(OPT_WALK_HRP2)
  else
    r1Mr2      = expMapDirectRxRyRz(vrobotReal,dt);
    wMr        = wMr*r1Mr2;
    wMr        = wMr.*(abs(wMr)>1e-10);
  end
  wMc          = wMr*rMc;
  cMo          = inv(wMc)*wMo;       

 
 
  //----------------------------------------------------//
  // update the position    
  //save the 2D position
  if (OPT_DEBUG), disp('MOTION SIMULATION-------------------'),end
  VDes       = [VDes;vcam'];
  VReal      = [VReal;vcamReal'];
  VRobotDes  = [VRobotDes;vrobotReal([1;2;6])'];
  VRobotReal = [VRobotReal;vrobot([1;2;6])'];
  balancement= vcamReal-vcam;
  balancement= balancement';
  
  cP         = changeFrameMire(oP,cMo); // compute the 3D points
  p          = projectMireDirect(cP);  // compute the 2D points
  Z          = cP(3:3:length(cP));     // compute Z
    

  P          = [P p]; //preel
  Pcorr      = [Pcorr ecorr+pdes]; //pcorrected
  poser      = pFromHomogeneousMatrix(wMr);
  posec      = pFromHomogeneousMatrix(wMc); 
  Xc         = [Xc;posec(1)];
  Yc         = [Yc;posec(2)]; 
  Zc         = [Zc;posec(3)];
  Xr         = [Xr;poser(1)];
  Yr         = [Yr;poser(2)];
  Zr         = [Zr;poser(3)];
  XPGr       = [XPGr;RobotReal(1)];
  YPGr       = [YPGr;RobotReal(4)];
  ZPGr       = [ZPGr;RobotReal(7)];

  zmpx       = [zmpx;RobotReal(1)-h*RobotReal(3)/g];
  zmpy       = [zmpy;RobotReal(4)-h*RobotReal(6)/g];
 
  //----------------------------------------------------------//
  // write in files
 
  if(OPT_SAVE)
    mfprintf(fError,'%5.3f \t %5.3f\n',norm(e),norm(ecorr));

    mfprintf(fCamData,'%5.3f \t %5.3f \t %5.3f \t %5.3f \t %5.3f \t %5.3f\t',posec(1),posec(2),posec(3),posec(4),posec(5),posec(6));
    mfprintf(fCamData,'%5.3f \t %5.3f \t %5.3f \t %5.3f \t %5.3f \t %5.3f\n',vcam(1),vcam(2),vcam(3),vcamReal(1),vcamReal(3),vcamReal(5));
 

    mfprintf(fComData,'%5.3f \t %5.3f \t %5.3f \t %5.3f \t %5.3f \t %5.3f\t',poser(1),poser(2),poser(3),poser(4),poser(5),poser(6));   
    mfprintf(fComData,'%5.3f \t %5.3f \t %5.3f \t %5.3f \t %5.3f \t %5.3f\n',vrobot(1),vrobot(2),vrobot(3),vrobotReal(1),vrobotReal(3),vrobotReal(5))  ;
  end
  
 
  //printf('---Fin Boucle---%d\n',iter)
  //printf('VcamReal : ( %f, %f, %f )\n',vcamReal([1,3,5]));
  //printf('VcamDesired :( %f, %f, %f )\n',vcam);
if(OPT_DEBUG & modulo(iter,10)==0)  
hf_2            = createFigure3D(20,"Camera Motion",2);
  Camera3DDrawColor(0.1,wMcfirst,3);// display the first camera
  Camera3DDrawColor(0.1,wMc,3);// display the first camera
  Camera3DDrawColor(0.1,wMcdes,5);// display the first camera
  Mire3DDraw5pts(wP);
  if (iter>=2)
    plot3d(Xc,Yc,Zc);
    plot3d(Xr,Yr,Zr);
  end
  Camera3DDraw(0.1,wMc);
  show_pixmap()
  
   xset("window",50);
  clf 
  hf=scf(50);
  hf.figure_name = "velocity";
  if(size(VDes,1)>1)
    plot(VDes(:,1),'r-.');
    plot(VDes(:,2),'g-.');
    plot(VDes(:,3),'b-.');
    plot(VDes(:,4),'c-.');
    plot(VDes(:,5),'m-.');
    plot(VDes(:,6),'k-.');
    
    plot(VReal(:,1),'r');
    plot(VReal(:,2),'g');
    plot(VReal(:,3),'b');
    plot(VReal(:,4),'c');
    plot(VReal(:,5),'m');
    plot(VReal(:,6),'k');
    show_pixmap()
  end


  xset("window",51);
  clf 
  hf=scf(51);
  hf.figure_name = "velocity robot";
  if(size(VDes,1)>1)
    plot(VRobotDes(:,1),'r-.');
    plot(VRobotDes(:,2),'g-.');
    plot(VRobotDes(:,3),'b-.');
    plot(VRobotReal(:,1),'r');
    plot(VRobotReal(:,2),'g');
    plot(VRobotReal(:,3),'b');
    show_pixmap()
end
end
end;

if (OPT_DEBUG), disp('END OF THE SERVO LOOP-------------------'),end
 

  
// ---------------------------------------------//
// Display the results 
if(OPT_DISPLAY)
  xl             = [-0.11;-0.11]
  xu             = [0.3; 0.11]
  if (OPT_DEBUG), disp('DISPLAY THE RESULTS-------------------'),end
  hf_1           = createPlanImage(1,xl,xu,"Point 2D");
  mire2DDraw(pdes,cote,5);
  mire2DDraw(pinit,cote,3);
  if (iter>2)
    for i=1:Nbpts
      plot(Pcorr((i-1)*2+1,:),Pcorr((i-1)*2+2,:),'b')
      plot(P((i-1)*2+1,:),P((i-1)*2+2,:),'r')
      //plot(Plin((i-1)*2+1,:),Plin((i-1)*2+2,:),'g')
      //plot(Pav((i-1)*2+1,:),Pav((i-1)*2+2,:),'y')
    end  
  end
  show_pixmap();
  
  hf_2            = createFigure3D(2,"Camera Motion",2);
  Camera3DDrawColor(0.1,wMcfirst,3);// display the first camera
  Camera3DDrawColor(0.1,wMc,3);// display the first camera
  Camera3DDrawColor(0.1,wMcdes,5);// display the first camera
  Mire3DDraw5pts(wP);
  if (iter>=2)
    plot3d(Xc,Yc,Zc);
    plot3d(Xr,Yr,Zr);
  end
  Camera3DDraw(0.1,wMc);
  show_pixmap()
  
  xset("window",4);
  clf 
  hf=scf(4);
  hf.figure_name = "Error(plain lines)/correction (doted lines)";
  //Thres = threshold*ones(length(norme),1);
  
  if(iter>2)
    //plot(Thres,'k-.')
    plot(E');
    plot(Ecorr','-.')
  end 
  
  if (OPT_WALK_HRP2)
    xset("window",3);
    //AxeZ2DDraw(0.02,wMc);
    hf=scf(3);
    hf.figure_name = "Top view of the scene";
    plotframe([-0.2,-0.2,1.6,0.8]);
    plotFeet(FeetDown,RealRightFootEdges,RealLeftFootEdges);
    if (iter>2)
      plot(XPGr,YPGr,'g');
      plot(zmpx,zmpy,'r')
    end 
    show_pixmap();
  end
  
  
  xset("window",5);
  clf 
  hf=scf(5);
  hf.figure_name = "velocity";
  if(size(VDes,1)>1)
    plot(VDes(:,1),'r-.');
    plot(VDes(:,2),'g-.');
    plot(VDes(:,3),'b-.');
    plot(VDes(:,4),'c-.');
    plot(VDes(:,5),'m-.');
    plot(VDes(:,6),'k-.');
    
    plot(VReal(:,1),'r');
    plot(VReal(:,2),'g');
    plot(VReal(:,3),'b');
    plot(VReal(:,4),'c');
    plot(VReal(:,5),'m');
    plot(VReal(:,6),'k');
    show_pixmap()
  end
  
  xset("window",6);
  clf 
  hf=scf(6);
  hf.figure_name = "velocity difference";  
  if(size(VReal,1)>1)
    plot(VReal(:,1)-VDes(:,1),'r');
    plot(VReal(:,2)-VDes(:,2),'g');
    plot(VReal(:,3)-VDes(:,3),'b');
    plot(VReal(:,4)-VDes(:,4),'c');
    plot(VReal(:,5)-VDes(:,5),'m');
    plot(VReal(:,6)-VDes(:,6),'k');
    
    show_pixmap()
  end 

  xset("window",7);
  clf 
  hf=scf(7);
  hf.figure_name = "E";  
  if(size(E,1)>1)
    plot(normE);
    plot(normEcorr,'-.')
    show_pixmap()
  end 
  
  
  zmp=[zmpx,zmpy];
  
  xset("window",10);
  clf 
  hf=scf(10);
  hf.figure_name = "SUM SWAY MOTION";  
  if(size(SWAY,1)>1)
    plot(SWAY(1,:),'r');
    plot(SWAY(2,:),'g');
    plot(SWAY(3,:),'y');
    plot(SWAY(4,:),'c');
    plot(SWAY(5,:),'m');
    plot(SWAY(6,:),'k');
    show_pixmap()
  end
end
  xset("window",11);
  clf 
  hf=scf(11);
  hf.figure_name = "robot real";
  if(size(VDes,1)>1)
    plot(VRobotReal(:,1),'r');
    plot(VRobotReal(:,2),'g');
    plot(VRobotReal(:,3),'b');
    show_pixmap()
  end  

  
if(OPT_SAVE)
  fprintfMat(pathExp+'zmp.dat',zmp,'%5.2f');
if(OPT_WALK_HRP2)
  fprintfMat(pathExp+'RealRightFootEdges.dat',RealRightFootEdges,'%5.2f');
  fprintfMat(pathExp+'RealLeftFootEdges.dat',RealLeftFootEdges,'%5.2f');
  fprintfMat(pathExp+'FeetDown.dat',FeetDown,'%5.2f');
if(OPT_DISPLAY)
  end
  xs2fig(1,pathExp+'point2D.fig')
  xs2fig(2,pathExp+'cameraPosition.fig')
  xs2fig(3,pathExp+'topview.fig')
  xs2fig(4,pathExp+'error.fig')
  xs2fig(5,pathExp+'velocity.fig')
  xs2fig(6,pathExp+'errPosition.fig')
  xs2fig(7,pathExp+'normerror.fig')
  xs2fig(10,pathExp+'sway.fig')
  xs2fig(11,pathExp+'velocityWorld.fig')
 end
  mclose(fError);
  mclose(fCamData);
  mclose(fComData);
  scf();
end 

if(OPT_DISPLAY)
xset("pixmap",0);
end


endfunction


