// this code test the asser visu 
// given as an input to Andreis function
// for step computing
//;exec('main/testAVrobotErreurMarche.sce');


maxiter        = 150;               //
h              = 0.8;               // CoM height
g              = 9.81;              // gravity

// -- Pose init and final of the robot in the world frame
poserInit      = [0 0 h -%pi 0 0*%pi/180 ]; // CoM init pose in the world reference frame
//poserFinal     = [1 0.6 h -%pi 0 0*%pi/180];
poserFinal     = [1 0.1 h -%pi 0 0*%pi/180 ]; // CoM final pose in the world reference frame
posecMoInit    = [ 0 0 1.5 0 0 0 ]; // Object position in the init camera frame

// -- For Simulation
dt             = .1;               // time delay between two command
cote           = .01;              // size of the target dot
// -- For Visual Servoing
lambda         = .5;               // aservissement gain
threshold      = .01 ;             // threshold on the error to stop
OPT_SAT        = %T;               // velocity saturation
satX           = 0.15;             // value of the saturation x translation
satZ           = 0.2;              // y translation
satTheta       = 0.2;              // Z rotation
OPT_WARMING    = %F;               // affine increase of the velocity during the 
                                   // the double support pahse
OPT_CORR       = %T;               // correction of the error
// ----- For Walking
OPT_WALK_SIMU  = %T;               // sinusoidal addition
OPT_WALK_HRP2  = %F;               // walk simulation 
OPT_DISTURB    = %F;               // addition of a disturbance
disturb=[0.4,0.1,0.1,0,0,0];
// ----- User Option
pathExp        = 'tmp/'            // out put path
OPT_DEBUG      = %F; 
OPT_DISPLAY    = %F;
OPT_SAVE       = %F;


if(OPT_WALK_HRP2)
  
setActuationSamplingPeriod(2.5e-3);
setActuationSamplingDelay(0.1e-3);
setNbActuators(30);
end



//-------- Matrix Position -------------//
wMr            = homogeneousMatrixFromPos(poserInit);
wMrdes         = homogeneousMatrixFromPos(poserFinal);
cMo            = homogeneousMatrixFromPos(posecMoInit);
[rMc,rVc]      = loadHRP2camCom();
wMc            = wMr*rMc;
wMcdes         = wMrdes*rMc
cdesMc         = inv(wMcdes)*wMc;
cdesMo         = cdesMc*cMo;
wMo            = wMc*cMo;         

// set to 0 the values that are < small
tooSmall       = 1e-10;
wMo            = wMo.*(abs(wMo)>tooSmall);
wMc            = wMc.*(abs(wMc)>tooSmall);
wMcdes         = wMcdes.*(abs(wMcdes)>tooSmall);

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

//------create the figures-------------//
xl             = [-0.3;-0.3]
xu             = [0.3; 0.3]

//Image plane
hf_1           = createPlanImage(1,xl,xu,"Point 2D");
mire2DDraw(p,cote,3);
show_pixmap()
mire2DDraw(pdes,cote,5);
show_pixmap()

// 3D view
wMcfirst       = wMc;
wMrfirst       = wMr;
hf_2           = createFigure3D(2,"Camera Motion",2);
Camera3DDraw(0.3,wMcfirst);       // display the first camera
Camera3DDraw(0.3,wMcdes);         // display the desired camera
Camera3DDraw(0.3,wMrfirst);       // display the first com
Camera3DDraw(0.3,wMrdes);         // display the desired com
Mire3DDraw5pts(wP);
show_pixmap()





//----------------- VISUAL SERVOING LOOP -----------------//
Lfunction = matIntMireC;
vcamReal  = zeros(1,6); // real camera montion
vN        = zeros(1,6);
vT        = zeros(1,6);

er = 10; // error value init
iter        = 0;
vcam        = zeros(1,3);
esav        = zeros(length(p),1);
xsav        = zeros(length(p),1);
mu          = 0.8;
E           = [];
vsav        = zeros(3,1);
vcamReal    = zeros(1,6);

L           = Lfunction(p,Z); 

Xc          = [ wMc(1,4)];
Yc          = [ wMc(2,4)];
Zc          = [ wMc(3,4)];
Xr          = [ wMr(1,4)];
Yr          = [ wMr(2,4)];
Zr          = [ wMr(3,4)];
P           = [pinit];
Pcorr       = [pinit];
Plin        = [pinit];
Pav         = [pinit];
zmpx        = [];
zmpy        = [];
normMod     = [];
norme       = [];
velocity    = [];
velocity2   = [];
velocity3   = [];

psav        = 0;
pav         = 0;

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
RobotReal    = zeros(9,1);
//we must enter in robot init the value of pose init
RobotReal(1) = poserInit(1) ;  
RobotReal(4) = poserInit(2) ;
RobotReal(7) = poserInit(6) ;
k = 0;
//Andrei
warming =16;
// launch the servo
while( iter < maxiter)

  iter = iter+1;
  disp('------------------------',iter)
  if(OPT_DISTURB & iter==50)
     
     c1Mc2 = homogeneousMatrixFromPos(disturb);
     cMo   = c1Mc2*cMo
      
  end  
 
  cP         = changeFrameMire(oP,cMo);                    // compute the 3D points
  p          = projectMireDirect(cP);                      // compute the 2D points
  Z          = cP(3:3:length(cP));                         // compute Z
 
  
  e          = p-pdes;                                     // compute the error  
  norme      = [norme;norm(e)];  
   
  esav       = esav + dt*L(:,[1,3,5])*(vcamReal([1,3,5])-vcam)';
  psav       = psav + dt*L(:,[1,3,5])*(vcamReal([1,3,5]))';
  pav        = pav  + dt*L(:,[1,3,5])*(vcam)';
  E =[E;norm(e-esav)'];
  L          = Lfunction(p,Z);                             // compute the interaction matrix
  
  if (OPT_DEBUG)
    disp('COMPUTE VCAM-------------------')
  end

  if(OPT_CORR) 
     vcam     = computeVelocity(lambda, L(:,[1,3,5]),e-esav);
  else
     vcam     = computeVelocity(lambda, L(:,[1,3,5]),e);
  end

  if (OPT_SAT)
     dv    = 0.02; 
     vmax  = [satX,satZ,satTheta];
     if(iter<warming&OPT_WARMING)
        vmax = vmax*(iter-1)/warming ;
     end
     v1  = vmax-dv;
     v2  = vmax+dv;
     avcam = abs(vcam);
     // for all the coeff
     fac = 1;
     for i=1:3
       fac = min(abs(fac),vmax(i)/(avcam(i)+%eps));
       if( (v1(i)<=avcam(i)) & (avcam(i) <= v2(i)) )
         nout = 1/(2*dv*avcam(i))  *  ( (avcam(i)-v1(i))*vmax(i)+(v2(i)-avcam(i))*v1(i));
         fac  = min(abs(fac),abs(nout));
       end     
     end
     vcam = vcam*fac;
  end
  
  
  vcam         = vcam.*(abs(vcam) > 1e-6);
  vrobot       = convertVelocityCamRobot(vcam,rVc) ;
  wVr          = twistMatrix([wMr(:,1:3) [zeros(3,1);1]]);
  vrobotW      = wVr*[vrobot(1) vrobot(2) 0 0 0 vrobot(3)]';
  vrobotAndrei = [vrobot(1) vrobot(2) vrobotW(6)];
  


 
   //---------Approximation of what andrei code should do---------//
  // input is the robot relative velocity
  // output is the robot velocity in the world frame
  //vrobotReal = [vrobot(1) vrobot(2) 0 0 0 vrobot(3)];
  // the real velocity is not vcam but something based on vDes
  if (OPT_WALK_SIMU)
      vrobotReal=[vrobot(1) vrobot(2)+0.2*sin(%pi/(16/2)*(iter-1)) 0 0 0 vrobot(3)] ;
      vrobotRealW= wVr*vrobotReal';
      vrobotRealW= [vrobotRealW(1) vrobotRealW(2) vrobotRealW(6)];
   
  // ------------Andrei Code -------------------------------------//
  elseif (OPT_WALK_HRP2)
     [RobotReal, RobotPrw] = WalkingTrajectory_withFP(vrobotAndrei,RobotReal,k);
     k = k + 1;
     vrobotRealW = RobotReal([2;5;8]);
  else
     vrobotRealW = vrobotW([1,2,6]);
  end
  //--------------------------------------------------------------//
  
  
  vrobotReal = (inv(wVr)*[vrobotRealW(1) vrobotRealW(2) 0 0 0 vrobotRealW(3)]')'; 
  vrobotReal = vrobotReal.*(abs(vrobotReal) > 1e-6);
  r1Mr2      = expMapDirectRxRyRz(vrobotReal,dt);
  wMr        = wMr*r1Mr2;
  wMr        = wMr.*(abs(wMr)>1e-10);
  
  if(OPT_DEBUG)
    disp('Vrobot desired/andrei/vrobotMonde/vrobotreal')
    disp(vrobot);
    disp(vrobotAndrei);
    disp(vrobotRealW');
    disp(vrobotReal);
  end
  // deduce vcamReal   
  vcamReal   = convertVelocityRobotCam(vrobotReal,rVc) ;
  vcamReal   = [vcamReal(1),0,vcamReal(2),0,vcamReal(3),0 ]; 
  vcamReal   = vcamReal.*(abs(vcamReal) > 1e-6);
  
  if (iter ==20&OPT_DISTURB)
      vcamReal(1) = vcamReal(1)+0.5;
  end
 
  if(OPT_DEBUG)
    disp('Vcam desired/real')
    disp(vcam);
    disp(vcamReal([1,3,5]));
  end
  
  c1Mc2      = expMapDirectRxRyRz(vcamReal,dt);
  cMo        = inv(c1Mc2)*cMo;
  cMo        = cMo.*(abs(cMo)>1e-10);
  wMc        = wMo*inv(cMo);
  wMc        = wMc.*(abs(wMc)>1e-10);
  
 // update the position    
   //save the 2D position
  P          = [P p]; //preel
  Pcorr      = [Pcorr p-esav]; //pcorrected
  Plin       = [Plin pinit+psav];
  Pav        = [Pav pinit+pav];
  
  posec      = pFromHomogeneousMatrix(wMc);
  poser      = pFromHomogeneousMatrix(wMr);
  Xc         = [Xc;posec(1)];
  Yc         = [Yc;posec(2)]; 
  Zc         = [Zc;posec(3)];
  Xr         = [Xr;poser(1)];
  Yr         = [Yr;poser(2)];
  Zr         = [Zr;poser(3)];

  zmpx       = [zmpx;posec(1)-h*RobotReal(3)/g];
  zmpy       = [zmpy;posec(2)-h*RobotReal(6)/g];
 
  //---- Write in files--------------------------------//
  if(OPT_SAVE)
    mfprintf(fError,'%5.3f \t %5.3f\n',norm(e),norm(e-esav));

    mfprintf(fCamData,'%5.3f \t %5.3f \t %5.3f \t %5.3f \t %5.3f \t %5.3f\t',posec(1),posec(2),posec(3),posec(4),posec(5),posec(6));
    mfprintf(fCamData,'%5.3f \t %5.3f \t %5.3f \t %5.3f \t %5.3f \t %5.3f\n',vcam(1),vcam(2),vcam(3),vcamReal(1),vcamReal(3),vcamReal(5));
 

    mfprintf(fComData,'%5.3f \t %5.3f \t %5.3f \t %5.3f \t %5.3f \t %5.3f\t',poser(1),poser(2),poser(3),poser(4),poser(5),poser(6));   
    mfprintf(fComData,'%5.3f \t %5.3f \t %5.3f \t %5.3f \t %5.3f \t %5.3f\n',vrobot(1),vrobot(2),vrobot(3),vrobotReal(1),vrobotReal(3),vrobotReal(5))  ;
  end
  


 
end;


  
  // --- Displays--------------------------------------//
  //xset("window",1); 
  hf_1           = createPlanImage(1,xl,xu,"Point 2D");
  mire2DDraw(pdes,cote*2,5);
  if (iter>2)
      for i=1:Nbpts
        plot(Pcorr((i-1)*2+1,:),Pcorr((i-1)*2+2,:),'b')
        plot(P((i-1)*2+1,:),P((i-1)*2+2,:),'r')
        plot(Plin((i-1)*2+1,:),Plin((i-1)*2+2,:),'g')
        plot(Pav((i-1)*2+1,:),Pav((i-1)*2+2,:),'y')
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
  Thres = threshold*ones(length(norme),1);
  
  if(iter>2)
  plot(Thres,'k-.')
  plot(norme,'g');
  plot(E,'g-.')
  end 
  
  if(OPT_WALK_HRP2)
  xset("window",3);
  //AxeZ2DDraw(0.02,wMc);
  plotframe([-0.3,-0.5,1.5,0.4]);
  plotFeet(FeetDown,RealRightFootEdges,RealLeftFootEdges);
  if (iter>2)
    plot(Xc,Yc,'g');
    plot(zmpx,zmpy,'r')
  end 
  show_pixmap();
  end
  xset("window",5);
    clf 
  velocity =[velocity;vcam];
  velocity2 =[velocity2;vcamReal([1,3,5])];
  if(size(velocity,1)>1)
    plot(velocity(:,1),'r-.');
    plot(velocity(:,2),'g-.');
    plot(velocity(:,3),'b-.');
    plot(velocity2(:,1),'r');
    plot(velocity2(:,2),'g');
    plot(velocity2(:,3),'b');
    show_pixmap()
  end

  xset("window",6);
    clf 
  if(size(velocity2,1)>1)
    plot(velocity2(:,1)-velocity(:,1),'r');
    plot(velocity2(:,2)-velocity(:,2),'g');
    plot(velocity2(:,3)-velocity(:,3),'b');
    show_pixmap()
  end 
 

  xset("window",7);
    clf 
  if(size(E,1)>1)
    plot2d(E);
    show_pixmap()
  end 
 
  
  if (iter ==20&OPT_DISTURB)
      vcamReal(1) = vcam(1)-0.1;
  end
 
zmp=[zmpx,zmpy];

disp('-------The End------')
if(OPT_SAVE)
  fprintfMat(pathExp+'zmp.dat',zmp,'%5.2f');
  fprintfMat(pathExp+'RealRightFootEdges.dat',RealRightFootEdges,'%5.2f');
  fprintfMat(pathExp+'RealLeftFootEdges.dat',RealLeftFootEdges,'%5.2f');
  fprintfMat(pathExp+'FeetDown.dat',FeetDown,'%5.2f');
  xs2fig(1,pathExp+'point2D.fig')
  xs2fig(2,pathExp+'cameraPosition.fig')
  xs2fig(3,pathExp+'topview.fig')
  xs2fig(4,pathExp+'error.fig')
  xs2fig(5,pathExp+'velocity.fig')
  xs2fig(6,pathExp+'errPosition.fig')
  mclose(fError);
  mclose(fCamData);
  mclose(fComData);
  scf();
end 
xset("pixmap",0);
disp('pause before ending')
pause
delete(hf_1);  
delete(hf_2);
delete(hf_3);
delete(hf_4);  
delete(hf_5);
delete(hf_6);
delete(hf_7);


