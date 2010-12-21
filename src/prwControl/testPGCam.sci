// this code test the asser visu 
// given as an input to Andreis function
// for step computing
// 
//
// test another correction
//;exec('main/testAVrobotErreurMarche.sce');
function isTestPGFileLoaded()
disp('test PG file is loaded')
endfunction
  

function testPGCam(pathExp)
disp("les fichier seront enreigstres sous path Exp:")
disp(pathExp)


// -- Pose init and final of the robot in the world frame
h              = 0.8;               // CoM height
g              = 9.81;              // gravity
maxiter        = 100;               //
//
poserInit      = [0 0  h -%pi 0 0];
wMrinit        = homogeneousMatrixFromPos(poserInit);
[rMc,rVc]      = loadHRP2camCom();
wMcinit        = wMrinit*rMc;

dt             = .1;
cote           = .01;              // size of the target dot
//-------- Matrix Position -------------//

wMr         = eye(4,4);
iter        = 0;
Xr          = [ wMr(1,4)];
Yr          = [ wMr(2,4)];
Zr          = [ wMr(3,4)];
zmpx        = [];
zmpy        = [];
wVr         = twistMatrix(wMr);

//state of the robot [x;dx;ddx;y;dy;ddy;theta;dtheta;ddtheta]
RobotReal    = zeros(9,1);
//we must enter in robot init the value of pose init
RobotReal(1) = poserInit(1) ;  
RobotReal(4) = poserInit(2) ;
RobotReal(7) = poserInit(6) ;


// Servo LOOP
Vdes   =[];
Vi     =[];
Vo     =[];
hf_2            = createFigure3D(2,"position du com",2);
  
k = 0;
while( iter < maxiter)
  iter   = iter+1;
  printf('------------------------%d\n',iter)
  
  if (iter>50&iter<60 )
    vcam = [0.1 0 0.1 0 0 0.1 0];
  else
    vcam = [0.1 0 0.1 0 0 0];
  end

  disp('desired cam velocity')
  disp(vcam)
  vrobot = rVc*vcam';
  disp('desired velocity')
  disp(vrobot)
  
  [RobotReal,wMr,vrobotrobot]=runPG(vrobot',RobotReal,k,wMrinit);
 
  //[wMr,vout]            = computePGoutput(RobotReal);
  //disp('current position avant PG dans monde robot')
  //disp(pFromHomogeneousMatrix(wMr)')
  //disp('current position avant PG dans monde')
  //disp(pFromHomogeneousMatrix(wMrinit*wMr)')
  //vinput                = computePGinput(vrobot',wMrinit*wMr);
  //disp('vitesse input')
  //disp(vinput)
  //[RobotReal, RobotPrw] = WalkingTrajectory_withFP(vinput,RobotReal,k);
  //[wMr,vrobotrobot]     = computePGoutput(RobotReal);
 
  k=k+1;
 
  disp('resulting velocity')
  disp(vrobotrobot')     
  
  vcamreal=inv(rVc)*vrobotrobot;
  disp('vitesse real cam')
  disp(vcamreal)

  wMc=wMr*rMc;
 
  //----------------------------------------------------//
  // update the position    
  //save the 2D position
  poser       = pFromHomogeneousMatrix(wMr);
  Xr          = [Xr;poser(1)];
  Yr          = [Yr;poser(2)];
  Zr          = [Zr;poser(3)];
  zmpx        = [zmpx;poser(1)-h*RobotReal(3)/g];
  zmpy        = [zmpy;poser(2)-h*RobotReal(6)/g];
  Vdes        = [Vdes;vrobot([1,2,6])];
  Vi          = [Vi;vrobotrobot([1,2,6])'];
  xset("window",2)
  Camera3DDraw(0.1,wMr);// display the first camera
  Camera3DDraw(0.1,wMc);// display the first camera
  plot3d(Xr,Yr,Zr);
  show_pixmap()
  
end

xset("window",5)
plot(Vdes)
xset("window",6)
plot(Vi)
//xset("window",3)
//plot(Vo)

endfunction

function isTestPGLoaded()
disp('test PG is loaded')
endfunction
  
