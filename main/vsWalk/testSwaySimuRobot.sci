// this code test the asser visu 
// given as an input to Andreis function
// for step computing
// 
//
// test another correction
//;exec('main/testAVrobotErreurMarche.sce');

function testSwayRobot(OPT_CORR,OPT_L_COURANT,pathExp)

disp('Entering  testSwayRobot')
maxiter        = 200;
// -- Pose init and final of the robot in the world frame
posecMoInit    = [ 0.1, 0.0, 1.1, 0*%pi/180, 0*%pi/180, 0*%pi/180]; 
posecMoDes     = [ 0.0, 0.0, 1, 0*%pi/180, 0*%pi/180, 0*%pi/180 ]; 
                                             
OPT_DISPLAY    = %T; 
OPT_SAT        = %F; // velocity saturation
OPT_WARMING    = %F; // transient affine increase of the velocity during the 

// -- For Saturation
warming        = 16;
dv             = 0.02;
satX           = 0.2;             // value of the saturation x translation
satZ           = 0.2;              // y translation
satTheta       = 0.2;              // Z rotation
vmax           = [satX satZ satTheta];

// -- For Simulation
dt             = .1;              // time delay between two command
cote           = .01;              // size of the target dot

// -- For Visual Servoing
lambda         = 1;               // aservissement gain
threshold      = .001 ;             // threshold on the error to stop


//-------- Matrix Position -------------//
cMo            = homogeneousMatrixFromPos(posecMoInit);
cdesMo         = homogeneousMatrixFromPos(posecMoDes);
oMcdes         = inv(cdesMo);
// set to 0 the values that are < small
tooSmall       = 1e-10;
sat=60;

//-------- Create the target -----------//
a              = .1;               // related to the target size
Nbpts          = 5;                // nb points 
oP             = mire5points (a);  // target
cP             = changeFrameMire(oP,cMo); 

p              = projectMireDirect(cP);
pinit          = p;               
Z              = cP(3:3:length(cP)); 
cdesP          = changeFrameMire(oP,cdesMo); 
pdes           = projectMireDirect(cdesP);
Zdes           = cdesP(3:3:length(cP)); 
corre          = 0 ;
disp('pdes')
disp(pdes')

//----------------- VISUAL SERVOING LOOP -----------------//
Lfunction = matIntMireC;
er = 10; // error value init
iter        = 0;
vcam        = zeros(1,6);
vcamReal    = zeros(1,6);
oMc         = inv(cMo); 
oMcfirst    = oMc;
Xc          = [ oMc(1,4)];
Yc          = [ oMc(2,4)];
Zc          = [ oMc(3,4)];
PERIOD      = 16;

//------------------//
// to estimate de/dt

 
integrale   = zeros(length(p),1);
Ldes        = Lfunction(pdes,Zdes);

STOP_CRITERION = %F;
VDes        =[];
VReal       =[];
P           =[];
Pcorr       =[];
E           =[];
Ecorr       =[];
NORME       =[];
INTEGRALE   =[];
NORMECORR   =[];
MEANE       =[];
//----------------------------------//
// Servo LOOP
// vcam(6) =0.1;


while( iter < maxiter)// & STOP_CRITERION==%F)

  iter = iter+1;
  printf('----------------------%d\n',iter)
  e          = p-pdes;                       // compute the error  
  //---------------------------///
  // Compute the correction 
  if(OPT_L_COURANT)
    L        = Lfunction(p,Z);              // compute the interaction matrix
  else
    L        = Ldes;
  end 
  

  if (OPT_CORR)
    // estimation of the motion of the feature due to the sway
    corre      = dt*L*(vcamReal-vcam)';
    // sum with the previous values
    integrale  = integrale + corre ;
    if (iter>PERIOD+1)
      meanint=mean(INTEGRALE(:,iter-PERIOD:iter-1),2)
    else
      meanint=mean(INTEGRALE,2);
    end
    erreurCourante=e-integrale+meanint;
    
    // save the current mean 
    MEANE     =[MEANE meanint];
    normecorr = norm(erreurCourante);
    NORMECORR =[NORMECORR normecorr];
  else
    erreurCourante=e;
  end 
  vcam       = computeVelocity(lambda, L,erreurCourante);
  //-----SIMU ROBOT----//
  //deduce vcamReal   
  a = satVelo(vcam,iter,dv,vmax,warming,OPT_WARMING);
  vcamReal =a;
  
  vcamReal(1)= a(1)+0.1*sin(%pi/(PERIOD/2)*(iter-1));
  vcamReal(2)=0;
  vcamReal(4)=0;
  vcamReal(6)=0;
 
 
  //----------------------------------------------------//
  // update the position    
  //save the 2D position
  //-----------------------------------------------------//
  // Update variables
  c1Mc2      = expMapDirectRxRyRz(vcamReal,dt);
  cMo        = inv(c1Mc2)*cMo;
  cMo        = cMo.*(abs(cMo)>1e-10);
  VDes       =[VDes;vcam];
  VReal      =[VReal;vcamReal];
  balancement= vcamReal-vcam;
  cP         = changeFrameMire(oP,cMo); // compute the 3D points
  p          = projectMireDirect(cP);   // compute the 2D points
  Z          = cP(3:3:length(cP));      // compute Z
  P          = [P p]; //preel
  Pcorr      = [Pcorr erreurCourante+pdes]; //pcorrected
  E          = [E e]; //error
  Ecorr      = [Ecorr erreurCourante]; 
  INTEGRALE  = [INTEGRALE integrale];
  posec      = pFromHomogeneousMatrix(inv(cMo));
  Xc         = [Xc;posec(1)];
  Yc         = [Yc;posec(2)]; 
  Zc         = [Zc;posec(3)];
  disp('Pose--->')
  disp(posec')
  disp('----')
  
  NORME      = [NORME;norm(e)];

  if(norm(e)<threshold) 
    STOP_CRITERION=%T;
  end 
  
  
  
end;

  
// ---------------------------------------------//
// Display the results 
if(OPT_DISPLAY)
xl             = [-0.3;-0.3]
xu             = [0.3; 0.3]

hf_1           = createPlanImage(1,xl,xu,"Point 2D");
axe=gca();
axe.data_bounds=[0,-0.5,0;1,0.5,1]; 
mire2DDraw(pdes,cote*2,5);
mire2DDraw(pinit,cote*2,3);
if (iter>2)
  for i=1:Nbpts
    if(OPT_CORR)
      plot(Pcorr((i-1)*2+1,:),Pcorr((i-1)*2+2,:),'b')
    end
    plot(P((i-1)*2+1,:),P((i-1)*2+2,:),'r')
  end  
end
show_pixmap();

hf_2            = createFigure3D(2,"Camera Motion",1);
Camera3DDrawColor(0.1,oMcfirst,3);// display the first camera
Camera3DDrawColor(0.1,oMc,3);// display the first camera
Camera3DDrawColor(0.1,oMcdes,5);// display the first camera
Mire3DDraw5pts(oP);
if (iter>=2)
  plot3d(Xc,Yc,Zc);
 end
Camera3DDraw(0.1,oMc);
show_pixmap()

xset("window",4);
clf 
hf=scf(4);
hf.figure_name = "Error(plain lines)/correction (doted lines)";
//Thres = threshold*ones(length(norme),1);
if(iter>2)
  //plot(Thres,'k-.')
  plot(E','r');
  plot(Ecorr','g');
  plot(MEANE','b')
end 
  



xset("window",5);
clf 
hf=scf(5);
hf.figure_name = "velocity";
if(size(VDes,1)>1)
  plot(VDes(:,1),'r-.');
  plot(VDes(:,2),'g-.');
  plot(VDes(:,3),'b-.');
  plot(VDes(:,4),'y-.');
  plot(VDes(:,5),'c-.');
  plot(VDes(:,6),'m-.');
  plot(VReal(:,1),'r');
  plot(VReal(:,2),'g');
  plot(VReal(:,3),'b');
  plot(VReal(:,4),'y');
  plot(VReal(:,5),'c');
  plot(VReal(:,6),'m');
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
  plot(VReal(:,4)-VDes(:,4),'y');
  plot(VReal(:,5)-VDes(:,5),'c');
  plot(VReal(:,6)-VDes(:,6),'m');
  show_pixmap()
end 

xset("window",7);
clf 
hf=scf(7);
hf.figure_name = "terme correctif";  
if(size(INTEGRALE,1)>1)
  plot(INTEGRALE');
end 
  
xset("window",8);
clf 
hf=scf(8);
hf.figure_name = "norme de l erreur";  
if(size(NORME,1)>1)
  plot(NORME);
  if (OPT_CORR)
    plot(NORMECORR,'r-.');
  end
end 
end  


//f2dPoints = mopen(pathExp+'/2Dpoints.dat');
//m/fprintf(f2dPoints,'#realpoint / corrected points');
//for i=1:maxiter 
//  for k=1:Nbpts
//    mfprintf(f2dPoints,'%5.3f\t%5.3f\t',P((i-1)));
//  end  
//  mfprintf(f2dPoints,'\n');
//end

xs2fig(1,pathExp+'point2D.fig')
xs2fig(2,pathExp+'cameraPosition.fig')
xs2fig(4,pathExp+'error.fig')
xs2fig(5,pathExp+'velocity.fig')
xs2fig(6,pathExp+'diffVelo.fig')  
xs2fig(7,pathExp+'sumError.fig')
xs2fig(8,pathExp+'normError.fig')





endfunction


function isTestSwayRobotLoaded()
 disp('TestSwayRobot is Loaded !')  
endfunction