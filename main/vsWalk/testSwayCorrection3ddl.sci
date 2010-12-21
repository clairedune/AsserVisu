// this code test the asser visu 
// given as an input to Andreis function
// for step computing
// 
//
// test another correction
//;exec('main/testAVrobotErreurMarche.sce');


function testSway3ddl(OPT_CORR,OPT_L_COURANT)
disp('Entering  testSway3ddl')
maxiter        = 100;
OPT_DISPLAY    = %T; 
OPT_SAT        = %F;               // velocity saturation
OPT_WARMING    = %F;               // transient affine increase of the velocity during the 
warming        = 16;
dv             = 0.02;
satX           = 0.2;             // value of the saturation x translation
satZ           = 0.2;              // y translation
satTheta       = 0.2;              // Z rotation
vmax           = [satX satZ satTheta];
             
// -- Pose init and final of the robot in the world frame
posecMoInit    = [ 0.0 0 1.5  0*%pi/180 0 0]; // Object position in the init camera frame
posecMoDes     = [ 0.1 0 1 0 0*%pi/180 0 ]; // Object position in the init camera frame

// -- For Simulation
dt             = .1;              // time delay between two command
cote           = .01;              // size of the target dot
// -- For Visual Servoing
lambda         = 0.6;               // aservissement gain
threshold      = .001 ;             // threshold on the error to stop
BIAIS          = 0 ;
//OPT_CORR       = %T;

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
vcam        = zeros(1,3);
vcamReal    = zeros(1,3);
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
//----------------------------------------------------------------------//
// Servo LOOP
//vcam(6) =0.1;
while( iter < maxiter)// & STOP_CRITERION==%F)

  iter = iter+1;
  printf('----------------------%d\n',iter)
  
  e          = p-pdes;                       // compute the error  
  disp('p')
  disp(p')
  disp('e')
  disp(e')
  //-----------------------------------------------------------///
  // Compute the correction 
  if(OPT_L_COURANT)
    L        = Lfunction(p,Z);              // compute the interaction matrix
  else
    L        = Ldes;
  end 
  L = L(:,[1,3,5]);
  corre      = dt*L*(vcamReal-vcam)';
  integrale  = integrale + corre ;
  
  
  
  if (OPT_CORR)
     if (iter>PERIOD+1)
        meanint=mean(INTEGRALE(:,iter-PERIOD:iter-1),2)
     else
       meanint=mean(INTEGRALE,2);
     end
    
    erreurCourante=e-integrale+meanint;
    MEANE     =[MEANE meanint];
    normecorr = norm(erreurCourante);
    NORMECORR =[NORMECORR normecorr];
  else
    erreurCourante=e;
  end 
  vcam       = computeVelocity(lambda, L,erreurCourante);
  disp(vcam)
  // vcam(3)    = 0 ;
 // vcam(4)    = 0 ;
 // vcam(5)    = 0 ;
  
   //-----SIMU ROBOT----//
  //deduce vcamReal   
  a = satVelo(vcam,iter,dv,vmax,warming,OPT_WARMING);
  vcamReal(1)= a(1)+0.1*sin(%pi/(PERIOD/2)*(iter-1));
   
  c1Mc2      = expMapDirectRxRyRz([vcamReal(1)  0 vcamReal(2) 0 0 vcamReal(3) 0],dt);
  //disp([vcamReal([1:2]) 0 0 0 vcamReal(3)])
  cMo        = inv(c1Mc2)*cMo;
  cMo        = cMo.*(abs(cMo)>1e-10);
 
 
  //----------------------------------------------------//
  // update the position    
  //save the 2D position
  //-----------------------------------------------------//
  // Update variables
  VDes       =[VDes;vcam];
  VReal      =[VReal;vcamReal];
  balancement= vcamReal-vcam;
  cP         = changeFrameMire(oP,cMo); // compute the 3D points
  p          = projectMireDirect(cP);   // compute the 2D points
  Z          = cP(3:3:length(cP));      // compute Z
 
  P          = [P p]; //preel
  Pcorr      = [Pcorr corre]; //pcorrected
  E          = [E e]; //error
  Ecorr      = [Ecorr p-pdes]; 
  INTEGRALE  = [INTEGRALE integrale];
  posec      = pFromHomogeneousMatrix(inv(cMo));
  Xc         = [Xc;posec(1)];
  Yc         = [Yc;posec(2)]; 
  Zc         = [Zc;posec(3)];

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
mire2DDraw(pdes,cote*2,5);
if (iter>2)
  for i=1:Nbpts
    plot(Pcorr((i-1)*2+1,:),Pcorr((i-1)*2+2,:),'b')
    plot(P((i-1)*2+1,:),P((i-1)*2+2,:),'r')
  end  
end
show_pixmap();

hf_2            = createFigure3D(2,"Camera Motion",2);
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
  plot(VReal(:,1),'r');
  plot(VReal(:,2),'g');
  plot(VReal(:,3),'b');
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
    plot(NORMECORR,'-');
  end
end 
  
end
endfunction



function isTestSwayLoaded()
 disp('TestSway is Loaded !')  
endfunction