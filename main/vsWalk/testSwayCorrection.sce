// this code test the asser visu 
// given as an input to Andreis function
// for step computing
// 
//
// test another correction
//;exec('main/testAVrobotErreurMarche.sce');
maxiter        = 200;               //

// -- Pose init and final of the robot in the world frame
posecMoInit    = [ 0.4 0 1 0 0 5*%pi/180]; // Object position in the init camera frame
posecMoDes     = [ 0 0 0.5 0 0 0 ]; // Object position in the init camera frame

// -- For Simulation
dt             = .1;              // time delay between two command
cote           = .01;              // size of the target dot
// -- For Visual Servoing
lambda         = .6;               // aservissement gain
threshold      = .001 ;             // threshold on the error to stop
BIAIS          = 0 ;
OPT_CORR       = %T;

//-------- Matrix Position -------------//
cMo            = homogeneousMatrixFromPos(posecMoInit);
cdesMo         = homogeneousMatrixFromPos(posecMoDes);
oMcdes         = inv(cdesMo);
// set to 0 the values that are < small
tooSmall       = 1e-10;


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
eprec       = zeros(length(p),1);
pprec       = zeros(length(p),1); 
integrale   = zeros(length(p),1);
L           = Lfunction(pdes,Zdes);
Lcourant    = L;
STOP_CRITERION = %F;

VDes        =[];
VReal       =[];
P           =[];
Pcorr       =[];
E           =[];
Ecorr       =[];

//----------------------------------------------------------------------//
// Servo LOOP
while( iter < maxiter & STOP_CRITERION==%F)

  iter = iter+1;
  printf('----------------------%d\n',iter)
  
  e          = p-pdes;                       // compute the error  
  
  //-----------------------------------------------------------///
  // Compute the correction 
  //L          = Lfunction(p,Z);              // compute the interaction matrix
 
  if(iter>1)
    dedt     = (e-eprec)/dt-L*(vcam');
  end
  eprec      = e;
  corre      = dt*L*(vcamReal-vcam)';
  integrale  = integrale + corre ;
  
  if (OPT_CORR)
    erreurCourante=e-integrale;
  else
    erreurCourante=e;
  end 
  vcam       = computeVelocity(lambda, L,erreurCourante);
  //-----SIMU ROBOT----//
  //deduce vcamReal   
  vcamReal   = vcam;
  vcamReal(3)= 0;
  vcamReal(4)= 0;
  vcamReal(5)= 0;
  vcamReal(2)= vcam(2)+0.2*sin(%pi/(PERIOD/2)*(iter-1));
   
  c1Mc2      = expMapDirectRxRyRz(vcamReal,dt);
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
  
  posec      = pFromHomogeneousMatrix(inv(cMo));
  Xc         = [Xc;posec(1)];
  Yc         = [Yc;posec(2)]; 
  Zc         = [Zc;posec(3)];

  
  

  if(norm(e)<threshold) 
    STOP_CRITERION=%T;
  end 
  
  
  
end;

  
// ---------------------------------------------//
// Display the results 

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


