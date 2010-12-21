// this code test the asser visu 
// given as an input to Andreis function
// for step computing
// Andrei
//exec('LoadMarche.sce');

clear

path=get_absolute_file_path("scilab-src");
  disp('HOME:'+path),
  getd(path + "src/graphisme"); // pour charger un repertoire en entier
  getd(path + "src/transformation");  
  getd(path + 'src/projectionPers');
  getd(path + 'src/asserVisu');
  getd(path + 'src/hrp2')
  getd(path + 'src/optimisation')
  getd(path)


// pose init and final of the robot in the world frame
poserInit      = [ 0.01 0.5 0 0 0 30*%pi/180];
poserFinal     = [ 1.5 1.5 0 0 0 0 ];
posecMoInit    = [ 0 0 2 0 -30*%pi/180 0 ];

// -- For Simulation
dt             = 1/10; // time delay between two command
cote           = 0.01; // size of the target dot

// -- For Visual Servoing
lambda         = 0.7;//aservissement gain
threshold      = 1e-5 ;// threshold on the error to stop
OPT_SAT        = %F;
OPT_WALK       = %T;
OPT_CORR       = %T;
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
wMo            = wMo.*(abs(wMo)>1e-10);
wMc            = wMc.*(abs(wMc)>1e-10);
wMcdes         = wMcdes.*(abs(wMcdes)>1e-10);

//-------- Create the target -----------//
a              = 0.2; // related to the target size
Nbpts          = 5;
oP             = mire5points (a); 
cP             = changeFrameMire(oP,cMo); 
wP             = changeFrameMire(oP,wMo); 
p              = projectMireDirect(cP);
pinit          = p;
Z              = cP(3:3:length(cP)); 
cdesP          = changeFrameMire(oP,cdesMo); 
pdes           = projectMireDirect(cdesP);
Zdes              = cdesP(3:3:length(cP)); 

//------create the figures-------------//
xl             = [-0.6;-0.6]
xu             = [0.6; 0.6]
hf_1           = createPlanImage(1,xl,xu,"Point 2D");
mire2DDraw(p,cote,3);
show_pixmap()
mire2DDraw(pdes,cote,5);
show_pixmap()
// create the window
wMcfirst       = wMc;
hf_2           = createFigure3D(2,"Camera Motion",2);
Camera3DDrawColor(0.1,wMcfirst,3);// display the first camera
Camera3DDrawColor(0.1,wMcdes,5);// display the first camera
Mire3DDraw5pts(wP);
show_pixmap()
//create a windows to display the top view
hf_3           = createFigure2D(3,"TopView");

//----------------- VISUAL SERVOING LOOP -----------------//
Lfunction = matIntMireC;
vcamReal  = zeros(1,6); // real camera montion
vN        = zeros(1,6);
vT        = zeros(1,6);
  
// create a window to display the error
norme     = [];
hf_4      = createFigure2D(4,"Error");

//create a windows to display the top view
velocity  = [];
hf_5      = createFigure2D(5,"Desired Velocity");

//create a windows to display the top view
velocity2 = [];
hf_6      = createFigure2D(6,"Real Velocity");

//create a windows to display the top view
velocity3 = [];
hf_7      = createFigure2D(7,"Corrected Error");



// launch the servo
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
sat         = 0.25; // terme de saturation
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

N           = 100;
psav        = 0;
pav         = 0;

while( iter < N)

  iter = iter+1;
  //disp('------------------------',iter)
  cP         = changeFrameMire(oP,cMo);                    // compute the 3D points
  p          = projectMireDirect(cP);                      // compute the 2D points
  Z          = cP(3:3:length(cP));                         // compute Z
 
  // classical visual servoing
  e          = p-pdes;                                     // compute the error  
  norme      = [norme norm(e)];  
  esav       = esav + dt*L(:,[1,3,5])*(vcamReal([1,3,5])-vcam)';
  psav       = psav + dt*L(:,[1,3,5])*(vcamReal([1,3,5]))';
  pav        = pav  + dt*L(:,[1,3,5])*(vcam)';
  

  
   
  L          = Lfunction(p,Z);                             // compute the interaction matrix
  if(OPT_CORR) 
    vcam       = computeVelocity(lambda, L(:,[1,3,5]),e-esav);
    
  else
    vcam       = computeVelocity(lambda, L(:,[1,3,5]),e);
    
  end




  if (OPT_SAT)
    vcam = -sat*(vcam <= -sat)+sat*(vcam >= sat)+vcam.*(abs(vcam)<sat);
  end; 
  vcam       = vcam.*(abs(vcam) > 1e-6);
  

  // compute vrobotReal
  vrobot     = convertVelocityCamRobot(vcam,rVc) ;
 
  //---------Approximation of what andrei code should do---------//
  // input is the robot relative velocity
  // output is the robot velocity in the world frame
  vrobotReal = [vrobot(1) vrobot(2) 0 0 0 vrobot(3)];
  // the real velocity is not vcam but something based on vDes
  if (OPT_WALK)
      vrobotReal=vrobotReal+[0 +0.2*sin(dt*(iter-1)*%pi) 0 0 0 0] ;
  end
  wVr        = twistMatrix(wMr);
  vrobotRealW= wVr*vrobotReal';
  vrobotRealW= [vrobotRealW(1) vrobotRealW(2) vrobotRealW(6)];
  
  //--------------------------------------------------------------//

 
  vrobotReal = (inv(wVr)*[vrobotRealW(1) vrobotRealW(2) 0 0 0 vrobotRealW(3)]')'; 
  vrobotReal = vrobotReal.*(abs(vrobotReal) > 1e-6);
  r1Mr2      = expMapDirectRxRyRz(vrobotReal,dt);
  wMr        = wMr*r1Mr2;
  wMr        = wMr.*(abs(wMr)>1e-10);
  
  // deduce vcamReal   
  vcamReal   = convertVelocityRobotCam(vrobotReal,rVc) ;
  vcamReal   = [vcamReal(1),0,vcamReal(2),0,vcamReal(3),0 ]; 
  vcamReal   = vcamReal.*(abs(vcamReal) > 1e-6);
  // the resulting camera motion is then  
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
  Pav       =  [Pav pinit+pav];
  Xc         = [Xc ;wMc(1,4)];
  Yc         = [Yc; wMc(2,4)]; 
  Zc         = [Zc; wMc(3,4)];
  Xr         = [Xr; wMr(1,4)];
  Yr         = [Yr; wMr(2,4)];
  Zr         = [Zr; wMr(3,4)];




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
  
  hf             = createFigure3D(2,"Camera Motion",2);
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
  plot(threshold*ones(1,length(norme)))
  her=gce();
  her.foreground=5;
  plot(norme);
  her=gce();
  her.foreground=3;
  show_pixmap();
  
  xset("window",3);
  AxeZ2DDraw(0.02,wMc);
  if (iter>2)
    plot(Xc,Yc);
  end 
  show_pixmap();
  
  xset("window",5);
  velocity =[velocity;vcam];
  if(size(velocity,1)>1)
    plot2d(velocity);
        show_pixmap()
  end

  xset("window",6);
  velocity2 =[velocity2;vcamReal];
  if(size(velocity2,1)>1)
    plot2d(velocity2);
    show_pixmap()
  end 
 

  xset("window",7);
  E =[E;norm(e-esav)'];
  if(size(E,1)>1)
    plot2d(E);
    show_pixmap()
  end 
 

 
end;
halt()
disp('-------The End------')
xset("pixmap",0);

xs2fig(1,'point2D.fig')
xs2fig(2,'cameraPosition.fig')
xs2fig(3,'topview.fig')
xs2fig(4,'error.fig')
xs2fig(5,'velocity.fig')
xs2fig(6,'errPosition.fig')

delete(hf_1);  
delete(hf_2);
delete(hf_3);
delete(hf_4);  
delete(hf_5);
delete(hf_6);
delete(hf_7);


