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


posecMoInit    = [ 0 0 1 0 0 0 ];
posecMoFinal   = [ 0 0 0.5 0 0 0 ];
poseMire       = [ 0 0 0 %pi/2 0 0 ];;

// -- For Simulation
dt             = 1/10;// time delay between two command
cote           = 0.01; // size of the target dot

// -- For Visual Servoing
lambda         = 0.4;//aservissement gain
threshold      = 1e-3 ;// threshold on the error to stop

//-------- Matrix Position -------------//
cMo            = homogeneousMatrixFromPos(posecMoInit);
cdesMo         = homogeneousMatrixFromPos(posecMoFinal);
wMo            = homogeneousMatrixFromPos(poseMire);
wMo            = wMo.*(abs(wMo)>1e-10);
wMc            = wMo*inv(cMo);
wMc            = wMc.*(abs(wMc)>1e-10);
wMcdes         = wMo*inv(cdesMo);
wMcdes         = wMcdes.*(abs(wMcdes)>1e-10);

//-------- Create the target -----------//
L              = 0.5; // related to the target size 
oP             = mire5points (L); 
cP             = changeFrameMire(oP,cMo); 
wP             = changeFrameMire(oP,wMo); 
p              = projectMireDirect(cP);
Z              = cP(3:3:length(cP)); 
cdesP          = changeFrameMire(oP,cdesMo); 
pdes           = projectMireDirect(cdesP);
Zdes           = cdesP(3:3:length(cP)); 

//------create the figures-------------//
xl             = [-0.5;-0.5]
xu             = [0.5; 0.5]
hf2d           = createPlanImage(1,xl,xu,"Point 2D");
mire2DDraw(p,cote,3);
show_pixmap()
mire2DDraw(pdes,cote,5);
show_pixmap()
// create the window
wMcfirst       = wMc;
hf             = createFigure3D(2,"Camera Motion",2);
Camera3DDrawColor(0.1,wMcfirst,3);// display the first camera
Camera3DDrawColor(0.1,wMcdes,5);// display the first camera
Mire3DDraw5pts(wP);
show_pixmap()
//create a windows to display the top view
hf2d_2         = createFigure2D(4,"TopView");

//----------------- VISUAL SERVOING LOOP -----------------//
Lfunction = matIntMireC;
vcamReal  = zeros(1,6); // real camera montion
vN        = zeros(1,6);
vT        = zeros(1,6);
  
// create a window to display the error
norme     = [];
hf2d_2    = createFigure2D(3,"Error");

//create a windows to display the top view
velocity  = [];
hf2d_3    = createFigure2D(5,"Desired Velocity");

//create a windows to display the top view
velocity2 = [];
hf2d_4    = createFigure2D(6,"Real Velocity");

//create a windows to display the top view
velocity3 = [];
hf2d_5    = createFigure2D(7,"Sum des error ");



// launch the servo
er = 10; // error value init



iter  = 0;
vcam  = zeros(1,3);
esav  = zeros(length(p),1);
mu    =0.8;
E     =[];
vsav  = zeros(3,1);
vcamReal  = zeros(1,6);

L     = Lfunction(p,Z); 

sat   = 0.25; // term de saturation

while(er>threshold & iter<300)

  iter = iter+1;
  //disp('------------------------',iter)
  
  // --- Classical visual servoing     
  cP    = changeFrameMire(oP,cMo);                    // compute the 3D points
  p     = projectMireDirect(cP);                      // compute the 2D points
  Z     = cP(3:3:length(cP));                         // compute Z
  
  e     = p-pdes;                                     // compute the error  
  norme = [norme norm(e)];  
  er    = norm(e);
  esav  = esav + dt*L(:,[1,3,5])*( vcamReal([1,3,5])-vcam)';

  L     = Lfunction(p,Z);                             // compute the interaction matrix
  vcam  = computeVelocity(lambda, L(:,[1,3,5]),e-esav)
  
  
  
  vDes = [vcam(1) 0 vcam(2) 0 vcam(3) 0]
  // the real velocity is not vcam but something based on vcam
  vcamReal = vDes+[0.40*sin(iter*%pi/6),0,0,0,0,0] ;
  //vcamReal = sat*(abs(vcamReal) >= sat)+vcamReal.*(abs(vcamReal)<sat);
  
  
    
  // the resulting camera motion is then  
  c1Mc2 = expMapDirectRxRyRz( vcamReal,dt);
  cMo   = inv(c1Mc2)*cMo;
  wMc   = wMo*inv(cMo);
  wMc   = wMc.*(abs(wMc)>1e-10);
  
 // --- Displays--------------------------------------//
  xset("window",1); 
  mire2DDraw(p,cote,3);
  show_pixmap();
  
  hf             = createFigure3D(2,"Camera Motion",2);
  Camera3DDrawColor(0.1,wMcfirst,3);// display the first camera
  Camera3DDrawColor(0.1,wMc,3);// display the first camera
  Camera3DDrawColor(0.1,wMcdes,5);// display the first camera
  Mire3DDraw5pts(wP);
  
  Camera3DDraw(0.1,wMc);
  show_pixmap()
  
  xset("window",3);
  plot(threshold*ones(1,length(norme)))
  her=gce();
  her.foreground=5;
  plot(norme);
  her=gce();
  her.foreground=3;
  show_pixmap();
  
  xset("window",4);
  AxeZ2DDraw(0.02,wMc);
  show_pixmap();
  
  xset("window",5);
  velocity =[velocity;vDes];
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

xset("pixmap",0);




