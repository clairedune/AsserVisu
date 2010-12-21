// main programm
//-----------------------------------------------//
path=get_absolute_file_path("scilab-src");
disp('HOME:'+path),
getd(path + "src/graphisme"); // pour charger un repertoire en entier
getd(path + "src/transformation");  
getd(path + 'src/projectionPers');
getd(path + 'src/asserVisu');
getd(path + 'src/hrp2')
//-------------------------------------------------//

// Init Pose of the CoM
poseCom = [ 0 0 0 0 0 0 ];
wMcom = homogeneousMatrixFromPos(poseCom);
// link camera CoM
[comMc,comVc]=loadHRP2camCom();

// build the homogeneous matrix from the pose vector
disp("Initial Camera Position: ");
wMc = wMcom*comMc ;

//create 5 points
pose_cMo = [0. 0 3 0 0 0];
cMo = homogeneousMatrixFromPos(pose_cMo); // position of the camera in the target frame
L = 0.5; // size of the target
oP = mire5points (L); // mire dans le repere mire
wMo = wMc*cMo; // position of the taget in the world
wP = changeFrameMire(oP,wMo); // mire coordinates expressed in the world
// compute the first projection on the view
p = projectMire(oP,inv(cMo));

// create the desired pose
pose_cdesMo = [0.2 0 1 0 0*%pi/180 0];
cdesMo = homogeneousMatrixFromPos(pose_cdesMo);
pdes = projectMire(oP,inv(cdesMo));
// camera des
wMcdes = wMo*inv(cdesMo);


// create the image plan
hf2d = createPlanImage(1,"Point 2D");
cote=0.02;
mireDraw(p,cote,3);
show_pixmap()
mireDraw(pdes,cote,5);
show_pixmap()
disp('This is the first point projection')
disp('Green : current one, Red Desired one')
//disp('Click to display the servoing')
//xclick;



// ------------ Visual Servo Loop --------//

// create the window
hf = createFigure3D(2,"Camera Motion",2);
Camera3DDrawColor(0.1,wMc,3);// display the first camera
Camera3DDrawColor(0.1,wMcdes,5);// display the first camera
Mire3DDraw(wP);
show_pixmap()

// create a window to display the error
hf2d_2 = createFigure2D(3,"Error");

//create a windows to display the top view
hf2d_2 = createFigure2D(4,"TopView");

//create a windows to display the top view
hf2d_3 = createFigure2D(5,"Velocity");

// Visual servoing param

Zint=1; // approximation of the object depth
lambda =0.6;//aservissement gain
L = matIntMire3ddl(p,Zint); // compute the first interaction Matrix 

vcam3ddl=[0 0 0];
disp("Enter the loop to display  camera position")
// nombre d'iteration
NbIt =10; 
norme=[];
er = 10; // error value init
dt = 1/30;// time to apply it 
threshold = 1e-3 ;// threshold on the error to stop
velocity=[];
while(er>threshold)
  
  // motion performed
  c1Mc2 = computeMotionCoM(vcam3ddl,dt);
  //  c1Mc2 = computeMotionCam(vcam6ddl,dt);
  // new camera position
  wMc=wMc*c1Mc2; 
  
  //x = projection3dPoint(wX,wMc);
  p = projectMire(wP,wMc);
  cMo = inv(wMc)*wMo;
  //Zint = cMo(3,4);
       
  // compute the error
  E = p-pdes;
  e = [E(:,1)'  E(:,2)'  E(:,3)'  E(:,4)'  E(:,5)'];
  norme = [norme norm(e)];
  er = norm(e);

// Displays
  xset("window",1); 
  mireDraw(p,0.01,3);
  show_pixmap();
  
  xset("window",2);
  Camera3DDraw(cote,wMc);
  show_pixmap()
  
  xset("window",3);
  plot(threshold*ones(1,length(norme)))
  her=gce();
  //her.thickness=3;
  her.foreground=5;
  plot(norme);
  her=gce();
  //her.thickness=2;
  her.foreground=3;
  show_pixmap();
  
  xset("window",4);
  AxeZ2DDraw(0.02,wMc);
  show_pixmap();
  
  xset("window",5);
  velocity =[velocity;vcam3ddl ];
  plot2d(velocity);
  show_pixmap()
  
  // compute interaction matrix
  L = matIntMire3ddl(p,Zint); 
  vcam3ddl = computeVelocity(lambda, L,e');
 
end;

xset("pixmap",0);



