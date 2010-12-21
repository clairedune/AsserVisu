// main programm

//-----------------------------------------------//
path=get_absolute_file_path("scilab-src");
disp('HOME:'+path),
getd(path + "src/graphisme"); // pour charger un repertoire en entier
getd(path + "src/transformation");  
getd(path + 'src/projectionPers');
getd(path + 'src/asserVisu');
//-------------------------------------------------//

// create the initial position Rx Ry Rz
// of the camera
poseCam = [ 0 0 0 0 0 0 ];

// build the homogeneous matrix from the pose vector
disp("Initial Camera Position: ");
wMc = homogeneousMatrixFromPos(poseCam);

//create 5 points
poseInit = [0.5 0.4 3 0 0 0];
cinitMo = homogeneousMatrixFromPos(poseInit);
oP = mire5points (0.1); // mire dans le repere mire
wMo = wMc*cinitMo;
wP = changeFrameMire(oP,wMo);
p = projectMire(wP,wMc);


// create the desired pose
poseDes = [0.01 0.01 0.1 0 0 30*%pi/180];
cdesMo = homogeneousMatrixFromPos(poseDes);
pdes = projectMire(oP,inv(cdesMo));


// create the image plan
hf2d = createPlanImage(1,"Point 2D");
cote=0.05;
mireDraw(p,cote,3);
show_pixmap()
mireDraw(pdes,cote,5);
show_pixmap()
disp('This is the first point projection')
disp('Green : current one, Red Desired one')
disp('Click to display the servoing')
xclick;


// velocity vector to apply exprime en rxryrz
v = [0 0 0 0*%pi/180 0*%pi/180 0*%pi/180];
// time to apply it 
dt = 1/30;


// create the window
hf = createFigure3D(2,"Camera Motion",5);
// display the first camera
Camera3DDraw(0.1,wMc);
show_pixmap()

hf2d_2 = createFigure2D(3,"Error");


// nombre d'iteration
NbIt =10; 
norme=[];


// choose a Z arbitrarily
Zint=1;
lambda =1;
L = matIntMire6ddl(p,Zint); 
disp("Enter the loop to display  camera position")

er = 10;
threshold = 1e-1 ;
while(er>threshold)
   
  c1Mc2 = expMapDirectRxRyRz(v,dt);
  deplacement =  pFromHomogeneousMatrix(c1Mc2)';
  wMc=wMc*c1Mc2; 
  
  xset("window",2);
  Camera3DDraw(cote,wMc);
  show_pixmap()
 
  //x = projection3dPoint(wX,wMc);
  p = projectMire(wP,wMc);
  cMo = inv(wMc)*wMo;
  //Zint = cMo(3,4);
  
  xset("window",1); 
  mireDraw(p,cote,3);
  //point2DDraw(x,cote,15);
  show_pixmap();
     
  // compute the error
  E = p-pdes;
  e = [E(:,1)'  E(:,2)'  E(:,3)'  E(:,4)'  E(:,5)'];
  norme = [norme norm(e)];
  er = norm(e);
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

  // compute interaction matrix
  L = matIntMire6ddl(p,Zint); 


  v = computeVelocity(lambda, L,e');

  
  

//disp("click to continue");
//xclick;
end;
xset("pixmap",0);












