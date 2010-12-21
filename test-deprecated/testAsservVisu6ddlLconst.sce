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
pose = [ 0 0 0 0 0 0 ];

// build the homogeneous matrix from the pose vector
disp("Initial Camera Position: ");
wMc = homogeneousMatrixFromPos(pose);

//create a 3D point
wX = [1 1 2];

// project this point in the view
x = projection3dPoint(wX,wMc),
// create the desired 2D point
xdes = [0 ; 0];


//create 5 points
poseMire = [0.5 0.4 1 0 0 0];
wMo = homogeneousMatrixFromPos(poseMire);
oP = mire5points (0.1); // mire dans le repere mire
wP = changeFrameMire(oP,wMo);
p = projectMire(wP,wMc);


// create the desired pose
poseDes = [0.01 0.01 0.5 0 0 30*%pi/180];
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
v = [0.1 0 0 0*%pi/180 0*%pi/180 0*%pi/180];
// time to apply it 
dt = 1;


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
lambda =0.3;
  L= [ matIntPoint6ddl(p(1,1),p(2,1),Zint),
       matIntPoint6ddl(p(1,2),p(2,2),Zint);
       matIntPoint6ddl(p(1,3),p(2,3),Zint)
       matIntPoint6ddl(p(1,4),p(2,4),Zint)
       matIntPoint6ddl(p(1,5),p(2,5),Zint)]; 
disp("Enter the loop to display  camera position")

er = 10;
threshold = 1e-2 ;
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
  er = norm(e),
  xset("window",3);
  plot(norme);
  show_pixmap();

  // compute interaction matrix
  L= [ matIntPoint6ddl(p(1,1),p(2,1),Zint),
       matIntPoint6ddl(p(1,2),p(2,2),Zint);
       matIntPoint6ddl(p(1,3),p(2,3),Zint)
       matIntPoint6ddl(p(1,4),p(2,4),Zint)
       matIntPoint6ddl(p(1,5),p(2,5),Zint)]; 

  v = computeVelocity(lambda, L,e');

  
  

//disp("click to continue");
//xclick;
end;
xset("pixmap",0);












