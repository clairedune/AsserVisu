// main programm

//-----------------------------------------------//
path=get_absolute_file_path("scilab-src");
disp('HOME:'+path),
getd(path + "src/graphisme"); // pour charger un repertoire en entier
getd(path + "src/transformation");  
getd(path + 'src/projectionPers');
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

// create the image plan
hf2d = createFigure2D(1,"Point 2D",1);
cote=0.02;
point2DDraw(x,cote,3);
show_pixmap()
point2DDraw(xdes,cote,5);
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
hf = createFigure3D(2,"Camera Motion",10,2);
// display the first camera
Camera3DDraw(0.1,wMc);
show_pixmap()
// nombre d'iteration

NbIt =10; 
disp("Enter the loop to display  camera position")
for i=1:NbIt
 
  c1Mc2 = expMapDirectRxRyRz(v,dt);
  deplacement =  pFromHomogeneousMatrix(c1Mc2)';
  wMc=wMc*c1Mc2; 
  xset("window",2);
  Camera3DDraw(0.1,wMc);
  show_pixmap()
 
  x = projection3dPoint(wX,wMc);
  xset("window",1); 
  point2DDraw(x,cote,15);
  show_pixmap();
  
//disp("click to continue");
//xclick;
end;
xset("pixmap",0);












