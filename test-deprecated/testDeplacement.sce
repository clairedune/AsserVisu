// main programm

//-----------------------------------------------//
path=get_absolute_file_path("scilab-src");
disp('HOME:'+path),
getd(path + "src/graphisme"); // pour charger un repertoire en entier
getd(path + "src/transformation"); 
//-------------------------------------------------//


// create the initial position Rx Ry Rz
// of the camera
pose = [ 0 0 0 0 0 0 ];

// build the homogeneous matrix from the pose vector
disp("Initial Camera Position: ");
wMc = homogeneousMatrixFromPos(pose);
p = pFromHomogeneousMatrix(wMc),

// velocity vector to apply exprime en rxryrz
v = [1 0 0 30*%pi/180 0 0];

// time to apply it 
dt = 1;

// create the window
hf = createFigure(1,"Camera Motion",10);

// display the first camera
Camera3DDraw(0.5,wMc);

// nombre d'iteration
NbIt =5; 
disp("Enter the loop to display 20 camera position")
for i=1:NbIt
  c1Mc2 = expMapDirectRxRyRz(v,dt);
  deplacement =  pFromHomogeneousMatrix(c1Mc2)';
  wMc=wMc*c1Mc2; 
  Camera3DDraw(0.5,wMc); 
//disp("click to continue");
//xclick;
end;

v = [0 2 0 0 30*%pi/180 30*%pi/180];
for i=1:NbIt
  c1Mc2 = expMapDirectRxRyRz(v,dt);
  deplacement =  pFromHomogeneousMatrix(c1Mc2)';
  wMc=wMc*c1Mc2; 
  Camera3DDraw(0.5,wMc); 
//disp("click to continue");
//xclick;
end;

v = [0 0 3 0 30*%pi/180 30*%pi/180];
for i=1:NbIt
  c1Mc2 = expMapDirectRxRyRz(v,dt);
  deplacement =  pFromHomogeneousMatrix(c1Mc2)';
  wMc=wMc*c1Mc2; 
  Camera3DDraw(0.5,wMc); 
//disp("click to continue");
//xclick;
end;











