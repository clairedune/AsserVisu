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
pose_cMo = [0.5 0 3 0 0 0];
cMo = homogeneousMatrixFromPos(pose_cMo);
oP = mire5points (0.5); // mire dans le repere mire
wMo = wMc*cMo;
wP = changeFrameMire(oP,wMo);
// compute the first projection on the view
p = projectMire(oP,inv(cMo));


// create the desired pose
pose_cdesMo = [0.05 0 1 0 30*%pi/180 0];
cdesMo = homogeneousMatrixFromPos(pose_cdesMo);
pdes = projectMire(oP,inv(cdesMo));

// camera des
wMcdes = wMo*inv(cdesMo);

// create the image plan
hf2d = createPlanImage(1,"Point 2D");
cote=0.002;
mire2DDraw(p,cote,3);
show_pixmap()
mire2DDraw(pdes,cote,5);
show_pixmap()
disp('This is the first point projection')
disp('Green : current one, Red Desired one')
//disp('Click to display the servoing')
//xclick;

hf = createFigure3D(2,"Camera Motion",1);
Camera3DDrawColor(0.1,wMc,3);// display the first camera
Camera3DDrawColor(0.1,wMcdes,5);// display the first camera
Mire3DDraw(wP);


show_pixmap()
