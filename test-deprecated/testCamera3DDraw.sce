// main programm

path=get_absolute_file_path("Main.sce"),
getf(path+'Camera3D.sci')
getf(path+'Camera3DDraw.sci')
getf(path+'changementDeRepere.sci')
getf(path+'createFigure.sci')


  
scale=0.5;
//M = [1 0 0 0 ; 0 1 0 0 ; 0 0 1 0 ; 0 0 0 1 ];

getf(path+'homogeneousMatrix.sci');
M = homogeneousMatrix(0.3,0.1,1, 30*%pi/180,60*%pi/180,0);
M1 = homogeneousMatrix(1.3,0.1,1, 30*%pi/180,60*%pi/180,0);
M2= homogeneousMatrix(2.3,1.1,1, 30*%pi/180,60*%pi/180,0);
M3 = homogeneousMatrix(3.3,2.1,1, 30*%pi/180,60*%pi/180,0);


disp("click avant figure ")
xclick;
hf=createFigure(1,"testDrawCamera",5);
xclick;
Camera3DDraw(scale,M);
xclick;
Camera3DDraw(scale,M1);
xclick;
Camera3DDraw(scale,M2);
xclick;
Camera3DDraw(scale,M3);

disp("click avant close ")
xclick;
//xpause (5000000);
delete (hf);
