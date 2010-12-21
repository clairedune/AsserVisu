// main programm

//-----------------------------------------------//
path=get_absolute_file_path("scilab-src");
disp('HOME:'+path),
getd(path + "src/graphisme"); // pour charger un repertoire en entier
getd(path + "src/transformation"); 
//-------------------------------------------------//

// test de vecteur vitesse exprime en VxVyVzRxRyRz en VxVyVzThetaU

rxyz = [0*%pi/180 0*%pi/180 0*%pi/180 ],
M = rotationMatrixFromRxRyRz(rxyz),
rTu =  thetaUFromRotationMatrix(M),


//rTu = thetaUFromRxRyRz(rxyz),
//rxyz = RxRyRzFromThetaU(rTu),




