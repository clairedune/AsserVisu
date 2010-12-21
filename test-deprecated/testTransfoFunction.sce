// main programm

path=get_absolute_file_path("Main.sce"),
getf(path+'rotationMatrixFromRxRyRz.sci')
getf(path+'rotationMatrixFromThetaU.sci')
getf(path+'RxRyRzfromRotationMatrix.sci')
getf(path+'thetaUFromRotationMatrix.sci')



// test 1 : Rxyz -> M -> Rxyz

Rxyz = [30*%pi/180,0, 90*%pi/180],
M = rotationMatrixFromRxRyRz(Rxyz),
Rxyz2 = RxRyRzfromRotationMatrix(M),

// test 2 : Rxyz -> ThetaU -> M -> Rxyz

ThetaU = thetaUFromRotationMatrix(M),
M2 = rotationMatrixFromThetaU(ThetaU),
Rxyz3 = RxRyRzfromRotationMatrix(M2),


