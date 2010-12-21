

posefiMoi=[0 0 0.5 0*%pi/180 0*%pi/180 0];
posefcMoc=[1 1 0.4 10*%pi/180 10*%pi/180 100*%pi/180];
posefiMfc=[2 5 0 0 0 -0*%pi/180];
fiMoi = homogeneousMatrixFromPos(posefiMoi);
fcMoc = homogeneousMatrixFromPos(posefcMoc);
fiMfc = homogeneousMatrixFromPos(posefiMfc);


oiMo = inv(fiMoi)*fiMfc*fcMoc;
pvrai = pFromHomogeneousMatrix(oiMo)
oiMo =  inv(fiMoi)*fcMoc;
papprox = pFromHomogeneousMatrix(oiMo)