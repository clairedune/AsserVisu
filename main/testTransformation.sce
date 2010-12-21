clear
getd("src/transformation")

M = [-0.866025 1.18288e-16 -0.5
-1.18288e-16 -1 -3.16951e-17
-0.5 3.16951e-17 0.866025]


[thetaU, theta]= thetaUFromRotationMatrix(M)
M2 = rotationMatrixFromThetaU(thetaU)
