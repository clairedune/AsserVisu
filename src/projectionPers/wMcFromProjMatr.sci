function [K,cMv]=cMvFromProjMat(P)
// This function decomposes a projective matrix P with:
//   K : The intrinsic matrix
//   R : The rotation matrix of the camera
//   C : The position of the matrix.
// (c) Stephane EMBARKI, JRL, CNRS/AIST

// First Normalize the projective matrix.
w=sqrt(P(3,1)^2+P(3,2)^2+P(3,3)^2);
Pn=P/w;

M=Pn(1:3,1:3);
iM=inv(M);

// Decompostion QR : iM= Q*R
//with Q=vRc ( square orthogonal matrix )
//and R=inv(K) ( upper triangular matrix )
[vRc,iK]=qr(iM);

// Camere intrinsect parameters
K=inv(iK);

// Camera center in the reference vision frame
vC=-iM*Pn(1:3,4);

// Rotation matrix of the vision frame express in the camera frame
cRv=inv(vRc);

// cMv : Transformation matrix from object frame to camera frame
cMv=[[cRv,-cRv*vC];[0,0,0,1]];

endfunction

//--------------------------------------------------//


function cMh = cMhFromProjMat(P)
  
[K,cMv]=cMvFromProjMat(P)
// wMv : Transformation matrix from vison frame to head frame
hMv=[-0.009750 -0.980949 0.194020 1001.001831
0.999839 -0.012489 -0.012902 35.397411
0.015080 0.193863 0.980913 -121.058197
0.000000 0.000000 0.000000 1.000000 ];

//cMv : Transformation matrix from world coordinates to camera
cMh=cMv*inv(hMv);

endfunction


//--------------------------------------------------//
function cMhm = cMhFromProjMatMeter(P)

cMhmm = cMhFromProjMat(P)
cMhm  = cMhmm;
cMhm(1:3,4)=cMhm(1:3,4)/1000;
  
  
endfunction
