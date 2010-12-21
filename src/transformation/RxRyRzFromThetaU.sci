function vout=RxRyRzFromThetaU(vin)
//
// Auteur Claire Dune
// Date 11 2009
// Description : cette fonction permet de transformer une rotation
// exprimee en thetaU en rotation RxRyRz
//


 M = rotationMatrixFromThetaU(vin);
 vout = RxRyRzfromRotationMatrix(M);


endfunction
