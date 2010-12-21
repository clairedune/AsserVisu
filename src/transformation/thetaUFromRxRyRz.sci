function v=thetaUFromRxRyRz(Rxyz)
//
// Auteur Claire Dune
// Date 11 2009
// Description : cette fonction permet de transformer une rotation
// exprimee en RxRyRz en rotation thetau
M = rotationMatrixFromRxRyRz(Rxyz),
v = thetaUFromRotationMatrix(M),
endfunction
