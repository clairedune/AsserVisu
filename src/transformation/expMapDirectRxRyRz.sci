function M = expMapDirectRxRyRz(v,dt)
// compute the transformation from the pose t to the pose t+dt
// input : the velocity (m/s,rad/s) and the time to apply it (s) 
// the velocity vector is a RxRyRz 

vThetaU = vThetaUFromVRxRyRz(v),
M = expMapDirectThetaU(vThetaU,dt);
 
endfunction

//------------------------------------------------//
//
//-------------------------------------------------//
function vitesse = expMapInverseRxRyRz(M,dt)
  vitesse = expMapInverseThetaU(M,dt)
endfunction
