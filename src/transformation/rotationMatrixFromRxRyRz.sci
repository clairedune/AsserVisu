function Rot = rotationMatrixFromRxRyRz(v)
// auteur CLaire dune
// Date novembre 2009
// Mouvement de rotation

// Mouvement de rotation
Rx = v(1) ;
Ry = v(2);
Rz = v(3) ;


Rotx = [ 1 0 0
         0 cos(Rx) -sin(Rx)
         0 sin(Rx) cos(Rx) ];

Roty = [ cos(Ry) 0 sin(Ry)
         0 1 0 
         -sin(Ry) 0 cos(Ry)];

Rotz = [ cos(Rz) -sin(Rz) 0
         sin(Rz) cos(Rz) 0
         0 0 1 ];
Rot = Rotx*Roty*Rotz;

endfunction
