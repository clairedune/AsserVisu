function M = homogeneousMatrix(x,y,z,rx,ry,rz)
// fonction scilab pour construire une matrice homogene en partant 
// d'un vecteur x,y,z,rx,ry,rz
// les angles sont donnes en radians
  
Tr = [x;y;z];

// Mouvement de rotation
Rx = rx ;
Ry = ry ;
Rz = rz ;


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

// Matrice homogene correspondante 
M = [ Rot  Tr  
      0 0 0 1 ];

endfunction
