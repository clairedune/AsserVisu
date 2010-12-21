function [M] = homogeneousMatrixFromPos(pose)

// Ouput variables initialisation (not found in input variables)
M=[];

// Display mode
mode(0);

// Display warning for floating point exception
ieee(1);



Tr = [pose(1);pose(2);pose(3)];

// Mouvement de rotation
Rx = mtlb_e(pose,4);
//*pi/180;
Ry = mtlb_e(pose,5);
//*pi/180;
Rz = mtlb_e(pose,6);
//*pi/180;

// Rotx = [ 1 0 0
//          0 cos(Rx) sin(Rx)
//          0 -sin(Rx) cos(Rx) ];
// 
// Roty = [ cos(Ry) 0 -sin(Ry)
//          0 1 0 
//          sin(Ry) 0 cos(Ry)];
// 
// Rotz = [ cos(Rz) sin(Rz) 0
//          -sin(Rz) cos(Rz) 0
//          0 0 1 ];
Rotx = [1,0,0;
     0,cos(mtlb_double(Rx)),-sin(mtlb_double(Rx));
     0,sin(mtlb_double(Rx)),cos(mtlb_double(Rx))];

Roty = [cos(mtlb_double(Ry)),0,sin(mtlb_double(Ry));
     0,1,0;
     -sin(mtlb_double(Ry)),0,cos(mtlb_double(Ry))];

Rotz = [cos(mtlb_double(Rz)),-sin(mtlb_double(Rz)),0;
     sin(mtlb_double(Rz)),cos(mtlb_double(Rz)),0;
     0,0,1];

Rot = (Rotx*Roty)*Rotz;
// Rot2 =[
//     
//     cos(Ry)*cos(Rz)                               -cos(Ry)*sin(Rz)                     sin(Ry) 
//     sin(Rx)*sin(Ry)*cos(Rz)+cos(Rx)*sin(Rz)       -sin(Rx)*sin(Ry)*sin(Rz)+cos(Rx)*cos(Rz) -sin(Rx)*cos(Ry)
//     -cos(Rx)*sin(Ry)*cos(Rz)+sin(Rx)*sin(Rz) cos(Rx)*sin(Ry)*sin(Rz)+sin(Rx)*cos(Rz) cos(Rx)*cos(Ry)]
//    




// Matrice homogene correspondante 
M = [Rot,Tr;
     0,0,0,1];
     
     
M = M.* (abs(M)>1e-8);     
endfunction
