function p = pFromHomogeneousMatrix(M);
// compute a vector pose from a Homogeneous Matrix

Rot = M(1:3,1:3);
Trans = M(1:3,4);
r = RxRyRzfromRotationMatrix(Rot)';

p =[Trans;r];
  
  
endfunction


function p = tUFromHomogeneousMatrix(M);
// compute a vector pose from a Homogeneous Matrix

Rot = M(1:3,1:3);
Trans = M(1:3,4);
[r,theta] = thetaUFromRotationMatrix(Rot);
p =[Trans;r];
  
  
endfunction
