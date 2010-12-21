function v = RxRyRzfromRotationMatrix(R)
// auteur CLaire dune
// Date novembre 2009
// Mouvement de rotation

phi = 0;

if ((abs(R(2,3)) < %eps) & (abs(R(3,3)) < %eps)) 
  phi = 0 ;
else 
  phi = atan(-R(2,3), R(3,3)) ;
end;

si = sin(phi) ;
co = cos(phi) ;
theta = atan(R(1,3), -si*R(2,3) + co*R(3,3)) ;
psi = atan(co*R(2,1) + si*R(3,1), co*R(2,2) + si*R(3,2));

v(1) = phi ;
v(2) = theta ;
v(3) = psi ;

v=v';
endfunction
