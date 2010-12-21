function R = rotationMatrixFromThetaU(v)
// auteur CLaire dune
// Date novembre 2009
// Mouvement de rotation

theta = sqrt(v(1)*v(1) + v(2)*v(2) + v(3)*v(3));
si = sin(theta);
co = cos(theta);
sinca = sinc(theta);
//mcosc = (1-co)/(theta*theta);
if(theta<%eps)
  mcosc=0.5;
  else
  mcosc = (1-co)/(theta*theta);
end;



R = eye(3,3);  
R(1,1) = co + mcosc*v(1)*v(1);
R(1,2) = -sinca*v(3) + mcosc*v(1)*v(2);
R(1,3) = sinca*v(2) + mcosc*v(1)*v(3);
R(2,1) = sinca*v(3) + mcosc*v(2)*v(1);
R(2,2) = co + mcosc*v(2)*v(2);
R(2,3) = -sinca*v(1) + mcosc*v(2)*v(3);
R(3,1) = -sinca*v(2) + mcosc*v(3)*v(1);
R(3,2) = sinca*v(1) + mcosc*v(3)*v(2);
R(3,3) = co + mcosc*v(3)*v(3);
endfunction
