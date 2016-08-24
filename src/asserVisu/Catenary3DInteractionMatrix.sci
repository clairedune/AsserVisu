function  L  = catenary3DIntMat( rlen, hmax, param, xA, yA, zA )
// interactionMatrix Calculates the interaction matrix for catenary
// parameters a = h/hmax and b = sin(theta)
// rlen      : rope half-length
// hmax      : rope maximum sag     
// param     : vector with estimated parameters of catenary p=[a;b]
// xA, yA, zA: coordinates of the rope attatchement point (PA) on robot T1

a = param(1);
b = param(2);
h = a*hmax;
C = 2*h/(rlen^2 - h^2);
D = (1/C)*acosh(C*h + 1);
kc = 2*(rlen^2 + h^2)/(rlen^2 - h^2)^2;
ky = sinh(C*D)/(1 + (kc/C^2)*(cosh(C*D)-1 - C*D*sinh(C*D)));

l1 = (ky/2)*[-sqrt(1-b^2), -b, 0, zA*b, -zA*sqrt(1-b^2), yA*sqrt(1-b^2)-xA*b];
l2 = (1/(2*D))*[b*sqrt(1-b^2), -1+b^2, 0, zA*(1-b^2), zA*b*sqrt(1-b^2), -yA*b*sqrt(1-b^2)-xA*(1-b^2)];
L = [l1; l2];

endfunction

