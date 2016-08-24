
clear;
//close;
exec('Load.sce');

// Given D and R try to find H of a catenary

R = 0.5;
D = 0.3;

function e = fd (h,m)
    e = D-(R^2-h^2)/(2*h)*acosh((h^2+R^2)/(R^2-h^2));
endfunction

H0 = sqrt(R^2-D^2);
[xsol,v]=lsqrsolve(H0,fd,1);

H = xsol;

Dest = (R^2-H^2)/(2*H)*acosh((H^2+R^2)/(R^2-H^2));




