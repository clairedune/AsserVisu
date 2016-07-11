// test catenary projection in an image
//-------- CATENARY PARAMETERS------------//
// rope semi lenght
global R;
R    = 0.5;
// max rope sag == fixation point heigh
Hmax = R ;

// semi distance between the two fixation points
// se calcul par rapport a H et R
global D;
D    = 0.3;

if(D>R)
    disp('Erreur D must be < R')
end


function [f,g,ind] = costCatenary (h,ind)
    c = (2*h)/(R^2-h^2);
    f =  (1/c*(cosh(c*D)-1)-h)*(1/c*(cosh(c*D)-1)-h);
 
    heps = h+%eps;
    ceps = (2*heps)/(R^2-heps^2);
    feps =  (1/ceps*(cosh(ceps*D)-1)-heps)*(1/ceps*(cosh(ceps*D)-1)-heps);
    g = (f-feps)/%eps;
endfunction




h0 = 0.1;
[fopt,hopt]= optim(costCatenary, h0)


// C
//Cest = (2*Hest)/(R^2-Hest^2);
// D
Hest = 0.36;
Dest = (R^2-Hest^2)/(2*Hest)*acosh((Hest^2+R^2)/(R^2-Hest^2));
//disp('D = '+string(D))
//disp('Hest = '+string(Hest))
//disp('Dest = '+string(Dest))

// rope points coordinates in the rope frame
//X = -Dest:Dest/100:Dest;
//Z = abs( 1/Cest*(cosh(Cest*Dest)-1);
//Y = zeros(length(X),1)';
//line =  zeros(length(X),1)';
// coordinates of one 3D point P = (X,Y,Z,1)
//P = [X;Y;Z;ones(length(X),1)'];

//plot(P(1,:),P(3,:));

