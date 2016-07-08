// test catenary projection in an image

//-------- CATENARY PARAMETERS------------//
// rope semi lenght
R       = 0.5;
// rope sag
H       = 0.3 ;
// max rope sag == fixation point heigh
Hmax    = 0.5 ;

// semi distance between the two fixation points
// se calcul par rapport a H et R
D = (R^2-H^2)/(2*H)*acosh((H^2+R^2)/(R^2-H^2));

// C
C = (2*H)/(R^2-H^2);

// rope points coordinates in the rope frame
X = -D:D/100:D;
Z = 1/C*(cosh(C*X)-1);
Y = zeros(length(X),1)';
line =  zeros(length(X),1)';
// coordinates of one 3D point P = (X,Y,Z,1)
P = [X;Y;Z;ones(length(X),1)'];

plot(P(1,:),P(3,:));

