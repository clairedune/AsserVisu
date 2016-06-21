

// test catenary projection in an image

//-------- CATENARY PARAMETERS------------//
// rope semi lenght
R = 0.5;
// rope sag
H = 0.3 ;
// max rope sag == fixation point heigh
Hmax = 0.5 ;
// rope orientation wrt the robot frame
theta = 30 *%pi/180;


// semi distance between the two fixation points
// se calcul par rapport a H et R
D = (R^2-H^2)/(2*H)*acosh((H^2+R^2)/(R^2-H^2));

// C
C = (2*H)/(R^2-H^2);

// rope points coordinates in the rope frame
sigma1_X = -D:D/100:D;
sigma1_Z = 1/C*(cosh(C*sigma1_X)-1);
sigma1_Y = zeros(length(sigma1_X),1)';
// coordinates of one 3D point P = (X,Y,Z,1)
sigma1_P = [sigma1_X;sigma1_Y;sigma1_Z;ones(length(sigma1_X),1)'];

plot3d(sigma1_P(1,:),sigma1_P(2,:),sigma1_P(3,:));

//--------- Change of frame -------------- //
// sigma2 is the frame attached to the fixation point and oriented 
// towards the robot direction
pose_sigma1_M_sigma2 = [-D 0 H 0 0 theta];
sigma1_M_sigma2      = homogeneousMatrixFromPos(pose_sigma1_M_sigma2);
sigma2_M_sigma1      = inv(sigma1_M_sigma2);
sigma2_P             = changeFramePoints(sigma1_P,sigma2_M_sigma1);


param3d(sigma1_P(1,:),sigma1_P(2,:),sigma1_P(3,:),'g');
param3d(sigma2_P(1,:),sigma2_P(2,:),sigma2_P(3,:),'r');

//plot(sigma1_P(1),sigma1_P(3));

//pause
//plot3d(sigma2_P(1,:),sigma2_P(2,:),sigma2_P(3,:),'r');
