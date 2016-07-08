

// test catenary projection in an image

//-------- CATENARY PARAMETERS------------//


// rope semi lenght
R  = 0.5;
// rope sag
H = 0.3 ;
// max rope sag == fixation point heigh
Hmax = 0.5 ;
// rope orientation wrt the robot frame
theta = -85 *%pi/180;
a = H/Hmax;

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

//--------- Defining the system frames -------------- //
// pose of the attached point frame in the camera frame
Tx                   = 0.3; // rayon du turle
Ty                   = -0.002;
Tz                   = 0.3;
pose_r_M_sigma2      =  [Tx,Ty,Tz,0,0,0];
r_M_sigma2           = homogeneousMatrixFromPos(pose_r_M_sigma2);
sigma2_M_r           = inv(r_M_sigma2);
 
 
Tx                   = -0.3; // rayon du turle
Ty                   = 0.002;
Tz                   = Tz/2; 
pose_r_M_c           = [Tx,Ty,Tz,-%pi/2,%pi/2,0];
r_M_c                = homogeneousMatrixFromPos(pose_r_M_c);
c_M_r                = inv(r_M_c);

// pose of the robot in the world frame
pose_w_M_r           = [1,1,0.2,0,0,0];
w_M_r                = homogeneousMatrixFromPos(pose_w_M_r);
r_M_w                = inv(w_M_r);

// then it follows the frame in the world reference frame
w_M_sigma2           = w_M_r*r_M_sigma2;
w_M_c                = w_M_r*r_M_c; 



figure(3);
Camera3DDraw(0.1,eye(4,4));
Camera3DDraw(0.1,w_M_r);
Camera3DDraw(0.1,w_M_c);
Camera3DDraw(0.1,w_M_sigma2);


//--------- Change point frame --------------------------//
// change point frame
// sigma2 is the frame attached to the fixation point and oriented 
// towards the robot direction

index = 0;


for thetai=-abs(theta):abs(theta)/5:abs(theta)

    index =index + 1;
    
    disp(index);
    disp(thetai/%pi*180);
    
    thetai =theta;

    pose_sigma1_M_sigma2 = [-D 0 H 0 0 thetai];
    sigma1_M_sigma2      = homogeneousMatrixFromPos(pose_sigma1_M_sigma2);
    sigma2_M_sigma1      = inv(sigma1_M_sigma2);
    w_M_sigma1           = w_M_sigma2*sigma2_M_sigma1;
    w_P                  = changeFramePoints(sigma1_P,w_M_sigma1);

scf(3);
    Camera3DDraw(0.1,w_M_sigma1);
    param3d(w_P(1,:),w_P(2,:),w_P(3,:),'r');

//end 
    c_M_sigma1           = c_M_r * r_M_sigma2 * sigma2_M_sigma1;
    c_P                  = changeFramePoints(sigma1_P,c_M_sigma1);
    c_PX                 = c_P(1,:);
    c_PY                 = c_P(2,:);

//cut frame
    c_px                 = [];
    c_py                 = [];
    
for i=1:(size(c_P,2))
    x = c_P(1,i)/c_P(3,i);
    y = c_P(2,i)/c_P(3,i);
    if(x>-0.2 & y>-0.15 & x<0.2 & y<0.15)
        c_px                 = [c_px;x];
        c_py                 = [c_py;y];
    end 
end

        
scf(5);
plot(c_P(1,:)./c_P(3,:), -c_P(2,:)./c_P(3,:),'r');
plot(c_px,-c_py,'bx');

points = [c_px,c_py];

savematfile('data'+string(index)+'.mat','points','-v7');

end


