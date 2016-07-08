// test catenary projection in an image

//-------- CATENARY PARAMETERS------------//
// rope semi lenght
R       = 0.5;
// rope sag
H       = 0.3 ;
// max rope sag == fixation point heigh
Hmax    = 0.5 ;
param_a = H/Hmax;


// rope orientation wrt the robot frame
theta   = -15 *%pi/180;
param_ b= sin(theta);



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

// ------- Defining the camera parameters -----------//
im_px      = 600*10^(-6); 
im_py      = 600*10^(-6); 
im_width   = 800;
im_height  = 600; 
im_u0      = im_width/2;
im_v0      = im_height/2;


 
//--------- Defining the system frames -------------- //
// pose of the attached point frame in the robot frame
rTx                  = 0.3; // turtle semi diameter
rTy                  = -0.002;
rTz                  = 0.4;
pose_r_M_sigma2      =  [rTx,rTy,rTz,0,0,0];
r_M_sigma2           = homogeneousMatrixFromPos(pose_r_M_sigma2);
sigma2_M_r           = inv(r_M_sigma2);
 
 
// pose of the camera in the robot frame 
cTx                  = -rTx; // turtle semi diameter
cTy                  = 0.01;
cTz                  = rTz/2; 
pose_r_M_c           = [cTx,cTy,cTz,-%pi/2,%pi/2,0];
r_M_c                = homogeneousMatrixFromPos(pose_r_M_c);
c_M_r                = inv(r_M_c);

// pose of the robot in the world frame
pose_w_M_r           = [1,1,0.2,0,0,0];
w_M_r                = homogeneousMatrixFromPos(pose_w_M_r);
r_M_w                = inv(w_M_r);

// then it follows the frame in the world reference frame
w_M_sigma2           = w_M_r*r_M_sigma2;
w_M_c                = w_M_r*r_M_c; 



figure(1);
Camera3DDraw(0.1,eye(4,4));
Camera3DDraw(0.1,w_M_r);
Camera3DDraw(0.1,w_M_c);
Camera3DDraw(0.1,w_M_sigma2);


figure(2);

figure(3);

//--------- Change point frame --------------------------//
// change point frame
// sigma2 is the frame attached to the fixation point and oriented 
// towards the robot direction

index = 0;


for thetai=-abs(theta):abs(theta)/5:abs(theta)

    index =index + 1;
    
    disp(string(index)+' : '+string(thetai/%pi*180));
    
   // thetai =theta;

    pose_sigma1_M_sigma2 = [-D 0 H 0 0 thetai];
    sigma1_M_sigma2      = homogeneousMatrixFromPos(pose_sigma1_M_sigma2);
    sigma2_M_sigma1      = inv(sigma1_M_sigma2);
    w_M_sigma1           = w_M_sigma2*sigma2_M_sigma1;
    w_P                  = changeFramePoints(sigma1_P,w_M_sigma1);

scf(1);
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
    c_u                 = [];
    c_v                 = [];
    
    
for i=1:(size(c_P,2))
    x = c_P(1,i)/c_P(3,i);
    y = c_P(2,i)/c_P(3,i);
    
    [u,v]= convertMeter2Pixel(x, y,im_u0,im_v0, im_px,im_py)
    if(u>0 & v>0 & u<im_width & v<im_height)
        c_px                 = [c_px;x];
        c_py                 = [c_py;y];
        c_u                  = [c_u;u];
        c_v                  = [c_v;v];
    end 
end

        
scf(2);
plot(c_P(1,:)./c_P(3,:), -c_P(2,:)./c_P(3,:),'r');
plot(c_px,-c_py,'bx');

scf(3);
plot(c_u,-c_v,'bx');

points = [c_px,c_py];
pixels  = [c_u ,c_v];

savematfile('data/data2_'+string(index)+'.mat','points','pixels','-v7');
pause
end


