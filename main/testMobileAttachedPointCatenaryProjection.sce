// test catenary projection in an image
// when the attached points are mobile
clear;
//close;
exec('Load.sce');


//--------- FIXED PARAMETERS -------------------------//

//-------- CATENARY PARAMETERS------------//
    
// rope semi lenght
    R       = 0.7;
// max rope sag == fixation point heigh
    Hmax    = R ;

// ------- Defining the camera parameters -----------//
im_px      = 600*10^(-6); 
im_py      = 600*10^(-6); 
im_width   = 800;
im_height  = 600; 
im_u0      = im_width/2;
im_v0      = im_height/2;


 
//--------- Defining the system frames -------------- //

// pose of the attached point frame in the leader robot frame r1
r1Tx                  = -0.3; //::::::::::::::::::::::::::::::::::::::::::: turtle semi diameter
r1Ty                  = -%eps;
r1Tz                  = 0.4;
pose_r1_M_sigma3      =  [r1Tx,r1Ty,r1Tz,0,0,0];
r1_M_sigma3           = homogeneousMatrixFromPos(pose_r1_M_sigma3);
sigma3_M_r1           = inv(r1_M_sigma3);

// pose of the attached point frame in the follower robot frame r2
r2Tx                  = 0.3; // turtle semi diameter
r2Ty                  = %eps;
r2Tz                  = 0.4;
pose_r2_M_sigma2      =  [r2Tx,r2Ty,r2Tz,0,0,0];
r2_M_sigma2           = homogeneousMatrixFromPos(pose_r2_M_sigma2);
sigma2_M_r2           = inv(r2_M_sigma2);
 
// pose of the camera in the follower robot frame 
cTx                  = -r2Tx; // turtle semi diameter
cTy                  = 0.01;
cTz                  = r2Tz/2; 
pose_r2_M_c           = [cTx,cTy,cTz,-%pi/2,%pi/2,0];
r2_M_c                = homogeneousMatrixFromPos(pose_r2_M_c);
c_M_r2                = inv(r2_M_c);


//--------------MOVING FRAMES -----------------------------//


for time = 0:0.1:1

    // pose of the follower robot in the world frame
    angle2                = 15*%pi/180;
    pose_w_M_r2           = [1+time*1.05,1,0.2,0,0,angle2+time];
    w_M_r2                = homogeneousMatrixFromPos(pose_w_M_r2);
    r2_M_w                = inv(w_M_r2);
    
    angle1                = 0;
    pose_w_M_r1           = [2.5+time,1,0.2,0,0,angle1];
    w_M_r1                = homogeneousMatrixFromPos(pose_w_M_r1);
    r1_M_w                = inv(w_M_r1);
    
    
    // then it follows the frame in the world reference frame
    w_M_sigma3           = w_M_r1*r1_M_sigma3;
    w_M_sigma2           = w_M_r2*r2_M_sigma2;
    w_M_c                = w_M_r2*r2_M_c; 
    
    
    figure(1);
    if(time==0)
        Camera3DDraw(0.1,eye(4,4));
        Camera3DDrawColor(0.1,w_M_r2,5);
        Camera3DDrawColor(0.1,w_M_sigma2,5);
        Camera3DDrawColor(0.1,w_M_r1,3);
        Camera3DDrawColor(0.1,w_M_sigma3,3);
        Camera3DDrawColor(0.1,w_M_c,5);
    end
    
    // transformation between sigma2 and sigma3 to know the distance between the 
    // attached points
    
    sigma2_M_sigma3      = inv(w_M_sigma2)*w_M_sigma3;
    pose_sigma2_M_sigma3 = pFromHomogeneousMatrix(sigma2_M_sigma3);
    
    sigma3_M_sigma2      = inv(sigma2_M_sigma3);
    pose_sigma3_M_sigma2 = pFromHomogeneousMatrix(sigma3_M_sigma2);
    
    theta                =atan(pose_sigma2_M_sigma3(2)/pose_sigma2_M_sigma3(1));
    D                    = norm(pose_sigma2_M_sigma3(1:3))/2;
    
    disp("theta");
    disp(theta*180/%pi);
    
    disp("D");
    disp(D);
    

    
    
    // semi distance between the two fixation points
    // se calcul par rapport a H et R
    if D>R then
        disp("Error the distance between the two robots cannot be greater than 2R");
        pause;
    elseif D==R then
        H=0;
    else
        // rope sag estimation
        H = findCatenaryH(R,D);
      
    end
    
    // C
    C = (2*H)/(R^2-H^2);
    
    // rope points coordinates in the rope frame
    sigma1_X = -D:D/100:D;
    sigma1_Z = 1/C*(cosh(C*sigma1_X)-1);
    sigma1_Y = zeros(length(sigma1_X),1)';
    // coordinates of one 3D point P = (X,Y,Z,1)
    sigma1_P = [sigma1_X;sigma1_Y;sigma1_Z;ones(length(sigma1_X),1)'];
    
    pose_sigma1_M_sigma2 = [-D 0 H 0 0 -theta];
    sigma1_M_sigma2      = homogeneousMatrixFromPos(pose_sigma1_M_sigma2);
    sigma2_M_sigma1      = inv(sigma1_M_sigma2);
    w_M_sigma1           = w_M_sigma2*sigma2_M_sigma1;
    w_P                  = changeFramePoints(sigma1_P,w_M_sigma1);
    
    scf(1);
        Camera3DDraw(0.1,w_M_sigma1);
        Camera3DDrawColor(0.1,w_M_r2,5);
        Camera3DDrawColor(0.1,w_M_sigma2,5);
        Camera3DDrawColor(0.1,w_M_r1,3);
        Camera3DDrawColor(0.1,w_M_sigma3,3);
        Camera3DDrawColor(0.1,w_M_c,5);
        param3d(w_P(1,:),w_P(2,:),w_P(3,:),'r');




    
    
    figure(2);
    
    figure(3);
    
    
        c_M_sigma1           = c_M_r2 * r2_M_sigma2 * sigma2_M_sigma1;
        c_P                  = changeFramePoints(sigma1_P,c_M_sigma1);
        c_PX                 = c_P(1,:); // points 3D en m dans le repere camera
        c_PY                 = c_P(2,:); // points 3D en m dans le repere camera
        c_PZ                 = c_P(3,:); // points 3D en m dans le repere camera
    
    //cut frame
        c_px                 = []; // points 2D en m dans le plan image
        c_py                 = []; // points 2D en m dans le plan image
        c_u                  = []; // points 2D en pixels dans le plan image
        c_v                  = []; // points 2D en pixels dans le plan image
        
        
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
    
    points3D  = [c_PX,c_PY,c_PZ] ; 
    points2D  = [c_px,c_py];
    pixels2D  = [c_u ,c_v];
    

end


//savematfile('data/data2_'+string(index)+'.mat','points','pixels','-v7');
//pause
//end





