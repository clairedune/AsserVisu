// test catenary projection in an image
// when the attached points are mobile
clear;
//close;
exec('Load.sce');


//--------- FIXED PARAMETERS -------------------------//
turtleD = 0.6; //diameter
turtleH = 0.6; //height
//-------- CATENARY PARAMETERS------------//
    
// rope semi lenght
    R       = 1;
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
r1Tx                  = -turtleD/2; //turtle semi diameter
r1Ty                  = 0;
r1Tz                  = turtleH/2;
pose_r1_M_sigma3      =  [r1Tx,r1Ty,r1Tz,0,0,0];
r1_M_sigma3           = homogeneousMatrixFromPos(pose_r1_M_sigma3);
sigma3_M_r1           = inv(r1_M_sigma3);

// pose of the attached point frame in the follower robot frame r2
r2Tx                  = turtleD/2; // turtle semi diameter
r2Ty                  = 0;
r2Tz                  = turtleH/2;
pose_r2_M_sigma2      =  [r2Tx,r2Ty,r2Tz,0,0,0];
r2_M_sigma2           = homogeneousMatrixFromPos(pose_r2_M_sigma2);
sigma2_M_r2           = inv(r2_M_sigma2);
 
// pose of the camera in the follower robot frame 
cTx                   = -r2Tx; // turtle semi diameter
cTy                   = 0.01;
cTz                   = r2Tz/2; 
pose_r2_M_c           = [cTx,cTy,cTz,-%pi/2,%pi/2,0];
r2_M_c                = homogeneousMatrixFromPos(pose_r2_M_c);
c_M_r2                = inv(r2_M_c);

// field of view limits for graphic display
c_Fov_t                  = [0,-im_v0*im_py, 1, 1]; // top field of view point for z=1m
c_Fov_b                  = [0,im_v0*im_py, 1, 1]; // bottom field of view point for z=1m
c_Fov_l                  = [-im_u0*im_px,0, 1, 1]; // left field of view point for z=1m
c_Fov_r                  = [im_u0*im_px,0, 1, 1]; // right field of view point for 


//--------------MOVING FRAMES -----------------------------//

// initial position of the robots

// pose of the follower robot in the world frame
angle2                = 0*%pi/180;
pose_w_M_r2           = [0,1,turtleH/2,0,0,angle2];
w_M_r2                = homogeneousMatrixFromPos(pose_w_M_r2);

angle1                = 0;
pose_w_M_r1           = [1.5,1,turtleH/2,0,0,angle1];
w_M_r1                = homogeneousMatrixFromPos(pose_w_M_r1);


//------------- GRAPHICS ------------------------------------//


figure(1);
subplot(211)
a = gca();
a.isoview = "on";
a.data_bounds = [0;4;-2;2];
a.grid=[1,1];
a.filled = "off";

subplot(212)
a = gca();
a.box = "off";
a.filled = "off";
a.isoview = "on";
a.data_bounds = [0;4;-0.1;1];
a.grid=[1,1];

figure(2)
subplot(211);
a = gca();
a.filled = "off";
a.isoview = "on";
a.data_bounds = [-2;2;-2;2];
subplot(212);
a = gca();
a.filled = "off";
a.isoview = "on";
a.data_bounds = [0;800;-600;0];

dt          = 0.1; //second


OPT_3D = 0;

for time = 0:dt:3

    
    r1_M_w                = inv(w_M_r1);
    r2_M_w                = inv(w_M_r2);
    
    // then it follows the frame in the world reference frame
    w_M_sigma3           = w_M_r1*r1_M_sigma3;
    w_M_sigma2           = w_M_r2*r2_M_sigma2;
    w_M_c                = w_M_r2*r2_M_c; 
    
   

    
    sigma2_M_sigma3      = inv(w_M_sigma2)*w_M_sigma3;
    pose_sigma2_M_sigma3 = pFromHomogeneousMatrix(sigma2_M_sigma3);
    
    sigma3_M_sigma2      = inv(sigma2_M_sigma3);
    pose_sigma3_M_sigma2 = pFromHomogeneousMatrix(sigma3_M_sigma2);
    
    // compute theta and D from the relative position of the robots
    angle                = atan(pose_sigma2_M_sigma3(2)/pose_sigma2_M_sigma3(1));
    theta                = -angle ;
    D                    = norm(pose_sigma2_M_sigma3(1:3))/2;
    
    // semi distance between the two fixation points
    // se calcul par rapport a H et R
    if D>R then
        disp("Error the distance between the two robots cannot be greater than the rope length");
    pause
    
    elseif D==R then
        H=0;
    else
        // rope sag estimation
        H = findCatenaryH(R,D);
      
    end
    
    // rope points coordinates in the rope frame
   
    sigma1_X = -D:D/100:D;
    
    C = (2*H)/(R^2-H^2);
    sigma1_Z = 1/C*(cosh(C*sigma1_X)-1);
    
    sigma1_Y = zeros(length(sigma1_X),1)';
    
    // coordinates of one 3D point P = (X,Y,Z,1)
     
    sigma1_P = [sigma1_X;sigma1_Y;sigma1_Z;ones(length(sigma1_X),1)'];
    
    pose_sigma1_M_sigma2 = [-D 0 H 0 0 theta];
    sigma1_M_sigma2      = homogeneousMatrixFromPos(pose_sigma1_M_sigma2);
    sigma2_M_sigma1      = inv(sigma1_M_sigma2);
    
    //deduce the pose of sigma in the world
    w_M_sigma1           = w_M_sigma2*sigma2_M_sigma1;
    w_P                  = changeFramePoints(sigma1_P,w_M_sigma1);
    if(abs(w_P(1,$)-w_M_sigma3(1,4))>0.1)
       theta             = -angle + %pi; 
       pose_sigma1_M_sigma2 = [-D 0 H 0 0 theta];
       sigma1_M_sigma2      = homogeneousMatrixFromPos(pose_sigma1_M_sigma2);
       sigma2_M_sigma1      = inv(sigma1_M_sigma2);
    
       //deduce the pose of sigma in the world
        w_M_sigma1           = w_M_sigma2*sigma2_M_sigma1;
        w_P                  = changeFramePoints(sigma1_P,w_M_sigma1); 
    end 

    
    // Set to zero all the points that are beyond the ground.
    w_P(3,:)=w_P(3,:).*(w_P(3,:)>0);
    
    //--------------------GRAPHICS---------------------------------------//
     // field of view limits for graphic display
    w_Fov_t                  = w_M_c*c_Fov_t'; // top field of view point for z=1m
    w_Fov_b                  = w_M_c*c_Fov_b'; // bottom field of view point for z=1m
    w_Fov_l                  = w_M_c*c_Fov_l'; // left field of view point for z=1m
    w_Fov_r                  = w_M_c*c_Fov_r'; // right field of view point for 

    Fov_top_x                = [w_Fov_l(1),w_M_c(1,4),w_Fov_r(1)];
    Fov_top_y                = [w_Fov_l(2),w_M_c(2,4),w_Fov_r(2)];
    Fov_side_x               = [w_Fov_t(1),w_M_c(1,4),w_Fov_b(1)];
    Fov_side_z               = [w_Fov_t(3),w_M_c(3,4),w_Fov_b(3)];
    
    //------------------- TOP VIEW --------------------------------------//
    pose_w_M_r1              = pFromHomogeneousMatrix(w_M_r1);
    pose_w_M_r2              = pFromHomogeneousMatrix(w_M_r2);
    pose_w_M_sigma2          = pFromHomogeneousMatrix(w_M_sigma2);
    pose_w_M_sigma3          = pFromHomogeneousMatrix(w_M_sigma3);
    
    scf(1);
    subplot(211)
    a = gca();
    delete(a.children);
    drawlater();
    drawTurtleTop(pose_w_M_r1(1),pose_w_M_r1(2),turtleD);
    drawTurtleTop(pose_w_M_r2(1),pose_w_M_r2(2),turtleD);
    plot(pose_w_M_sigma2(1),pose_w_M_sigma2(2),'s');
    plot(pose_w_M_sigma3(1),pose_w_M_sigma3(2),'s');
    plot(Fov_top_x,Fov_top_y);

    if(D>0.8*R)
        plot(w_P(1,:),w_P(2,:),'r');
    else
        plot(w_P(1,:),w_P(2,:),'b');
    end
    drawnow();
    
    //------------------- LATERAL VIEW ----------------------------------//
    subplot(212)
    a = gca();
    delete(a.children);
    drawlater();
    drawTurtleSide(pose_w_M_r1(1),pose_w_M_r1(3),turtleD);
    drawTurtleSide(pose_w_M_r2(1),pose_w_M_r2(3),turtleH);
    plot(pose_w_M_sigma2(1),pose_w_M_sigma2(3),'s');
    plot(pose_w_M_sigma3(1),pose_w_M_sigma3(3),'s');
    plot(Fov_side_x,Fov_side_z);
    
    if(D>0.8*R)
        plot(w_P(1,:),w_P(3,:),'r');
    else
        plot(w_P(1,:),w_P(3,:),'b');
    end        

    drawnow();
    
    if(OPT_3D)
    figure(3)
    Camera3DDraw(0.1,w_M_sigma1);
    Camera3DDrawColor(0.1,w_M_r2,5);
    Camera3DDrawColor(0.1,w_M_sigma2,5);
    Camera3DDrawColor(0.1,w_M_r1,3);
    Camera3DDrawColor(0.1,w_M_sigma3,3);
    Camera3DDrawColor(0.1,w_M_c,5);
    param3d(w_P(1,:),w_P(2,:),w_P(3,:),'r');
    end
    
    // -------- IMAGE FORMATION --------//
    
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
        
    nbpoints = 0;    
    for i=1:(size(c_P,2))
        x = c_P(1,i)/c_P(3,i);
        y = c_P(2,i)/c_P(3,i);
        
        [u,v]= convertMeter2Pixel(x, y,im_u0,im_v0, im_px,im_py)
        if(u>0 & v>0 & u<im_width & v<im_height)
            c_px                 = [c_px;x];
            c_py                 = [c_py;y];
            c_u                  = [c_u;u];
            c_v                  = [c_v;v];
            nbpoints             = nbpoints + 1; 
        end 
    end
    
   if(nbpoints>1)
   scf(2);
   subplot(211);
   a = gca();
   
   
   delete(a.children);
   plot(c_P(1,:)./c_P(3,:), -c_P(2,:)./c_P(3,:),'r');
   plot(c_px,-c_py,'bx');
   
   subplot(212);
   a = gca();
   delete(a.children)
   plot(c_u,-c_v,'bx');
   end
    
   points3D  = [c_PX,c_PY,c_PZ] ; 
   points2D  = [c_px,c_py];
   pixels2D  = [c_u ,c_v];
    
  
  
  // position update
  velocity_r1 = [1,0,0,0,0,0.1]; // arbitrary velocity of the leader robot
  velocity_r2 = [0.5,0,0,0,0,0.1]; // arbitrary velocity of the follower robot
  
  r1_curr_M_r1_next = expMapDirectRxRyRz(velocity_r1,dt);
  r2_curr_M_r2_next = expMapDirectRxRyRz(velocity_r2,dt);
  
  w_M_r1 = w_M_r1*r1_curr_M_r1_next;
  w_M_r2 = w_M_r2*r2_curr_M_r2_next;
  
   sleep(300);
  
end


disp('Pausing .... write resume to exit the application')
pause

//savematfile('data/data2_'+string(index)+'.mat','points','pixels','-v7');
//pause
//end





