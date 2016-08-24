// test catenary projection in an image
// when the attached points are mobile
clear;
//close;
exec('Load.sce');


//--------- FIXED PARAMETERS -------------------------//
turtleD = 0.6; //diameter
turtleH = 1; //height

//-------- CATENARY PARAMETERS------------//
    
// rope semi lenght
R       = 1;
// max rope sag == fixation point heigh
Hmax    = R ;

// -------- VISUAL SERVOING GAIN --------------------//
lambda  = 1;

//--------- VITESSE DU ROBOT LEADER ---------------//
v_r1 = [0,0,0,0,0,0]'; // arbitrary velocity of the leader robot


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
c_Fov_t               = [0,-im_v0*im_py, 1, 1]; // top field of view point for z=1m
c_Fov_b               = [0,im_v0*im_py, 1, 1]; // bottom field of view point for z=1m
c_Fov_l               = [-im_u0*im_px,0, 1, 1]; // left field of view point for z=1m
c_Fov_r               = [im_u0*im_px,0, 1, 1]; // right field of view point for 


//--------------MOVING FRAMES -----------------------------//

// initial position of the robots

// pose of the follower robot in the world frame
angle2                = 0*%pi/180;
pose_w_M_r2           = [0,0.8,turtleH/2,0,0,angle2];
w_M_r2                = homogeneousMatrixFromPos(pose_w_M_r2);

angle1                = 0;
pose_w_M_r1           = [2,1,turtleH/2,0,0,angle1];
w_M_r1                = homogeneousMatrixFromPos(pose_w_M_r1);

// desired position of the follower frame
angled                = 0*%pi/180;
pose_w_M_r2d          = [0.5,1.4,turtleH/2,0,0,angled];
w_M_r2d               = homogeneousMatrixFromPos(pose_w_M_r2d);

[paramd,D,w_Pd,xAd,yAd,zAd,w_M_sigma1d] = thetheredRobotCatenary(w_M_r1,w_M_r2d,r1_M_sigma3,r2_M_sigma2,R,Hmax);
w_M_cd                = w_M_r2d * r2_M_c;
cd_M_sigma1d          = inv(w_M_cd) * w_M_sigma1d ;
[cd_P,cd_pm,cd_pp,nbpoints] = imageProjection(inv(w_M_cd),w_Pd,im_u0,im_v0,im_px,im_py);
disp("paramd")
disp(paramd);

//------------- GRAPHICS ------------------------------------//


figure(1);
subplot(221)
a = gca();
a.isoview = "on";
a.data_bounds = [0;4;-2;2];
a.grid=[1,1];
a.filled = "off";

subplot(222)
a = gca();
a.box = "off";
a.filled = "off";
a.isoview = "on";
a.data_bounds = [0;4;-0.1;1];
a.grid=[1,1];

subplot(223);
a = gca();
a.filled = "off";
a.isoview = "on";
a.data_bounds = [-2;2;-2;2];
subplot(224);
a = gca();
a.filled = "off";
a.isoview = "on";
a.data_bounds = [0;800;-600;0];

dt          = 0.1; //second


OPT_3D = 0;

E      = [];
VR1    = [];
VR2    = [];
PARAM  = [];




for time = 0:dt:10

    // Catenary parametres
    [param,D,w_P,xA,yA,zA,w_M_sigma1] = thetheredRobotCatenary(w_M_r1,w_M_r2,r1_M_sigma3,r2_M_sigma2,R,Hmax);
    
    // -------- IMAGE FORMATION --------//
    w_M_c                = w_M_r2 * r2_M_c;
    c_M_sigma1           = inv(w_M_c) * w_M_sigma1 ;
    [c_P,c_pm,c_pp,nbpoints] = imageProjection(inv(w_M_c),w_P,im_u0,im_v0,im_px,im_py);
 
    // ---- INTERACTION MATRIX FOR 3D  CATENARY ------ //
    e     = param' - paramd';
    L     = catenary3DIntMat( R, Hmax, param, xA, yA, zA )  ;  
    v_r2  = - lambda*pinv(L)*e; 
  
    // position update
    
    E   = [E,e];
    VR1 = [VR1, v_r1];
    VR2 = [VR2, v_r2];     
    PARAM = [PARAM, param];
    
    
    //------------------- TOP VIEW --------------------------------------//
    pose_w_M_r1              = pFromHomogeneousMatrix(w_M_r1);
    pose_w_M_r2              = pFromHomogeneousMatrix(w_M_r2);
    pose_w_M_sigma2          = pFromHomogeneousMatrix(w_M_r2*r2_M_sigma2);
    pose_w_M_sigma3          = pFromHomogeneousMatrix(w_M_r1*r1_M_sigma3);
    
    
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

    subplot(221)
    a = gca();
    delete(a.children);
    drawlater();
    drawTurtleTop(pose_w_M_r1(1),pose_w_M_r1(2),turtleD);
    drawTurtleTop(pose_w_M_r2(1),pose_w_M_r2(2),turtleD);
    drawTurtleTop(pose_w_M_r2d(1),pose_w_M_r2d(2),turtleD);
    plot(pose_w_M_sigma2(1),pose_w_M_sigma2(2),'s');
    plot(pose_w_M_sigma3(1),pose_w_M_sigma3(2),'s');
    plot(Fov_top_x,Fov_top_y);

    if(D>0.8*R)
        plot(w_P(1,:),w_P(2,:),'c');
    else
        plot(w_P(1,:),w_P(2,:),'b');
    end
    
    plot(w_Pd(1,:),w_Pd(2,:),'r--');
    drawnow();
    
    //------------------- LATERAL VIEW ----------------------------------//
    subplot(222)
    a = gca();
    delete(a.children);
    drawlater();
    drawTurtleSide(pose_w_M_r1(1),pose_w_M_r1(3),turtleH);
    drawTurtleSide(pose_w_M_r2(1),pose_w_M_r2(3),turtleH);
    drawTurtleSide(pose_w_M_r2d(1),pose_w_M_r2d(3),turtleH);
    plot(pose_w_M_sigma2(1),pose_w_M_sigma2(3),'s');
    plot(pose_w_M_sigma3(1),pose_w_M_sigma3(3),'s');
    
    
    plot(Fov_side_x,Fov_side_z);
    
    if(D>0.8*R)
        plot(w_P(1,:),w_P(3,:),'c');
    else
        plot(w_P(1,:),w_P(3,:),'b');
    end        
    
    plot(w_Pd(1,:),w_Pd(3,:),'r--');
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
    
      
   if(nbpoints>1)
   subplot(223);
   a = gca();
   
   
   delete(a.children);
   plot(c_P(1,:)./c_P(3,:), -c_P(2,:)./c_P(3,:),'b');
   plot(cd_P(1,:)./cd_P(3,:), -cd_P(2,:)./cd_P(3,:),'r--');
   plot(c_pm(:,1),-c_pm(:,2),'bx');
   
   subplot(224);
   a = gca();
   delete(a.children)
   plot(c_pp(:,1),-c_pp(:,2),'bx');
   plot(cd_pp(:,1),-cd_pp(:,2),'r--');

   end
    
  
  // ---------------------UPDATE --------------------------------
  r1_curr_M_r1_next     = expMapDirectRxRyRz(v_r1',dt);
  r2_curr_M_r2_next     = expMapDirectRxRyRz(v_r2',dt);
  
  w_M_r1                = w_M_r1*r1_curr_M_r1_next;
  w_M_r2                = w_M_r2*r2_curr_M_r2_next;
  r1_M_w                = inv(w_M_r1);
  r2_M_w                = inv(w_M_r2);  
  sleep(300);
  
end


disp('Pausing .... write resume to exit the application')
pause

//savematfile('data/data2_'+string(index)+'.mat','points','pixels','-v7');
//pause
//end





