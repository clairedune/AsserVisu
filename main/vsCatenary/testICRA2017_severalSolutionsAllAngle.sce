// test catenary projection in an image
// when the attached points are mobile
clear;
//close;
exec('../../Load.sce');

//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

//            USER PARAMETERS

//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

//--------- TURTLE DIMENSIONS -------------------------//
turtleD = 0.6; //diameter
turtleH = 0.8; //height

//-------- CATENARY PARAMETERS------------//
R       = 0.6; // rope semi lenght
Hmax    = R ;// max rope sag == fixation point heigh

//--------- INITIAL EXPE SETUP-------------------------//
// LEADER INIT POSITION AND VELOCITY
angle1                = 0*%pi/180;                       // angle around vertical axis
pose_w_M_r1           = [2,0,turtleH/2,0,0,angle1];      // pose in general frame
v_r1                  = [0,0,0,0,0,0]';                  // arbitrary velocity 


//FOLLOWER DESIRED POSITION
angled                = 0*%pi/180;                       // angle around vertical axis
pose_w_M_r2d          = [0.5,0.4,turtleH/2,0,0,angled];  // pose in general frame

// -------- VISUAL SERVOING GAIN --------------------//
lambda  = 1;

// --------- OPTION FOR GRAPHICAL DISPLAY ----------//
OPT_3D  = 1; // set to 1 to display 3D view

// ------- Camera parameters -----------//
im_px      = 600*10^(-6); 
im_py      = 600*10^(-6); 
im_width   = 800;
im_height  = 600; 
im_u0      = im_width/2;
im_v0      = im_height/2;

// pose of the camera in the follower robot frame 
r2Tc_x                = -turtleD/2; // turtle semi diameter
r2Tc_y                = 0;
r2Tc_z                = 0; 
pose_r2_M_c           = [r2Tc_x,r2Tc_y,r2Tc_z,-%pi/2,%pi/2,0];

//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

//GRAPHICAL INTERFACE

//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
figure(1);
a = gca();
a.isoview = "on";
a.data_bounds = [0;4;-2;2];
//a.grid=[1,1];

//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

// define 3D points to draw camera FOV
c_FoV      = FoV (im_u0,im_v0,im_px,im_py);
 
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
 
//// pose of the camera in the follower robot frame 
r2_M_c                = homogeneousMatrixFromPos(pose_r2_M_c);
c_M_r2                = inv(r2_M_c);

//--------------ROBOT FRAMES DEFINITION -----------------------------//
// pose of the leader robot in the world frame
w_M_r1                = homogeneousMatrixFromPos(pose_w_M_r1);
// desired position of the follower frame
w_M_r2d               = homogeneousMatrixFromPos(pose_w_M_r2d);

//----------------- COMPUTE THE DESIRED PARAMETERS ----------------------------//

//compute the desired parameters  
[paramd,Dd,w_Pd,xAd,yAd,zAd,w_M_sigma1d] = thetheredRobotCatenary(w_M_r1,w_M_r2d,r1_M_sigma3,r2_M_sigma2,R,Hmax);

//desired camera position
w_M_cd                = w_M_r2d * r2_M_c;

//desired rope frame wrt desired camera
cd_M_sigma1d          = inv(w_M_cd) * w_M_sigma1d ;

//image projection and desired 2D points definition
[cd_P,cd_pm,cd_pp,nbpoints] = imageProjection(inv(w_M_cd),w_Pd,im_u0,im_v0,im_px,im_py);

// display the desired parameters
disp("Parameters to reach")
disp(paramd);

//---------------- FOR N FOLLOWER ROBOTS ON A CIRCLE AROUND THE ATTACHED POINT -----------//

for alpha = %pi/2+45*%pi/180:45*%pi/180:3*%pi/2-45*%pi/180

    // initial pose of the follower robot in the world frame
    angle2                = 0*%pi/180;                       // angle around vertical axis
    pose_sigma3_M_sigma2  = [2*Dd*cos(alpha),2*Dd*sin(alpha),0,0,0,angle2];    // pose in general frame
    sigma3_M_sigma2       = homogeneousMatrixFromPos(pose_sigma3_M_sigma2);
    w_M_r2                = w_M_r1*r1_M_sigma3 * sigma3_M_sigma2 * inv(r2_M_sigma2);
    [param,D,w_P,xA,yA,zA,w_M_sigma1] = thetheredRobotCatenary(w_M_r1,w_M_r2,r1_M_sigma3,r2_M_sigma2,R,Hmax);
    
    
    // find the final follower robot position so that the desired features theta and H/Hmax are reached
    [w_M_r1,w_M_r2,w_P,param] = vsCatenaryHth(w_M_r1,v_r1,w_M_r2,r1_M_sigma3,r2_M_sigma2,r2_M_c,R,Hmax, 0.01,0.1);
    
    
    
    disp("alpha")
    disp(alpha)
    disp("Parameters reached")
    disp(param);
   
    if(length(w_M_r2)>0)
        scf(1)
        //------------------- TOP VIEW --------------------------------------//
        pose_w_M_r1              = pFromHomogeneousMatrix(w_M_r1);
        pose_w_M_r2              = pFromHomogeneousMatrix(w_M_r2);
        pose_w_M_sigma2          = pFromHomogeneousMatrix(w_M_r2*r2_M_sigma2);
        pose_w_M_sigma3          = pFromHomogeneousMatrix(w_M_r1*r1_M_sigma3);
        pose_w_M_sigma4          = pFromHomogeneousMatrix(w_M_r1*r2_M_sigma2);
        drawTurtleTop(pose_w_M_r1(1),pose_w_M_r1(2),turtleD);// leader
        drawTurtleTop(pose_w_M_r2(1),pose_w_M_r2(2),turtleD);//Follower
        //    drawFoVTop(c_FoV,w_M_c) ;                            // FoV 
        plot(pose_w_M_sigma2(1),pose_w_M_sigma2(2),'.r');     // Leader Robot Center
        plot(pose_w_M_sigma3(1),pose_w_M_sigma3(2),'.r');     // Follower Robot Center  
        nx = [pose_w_M_r2(1) pose_w_M_sigma2(1)];
        ny = [pose_w_M_r2(2) pose_w_M_sigma2(2)];
        xarrows(nx, ny) ;
        nx = [pose_w_M_r1(1) pose_w_M_sigma4(1)];
        ny = [pose_w_M_r1(2) pose_w_M_sigma4(2)];
        xarrows(nx, ny) ;
        xarc(pose_w_M_sigma3(1)-2*Dd,pose_w_M_sigma3(2)+2*Dd,4*Dd,4*Dd,0,360*64);
        plot(w_P(1,:),w_P(2,:),'r');
    
        
        
    
    
    
    drawnow();
    
    
    scf(2)
    //a = gca();
   // delete(a.children);

    drawlater()
    Camera3DDrawColor(0.1,w_M_r2,5);
    Camera3DDrawColor(0.1,w_M_r2*r2_M_sigma2,5);
    Camera3DDrawColor(0.1,w_M_r1,3);
    Camera3DDrawColor(0.1,w_M_r1*r1_M_sigma3,3);
   // Camera3DDrawColor(0.1,w_M_c,5);
    param3d(w_P(1,:),w_P(2,:),w_P(3,:),'r');
    drawnow();

end
end



disp('Pausing .... write resume to exit the application')
pause

//savematfile('data/data2_'+string(index)+'.mat','points','pixels','-v7');
//pause
//end





