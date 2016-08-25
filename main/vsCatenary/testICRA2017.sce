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
turtleH = 0.6; //height

//-------- CATENARY PARAMETERS------------//
R       = 1; // rope semi lenght
Hmax    = R ;// max rope sag == fixation point heigh

//--------- INITIAL EXPE SETUP-------------------------//
// LEADER INIT POSITION AND VELOCITY
angle1                = 0*%pi/180;                       // angle around vertical axis
pose_w_M_r1           = [2.5,0,turtleH/2,0,0,angle1];      // pose in general frame
v_r1                  = [0,0,0,0,0,0]';                  // arbitrary velocity 

//FOLLOWER INIT POSITION
angle2                = 0*%pi/180;                       // angle around vertical axis
pose_w_M_r2           = [0,0.4,turtleH/2,0,0,angle2];    // pose in general frame
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
//cTx                   = -turtleD/2; // turtle semi diameter
//cTy                   = 0;
//cTz                   = turtleH/2; 
//pose_r2_M_c           = [cTx,cTy,cTz,-%pi/2,%pi/2,0];
r2_M_c                = homogeneousMatrixFromPos(pose_r2_M_c);
c_M_r2                = inv(r2_M_c);

//--------------ROBOT FRAMES DEFINITION -----------------------------//
// pose of the leader robot in the world frame
w_M_r1                = homogeneousMatrixFromPos(pose_w_M_r1);

// pose of the follower robot in the world frame
w_M_r2                = homogeneousMatrixFromPos(pose_w_M_r2);

//----------------- COMPUTE THE DESIRED PARAMETERS ----------------------------//
// desired position of the follower frame
w_M_r2d               = homogeneousMatrixFromPos(pose_w_M_r2d);

//compute the desired parameters  
[paramd,D,w_Pd,xAd,yAd,zAd,w_M_sigma1d] = thetheredRobotCatenary(w_M_r1,w_M_r2d,r1_M_sigma3,r2_M_sigma2,R,Hmax);

//desired camera position
w_M_cd                = w_M_r2d * r2_M_c;

//desired rope frame wrt desired camera
cd_M_sigma1d          = inv(w_M_cd) * w_M_sigma1d ;

//image projection and desired 2D points definition
[cd_P,cd_pm,cd_pp,nbpoints] = imageProjection(inv(w_M_cd),w_Pd,im_u0,im_v0,im_px,im_py);

// display the desired parameters
disp("Parameters to reach")
disp(paramd);

//------------- GRAPHICS ------------------------------------//
figure(1);
subplot(221)
a = gca();
a.isoview = "on";
a.data_bounds = [0;4;-2;2];
a.grid=[1,1];

subplot(223)
a = gca();
a.isoview = "on";
a.data_bounds = [0;4;-0.1;1];
a.grid=[1,1];

subplot(222);
a = gca();
a.isoview = "on";
a.data_bounds = [-1;1;-1;1];
subplot(224);
a = gca();
a.isoview = "on";
a.data_bounds = [0;800;-600;0];

if(OPT_3D)
    figure(2)
    a=gca()
    a.isoview="on";
end


dt          = 0.1; //second

E      = [];
VR1    = [];
VR2    = [];
PARAM  = [];


scf(1)

for time = 0:dt:10

    // Catenary parametres
    [param,D,w_P,xA,yA,zA,w_M_sigma1] = thetheredRobotCatenary(w_M_r1,w_M_r2,r1_M_sigma3,r2_M_sigma2,R,Hmax);
    // Set to zero all the points that are beyond the ground.
    w_P_positif      = w_P;
    w_P_positif(3,:) = w_P(3,:).*(w_P(3,:)>0);
    
    // -------- IMAGE FORMATION --------//
    w_M_c                = w_M_r2 * r2_M_c;
    c_M_sigma1           = inv(w_M_c) * w_M_sigma1 ;
    [c_P,c_pm,c_pp,nbpoints] = imageProjection(inv(w_M_c),w_P,im_u0,im_v0,im_px,im_py);
    
    // ---- INTERACTION MATRIX FOR 3D  CATENARY ------ //
    e     = param' - paramd';
    L     = catenary3DIntMat( R, Hmax, param, xA, yA, zA )  ;  
    v_r2  = - lambda*pinv(L)*e; 
  
    
    //------------------- TOP VIEW --------------------------------------//
    pose_w_M_r1              = pFromHomogeneousMatrix(w_M_r1);
    pose_w_M_r2              = pFromHomogeneousMatrix(w_M_r2);
    pose_w_M_sigma2          = pFromHomogeneousMatrix(w_M_r2*r2_M_sigma2);
    pose_w_M_sigma3          = pFromHomogeneousMatrix(w_M_r1*r1_M_sigma3);
    
    //--------------------GRAPHICS---------------------------------------//

    subplot(221)
    a = gca();
    delete(a.children);
    drawlater();
    drawTurtleTop(pose_w_M_r1(1),pose_w_M_r1(2),turtleD);// leader
    drawTurtleTop(pose_w_M_r2(1),pose_w_M_r2(2),turtleD);//Follower
 //   drawTurtleTop(pose_w_M_r2d(1),pose_w_M_r2d(2),turtleD);//Desired init Follower
    drawFoVTop(c_FoV,w_M_c) ;                            // FoV 
    plot(pose_w_M_sigma2(1),pose_w_M_sigma2(2),'s');     // Leader Robot Center
    plot(pose_w_M_sigma3(1),pose_w_M_sigma3(2),'s');     // Follower Robot Center  
    if(D>0.8*R)
        plot(w_P_positif(1,:),w_P_positif(2,:),'c');
    else
        plot(w_P_positif(1,:),w_P_positif(2,:),'b');
    end
    //plot(w_Pd(1,:),w_Pd(2,:),'r--');
    drawnow();
    
    //------------------- LATERAL VIEW ----------------------------------//
    subplot(223)
    a = gca();
    delete(a.children);
    drawlater();
    drawTurtleSide(pose_w_M_r1(1),pose_w_M_r1(3),turtleH);   // Leader
    drawTurtleSide(pose_w_M_r2(1),pose_w_M_r2(3),turtleH);   // Follower
    drawFoVSide(c_FoV,w_M_c);                                // FoV of the Follower
    //drawTurtleSide(pose_w_M_r2d(1),pose_w_M_r2d(3),turtleH); // Desired

    plot(pose_w_M_sigma2(1),pose_w_M_sigma2(3),'s');
    plot(pose_w_M_sigma3(1),pose_w_M_sigma3(3),'s');
    
    if(D>0.8*R)
        plot(w_P_positif(1,:),w_P_positif(3,:),'c');
    else
        plot(w_P_positif(1,:),w_P_positif(3,:),'b');
    end        
    
  //  plot(w_Pd(1,:),w_Pd(3,:),'r--');
    drawnow();
        
    if(OPT_3D)  // if the 3D option is turned on display 3D view of the experiment
    scf(2)
    a = gca();
    delete(a.children);

    drawlater()
    Camera3DDraw(0.1,w_M_sigma1);
    Camera3DDrawColor(0.1,w_M_r2,5);
    Camera3DDrawColor(0.1,w_M_r2*r2_M_sigma2,5);
    Camera3DDrawColor(0.1,w_M_r1,3);
    Camera3DDrawColor(0.1,w_M_r1*r1_M_sigma3,3);
    Camera3DDrawColor(0.1,w_M_c,5);
    param3d(w_P_positif(1,:),w_P_positif(2,:),w_P_positif(3,:),'r');
    drawnow();
    scf(1)
    end
    
      
   if(nbpoints>1) // if there are some points in the fov
        subplot(222);
       a = gca();
       delete(a.children);
       plot(c_P(1,:)./c_P(3,:), -c_P(2,:)./c_P(3,:),'b');
       plot(cd_P(1,:)./cd_P(3,:), -cd_P(2,:)./cd_P(3,:),'r--');
       plot(c_pm(:,1),-c_pm(:,2),'bx');
       
       subplot(224);
       a = gca();
       delete(a.children)
       plot(c_pp(:,1),-c_pp(:,2),'bx');
       plot(cd_pp(:,1),-cd_pp(:,2),'ro');
   end
    
    
  //---------------- store the control values -------------------
    E   = [E,e];
    VR1 = [VR1, v_r1];
    VR2 = [VR2, v_r2];     
    PARAM = [PARAM, param];
      
  
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





