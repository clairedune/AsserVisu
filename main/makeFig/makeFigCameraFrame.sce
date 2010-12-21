//
// Create a figure that display the HRP2 camera frame
// with regards to the waist frame
//
// date : 20/05/10
//--------------
//;exec('main/makeFig/makeFigCameraFrame.sce')




// Init Pose of the Waist expressed in the world frame
posewMwaist	= [0 0 0 0 0 0 ];
wMwaist     = homogeneousMatrixFromPos(posewMwaist);

// pose of the head / waist
//posewaistMh	= [-0.025 0 0.648 0 0 0 ];

//waistMh 	   = homogeneousMatrixFromPos(posewaistMh) ;
 waistMhrbt 	    =[ 0 1 0  0.025
                 1 0 0  0
                 0 0 -1 0.648
                 0 0 0  1    ];

 // change frame head vision / robot 

posehrbtMhvision = [0 0 0 0  %pi %pi/2 ];
hrbtMhvision 	   =homogeneousMatrixFromPos(posehrbtMhvision);             
             


// pose of the head robot / waist
wMhrbt		  = wMwaist*waistMhrbt;

// pose of the head vision / waist
wMhvision		  = wMwaist*waistMhrbt*hrbtMhvision;


// pose of the head / object
visionhMo		  = fscanfMat(home+'/data/hrp2CamParam/headTorg.hmat');

// pose of the vision / world 
wMo         = wMhvision*visionhMo;

 



// pose of the cameras / head vision
c0Mhvision  = fscanfMat(home+'/data/hrp2CamParam/cMh.0');
c1Mhvision  = fscanfMat(home+'/data/hrp2CamParam/cMh.1');
c2Mhvision  = fscanfMat(home+'/data/hrp2CamParam/cMh.2');
c3Mhvision  = fscanfMat(home+'/data/hrp2CamParam/cMh.3');



// compute the cameras positions in the waist frame
wMc0		  = wMhvision*inv(c0Mhvision);
wMc1		  = wMhvision*inv(c1Mhvision);
wMc2		  = wMhvision*inv(c2Mhvision);
wMc3		  = wMhvision*inv(c3Mhvision);


// create the window
hf_1        = createFigure3D(1,"Camera Position llvs/ Waist",1);

Camera3DDraw(0.2,wMwaist)
Camera3DDrawColor(0.2,wMhrbt,9)
Camera3DDrawColor(0.2,wMo,6)

//Camera3DDrawColor(0.4,wMh,9)

//Camera3DDrawColor(0.1,wMc0,5)
//Camera3DDrawColor(0.1,wMc1,3)
//Camera3DDrawColor(0.1,wMc2,4)
//Camera3DDrawColor(0.1,wMc3,2)

Camera3DDraw(0.1,wMhvision)
Camera3DDraw(0.1,wMc0)
Camera3DDraw(0.1,wMc1)
Camera3DDraw(0.1,wMc2)
Camera3DDraw(0.1,wMc3)



show_pixmap()
disp('Pausing .... write resume to exit the application')
pause

//close figure
disp('Closing the figures.... ')
xset("pixmap",0);
delete(hf_1);  
  

disp('----- the end ---- ')




