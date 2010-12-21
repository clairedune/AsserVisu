//--------------------------------------------------//
// main program
//
// author Claire Dune
// date 24/01/2010
// ;exec('testAsserVisuTous.sce');
//--------------------------------------------------//

clear

//------- Load the functions


// ------ Describe the problem
[cInitMo_m,cFinalMo_m,LFunction_m,target_m,targetSize_m,lambda_m,iterMax_m,Te_m]=avLoopDefaultSettings();
//redefine the problem
cInitMo_m  =[0.1 0.1 1 10*%pi/180 0 0];
cFinalMo_m =[0 0 2 0 0 0];

//--------- AV loop
[avPose_m,avError_m,avVelocity_m,avS_m,avSDes_m] = avLoop(cInitMo_m,cFinalMo_m,LFunction_m,target_m,targetSize_m,lambda_m,iterMax_m,Te_m,0);
[avPoseD_m,avErrorD_m,avVelocityD_m,avSD_m,avSDesD_m] = avLoop(cInitMo_m,cFinalMo_m,matIntMireD,target_m,targetSize_m,lambda_m,iterMax_m,Te_m,0);


//--------- display
hf=scf(1);
hf.figure_name = "Evolution Mire";
mireEvolutionDraw(iterMax_m,avS_m,3)
mireEvolutionDraw(iterMax_m,avSD_m,3)
show_pixmap()

// create a window to display the error
hf2=scf(2);
hf2.figure_name = "Error";
plot(avError_m,'b')
plot(avErrorD_m,'r')
show_pixmap()    

// create a window to display the error
hf3=scf(3);
hf3.figure_name = "Velocities";
plot(avVelocity_m,'b');
plot(avVelocityD_m,'r');
show_pixmap()

//create a window to display the camera pose
hf3d1_m   = createFigure3D(4,"Camera Motion",1);
for (i=1:4:iterMax_m)
 cMo = homogeneousMatrixFromPos(avPose_m(i,:)) ;
 Camera3DDraw(0.1,inv(cMo));                  // display the current camera  
end
show_pixmap()
