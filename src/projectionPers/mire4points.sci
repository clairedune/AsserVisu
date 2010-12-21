//--------------------------------------//
// Define a 5 dots target
//--------------------------------------//
function [oX] = mire4points(L)
// on definit 5 points dans le repere objet
//
//      Y
//      ^
// P1   |  P2
//    --------->x
// P3      P4


// create target
oP1 = [-L/2   L/2  0]';
oP2 = [ L/2   L/2  0]';
oP3 = [-L/2  -L/2  0]';
oP4 = [ L/2  -L/2  0]';

oX = [oP1 ;oP2 ;oP3 ;oP4 ];

endfunction

//--------------------------------------//
// Create a target in the camera from
// Compute the point in the camera frame
// and its projections
//--------------------------------------//
function [oP,cP,p]=mire4pointsInCam(L,cMo)
  // create a target in the camera frame,
  // oP 3D points in the target frame
  // cP 3D points in the camera frame
  // p the projeciton in the image plane
  
  // create the target in teh taget frame
  L  = 0.5; // related to the target size 
  oP = mire4points (L)          ; 
  cP = changeFrameMire(oP,cMo)  ; 
  // compute the  projection on the view
  p  = projectMireDirect(cP) ;

endfunction





