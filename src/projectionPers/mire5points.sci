//--------------------------------------//
// Define a 5 dots target
//--------------------------------------//
function [oX] = mire5points(L)
// on definit 5 points dans le repere objet
//
//          Y
//          ^
// P1       |  P2
//         P3 --------->x
// P4          P5


// create target
oP1 = [-L    L/2  0]';
oP2 = [ L/2  L/2  0]';
oP3 = [ 0    0    0]';
oP4 = [-L   -L/2  0]';
oP5 = [ L/2 -L/2  0]';

oX = [oP1 ;oP2 ;oP3 ;oP4 ;oP5];

endfunction



//--------------------------------------//
// Change the frame
//--------------------------------------//
function wX = changeFrameMire(oX,wMo)
//change frame
  N = length(oX)/3;
  wX=[];
  for i=1:N
   wP = wMo*[oX((i-1)*3+1:(i-1)*3+3) ; 1];
   wX = [wX; wP(1:3)]
  end
endfunction

//--------------------------------------//
// Change the frame and project
//--------------------------------------//
function x = projectMire(wX,wMc)
  // given the position of the target in the world frame
  // given the position of the camera in the world frame
  // project the target in the view
  N = length(wX)/3;
  x=[];
  for i=1:N
    x = [x;projection3dPoint((wX((i-1)*3+1:(i-1)*3+3))',wMc)];
  end
endfunction

// ---------------------------------//
// If the points are expressed      //
// in the camera frame, project     //
// direct                           //
//----------------------------------//

function x = projectMireDirect(cX)
  // given the position of the target in the camera frame
  // project the target in the view
  N = length(cX)/3;
  x=[];
  for i=1:N
    x = [x;projection3dPointDirect( (cX((i-1)*3+1:(i-1)*3+3))' )];
    
  end

endfunction

//--------------------------------------//
// Create a target in the camera from
// Compute the point in the camera frame
// and its projections
//--------------------------------------//
function [oP,cP,p]=mire5pointsInCam(L,cMo)
  // create a target in the camera frame,
  // oP 3D points in the target frame
  // cP 3D points in the camera frame
  // p the projeciton in the image plane
  
  // create the target in teh taget frame
  L  = 0.5; // related to the target size 
  oP = mire5points (L)          ; 
  cP = changeFrameMire(oP,cMo)  ; 
  // compute the  projection on the view
  p  = projectMireDirect(cP) ;

endfunction

//----------------------------------------------//
// Compute 3D points motion in the camera frame  //
// given the camera velocity, the previous pose //
// and the sampling time                        //
//----------------------------------------------//
function [c2P, p2] = mireMotion(c1P,v,dt)
 c1Mc2 = expMapDirectRxRyRz(v,dt)       ;
 c2P   = changeFrameMire(c1P,inv(c1Mc2));
 p2    = projectMireDirect(c2P)   ;
endfunction



