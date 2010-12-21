
//------------------------------------------------//
//  PROJECTION 3D POINT
//  author : Claire Dune
//  date   : 04/01/2010 
//------------------------------------------------//
function x     = projection3dPoint(wX,wMc);
  // project a 3D point on a camera image plane
  // return [] if the point id behind the view
  // change the referentiel
  wXhomogeneous= [wX 1]'
  cX           = inv(wMc)*wXhomogeneous;
  if(cX(3)>0)
    x(1)       = cX(1)/cX(3);
    x(2)       = cX(2)/cX(3);
  else
    disp('Warning, the point is behind the camera');
    x          =[];
  end;  
endfunction

//------------------------------------------------//
//  PROJECTION 3D POINT DIRECT
//  author : Claire Dune
//  date   : 04/01/2010 
//------------------------------------------------//
function x   = projection3dPointDirect(cX);
  // project a 3D point on a camera image plane 
  // return [] if the point id behind the view
  if(cX(3)>0)
    x(1)     = cX(1)/cX(3);
    x(2)     = cX(2)/cX(3);
  else
    disp('Warning, the point is behind the camera');
    x(1)     = cX(1)/cX(3);
    x(2)     = cX(2)/cX(3);
  end  
endfunction

//------------------------------------------------//
//  POINT 3D MOTION
//  author : Claire Dune
//  date   : 04/01/2010 
//------------------------------------------------//
function [c2P]  = point3dMotion(c1P,v,dt)
 // compute the motion of a static point
 // in a mobile camera frame
 // moving at velocity v during time dt
 c1Mc2          = expMapDirectRxRyRz(v,dt) ;
 c1Phomogeneous = [c1X 1]'                 ;
 c2Phomogeneous = inv(c1Mc2)*c1Phomogeneous;
 c2P            = c2Phomogeneous(1:3)      ;
endfunction
