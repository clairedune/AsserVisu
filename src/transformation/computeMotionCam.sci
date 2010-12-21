
function M = computeMotion(v,dt)
 if(length(v)==6)
    M = expMapDirectRxRyRz(v,dt);
 else 
    M = eye(4,4);
    disp('WARNING <<computeMotionCam: v = 3');
 end;
endfunction

function cam1Mcam2 = computeMotionCam(vCam,dt)
//  compute the camera motion if a velocity
// vx vy thetaz is applied during dt seconds
 if(length(vCam)==6)
   v = vCam;
 elseif(length(vCam)==3)
   v = [vCam(1) 0 vCam(2) 0 vCam(3) 0]; //expresse the velocity vector in6d
  // expresse the motion
  else 
    v = zeros(1,6);
    disp('warning v dimensions');
end;
   cam1Mcam2 = expMapDirectRxRyRz(v,dt);
endfunction
