function com1Mcom2 = computeMotionCoM(vCoM,dt)
//  compute the COM motion if a velocity
// vx vy thetaz is applied during dt seconds

  vcom6ddl = [vCoM(1) vCoM(2) 0 0 0 vCoM(3)]; //expresse the velocity vector in6d
  com1Mcom2 = expMapDirectRxRyRz(vcom6ddl,dt); // expresse the motion
 
  
endfunction
