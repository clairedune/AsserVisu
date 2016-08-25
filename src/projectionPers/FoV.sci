function c_FoV = FoV (im_u0,im_v0,im_px,im_py)
    // field of view limits for graphic display
    // Define 4 3D points that represent the limits of the FoV in 3D
    
    c_FoV_t               = [0,-im_v0*im_py, 1, 1]; // top field of view point for z=1m
    c_FoV_b               = [0,im_v0*im_py, 1, 1]; // bottom field of view point for z=1m
    c_FoV_l               = [-im_u0*im_px,0, 1, 1]; // left field of view point for z=1m
    c_FoV_r               = [im_u0*im_px,0, 1, 1]; // right field of view point for 
    
    c_FoV = [c_FoV_t',c_FoV_b',c_FoV_l',c_FoV_r'];
    
endfunction

function drawFoVTop(c_Fov,w_M_c)
    w_Fov                    = changeFramePoints(c_FoV, w_M_c); 
    Fov_top_x                = [w_Fov(1,3),w_M_c(1,4),w_Fov(1,4)];
    Fov_top_y                = [w_Fov(2,3),w_M_c(2,4),w_Fov(2,4)];
    plot(Fov_top_x,Fov_top_y);
endfunction


function drawFoVSide(c_Fov, w_M_c)
    w_Fov                    = changeFramePoints(c_Fov, w_M_c);
    Fov_side_x               = [w_Fov(1,1),w_M_c(1,4),w_Fov(1,2)];
    Fov_side_z               = [w_Fov(3,1),w_M_c(3,4),w_Fov(3,2)];
    plot(Fov_side_x,Fov_side_z);
endfunction
