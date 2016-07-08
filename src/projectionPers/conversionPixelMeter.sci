function [x,y]=convertPixel2Meter(u,v,u0,v0,px,py)
    x = (u-u0)*px;
    y = (v-v0)*py;
endfunction


function [u,v]=convertMeter2Pixel(x,y,u0,v0,px,py)
    u = round(x/px+u0);
    v = round(y/py+v0);
endfunction
