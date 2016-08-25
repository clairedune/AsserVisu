function [param,D,wP,XA,YA,ZA,wMsigma1] = thetheredRobotCatenary(wMr1,wMr2,r1Msigma3,r2Msigma2,R,Hmax)
    
     // then it follows the frame in the world reference frame
    wMsigma3           = wMr1*r1Msigma3;
    wMsigma2           = wMr2*r2Msigma2;
    
    sigma2Msigma3      = inv(wMsigma2)*wMsigma3;
    posesigma2Msigma3  = pFromHomogeneousMatrix(sigma2Msigma3);
    
    sigma3Msigma2      = inv(sigma2Msigma3);
    posesigma3Msigma2  = pFromHomogeneousMatrix(sigma3Msigma2);
    
    // compute the attached point of the rope on the leader robot in the follower frame
    XA = posesigma2Msigma3(1);
    YA = posesigma2Msigma3(2);
    ZA = posesigma2Msigma3(3);
    
    
    // compute theta and D from the relative position of the robots
    angle                = atan(posesigma2Msigma3(2),posesigma2Msigma3(1)); 
    theta                = -angle;
    D                    = norm(posesigma2Msigma3(1:3))/2;
    
    // semi distance between the two fixation points
    // se calcul par rapport a H et R
    if D>R then
        disp("Error the distance between the two robots cannot be greater than the rope length");
    pause
    
    elseif D==R then
        H=0;
    else
        // rope sag estimation
        H = findCatenaryH(R,D);     
    end
    
    // rope points coordinates in the rope frame
    sigma1X = -D:D/100:D;
    C = (2*H)/(R^2-H^2);
    sigma1Z = 1/C*(cosh(C*sigma1X)-1);
    sigma1Y = zeros(length(sigma1X),1)';
    
    // coordinates of one 3D point P = (X,Y,Z,1)     
    sigma1P = [sigma1X;sigma1Y;sigma1Z;ones(length(sigma1X),1)'];
    posesigma1Msigma2 = [-D 0 H 0 0 theta];
    sigma1Msigma2     = homogeneousMatrixFromPos(posesigma1Msigma2);
    sigma2Msigma1     = inv(sigma1Msigma2);
    
    //deduce the pose of sigma in the world
    wMsigma1          = wMsigma2*sigma2Msigma1;
    wP                = changeFramePoints(sigma1P,wMsigma1);

    // Set to zero all the points that are beyond the ground.
//    wP(3,:)=wP(3,:).*(wP(3,:)>0);
    
    param = [H/Hmax,sin(-theta)];
    
endfunction


function [cP,pmetre,ppixel,nbpoints] = imageProjection(cMw,wP,u0,v0,px,py)
    
    cP                  = changeFramePoints(wP,cMw);
    cPX                 = cP(1,:); // points 3D en m dans le repere camera
    cPY                 = cP(2,:); // points 3D en m dans le repere camera
    cPZ                 = cP(3,:); // points 3D en m dans le repere camera

//cut frame
    cpx                 = []; // points 2D en m dans le plan image
    cpy                 = []; // points 2D en m dans le plan image
    cu                  = []; // points 2D en pixels dans le plan image
    cv                  = []; // points 2D en pixels dans le plan image
        
    nbpoints = 0;    
    for i=1:(size(cP,2))
        x = cP(1,i)/cP(3,i);
        y = cP(2,i)/cP(3,i);
        
        [u,v]= convertMeter2Pixel(x, y,u0,v0, px,py)
        if(u>0 & v>0 & u<im_width & v<im_height)
            cpx                 = [cpx;x];
            cpy                 = [cpy;y];
            cu                  = [cu;u];
            cv                  = [cv;v];
            nbpoints             = nbpoints + 1; 
        end 
    end
    
   pmetre  = [cpx,cpy];
   ppixel  = [cu ,cv];
    

    
endfunction
    
