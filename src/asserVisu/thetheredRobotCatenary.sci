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
    //pause
    // semi distance between the two fixation points
    // se calcul par rapport a H et R
    if D>=R then
        disp("Error the distance between the two robots cannot be greater than the rope length");
        param = [%eps,sin(%eps)];  
        wMsigma1          = wMsigma2+wMsigma3/2;
        wP    = [];
    else
        // rope sag estimation
        H = findCatenaryH(R,D); 
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
    
        param = [H/Hmax,sin(-theta)];    
    end
    
    
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
            nbpoints            = nbpoints + 1; 
        end 
    end
    
   pmetre  = [cpx,cpy];
   ppixel  = [cu ,cv];
    
endfunction


function [w_M_r1,w_M_r2,w_P_positif,param] = vsCatenaryHth(w_M_r1,v_r1,w_M_r2,r1_M_sigma3,r2_M_sigma2,r2_M_c,R,Hmax, threshold,dt)

e = 1;
   
while (norm(e)>threshold) 
    // Catenary parametres
    [param,D,w_P,xA,yA,zA,w_M_sigma1] = thetheredRobotCatenary(w_M_r1,w_M_r2,r1_M_sigma3,r2_M_sigma2,R,Hmax);
    // Set to zero all the points that are beyond the ground.
    
    // if the rope length is greater that 0
    if length(w_P>0) then
        w_P_positif      = w_P;
        w_P_positif(3,:) = w_P(3,:).*(w_P(3,:)>0);
    
        // ---- INTERACTION MATRIX FOR 3D  CATENARY ------ //
        e     = param' - paramd';
        L     = catenary3DIntMat( R, Hmax, param, xA, yA, zA )  ;  
        
        
        //change of frame between sigma2 and r2 
        r2_V_sigma2 = twistMatrix(r2_M_sigma2);
        v_sigma2    = - lambda*pinv(L)*e;
        v_r2        = r2_V_sigma2*v_sigma2;
      
        disp(v_r2)
        v_r2(2)=0;
        v_r2(3)=0;
        v_r2(4)=0;
        v_r2(5)=0;
        
        
        
        // ---------------------UPDATE --------------------------------
        r1_curr_M_r1_next     = expMapDirectRxRyRz(v_r1',dt);
        r2_curr_M_r2_next     = expMapDirectRxRyRz(v_r2',dt);
  
        w_M_r1                = w_M_r1*r1_curr_M_r1_next;
        w_M_r2                = w_M_r2*r2_curr_M_r2_next;
        r1_M_w                = inv(w_M_r1);
        r2_M_w                = inv(w_M_r2);  
  else
      w_M_r2 = [];
      e=0;
      break;  
  end
  end
endfunction



    
