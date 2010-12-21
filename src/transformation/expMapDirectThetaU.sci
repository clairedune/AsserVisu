function M = expMapDirectThetaU(v,dt)
// compute the transformation from the pose t to the pose t+dt
// input : the velocity (m/s,rad/s) and the time to apply it (s) 
// the velocity vector is a ThetaU 

// resulting motion in translation and theta U 
//disp("1.Create the vector v_dt");
v_dt = v * dt;
u = v_dt(4:6);
// compute the rotation matrix 
//disp("2.Compute the rotationmatrix");
rd = rotationMatrixFromThetaU(u),

//disp("3.Compute theta");
// compute the translation
theta = sqrt(u(1)*u(1) + u(2)*u(2) + u(3)*u(3));
si = sin(theta);
co = cos(theta);

//disp("4.Compute scinc");
sinca = sinc(theta); 
//mcosc
if(theta<%eps)
  mcosc=0.5;
else
  mcosc = (1-co)/(theta*theta);
end;
if(theta<%eps)
  msinc =1/6;
else
  msinc = (1-si)/(theta*theta);
end;
 
td(1) = v_dt(1)*(sinca + u(1)*u(1)*msinc)+ v_dt(2)*(u(1)*u(2)*msinc - u(3)*mcosc)+ v_dt(3)*(u(1)*u(3)*msinc + u(2)*mcosc);
td(2) = v_dt(1)*(u(1)*u(2)*msinc + u(3)*mcosc)+ v_dt(2)*(sinca + u(2)*u(2)*msinc)+ v_dt(3)*(u(2)*u(3)*msinc - u(1)*mcosc);
td(3) = v_dt(1)*(u(1)*u(3)*msinc - u(2)*mcosc)+ v_dt(2)*(u(2)*u(3)*msinc + u(1)*mcosc)+ v_dt(3)*(sinca + u(3)*u(3)*msinc);

troiszeros = zeros(1,3);
M = [rd td;troiszeros 1]
 
endfunction



//------------------------------------------------------------//
//
//
//
//------------------------------------------------------------//
function vitesse = expMapInverseThetaU(M,dt)
  disp('debut function')
   
   R = M(1:3,1:3);
   vectThetaU = thetaUFromRotationMatrix(R);
   disp(size(vectThetaU))

   vitesse =[0 0 0 vectThetaU'];
   u = vectThetaU;
 
   theta   = sqrt(u'*u);
   si      = sin(theta);
   co      = cos(theta);
   sincard = sinc(theta);
   mcosc   = (1-co)/(theta*theta);
   msinc   = (1-si)/(theta*theta);
 
   A       = zeros(3,3);
   A(1,1)  = sincard + u(1)*u(1)*msinc;
   A(1,2)  = u(1)*u(2)*msinc - u(3)*mcosc;
   A(1,3)  = u(1)*u(3)*msinc + u(2)*mcosc;

   A(2,1)  = u(1)*u(2)*msinc + u(3)*mcosc;
   A(2,2)  = sincard + u(2)*u(2)*msinc;
   A(2,3)  = u(2)*u(3)*msinc - u(1)*mcosc;

   A(3,1)  = u(1)*u(3)*msinc - u(2)*mcosc;
   A(3,2)  = u(2)*u(3)*msinc + u(1)*mcosc;
   A(3,3)  = sincard + u(3)*u(3)*msinc;
 
   detA = det(A);
   
   if (abs(detA) > 1.e-5)
   
     vitesse(1) =  (M(1,4)*A(2,2)*A(3,3)...
            +   M(2,4)*A(3,2)*A(1,3)...
            +   M(3,4)*A(1,2)*A(2,3)...
            -   M(3,4)*A(2,2)*A(1,3)...
            -   M(2,4)*A(1,2)*A(3,3)...
            -   M(1,4)*A(3,2)*A(2,3))/detA;
     vitesse(2) =  (A(1,1)*M(2,4)*A(3,3)...
            +   A(2,1)*M(3,4)*A(1,3)...
            +   M(1,4)*A(2,3)*A(3,1)...
            -   A(3,1)*M(2,4)*A(1,3)...
            -   A(2,1)*M(1,4)*A(3,3)...
            -   A(1,1)*M(3,4)*A(2,3))/detA;
      vitesse(3) =  (A(1,1)*A(2,2)*M(3,4)...
            +   A(2,1)*A(3,2)*M(1,4)...
            +   A(1,2)*M(2,4)*A(3,1)...
            -   A(3,1)*A(2,2)*M(1,4)...
            -   A(2,1)*A(1,2)*M(3,4)...
            -   A(1,1)*A(3,2)*M(2,4))/detA;
   
   else
   
      vitesse(1) = M(1,4);
      vitesse(2) = M(2,4);
      vitesse(3) = M(3,4);
   end
 
   // Apply the sampling time to the computed velocity
   vitesse = vitesse ./dt;

  
endfunction
