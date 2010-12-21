//---------------------------------------//
// compute the interaction matrix
// associated to a point
// and for a 6ddl control
// Typically, a free 6ddl camera
// author : Claire Dune
// date : decembre 2009
//---------------------------------------//
function L = matIntPoint6ddl(x,y,Z)
// compute the interaction matrix for a 6ddl camera
  L = [-1/Z , 0  ,  x/Z, x*y ,-(1+x^2),  y ; 0 , -1/Z , y/Z ,1+y^2 , - x*y  , -x ] ;
endfunction

//---------------------------------------//
// compute the interaction matrix
// associated to a point
// and for a 3ddl control
// Typicaly the COM of the HRP2
//
// author : Claire Dune
// date : decembre 2009
//---------------------------------------//
function L = matIntPoint3ddl(x,y,Z)
// 3ddl vx, vz and thetay  
  L = [ -1/Z ,   x/Z , -(1+x^2); 0 ,y/Z , -x*y ];
endfunction

//---------------------------------------//
// compute the interaction matrix
// associated to 5 points
// and for a 6ddl control
// Typically, a free 6ddl camera
//
// only the projection of the points in the
// image plane are known
//
// author : Claire Dune
// date : decembre 2009
//---------------------------------------//
function L = matIntMire6ddl(p,Zint)
  Z = 0;
  // point mire, 2 feature per point
  N = length(p)/2;
  //----- test on Zint
  if (length(Zint)==N)
    Z = Zint;
    //disp('Z was a vector, no change')
  else
    Z = Zint*ones(N,1);
    //disp('Z was a scalar, create a vector, change')
  end



  L=[];
  for i=1:N
      L= [L; matIntPoint6ddl(p((i-1)*2+1),p((i-1)*2+2),Z(i))];
  end
endfunction





//---------------------------------------//
// compute the interaction matrix
// associated to 5 points
// and for a 3ddl control
// Typicaly the COM of the HRP2
//
// only the projection of the points in the
// image plane are known
//
// author : Claire Dune
// date : decembre 2009
//---------------------------------------//
function L = matIntMire3ddl(p,Zint)
  Z = 0;
  // point mire, 2 feature per point
  N = length(p)/2;
  
  //----- test on Zint
  if (length(Zint)==N)
    Z = Zint;
    //disp('Z was a vector, no change')
  else
    Z = Zint*ones(N,1);
    //disp('Z was a scalar, create a vector, change')
  end



  L=[];
  for i=1:N
      L= [L; matIntPoint3ddl(p((i-1)*2+1),p((i-1)*2+2),Z(i))];
  end
endfunction


//---------------------------------------//
// compute the interaction matrix
// associated to 5 points
// and for a 6ddl control
// Typically, a free 6ddl camera
//
// In this case, the 3D positions of the
// points have been estimated
//
// author : Claire Dune
// date : decembre 2009
//---------------------------------------//
function L = matInt3dMire6ddl(cP)
  N = length(cP)/3;
  L = [] ;
  for i=1:N
    L= [ L ; matIntPoint6ddl(cP(1,i)/cP(3,i),cP(2,i)/cP(3,i),cP(3,i))];
  end
endfunction

//---------------------------------------//
// compute the interaction matrix
// associated to 5 points
// and for a 3ddl control
// Typicaly the COM of the HRP2
//
// In this case, the 3D positions of the
// points have been estimated
//
// author : Claire Dune
// date : decembre 2009
//---------------------------------------//
function L = matInt3dMire3ddl(cP)
  N = length(cP)/3;
  L = [] ;
  for i=1:N
    L= [ L ; matIntPoint3ddl(cP(1,i)/cP(3,i),cP(2,i)/cP(3,i),cP(3,i))];
  end
endfunction

//-------------------------------//
// compute the interaction matrix
// associated to 5 points
// and for a 3ddl control
// Typicaly the COM of the HRP2
//
// only the projection of the points in the
// image plane are known
//
// author : Claire Dune
// date : decembre 2009
//---------------------------------//
function L_out = matIntMireD(s_in,Z_in)
  global Zdes_global;
  global sdes_global;
  L_out = matIntMire6ddl(sdes_global,Zdes_global); 
endfunction

//-------------------------------//
// compute the interaction matrix
// associated to 5 points
// and for a 3ddl control
// Typicaly the COM of the HRP2
//
// only the projection of the points in the
// image plane are known
//
// author : Claire Dune
// date : decembre 2009
//---------------------------------//
function L_out = matIntMireP(s_in,Z_in)
  global Zdes_global;
  L_out = matIntMire6ddl(s_in,Zdes_global); 
endfunction

//-------------------------------//
// compute the interaction matrix
// associated to 5 points
// and for a 3ddl control
// Typicaly the COM of the HRP2
//
// only the projection of the points in the
// image plane are known
//
// author : Claire Dune
// date : decembre 2009
//---------------------------------//
function L_out = matIntMireM(s_in,Z_in)
   global Zdes_global;
   global sdes_global;
   L_des     = matIntMire6ddl(sdes_global,Zdes_global); 
   L_current = matIntMire6ddl(s_in,Z_in);
   L_out     = 0.5*(L_des+L_current); 
endfunction

//-------------------------------//
// compute the interaction matrix
// associated to 5 points
// and for a 3ddl control
// Typicaly the COM of the HRP2
//
// only the projection of the points in the
// image plane are known
//
// author : Claire Dune
// date : decembre 2009
//---------------------------------//
function L_out = matIntMireC(s_in,Z_in)
    L_out = matIntMire6ddl(s_in,Z_in); 
endfunction


//----------------------------------//
// First try of visual servoing for walking
// 10/02/10
// The cost funciton has to be linear 
// the interaction matrix used is the desired matrix
// only for x and z
//------------------------------------//

function L_out = matIntDLinearWalk(s_des,Z_des,Np, Nbpts)
// Np is the lenght of the horizon
// Nbpts is the number of point of the target
// sdes id a vector of 2*Np
// Zdes is a vector of Np


  // compute the whole interaction matrix
  L  = matIntMire6ddl(s_des,Z_des);

  
  // take the 2 usefull columns X and Z
  Lx = L(:,1);
  Ly = L(:,3);
  
  // stack the matrices
  L_out_x = [];
  L_out_y = [];
  
  
  Zer = zeros(Nbpts*2,1);
  for i = 1:Np;
      L_ligne = [];
    for j=1:Np
      if i==j
         L_ligne = [L_ligne Lx];
      else
         L_ligne = [L_ligne Zer];
      end
    end 
    L_out_x = [L_out_x;L_ligne]
  end
  for i = 1:Np;
      L_ligne = [];
    for j=1:Np
      if i==j
         L_ligne = [L_ligne Ly];
      else
         L_ligne = [L_ligne Zer];
      end
    end 
    L_out_y = [L_out_y;L_ligne]
  end

  L_out = [L_out_x L_out_y];
   
endfunction

//------------------------------//
// R Matrix
//------------------------------//
function [Rres,Rx] = matR(Np)

   R   = zeros(Np,Np);
   Zer = R;
   for i=1:Np
      for j=1:i
      R(i,j)=1;  
      end
   end
   Rres = [R Zer ; Zer R];
   Rx =R;
endfunction

//------------------------------//
// L Matrix
//------------------------------//
function [Lxx,Lyy,Lxy,Lyx] = matL(L_in,Np)
  
  // Lin'Lin
  LL = L_in'*L_in;
  
  Lxx = LL(1:Np,1:Np);
  Lyy = LL(Np+1:2*Np,Np+1:2*Np);
  Lxy = LL(1:Np,Np+1:2*Np);
  Lyx = LL(Np+1:2*Np,1:Np);
   
endfunction
  
  
//----------------------------------//
// M Matrix
//-----------------------------------//

function [Mxx,Myy,Mxy,Myx,Rres,Ldes] = matLinearAsserVisu(sdes,Zdes,Np,Nbpts)
   
  [Rres,Rx] = matR(Np);
  
  Ldes = matIntDLinearWalk(sdes,Zdes,Np, Nbpts);
  [Lxx,Lyy,Lxy,Lyx] = matL(Ldes,Np);
  
  Mxx = Rx' * Lxx * Rx ;
  Myy = Rx' * Lyy * Rx ;
  Mxy = Rx' * Lxy * Rx ;
  Myx = Rx' * Lyx * Rx ;
  
  
endfunction

//--------------------------------//
// ThetaU interaction matrix
// cdRc: current rotation in the reference frame
//--------------------------------//
function L = matIntThetaU(thetaU)
         theta = norm(thetaU);
         L0    = zeros(3,3);
         if(theta~=0)
          u     = thetaU./theta;
         else 
          u      = thetaU;
         end 
         Ux    = skew(u);
         Lw    = eye(3,3)+ theta/2*Ux+ (1-sinc(theta)/(sinc(theta/2))^2)*Ux*Ux; 
         L     = [L0 Lw] 
endfunction  

function L = matIntposeThetaU(posetU)
         cdRc = rotationMatrixFromThetaU(posetU(4:6));
         Lt   = [cdRc zeros(3,3)];  
         Lr   = matIntThetaU(posetU(4:6));
         L    = [Lt;Lr];
    
endfunction  



//---------------------------------------//
function IsMatIntLoaded()
   disp('Matrice interaction is loaded')
endfunction



