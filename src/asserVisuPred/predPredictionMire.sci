//-----------------------------------------------//
// INCREMENTAL MODEL FUNCTION ITRO 2010 ALLIBERT
// author Claire Dune
// date 04/01/2010
//-----------------------------------------------//
function [sm_out,Z_out] = ga_predGlobalMire(s_in,Z_in,v_in,Te_in)
  //% 
  // update the predicted model  
  // 
  // @param Zint is an arbitrarily chosen depth
  // @param L is the interaction matrix
  // @param v is the camera velocity or the control at time t
  // @param Te is  the sampling time
  

  Ndofin = length(s_in)/2; //number of tracked points
  if(length(Z_in)~=Ndofin)    //build  Z vector if necessary
     Z_current = Z_in(1)*ones(Ndofin,1);
  else
     Z_current = Z_in;
  end

  c1P =[];     //to store the coeff
  for (i=1:Ndofin)
     c1P       =[c1P
                 s_in((i-1)*2+1)*Z_current(i)
                 s_in((i-1)*2+2)*Z_current(i)
                 Z_current(i)]
  end
  c1Mc2      = computeMotion(v_in',Te_in)  ;         // resulting motion
  c2P        = changeFrameMire(c1P,inv(c1Mc2)); 
  
  
  Z_out      = c2P(3:3:length(c2P)) ;            // init depth for the step 
  sm_out     = projectMireDirect(c2P);
endfunction

//-----------------------------------------------//
// INCREMENTAL MODEL FUNCTION ITRO 2010 ALLIBERT
// author Claire Dune
// date 04/01/2010
//-----------------------------------------------//
function c1McN = ga_predHorPosition(U_in,Te_in,Np_in)
  // 
  // given a velocity vector and a Time, predict the camera motion
  //
  c1McK = eye(4,4);
  c1McN = [];
  
  for i=1:Np_in,
      cprecMcnext = computeMotion(U_in((i-1)*6+1:(i-1)*6+6)',Te_in);
      c1McK = c1McK * (cprecMcnext);
      c1McN =[c1McN;c1McK];
  end
  
endfunction

//-----------------------------------------------//
// INCREMENTAL LOCAL MODEL FUNCTION ITRO 2010 ALLIBERT
// author Claire Dune
// date 28/01/2010
//-----------------------------------------------//
function sm_out = ga_predLoc2dMire(s_in,Z_in,v_in,Te_in)
  //% 
  // update the predicted model  
  // @compute L is the name of the function to computeL
  // @param Zint is an arbitrarily chosen depth
  // @param L is the interaction matrix
  // @param v is the camera velocity or the control at time t
  // @param Te is  the sampling time
  global computeL_global;
  L_out  = computeL_global(s_in,Z_in);
  sm_out = s_in+ Te_in*L_out*v_in;
endfunction

//-----------------------------------------------//
// PREDICTION MODEL FUNCTION ITRO 2010 ALLIBERT
// author Claire Dune
// date 04/01/2010
// modified 01/03/2010
//-----------------------------------------------//
function sm_out = ga_predHorLoc2dMire(s_in,Z_in,U_in,Te_in,Np_in)
  //% 
  //predict sm given the whole control U on the horizon
  //
  // @param Np prediction horizon size
  // @param sm0 is the fist model [smx0 smy0 smx1 ... smyNbpts]'
  // @param U is the control [U11..U61..U1Np..U6Np]'
  // @param L is the interaction matrix
  // @param Te is the sampling time
  // @param Z is the depth it is fixed on the horizon

  smPrec    = s_in;               // init the model
  sm_out    = [];                 // create the output vector
  L_out     = [];
  ndof      = length(U_in)/Np_in; // number degree of freedom   
  // U is a column vector that has ndof componants
  for i=1:Np_in,
    // compute the next prediction
    smNext = ga_predLoc2dMire(smPrec,Z_in,U_in((i-1)*ndof+1:(i-1)*ndof+ndof),Te_in);
    // store the prediction
    sm_out = [sm_out ; smNext];
    
    // update smPrecedent
    smPrec = smNext; 
  end
 
endfunction


//-----------------------------------------------//
// PREDICTION MODEL FUNCTION GLOBAL
// 
// date 30/01/2010
// modified 01/03/2010
//-----------------------------------------------//
function sm_out = ga_predHorGlobalMire(s_in,Z_in,U_in,Te_in,Np_in)
  //% 
  //predict sm given the whole control U on the horizon
  //
  // @param Np prediction horizon size
  // @param s_in is the fist model [smx0 smy0 smx1 ... smyNbpts]'
  // @param U is the control [U11..U61..U1Np..U6Np]'
  // @param Te is the sampling time
  // @param Z is the init depth 

  smCurrent = s_in;               // init the model
  sm_out    = [];                 // create the output vector
  ZCurrent  = Z_in;               // init the current depth
  ndof      = length(U_in)/Np_in; // number degree of freedom   
 
  
  // U is a column vector that has ndof componants
  for i=1:Np_in,
    // compute the next prediction
    [smNext,ZNext] = ga_predGlobalMire(smCurrent,ZCurrent,...
                     U_in((i-1)*ndof+1:(i-1)*ndof+ndof),Te_in);
    // store the prediction
    sm_out         = [sm_out ; smNext];
    // update smPrecedent
    smCurrent      = smNext; 
    ZCurrent       = ZNext;
  end
 
endfunction


//-------------------------------------------------------//
// Prediction model from jerk
// date 1 mars 2010
// 
// s_in is a vector of the initial feature position
// Z_in is a vector of the initial depth
// stateCoM is the state of the CoM
// rVc is the velocity frame change 
// Sv and Uv are defined in predMatrix
//-------------------------------------------------------//

function sm_out= predHorLocalMireJerk(s_in,Z_in,stateCoMIn,rMcIn,jerkIn,SpIn,UpIn,SvIn,UvIn,SaIn,UaIn,TeIn,NpIn,ndofIn)
   
  
   //build  Z vector if necessary
   Nbpts        = length(s_in)/2;
   if( length(Z_in)~=Nbpts )
     Z_current  = Z_in(1)*ones(Nbpts ,1);
   else
     Z_current  = Z_in;
   end 
   
   // position vitesse acceleration
   [PIn , Pstack , VIn ,Vstack, A , Astack] = ... 
          stateFromJerkHorizon(stateCoMIn, jerkIn, SpIn, UpIn, SvIn, UvIn, SaIn, UaIn); 
   [Pcam,Vcam]  = stateCamFromComHor(rMcIn,PIn,VIn,ndofIn,NpIn); // horizon
  // [pcam0,vcam0]= stateCamFromCom(rMcIn,stateCoMIn,ndofIn); // init 
    
   sm_out = ga_predHorLoc2dMire(s_in,Z_in,Vcam,TeIn,NpIn);

endfunction


//-------------------------------------------------------//
// Prediction des matrices d'interaction pour le QP
//--------------------------------------------------------//

function L_out   = predIntMatGlobalJerk(L_fun,sm_out,Z_out,NpIn,Nbpts)
  L_out =[];
  for i=1:NpIn
       scurrent  = sm_out((i-1)*(2*Nbpts)+1:(i-1)*(2*Nbpts)+(2*Nbpts)) ;
       Zcurrent  = Z_out((i-1)*(Nbpts)+1:(i-1)*(Nbpts)+(Nbpts)) ;
       L_out     = [L_out;L_fun(scurrent,Zcurrent)];
  end
endfunction


//-------------------------------------------------------//
// Grande diagolnale de L
// Np is the lenght of the horizon
// Nbpts is the number of point of the target
//--------------------------------------------------------//
function L_out2   = bigPredBigMatrixGlobal(L_in,Np, Nbpts)

  Zer = zeros(Nbpts*2,6);
 L_out2=[];
  for i = 1:Np;
      L_ligne = [];
    for j=1:Np
      if i==j
         L_ligne = [L_ligne L_in((j-1)*(Nbpts*2)+1:(j-1)*(Nbpts*2)+Nbpts*2,:)];
      else
         L_ligne = [L_ligne Zer];
      end
    end 
    L_out2 = [L_out2;L_ligne];
  end

endfunction


//--------------------------------------------------------//
// Matrice de selection
//---------------------------------------------------------//

function E = selectJerk(ndof,Np)   
   A=[]; 
   for i=1:Np 
      ligne = zeros(1,Np);
      ligne(i)=1;
      lignesup=zeros(ndof-1,Np);
      A=[A;ligne;lignesup];
    end 

    B=[A]; 
    for i=1:ndof-1
        B=[B  [zeros(i,Np);A(1:$-i,:)]];
     end

E=B;
endfunction
//-------------------------------------------------------//
// Prediction model from jerk
// date 1 mars 2010
// 
// s_in is a vector of the initial feature position
// Z_in is a vector of the initial depth
// stateCoM is the state of the CoM
// rVc is the velocity frame change 
// Sv and Uv are defined in predMatrix
//-------------------------------------------------------//
function [sm_out,Z_out] = predHorGlobalMireJerk(s_in,Z_in,stateCoMIn,rMcIn,jerkIn,SpIn,UpIn,SvIn,UvIn,SaIn,UaIn,TeIn,NpIn,ndofIn)
   
   // output
   sm_out       = [];
   Z_out        = [];
 
  //build  Z vector if necessary
   Nbpts        = length(s_in)/2;
   if( length(Z_in)~=Nbpts )
     Z_current  = Z_in(1)*ones(Nbpts ,1);
   else
     Z_current  = Z_in;
   end 
   
   // position vitesse acceleration
   [PIn , Pstack , VIn ,Vstack, A , Astack] = ... 
          stateFromJerkHorizon(stateCoMIn, jerkIn, SpIn, UpIn, SvIn, UvIn, SaIn, UaIn); 
   [Pcam,Vcam]  = stateCamFromComHor(rMcIn,PIn,VIn,ndofIn,NpIn); // horizon
   [pcam0,vcam0]= stateCamFromCom(rMcIn,stateCoMIn,ndofIn); // init 
   //disp('pose')
   //disp(Pcam)   
  

   // initial pose
   pose0 = pcam0(1:6)';  
   wMc0 = homogeneousMatrixFromPos(pose0);
   
   // initial points 
   c0P =[];     
   for i=1:Nbpts 
     c0P       = [c0P 
                  s_in((i-1)*2+1)*Z_current(i)
                  s_in((i-1)*2+2)*Z_current(i)
                  Z_current(i)];
   end

   // then on all the horizon
   wMcPrec = wMc0;   
   cPrecP  = c0P;
   for i=1:NpIn 
 
        poseNext =Pcam((i-1)*6+1:(i-1)*6+6)'; 
        wMcNext = homogeneousMatrixFromPos(poseNext);
        // motion of the camera
        cPrecMcNext  = inv(wMcPrec)*wMcNext  ;
        
        // new feature  
        cNextP       = changeFrameMire(cPrecP,inv(cPrecMcNext)); 
        Z_out        = [Z_out; cNextP(3:3:$)] ;            
        sm_out       = [sm_out;projectMireDirect(cNextP)];
        
        // update
        wMcPrec      = wMcNext;   
        cPrecP       = cNextP;
  end

  

endfunction




 
//------------------------------------------------------//
// Predict the velocity 
// from the current system state 
// position x,y,z,rx,ry,rz 
// velocity dx,dy,dz,drx,dry,drz 
// acceleration ddx,ddy,ddz,ddrx,ddry,ddrz
//------------------------------------------------------//

function [v, vstack] = velocityFromJerkHorizon(state,SvIn,UvIn,jerkIn)

   // state = [x,dx,ddx], whith length(x)=length(dx)=length(ddx)=ndof
   ndof     = length(state)/3;
   vstack   = [];
   vtmp     = [];
   N_in     = length(jerkIn)/ndof;
   v        = [];
   for i=1:ndof
  
        stateVar = [state(i),state(ndof+i),state(2*ndof+i)]';
        jerkVar  = jerkIn((i-1)*N_in+1:(i-1)*N_in+N_in);
        dxHor    = horizonFromJerk(stateVar,jerkVar,SvIn,UvIn);
        vtmp     = [vtmp dxHor];
        vstack   = [vstack;dxHor];                

   end
        
   for i=1:size(vtmp,1)
      
        v       = [v; vtmp(i,:)'];
   
   end 
   
 
endfunction


//------------------------------------------------------//
// state is a 3d vertical vector x,dx,ddx
// SpIn, UpIn are defined in predMatrix.sci
// jerkIn is the horizon of jerk 
//-------------------------------------------------------//
function hor = horizonFromJerk(state,jerkIn, S,U)
      
    hor = S*state+U*jerkIn;

endfunction



//------------------------------------------------------//
// compute the state from the jerk
// 
//------------------------------------------------------//
function [P,Pstack,V,Vstack,A,Astack] = stateFromJerkHorizon(state,jerkIn,SpIn,UpIn,SvIn,UvIn,SaIn,UaIn)

   // state = [x,dx,ddx], whith length(x)=length(dx)=length(ddx)=ndof
   ndof     = length(state)/3;
   N_in     = length(jerkIn)/ndof;
   Pstack   = [];
   P        = [];
   ptmp     = [];
   Vstack   = [];
   V        = [];
   vtmp     = [];
   Astack   = [];
   A        = [];
   atmp     = [];

  
   for i=1:ndof
      
        stateVar = [state(i),state(ndof+i),state(2*ndof+i)]';
        jerkVar  = jerkIn((i-1)*N_in+1:(i-1)*N_in+N_in);
        xHor     = horizonFromJerk(stateVar,jerkVar,SpIn,UpIn);
        dxHor    = horizonFromJerk(stateVar,jerkVar,SvIn,UvIn);
        ddxHor   = horizonFromJerk(stateVar,jerkVar,SaIn,UaIn);        
        ptmp     = [ptmp xHor];
        Pstack   = [Pstack;xHor];     
        vtmp     = [vtmp dxHor];
        Vstack   = [Vstack;dxHor]; 
        atmp     = [atmp ddxHor];
        Astack   = [Astack;ddxHor];                 
   
   end
        
   for i=1:size(vtmp,1)
        P       = [P; ptmp(i,:)'];       
        V       = [V; vtmp(i,:)'];
        A       = [A; atmp(i,:)'];
   end 
   
endfunction

//-----------------------------------------------------//
// 
// Position and velocity of the camera from jerk horizon
// the state of the robot is expressed in the world frame
// ndof is the number of degrees of freedom
//     if 2 then X Y 
//     if 3 then X Y thetaZ
//     if 6 then X Y Z thetaX thetaY thetaZ
// NpIn is the length of the Horizon
//     
//-----------------------------------------------------//
function [p, v] = stateCamFromComHor(rMcIn,PIn,VIn,ndofIn,NpIn)
   p=[];
   v=[];
   for i=1:NpIn
      
      if (ndofIn==2)//only X and Y 
           poseIn = [PIn((i-1)*ndofIn+1);PIn((i-1)*ndofIn+2);0;-%pi;0;0]; 
           vIn    = [VIn((i-1)*ndofIn+1);VIn((i-1)*ndofIn+2);0;0;0;0]; 
      elseif (ndofIn==3) // X Y ThetaZ
           poseIn = [PIn((i-1)*ndofIn+1);PIn((i-1)*ndofIn+2);0;-%pi;0;PIn((i-1)*ndofIn+3)];
           vIn    = [VIn((i-1)*ndofIn+1);VIn((i-1)*ndofIn+2);0;0;0;VIn((i-1)*ndofIn+3)];
      elseif (ndofIn==6)// X Y Z ThetaX ThetaY ThetaZ 
           poseIn = PIn((i-1)*ndofIn+1:(i-1)*ndofIn+ndofIn);
           vIn    = VIn((i-1)*ndofIn+1:(i-1)*ndofIn+ndofIn);
      end
     
        
      wMrIn    = homogeneousMatrixFromPos(poseIn');
      wMcIn    = wMrIn*rMcIn;
      wVcIn    = twistMatrix(wMcIn);
      p        = [p;pFromHomogeneousMatrix(wMcIn)];
      v        = [v;inv(wVcIn)*vIn];
   end
endfunction



//-----------------------------------------------------//
// 
// Position and velocity of the camera from jerk horizon
// the state of the robot is expressed in the world frame
// ndof is the number of degrees of freedom
//     if 2 then X Y 
//     if 3 then X Y thetaZ
//     if 6 then X Y Z thetaX thetaY thetaZ
// NpIn is the length of the Horizon
//     
//-----------------------------------------------------//
function [p, v] = stateCamFromCom(rMcIn,stateComIn,ndofIn)
   
   
   if (ndofIn==2)//only X and Y 
           poseIn = [stateComIn(1);stateComIn(2);0;-%pi;0;0]; 
           vIn    = [stateComIn(3);stateComIn(4);0;0;0;0]; 
   elseif (ndofIn==3) // X Y ThetaZ
           poseIn = [stateComIn(1);stateComIn(2);0;-%pi;0;stateComIn(3)];
           vIn    = [stateComIn(4);stateComIn(5);0;0;0;stateComIn(6)];
   elseif (ndofIn==6)// X Y Z ThetaX ThetaY ThetaZ 
           poseIn = stateComIn(1:ndofIn);
           vIn    = stateComIn(ndofIn+1:2*ndofIn);
   end

  
   wMrIn    = homogeneousMatrixFromPos(poseIn');
   wMcIn    = wMrIn*rMcIn;
   wVcIn    = twistMatrix(wMcIn);
   p        = pFromHomogeneousMatrix(wMcIn)';
   v        = inv(wVcIn)*vIn;


endfunction

//-----------------------------------------------------//
//             IsPredCtrlMireLoaded
//-----------------------------------------------------//

function IsPredCtrlMireLoaded()
 disp('Predictive control is Loaded')  
endfunction 
