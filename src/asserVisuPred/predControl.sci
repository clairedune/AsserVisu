//------------------------------------------------//
//
// Norm Up
//
//------------------------------------------------//

function Un = normalizeU(Up_in,val_in)
   Un    = Up_in;
   supUp = max(abs(Un));
   
   if(supUp > val_in)
      Un = val_in / supUp * Un
   end
endfunction

//-----------------------------------------------//
// BUILD THE CONTROL on the PREDICTIVE HORIZON
// author claire Dune
// january 2010
//------------------------------------------------//
function Up_out = computeControlOnHorizon(Uc_in,Nc_in,Np_in)
// 
// this function compute the control on
// the whole prediction horizon
// if Nc <Np
// then for j > Nc, U(j)=U(Nc)
//
  Up_out = Uc_in;
    for k=1:Np_in - Nc_in
      Up_out = [ Up_out ; Uc_in( (Nc_in*6-5): Nc_in*6 )];
    end    
endfunction

//-------------------------------//
// compute the Q weighted matrix
// author : Claire Dune
// date : janvier 2010
//---------------------------------//

function Q = matWeightIdentity(Np_in,Nbpts_in)
    Q=[];
    for i = 1:Np_in
       Q = [Q;eye(Nbpts_in*2,Nbpts_in*2)];   
    end
endfunction

function Q = matWeightIdentityZero(Np_in,Nbpts_in,stop)
    Q=[];
    for i = 1:stop
       Q = [Q;eye(Nbpts_in*2,Nbpts_in*2)];   
    end
    for i = stop:Np_in
       Q = [Q;zeros(Nbpts_in*2,Nbpts_in*2)];   
    end
endfunction

function Q = matWeightTV(Np_in,Nbpts_in)
    Q = eye(Nbpts_in*2,Nbpts_in*2);
    Qdouble=2*Q;
    for i = 1:Np_in-1
       Q = [Q;Qdouble];
       Qdouble = 2*Qdouble; 
    end
endfunction

//-----------------------------------------------//
// COST FUNCTION ITRO 2010 ALLIBERT
// author Claire Dune
// date 04/01/2010
//-----------------------------------------------//
function cost = ga_cost(Np,sm,Q,sd)
  //%
  // compute the cost Function on the time horizon
  // @return cost  costAllibert(sm,Q,sd)
  // @param sd  matrix that stores the desired features over the horizon
  // @param sm matrix that stores the predicted features over the horizon
  // @param cost  sum((sd-sm)'*Q*(sd-sm))                   
 
  
  N = length(sd)/Np;
  

  
  if(length(sm)~=length(sd) |length(sm)~=Np*N   )
    disp('        WARNING')
    disp('>>-------------------<<')
    disp('Warning:ga_cost << length(sd)~=length(sm)|~=Np*N');  
    disp('length(sm)')
    disp(length(sm))
    disp('length(sd)')
    disp(length(sd))
    disp('Np*N')
    disp(Np*N)
    disp('>>-------------------<<')
    cost = %nan;
    resume;
  end
  
  if(size(Q,2)~=N)
    disp('        WARNING')
    disp('>>-------------------<<')
    disp('Warning:ga_cost << size Q ~= size error');  
    disp('size Q is')
    disp(size(Q))
    disp('while size sd is')
    disp(size(sd,1))
    disp('>>-------------------<<')
    cost = %nan;
    resume;
  end
  
  cost = 0;

  // compute the quadratic sum of the error
  for i=1:Np
    erreur = (sd((i-1)*N+1:(i-1)*N+N)-sm((i-1)*N+1:(i-1)*N+N))
    cost = cost + erreur'*Q((i-1)*N+1:(i-1)*N+N,:)*erreur;
  end

endfunction


//-----------------------------------------------//
// COST FUNCTION ITRO 2010 ALLIBERT
// author Claire Dune
// date 04/01/2010
//-----------------------------------------------//
function cost = ga_costHorizon(Np,sm,Q,e0,sStar)
  //%
  // compute the cost function on a whole horizon 
  // @param U  the control on the horizon
  // @param Te  sampling time
  // @param Z  a fixed depth
  // @param sm0  the first feature
  // @param Q  a weight symmetrix positive defined matrix 
  // @param e0  the first error
  // @param sStar  the desired features
  length(sm)
  // test vectorsize
  if(length(e0)~=length(sStar))
    disp('Warning:ga_costHorizon << length(e0)~=length(sStar)');  
    disp(length(e0))
    disp(length(sStar))
    cost = %nan;
    resume;
  end
 
     
  // compute the matrix of desired features
  // it is equal to the desired features - the error
  // between the model sm0 and the measure s0
  sd =[];
  for i=1:Np 
   sd = [sd ;sStar-e0];
  end
 
   
  // compute the cost function unsing sm Q and sd
  cost = ga_cost(Np,sm,Q,sd);
endfunction



function [x,sm,res]=predControl(Uc_in,predmodel,funcost,funcstr,gradobj,gradcstr)
// This function output the U and the sm computed
// thanks to predictive control at the time k
//
// @param Nbts   : number of points of the target (5)
// @param xl     : position min of the a 2D point in the image plane
// @param xu     : position max of the a 2D point in the image plane 
// @param U0     : initial state guess
// @param sm0    : init features, first projection of the target
// @param sStar  : desired features
// @param e0     : error between the model and the real measurements
// @param Q      : the weight matrix 
// @param Te     : the sampling time
// @param  Z     : the init depth

// We need global variable to make the cost fonction only depedent on U
global Nc_global ;
global Np_global ;
global Te_global ;
global Z_global;     
global s_global;    
global e0_global;    
global sdes_global;  
global Zdes_global; 
global bu_global; 
global bl_global; 
global ipar_global;
global rpar_global;

global store;
store=[];

disp('--------------debut---------------')
//disp('Pause avant de calculer l optimisation')
x=fsqp(Uc_in,ipar_global,rpar_global,[bl_global,bu_global],funcost,funcstr,gradobj,gradcstr);
disp('--------------x---------------')
disp(x)
//deduce the evolution of sm
res = computeControlOnHorizon(x,Nc_global,Np_global);
sm  = predmodel(s_global,Z_global,res,Te_global,Np_global);
endfunction
  
