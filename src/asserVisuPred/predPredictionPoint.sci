//-----------------------------------------------//
// INCREMENTAL MODEL FUNCTION ITRO 2010 ALLIBERT
// author Claire Dune
// date 04/01/2010
//-----------------------------------------------//
function sm = ga_predLocal2dPoint(smPrec,v,Te,Zint)
  // 
  // update the predicted model  
  // Zint is arbitrarily chosen
  // v is the camera velocity or the control at time t
  // Te is  the sampling time
  L = matIntPoint6ddl(smPrec(1),smPrec(2),Zint);
  sm = smPrec + L*Te*v;
  
endfunction


//-----------------------------------------------//
// INCREMENTAL MODEL FUNCTION ITRO 2010 ALLIBERT
// author Claire Dune
// date 04/01/2010
//-----------------------------------------------//
function sm = ga_predLocal2dPointL(L,smPrec,v,Te,Zint)
  // 
  // update the predicted model  
  // Zint is arbitrarily chosen
  // v is the camera velocity or the control at time t
  // Te is  the sampling time
  // L is the interaction matrix
  sm = smPrec + L*Te*v;
  
endfunction


//-----------------------------------------------//
// PREDICTION MODEL FUNCTION ITRO 2010 ALLIBERT
// author Claire Dune
// date 04/01/2010
//-----------------------------------------------//
function sm = ga_predHorLoc2dPoint(sm0,U,Te,Z)
  //% 
  //predict sm given the whole control U on the horizon
  //
  // U is the control   
  // sm0 is the fist model
  // L is the interaction matrix
  // Te is the sampling time
  // Z is the depth it is fixed on the horizon

  smPrec= sm0; // init the model
  sm =[]; // create the output vector
  N = length(U)/6;  // N is the length of the Horizon

  //L = matIntPoint6ddl(sm0(1),sm0(2),Z);
  
  for i=1:N
   // compute the next local prediction for 2d features
    smNext = ga_predLocal2dPoint(smPrec,U((i-1)*6+1:(i-1)*6+6),Te,Z);
    //smNext = ga_predLocal2dPointL(L,smPrec,v,Te,Z);
    // store the prediction
    sm = [sm ; smNext];
    // update smPrecedent
    smPrec = smNext; 
    
  end
 
endfunction

//-----------------------------------------------//
// PREDICTION MODEL FUNCTION ITRO 2010 ALLIBERT
// author Claire Dune
// date 04/01/2010
//-----------------------------------------------//
function sm = ga_predVerifLoc2dPoint(sm0,U,Te,Z)
  //% 
  //predict sm given the whole control U on the horizon
  // ONLY if L is constant on the intervalle
  // U is the control   
  // sm0 is the fist model
  // L is the interaction matrix
  // Te is the sampling time
  // Z is the depth it is fixed on the horizon

  sm =[]; // create the output vector
  N = length(U)/6;  // N is the length of the Horizon
  L = matIntPoint6ddl(sm0(1),sm0(2),Z);
 
  for i= 1:N
   smi = sm0;
   for k= 1:i
    // compute the next local prediction for 2d features
    smi = smi + Te*L*U((k-1)*6+1:(k-1)*6+6);  
   end
    // store the prediction
    sm = [sm ; smi];
  end
 
endfunction


//-----------------------------------------------//
// COST HORIZON FUNCTION ITRO 2010 ALLIBERT 
// author Claire Dune
// date 04/01/2010
//-----------------------------------------------//
function cost = ga_costHorLoc2dPoint(U,Te,Z,sm0,Q,e0,sStar)
  //%
  // compute the cost function on a whole horizon 
  // @param U  the control on the horizon
  // @param Te  sampling time
  // @param Z  a fixed depth
  // @param sm0  the first feature
  // @param Q  a weight symmetrix positive defined matrix 
  // @param e0  the first error
  // @param sStar  the desired features

  // compute the matrix sm of the predicted features
  sm = ga_predHorLoc2dPoint(sm0,U,Te,Z);
  N=length(U)/6 ;
  // compute the cost function unsing sm Q and sd
  cost =ga_costHorizon(N,sm,Q,e0,sStar);
endfunction



