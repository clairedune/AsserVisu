//-----------------------------------------------//
// COST HORIZON FUNCTION ITRO 2010 ALLIBERT 
// author Claire Dune
// date 04/01/2010
//-----------------------------------------------//
function cost = ga_costHorLoc2dMire(s_in,Z_in,Up_in,Te_in,Np_in,Q_in,e0_in,sdes_in)   //% 
  // compute the cost function on a whole horizon 
  // @param Up  the control on the horizon
  // @param Te  sampling time
  // @param Z  a fixed depth
  // @param sm0  the first feature
  // @param Q  a weight symmetrix positive defined matrix 
  // @param e0  the first error
  // @param sStar  the desired features
  // compute the matrix sm of the predicted features
  global store;
  store =[store; Up_in(1:6)'];
  sm_out = ga_predHorLoc2dMire(s_in,Z_in,Up_in,Te_in,Np_in);
  // compute the cost function unsing sm Q and sd
  cost   = ga_costHorizon(Np_in,sm_out,Q_in,e0_in,sdes_in);
endfunction

function cost = ga_costHorGlobaldMire(s_in,Z_in,Up_in,Te_in,Np_in,Q_in,e0_in,sdes_in)   //% 
  // compute the cost function on a whole horizon 
  // @param Up  the control on the horizon
  // @param Te  sampling time
  // @param Z  a fixed depth
  // @param sm0  the first feature
  // @param Q  a weight symmetrix positive defined matrix 
  // @param e0  the first error
  // @param sStar  the desired features
  // compute the matrix sm of the predicted features
  sm_out = ga_predHorGlobalMire(s_in,Z_in,Up_in,Te_in,Np_in);
  // compute the cost function unsing sm Q and sd
  cost   = ga_costHorizon(Np_in,sm_out,Q_in,e0_in,sdes_in);
endfunction


function cost = ga_costHorGlobalJerk(s_in,Z_in,sdes_in,Q_in,e0_in,stateCoMIn,rMcIn,jerkIn,SpIn,UpIn,SvIn,UvIn,SaIn,UaIn,TeIn,NpIn,ndofIn)
   sm_out = predHorGlobalMireJerk(s_in, Z_in, stateCoMIn, rMcIn, jerkIn, SpIn, UpIn, SvIn, UvIn, SaIn, UaIn, TeIn, NpIn, ndofIn);

   cost   = ga_costHorizon(Np_in,sm_out,Q_in,e0_in,sdes_in);

endfunction

function cost = ga_costHorLocalJerk(s_in,Z_in,sdes_in,Q_in,e0_in,stateCoMIn,rMcIn,jerkIn,SpIn,UpIn,SvIn,UvIn,SaIn,UaIn,TeIn,NpIn,ndofIn)
   sm_out = predHorLocalMireJerk(s_in, Z_in, stateCoMIn, rMcIn, jerkIn, SpIn, UpIn, SvIn, UvIn, SaIn, UaIn, TeIn, NpIn, ndofIn);

   cost   = ga_costHorizon(Np_in,sm_out,Q_in,e0_in,sdes_in);

endfunction




//----TODO 
// on pourrait fusionner ces deux fonction en une seule
// en passant en argument la fonction de prediction
// rq: la fonction global n'a pas bsoin du calcul de L
//-----------------------------------------------//
//  
//    cost function for SQP 
//
//-----------------------------------------------//
function cost = cld_costSQPMire(index,Uc_in)
  global Z_global;
  global s_global;
  global sdes_global;               
  global computeL_global;
  L_out  = computeL_global(s_global,Z_global);
  costv  = L_out*Uc_in - (-s_global+sdes_global);
  cost   = costv'*costv;   
endfunction

function cost = ga_costSQPMire(index,Uc_in)
  global Te_global ;
  global Z_global;
  global Q_global;
  global s_global;
  global e0_global;                    // error between model and truth 
  global sdes_global;               
  global computeL_global;
  L_out  = computeL_global(s_global,Z_global);
  sm_out = s_global+ Te_global*L_out*Uc_in;
  err   = sdes_global - sm_out;
  cost   = err'*err;     
endfunction

function cost = ga_costSQPGlobalMire(index,Uc_in)
  global Te_global ;
  global Z_global;
  global Q_global;
  global s_global;
  global e0_global;                    // error between model and truth 
  global sdes_global;  
  [sm_out,Z_out] = ga_predGlobalMire(s_global,Z_global,Uc_in,Te_global);
    err   = sdes_global - sm_out;
  cost   = err'*err;
endfunction

function cost = testSQP(index,Uc_in)
  err    = Uc_in-[1 2 3 4 5 6]';
  cost   = err'*err;     
endfunction

//-----------------------------------------------//
// COST HORIZON FUNCTION ITRO 2010 ALLIBERT 
// author Claire Dune
// date 04/01/2010
//-----------------------------------------------//
function cost = ga_costLocalMire(index,Uc_in)
  global Nc_global ;
  global Np_global ;
  global Te_global ;
  global Z_global;
  global Q_global;
  global s_global;
  global e0_global;                    // error between model and truth 
  global sdes_global;               
  global Nbpts_global;
  Up_out = computeControlOnHorizon(Uc_in,Nc_global,Np_global);
  
  cost = ga_costHorLoc2dMire(s_global,Z_global,Up_out,Te_global,Np_global,Q_global,e0_global,sdes_global);
  if (isnan(cost))
    disp('cost is nan')
    halt()
  end

endfunction

function cost = ga_costGlobalMire(index,Uc_in)
  global Nc_global ;
  global Np_global ;
  global Te_global ;
  global Z_global;
  global Q_global;
  global s_global;
  global e0_global;                    // error between model and truth 
  global sdes_global;               
  global Nbpts_global;
  Up_out = computeControlOnHorizon(Uc_in,Nc_global,Np_global);
  cost = ga_costHorGlobaldMire(s_global,Z_global,Up_out,Te_global,Np_global,Q_global,e0_global,sdes_global);
endfunction

function cost = cld_costLocalMire(index,Uc_in)
  global Nc_global ;
  global Np_global ;
  global Te_global ;
  global Z_global;
  global Q_global;
  global s_global;
  global e0_global;                    // error between model and truth 
  global sdes_global;               
  global Nbpts_global;
  Up_out = computeControlOnHorizon(Uc_in,Nc_global,Np_global);
  cost = ga_costHorLoc2dMire(s_global,Z_global,Up_out,Te_global,Np_global,Q_global,e0_global,sdes_global);
  alpha = 0.01;
  cost = cost + alpha* Up_out'*Up_out; 
endfunction


function cost = cld_costGlobalMire(index,Uc_in)
  global Nc_global ;
  global Np_global ;
  global Te_global ;
  global Z_global;
  global Q_global;
  global s_global;
  global e0_global;                    // error between model and truth 
  global sdes_global;               
  global Nbpts_global;
  Up_out = computeControlOnHorizon(Uc_in,Nc_global,Np_global);
  cost = ga_costHorGlobaldMire(s_global,Z_global,Up_out,Te_global,Np_global,Q_global,e0_global,sdes_global);
  alpha = 0.01;
  cost = cost + alpha* Up_out'*Up_out; 
endfunction


function cost = costGlobalMireJerk(index,Jerk)

  global Te_global;
  global Np_global;
  global Q_global ;
  global s_global ;
  global Z_global ;
  global e0_global;                    
  global sdes_global;               
  global Nbpts_global;  
  global ndof_global;
  global rMc_global;                 
  global stateCoM_global;  
  global Sp_global;
  global Up_global;
  global Sv_global;
  global Uv_global;  
  global Sa_global; 
  global Ua_global;
 
  
  cost = ga_costHorGlobalJerk(s_global,... 
                              Z_global,...
                              sdes_global,...
                              Q_global,...
                              e0_global,...
                              stateCoM_global,...
                              rMc_global,...
                              Jerk,...
                              Sp_global,...
                              Up_global,... 
                              Sv_global,...
                              Uv_global,... 
                              Sa_global,... 
                              Ua_global,...
                              Te_global,... 
                              Np_global,...
                              ndof_global)
 

endfunction

function cost = costLocalMireJerk(index,Jerk)

  global Te_global;
  global Np_global;
  global Q_global ;
  global s_global ;
  global Z_global ;
  global e0_global;                    
  global sdes_global;               
  global Nbpts_global;  
  global ndof_global;
  global rMc_global;                 
  global stateCoM_global;  
  global Sp_global;
  global Up_global;
  global Sv_global;
  global Uv_global;  
  global Sa_global; 
  global Ua_global;
 
  
  cost = ga_costHorLocalJerk(s_global,... 
                              Z_global,...
                              sdes_global,...
                              Q_global,...
                              e0_global,...
                              stateCoM_global,...
                              rMc_global,...
                              Jerk,...
                              Sp_global,...
                              Up_global,... 
                              Sv_global,...
                              Uv_global,... 
                              Sa_global,... 
                              Ua_global,...
                              Te_global,... 
                              Np_global,...
                              ndof_global)
 

endfunction

