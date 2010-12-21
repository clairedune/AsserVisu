//---------------------------------------//
//
// Computes the Q and P matrices for the linearised visual task
//
//---------------------------------------//
function [Qvisu, pTvisu]= computeQPvisuDelta(Sk,Sdes, Lbig)

global Udiag_global; 
global Vdiag_global;  
global Eframe_global;  
global Sdes_global ; 


Qvisu   = Udiag_global' * Eframe_global'*Vdiag_global'*Lbig'*Lbig*Vdiag_global*Eframe_global*Udiag_global;
pTvisu  = (Sk-Sdes)'*Lbig*Vdiag_global*Eframe_global*Udiag_global;

endfunction

//---------------------------------------//
//
// Computes the Q and P matrices for the linearised visual task
//
// take as an input dddotX Fx dddotY Fy
//
//---------------------------------------// 
function [Qfeet, pTfeet]= computeQPvisuDeltaFeet(Sk,Sdes,StepNumber, Lbig)

   global Ewalk_global  ; 
   [Qvisu pvisu]= computeQPvisuDelta(Sk,Sdes);
   Qfeet        = Ewalk_global'*Qvisu*Ewalk_global;
   pTfeet       = pTvisu*Ewalk_global;

endfunction


//---------------------------------------//
//
// Compute QP for jerk and not delta jerk
//
//---------------------------------------//
function [Qjerk, pjerk]=computeQPvisu(Sk,Sdes, Lbig,previousJerk)
    [Q pT]= computeQPvisuDelta(Sk,Sdes, Lbig);
    pT = pT-previousJerk'*Q;
endfunction

//---------------------------------------//
//
// Compute QP for jerk and not delta jerk with feet
//
//---------------------------------------//
function [Qjerk, pjerk]=computeQPvisuFeet(Sk,Sdes,StepNumber,Lbig,previousJerk)
    [Qfeet pTfeet]= computeQPvisuDeltaFeet(Sk,Sdes,StepNumber,Lbig);
    pTfeet = pTfeet-previousJerk'*Qfeet;
endfunction




