//
//  This file contains the 2D constraints computation
//  For predictive control
//



//-----------------------------------------------//
// CONSTRAINTS ON THE IMAGES
// author Claire Dune
// date 04/01/2010
//-----------------------------------------------//
function c = ga_constraintsLocalMire(index,Ucin)
  //% 
  // return [smxi -smxi smyi -smyi] 
  // 
  c=0;
  global Nc_global;
  global Np_global;
  global s_global;
  global Te_global;
  global Z_global;
  global Nbpts_global;
  global constraints_global;
  global computeL_global;
  //build Uc on the whole horizon
  Up = computeControlOnHorizon(Ucin,Nc_global,Np_global);
  
  sm = ga_predHorLoc2dMire(s_global,Z_global,Up,Te_global,computeL_global,Np_global);
 
  sx=[];
  sy=[];
  for k=1:Nbpts_global
    for i=1:Np_global
      index1 = (Nbpts_global*2)*(i-1)+(k-1)*2+1;
      index2 = (Nbpts_global*2)*(i-1)+(k-1)*2+2;
      sx = [sx;sm(index1)];
      sy = [sy;sm(index2)];
    end
  end

  // the measured features 
  cm = [sx;sy;-sx;-sy] ;
    
  if(length(constraints_global)~=length(cm))
    disp("--------------------------------")
    disp('Warning : ga_constraintsMire >>length(constraints_global)~=length(cm) ')
    disp('constraints_global length')
    disp(length(constraints_global))
    disp('cm length')
    disp(length(cm))
    disp("--------------------------------")
    resume
  end
 
  // difference btw the measured features and their bounds
  ctotal = cm - constraints_global;
  c = ctotal(index);
endfunction




//-----------------------------------------------//
// CONSTRAINTS ON THE IMAGES
// author Claire Dune
// date 04/01/2010
//-----------------------------------------------//
function c = ga_constraintsLimitsUp(Np,Nbts,xmax,ymax)
 Xmax=xmax*ones(Np*Nbts,1);
 Ymax=ymax*ones(Np*Nbts,1);
 c=[Xmax;Ymax];
endfunction

function c = ga_constraintsLimitsLow(Np,Nbts,xmin,ymin)
 Xmin=xmin*ones(Np*Nbts,1);
 Ymin=ymin*ones(Np*Nbts,1);
 c=[-Xmin;-Ymin];
endfunction

function c = ga_constraintsLimits(Np,Nbts,xmax,ymax,xmin,ymin)
 c=[ga_constraintsLimitsUp(Np,Nbts,xmax,ymax);ga_constraintsLimitsLow(Np,Nbts,xmin,ymin)];
endfunction


