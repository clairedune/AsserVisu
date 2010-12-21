//--------
// this function creates a target of Nbpts_in points
// if Nbpts=4 the target is a rectangle
// if Nbpts=5 the target is a rectangle + a non center dot 
// default is 5 dots
//--------
function [oX_out] = mirePoints(Nbpts_in, L_in)
  
  if( Nbpts_in ==4 )
    oX_out = mire4points(L_in);
  elseif (Nbpts_in==5)
    oX_out = mire5points(L_in);
  else
    oX_out = mire5points(L_in);  
  end
endfunction


//------------
// this function create a target and 
// gives its position in the camera frame
//
// if Nbpts=4 the target is a rectangle
// if Nbpts=5 the target is a rectangle + a non center dot 
// default is 5 dots
//------------------

function [oP,cP,p]=mirePointsInCam(Nbpts_in, L_in,cMo_in)
	if( Nbpts_in ==4 )
		[oP,cP,p]=mire4pointsInCam(L_in,cMo_in);
  	elseif ( Nbpts_in==5)
 		[oP,cP,p]=mire5pointsInCam(L_in,cMo_in);
	else
		[oP,cP,p]=mire5pointsInCam(L_in,cMo_in);
        end
 endfunction


//-------//
// test //

function isMirePointsLoaded()
 
disp('Mire Points is Loaded')

endfunction

