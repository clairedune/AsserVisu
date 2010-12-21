//-------------------------------------------//
//author Claire Dune
//date 06/04/10
//-------------------------------------------//

function [cInitMo_out,cFinalMo_out,LFunction_out,targetFun_out,targetSize_out,lambda_out,iterMax_out,Te_out]=avLoopDefaultSettings()
// set some default settings for the avLoop

	cFinalMo_out   = [0 0 1 0 0 0 ];              // pose target/object desired
	cInitMo_out    = [0 0 0.8 0 0 0];             // pose target/object init  
	
        LFunction_out  = matIntMireC;                 // method to compute the interaction matrix
	targetFun_out  = mire4points;                 // method to buld the target
        targetSize_out = 0.2;                         // dimension of the target    
   
        lambda_out     = 0.7;                         // visual servo gain
	iterMax_out    = 40 ;                          // number iteration of the loop  
	Te_out         = 0.25;                        // step time

endfunction

//-------------------------------------------//
//author Claire Dune
//date 06/04/10
//-------------------------------------------//

function [avPose,avError,avVelocity,avS,avSDes] = avLoop(cInitMo,cFinalMo,LFunction,target,targetSize,lambda,iterMax,Te,OPT_VERBOSE)
//
// run a visual servoing loop during iterMax iteration
// 
// arg:
// cInitMo is the first position of the camera wrt the target
// cFinalMo is the final position of the camera wrt the target
// intMatrixFunction is the type of interaction matrix : matIntMireD,matIntMireP,matIntMireC,matIntMireM
// target is the type of mire
// lambda is the asservisu gain
// iterMax is the number of iter
// 
//
// output:
// camera pose evolution
// 

// depending on the computation of L, it is needed
global Zdes_global;
global sdes_global;

//------- create the object
oP_in        = target(targetSize);                 // create the Npbts Points Target
Nbpts_in     = length(oP_in)/3 ;

// ------ First Camera Object Position
cMo_in       = homogeneousMatrixFromPos(cInitMo);  // pose target/object init   
posecMo_in   = pFromHomogeneousMatrix(cMo_in)
// compute the init projection on the view
cP_in        = changeFrameMire(oP_in,cMo_in);      // target Points in the camera frame
s_in         = projectMireDirect(cP_in);           // projection of the target points in the image plane 
Z_in         = cP_in(3:3:$) ;                      // depth of the target points in the camera frame

//------- Desired Camera Object Position 
cDesMo_in    = homogeneousMatrixFromPos(cFinalMo);

// compute the desired projection on the view
cDesP_in     = changeFrameMire(oP_in,cDesMo_in);   // desired target Points in the camera frame
sDes_in      = projectMireDirect(cDesP_in);        // desired target Points projection
ZDes_in      = cDesP_in(3:3:$) ;                    // desired depth
Zdes_global  = ZDes_in;
sdes_global  = sDes_in;
avSDes       = sdes_global;

avPose       = [];
avError      = [];
avVelocity   = [];
avS          = [];

if(OPT_VERBOSE)
	disp('--- servo loop---')
end

iter_in      = 0 ;

while(iter_in < iterMax )

  iter_in = iter_in + 1;
  
  //display the current iteration
  if(OPT_VERBOSE)
  	disp('---------------------------------')
  	disp(iter_in)
  end

  L_in       = LFunction (s_in,Z_in);
  v_in       = computeVelocity(lambda, L_in,s_in-sDes_in);
  v_in       = v_in';
  cost       = (s_in-sDes_in)'*(s_in-sDes_in);
  

  //         Update data for next iter                      
  c1Mc2_in   = computeMotion(v_in',Te)  ;         // resulting motion
  motion     = pFromHomogeneousMatrix(c1Mc2_in )
  


  //display some information about the current step
  if(OPT_VERBOSE)
  	disp('current cost: ')
  	disp(cost)
        disp('current velocity:')
        disp(v_in')
        disp('current cMo:')
        disp(posecMo_in) 
        disp('Motion')
        disp(motion) 
  end

   
  
  cMo_in     = inv(c1Mc2_in)*cMo_in;
  posecMo_in = pFromHomogeneousMatrix(cMo_in);
  cP_in      = changeFrameMire(oP_in,cMo_in); 
  Z_in       = cP_in(3:3:$) ;                     // depth for the step 
  s_in       = projectMireDirect(cP_in);
  
  //save the position 
  avPose     = [avPose;posecMo_in'];
  // save the velocity
  avVelocity = [avVelocity ; v_in'];
  // save the current error
  avError    = [avError ; cost];
  // save the point position
  avS        = [avS ;s_in];
   
end

endfunction 


