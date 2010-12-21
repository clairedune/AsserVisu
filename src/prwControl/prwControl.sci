// Copyright (C) INRIA 1999-2009
// 
// This program is free software; you can redistribute it and/or modify it
// under the terms of the GNU General Public License version 2 as published
// by the Free Software Foundation.
// 
// This program is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
// Public License for more details.
// 
// You should have received a copy of the GNU General Public License along
// with this program; if not, write to the Free Software Foundation, Inc.,
// 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
disp('Loading preview Orientation ...')
exec('src/prwControl/previewOrientations.sci');
disp('Loading preview setSupportState ...')
exec('src/prwControl/setSupportState.sci');
disp('Loading preview ConstraintsAsLinearSystem ...')
exec('src/prwControl/ConstraintsAsLinearSystem.sci');


OPT_VERBOSE     = %F;                //to display comments/debug
disp('Loading walking parameters ...')
// Simulation parameters
ONLINE_PLOT     = %F;
CARRY_TABLE     = %F;
COMHEIGHT_CONST = %T;
perturbation    = %F;
PLDP            = %F;
zeroActiveSet   = %T;
ASSERVISU       = %F;
// parameters for inverse kinematics

AnkleHeight     = 10.5e-2;           // meters
HandForward     = 4.2e-2;            // meters
HandHeight      = 70e-2;             // meters
HandWidth       = 66.2e-2;           // meters
HipHeight       = 64.5e-2 - 70.5e-2; // meters
CoMHeight       = 80e-2;             // meters
Gravity         = -9.81;             // meters*seconds^-2
ArmGainX        = -0*0.6;
StepHeight      = 7e-2;              // meters
StepWidth       = 19e-2;             // meters
MaxAcceleration = 1e9;               // meters*seconds^-2
SecurityMargin  = 2e-2;              // meters
MinFeetDistance = 0.1;               // meters

InitialWalkingPosition = [0;...
		    0.003409;...
		    -0.474919;...
		    0.901748;...
		    -0.426830;...
		    -0.003409;...
		    0;...
		    0.003408;...
		    -0.473995;...
		    0.899922;...
		    -0.425927;...
		    -0.003408;...
		    0;...
		    0;...
		    0;...
		    0;...
		    0.273002;...
		    -0.174075;...
		    0;...
		    -0.514174;...
		    0;...
		    0;...
		    0.174533;...
		    0.273002;...
		    0.174075;...
		    0;...
		    -0.514174;...
		    0;...
		    0;...
		    0.174533;...
		    -0.012984;...
		    -0.060000;...
		    0.001840;...
		    0;...
		    0;...
		    0];


// QP gains
// Qalpha = 1e-2;
// Qbeta = 0*1e-0;
// QgammaSag = 1e-6;
// QgammaLat = 1e-6;
// Qdelta = 1e-0;
Qalpha    = 1e-6;     //jerk
Qbeta     = 1e-3;     //velocity  1e-2 for projection
QgammaSag = 1e-6;     //ZMP
QgammaLat = 1e-6;     //ZMP
Qdelta    = 0*1e+0;   //mean velocity
Qepsilon  = 0*1e+0;   //instantaneous, relative position
Qdzeta    = 1e+0;
Qeta      = 1e+0;
CoeffVisu = 0;

//if ASSERVISU
//CoeffVisu = 1e-3;
//Qbeta     = 0;
//Qdelta    = 0;      //mean velocity
//Qepsilon  = 0;      //instantaneous, relative position
//Qdzeta    = 0;
//Qeta      = 0;  
//end


//Walking Pattern
NumberSteps = 1;      //Number of steps befor stop in DS
SSDuration = 0.8;     //Duration of one step
DSDuration = 1e9;     //Duration of the DS phase
DSSSDuration = 0.4;   //Minimal stay length in the DS phase before a switch to SS
DSDistance = 2e-1;    //Distance between the feet in the DS phase
DistanceCoHCoM = 0;
StartSupportFoot = 1; //First support foot
FirstStep = %F;       //First support phase after double support

disp('Loading preview parameters...')
//Preview window
N = 32;               //Number of samples previewed
T= 0.1; 
Tend = 20;            //Simulation length in sec

disp('Loading Limits on velocity and accelerations...')
//Limitations
MaxAccTrunk = 2;
MaxAngVelFoot = %pi/5;
MaxIntAngleTrunk = 30*%pi/180;
MaxExtAngleTrunk = 45*%pi/180;
MaxIntAngleFeet = 5*%pi/180;

minDist = 0.20;
dF = 0.1569343;
alpha0 = 0.5275239;


disp('Loading Polygonal CoP Constraints ...')
//Polygonal CoP constraints
LeftFootEdges = [0.1356 - SecurityMargin,   0.079-SecurityMargin;
		 0.1356-SecurityMargin,-0.059+SecurityMargin;
		-0.1056+SecurityMargin,-0.059+SecurityMargin;
		-0.1056+SecurityMargin,   0.079-SecurityMargin;
		 0.1356-SecurityMargin,   0.079-SecurityMargin];
RightFootEdges = [0.1356-SecurityMargin,   -0.079+SecurityMargin;
		 0.1356-SecurityMargin,0.059-SecurityMargin;
		 -0.1056+SecurityMargin,0.059-SecurityMargin;
		 -0.1056+SecurityMargin,   -0.079+SecurityMargin;
		  0.1356-SecurityMargin,   -0.079+SecurityMargin];
LeftDSEdges = [0.1356-SecurityMargin,   0.079-SecurityMargin;
		    0.1356-SecurityMargin, -DSDistance/2-0.079+SecurityMargin;
		   -0.1056+SecurityMargin, -DSDistance/2-0.079+SecurityMargin;
		   -0.1056+SecurityMargin,   0.079-SecurityMargin;
		    0.1356-SecurityMargin,   0.079-SecurityMargin];
RightDSEdges = [0.1356-SecurityMargin,   DSDistance/2+0.079-SecurityMargin;
		    0.1356-SecurityMargin, -0.079+SecurityMargin;
		   -0.1056+SecurityMargin, -0.079+SecurityMargin;
		   -0.1056+SecurityMargin, DSDistance/2+0.079-SecurityMargin;
		    0.1356-SecurityMargin, DSDistance/2+0.079-SecurityMargin];

RealLeftFootEdges = [0.1356,  0.079;
		    0.1356, -0.059;
		    -0.1056, -0.059;
		    -0.1056,  0.079;
		    0.1356,  0.079];
RealRightFootEdges = [-0.1056, -0.079;
		    -0.1056,  0.059;
		    0.1356, 0.059;
		    0.1356, -0.079;
		    -0.1056, -0.079];

//Polygonal feet position constraints
LeftFootPosConst = [-0.3, 0.15;
		    -0.2, 0.3;
		    0, 0.4;
		    0.2, 0.3;
		    0.3, 0.15
		    -0.3, 0.15];
RightFootPosConst = [-0.3, -0.15;
		    -0.2,  -0.3;
		    0, -0.4;
		    0.2, -0.3;
		    0.3, -0.15;
		    -0.3, -0.15];


disp('Loading global variables ...')
global CurrentSupportPhase;	
global CurrentSupportFoot;
global CurrentTimeLeft;			//Time left in the present phase
global CurrentStepsLeft;		//Steps left before switch to DS
global CurrentLeftFootAngle;
global CurrentRightFootAngle;
global NextSupportPhase;	 	//Chain of previewed support phases
global NextSupportFoot;//Previewed support phases
global NextStepsLeft;			//Chain of resting steps
global NextFootAngle;//Previewed feet angles
global TrunkAngle;
global Xdotref;
global Ydotref;
global FeetDown;
global Xhist;
global Yhist;
global PreviewedSupportAngles;
global jerkPrec;


disp('Loading init FSM state ...')
//Initial state of the FSM
CurrentSupportPhase = 0;				//Start with DS
CurrentSupportFoot = StartSupportFoot;
CurrentTimeLimit = 1e9;
CurrentStepsLeft = NumberSteps;		
CurrentLeftFootAngle = 0;
CurrentRightFootAngle = 0;
StateChanged = -1;
PreviewedSupportAngles = zeros(1,ceil((N+1)*T/SSDuration));
PreviewedTrunkAngle = 0;
ActuationSamplingPeriod = 2.5e-3;



disp('Loading CoM sinusoidale Profile ...')
//Sinusoidal profile for the CoM
CoMHeightVar = zeros(1,(Tend+N*T)/ActuationSamplingPeriod);

if(OPT_RECOMPUTE_MATRICES)

if COMHEIGHT_CONST == %F
  for i = 1:2*Tend/ActuationSamplingPeriod
    CoMHeightVar(i) = 0.05*sin(2*%pi*(i*ActuationSamplingPeriod- ...
					  DSSSDuration)/0.8-%pi/2);
    dCoMHeightVar(i) = 2*%pi/0.8*0.05*cos(2*%pi*(i*ActuationSamplingPeriod- ...
					   DSSSDuration)/0.8-%pi/2);
    ddCoMHeightVar(i) = -2*%pi/0.8*2*%pi/0.8*0.05*sin(2*%pi*(i*ActuationSamplingPeriod-DSSSDuration)/0.8-%pi/2);
  end
else
  for i = 1:2*Tend/ActuationSamplingPeriod
    CoMHeightVar(i) = 0;
    dCoMHeightVar(i) = 0;
    ddCoMHeightVar(i) = 0;
  end
end


disp('Loading Sp,Ss,Sa and Up, Us, Ua matrices ...')
for i = 1:N
  Pps(i, :) = [1, i*T, i^2*T^2/2];
  Pvs(i, :) = [0, 1, i*T];
  Pas(i, :) = [0, 0, 1];
  for j = 1:N
    if j>i 
      Ppu(i, j) = 0;
      Pvu(i, j) = 0;
      Pau(i, j) = 0;
    else
      Ppu(i, j) = (3*(i-j)^2+3*(i-j)+1)*T^3/6;
      Pvu(i, j) = (2*(i-j)+1)*T^2/2;
      Pau(i, j) = T;
    end
  end
end

disp('Loading Spvisu,Ssvisu,Savisu and Upvisu, Usvisu, Uavis matrices ...')
PvsVisu = [[0,1,0];Pvs];
PvsVisu = PvsVisu(1:N,:);
PvuVisu = [zeros(1,N);Pvu];
PvuVisu = PvuVisu(1:N,:);

disp('Loading Pzs,Pzu matrices ...')
for n = 1:Tend/T
  for i = 1:N
    Pzs(i, :,n) = [1, i*T, i^2*T^2/2+(CoMHeight+CoMHeightVar((n-1)*T/ActuationSamplingPeriod+1))/Gravity];
    for j = 1:N
      if j>i 
	Pzu(i, j,n) = 0;
      else
	Pzu(i, j,n) = (3*(i-j)^2+3*(i-j)+1)*T^3/6+T*(CoMHeight+CoMHeightVar((n-1+j)*T/ActuationSamplingPeriod+1))/Gravity;
      end
    end
  end
end
end//  if(OPT_RECOMPUTE_MATRICES)
Xhist = [];
Yhist = [];


TEST = %F;

//Visu
jerkPrec = [];


disp('Loading functions ...')

//--------------------------
// Compose QP Visu
//--------------------------

function [C, Cmax, b, Q, pT, U, Uc, AngVelCoH, StepNumber, FeetDown,PreviewedSupportAngles, PreviewedTrunkAngle] = composeQPVisu(Time, CoM,FeetDown, FP, TrunkAngle, Ref, PreviewedSupportAngles,Sk,Lbig);


  //Reference Vector auxiliary variable

  global CurrentSupportPhase;
  global CurrentSupportFoot;
  global CurrentTimeLimit;		
  global CurrentStepsLeft;
  global CurrentLeftFootAngle;
  global CurrentRightFootAngle;
  global Xdotref;
  global Ydotref;
  global StateChanged;
  global Udiag_global; 
  global Vdiag_global;  
  global Eframe_global;  
  global Sdes_global ; 
  global Ewalk_global  ; 
  global Sdes_global;


  //-----------------------------------//
  //  Gestion des pieds
  //-----------------------------------//
  if CurrentSupportFoot == 1
    CurrentSupportAngle = CurrentLeftFootAngle;
  else
    CurrentSupportAngle = CurrentRightFootAngle;
  end
  
  [CurrentSupportPhase, CurrentSupportFoot, StateChanged, CurrentTimeLimit, CurrentStepsLeft,    StepNumber] = setSupportState(Time, ...
   			      CurrentTimeLimit,... 
                              Sk-Sdes_global,... 
                              CurrentSupportPhase, ...
			      CurrentSupportFoot, ...
			      0, ...
                              CurrentStepsLeft, ...
                              NumberSteps);
  if(OPT_VERBOSE)
    printf('CurrentState:');
    disp([CurrentSupportPhase, CurrentSupportFoot, ...
		    StateChanged, CurrentTimeLimit, CurrentStepsLeft, StepNumber]);
  end
  
  if StateChanged == 1
    if StepNumber == 0
      //Position of the support foot is fixed
      FP = FeetDown($,[1,2]);
    else
      CurrentSupportAngle = PreviewedSupportAngles(1);
    end
    FeetDown = [FeetDown; [FP,CurrentSupportAngle,CurrentSupportFoot, ...
		    Time]];
    
    if CurrentSupportFoot == 1 
      CurrentLeftFootAngle = CurrentSupportAngle;
    else
      CurrentRightFootAngle = CurrentSupportAngle;
    end
  end
    
  [PreviewedSupportAngles, AngVelCoH, PreviewedTrunkAngle] = previewOrientations(AngVelCoH, CurAngVelCoH, ...
						  TrunkAngle, ...
						  CurrentTimeLimit, ...
						  Time, CurrentSupportPhase, ...
						  CurrentSupportFoot, ...
						  CurrentSupportAngle, ...
						  CurrentRightFootAngle, ...
						  CurrentLeftFootAngle);

  //------------------------------------
  //       Initialization of the FSM 
  //-------------------------------------
  SupportPhase   = CurrentSupportPhase;
  SupportFoot    = CurrentSupportFoot;
  StepsLeft      = CurrentStepsLeft;
  TimeLimit      = CurrentTimeLimit;
  StepNumber     = 0;
  
  if CurrentSupportFoot == 1
    SupportAngle         = CurrentLeftFootAngle;
    PreviousSupportAngle = CurrentRightFootAngle;
  else
    SupportAngle         = CurrentRightFootAngle;
    PreviousSupportAngle = CurrentLeftFootAngle;
  end

  //Declare the matrices
  D         = zeros(4*N, 2*N);
  DRP       = zeros(4*N, 1);
  DRPmax    = zeros(5, 1);
  U         = [];
  V         = zeros(1, 1);
  Uc        = zeros(N, 2);
  UFPmaxX   = zeros(1, 1);
  UFPmaxY   = zeros(1, 1);
  b         = zeros(4*N, 1);
  Zxy       = zeros(2*N,1);
  FPx       = [];
  FPy       = [];

  [d,dc,dmax,dcmax] = ConstraintsAsLinearSystem(SupportAngle, PreviousSupportAngle,SupportPhase, SupportFoot);

// $$$ printf('back in detcon');
// $$$ pause

  for i = 1:N
    [SupportPhase, SupportFoot, StateChanged, TimeLimit, StepsLeft, StepNumber] = ...
                 setSupportState(Time+i*T, ...
  			  	 TimeLimit, ...
                                 Sk-Sdes_global, ...
                                 SupportPhase, ...
				 SupportFoot, ...
                                 StepNumber, ...
				 StepsLeft, ...
                                 NumberSteps);
    
    if StepNumber > 0
      SupportAngle         = PreviewedSupportAngles(StepNumber);
    else
      PreviousSupportAngle = SupportAngle;
      SupportAngle         = PreviousSupportAngle;
    end
    
   // disp(Time+i*T);
// $$$     disp([SupportPhase, SupportFoot, StateChanged, TimeLimit, StepsLeft, NextSupportPhase, ...
// $$$      NextSupportFoot, NextStepsLeft, SupportAngle, StepNumber])

    if(OPT_VERBOSE)
      disp(Time+i*T)
      printf('SupportAngle:')
      disp(SupportAngle)
    end
    
    if StepNumber > 1
      V(StepNumber, [StepNumber-1,StepNumber]) = [-1,1];
      UFPmaxX(StepNumber)  = 0;
      UFPmaxY(StepNumber)  = 0;
      if SupportPhase == 0 
	U(i, StepNumber)   = 1/2;
	U(i, StepNumber-1) = 1/2;
      else
	U(i, StepNumber)   = 1;
      end
    elseif StepNumber == 1 
      if SupportFoot == -FeetDown($,4);
	UFPmaxX(StepNumber) = FeetDown($,1);
	UFPmaxY(StepNumber) = FeetDown($,2);
      else
	UFPmaxX(StepNumber) = FeetDown($-1,1);
	UFPmaxY(StepNumber) = FeetDown($-1,2);
      end
      V(StepNumber, 1) = 1;
      if SupportPhase == 0   
	U(i, StepNumber) = 1/2;
	Uc(i, 2) = 1/2;
      elseif SupportPhase ~= 0 
	U(i, StepNumber) = 1;
      end
    else
      if SupportPhase == 0 
	Uc(i, 1) = 1/2;
	Uc(i, 2) = 1/2;
      else
	if SupportFoot == FeetDown($,4);
	  Uc(i, 2) = 1;
	else
	  Uc(i, 1) = 1;
	end  
      end
    end		
    
    
    if StateChanged == 1  
      StateChanged = -1;

      [d,dc,dmax,dcmax] = ConstraintsAsLinearSystem(SupportAngle, ...
						    PreviousSupportAngle,SupportPhase, SupportFoot);
      
    end
    


    D(4*i-3:4*i, i) = d(1:4, 1);
    D(4*i-3:4*i, N+i) = d(1:4, 2);
    DRP(4*i-3:4*i) = dc(1:4);
    //Min-Max constraints 
    if StepNumber > 0
      DmaxX(5*StepNumber-4:5*StepNumber, StepNumber) = dmax(1:5, 1);
      DmaxY(5*StepNumber-4:5*StepNumber, StepNumber) = dmax(1:5, 2);
      DRPmax(5*StepNumber-4:5*StepNumber) = dcmax(1:5);
    end
    
    
    if TEST == %T
      if TEST_STORE == %T
	fprintfMat('/Users/andrei/Devel/UnitTestPG/D'+string(k*100+i),D);
	fprintfMat('/Users/andrei/Devel/UnitTestPG/DRP'+string(k*100+i), ...
		   DRP);
	if StepNumber >0
	  fprintfMat('/Users/andrei/Devel/UnitTestPG/DmaxX'+string(k*100+i),DmaxX);
	  fprintfMat('/Users/andrei/Devel/UnitTestPG/DRPmax'+string(k*100+i), ...
		     DRPmax);
	end
      else 
	testD = fscanfMat('/Users/andrei/Devel/UnitTestPG/D'+string(k*100+i));
	testDRP = fscanfMat('/Users/andrei/Devel/UnitTestPG/DRP'+string(k* ...
						  100+i));
	if StepNumber >0
	  testDmaxX= fscanfMat('/Users/andrei/Devel/UnitTestPG/DmaxX'+string(k*100+i));
	  testDRPmax = fscanfMat('/Users/andrei/Devel/UnitTestPG/DRPmax'+ ...
				 string(k*100+i));
	  if max(abs(DmaxX-testDmaxX))>1e-6 | max(abs(DRPmax-testDRPmax))> ...
		1e-6
	    printf('max(abs(DmaxX-testDmaxX)),max(abs(DRPmax-testDRPmax))');
	    disp([max(abs(DmaxX-testDmaxX)),max(abs(DRPmax-testDRPmax))]);
	    pause
	  end
	end
	
	if max(abs(D-testD))>1e-6 | max(abs(DRP-testDRP))>1e-6
	  printf('max(abs(D-testD)),max(abs(DRP-testDRP))');
	  disp([max(abs(D-testD)),max(abs(DRP-testDRP))]);
	  pause
	end
      end
    end

    
    TransVelLatSum = TransVelLat+DistanceCoHCoM*AngVelCoH; 
    //Global reference
    Xdotref(i) = TransVelSag*cos(PreviewedTrunkAngle+(i-1)*T*AngVelCoH)+TransVelLatSum*sin(PreviewedTrunkAngle+(i-1)*T*AngVelCoH);
    Ydotref(i) = TransVelSag*sin(PreviewedTrunkAngle+(i-1)*T*AngVelCoH)-TransVelLatSum*cos(PreviewedTrunkAngle+(i-1)*T*AngVelCoH);

  end;
  



  UFP(1:N) =  Uc*[FeetDown($-1, 1);FeetDown($, 1)];
  UFP(N+1:2*N) = Uc*[FeetDown($-1, 2);FeetDown($,2)];

  C = D*[Pzu(:,:,Time/T+1), -U, zeros(N, N+StepNumber); zeros(N, N+StepNumber), Pzu(:,:,Time/T+1), -U];

  if  StepNumber > 0
    Cmax = [DmaxX,DmaxY]*[zeros(StepNumber, N), V, zeros(StepNumber, N+StepNumber); zeros(StepNumber, N+StepNumber), zeros(StepNumber, N), V];
    C = [C; Cmax];
  else
    Cmax = [];
  end
  
  b = D*(UFP-[Pzs(:,:,Time/T+1), zeros(N,3); zeros(N,3), Pzs(:,:,Time/T+1)]*CoM)+DRP;
  if  StepNumber > 0
    bmax = [DmaxX,DmaxY]*([UFPmaxX;UFPmaxY])+DRPmax;
    b = [b; bmax];
  else
    bmax = [];
  end



Xprime = Xhist(:,1)+2*SSDuration*Xdotref;
Yprime = Yhist(:,1)+2*SSDuration*Ydotref;

Xpdzeta = 2*SSDuration*Xdotref(1:N-2*SSDuration/T);
Ypdzeta = 2*SSDuration*Ydotref(1:N-2*SSDuration/T);


Z = ones(N,N);

Edzeta = zeros(N-2*SSDuration/T,N);
Edzeta(1:$,1:N-2*SSDuration/T) = -eye(N-2*SSDuration/T,N-2*SSDuration/T);
Edzeta(1:$,$-N+2*SSDuration/T+1:$) =eye(N-2*SSDuration/T,N-2* ...
					SSDuration/T);

disp('avant calculClaire')
pause
P2 = eye(N,N);
//Claire:Computation of Ewalk
global Ewalk_global  ; 
Ewalk_global= [[eye(N,N), zeros(N,StepNumber*2+N)];[zeros(N,StepNumber+N),eye(N,N), zeros(N,StepNumber)]] ;

QVisu       = Ewalk_global'*Udiag_global'*Eframe_global'*Vdiag_global'*Lbig'*Lbig*Vdiag_global*Eframe_global*Udiag_global*Ewalk_global;
pTVisu      =2*(Sk-Sdes_global)'*Lbig*Vdiag_global*Eframe_global*Udiag_global*Ewalk_global;

Qprime = [Qalpha*eye(N,N)+Qbeta*P2*Pvu'*Pvu*P2+Qepsilon*Ppu'*Ppu+Qdzeta*Ppu'*Edzeta'*Edzeta*Ppu+QgammaSag*Pzu(:,:,Time/T+1)'*Pzu(:,:,Time/T+1)+Qdelta*Pvu'*Z*Pvu, -QgammaSag*Pzu(:,:,Time/T+1)'*U; -QgammaLat*U'*Pzu(:,:,Time/T+1), QgammaLat*U'*U];
Q = [Qprime, zeros(Qprime); zeros(Qprime), Qprime]+CoeffVisu*QVisu;

pT = [Qbeta*(CoM(1:$/2)'*Pvs'-Xdotref')*Pvu*P2+Qepsilon*(CoM(1:$/2)'*Pps'-Xprime')*Ppu+Qdzeta*(CoM(1:$/2)'*Pps'*Edzeta'-Xpdzeta')*Edzeta*Ppu+...
      QgammaSag*(CoM(1:$/2)'*Pzs(:,:,Time/T+1)'-[FeetDown($-1, 1),FeetDown($,1)]*Uc')*Pzu(:,:,Time/T+1)+Qdelta*(CoM(1:$/2)'*Pvs'*Z*Pvu-Xdotref'*Z*Pvu),...
      -QgammaSag*(CoM(1:$/2)'*Pzs(:,:,Time/T+1)'-[FeetDown($-1, 1),FeetDown($,1)]*Uc')*U,...
      Qbeta*(CoM($/2+1:$)'*Pvs'-Ydotref')*Pvu*P2+Qepsilon*(CoM($/2+1:$)'*Pps'-Yprime')*Ppu+Qdzeta*(CoM($/2+1:$)'*Pps'*Edzeta'-Ypdzeta')*Edzeta*Ppu+...
      QgammaLat*(CoM($/2+1:$)'*Pzs(:,:,Time/T+1)'-[FeetDown($-1, 2),FeetDown($,2)]*Uc')*Pzu(:,:,Time/T+1)+Qdelta*(CoM($/2+1:$)'*Pvs'*Z*Pvu-Ydotref'*Z*Pvu),...
      -QgammaLat*(CoM($/2+1:$)'*Pzs(:,:,Time/T+1)'-[FeetDown($-1,2),FeetDown($,2)]*Uc')*U];

global jerkPrec
if jerkPrec == []
  jerkPrec = zeros(2*N+2*StepNumber,1);
end
disp(size(jerkPrec))
disp(size(pT))
disp(StepNumber)
pT = pT+CoeffVisu*(pTVisu-2*jerkPrec'*QVisu);

endfunction


//--------------------------------------
//  detconqld
//--------------------------------------

function [C, Cmax, b, Q, pT, U, Uc, AngVelCoH, StepNumber, FeetDown,PreviewedSupportAngles, PreviewedTrunkAngle] = detcon_qld(Time, CoM,FeetDown, FP, TrunkAngle, Ref, PreviewedSupportAngles);

// $$$   if FP == []
// $$$     printf('FP = []')
// $$$     pause
// $$$   end
  
// $$$   //out variable cC, cCmax, cb, cQ, cpT, cU, cUc
// $$$   //cStepNumber 
// $$$ [ cAngVelCoH, cFeetDown, cPreviewedSupportAngles, cPreviewedTrunkAngle]=fort("composeQP", AngVelCoH,1,"d",N,2,"i", ...
// $$$ 						  Pzs(:,:,Time/T+1)',3,"d", Pzu(:,:,Time/T+1)',4,"d", FeetDown',5,"d", size(FeetDown,1),6,"i",...
// $$$ 						  PreviewedSupportAngles',7,"d",...
// $$$ 						  Time,8,"d", CoM,9,"d", FP,10,"d",...
// $$$ 						  TrunkAngle,11,"d", ...
// $$$ 						  size(PreviewedSupportAngles,2),12,"i",CurrentSupportPhase,13,"i",CurrentSupportFoot,14,"i", ...
// $$$ 						  CurrentTimeLimit,15,"d",CurrentStepsLeft,16,"i",NumberSteps,17,"i",CurrentLeftFootAngle,18,"d",...
// $$$ 						  CurrentRightFootAngle,19,"d",Ref,20,"d",T,21,"d",SSDuration,22,"d",CurAngVelCoH,23,"d",...
// $$$ 						  MaxIntAngleTrunk,24,"d", MaxExtAngleTrunk,25,"d", MaxAngVelFoot,26,"d",...
// $$$ 						  MaxIntAngleFeet,27,"d",PreviewedTrunkAngle,28,"d", ...
// $$$ 						  "out",[1,1],1,"d",[size(FeetDown,1),5],5,"d",[ceil((N+1)*T/SSDuration),1],7,"d",[1,1],28,"d");

//size of FeetDown not know in advance.


  //Reference Vector auxiliary variable

  global CurrentSupportPhase;
  global CurrentSupportFoot;
  global CurrentTimeLimit;		
  global CurrentStepsLeft;
  global CurrentLeftFootAngle;
  global CurrentRightFootAngle;
  global Xdotref;
  global Ydotref;
  global StateChanged;
  global PreviewedSupportAngles;
  

  if CurrentSupportFoot == 1
    CurrentSupportAngle = CurrentLeftFootAngle;
  else
    CurrentSupportAngle = CurrentRightFootAngle;
  end
  
  [CurrentSupportPhase, CurrentSupportFoot, StateChanged, CurrentTimeLimit, CurrentStepsLeft, StepNumber] = setSupportState(Time, ...
						  CurrentTimeLimit, Ref, CurrentSupportPhase, ...
						  CurrentSupportFoot, ...
						  0, CurrentStepsLeft, NumberSteps);
 
  if(OPT_VERBOSE)
    printf('CurrentState:');
    disp([CurrentSupportPhase, CurrentSupportFoot, ...
		    StateChanged, CurrentTimeLimit, CurrentStepsLeft, StepNumber]);
  end
  
  
  if StateChanged == 1
    if StepNumber == 0
      //Position of the support foot is fixed
      FP = FeetDown($,[1,2]);
    else
      CurrentSupportAngle = PreviewedSupportAngles(1);
    end
    FeetDown = [FeetDown; [FP,CurrentSupportAngle,CurrentSupportFoot, ...
		    Time]];
    
    if CurrentSupportFoot == 1 
      CurrentLeftFootAngle = CurrentSupportAngle;
    else
      CurrentRightFootAngle = CurrentSupportAngle;
    end
  end
  


  [PreviewedSupportAngles, AngVelCoH, PreviewedTrunkAngle] = previewOrientations(AngVelCoH, CurAngVelCoH, ...
						  TrunkAngle, ...
						  CurrentTimeLimit, ...
						  Time, CurrentSupportPhase, ...
						  CurrentSupportFoot, ...
						  CurrentSupportAngle, ...
						  CurrentRightFootAngle, ...
						  CurrentLeftFootAngle);
  

 //test
// $$$   if max(abs(cPreviewedSupportAngles'-PreviewedSupportAngles))>1e-6 | max(abs(cAngVelCoH-AngVelCoH))>1e-6 | max(abs(cPreviewedTrunkAngle-PreviewedTrunkAngle))>1e-6 
// $$$        printf('mismatch of the c version in detcon');
// $$$        //printf("[max(abs(cPreviewedSupportAngles'-PreviewedSupportAngles)), max(abs(cAngVelCoH-AngVelCoH)), max(abs(cPreviewedTrunkAngle-PreviewedTrunkAngle))]");
// $$$        disp([max(abs(cPreviewedSupportAngles'-PreviewedSupportAngles)), ...
// $$$ 	     max(abs(cAngVelCoH-AngVelCoH)), max(abs(cPreviewedTrunkAngle-PreviewedTrunkAngle))]);
// $$$        pause
// $$$      end

  //Initialization of the FSM 
  SupportPhase = CurrentSupportPhase;
  SupportFoot = CurrentSupportFoot;
  StepsLeft = CurrentStepsLeft;
  TimeLimit = CurrentTimeLimit;
  StepNumber = 0;
  
  if CurrentSupportFoot == 1
    SupportAngle = CurrentLeftFootAngle;
    PreviousSupportAngle = CurrentRightFootAngle;
  else
    SupportAngle = CurrentRightFootAngle;
    PreviousSupportAngle = CurrentLeftFootAngle;
  end

  //Declare the matrices
  D = zeros(4*N, 2*N);
  DRP = zeros(4*N, 1);
  DRPmax = zeros(5, 1);
  U = [];
  V = zeros(1, 1);
  Uc = zeros(N, 2);
  UFPmaxX = zeros(1, 1);
  UFPmaxY = zeros(1, 1);
  b = zeros(4*N, 1);
  Zxy = zeros(2*N,1);
  FPx = [];
  FPy = [];

  [d,dc,dmax,dcmax] = ConstraintsAsLinearSystem(SupportAngle, PreviousSupportAngle,SupportPhase, SupportFoot);

// $$$ printf('back in detcon');
// $$$ pause

  for i = 1:N

    [SupportPhase, SupportFoot, StateChanged, TimeLimit, StepsLeft, StepNumber] = setSupportState(Time+i*T, ...
						  TimeLimit, Ref, SupportPhase, ...
						  SupportFoot, StepNumber, ...
						  StepsLeft, NumberSteps);
    
    if StepNumber > 0
      SupportAngle = PreviewedSupportAngles(StepNumber);
    else
      PreviousSupportAngle = SupportAngle;
      SupportAngle = PreviousSupportAngle;
    end
    
    
    if StepNumber > 1
      V(StepNumber, [StepNumber-1,StepNumber]) = [-1,1];
      UFPmaxX(StepNumber) = 0;
      UFPmaxY(StepNumber) = 0;
      if SupportPhase == 0 
	U(i, StepNumber) = 1/2;
	U(i, StepNumber-1) = 1/2;
      else
	U(i, StepNumber) = 1;
      end
    elseif StepNumber == 1 
      if SupportFoot == -FeetDown($,4);
	UFPmaxX(StepNumber) = FeetDown($,1);
	UFPmaxY(StepNumber) = FeetDown($,2);
      else
	UFPmaxX(StepNumber) = FeetDown($-1,1);
	UFPmaxY(StepNumber) = FeetDown($-1,2);
      end
      V(StepNumber, 1) = 1;
      if SupportPhase == 0   
	U(i, StepNumber) = 1/2;
	Uc(i, 2) = 1/2;
      elseif SupportPhase ~= 0 
	U(i, StepNumber) = 1;
      end
    else
      if SupportPhase == 0 
	Uc(i, 1) = 1/2;
	Uc(i, 2) = 1/2;
      else
	if SupportFoot == FeetDown($,4);
	  Uc(i, 2) = 1;
	else
	  Uc(i, 1) = 1;
	end  
      end
    end		
    
    
    if StateChanged == 1  
      StateChanged = -1;

      [d,dc,dmax,dcmax] = ConstraintsAsLinearSystem(SupportAngle, ...
						    PreviousSupportAngle,SupportPhase, SupportFoot);
      
    end
    

    D(4*i-3:4*i, i) = d(1:4, 1);
    D(4*i-3:4*i, N+i) = d(1:4, 2);
    DRP(4*i-3:4*i) = dc(1:4);
    //Min-Max constraints 
    if StepNumber > 0
      DmaxX(5*StepNumber-4:5*StepNumber, StepNumber) = dmax(1:5, 1);
      DmaxY(5*StepNumber-4:5*StepNumber, StepNumber) = dmax(1:5, 2);
      DRPmax(5*StepNumber-4:5*StepNumber) = dcmax(1:5);
    end
    
    
    if TEST == %T
      if TEST_STORE == %T
	fprintfMat('/Users/andrei/Devel/UnitTestPG/D'+string(k*100+i),D);
	fprintfMat('/Users/andrei/Devel/UnitTestPG/DRP'+string(k*100+i), ...
		   DRP);
	if StepNumber >0
	  fprintfMat('/Users/andrei/Devel/UnitTestPG/DmaxX'+string(k*100+i),DmaxX);
	  fprintfMat('/Users/andrei/Devel/UnitTestPG/DRPmax'+string(k*100+i), ...
		     DRPmax);
	end
      else 
	testD = fscanfMat('/Users/andrei/Devel/UnitTestPG/D'+string(k*100+i));
	testDRP = fscanfMat('/Users/andrei/Devel/UnitTestPG/DRP'+string(k* ...
						  100+i));
	if StepNumber >0
	  testDmaxX= fscanfMat('/Users/andrei/Devel/UnitTestPG/DmaxX'+string(k*100+i));
	  testDRPmax = fscanfMat('/Users/andrei/Devel/UnitTestPG/DRPmax'+ ...
				 string(k*100+i));
	  if max(abs(DmaxX-testDmaxX))>1e-6 | max(abs(DRPmax-testDRPmax))> ...
		1e-6
	    printf('max(abs(DmaxX-testDmaxX)),max(abs(DRPmax-testDRPmax))');
	    disp([max(abs(DmaxX-testDmaxX)),max(abs(DRPmax-testDRPmax))]);
	    pause
	  end
	end
	
	if max(abs(D-testD))>1e-6 | max(abs(DRP-testDRP))>1e-6
	  printf('max(abs(D-testD)),max(abs(DRP-testDRP))');
	  disp([max(abs(D-testD)),max(abs(DRP-testDRP))]);
	  pause
	end
      end
    end

    
    TransVelLatSum = TransVelLat+DistanceCoHCoM*AngVelCoH; 
    poseWMr = [0,0,0,0,0,TrunkAngle];
    WMr = homogeneousMatrixFromPose(poseWMr);
    wVr = twistMatrix(WMr);
    velLocal = [TransVelSag, TransVelLatSum, 0,0,0, AngVelCoH];
    velGlobal = wVr*velLocal';
    AngVelCoH = velGlobal(6);
    valX = velGlobal(1);
    valY = velGlobal(2);
    Xdotref(i) = velGlobal(1);
    Ydotref(i) = velGlobal(2);
    
// $$$     //Global reference
// $$$     Xdotref(i) = TransVelSag*cos(TrunkAngle+i*T*valTheta)+TransVelLatSum*sin(TrunkAngle+i*T*AngVelCoH);
// $$$     Ydotref(i) = TransVelSag*sin(TrunkAngle+i*T*)-TransVelLatSum*cos(TrunkAngle+i*T*AngVelCoH);
       
  end;

  UFP(1:N) =  Uc*[FeetDown($-1, 1);FeetDown($, 1)];
  UFP(N+1:2*N) = Uc*[FeetDown($-1, 2);FeetDown($,2)];

  C = D*[Pzu(:,:,Time/T+1), -U, zeros(N, N+StepNumber); zeros(N, N+StepNumber), Pzu(:,:,Time/T+1), -U];

  if  StepNumber > 0
    Cmax = [DmaxX,DmaxY]*[zeros(StepNumber, N), V, zeros(StepNumber, N+StepNumber); zeros(StepNumber, N+StepNumber), zeros(StepNumber, N), V];
    C = [C; Cmax];
  else
    Cmax = [];
  end
  
  b = D*(UFP-[Pzs(:,:,Time/T+1), zeros(N,3); zeros(N,3), Pzs(:,:,Time/T+1)]*CoM)+DRP;
  if  StepNumber > 0
    bmax = [DmaxX,DmaxY]*([UFPmaxX;UFPmaxY])+DRPmax;
    b = [b; bmax];
  else
    bmax = [];
  end



Xprime = Xhist(:,1)+2*SSDuration*Xdotref;
Yprime = Yhist(:,1)+2*SSDuration*Ydotref;

Xpdzeta = 2*SSDuration*Xdotref(1:N-2*SSDuration/T);
Ypdzeta = 2*SSDuration*Ydotref(1:N-2*SSDuration/T);


Z = ones(N,N);

Edzeta = zeros(N-2*SSDuration/T,N);
Edzeta(1:$,1:N-2*SSDuration/T) = -eye(N-2*SSDuration/T,N-2*SSDuration/T);
Edzeta(1:$,$-N+2*SSDuration/T+1:$) =eye(N-2*SSDuration/T,N-2* ...
					SSDuration/T);


P2 = eye(N,N);

Qprime = [Qalpha*eye(N,N)+Qbeta*P2*Pvu'*Pvu*P2+Qepsilon*Ppu'*Ppu+Qdzeta*Ppu'*Edzeta'*Edzeta*Ppu+QgammaSag*Pzu(:,:,Time/T+1)'*Pzu(:,:,Time/T+1)+Qdelta*Pvu'*Z*Pvu, -QgammaSag*Pzu(:,:,Time/T+1)'*U; -QgammaLat*U'*Pzu(:,:,Time/T+1), QgammaLat*U'*U];
Q = [Qprime, zeros(Qprime); zeros(Qprime), Qprime];

pT = [Qbeta*(CoM(1:$/2)'*Pvs'-Xdotref')*Pvu*P2+Qepsilon*(CoM(1:$/2)'*Pps'-Xprime')*Ppu+Qdzeta*(CoM(1:$/2)'*Pps'*Edzeta'-Xpdzeta')*Edzeta*Ppu+...
      QgammaSag*(CoM(1:$/2)'*Pzs(:,:,Time/T+1)'-[FeetDown($-1, 1),FeetDown($,1)]*Uc')*Pzu(:,:,Time/T+1)+Qdelta*(CoM(1:$/2)'*Pvs'*Z*Pvu-Xdotref'*Z*Pvu),...
      -QgammaSag*(CoM(1:$/2)'*Pzs(:,:,Time/T+1)'-[FeetDown($-1, 1),FeetDown($,1)]*Uc')*U,...
      Qbeta*(CoM($/2+1:$)'*Pvs'-Ydotref')*Pvu*P2+Qepsilon*(CoM($/2+1:$)'*Pps'-Yprime')*Ppu+Qdzeta*(CoM($/2+1:$)'*Pps'*Edzeta'-Ypdzeta')*Edzeta*Ppu+...
      QgammaLat*(CoM($/2+1:$)'*Pzs(:,:,Time/T+1)'-[FeetDown($-1, 2),FeetDown($,2)]*Uc')*Pzu(:,:,Time/T+1)+Qdelta*(CoM($/2+1:$)'*Pvs'*Z*Pvu-Ydotref'*Z*Pvu),...
      -QgammaLat*(CoM($/2+1:$)'*Pzs(:,:,Time/T+1)'-[FeetDown($-1,2),FeetDown($,2)]*Uc')*U];


endfunction

function [a] = polyinterpol(x_start, x_end);
  V = [0, 0, 0, 0, 0, 1;
       0, 0, 0, 0, 1, 0;
       0, 0, 0, 2, 0, 0;
       1, 1, 1, 1, 1, 1;
       5, 4, 3, 2, 1, 0;
       20,12,6, 2, 0, 0];
  invV = inv(V);
  b = [x_start; x_end];
  a = invV*b;
endfunction

function [a] = polyinterpol2(x_start, x_end);
  W = [0, 0, 0, 1;
       0, 0, 1, 0;
       1, 1, 1, 1;
       3, 2, 1, 0];
  invW = inv(W);
  b = [x_start; x_end];
  a = invW*b;
endfunction


//-----------------------------------------------
//
// Walking trajectory with FP
//
//
//-------------------------------------------------

function [RobotReal, RobotPrw, RobotMean] = WalkingTrajectory_withFP(vRobotDes,Robot,k)//Sstar,Sk,L,R,Mxy,Myx,Myy)

 
  if k == Tend/T-2
    printf('Pause in Walking Trajecoty with Foot Prints :sorry can not go any further');
    pause
  end

  global TrunkAngle;
  global CurrentLeftFootAngle;
  global CurrentRightFootAngle;
  global NextFootAngle;
  global NextAngVelFoot;
  global Xdotref;
  global Ydotref;
  global FeetDown;
  global Xhist;
  global Yhist;
  global FP;
  global jerkPrec;
  
  standalone = %F;
  
  if standalone == %F 
    ActuationSamplingPeriod = getActuationSamplingPeriod();
  else
    ActuationSamplingPeriod = 2.5e-3;
  end;


    RaisingFoot = %F;
    FP_tz = 0;
    
    x = Robot(1:3);
    y = Robot(4:6);
    TrunkAngle = Robot(7);
    

    // CoM vars for interpolation; using PB's names from inv kinematic code
    CoM_ins = [];
    ZMP_ins = [];
    FP_ins = [];
    FP_ins_x_vec = [];
    FP_ins_x_vec2 = [];
    FP_ins_y_vec = [];
    FP_ins_y_vec2 = [];
    XdotrefVec = [];
    YdotrefVec = [];    
    XdotMeanVec = [];
    YdotMeanVec = [];
    RefSag = [];
    RefLat = [];
    RefRot = [];
    RefTrunkAngle = 0;
    x_av = 0;
    y_av = 0;
    CoH     = [];
    CoM     = [];
    dCoM    = [];
    ddCoM   = [];
    ZMP     = [];
    Supports= [];

    //Initial values for CoM, Ankle, ZMP
    if k == 0
      CurrentLeftFootAngle = TrunkAngle;
      CurrentRightFootAngle = TrunkAngle;
      if StartSupportFoot == 1
	FeetDown = [Robot(1)+0, Robot(4)-0.1, CurrentRightFootAngle, -1, 0; Robot(1)+0, Robot(4)+0.1, CurrentLeftFootAngle, ...
		    1, 0];
      else
	FeetDown = [0, 0.1, CurrentLeftFootAngle, 1, 0; 0, -0.1, ...
		    CurrentRightFootAngle, -1, 0];
      end
      
      //Initialize the state history 
      Xhist = [];
      Yhist = [];
      for hist = 1:N
	Xhist = [Xhist;x'];
	Yhist = [Yhist;y'];
      end
      FP = FeetDown($,[1,2]);//new
    end
    

    tic();
    


TransVelSag = vRobotDes(1);
TransVelLat = vRobotDes(2);
AngVelCoHRef = vRobotDes(3);
CurAngVelCoH = Robot(8);
AngVelCoH = AngVelCoHRef;
Ref = [vRobotDes(1),vRobotDes(2),vRobotDes(3)];

//Verify change in velocity against the maximal acceleration
if abs(AngVelCoH-CurAngVelCoH) > 2/3*T*MaxAccTrunk
  //printf('maximal acceleration');
  AngVelCoH = CurAngVelCoH + sign(AngVelCoH-CurAngVelCoH)*2/3*T* ...
      MaxAccTrunk;
end

if ((perturbation == %T) & (k*T >= 5)) then
// $$$   x(2,k) = x(2,k) - 0.3;
  y(2,k) = y(2,k) + 0.5;
  perturbation = %F;
end;


  [C, Cmax, b, Q, pT, U, Uc, AngVelCoH, StepNumber, FeetDown, PreviewedSupportAngles,PreviewedTrunkAngle] ...
      = detcon_qld(k*T, [x; y], FeetDown, FP, TrunkAngle, Ref, ...
		   PreviewedSupportAngles);


lb = -1e9*ones(size(Q, 1), 1);
ub =  1e9*ones(size(Q, 1), 1);

me = 0;
[res, lagr, info] = qld(Q, pT, C, b, lb, ub, me, 1e-13);

jerkPrec = res;

if info ~= 0 
  printf('qld.info: ');
  disp(info);
end

// store results from the first interval
jerk = [res(1), res(N+StepNumber+1)];

if StepNumber > 0
  FP = [res(N+1), res(2*N+StepNumber+1)];
else
  if CurrentSupportFoot == 1
    FP = [FeetDown($,1)+DSDistance*sin(FeetDown($,3)),FeetDown($,2)- ...
	  DSDistance*cos(FeetDown($,3))];
  else
    FP = [FeetDown($,1)-DSDistance*sin(FeetDown($,3)),FeetDown($,2)+ ...
	  DSDistance*cos(FeetDown($,3))];
  end
  //pause
end

xnew = [1, T, T^2/2; 0, 1, T; 0, 0, 1]*x+[T^3/6; T^2/2; ...
		    T]*jerk(1);
ynew = [1, T, T^2/2; 0, 1, T; 0, 0, 1]*y+[T^3/6; T^2/2;T]* ...
    jerk(2);

//Update the history
Xhist = [Xhist;xnew'];
Xhist = Xhist($-N+1:$,1:3);
Yhist = [Yhist;ynew'];
Yhist = Yhist($-N+1:$,1:3);

//For plots
x_ins(:, 1) = x;
y_ins(:, 1) = y;
ZMP_ins(:, 1) = [[1, 0, (CoMHeight+CoMHeightVar((k+2)*T/ActuationSamplingPeriod+1))/Gravity]*[x_ins(1,1); x_ins(2,1); ...
		    x_ins(3,1)]; [1, 0, (CoMHeight+CoMHeightVar((k+2)*T/ActuationSamplingPeriod+1))/Gravity]*[y_ins(1,1); y_ins(2,1); y_ins(3,1)]];

if StepNumber > 1
  FP_ins_x = [res(N+1),res(N+2)];
  FP_ins_x_vec = [FP_ins_x_vec,res(N+1)];
  FP_ins_x_vec2 = [FP_ins_x_vec2,res(N+2)];
  FP_ins_y = [res(2*N+StepNumber+1),res(2*N+StepNumber+2)];
  FP_ins_y_vec = [FP_ins_y_vec,res(2*N+StepNumber+1)];
  FP_ins_y_vec2 = [FP_ins_y_vec2,res(2*N+StepNumber+2)];
  FP_ins = [FP_ins_x, FP_ins_y];
elseif StepNumber == 1
  FP_ins_x = [res(N+1),0];
  FP_ins_x_vec = [FP_ins_x_vec,res(N+1)];
  FP_ins_x_vec2 = [FP_ins_x_vec2,0];
  FP_ins_y = [res(2*N+StepNumber+1),0];
  FP_ins_y_vec = [FP_ins_y_vec,res(2*N+StepNumber+1)];
  FP_ins_y_vec2 = [FP_ins_y_vec2,0];
  FP_ins = [FP_ins_x, FP_ins_y];
else
  FP_ins_x = [0,0];
  FP_ins_x_vec = [FP_ins_x_vec,0];
  FP_ins_x_vec2 = [FP_ins_x_vec2,0];
  FP_ins_y = [0,0];
  FP_ins_y_vec = [FP_ins_y_vec,0];
  FP_ins_y_vec2 = [FP_ins_y_vec2,0];
  FP_ins = [FP_ins_x, FP_ins_y];
end


//Previewed state
TrunkAnglePrw = [TrunkAngle];
for i = 1 : N
  x_ins(:, i+1) = [1, T, T^2/2; 0, 1, T; 0, 0, 1]*x_ins(:, i)+[T^3/6; ...
		    T^2/2; T]*res(i);
  y_ins(:, i+1) = [1, T, T^2/2; 0, 1, T; 0, 0, 1]*y_ins(:, i)+[T^3/6; ...
		    T^2/2; T]*res(N+StepNumber+i);
  ZMP_ins(:, i+1) = [[1, 0, (CoMHeight+CoMHeightVar((k-1+i)*T/ActuationSamplingPeriod+1))/Gravity]*[x_ins(1, i+1); x_ins(2, ...
						  i+1); x_ins(3, i+1)]; ...
		     [1, 0, (CoMHeight+CoMHeightVar((k-1+i)*T/ActuationSamplingPeriod+1))/Gravity]*[y_ins(1,i+1); y_ins(2, ...
						  i+1); y_ins(3, i+ ...
						  1)]];
  if i == 1
    TrunkAnglePrw = [TrunkAnglePrw; PreviewedTrunkAngle];
  else
    TrunkAnglePrw = [TrunkAnglePrw; TrunkAngle + AngVelCoH*i*T];
  end
end
//

XdotrefVec = [XdotrefVec, Xdotref(1)];
YdotrefVec = [YdotrefVec, Ydotref(1)];
XdotMean = mean(x_ins(2,:));
XdotMeanVec = [XdotMeanVec, XdotMean];
YdotMean = mean(y_ins(2,:));
YdotMeanVec = [YdotMeanVec, YdotMean];
RefSag = [RefSag, TransVelSag];
RefLat = [RefLat, TransVelLat];
RefRot = [RefRot, AngVelCoH];
RefTrunkAngle = [RefTrunkAngle, RefTrunkAngle($)+AngVelCoHRef*T];


if ONLINE_PLOT==%T & k>=0
  //if k>20
  //if modulo(k,4)==0
  xset('window',11);
  subplot(2,2,1);
  plot2d([k*T:T:(k+N)*T], x_ins(1,:), style=[color("blue")]);
  xtitle("sagittal CoM","time [s]","CoM_x");
  
  // xset('window',1);
  subplot(2,2,2); 
  //xname('CoM_y(t)');
  plot2d([k*T:T:(k+N)*T], y_ins(1,:), style=[color("green")]);
  xtitle("lateral CoM","time [s]","CoM_y");
  
  //xset('window',2);
  subplot(2,2,3);
  //xname('CoM_v_x(t)');
  plot2d([k*T:T:(k+N)*T], x_ins(2,:), style=[color("blue")]);
  xtitle("sagittal velocity","time [s]","dX");
  
  //xset('window',3);
  subplot(2,2,4);
  //xname('CoM_v_y(t)');
  plot2d([k*T:T:(k+N)*T], y_ins(2,:), style=[color("green")]);
  xtitle("lateral velocity","time [s]","dY");
  
  xset('window',12);
  subplot(2,2,1);
  //xname('ZMPx(k*T)');
  plot2d([k*T:T:(k+N)*T], ZMP_ins(1,:), style=[color("blue")]);
  xtitle("sagittal ZMP","time [s]","ZMP_x");
  
  //xset('window',5);
  subplot(2,2,2);
  //xname('ZMPy(k*T)');
  plot2d([k*T:T:(k+N)*T], ZMP_ins(2,:), style=[color("green")]);
  plot2d([k*T:T:(k+N)*T], y_ins(1,:), style=[color("red")]);
  xtitle("lateral ZMP","time [s]","ZMP_y, CoM_y");
  
end//ONLINE_PLOT

if(OPT_VERBOSE)
  printf('vRobotDes');
  disp(vRobotDes);
  printf('RobotReal');
  disp(RobotReal);
end
//pause
TrunkAngle = PreviewedTrunkAngle;

//Output
//mean velocity
xdotmean = (xnew(1)-x(1))/T;
ydotmean = (ynew(1)-y(1))/T;
RobotReal = [xnew;ynew;TrunkAngle;AngVelCoH;0];
RobotPrw = [x_ins;y_ins;TrunkAnglePrw'];
RobotMean = [xdotmean;ydotmean];

endfunction





///--------------------------------------------------------------------------------------------//
//  Vision inside
///-------------------------------------------------------------------

function [RobotReal, RobotPrw, RobotMean] = WalkingWithVision(vRobotDes,Robot,k,Sk,Lbig)

  if k == Tend/T-2
    printf('sorry can not go any further');
    pause
  end

  global TrunkAngle;
  global CurrentLeftFootAngle;
  global CurrentRightFootAngle;
  global NextFootAngle;
  global NextAngVelFoot;
  global Xdotref;
  global Ydotref;
  global FeetDown;
  
  global Xhist;
  global Yhist;
  global FP;
  global jerkPrec;
  
  standalone = %F;
  
  if standalone == %F 
    ActuationSamplingPeriod = getActuationSamplingPeriod();
  else
    ActuationSamplingPeriod = 2.5e-3;
  end;


    RaisingFoot = %F;
    FP_tz = 0;
    
    x = Robot(1:3);
    y = Robot(4:6);
    TrunkAngle = Robot(7);
    

    // CoM vars for interpolation; using PB's names from inv kinematic code
    CoM_ins = [];
    ZMP_ins = [];
    FP_ins = [];
    FP_ins_x_vec = [];
    FP_ins_x_vec2 = [];
    FP_ins_y_vec = [];
    FP_ins_y_vec2 = [];
    XdotrefVec = [];
    YdotrefVec = [];    
    XdotMeanVec = [];
    YdotMeanVec = [];
    RefSag = [];
    RefLat = [];
    RefRot = [];
    RefTrunkAngle = 0;
    x_av = 0;
    y_av = 0;
    CoH     = [];
    CoM     = [];
    dCoM    = [];
    ddCoM   = [];
    ZMP     = [];
    Supports= [];

    //Initial values for CoM, Ankle, ZMP
    if k == 0
      CurrentLeftFootAngle = Robot(7);
      CurrentRightFootAngle = Robot(7);
      if StartSupportFoot == 1
	FeetDown = [Robot(1)+0, Robot(4)-0.1, CurrentRightFootAngle, -1, 0; Robot(1)+0, Robot(4)+0.1, CurrentLeftFootAngle, ...
		    1, 0];
      else
	FeetDown = [0, 0.1, CurrentLeftFootAngle, 1, 0; 0, -0.1, ...
		    CurrentRightFootAngle, -1, 0];
      end
      
      //Initialize the state history 
      Xhist = [];
      Yhist = [];
      for hist = 1:N
	Xhist = [Xhist;x'];
	Yhist = [Yhist;y'];
      end
      FP = FeetDown($,[1,2]);//new
    end
    

    tic();
    


TransVelSag = vRobotDes(1);
TransVelLat = vRobotDes(2);
AngVelCoHRef = vRobotDes(3);
CurAngVelCoH = Robot(8);
AngVelCoH = AngVelCoHRef;
Ref = [vRobotDes(1),vRobotDes(2),vRobotDes(3)];

//Verify change in velocity against the maximal acceleration
if abs(AngVelCoH-CurAngVelCoH) > 2/3*T*MaxAccTrunk
  //printf('maximal acceleration');
  AngVelCoH = CurAngVelCoH + sign(AngVelCoH-CurAngVelCoH)*2/3*T* ...
      MaxAccTrunk;
end

if ((perturbation == %T) & (k*T >= 5)) then
// $$$   x(2,k) = x(2,k) - 0.3;
  y(2,k) = y(2,k) + 0.5;
  perturbation = %F;
end;


  [C, Cmax, b, Q, pT, U, Uc, AngVelCoH, StepNumber, FeetDown, PreviewedSupportAngles,PreviewedTrunkAngle] ...
      = composeQPVisu(k*T, Robot(1:6),FeetDown, FP, TrunkAngle, Ref, PreviewedSupportAngles,Sk,Lbig);





lb = -1e9*ones(size(Q, 1), 1);
ub =  1e9*ones(size(Q, 1), 1);
me = 0;
[res, lagr, info] = qld(Q, pT, C, b, lb, ub, me, 1e-13);

//claire Change
//lb = -1e1*ones(size(Q, 1), 1);
//ub =  1e1*ones(size(Q, 1), 1);
//[res, lagr, info] = qld(Q, pT, [], [], lb, ub, 0, 1e-13);


jerkPrec = res;

if info ~= 0 
  printf('qld.info: ');
  disp(info);
end

// store results from the first interval
jerk = [res(1), res(N+StepNumber+1)];

if StepNumber > 0
  FP = [res(N+1), res(2*N+StepNumber+1)];
else
  if CurrentSupportFoot == 1
    FP = [FeetDown($,1)+DSDistance*sin(FeetDown($,3)),FeetDown($,2)- ...
	  DSDistance*cos(FeetDown($,3))];
  else
    FP = [FeetDown($,1)-DSDistance*sin(FeetDown($,3)),FeetDown($,2)+ ...
	  DSDistance*cos(FeetDown($,3))];
  end
 // pause
end

xnew = [1, T, T^2/2; 0, 1, T; 0, 0, 1]*x+[T^3/6; T^2/2; ...
		    T]*jerk(1);
ynew = [1, T, T^2/2; 0, 1, T; 0, 0, 1]*y+[T^3/6; T^2/2;T]* ...
    jerk(2);



//Update the history
Xhist = [Xhist;xnew'];
Xhist = Xhist($-N+1:$,1:3);
Yhist = [Yhist;ynew'];
Yhist = Yhist($-N+1:$,1:3);

//For plots
x_ins(:, 1) = x;
y_ins(:, 1) = y;
ZMP_ins(:, 1) = [[1, 0, (CoMHeight+CoMHeightVar((k+2)*T/ActuationSamplingPeriod+1))/Gravity]*[x_ins(1,1); x_ins(2,1); ...
		    x_ins(3,1)]; [1, 0, (CoMHeight+CoMHeightVar((k+2)*T/ActuationSamplingPeriod+1))/Gravity]*[y_ins(1,1); y_ins(2,1); y_ins(3,1)]];
if StepNumber > 1
  FP_ins_x = [res(N+1),res(N+2)];
  FP_ins_x_vec = [FP_ins_x_vec,res(N+1)];
  FP_ins_x_vec2 = [FP_ins_x_vec2,res(N+2)];
  FP_ins_y = [res(2*N+StepNumber+1),res(2*N+StepNumber+2)];
  FP_ins_y_vec = [FP_ins_y_vec,res(2*N+StepNumber+1)];
  FP_ins_y_vec2 = [FP_ins_y_vec2,res(2*N+StepNumber+2)];
  FP_ins = [FP_ins_x, FP_ins_y];
elseif StepNumber == 1
  FP_ins_x = [res(N+1),0];
  FP_ins_x_vec = [FP_ins_x_vec,res(N+1)];
  FP_ins_x_vec2 = [FP_ins_x_vec2,0];
  FP_ins_y = [res(2*N+StepNumber+1),0];
  FP_ins_y_vec = [FP_ins_y_vec,res(2*N+StepNumber+1)];
  FP_ins_y_vec2 = [FP_ins_y_vec2,0];
  FP_ins = [FP_ins_x, FP_ins_y];
else
  FP_ins_x = [0,0];
  FP_ins_x_vec = [FP_ins_x_vec,0];
  FP_ins_x_vec2 = [FP_ins_x_vec2,0];
  FP_ins_y = [0,0];
  FP_ins_y_vec = [FP_ins_y_vec,0];
  FP_ins_y_vec2 = [FP_ins_y_vec2,0];
  FP_ins = [FP_ins_x, FP_ins_y];
end


//Previewed state
TrunkAnglePrw = [TrunkAngle];
for i = 1 : N
  x_ins(:, i+1) = [1, T, T^2/2; 0, 1, T; 0, 0, 1]*x_ins(:, i)+[T^3/6; ...
		    T^2/2; T]*res(i);
  y_ins(:, i+1) = [1, T, T^2/2; 0, 1, T; 0, 0, 1]*y_ins(:, i)+[T^3/6; ...
		    T^2/2; T]*res(N+StepNumber+i);
  ZMP_ins(:, i+1) = [[1, 0, (CoMHeight+CoMHeightVar((k-1+i)*T/ActuationSamplingPeriod+1))/Gravity]*[x_ins(1, i+1); x_ins(2, ...
						  i+1); x_ins(3, i+1)]; ...
		     [1, 0, (CoMHeight+CoMHeightVar((k-1+i)*T/ActuationSamplingPeriod+1))/Gravity]*[y_ins(1,i+1); y_ins(2, ...
						  i+1); y_ins(3, i+ ...
						  1)]];
  if i == 1
    TrunkAnglePrw = [TrunkAnglePrw; PreviewedTrunkAngle];
  else
    TrunkAnglePrw = [TrunkAnglePrw; TrunkAngle + AngVelCoH*i*T];
  end
end
//

XdotrefVec = [XdotrefVec, Xdotref(1)];
YdotrefVec = [YdotrefVec, Ydotref(1)];
XdotMean = mean(x_ins(2,:));
XdotMeanVec = [XdotMeanVec, XdotMean];
YdotMean = mean(y_ins(2,:));
YdotMeanVec = [YdotMeanVec, YdotMean];
RefSag = [RefSag, TransVelSag];
RefLat = [RefLat, TransVelLat];
RefRot = [RefRot, AngVelCoH];
RefTrunkAngle = [RefTrunkAngle, RefTrunkAngle($)+AngVelCoHRef*T];

//disp('before plot');
//pause

if ONLINE_PLOT==%T & k>=0
  //if k>20
  //if modulo(k,4)==0
  xset('window',11);
  subplot(2,2,1);
  plot2d([k*T:T:(k+N)*T], x_ins(1,:), style=[color("blue")]);
  xtitle("sagittal CoM","time [s]","CoM_x");
  
  // xset('window',1);
  subplot(2,2,2); 
  //xname('CoM_y(t)');
  plot2d([k*T:T:(k+N)*T], y_ins(1,:), style=[color("green")]);
  xtitle("lateral CoM","time [s]","CoM_y");
  
  //xset('window',2);
  subplot(2,2,3);
  //xname('CoM_v_x(t)');
  plot2d([k*T:T:(k+N)*T], x_ins(2,:), style=[color("blue")]);
  xtitle("sagittal velocity","time [s]","dX");
  
  //xset('window',3);
  subplot(2,2,4);
  //xname('CoM_v_y(t)');
  plot2d([k*T:T:(k+N)*T], y_ins(2,:), style=[color("green")]);
  xtitle("lateral velocity","time [s]","dY");
  
  xset('window',12);
  subplot(2,2,1);
  //xname('ZMPx(k*T)');
  plot2d([k*T:T:(k+N)*T], ZMP_ins(1,:), style=[color("blue")]);
  xtitle("sagittal ZMP","time [s]","ZMP_x");
  
  //xset('window',5);
  subplot(2,2,2);
  //xname('ZMPy(k*T)');
  plot2d([k*T:T:(k+N)*T], ZMP_ins(2,:), style=[color("green")]);
  plot2d([k*T:T:(k+N)*T], y_ins(1,:), style=[color("red")]);
  xtitle("lateral ZMP","time [s]","ZMP_y, CoM_y");
  
end//ONLINE_PLOT




TrunkAngle = PreviewedTrunkAngle;

//Output
//mean velocity
xdotmean = (xnew(1)-x(1))/T;
ydotmean = (ynew(1)-y(1))/T;
RobotReal = [xnew;ynew;TrunkAngle;AngVelCoH;0];
RobotPrw = [x_ins;y_ins;TrunkAnglePrw'];
RobotMean = [xdotmean;ydotmean];

if(OPT_VERBOSE)
  printf('vRobotDes');disp(vRobotDes);
  printf('RobotReal');disp(RobotReal);
end
endfunction

function isPrwControlLoaded()
disp('PrwControl is Loaded')  
endfunction
