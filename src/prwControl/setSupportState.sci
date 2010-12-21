
function [SupportPhase, SupportFoot, StateChanged, TimeLimit, StepsLeft,  StepNumber] = setSupportState(Time,TimeLimit, Ref, SupportPhase, ...
						  SupportFoot, StepNumber, ...
						  StepsLeft, NumberSteps);
  
// $$$   [cSupportPhase, cSupportFoot, cStateChanged, cTimeLimit, cStepsLeft, ...
// $$$    cStepNumber]=fort("setSupportState",Time,1,"d",TimeLimit,2,"d",Ref,3,"d",SupportPhase,4,"i",SupportFoot,5,"i",StepNumber,6,"i",StepsLeft,7,"i",NumberSteps,8,"i",StateChanged,9,"i","out",[1,1],4,"i",[1,1],5,"i",[1,1],9,"i",[1,1],2,"d",[1,1],7,"i",[1,1],6,"i");
  
  
  //Limited ds phase when reference is given
  if (max(abs(Ref))-sqrt(%eps)>0) 
    if SupportPhase == 0 & TimeLimit-Time-sqrt(%eps) > DSSSDuration
      TimeLimit = Time+DSSSDuration;
    end
  end
  
  //verifyTimeLeft

  if Time+sqrt(%eps) >= TimeLimit
    //switchState (FSM)
    //SS->DS
    if SupportPhase == 1  & ...
	  max(abs(Ref))-sqrt(%eps)<0 & StepsLeft-sqrt(%eps)<0 
      //printf('SSDS');
      SupportPhase = 0;
      TimeLimit = Time + 1e9;
      StateChanged = 1;
      //DS->SS
    elseif SupportPhase == 0 & (max(abs(Ref))-sqrt(%eps)>0) 
      //printf('DSSS');
      SupportPhase = 1;
      SupportFoot = StartSupportFoot;
      TimeLimit = Time + SSDuration;
      StepsLeft = NumberSteps;
      StateChanged = 1;
      //SS->SS
    elseif SupportPhase == 1 & StepsLeft>0 | StepsLeft<=0 & ...
	  max(abs(Ref))-sqrt(%eps)>0
      //printf('SSSS');
      SupportFoot = -SupportFoot;
      StateChanged = 1;
      TimeLimit = Time + SSDuration;
      StepNumber = StepNumber+1;
      if max(abs(Ref))-sqrt(%eps)<0
	StepsLeft = StepsLeft-1;
      end
    end //End of FSM
  end
  

// $$$    //tests
// $$$    if CTEST == %T
// $$$      if max(abs(cSupportPhase-SupportPhase))>1e-6 | max(abs(cSupportFoot-SupportFoot))>1e-6 | max(abs(StateChanged-cStateChanged))>1e-6 ...
// $$$ 	   | max(abs(cTimeLimit-TimeLimit))>1e-6 | max(abs(cStepsLeft- ...
// $$$ 						  StepsLeft))>1e-6 | ...
// $$$ 	   max(abs(cStepNumber-StepNumber))>1e-6
// $$$     
// $$$        printf('mismatch of the c version in fsm');
// $$$        printf('[max(abs(cSupportPhase-SupportPhase)),max(abs(cSupportFoot-SupportFoot)),max(abs(StateChanged-cStateChanged)),max(abs(cTimeLimit-TimeLimit)),max(abs(cStepsLeft-  StepsLeft)),max(abs(cStepNumber-StepNumber))]');
// $$$        disp([max(abs(cSupportPhase-SupportPhase)),max(abs(cSupportFoot-SupportFoot)),max(abs(StateChanged-cStateChanged)),max(abs(cTimeLimit-TimeLimit)),max(abs(cStepsLeft-  StepsLeft)),max(abs(cStepNumber-StepNumber))]);
// $$$        pause
// $$$      end
// $$$    end
endfunction