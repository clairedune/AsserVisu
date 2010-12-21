function [PrwSupAngles, AngVelCoH, PreviewedTrunkAngle] = previewOrientations(AngVelCoH, CurAngVelCoH, TrunkAngle, TimeLimit, Time, SupportPhase, SupportFoot, SupportAngle, RightFootAngle, LeftFootAngle)
  
  DEBUG = %F;
  //c version
// $$$   [cPrwSupAngles, cAngVelCoH, cPreviewedTrunkAngle]=fort("previewOrientations",AngVelCoH,2,"d",CurAngVelCoH,4,"d",TrunkAngle,5,"d",TimeLimit,6,"d",Time,7,"d",SupportPhase,8,"i",SupportFoot,9,"i",SupportAngle,10,"d",RightFootAngle,11,"d",LeftFootAngle,12,"d",T,13,"d",MaxIntAngleTrunk,14,"d",MaxExtAngleTrunk,15,"d",SSDuration,16,"d",N,17,"i",MaxAngVelFoot,18,"d",MaxIntAngleFeet,19,"d","out",[ceil((N+1)*T/SSDuration),1],1,"d",[1,1],2,"d",[1,1],3,"d");

 if DEBUG == %T
    printf('double variables');
    disp([AngVelCoH, PreviewedTrunkAngle, CurAngVelCoH, TrunkAngle, TimeLimit, Time, SupportAngle, RightFootAngle, LeftFootAngle, T]);
    printf("Integers:");
    disp([SupportFoot, SupportPhase]);
  end
  
  
  PrwSupAngles = zeros(PreviewedSupportAngles);
  TrunkVelOK = -1;
  StartPreview = 1;
  TrunkAngleT = 0;
  
  
  while TrunkVelOK == -1

    //Compute the trunk angle at the end of the acceleration phase
    if abs(AngVelCoH-CurAngVelCoH) > sqrt(%eps)
      c = 3*(AngVelCoH-CurAngVelCoH)/T^2;
      d = -2*c/(3*T);
      TrunkAngleT = TrunkAngle + CurAngVelCoH*T+1/3*c*T^3+1/4*d*T^4;
    else
      TrunkAngleT = TrunkAngle + CurAngVelCoH*T;
    end
    
    //Verify the angle between and the trunk at the end of the current
    //support period
    if SupportPhase == 0
      TimeDifference = TimeLimit+SSDuration-Time;
      StartPreview = 2;
    else
      TimeDifference = TimeLimit-Time;
    end
    TrunkAngleEnd = TrunkAngleT + AngVelCoH*(TimeDifference-T);
    //Which limitation only has to be considered
    if SupportPhase == 0
      MaxDeviationTrunk = MaxIntAngleTrunk;
    elseif sign(AngVelCoH)*SupportFoot == 1 & SupportPhase ~= 0
      MaxDeviationTrunk = MaxIntAngleTrunk;
    else
      MaxDeviationTrunk = MaxExtAngleTrunk;
    end
    if DEBUG == %T
      printf('MaxDeviationTrunk');disp(MaxDeviationTrunk);
      //Reduce the velocity of the trunk if necessary
      printf('abs(TrunkAngleEnd - SupportAngle)');disp(abs(TrunkAngleEnd - SupportAngle));
    end

    if abs(TrunkAngleEnd - SupportAngle)>MaxDeviationTrunk
      AngVelCoH = (SupportAngle+sign(AngVelCoH)*MaxDeviationTrunk- ...
		   TrunkAngle-CurAngVelCoH*T/2)/(TimeDifference-T/2);
      if DEBUG == %T
	printf('trunk velocity reduced');
	printf('trunk velocity');disp(AngVelCoH);
      end

      //Recompute the trunk angle at the end of the acceleration phase
      if abs(AngVelCoH-CurAngVelCoH) > sqrt(%eps)
	c = 3*(AngVelCoH-CurAngVelCoH)/T^2;
	d = -2*c/(3*T);
	TrunkAngleT = TrunkAngle + CurAngVelCoH*T+1/3*c*T^3+1/4*d*T^4;
      else
	if DEBUG == %T
	  printf('no change')
	end
	;
	TrunkAngleT = TrunkAngle + CurAngVelCoH*T;
      end
      TrunkAngleEnd = TrunkAngleT + AngVelCoH*(TimeDifference-T);
    end

    //Prepare for the next step
    if SupportFoot == 1
      SupportAngleBefore = RightFootAngle;
    elseif SupportFoot == -1
      SupportAngleBefore = LeftFootAngle;
    end
    PrwSupportFoot = SupportFoot;
    AnotherSupportAngle = SupportAngle;
    
    if DEBUG == %T
      printf('PrwSupportFoot');disp(PrwSupportFoot');
      printf('AnotherSupportAngle');disp(AnotherSupportAngle);
      printf('SupportAngleBefore');disp(SupportAngleBefore);
      printf('RightFootAngle/LeftFootAngle');disp([RightFootAngle,LeftFootAngle]);
    end

      PrwRightFootAngle = RightFootAngle;
      PrwLeftFootAngle = LeftFootAngle
    for PrwSup = StartPreview:size(PrwSupAngles,2)
      
      PrwSupportFoot = -PrwSupportFoot;
      //compute the optimal solution
      PrwSupportAngle = TrunkAngleEnd + AngVelCoH*SSDuration/2;
      
      if DEBUG == %T
	printf('optimal angle:');disp(PrwSupportAngle);
	//verify the necessary, max., relative foot velocity 
	printf('SupportAngleBefore:');disp(SupportAngleBefore);
      end

      AngVelFoot = (PrwSupportAngle-SupportAngleBefore)/(SSDuration- ...
						  T);
      if DEBUG == %T
	printf('AngVelFoot,MaxAngVelFoot');disp([AngVelFoot,MaxAngVelFoot]);
      end
      //If necessary reduce the velocity to the maximum
      if 3/2*abs(AngVelFoot)>MaxAngVelFoot
	////printf('feet velocity reduced');
	AngVelFoot = 2/3*sign(AngVelCoH)*MaxAngVelFoot;
	//Compute the resulting angle
	PrwSupportAngle = SupportAngleBefore+AngVelFoot*(SSDuration-T);
	if DEBUG == %T
	  printf('maximal velocity');
	  printf('angle before');disp(SupportAngleBefore);
	end

      end
      
      //Verify the angle to avoid:
      //self-collision
      if PrwSupportFoot*(AnotherSupportAngle-PrwSupportAngle)-sqrt(%eps) > ...
	    MaxIntAngleFeet
	PrwSupportAngle = AnotherSupportAngle+sign(AngVelCoH)* ...
	    MaxIntAngleFeet;
	if DEBUG == %T
	  printf('self collision');
	end

	//not being able to catch-up for a rectangular DS phase
      elseif abs(PrwSupportAngle-AnotherSupportAngle) > MaxAngVelFoot* ...
	    SSDuration
	PrwSupportAngle = AnotherSupportAngle+PrwSupportFoot*MaxAngVelFoot* ...
	    (SSDuration-T);
	if DEBUG == %T
	  printf('catch up constraint');
	end

      end
      if DEBUG == %T
	printf('PrwSupportAngle');
	disp(PrwSupportAngle);
      end

      //Verify the angle between the feet and the trunk at the end of the current
      //support period
      TrunkAngleEnd = TrunkAngleEnd + AngVelCoH*SSDuration;
      //Which limitation only has to be considered?
      if sign(AngVelCoH)*PrwSupportFoot == 1
	MaxDeviationTrunk = MaxIntAngleTrunk;
      else
	MaxDeviationTrunk = MaxExtAngleTrunk;
      end
      
      //Reduce the velocity of the trunk if necessary
      if abs(TrunkAngleEnd - PrwSupportAngle) > MaxDeviationTrunk+%eps
	if DEBUG == %T
	  printf('AngVelCoH');disp(AngVelCoH);
	end

	AngVelCoH = 0.95*(PrwSupportAngle+sign(AngVelCoH)* ...
			  MaxDeviationTrunk-TrunkAngle-CurAngVelCoH*T/2)/(TimeLimit-Time+PrwSup*SSDuration-T/2);
	if DEBUG == %T
	  disp(AngVelCoH);
	end

	TrunkVelOK = -1;
	if DEBUG == %T
	  //restart the computation of all angles
	  //debug
	  printf('Angle not passed');disp(PrwSup);
	  printf('trunkvelocity:');disp(AngVelCoH)
	end
	;
	PrwSupAngles = zeros(PrwSupAngles);
	break;
      else
	//debug
	if DEBUG == %T
	  printf('Angle passed');disp(PrwSup);
	end
	TrunkVelOK = 1;
	PrwSupAngles(PrwSup-StartPreview+1) = PrwSupportAngle;
      end

      //Prepare for the next step
      if PrwSupportFoot == 1
	PrwLeftFootAngle = PrwSupportAngle;
	SupportAngleBefore = PrwRightFootAngle;
      else
	PrwRightFootAngle = PrwSupportAngle;
	SupportAngleBefore = PrwLeftFootAngle;
      end
      if DEBUG == %T
	printf('LeftFoot/RightFootAngle');disp([PrwLeftFootAngle, ...
		    PrwRightFootAngle]);
	printf('SupportAngleBefore');disp(SupportAngleBefore);
      end

      AnotherSupportAngle = PrwSupportAngle;
    end//for
  end//while

  PreviewedTrunkAngle = TrunkAngleT;


// $$$   //test of the c version
// $$$   if CTEST == %T
// $$$   if max(abs(cPrwSupAngles(1:size(PrwSupAngles,2))-PrwSupAngles'))>1e-6 | max(abs(cAngVelCoH-AngVelCoH))>1e-6 | max(abs(cPreviewedTrunkAngle-PreviewedTrunkAngle))>1e-6 
// $$$     printf('mismatch of cpreviewOrientations');
// $$$     pause
// $$$   end
// $$$   end
  
endfunction