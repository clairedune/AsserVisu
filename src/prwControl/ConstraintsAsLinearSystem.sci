function [d,dc,dmax,dcmax] = ConstraintsAsLinearSystem(SupportAngle, ...
						  PreviousSupportAngle, ...
						  SupportPhase, SupportFoot, StepNumber)


  
  sinSA = sin(SupportAngle);
  cosSA = cos(SupportAngle);

  
  R = [cosSA, sinSA; 
       -sinSA, cosSA];

//[cd,cdc,cdmax,cdcmax]=fort("ConstraintsAsLS",SupportAngle,5,"d",PreviousSupportAngle,6,"d",SupportPhase,7,"i",SupportFoot,8,"i",LeftFootPosConst',9,"d",LeftDSEdges',10,"d",LeftFootEdges',11,"d",RightFootPosConst',12,"d",RightDSEdges',13,"d",RightFootEdges',14,"d","out",[4,2],1,"d",[4,1],2,"d",[5,2],3,"d",[5,1],4,"d");



  if SupportPhase == 1 & SupportFoot == 1
     LFEdgesTheta = LeftFootEdges*R;
    for i = 1:4 
      d(i,1) = LFEdgesTheta(i,2)-LFEdgesTheta(i+1,2);
      d(i,2) = LFEdgesTheta(i+1,1)-LFEdgesTheta(i,1);
      dc(i) = d(i,:)*LFEdgesTheta(i,:)';
    end
  elseif SupportPhase == 1 & SupportFoot == -1 
      RFEdgesTheta = RightFootEdges*R;
    for i = 1:4
      d(i,1) = RFEdgesTheta(i,2)-RFEdgesTheta(i+1,2);
      d(i,2) = RFEdgesTheta(i+1,1)-RFEdgesTheta(i,1);
      dc(i) = d(i,:)*RFEdgesTheta(i,:)';
    end
    //symmetrize
    d = -d;
    dc = -dc;
  elseif SupportPhase == 0 & SupportFoot == 1 
      LDSEdgesTheta = LeftDSEdges*R;
    for i = 1:4 
      d(i,1) = LDSEdgesTheta(i,2)-LDSEdgesTheta(i+1,2);
      d(i,2) = LDSEdgesTheta(i+1,1)-LDSEdgesTheta(i,1);
      dc(i) = d(i,:)*LDSEdgesTheta(i,:)';
    end
  elseif SupportPhase == 0 & SupportFoot == -1 
    RDSEdgesTheta = RightDSEdges*R;
    for i = 1:4
      d(i,1) = RDSEdgesTheta(i,2)-RDSEdgesTheta(i+1,2);
      d(i,2) = RDSEdgesTheta(i+1,1)-RDSEdgesTheta(i,1);
      dc(i) = d(i,:)*RDSEdgesTheta(i,:)';
    end
    //symmetrize
    d = -d;
    dc = -dc;
  end

  if StepNumber > 0
    sinPSA = sin(PreviousSupportAngle);
    cosPSA = cos(PreviousSupportAngle);
    
    R = [cosPSA, sinPSA; 
	 -sinPSA, cosPSA];
    
    //Min-Max constraints
    mDF = minDist+dF*sin(abs(SupportAngle-PreviousSupportAngle)+alpha0) ...
	  -dF*sin(alpha0);
    
    LeftFootPosConst = [-0.3, mDF;
			-0.2, 0.3;
			0, 0.4;
			0.2, 0.3;
			0.3,  mDF
			-0.3, mDF];
    RightFootPosConst = [-0.3, -mDF;
		    -0.2, -0.3;
		    0, -0.4;
		    0.2, -0.3;
		    0.3, -mDF;
		    -0.3, -mDF];
    
    if  SupportFoot == 1
      LFPosConstTheta = LeftFootPosConst*R;
      for i = 1:5 
	dmax(i,1) = LFPosConstTheta(i,2)-LFPosConstTheta(i+1,2);
	dmax(i,2) = LFPosConstTheta(i+1,1)-LFPosConstTheta(i,1);
	dcmax(i) = dmax(i,:)*LFPosConstTheta(i,:)';
      end
    elseif SupportFoot == -1 
      RFPosConstTheta = RightFootPosConst*R;
      for i = 1:5
	dmax(i,1) =  RFPosConstTheta(i,2)-RFPosConstTheta(i+1,2);
	dmax(i,2) = RFPosConstTheta(i+1,1)-RFPosConstTheta(i,1);
	dcmax(i) = dmax(i,:)*RFPosConstTheta(i,:)';
      end
      //symmetrize
      dmax = -dmax;
      dcmax = -dcmax;
    end
  elseif StepNumber == 0
    dmax = 0;
    dcmax = 0;
  end


// $$$ 
// $$$   //tests
// $$$   if max(abs(dnew-d))>1e-6 | max(abs(dcnew-dc))>1e-6 | max(abs(dmaxnew-dmax))>1e-6 ...
// $$$ 	| max(abs(dcmaxnew-dcmax))>1e-6
// $$$ e    printf('mismatch of the new version')
// $$$     pause
// $$$   end
// $$$   if CTEST == %T
// $$$     if max(abs(cd-d))>1e-6 | max(abs(cdc-dc))>1e-6 | max(abs(cdmax-dmax))>1e-6 ...
// $$$ 	  | max(abs(cdcmax-dcmax))>1e-6
// $$$       printf('mismatch of the c version');
// $$$     end
// $$$   end
endfunction
