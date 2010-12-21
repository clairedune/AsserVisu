
function [RobotReal,  wMr, vout]=runPG(vrobot,RobotReal,k,wMri)
  vinput = vrobot([1,2,6]);
 [RobotReal, RobotPrw] = WalkingTrajectory_withFP(vinput,RobotReal,k);
 [riMr,vout]           = computePGoutput(RobotReal)
 wMr = wMri*riMr;
endfunction
  

//function vinput = computePGinput(vref,riMr,wMri)
//  wVri          = twistMatrix(wMri);
//  vref2         = inv(wVri)*vref;
//  vinput        = zeros(1,3);
//  vinput(1)     = vref2(1);
//  vinput(2)     = vref2(2);
// riVr          = twistMatrix([riMr(:,1:3) [zeros(3,1);1]]);
//  vinput(3)     = vref(6);
//endfunction

function [wMr,vout]=computePGoutput(RobotReal)
  pout = [RobotReal(1) RobotReal(4) 0 0 0 RobotReal(7)];
  vout = [RobotReal(2) RobotReal(5) 0 0 0 RobotReal(8)]';
  wMr  = homogeneousMatrixFromPos(pout);
  wVr  = twistMatrix([wMr(:,1:3) [zeros(3,1);1]]);
  vout = inv(wVr)*vout; 
endfunction


function [comMc,comVc,wMcom,wVcom] = loadHRP2camComAll()
  //transcmoc = [ 0 0 -1 0*%pi/180 90*%pi/180 -90*%pi/180 ];
  transcmoc = [ 0 0 1 0 90*%pi/180 90*%pi/180];
  poser     = [ 0 0 0 -%pi 0 0]';
  comMc = homogeneousMatrixFromPos(transcmoc);
  wMcom = homogeneousMatrixFromPos(poser);
  wVcom = twistMatrix(wMcom );
  comVc = twistMatrix(comMc);
endfunction

function [comMc,comVc] = loadHRP2camCom()
  //transcmoc = [ 0 0 -1 0*%pi/180 90*%pi/180 -90*%pi/180 ];
  transcmoc = [ 0 0 1 0 90*%pi/180 90*%pi/180 ];
  comMc = homogeneousMatrixFromPos(transcmoc);
   comVc = twistMatrix(comMc);
endfunction

function vrobot = convertVelocityCamRobot(vcam,rVc)
  // convert the velocity expressed in the cam frame to
  // the robot frame

  if(length(vcam)==6)
    vr = rVc*vcam';
  elseif(length(vcam)==3)  
    vc = [vcam(1) 0 vcam(2) 0 vcam(3) 0];
    vr = rVc*vc';
  end;
    
    vrobot =[vr(1) vr(2) vr(6)];
endfunction

  
function vcam = convertVelocityRobotCam(vrobot,rVc)
  // convert the velocity expressed in the cam frame to
  // the robot frame
    if(length(vrobot)==6)
       vr= [vrobot(1) vrobot(2) vrobot(3) vrobot(4) vrobot(5) vrobot(6)];
    else
       vr= [vrobot(1) vrobot(2) 0 0 0 vrobot(3)];
    end
     vc = inv(rVc)*vr';
    vcam =[vc(1) vc(3) vc(5)];
endfunction


function vWorld = convVelocityRobotWorld(vrobot,wVr)
  // convert the velocity expressed in the cam frame to
  // the robot frame

    vr= [vrobot(1) vrobot(2) 0 0 0 vrobot(3)];
    vw = wVr*vr';
    vWorld =[vc(1) vc(2) vc(6)];
endfunction


//-----------------------------------//
// create a diagonal matrix of Np V matrix
//------------------------------------//
function diagV = buildDiagV(Np)
   
    [comMc,comVc] = loadHRP2camCom();

    kVc = inv(comVc);
    Zero     = zeros(6,6);
    diagV  = [];
    
    for i = 1:Np;
        L_ligne = [];
        for j=1:Np
        if i==j
           L_ligne = [L_ligne  kVc];
        else
           L_ligne = [L_ligne Zero];
        end
      end 
          diagV = [diagV;L_ligne];
  end
 

endfunction






