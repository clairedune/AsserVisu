clear
;exec('Load.sce');
OPT_RECOMPUTE_MATRICES = %T;
;exec('LoadMarche.sce');

function start(OPT,OPT_CORR,OPT_CENTER,OPT_DISTURB,pathExp)
  getd('main/vsWalk');
  isTestSwayLoaded()
  if(OPT=="PG")
    disp('You enter the PG test')
    OPT_RECOMPUTE_MATRICES = %F;
    ;exec('LoadMarche.sce');
    testPG(pathExp);
    
  elseif(OPT=="2TASKS")
    disp("You are testing the two tasks algorithm")
    OPT_RECOMPUTE_MATRICES = %F;
    ;exec('LoadMarche.sce');
    testSwayCorr3D2Tasks(OPT_CORR,OPT_CENTER,OPT_DISTURB,pathExp);
elseif (OPT=="3D")
    disp('You enterthe 3D test')
     OPT_RECOMPUTE_MATRICES = %F;
    ;exec('LoadMarche.sce');
    testSwayCorrection3D(OPT_CORR,OPT_DISTURB,pathExp)
  elseif (OPT=="PGCAM")
    disp('You enter the PG CAMtest')
    OPT_RECOMPUTE_MATRICES = %F;
    ;exec('LoadMarche.sce');
    testPGCam(pathExp);    
elseif (OPT=="HRP2")
   disp('You enter the HRP2test')
    OPT_RECOMPUTE_MATRICES = %F;
    ;exec('LoadMarche.sce');
    testSwayCorrection(OPT_CORR,OPT_DISTURB,pathExp);
elseif( OPT =="3ddl")
   disp('You enter the 3ddl')
    testSway3ddl(OPT_CORR,%T);
elseif( OPT =="6ddl")
   disp('You enter the 6ddl')
    testSway(OPT_CORR,%T);
elseif( OPT =="Robot")
   disp('You enter the 6ddl test')
    testSwayRobot(OPT_CORR,%T,pathExp);   
  end

endfunction
