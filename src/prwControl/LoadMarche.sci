function LoadMarche(OPT_RECOMPUTE_MATRICES)
// load source visual servoing
exec('Load.sce')

//Andrei
exec('src/prwControl/prwControl.sci');

if ~isdir('../HuMAnS/LagrangianModel/HRP2') then
  printf("\n");
  printf("!!!\n");
  printf("!!! Be Careful: \n");
  printf("!!! In order to run this sample application, you need to \n");
  printf("!!! be allowed to manipulate the HRP2 model. \n");
  printf("!!! If you are allowed to manipulate the HRP2 model, \n");
  printf("!!! send an email to humans-devel@inrialpes.fr to obtain \n");
  printf("!!! the HRP2 LagrangianModel.\n");
  printf("!!!\n");
  return;
end;



exec('../HuMAnS/KickStart.sci');
execstr(LoadModule('../HuMAnS/Kernel'));
execstr(LoadModule('../HuMAnS/LagrangianModel/HRP2'));
execstr(LoadModule('../HuMAnS/LagrangianDynamics/Complete'));
execstr(LoadModule('../HuMAnS/LagrangianModel/HRP2/TaskFunctionDefinition'));
execstr(LoadModule('../HuMAnS/LagrangianModel/HRP2/JacadiModel'));
execstr(LoadModule('../HuMAnS/ActuationModel/DelayedSampling/ComputedTorqueControl'));
execstr(LoadModule('../HuMAnS/ActuationModel/DelayedSampling'));

if(CPPVERSION) then
  execstr(LoadModule2('../HuMAnS/Kernel'));
  execstr(LoadModule('../HuMAnS/ActuationModel/BipActuators/ActuatorsGeometry'));
  //LoadJacadi();
end


endfunction

