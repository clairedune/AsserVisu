
  //--------------------------------------------------//
  //              LOAD The Files                      //
  //--------------------------------------------------//
  path=get_absolute_file_path("AsserVisu");
  disp('HOME:'+path),
  getd(path + "src/graphisme");           // charge les fonctions d'affichage graphique         
  getd(path + "src/transformation");      // algebre dans Se3, changement de repere
  getd(path + 'src/projectionPers');      // projection perspective   
  getd(path + 'src/asserVisu');           // asservisu classique
  getd(path + 'src/asserVisuPred');       // asservisu predictif
  getd(path + 'src/hrp2');                // hrp2 variable, eg position de la camera/repere CoM 


  


