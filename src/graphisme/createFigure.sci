function drawTurtleTop(x,y,s)
    plot(x,y,'.k');
    xarc(x-(s/2),y+(s/2),s,s,0,360*64);
endfunction

function drawWheeledTurtleTop(wMr,s)
    pose_wMr = pFromHomogeneousMatrix(wMr);
    x        = pose_wMr(1);
    y        = pose_wMr(2);

    plot(x,y,'.k');
    xarc(x-(s/2),y+(s/2),s,s,0,360*64);

    // direction du turtle
    pose_rMd = [s/2*1.2,0,0,0,0,0];
    rMd      = homogeneousMatrixFromPos(pose_rMd);
    wMd      = wMr*rMd;
    xd       = wMd(1,4);
    yd       = wMd(2,4);
    
    nx = [x xd];
    ny = [y yd];
    xarrows(nx, ny) ;
    
    // centre de la roue gauche
    pose_rMg = [0,-s/4,0,0,0,0];
    rMg      = homogeneousMatrixFromPos(pose_rMg);
    wMg      = wMr*rMg;

    // centre de la roue droite
    pose_rMd = [0,+s/4,0,0,0,0];
    rMd      = homogeneousMatrixFromPos(pose_rMd);
    wMd      = wMr*rMd;
    
    // dessin de la roue
    oP       = [0.1 0.11 0.1 -0.1 -0.11 -0.1 0.1
               -0.04 0 0.04 0.04 0 -0.04  -0.04
               0 0 0 0 0 0 0
               1 1 1 1 1 1 1];
               
    wPg      = changeFramePoints(oP,wMg);
    wPd      = changeFramePoints(oP,wMd);    
               
    plot(wPg(1,:),wPg(2,:),'k')  ; 
    plot(wPd(1,:),wPd(2,:),'k')  ; 
               
endfunction


function drawTurtleSide(x,z,s)
////    plot(x,z,'.k');
    xrect(x-(s/2),z+(s/2),s,s);
endfunction


function hf = createFigure3D(number,titlefig,scale)
  xset("window",number);
  xset("pixmap",1);
  xbasc()//effacement de la fenêtre
  clear_pixmap()//et buffer
  hf=scf(number);
  hf.figure_name = titlefig;
  ha=hf.children;
  ha.box="on"; 
  ha.view="3d";
  ha.thickness=1;
  ha.foreground=0;
  axe=gca(); // recupere un pointeur sur les axes
  axe.x_label.text="x"; // texte des label
  axe.y_label.text="y"; // y label
  axe.z_label.text="z"; // z label
  //axe.data_bounds=[-scale,-scale,-scale;scale,scale,scale]; // set the boundary values for the x y and z axis 
  axe.isoview="off"; // empeche le changement d'echelle
  axe.grid =[1 1 1];
  axe.auto_clear = "off" ;
  axe.data_bounds=[[-2,2]'; [-2,2]';[0,1]'];
endfunction


function hf = createPlanImage(number,xlimit,ylimit,titlefig)
  xset("window",number);
  xset("pixmap",1);
  //xbasc()//effacement de la fenêtre
  clear_pixmap()//et buffer
  hf=scf(number);
  hf.figure_name = titlefig;
  ha=hf.children;
  ha.box="on"; 
  ha.view="2d";
  ha.thickness=1;
  ha.foreground=0;
  axe=gca(); // recupere un pointeur sur les axes
  axe.data_bounds=[xlimit'; ylimit'];
  axe.grid =[0.1 0.1 -0.1];
  axe.auto_clear = "off" ;
endfunction

function hf = createFigure2D(number,titlefig)
  xset("window",number);
  xset("pixmap",1);
  xbasc()//effacement de la fenêtre
  clear_pixmap()//et buffer
  hf=scf(number);
  hf.figure_name = titlefig;
  ha=hf.children;
  ha.box="on"; 
  ha.view="2d";
  ha.thickness=1;
  ha.foreground=0;
  axe=gca(); // recupere un pointeur sur les axes
  auto_resize = "on" 
  //axe.data_bounds=[0,0;10,2];
  axe.grid =[1 1 -1];
  axe.auto_clear = "off" ;
endfunction
