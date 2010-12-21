function Camera3DDraw(scale,M)
// cette fonction prepare un objet camera pour
// l'affichage

[xc,yc,zc] = Camera3D(scale,M)

//// x axis
plot3d (xc(1:2), yc(1:2), zc(1:2));
hr1 = gce ();
hr1.thickness=1;
hr1.foreground=5;
//
//// y axis
plot3d (xc(3:4), yc(3:4), zc(3:4));
hr1 = gce ();
hr1.thickness=3;
hr1.foreground=3;

// z axis
plot3d (xc(5:6), yc(5:6), zc(5:6));
hr1 = gce ();
hr1.thickness=5;
hr1.foreground=2;

endfunction

function Camera3DDrawColor(scale,M,col)
// cette fonction prepare un objet camera pour
// l'affichage

[xc,yc,zc] = Camera3D(scale,M)

//// x axis
plot3d (xc(1:2), yc(1:2), zc(1:2));
hr1 = gce ();
hr1.thickness=1;
hr1.foreground=col;
//
//// y axis
plot3d (xc(3:4), yc(3:4), zc(3:4));
hr1 = gce ();
hr1.thickness=2;
hr1.foreground=col;

// z axis
plot3d (xc(5:6), yc(5:6), zc(5:6));
hr1 = gce ();
hr1.thickness=3;
hr1.foreground=col;
endfunction


function CameraPredDraw(scale,U,Te,Np,cMo,col)
  c1McN = ga_predHorPosition(U,Te,Np);
   Camera3DDrawColor(scale,inv(cMo),col);
  for i=1:Np
    c1Mc2    = c1McN(((i-1)*4+1:(i-1)*4+4),:)  ;         // resulting motion
    cMo_aff  = inv(c1Mc2)*cMo;
    Camera3DDrawColor(scale,inv(cMo_aff),col);
  end
endfunction
