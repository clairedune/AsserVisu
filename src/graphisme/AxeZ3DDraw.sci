function AxeZ3DDraw(scale,M)
// cette fonction prepare un objet camera pour
// l'affichage

[xc,yc,zc] = Camera3D(scale,M)



// z axis
plot3d (xc(5:6), yc(5:6), zc(5:6));
hr1 = gce ();
hr1.foreground=2;

endfunction
