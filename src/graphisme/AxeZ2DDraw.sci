function AxeZ2DDraw(scale,M)
// cette fonction prepare un objet camera pour
// l'affichage

[xc,yc,zc] = Camera3D(scale,M)

// z axis
plot2d (xc(5:6), yc(5:6),5);
// y axis
plot2d (xc(3:4), yc(3:4),3);
hr1 = gce ();
hr1.foreground=2;
endfunction
