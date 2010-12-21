// ce fichier contient toutes les fonction relative
// a l'affichage graphique d'une camera dans une fenetre

function [xc,yc,zc] = Camera3D(scale,M)
// cette fonction prepare un objet camera pour
// l'affichage
x = [0 scale 0 0     0 0];
y = [0 0     0 scale 0 0];
z = [0 0     0 0     0 scale];
[xc,yc,zc] = changementDeRepere(x,y,z,M)
endfunction
  

