function vout=vRxRyRzFromVThetaU(vin)
// auteur Claire Dune
// Date  11 2009 
// Description : permet de passer d'un vecteur
// vitesse dont les rotations sont exprimees en utheta
// a un vecteur vitesse avec les rotations rxryrz 
// prend en entree un vecteur ligne

// copy des translation
vout1 = vin(1:3);
vout2 = RxRyRzFromThetaU(vin(4:6));
vout = [vout1 vout2];
endfunction
