function vout=vThetaUFromVRxRyRz(vin)
// auteur Claire Dune
// Date  11 2009 
// Description : permet de passer d'un vecteur
// vitesse dont les rotations sont exprimees en rxryrz 
// a un vecteur vitesse avec les rotations utheta

// copy des translation
vout1 = vin(1:3);
vout2 = thetaUFromRxRyRz(vin(4:6));
vout = [vout1';vout2]'; 
endfunction
