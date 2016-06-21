function [xc,yc,zc] = changementDeRepere(x,y,z,M)
// soit un ensemble de point represente par 3 vecteurs de coordonnees x, y , z
// cette fonction permet d'appliquer le changement de repere relatif a la matrice homogene M 

//disp("Fonction de changement de repere debut")
nbPts = length(x);

//disp("Le nombre de point du vecteur est:")
//disp(nbPts)
// on applique le changement de repere a l'ensemeble des points 
for i = 1:nbPts
        vectC = M*[x(i) y(i) z(i) 1]'  ;
        xc(i) = vectC ( 1 ) ;
        yc(i) = vectC ( 2 ) ; 
        zc(i) = vectC ( 3 ) ;
end;
endfunction


function wX = changeFramePoints(oX,wMo)
//change frame
  N = length(oX)/4;
  wX=[];
  for i=1:N
   wP = wMo*[oX((i-1)*4+1:(i-1)*4+4)];
   wX = [wX wP];
  end
endfunction
