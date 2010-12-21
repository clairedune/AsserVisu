
// -----------------------------------------//
// build a matrix with A has diagonal
//------------------------------------------//
function bigA = bigDiag(A,n)

 Zer = zeros(size(A,1),size(A,2));
 bigA=[];
 for i = 1:n;
   L_ligne = [];
      for j=1:n
        if i==j
             L_ligne = [L_ligne A];
        else
             L_ligne = [L_ligne Zer];
       end
     end 
    bigA = [bigA;L_ligne];
  end

endfunction





//--------//
//
//--------//
function stateC = convertState(stateA)

stateC = [1 0 0 0 0 0 0 0 0 
               0 0 0 1 0 0 0 0 0
               0 0 0 0 0 0 1 0 0
               0 1 0 0 0 0 0 0 0 
               0 0 0 0 1 0 0 0 0
               0 0 0 0 0 0 0 1 0
               0 0 1 0 0 0 0 0 0
               0 0 0 0 0 1 0 0 0 
               0 0 0 0 0 0 0 0 1
                ] * stateA;

endfunction


function stateC = convertState6(stateA)

stateC = [1 0 0 0 0 0 0 0 0 
          0 0 0 1 0 0 0 0 0
               0 0 0 0 0 0 1 0 0
               0 1 0 0 0 0 0 0 0 
               0 0 0 0 1 0 0 0 0
               0 0 0 0 0 0 0 1 0
               0 0 1 0 0 0 0 0 0
               0 0 0 0 0 1 0 0 0 
               0 0 0 0 0 0 0 0 1
                ] * stateA;

endfunction
