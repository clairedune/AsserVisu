function Tx = skew(t)
// this function compute the skew Matrix of a translation
// vector
Tx = [0 -t(3) t(2)
      t(3) 0 -t(1)
      -t(2) t(1) 0 ];
  
endfunction
