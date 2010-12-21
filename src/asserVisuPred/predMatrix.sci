// In this file are store the function to
// build the PredictionMatrix


//
// This function build the matrices S and U for the CoM
//
function [Sp, Sv, Sa, Up, Uv, Ua] = buildC(N_in,Te_in)

 for i = 1:N_in
   Sp(i, :) = [1, i*Te_in, i^2*Te_in^2/2];
   Sv(i, :) = [0, 1, i*Te_in];
   Sa(i, :) = [0, 0, 1];
   for j = 1:N_in
     if j>i 
       Up(i, j) = 0;
       Uv(i, j) = 0;
       Ua(i, j) = 0;
     else
       Up(i, j) = (3*(i-j)^2+3*(i-j)+1)*Te_in^3/6;
       Uv(i, j) = (2*(i-j)+1)*Te_in^2/2;
       Ua(i, j) = Te_in;
     end
   end
  end
endfunction


//
// This function build the Sz and Uz matrix for the ZMP
//
function [Sz,Uz] = buildZ(Tend_in,Te_in, N_in, G_in, cz_in, ActuationSamplingPeriod)

  for n = 1:Tend_in/Te_in
    for i = 1:N_in
      Sz(i, :,n) = [1, i*Te_in, i^2*Te_in^2/2+(cz_in+CoMHeightVar((n-1)*...
      Te_in/ActuationSamplingPeriod+1))/G_in];
      for j = 1:N_in
        if j>i 
	   Uz(i, j,n) = 0;
        else
	   Uz(i, j,n) = (3*(i-j)^2+3*(i-j)+1)*Te_in^3/6+...
                        Te_in*(cz_in+CoMHeightVar((n-1+j)*...
                        Te_in/ActuationSamplingPeriod+1))/G_in;
        end
      end
    end
  end
endfunction

//
// Test loaded
//

function isPredMatrixLoaded()
     disp('Yes, predMatrix is loaded');
endfunction
