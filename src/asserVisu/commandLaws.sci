function vc = computeVelocity(lambda, Lin,e)
  vc = - lambda *pinv(Lin)*e;//* inv(L'*L)*L'*e,
  vc = vc';
endfunction

// compute an adaptative gain
function lambda = computeGA(lambdaI,lambdaF,betaGA,e)
  lambda = lambdaI+lambdaF*exp(-betaGA*norm(e)) ;
endfunction

function isCommandLawLoaded()
disp('commandLaw is loaded')
endfunction
