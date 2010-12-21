function V=twistMatrix(M)
V=M;
translation = M(1:3,4);
// skew t
tx = skew(translation);
// extract the translation
Rot = M(1:3,1:3);
// compute the twist matrix
V=[Rot  tx*Rot;zeros(3,3) Rot];
endfunction

