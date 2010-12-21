function vcamOut = satVelo(vcam,iter,dv,vmax,warming,OPT_WARMING)
      
  if(iter<warming&OPT_WARMING)
    vmax = vmax*(iter-1)/warming ;
  end
  v1  = vmax-dv;
  v2  = vmax+dv;
  avcam = abs(vcam);
  
  // for all the coeff
  fac = 1;
  for i=1:length(vmax)
   
    fac = min(abs(fac),vmax(i)/(avcam(i)+%eps));
    
    // to prevent from discontinuities
    if( (v1(i)<=avcam(i)) & (avcam(i) <= v2(i)) )
      nout = 1/(2*dv*avcam(i))  *  ( (avcam(i)-v1(i))*vmax(i)+(v2(i)-avcam(i))*v1(i));
      fac  = min(abs(fac),abs(nout));
    end
    
  end
  vcamOut = vcam*fac;
endfunction
  
