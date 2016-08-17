function H=findCatenaryH(R,D)
    // given the semi distance D between two attached points and
    // the semi rope distance , find the sag of the rope H

    function e = fd (h,m)
        e = D-(R^2-h^2)/(2*h)*acosh((h^2+R^2)/(R^2-h^2));
    endfunction

    H0 = sqrt(R^2-D^2);
    [xsol,v]=lsqrsolve(H0,fd,1);

    H = xsol;

endfunction


