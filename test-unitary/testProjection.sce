x = [2 2]
y = [1 2] 


// projection de x sur y
xy = x*y'
vectT = xy * y/(norm(y))^2
vectN = x-vectT

test = vectN*y'

