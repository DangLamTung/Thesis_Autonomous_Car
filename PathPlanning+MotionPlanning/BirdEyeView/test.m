h = [[eye(3) zeros(3,6)]]
x = [0.5 0 0]
m = [1 1 1]
p = eye(9)
r = eye(3)*10
i = 0
while i <100
   ht = transpose(h)
   
   hp = h*p
   
   hpht = hp*ht
   s = hpht + r
   
   ph = p*ht
   sinv = inv(s)
   
   k = ph*sinv
   %k = p*transpose(h)/(h*p*transpose(h) +  r)
   dx = k*transpose(m - x)
   x = x + transpose(dx(1:3))
   p = (eye(9) - k*h)*p
   i = i+1
end