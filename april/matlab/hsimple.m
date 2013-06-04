clear
reset(symengine)

syms x1 y1 x2 y2 x3 y3 x4 y4 real
%syms u1 v1 u2 v2 u3 v3 u4 v4 real

u1 = 0; v1 = 0;
u2 = 1; v2 = 0;
u3 = 1; v3 = 1;
u4 = 0; v4 = 1;

A = [ 
    u1 v1  1  0  0  0  -x1*u1  -x1*v1 
     0  0  0 u1 v1  1  -y1*u1  -y1*v1
    u2 v2  1  0  0  0  -x2*u2  -x2*v2 
     0  0  0 u2 v2  1  -y2*u2  -y2*v2
    u3 v3  1  0  0  0  -x3*u3  -x3*v3 
     0  0  0 u3 v3  1  -y3*u3  -y3*v3
    u4 v4  1  0  0  0  -x4*u4  -x4*v4 
     0  0  0 u4 v4  1  -y4*u4  -y4*v4
    ]

b = [
    x1
    y1
    x2
    y2
    x3
    y3
    x4
    y4
    ]

syms h1 h2 h3 h4 h5 h6 h7 h8

hh = [ h1 ; h2 ; h3 ; h4 ; h5 ; h6 ; h7 ; h8 ]

S = solve(A*hh-b, 'h1', 'h2', 'h3', 'h4', 'h5', 'h6', 'h7', 'h8')

h = [ S.h1 ; S.h2 ; S.h3 ; S.h4 ; S.h5 ; S.h6 ; S.h7 ; S.h8 ]

syms u v real

w = simplify( (S.h7 * u + S.h8 * v + 1) );
x = simplify( (S.h1 * u + S.h2 * v + S.h3) / w )
y = simplify( (S.h4 * u + S.h5 * v + S.h6) / w )

outvars = [ x; y ];
invars = [ x1; y1; x2; y2; x3; y3; x4; y4 ];

J = sym(zeros(2, 8));

for i=1:2
  for j=1:8
    J(i,j) = simplify(diff(outvars(i), invars(j)));
  end
end

save hsimple.mat

