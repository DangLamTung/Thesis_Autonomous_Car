function quatOut = QuatMult(q,p)
% Calculate the following quaternion product quatA * quatB using the 
% standard identity 
q_l = [[q(1) -q(2) -q(3) -q(4)]
       [q(2) q(1) -q(4) q(3)]
       [q(3) q(4) q(1) -q(2)]
       [q(4) -q(3) q(2) q(1)]];


quatOut = q_l*transpose(p);