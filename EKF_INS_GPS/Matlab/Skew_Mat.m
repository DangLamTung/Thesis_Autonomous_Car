function skew = Skew_Mat(v)
% Calculate the following quaternion product quatA * quatB using the 
% standard identity 

skew = [[0, -v(3), v(2)];
        [v(3), 0, -v(1)];
        [-v(2), v(1), 0]];