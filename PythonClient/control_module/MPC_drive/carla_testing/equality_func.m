function error = equality_func(s_prime, s, u)
%CEQ Summary of this function goes here
%   Detailed explanation goes here

    s_prime_correct = step(s,u);
    error = sum(abs(s_prime_correct - s_prime));

end

