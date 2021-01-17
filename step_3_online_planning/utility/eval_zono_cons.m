function [c, ceq] = eval_zono_cons(x, A_con, b_con)
    epsilon = 1e-3;
    ceq = [];

    c = A_con*x + b_con;
    c = reshape(c, 6, []);
    c = min(c, [], 1) + epsilon;
end