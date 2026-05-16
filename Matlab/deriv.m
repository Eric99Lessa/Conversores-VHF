function out = deriv(f, g) 
    % Função para derivada parcial de f em relacao a g=g(t)
    syms t x dx
    a = {g, diff(g, t)}; 
    b = {x, dx};
    f1 = subs(f, a, b);
    f2 = diff(f1, x);
    out = subs(f2, b, a);
end