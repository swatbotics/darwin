names = {'x' ; 'y'}
exprs = [x; y]

for i=1:2
  svar = sprintf('tmp%03d', i);
  [exprs sexpr] = subexpr(exprs, svar);
  if isempty(sexpr)
    break
  end
  names{end+1} = svar;
  exprs = simplify(exprs);
  exprs(end+1) = sexpr;
end
