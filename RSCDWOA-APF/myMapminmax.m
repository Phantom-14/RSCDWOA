function X0=myMapminmax(X0)
    X0(~isinf(X0))=(X0(~isinf(X0))-min(min(X0(~isinf(X0)))))/(max(max(X0(~isinf(X0))))-min(min(X0(~isinf(X0))))+eps);
end