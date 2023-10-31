function x=checkX(x,option,data)
    temp1=x<option.lb;
    x(temp1==1)=option.lb(temp1==1);
    temp2=x>option.ub;
    x(temp2==1)=option.ub(temp2==1);
end