function [bestY,bestX,recording]=myIWOA(x,y,option,data)
    %% WOA
    SearchAgents_no=option.numAgent;
    Max_iter=option.maxIteration;
    lb=option.lb;
    ub=option.ub;
    dim=option.dim;
    fobj=option.fobj;
    [Leader_score,Leader_pos,Convergence_curve]=IWOA(x,SearchAgents_no,Max_iter,lb,ub,dim,fobj,data);
    %% initialization
    recording.bestFit=[min(y),Convergence_curve]';
    recording.meanFit=zeros(option.maxIteration+1,1);
    bestY=Leader_score;
    bestX=Leader_pos;
end