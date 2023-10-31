%_________________________________________________________________________%
%  Whale Optimization Algorithm (WOA) source codes demo 1.0               %
%                                                                         %
%  Developed in MATLAB R2011b(7.13)                                       %
%                                                                         %
%  Author and programmer: Seyedali Mirjalili                              %
%                                                                         %
%         e-Mail: ali.mirjalili@gmail.com                                 %
%                 seyedali.mirjalili@griffithuni.edu.au                   %
%                                                                         %
%       Homepage: http://www.alimirjalili.com                             %
%                                                                         %
%   Main paper: S. Mirjalili, A. Lewis                                    %
%               The Whale Optimization Algorithm,                         %
%               Advances in Engineering Software , in press,              %
%               DOI: http://dx.doi.org/10.1016/j.advengsoft.2016.01.008   %
%                                                                         %
%_________________________________________________________________________%


% The Whale Optimization Algorithm
function [Leader_score,Leader_pos,Convergence_curve]=IWOA(x,numAgent,Max_iter,lb,ub,dim,fobj,data)

% initialize position vector and score for the leader
Leader_pos=zeros(1,dim);
Leader_score=inf; %change this to -inf for maximization problems


%Initialize the positions of search agents
Positions=x;

Convergence_curve=zeros(1,Max_iter);
%% Initialisation
for i=1:size(Positions,1)

    % Return back the search agents that go beyond the boundaries of the search space
    Flag4ub=Positions(i,:)>ub;
    Flag4lb=Positions(i,:)<lb;
    temp=rand(1,dim);
    Positions(i,Flag4ub)=temp(Flag4ub);
    Positions(i,Flag4lb)=temp(Flag4lb);
    % Calculate objective function for each search agent
    fitness(i)=fobj(Positions(i,:),[],data);

    % Update the leader
    if fitness(i)<Leader_score % Change this to > for maximization problem
        Leader_score=fitness(i); % Update alpha
        Leader_pos=Positions(i,:);
    end

end
Convergence_curve(1)=Leader_score;
%% chaos theory
x0=rand(1,dim);
% u = 0:0.0001:4;  %Custom steps to change the number of iterations
% xn = ft_logistic(x0,length(u)-1,u);
% figure
% plot(u(2:end),xn,'b.',"linewidth",1.2)
% xlabel('\mu')
% ylabel('Chaos value')
% grid on
for i=1:size(Positions,1)
    x0=4*x0.*(1-x0);
    %x0=(1-x0)/0.3;
    Positions0(i,:)=x0.*(ub-lb)+lb;
    % Calculate objective function for each search agent
    fitness0(i)=fobj(Positions0(i,:),[],data);
    if fitness0(i)<fitness(i)
        fitness(i)=fitness0(i);
        Positions(i,:)=Positions0(i,:);
        % Update the leader
        if fitness(i)<Leader_score % Change this to > for maximization problem
            Leader_score=fitness(i); % Update alpha
            Leader_pos=Positions(i,:);
        end
    end
end
%% reverse search
x0=rand(1,dim);
for i=1:size(Positions,1)
    Positions0(i,:)=ub-x0+lb;
    % Calculate objective function for each search agent
    fitness0(i)=fobj(Positions0(i,:),[],data);
    if fitness0(i)<fitness(i)
        fitness(i)=fitness0(i);
        Positions(i,:)=Positions0(i,:);
        % Update the leader
        if fitness(i)<Leader_score % Change this to > for maximization problem
            Leader_score=fitness(i); % Update alpha
            Leader_pos=Positions(i,:);
        end
    end
end
t=1;% Loop counter

% Main loop
while t<Max_iter

    disp(['CD-WOA,iter:',num2str(t),',minFit:',num2str(Leader_score)])
    fitness0=fitness;
    Nb=0;
    for i=1:size(Positions,1)

        % Return back the search agents that go beyond the boundaries of the search space
        Flag4ub=Positions(i,:)>ub;
        Flag4lb=Positions(i,:)<lb;
        temp=rand(1,dim);
        Positions(i,Flag4ub)=temp(Flag4ub);
        Positions(i,Flag4lb)=temp(Flag4lb);
        % Calculate objective function for each search agent
        fitness(i)=fobj(Positions(i,:),[],data);
        if fitness(i)<fitness0(i)
            Nb=Nb+1;
        end
        % Update the leader
        if fitness(i)<Leader_score % Change this to > for maximization problem
            Leader_score=fitness(i); % Update alpha
            Leader_pos=Positions(i,:);
        end

    end
    IR=Nb/size(Positions,1);
    if IR<0.2
        fp=IR*2.5;
    elseif IR>0.2
        fp=0.5*(1+IR);
    else
        fp=0.5;

    end
    a=2-t*((2)/Max_iter); % a decreases linearly fron 2 to 0 in Eq. (2.3)

    % a2 linearly dicreases from -1 to -2 to calculate t in Eq. (3.12)
    a2=-1+t*((-1)/Max_iter);

    % Update the Position of search agents
    for i=1:size(Positions,1)
        r1=rand(); % r1 is a random number in [0,1]
        r2=rand(); % r2 is a random number in [0,1]

        A=2*a*r1-a;  % Eq. (2.3) in the paper
        C=2*r2;      % Eq. (2.4) in the paper

        
        b=1;               %  parameters in Eq. (2.5)
        l=(a2-1)*rand+1;   %  parameters in Eq. (2.5)

        p = rand();        % p in Eq. (2.6)

        for j=1:size(Positions,2)

            if p<fp
                if abs(A)>=1
                    rand_leader_index = floor(numAgent*rand()+1);
                    X_rand = Positions(rand_leader_index, :);
                    D_X_rand=abs(C*X_rand(j)-Positions(i,j)); % Eq. (2.7)
                    Positions(i,j)=X_rand(j)-A*D_X_rand;      % Eq. (2.8)

                elseif abs(A)<1
                    D_Leader=abs(C*Leader_pos(j)-Positions(i,j)); % Eq. (2.1)
                    Positions(i,j)=Leader_pos(j)-A*D_Leader;      % Eq. (2.2)
                end

            else

                distance2Leader=abs(Leader_pos(j)-Positions(i,j));
                % Eq. (2.5)
                Positions(i,j)=distance2Leader*exp(b.*l).*cos(l.*2*pi)+Leader_pos(j);

            end

        end
    end
    %% differential evolution
    for i=1:size(Positions,1)
        Positions0(i,:)=Positions(i,:)+rand(1,dim).*(Positions(randi(numAgent),:)-Positions(randi(numAgent),:));
        Flag4ub=Positions0(i,:)>ub;
        Flag4lb=Positions0(i,:)<lb;
        temp=rand(1,dim);
        Positions0(i,Flag4ub)=temp(Flag4ub);
        Positions0(i,Flag4lb)=temp(Flag4lb);
        % Calculate objective function for each search agent
        fitness0(i)=fobj(Positions0(i,:),[],data);
        if fitness0(i)<fitness(i)
            fitness(i)=fitness0(i);
            Positions(i,:)=Positions0(i,:);
            % Update the leader
            if fitness(i)<Leader_score % Change this to > for maximization problem
                Leader_score=fitness(i); % Update alpha
                Leader_pos=Positions(i,:);
            end
        end
    end
    t=t+1;
    Convergence_curve(t)=Leader_score;
    %[t Leader_score]
end



