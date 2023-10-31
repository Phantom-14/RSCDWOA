function [fit,result,x0]=aimFcn_1(x,option,data)
x0=x;
for i=1:data.numUAV
    xUAV{i}=zeros(data.mapsize);
    [p1,p2]=find(xUAV{i}==0);
    index=sub2ind(data.mapsize,p1,p2);
    temp=x(1:prod(data.mapsize));
    x(1:prod(data.mapsize))=[];
    xUAV{i}(index)=temp;
end
Rep_S=data.Rep_S;
Uatt=data.Uatt;
nowP=data.UAV_S;
direction=getDirection(data.R);
direction=[direction;0,0];
numR=length(direction(:,1));
path=cell(data.numUAV,1);
map=myMapminmax(Rep_S);
for noUAV=1:data.numUAV
    map=map+myMapminmax(Uatt{noUAV});
end
for noUAV=1:data.numUAV

    H=map(nowP(noUAV,1),nowP(noUAV,2));
    path{noUAV}=[nowP(noUAV,:),H];
end

%%
%figure
data.Ob_M0=data.Ob_M;
flag=zeros(data.numUAV,1);
jishu=1;
while 1
    jishu=jishu+1;
    if jishu>200
        break;
    end
    if ~isempty(data.Ob_M)
        for i=1:length(data.Ob_M(:,1))
            if data.Ob_M(i,6)==0
                data.Ob_M(i,1:2)=data.Ob_M(i,1:2)+100*data.Ob_M(i,3:4)/1000;
            else
                data.Ob_M(i,1:2)=data.Ob_M(i,1:2)-100*data.Ob_M(i,3:4)/1000;
            end
            if norm(data.Ob_M(i,1:2)-data.Ob_M(i,8:9))<eps
                if data.Ob_M(i,6)==0
                    data.Ob_M(i,6)=1;
                    data.Ob_M(i,8:9)=data.Ob_M0(i,1:2);
                else
                    data.Ob_M(i,6)=0;
                    data.Ob_M(i,8:9)=data.Ob_M0(i,8:9);
                end
            end
        end
    end
    for noUAV=1:data.numUAV
        nowP0=nowP(noUAV,:);
        if norm(nowP0-data.UAV_E(noUAV,:))<eps
            flag(noUAV)=1;
            continue;
        end
        nextP0=repmat(nowP0(1:2),numR,1)+direction;
        nextP0=[nextP0,repmat(nowP0(3),numR,1)];
        %% Map range constraint
        index=[];
        for i=1:length(nextP0(:,1))
            for j=1:2
                if nextP0(i,j)<=0 || nextP0(i,j)>data.mapsize(j)
                    index=[index;i];
                end
            end
        end
        nextP0(index,:)=[];
        %% UAV collision constraint
        index=[];
        for i=1:length(nextP0(:,1))
            for noUAV0=1:data.numUAV
                if noUAV0~=noUAV
                    gap=abs(nextP0(i,:)-nowP(noUAV0,:));
                    [gap,no]=max(gap);
                    if gap<ceil(data.minGap(no)/1000) && flag(noUAV0)~=1
                        index=[index;i];
                    end
                end
            end
        end
        nextP0(index,:)=[];
        if isempty(nextP0)
            continue;
        end
        %% Distance to dynamic obstacles
        if ~isempty(data.Ob_M)
            D=pdist2(data.Ob_M(:,1:2),nextP0(:,1:2));
            for i=1:length(data.Ob_M(:,1))
                position=find(D(i,:)>data.Ob_M(i,7)*2);
                D(i,position)=0;
            end
            D=1./D;
            D(isinf(D))=0;
            D=sum(D);
            D=(D-min(min(D)))/(max(max(D))-min(min(D))+eps);
        else
            D=0;
        end
        %%
        index=sub2ind(data.mapsize,nextP0(:,1),nextP0(:,2));
        Pri=myMapminmax(xUAV{noUAV});

        Rep_M=Rep_S*0;
        if ~isempty(data.Ob_M)
        for i=1:length(data.Ob_M(:,1))
            x0=data.Ob_M(i,1);
            y0=data.Ob_M(i,2);
            R=data.Ob_M(i,7)*10;
            for j=1:R*2+1
                for k=1:R*2+1
                    x=round(x0-R+j-1);
                    y=round(y0-R+k-1);
                    R0=norm([x,y]-[x0,y0]);
                    H=10*(1/(R0+eps)-1/R)^0.1+2;
                    if x>0 && x<data.mapsize(1) && y>0 && y<data.mapsize(2)
                        if R0<R
                            Rep_M(x,y)=max(Rep_M(x,y),H);
                        end
                    end
                end
            end
        end
        end
        if max(D)>0
            a=1;
        end
        F=myMapminmax(Rep_S)+myMapminmax(Uatt{noUAV})+10*myMapminmax(Rep_M);
        %mesh(F)
        F=F(index)+0.002*Pri(index);
        [~,no]=min(F);
        nowP(noUAV,:)=nextP0(no,:);
        H=map(nextP0(no,1),nextP0(no,2));
        path{noUAV}=[path{noUAV};nowP(noUAV,:),H+0.1];
        if norm(nowP(noUAV,:)-data.UAV_E(noUAV,:))<eps
            flag(noUAV)=1;
            continue;
        end
    end
%     hold off
%     for noUAV=1:data.numUAV
%         plot3(path{noUAV}(:,1),path{noUAV}(:,2),path{noUAV}(:,4),'-','LineWidth',2)
%         hold on
%     end

%     mesh(map')
%     for i=1:length(data.Ob_M(:,1))
%         [x,y,z]=sphere();
%         
%         R=data.Ob_M(i,7);
%         H=map(round(data.Ob_M(i,1)),round(data.Ob_M(i,2)))+R;
%         surf(data.Ob_M(i,1)+5*R*x,data.Ob_M(i,2)+5*R*y,H+R*z)
%     end
%     title('Potential Force')
%     xlabel('km')
%     ylabel('km')
%     zlabel('km')
% 
% 
% 
%     %axis equal
%     view(80,70);
%     pause(0.1)
    if data.numUAV==sum( flag)
        break;
    end
end
D=0;
for noUAV=1:data.numUAV
    D(noUAV)=0;
    for i=1:length(path{noUAV}(:,1))-1
        D(noUAV)=D(noUAV)+norm(path{noUAV}(i,:)-path{noUAV}(i+1,:));
    end
    numN(noUAV)=length(path{noUAV}(:,1));
end
if jishu>200
    fit=sum(D)+jishu+sum(numN);
else
    fit=sum(D)+sum(numN);
end
if nargout>1
    result.fit=fit;    %target
    result.path=path;   %path
end
end