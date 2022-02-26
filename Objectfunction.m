function obj=Objectfunction(X,data,Dist,speed,num_order,max_load,w_dist,M_overweight,MT_early,MT_late)
%求距离最小，超重和超时惩罚

for i=1:size(X,1)
    Time=zeros(num_order,1);
    sol=X(i,:);
    temp=find(sol==0);
    for j=1:length(temp)-1
        circle=[];
        circle=sol(temp(j):temp(j+1));
        dist(j)=0;
        for k=1:length(circle)-1
            dist(j)=Dist(circle(k)+1,circle(k+1)+1)+dist(j);
        end
        load(j)=0;
        T=0;
        for kk=2:length(circle)-1
            load(j)=sum(data(circle(kk),5:7))+load(j);
            T=T+Dist(circle(kk-1)+1,circle(kk)+1)/speed;
            Time(circle(kk),1)=T;
            T=T+2;
        end
    end
    obj(i,1)=1/(w_dist*sum(dist)+M_overweight*sum(max(load-max_load,0))+MT_early*sum(max(data(:,3)-Time,0))+MT_late*sum(max(Time-data(:,4),0)));
end
end