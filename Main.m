%带软时间窗的车辆路径问题，配送的车辆数给定
%CFZ20160629
%02：28
clear;
clc;
%%
%读取具体的订单信息，并计算距离矩阵
location=xlsread('testdata3.xlsx','Sheet1','B2:D592');%点集合
E=xlsread('testdata3.xlsx','Sheet2','B2:VT592');      %边集合
data=xlsread('testdata3.xlsx','Sheet3','A2:J13');    %订单信息
%%
%问题参数
num_center=408;              %配送重点编号
num_car=3;                  %车辆数
speed=50;                 	%车速
num_order=size(data,1); 	%订单数
max_load=6;                 %每辆车的载重上限
w_dist=1;                   %距离权重
w_overweight=1000000;       %超重的惩罚值
w_earlytime=2;            	%早到的惩罚值
w_latetime=1000000;      	%迟到的惩罚值
%%
%算法参数
maxgen=500;             	%迭代次数上限
stopgen=40;                 %最优解最少保持代数
sizepop=80;                    %个体数目
GGAP=0.9;                   %选择概率
pmutation=0.2;              %变异率
%%
%生成点到点的距离矩阵，无边直接相连赋值为0，点到自身赋值为0
D=zeros(size(location,1));
for i=1:size(location,1)
    for j=i:size(location,1)
        if E(i,j)==1
            D(i,j)=((location(i,2)-location(j,2))^2+(location(i,3)-location(j,3))^2)^0.5; %计算存在边相连的两点之间的距离
        else
            D(i,j)=NaN; %两点之间无边直接相连，距离记为NaN
        end
    end
end
D(isnan(D))=0;          %将矩阵中的NaN变为0
D=D+D';                 %将上三角矩阵变成对称矩阵
% %验证道路数据是否连通
% for i=1:size(location,1)
%     Dist0(i,:)=graphshortestpath(sparse(D),i);
% end
% [aa,bb]=find(Dist0==Inf);
% cccc=[aa bb];%记录不连通的两点
% if ~isempty(cccc)
%     error('存在不连通图')
% end
% 生成订单到订单的距离矩阵
Seq=[num_center;data(:,2)];
for i=1:length(Seq)
    for j=1:length(Seq)
        Dist(i,j)=graphshortestpath(sparse(D),Seq(i),Seq(j));
    end
end
%%
%生成初始解
for i=1:sizepop
    Chrom2(i,:)=randperm(num_order);
    zero_location=randperm(num_order-1);
    zero_location=zero_location(1:num_car-1);
    Chrom(i,:)=insertzeros(Chrom2(i,:),zero_location);
end
Chrom=[zeros(sizepop,1) Chrom zeros(sizepop,1)];%初始解
ObjV=Objectfunction(Chrom,data,Dist,speed,num_order,max_load,w_dist,w_overweight,w_earlytime,w_latetime);%计算初始解的目标函数值
[bestObjV,bestindex]=max(ObjV);
bestsol=Chrom(bestindex,:);
%%
%循环开始
gen=1;  %初始遗传代数
gen0=0; %初始保持代数   
while gen0<stopgen && gen<=maxgen
    disp(gen)
    FitnV=ranking(-ObjV);                                   %分配适应度值(Assign fitness values)
    SelCh=select('sus', Chrom, FitnV, GGAP);                %选择
    SelCh=cross(SelCh,num_car+1,floor(sizepop*GGAP/2)*2);      %重组
    SelCh=mut(SelCh,pmutation,floor(sizepop*GGAP/2)*2);        %变异
    ObjVSel=Objectfunction(SelCh,data,Dist,speed,num_order,max_load,w_dist,w_overweight,w_earlytime,w_latetime);           %计算子代目标函数值
    [Chrom,ObjV]=reins(Chrom, SelCh, 1, 1, ObjV, ObjVSel);	%重插入
    [newbestObjV,newbestindex]=max(ObjV);
    [worestObjV,worestindex]=min(ObjV);
    if newbestObjV>bestObjV
        bestObjV=newbestObjV;
        bestsol=Chrom(newbestindex,:);
        gen0=0;
    else
        gen0=gen0+1;                    %最优值保持次数加1
    end
    Chrom(worestindex,:)=Chrom(newbestindex,:);
    ObjV(worestindex,:)=newbestObjV;
    trace(gen,1)=1/bestObjV;            %记录每一代进化中最好的适应度
    trace(gen,2)=sum(1./ObjV)/sizepop;  %记录每一代进化中平均适应度
    gen=gen+1;                          %遗传代数加1
end
gen=gen-1;
%计算每个车辆的线路
bestzero_location=find(bestsol==0);     %寻找最优解中0的位置
for i=1:num_car
    Seq_order{1,i}=bestsol(bestzero_location(i):bestzero_location(i+1));%每个车辆配送的订单编号顺序
	circle{1,i}=Seq(Seq_order{1,i}+1)'; %订单位置顺序
    route{1,i}=circle{1,i}(1);          %每个车辆配送的订单位置顺序
    for j=1:length(circle{1,i})-1
        [~,temp_route]=graphshortestpath(sparse(D),circle{1,i}(j),circle{1,i}(j+1));
        route{1,i}=[route{1,i} temp_route(2:end)];%每个车辆的行驶线路
    end
end
%%
%结果输出
disp('---------------------------------------------------------------------')
disp('最优订单顺序：')
disp(bestsol)
for i=1:num_car
    disp(['车辆',num2str(i),'配送的订单顺序：'])
    disp(Seq_order{1,i})
    disp(['车辆',num2str(i),'的配送路线：'])
    disp(route{1,i})
end
disp('最优值：')
disp(1/bestObjV)
%%
%绘图
%图1：优化过程图
figure
plot(1:gen,1./trace(:,1));
plot(1:gen,trace(:,1),'r-',1:gen,trace(:,2),'b--');
title(['目标函数优化曲线  ' '终止代数＝' num2str(gen)],'fontsize',12);
xlabel('进化代数','fontsize',12);
ylabel('目标函数值','fontsize',12);
legend('各代最佳值','各代平均值');
xlim([1,gen])
%图2：全部车辆的配送路线图
figure
hold on
plot(location(:, 2), location(:, 3), 'r.', 'MarkerSize', 10);
shift2=0.0003;
for i=1:size(location,1)
    text(location(i,2)+shift2,location(i,3)+shift2,num2str(i),'fontsize',8);
end
plot(location(num_center,2),location(num_center,3),'rp','MarkerSize',20);
for i=1:size(E,1)
    for j=i:size(E,1)
        if E(i,j)==1
            plot([location(i,2) location(j,2)], [location(i,3) location(j,3)], 'k:.', 'LineWidth', 1)
        end
    end
end
colour=linspecer(num_car);
shift=0.0004;%坐标位移量
for i=1:num_car
    LL=[];
    LL=location(route{1,i},2:3);
    for j=2:length(route{1,i})
        quiver(LL(j-1,1),LL(j-1,2)+(i-1)*shift,(LL(j,1)-LL(j-1,1))/0.9,(LL(j,2)-LL(j-1,2))/0.9,'color',colour(i,:),'linewidth',1.5);
%         [arrowx,arrowy] = dsxy2figxy(gca,LL(j-1:j,1)+(i-1)*shift,LL(j-1:j,2)+(i-1)*shift);%坐标转换
%         annotation('textarrow',arrowx,arrowy,'HeadWidth',9,'color',colour(i,:),'linewidth',1.5);
    end
end
xlabel('横坐标')
ylabel('纵坐标')
title('全部车辆的配送路线图')
box on
hold off;
% %图3：每个车辆的配送路线图
% % colour=linspecer(num_car);
% for i=1:num_car
%     figure
%     hold on
%     plot(location(:, 2), location(:, 3), 'r.', 'MarkerSize', 20);
%     for k=1:size(location,1)
%         text(location(k,2)+0.8,location(k,3)+0.8,num2str(k),'color',[1,0,0]);
%     end
%     plot(location(num_center,2),location(num_center,3),'rp','MarkerSize',20)
%     for ii=1:size(E,1)
%         for jj=ii:size(E,1)
%             if E(ii,jj)==1
%                 plot([location(ii,2) location(jj,2)], [location(ii,3) location(jj,3)], 'k:.', 'LineWidth', 1)
%             end
%         end
%     end
%     shift=0;%坐标位移量
%     LL=[];
%     LL=location(route{1,i},2:3);
%     for j=2:length(route{1,i})
%         quiver(LL(j-1,1),LL(j-1,2),(LL(j,1)-LL(j-1,1))/0.9,(LL(j,2)-LL(j-1,2))/0.9,'color',colour(i,:),'linewidth',1.5);
% %         [arrowx,arrowy] = dsxy2figxy(gca,LL(j-1:j,1)+(i-1)*shift,LL(j-1:j,2)+(i-1)*shift);%坐标转换
% %         annotation('textarrow',arrowx,arrowy,'HeadWidth',9,'color',colour(i,:),'linewidth',1.5);
%     end
%     xlabel('横坐标')
%     ylabel('纵坐标')
%     title(['车辆',num2str(i),'的配送路线图'])
%     box on
%     hold off;
% end