%带软时间窗的车辆路径问题，配送的车辆数给定
%CFZ20160629
%02：28
clear;
clc;
%%问题参数
num_center=200;              %配送中心编号
%%
%读取具体的订单信息，并计算距离矩阵
location=xlsread('Shanghai.xlsx','Sheet1','B2:D592');%点集合
E=xlsread('Shanghai.xlsx','Sheet2','B2:VT592');      %边集合
% seq_isolated_vertex=[18:29 31:33 35:36 38:41 47:48 58:71 127 323 371 416 421 475 539 551 572:574];
% location(seq_isolated_vertex,:)=[];
% E(seq_isolated_vertex,:)=[];
% E(:,seq_isolated_vertex)=[];
E2=xlsread('Shanghai.xlsx','Sheet3','L2:N796');    %内中外环
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
% else
%     disp('图是连通的')
% end
% %生成订单到订单的距离矩阵
% Seq=[num_center;data(:,2)];
% for i=1:length(Seq)
%     for j=1:length(Seq)
%         Dist(i,j)=graphshortestpath(sparse(D),Seq(i),Seq(j));
%     end
% end

%%
%绘图
figure
hold on
plot(location(:, 2), location(:, 3), 'b.', 'MarkerSize', 10);
shift2=0.0003;
% for i=1:size(location,1)
%     text(location(i,2)+shift2,location(i,3)+shift2,num2str(i),'fontsize',8);
% end
plot(location(num_center,2),location(num_center,3),'rp','MarkerSize',16);
for i=1:size(E,1)
    for j=i:size(E,1)
        if E(i,j)==1
            plot([location(i,2) location(j,2)], [location(i,3) location(j,3)], 'k:.', 'LineWidth', 1)
        end
    end
end
for ii=1:size(E2,1)
    if E2(ii,3)==1
        plot([location(E2(ii,1),2) location(E2(ii,2),2)], [location(E2(ii,1),3) location(E2(ii,2),3)], 'r-', 'LineWidth', 1)
    elseif E2(ii,3)==2
        plot([location(E2(ii,1),2) location(E2(ii,2),2)], [location(E2(ii,1),3) location(E2(ii,2),3)], 'y-', 'LineWidth', 1)
    elseif E2(ii,3)==3
        plot([location(E2(ii,1),2) location(E2(ii,2),2)], [location(E2(ii,1),3) location(E2(ii,2),3)], 'g-', 'LineWidth', 1)
%     elseif E2(ii,3)==4
%         plot([location(E2(ii,1),2) location(E2(ii,2),2)], [location(E2(ii,1),3) location(E2(ii,2),3)], 'c-', 'LineWidth', 1)
%     elseif E2(ii,3)==5
%         plot([location(E2(ii,1),2) location(E2(ii,2),2)], [location(E2(ii,1),3) location(E2(ii,2),3)], 'm-', 'LineWidth', 1)
    end
end
xlabel('维度')
ylabel('经度')
title('上海市道路地图')
axis([121.25 121.75 31.05 31.4])
box on
hold off;
