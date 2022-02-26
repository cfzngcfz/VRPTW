%����ʱ�䴰�ĳ���·�����⣬���͵ĳ���������
%CFZ20160629
%02��28
clear;
clc;
%%�������
num_center=200;              %�������ı��
%%
%��ȡ����Ķ�����Ϣ��������������
location=xlsread('Shanghai.xlsx','Sheet1','B2:D592');%�㼯��
E=xlsread('Shanghai.xlsx','Sheet2','B2:VT592');      %�߼���
% seq_isolated_vertex=[18:29 31:33 35:36 38:41 47:48 58:71 127 323 371 416 421 475 539 551 572:574];
% location(seq_isolated_vertex,:)=[];
% E(seq_isolated_vertex,:)=[];
% E(:,seq_isolated_vertex)=[];
E2=xlsread('Shanghai.xlsx','Sheet3','L2:N796');    %�����⻷
%%
%���ɵ㵽��ľ�������ޱ�ֱ��������ֵΪ0���㵽����ֵΪ0
D=zeros(size(location,1));
for i=1:size(location,1)
    for j=i:size(location,1)
        if E(i,j)==1
            D(i,j)=((location(i,2)-location(j,2))^2+(location(i,3)-location(j,3))^2)^0.5; %������ڱ�����������֮��ľ���
        else
            D(i,j)=NaN; %����֮���ޱ�ֱ�������������ΪNaN
        end
    end
end
D(isnan(D))=0;          %�������е�NaN��Ϊ0
D=D+D';                 %�������Ǿ����ɶԳƾ���
% %��֤��·�����Ƿ���ͨ
% for i=1:size(location,1)
%     Dist0(i,:)=graphshortestpath(sparse(D),i);
% end
% [aa,bb]=find(Dist0==Inf);
% cccc=[aa bb];%��¼����ͨ������
% if ~isempty(cccc)
%     error('���ڲ���ͨͼ')
% else
%     disp('ͼ����ͨ��')
% end
% %���ɶ����������ľ������
% Seq=[num_center;data(:,2)];
% for i=1:length(Seq)
%     for j=1:length(Seq)
%         Dist(i,j)=graphshortestpath(sparse(D),Seq(i),Seq(j));
%     end
% end

%%
%��ͼ
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
xlabel('ά��')
ylabel('����')
title('�Ϻ��е�·��ͼ')
axis([121.25 121.75 31.05 31.4])
box on
hold off;
