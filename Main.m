%����ʱ�䴰�ĳ���·�����⣬���͵ĳ���������
%CFZ20160629
%02��28
clear;
clc;
%%
%��ȡ����Ķ�����Ϣ��������������
location=xlsread('testdata3.xlsx','Sheet1','B2:D592');%�㼯��
E=xlsread('testdata3.xlsx','Sheet2','B2:VT592');      %�߼���
data=xlsread('testdata3.xlsx','Sheet3','A2:J13');    %������Ϣ
%%
%�������
num_center=408;              %�����ص���
num_car=3;                  %������
speed=50;                 	%����
num_order=size(data,1); 	%������
max_load=6;                 %ÿ��������������
w_dist=1;                   %����Ȩ��
w_overweight=1000000;       %���صĳͷ�ֵ
w_earlytime=2;            	%�絽�ĳͷ�ֵ
w_latetime=1000000;      	%�ٵ��ĳͷ�ֵ
%%
%�㷨����
maxgen=500;             	%������������
stopgen=40;                 %���Ž����ٱ��ִ���
sizepop=80;                    %������Ŀ
GGAP=0.9;                   %ѡ�����
pmutation=0.2;              %������
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
% end
% ���ɶ����������ľ������
Seq=[num_center;data(:,2)];
for i=1:length(Seq)
    for j=1:length(Seq)
        Dist(i,j)=graphshortestpath(sparse(D),Seq(i),Seq(j));
    end
end
%%
%���ɳ�ʼ��
for i=1:sizepop
    Chrom2(i,:)=randperm(num_order);
    zero_location=randperm(num_order-1);
    zero_location=zero_location(1:num_car-1);
    Chrom(i,:)=insertzeros(Chrom2(i,:),zero_location);
end
Chrom=[zeros(sizepop,1) Chrom zeros(sizepop,1)];%��ʼ��
ObjV=Objectfunction(Chrom,data,Dist,speed,num_order,max_load,w_dist,w_overweight,w_earlytime,w_latetime);%�����ʼ���Ŀ�꺯��ֵ
[bestObjV,bestindex]=max(ObjV);
bestsol=Chrom(bestindex,:);
%%
%ѭ����ʼ
gen=1;  %��ʼ�Ŵ�����
gen0=0; %��ʼ���ִ���   
while gen0<stopgen && gen<=maxgen
    disp(gen)
    FitnV=ranking(-ObjV);                                   %������Ӧ��ֵ(Assign fitness values)
    SelCh=select('sus', Chrom, FitnV, GGAP);                %ѡ��
    SelCh=cross(SelCh,num_car+1,floor(sizepop*GGAP/2)*2);      %����
    SelCh=mut(SelCh,pmutation,floor(sizepop*GGAP/2)*2);        %����
    ObjVSel=Objectfunction(SelCh,data,Dist,speed,num_order,max_load,w_dist,w_overweight,w_earlytime,w_latetime);           %�����Ӵ�Ŀ�꺯��ֵ
    [Chrom,ObjV]=reins(Chrom, SelCh, 1, 1, ObjV, ObjVSel);	%�ز���
    [newbestObjV,newbestindex]=max(ObjV);
    [worestObjV,worestindex]=min(ObjV);
    if newbestObjV>bestObjV
        bestObjV=newbestObjV;
        bestsol=Chrom(newbestindex,:);
        gen0=0;
    else
        gen0=gen0+1;                    %����ֵ���ִ�����1
    end
    Chrom(worestindex,:)=Chrom(newbestindex,:);
    ObjV(worestindex,:)=newbestObjV;
    trace(gen,1)=1/bestObjV;            %��¼ÿһ����������õ���Ӧ��
    trace(gen,2)=sum(1./ObjV)/sizepop;  %��¼ÿһ��������ƽ����Ӧ��
    gen=gen+1;                          %�Ŵ�������1
end
gen=gen-1;
%����ÿ����������·
bestzero_location=find(bestsol==0);     %Ѱ�����Ž���0��λ��
for i=1:num_car
    Seq_order{1,i}=bestsol(bestzero_location(i):bestzero_location(i+1));%ÿ���������͵Ķ������˳��
	circle{1,i}=Seq(Seq_order{1,i}+1)'; %����λ��˳��
    route{1,i}=circle{1,i}(1);          %ÿ���������͵Ķ���λ��˳��
    for j=1:length(circle{1,i})-1
        [~,temp_route]=graphshortestpath(sparse(D),circle{1,i}(j),circle{1,i}(j+1));
        route{1,i}=[route{1,i} temp_route(2:end)];%ÿ����������ʻ��·
    end
end
%%
%������
disp('---------------------------------------------------------------------')
disp('���Ŷ���˳��')
disp(bestsol)
for i=1:num_car
    disp(['����',num2str(i),'���͵Ķ���˳��'])
    disp(Seq_order{1,i})
    disp(['����',num2str(i),'������·�ߣ�'])
    disp(route{1,i})
end
disp('����ֵ��')
disp(1/bestObjV)
%%
%��ͼ
%ͼ1���Ż�����ͼ
figure
plot(1:gen,1./trace(:,1));
plot(1:gen,trace(:,1),'r-',1:gen,trace(:,2),'b--');
title(['Ŀ�꺯���Ż�����  ' '��ֹ������' num2str(gen)],'fontsize',12);
xlabel('��������','fontsize',12);
ylabel('Ŀ�꺯��ֵ','fontsize',12);
legend('�������ֵ','����ƽ��ֵ');
xlim([1,gen])
%ͼ2��ȫ������������·��ͼ
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
shift=0.0004;%����λ����
for i=1:num_car
    LL=[];
    LL=location(route{1,i},2:3);
    for j=2:length(route{1,i})
        quiver(LL(j-1,1),LL(j-1,2)+(i-1)*shift,(LL(j,1)-LL(j-1,1))/0.9,(LL(j,2)-LL(j-1,2))/0.9,'color',colour(i,:),'linewidth',1.5);
%         [arrowx,arrowy] = dsxy2figxy(gca,LL(j-1:j,1)+(i-1)*shift,LL(j-1:j,2)+(i-1)*shift);%����ת��
%         annotation('textarrow',arrowx,arrowy,'HeadWidth',9,'color',colour(i,:),'linewidth',1.5);
    end
end
xlabel('������')
ylabel('������')
title('ȫ������������·��ͼ')
box on
hold off;
% %ͼ3��ÿ������������·��ͼ
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
%     shift=0;%����λ����
%     LL=[];
%     LL=location(route{1,i},2:3);
%     for j=2:length(route{1,i})
%         quiver(LL(j-1,1),LL(j-1,2),(LL(j,1)-LL(j-1,1))/0.9,(LL(j,2)-LL(j-1,2))/0.9,'color',colour(i,:),'linewidth',1.5);
% %         [arrowx,arrowy] = dsxy2figxy(gca,LL(j-1:j,1)+(i-1)*shift,LL(j-1:j,2)+(i-1)*shift);%����ת��
% %         annotation('textarrow',arrowx,arrowy,'HeadWidth',9,'color',colour(i,:),'linewidth',1.5);
%     end
%     xlabel('������')
%     ylabel('������')
%     title(['����',num2str(i),'������·��ͼ'])
%     box on
%     hold off;
% end