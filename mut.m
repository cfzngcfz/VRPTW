function newchorm=mut(chorm,pmutation,NIND)
%±äÒì
for i=1:NIND  
    pick=rand;
    while pick==0
        pick=rand;
    end
	if pick>pmutation
        newchorm(i,:)=chorm(i,:);
        continue;
	end
    flag=0;
    while flag==0
        chormC=chorm(i,:);
        location=randperm(size(chormC,2)-2);
        location=location(1:2)+1;
        temp1=chormC(location(1));
        temp2=chormC(location(2));
        chormD=chormC;
        chormD(location(1))=temp2;
        chormD(location(2))=temp1;
        if sum(abs(chormD-chormC))==0 || ~isempty(find(chormD(1:13)-chormD(2:14)==00, 1))
            flag=0;
        else
            flag=1;
            newchorm(i,:)=chormD;
        end
    end
end