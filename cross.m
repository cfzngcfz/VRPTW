function newchrom=cross(chrom,K,sizepop)
%½»²æ
i=1;
while i<=sizepop/2
    chromA=chrom(2*i-1,:);
    chromB=chrom(2*i,:);
    A=find(chromA==0);
    B=find(chromB==0);
    a=randi(K-1);
    b=randi(K-1);
    chromA1=chromA(A(a):A(a+1));
    chromB1=chromB(B(b):B(b+1));
    chromB(chromB==0)=[];
    for j=1:length(chromA1)-2
        chromB(chromB==chromA1(j+1))=[];
    end
    chromA(chromA==0)=[];
     for j=1:length(chromB1)-2
        chromA(chromA==chromB1(j+1))=[];
    end
    locationA=randperm(length(chromB)-1);
	if length(locationA)<K-3
        continue;
	end
    locationA=locationA(1:K-3);
    chromA2=insertzeros(chromB,locationA);
    locationB=randperm(length(chromA)-1);
	if length(locationB)<K-3
        continue;
	end
    locationB=locationB(1:K-3);
    chromB2=insertzeros(chromA,locationB);
    newchrom(2*i-1,:)=[chromA1 chromA2 0];
    newchrom(2*i,:)=[chromB1 chromB2 0];
    i=i+1;
end
end