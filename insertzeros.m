function out=insertzeros(in,location)
location=sort(location,'descend');
for i=1:length(location)
    in=[in(1:location(i)) 0 in(location(i)+1:end)];
end
out=in;
end