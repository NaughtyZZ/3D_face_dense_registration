clear;
clc;
clear;
clc;
addpath('..');

[shape, normal, ~,triList]= plyRead( '../template.ply' );
load jetmap.mat
load ../Dfield.mat
error=Dfield(:,1);
[~,sortIdx]=sort(error);
errorSort=zeros(size(error));
for i=1:length(sortIdx)
    errorSort(sortIdx(i))=i;
end
colorAllocation=round(255*errorSort/length(errorSort));
colorAllocation(colorAllocation==255)=254;
colorAllocation=colorAllocation+1;
texture=jetmap(colorAllocation(1:length(error)),:);
writePlyFile('geo_map.ply', shape, normal, texture', triList);


