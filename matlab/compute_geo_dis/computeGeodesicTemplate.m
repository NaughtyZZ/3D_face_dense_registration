clear;
clc;
addpath('..');

[shapeR, ~, ~, triListR]= plyRead( '../template.ply' );
Gfield=zeros(length(shapeR),length(shapeR));
landmarksR=load('../template.pp');
idxF=knnsearch(shapeR',landmarksR);

parpool('local',8);
parfor i=1:length(shapeR)
    D=heat_geodesic(shapeR', triListR', i, 10);
    Gfield(:,i)=max(D)-D;
%     disp(int2str(i));
end
delete(gcp);
Gfield=(Gfield+Gfield')/2;
Gfield(logical(eye(size(Gfield))))=0;
Dfield=Gfield(:,idxF);
save ../Dfield.mat Dfield