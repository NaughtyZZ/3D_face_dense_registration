clear;
clc;
close all;

%%precomputing weight setting according to the distance to the nearest landmarks
% Dfield=heat_geodesic(shape', triList', idxE, 10);

load Dfield.mat

%load template and landmark indices
[shapeR, normalR, textureR, triListR] =   plyRead('template.ply');
shapeR=shapeR';
landmarksR=load('template.pp');
idxF=knnsearch(shapeR,landmarksR);
V=1:length(shapeR);
V=V';
V=setdiff(V,idxF);

%load target and landmarks
[shapeT, ~, ~, triListT] =   plyRead('target.ply');
shapeT=shapeT';
landmarksT=load('target.pp');

%compute the indices for edge vertices
faces=triListT';
alledges=[faces(:,1:2);faces(:,2:3);faces(:,[3 1])];
alledges=sort(alledges,2);
[~, ia1,~]=unique(alledges,'rows','first','legacy');
[~, ia2,~]=unique(alledges,'rows','last','legacy');
indexes=ia1-ia2;
idx_edges=alledges(ia1(indexes==0),:);
idxEdge=idx_edges;
idxE=unique(reshape(idxEdge,[],1));


%pre-set fixed coeff matrices
ring1= computeVertexRing_k(triListR,1,1:length(shapeR)); %compute 1-ring neighbors
cRing=ring1;
cRing(cRing>0)=1;
cRing=sum(cRing,2);
cRing=sparse(cRing);
B=diag(cRing);
A=speye(length(shapeR));

allShapeCell_ring1=cell(length(shapeR),1); % store local cell of 1-ring neighbors

for i=1:length(shapeR)
    j=1;
%     B(i,i)=0;
    while(j<=13&&ring1(i,j)~=0)
        B(i,ring1(i,j))=-1;
%         B(i,ring1(i,j))=-1*error(i);
%         B(i,i)=B(i,i)+error(i);
        j=j+1;
    end
    tempRing=ring1(i,:);
    allShapeCell_ring1{i,1}=shapeR(tempRing(tempRing>0),:)-shapeR(i,:);
end






% Build AABB tree
FV.faces    =  triListT';
FV.vertices = shapeT;


% exclude columns that correspond to landmarks 
A2=A;
B2=B;
A2(:,idxF)=[];
B2(:,idxF)=[];

%initial deformation according to CVPR-19 (fan et al.) Boosting
thresholdW=10;
shapeR2=shapeR;
for i=1:length(shapeR)

        tmp=Dfield(i,:);
        tmp2=tmp;
        tmp2(tmp2==0)=[];
        min_tmp=min(tmp2);
        tmp(tmp==0)=min_tmp/10;
        
        weightT=1./tmp.^4;
        weightT(isinf(weightT))=thresholdW;
        weightT(isnan(weightT))=thresholdW;
        weightT(weightT>thresholdW)=thresholdW;        
        [X, T] = rigidTransform3DW(shapeR(idxF,:), landmarksT, weightT');
        shapeR2(i,:)=shapeR(i,:)*X'+T';

end
shapeR=shapeR2;
shapeR(idxF,:)=landmarksT; %match landmark exactly
writePlyFile(strcat('.\output_step\','0.ply'), shapeR',  normalR, textureR, triListR);  %save initialization

%trunk of the IDD algorithm
for j=1:30
    Dfield1=zeros(size(shapeR));
    Dfield2=zeros(size(shapeR));
    lamda = 1; %constant lamda for simplified version
    
    %preliminary correspondence on AABB tree (dividing)
    idx=knnsearch(shapeT,shapeR(V,:));
    idxN=V(~ismember(idx,idxE));
    [~,des] = point2trimesh(FV, 'QueryPoints', shapeR(idxN,:),'Algorithm' ,'parallel_vectorized_subfunctions');
    Dfield1(idxN,:)=des-shapeR(idxN,:); %normal offset

    shapeR2=shapeR;
    shapeR2(idxN,:)=des;
    for i=1:length(shapeR)
        tempRing=ring1(i,:);
        points=shapeR2(tempRing(tempRing>0),:)-shapeR2(i,:);
        pointsR = allShapeCell_ring1{i,1};
        [~,Dfield2(i,:)]=rigidTransform3D(pointsR,points);  %tangential offset
    end

    Dfield=Dfield1+Dfield2;  %normal offset and tangential offset
    
        

    %smoothed deformation (diffusing)
    shiftD=zeros(size(shapeR));
    shiftD(V,:)=(A2+lamda*B2)\Dfield; 
    shapeR=shapeR+shiftD;


    %save preliminary results
    if rem(j,1)==0
        writePlyFile(strcat('.\output_step\',int2str(j),'.ply'), shapeR',  normalR, textureR, triListR);
        disp(int2str(j));
    end
end