function  ringK= computeVertexRing_k(triList,k,idx)

ring1=computeVertexRing1(triList);
ringK=ring1(idx,:);
for i=1:length(idx)
    for j=1:k
        if j==1
            continue;
        else
            [~,~,v_tmp]=find(ringK(i,:));
            ring1_tmp=ring1(v_tmp,:);
            ring1_tmp=reshape(ring1_tmp,1,[]);
            ring1_tmp(ring1_tmp==0)=[];
            ring_tmp=[ringK(i,:),ring1_tmp];
            
            ring_tmp(ring_tmp==0)=[];
            ring_tmp(ring_tmp==idx(i))=[];
            ring_tmp=unique(ring_tmp);
            
            ringK(i,1:length(ring_tmp))=ring_tmp;
        end 
    end
end


end


function  ring1= computeVertexRing1(triList)
ring1=reshape(triList,[],1);
ring1=zeros(length(unique(ring1)),13);
for j=1:length(ring1)
        [~,col_No]=find(triList==j);
        ring_1_tmp=triList(:,col_No);
        ring_1_tmp=unique(reshape(ring_1_tmp,[],1));
        ring_1_tmp(ring_1_tmp==j)=[];
        ring1(j,1:length(ring_1_tmp))=ring_1_tmp;
end

end

