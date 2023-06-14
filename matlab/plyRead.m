function  [shape, normal, texture, triList]= plyRead( filename )

fid= fopen(filename, 'r');
tLine = fgetl(fid);
count_N=[0 0];
k=0;
while ischar(tLine)
    if ~isempty(tLine)
        switch tLine(1:3)
            case  'ele' % vertex
                k=k+1;
                startIndex= regexp(tLine,'\d');
                count_N(k)=str2num(tLine(startIndex:end));
            case 'end' % texture
                break;
            otherwise
        end
    else
    end
    tLine = fgetl(fid);
end

pc_cld=fscanf(fid, '%f%f%f%f%f%f%d%d%d%d%d', [11,count_N(1)]);
shape=pc_cld(1:3,:);
normal=pc_cld(4:6,:);
texture=pc_cld(8:10,:);
triList = fscanf(fid, '%d%d%d%d%d', [5,count_N(2)]);
triList=triList(2:4,:)+1;
fclose(fid);

end



