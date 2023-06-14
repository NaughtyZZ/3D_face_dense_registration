function writePlyFile(fileName, shape,  normal, texture, triList)


%------------- BEGIN CODE --------------
narginchk(5, 5);

[fid, message] = fopen(fileName, 'w');
if fid < 0 
    error(['Cannot open the file ' fileName '\n' message]); 
end
fprintf(fid, 'ply\r\n');
fprintf(fid, 'format ascii 1.0\r\n');
textTmp=['element vertex ',int2str(length(shape)),'\r\n'];
fprintf(fid, textTmp);
fprintf(fid, 'property float x\r\n');
fprintf(fid, 'property float y\r\n');
fprintf(fid, 'property float z\r\n');
fprintf(fid, 'property float nx\r\n');
fprintf(fid, 'property float ny\r\n');
fprintf(fid, 'property float nz\r\n');
fprintf(fid, 'property int flags\r\n');
fprintf(fid, 'property uchar red\r\n');
fprintf(fid, 'property uchar green\r\n');
fprintf(fid, 'property uchar blue\r\n');
fprintf(fid, 'property uchar alpha\r\n');
textTmp=['element face ',int2str(length(triList)),'\r\n'];
fprintf(fid, textTmp);
fprintf(fid, 'property list uchar int vertex_indices\r\n');
fprintf(fid, 'property int flags\r\n');
fprintf(fid, 'end_header\r\n');

Ma_temp=[shape;normal;zeros(1,length(shape));texture;255*ones(1,length(shape))];
fprintf(fid, '%f %f %f %f %f %f %d %d %d %d %d\r\n', Ma_temp);

triList=triList-1;
Ma_temp=[3*ones(1,length(triList));triList;zeros(1,length(triList))];
fprintf(fid, '%d %d %d %d %d\r\n', Ma_temp);

fclose(fid);
end
%------------- END OF CODE --------------



