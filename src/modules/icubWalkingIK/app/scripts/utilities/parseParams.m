function [ params ] = parseParams( fileName )
%PARSEPARAMS Parses walking parameters from a txt file

fileID = fopen(fileName,'r');
if (fileID == -1)
    params = 1;
else
    paramsCell = textscan(fileID, '%s%f');
    params = cell2struct(num2cell(paramsCell{2}'),paramsCell{1}',2);
end
fclose(fileID);

end

