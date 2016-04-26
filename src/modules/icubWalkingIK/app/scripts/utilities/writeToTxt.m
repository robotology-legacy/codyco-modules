function [ ret ] = writeToTxt( name, input, formatSpec )
%WRITETOCSV Write input matrix into CSV file with the specified format.
%   WRITETOTXT(NAME,INPUT,FORMATSPEC) Writes INPUT matrix as is into
%   'name'.txt using the specified format.
%   name must contain the full path to the file.
%   formatSpec: e.g. formatSpec = '%6.5f, %10.5f, %10.5f, %10.5f\n'

fileID = fopen([ name '.txt'],'w');
if fileID == -1
    ret = 0;
end

% formatSpec = '%6.5f, %10.5f, %10.5f, %10.5f\n';
% Matlab required input to be transposed so that it's printed in the same
% order as in the matrix.
fprintf(fileID,formatSpec,input');
fclose(fileID);
ret = 1;

if (ret)
    display([name ' TXT written succesfully.']);
else
    error([name ' TXT NOT written successfully']);
end


end

