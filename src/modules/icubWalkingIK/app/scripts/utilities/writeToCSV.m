function [ ret ] = writeToCSV( name, input, dest )
%WRITETOCSV Write input matrix into CSV file separated by commas
%   WRITETOCSV(NAME,INPUT,DEST) Writes INPUT matrix as is into
%   DEST/NAME.csv using commas and a space after the
%   comma. No comma is added at the end of the line. 
%   Note: DEST contains the last backslash
ret = 1;

fileID = fopen([dest name '.csv'],'w');
if fileID == -1
    error(['Problem writing CSV file... ' name '\.csv' '. Check destination provided.']);
    ret = 0;
end
    
formatSpec = '%6.5f, %10.5f, %10.5f, %10.5f\n';
% Matlab required input to be transposed so that it's printed in the same
% order as in the matrix.
fprintf(fileID,formatSpec,input');
fclose(fileID);

if (ret)
    display([name ' saved as: ']);
    display([dest name '.csv']);
else
    error([dest name '.csv' '... CSV NOT written successfully']);
end


end

