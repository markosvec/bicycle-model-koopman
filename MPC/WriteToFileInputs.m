function [] = WriteToFileInputs(fileName, time, inputs, forces)
%WRITE TO FILE
%   The function writes data to file prepared for plotting in Latex using 
%   pgfplot package.

% Prepare data
steer = inputs(:,1);
slip_f = inputs(:,2);
slip_r = inputs(:,3);

Fxf = forces(:,1);
Fyf = forces(:,2);
Fxr = forces(:,3);

% Open the file to write in
fileID = fopen(fileName, 'w');

% Print the text line to file
firstLine = ['time steer slipF slipR Fxf Fyf Fxr\n'];

fprintf(fileID, firstLine);

% Load number of continuous time steps and discrete time steps
lenCont = length(time);

dataFormat = ['%f %f %f %f %f %f %f\n'];

for i=1:lenCont
    fprintf(fileID, dataFormat, time(i), steer(i), slip_f(i), slip_r(i), Fxf(i), Fyf(i), Fxr(i));   
end

fclose(fileID);
end

