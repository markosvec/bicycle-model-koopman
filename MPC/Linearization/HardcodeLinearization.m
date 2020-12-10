function [] = HardcodeLinearization(states, input, dStates)
%HARDCODE LINEARIZATION
%   The functions creates MATLAB script with hardcoded linearized system
%   model to avoid symbolic linearization in every step.

% Linearize the model

A = jacobian(dStates, states);
B = jacobian(dStates, input);

% Write to file
fileID = fopen('LinearizedMatrices.m','w');

% Write function initialization
fprintf(fileID,'function [Ad, Bd] = LinearizedMatrices(states,input, T)');
fprintf(fileID, '\n%% LINEARIZED MATRICES\n');
fprintf(fileID, '%%   The function returns linearized model of a vehicle.\n\n');

% Write variable initialization
fprintf(fileID,'%% Initialize state variables\n');
fprintf(fileID,'vx = states(1);\nvy = states(2);\nyawRate = states(3);\n');

fprintf(fileID,'\n%% Initialize input variables\n');
fprintf(fileID,'deltaF = input(1);\nsF = input(2);\nsR = input(3);\n');

fprintf(fileID,'\n%% Initialize matrices\n');
fprintf(fileID,'nx = length(states);\nnu = length(input);\nA = zeros(nx,nx);\nB = zeros(nx,nu);\n');

% Write hardcoded matrices
% Matrix A
fprintf(fileID,'\n%% Define A matrix elements\n%% ROW 1\n');
fprintf(fileID,'A(1,1) = %s;\n', char(A(1,1)));
fprintf(fileID,'A(1,2) = %s;\n', char(A(1,2)));
fprintf(fileID,'A(1,3) = %s;\n', char(A(1,3)));

fprintf(fileID,'%% ROW 2\n');
fprintf(fileID,'A(2,1) = %s;\n', char(A(2,1)));
fprintf(fileID,'A(2,2) = %s;\n', char(A(2,2)));
fprintf(fileID,'A(2,3) = %s;\n', char(A(2,3)));

fprintf(fileID,'%% ROW 3\n');
fprintf(fileID,'A(3,1) = %s;\n', char(A(3,1)));
fprintf(fileID,'A(3,2) = %s;\n', char(A(3,2)));
fprintf(fileID,'A(3,3) = %s;\n', char(A(3,3)));

% Matrix B
fprintf(fileID,'\n%% Define B matrix elements\n%% ROW 1\n');
fprintf(fileID,'B(1,1) = %s;\n', char(B(1,1)));
fprintf(fileID,'B(1,2) = %s;\n', char(B(1,2)));
fprintf(fileID,'B(1,3) = %s;\n', char(B(1,3)));

fprintf(fileID,'%% ROW 2\n');
fprintf(fileID,'B(2,1) = %s;\n', char(B(2,1)));
fprintf(fileID,'B(2,2) = %s;\n', char(B(2,2)));
fprintf(fileID,'B(2,3) = %s;\n', char(B(2,3)));

fprintf(fileID,'%% ROW 3\n');
fprintf(fileID,'B(3,1) = %s;\n', char(B(3,1)));
fprintf(fileID,'B(3,2) = %s;\n', char(B(3,2)));
fprintf(fileID,'B(3,3) = %s;\n', char(B(3,3)));

% Write function ending
fprintf(fileID, '\nend');
fclose(fileID);

end

