% https://robotics.stackexchange.com/questions/7723/matlab-3d-simulation-with-solidworks-model

function [points,success] = LoadSTL(path)
% LoadSTL loads an ASCII STL file with units in [mm]. 
% The output is a structure array where each element in the array is 
% a vertex. 
%
% [points , success] = LoadSTL(path)
%
% path = String containing the path and extension of the file to be loaded.
% points = Matrix of locations where each column is a vertex, rows are x/y/z/1:
%    points(1,:) = x
%    points(2,:) = y
%    points(3,:) = z
%    points(4,:) = 1
%           NOTE - every three columns is a new face/facet. 
% success = Bit indicating if the file was successfully opened or not. 

success = 0;
fileID = fopen(path);
if fileID <0
    fprintf('File not found at path %s\n',path);
    return;
end
fprintf('Loading Path %s...\n',path);
fileData = fread(fileID,'*char');
eol = sprintf('\n');
stlFile = strsplit(fileData',eol);
fclose(fileID);
fprintf('Done.\n')
pause(0.25);
clc
assignin('base' , 'stlFile' , stlFile)
pointsTracker = 1;
for i=2:size(stlFile,2)
    if mod(pointsTracker,100)==0
        clc
        fprintf('Parsing file at %s...\n',path);
        fprintf('Currently logged %d points\n',pointsTracker);
    end

    testLine = stlFile{i};
    rawStrip = strsplit(testLine , ' ' , 'CollapseDelimiters' , true);
    if numel(rawStrip) == 5
        points(1,pointsTracker) = str2double(rawStrip{1,3})/1000;
        points(2,pointsTracker) = str2double(rawStrip{1,4})/1000;
        points(3,pointsTracker) = str2double(rawStrip{1,5})/1000;
        points(4,pointsTracker) = 1;
        pointsTracker = pointsTracker + 1;
    end
end
disp('Done.')
pause(0.25);
clc;

if mod(size(points,2),3) > 0
    disp('File format in an unexpected type.')
    disp('Check the file specified is an STL format file with ASCII formatting.')
    disp('(Error - number of vertices not a multiple of 3)')
    disp(numel(points.x))
    return;
end

success = 1;
return;