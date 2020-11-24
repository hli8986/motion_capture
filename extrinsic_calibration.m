%% Select frame for extrinsic calibration
%   (Do it only once. Save an image file)
clear

filename = 'Your_video_file_here.avi';
v = VideoReader(filename);
v.CurrentTime = 0;

dt = 1 / 20;
figure, h = imshow(rgb2gray(readFrame(v)));

while hasFrame(v)
    tic
    frame = rgb2gray(readFrame(v));
    set(h, 'CData', frame)
    pause(dt - toc)
end

imwrite(frame, 'image_name.jpg')

%% Select image points by hand

frame = imread('image_name.jpg');
figure, imshow(frame)
[x, y] = ginput();

imagePoints = [x, y];
imagePoints = [imagePoints; imagePoints(1, :)];

% Define world points
worldPoints = [3, 0, 0
                3, 4, 0
                3, 8, 0
                3, 12, 0
                3, 16, 0
                3, 17.6, 0
                -3, 17.6, 0
                -3, 16, 0
                -3, 12, 0
                -3, 8, 0
                -3, 4, 0 
                -3, 0, 0
                3, 0, 0] * 30.48;

%% Extrinsic calibration
load('cameraParams.mat');

[imagePoints, err] = undistortPoints(imagePoints, cameraParams);
% [worldOrientation,worldLocation] = estimateWorldCameraPose(imagePoints,worldPoints,cameraParams, ...
%     'MaxNumTrials', 10000);
[worldOrientation,worldLocation] = estimateWorldCameraPose(imagePoints,worldPoints,cameraParams);
% worldOrientation maps world to camera coordinates
% worldLocation is in world coordinates

rotationMatrix = worldOrientation';
translationVector = -worldLocation * worldOrientation';

% test on imagePoints
worldPointsEst = pointsToWorld(cameraParams,rotationMatrix,translationVector,imagePoints);
error = sqrt(sum((worldPointsEst - worldPoints(:,1:2)).^2, 2))

% Show camera position
pcshow(worldPoints,'VerticalAxis','Y','VerticalAxisDir','down', ...
    'MarkerSize',30);
hold on
plotCamera('Size',10,'Orientation',worldOrientation,'Location',...
    worldLocation);
hold off
 
%% Animation
% This is optional

%   Click points on right subplot to see the projection on the court

% clear
% load('extrinsic_calibration.mat')

% outCourt = [10, 0
%             10, 40
%             -10, 40
%             -10, 0
%             10, 0 ];
        
outCourt = [3, 0
            3, 17.6
            -3, 17.6
            -3, 0
            3, 0 ];

% serveBox = [10, 20
%             10, 25
%             -10, 25
%             -10, 20
%             10, 20
%             NaN, NaN
%             8.5, 20
%             8.5, 25
%             7, 25
%             7, 20
%             8.5, 20
%             NaN, NaN
%             -8.5, 20
%             -8.5, 25
%             -7, 25
%             -7, 20
%             -8.5, 20];
%         
% shortLine = [10, 15
%             -10, 15];
        
figure
subplot(121), plot(outCourt(:,1), outCourt(:,2), 'k', 'linewidth', 5)
% hold on, plot(serveBox(:,1), serveBox(:,2), 'r', 'linewidth', 3)
% plot(shortLine(:,1), shortLine(:,2), 'r--', 'linewidth', 3)
axis equal, xlim([-3, 3]), ylim([0, 17.6]), grid on, xlabel('X_W'), ylabel('Y_W')
subplot(122), imshow(frame)
set(gcf, 'Position', [0.0010    0.0410    1.5360    0.7488]*1e3)

[x, y] = getpts();
subplot(122), hold on, plot(x, y, 'x', 'linewidth', 5, 'markersize', 15)
wPts = pointsToWorld(cameraParams,rotationMatrix,translationVector,[x, y]);
wPts_ft = wPts / 30.48;
subplot(121), plot(wPts_ft(:,1), wPts_ft(:,2), 'x', 'linewidth', 5, 'markersize', 20)

%% From 3D to image

filename = '/Users/hli8/Documents/MATLAB/motion_traking/testcar_1.avi';
v = VideoReader(filename);

%%
v.CurrentTime = 0;

frame = readFrame(v);
        
figure
subplot(121), plot(outCourt(:,1), outCourt(:,2), 'k', 'linewidth', 5)
% hold on, plot(serveBox(:,1), serveBox(:,2), 'r', 'linewidth', 3)
% plot(shortLine(:,1), shortLine(:,2), 'r--', 'linewidth', 3)
axis equal, xlim([-3, 3]), ylim([0, 17.6]), grid on, xlabel('X_W'), ylabel('Y_W')
subplot(122), imshow(frame)
hold on,
set(gcf, 'Position', [1          41        1680         933])

[x, y] = getpts();
wPts = pointsToWorld(cameraParams,rotationMatrix,translationVector,[x, y]);
wPts_ft = wPts / 30.48;

colors = [0.7, 0, 0
          0, 0.7, 0
          0, 0, 0.7
          0.8, 0.2, 0.6];

for k = 1 : length(x)
    footprint = [wPts_ft(k, 1), wPts_ft(k, 2), 0];  % in ft
    height = 0.5;

    wid = 0.5;
    box = ([wid/2, 0, 0
           wid/2, 0, height
           -wid/2, 0, height
           -wid/2, 0, 0
           wid/2, 0, 0 ] + footprint) * 30.48;

    iPts = worldToImage(cameraParams,rotationMatrix,translationVector, box);

    
    subplot(121)
    hold on
    plot(footprint(:,1), footprint(:,2), 'x', 'linewidth', 5, 'markersize', 20, 'color', colors(k,:))
    subplot(122), fill(iPts(:,1), iPts(:,2), colors(k,:), 'linewidth', 1, 'FaceAlpha', 0.3, 'EdgeColor', colors(k,:))
end
