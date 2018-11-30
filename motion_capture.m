%% Estimate background

% vsrc = VideoReader('data/videos/20170201_120045.MOV');
vsrc = VideoReader('C:\Users\Admin\Documents\GitHub\ground-pursuit\MATLAB\testcar_1.avi'); 
vsrc.CurrentTime = 0;

se1 = strel('square', 5);
se2 = strel('square', 3);

frame = vsrc.readFrame();
frames = zeros([size(frame), 50], 'uint8');
% bw = zeros([size(frame), 50]);
skipTime = (vsrc.Duration - vsrc.CurrentTime) / 50;

for k = 1 : size(frames, 4)-1
    frames(:,:,:,k) = vsrc.readFrame();
%     bw(:,:,:,k) = vsrc.readFrame();
%     [bw_mask, ~] = createMask1(bw(:,:,:,k));
%     bw_mask = imclose(bw_mask, se1);
%     bw_mask = imopen(bw_mask, se2); % Remove small bubbles to decrease noise
%     bw(:,:,:,k) = bw_mask;
    vsrc.CurrentTime = vsrc.CurrentTime + skipTime;
    if mod(k, 10) == 0
        fprintf('Completed %d out of %d...\n', k, size(frames, 4))
    end
end
fprintf('Done saving frames\n')

bg = median(frames, 4);
figure, imshow(bg)

%% find MAE threshold to call foreground
% bg_double = cast(bg, 'double');

err = zeros(numel(bg(:,:,1)), size(frames, 4));
for k = 1 : size(frames, 4)
    tmp = sum(abs(cast(frames(:,:,:,k), 'double') - bg_double), 3);
    err(:,k) = tmp(:);
end

figure, histogram(err)
% Pick a value between two hills as error minimum value

%%
maskPlayer = vision.VideoPlayer('Position', [1050, 200, 720, 760]);
videoPlayer = vision.VideoPlayer('Position', [140, 200, 720, 760]);
        
        
bg_double = cast(bg(:,440:1130,3), 'double');
vsrc.CurrentTime = 0;
% show frames
% figure

nextId = 1;
bboxes = cat(1, ones(1, 4));
centroids = zeros(4, 3);
kalmanFilter = configureKalmanFilter('ConstantVelocity', ...
    [0 0], [200, 50], [100, 25], 50);
tracks = repmat(struct(...
                'id', nextId, ...
                'bbox', ones(1,4), ...
                'kalmanFilter', kalmanFilter, ...
                'age', 1, ...
                'totalVisibleCount', 1, ...
                'consecutiveInvisibleCount', 0), [0, 1]);
            
for k = 1 : size(frames, 4)-1
    frame = frames(:,440:1130,:,k); % frames(:,:,:,k);
    
    error = sum(abs(cast(frame, 'double') - bg_double), 3) > 130; % foreground
    error = imclose(error, se2);
    error = imopen(error, se1);
    error = imfill(error, 'holes');

    stats = regionprops(logical(error), 'BoundingBox', 'Centroid');
    centroids = cat(1, stats.Centroid);
    bboxes = cat(1, stats.BoundingBox);
            
%     for obj = 1 :length(tracks)
%         tracks(obj).predict();
%         tracks(obj).bbox = stats(obj).BoundingBox;
%         bbox(obj)
%         centroids(obj, :) = stats(obj).Centroid;
%         rectangle('Position', bb, 'EdgeColor', 'r' , 'LineWidth' , 2)
%         plot(bc(1), bc(2), '-m+')
%         a = text(bc(1)+15, bc(2),strcat('X:', num2str(round(bc(1))), 'Y:' , num2str(round(bc(2)))));
%         set(a, 'FontName', 'Arial', 'FontWeight', 'bold', 'FontSize', 12, 'Color', 'black');
%     end
%     drawnow
            
    % Predict new track
    ntracks = length(tracks);
    ndetect = size(centroids, 1);
    wcentroids = pointsToWorld(cameraParams,rotationMatrix,...
                translationVector,centroids);
    for i = 1:ntracks
    tracks(i).kalmanFilter.predict()
%     predictedCentroid = predictedCentroid - tracks(i).bbox(3:4)/2;
%     tracks(i).bbox = [predictedCentroid, tracks(i).bbox(3:4)];
    end
    
    % Assign tracks
    cost = zeros(ntracks, ndetect);
    for i = 1:ntracks
            cost(i, :) = distance(tracks(i).kalmanFilter, wcentroids);
    end
    
    costOfNonAssignment = 20;
        [assignments, unassignedTracks, unassignedDetections] = ...
            assignDetectionsToTracks(cost, costOfNonAssignment);
    
    % Update assigned tracks
    numAssignedTracks = size(assignments, 1);
        for i = 1:numAssignedTracks
            trackIdx = assignments(i, 1);
            detectionIdx = assignments(i, 2);
            centroid = wcentroids(detectionIdx, :);
            bbox = bboxes(detectionIdx, :);
            
            % Correct the estimate of the object's location
            % using the new detection.
            correct(tracks(trackIdx).kalmanFilter, centroid);
            
            % Replace predicted bounding box with detected
            % bounding box.
            tracks(trackIdx).bbox = bbox;
            
            % Update track's age.
            tracks(trackIdx).age = tracks(trackIdx).age + 1;
            
            % Update visibility.
            tracks(trackIdx).totalVisibleCount = ...
                tracks(trackIdx).totalVisibleCount + 1;
            tracks(trackIdx).consecutiveInvisibleCount = 0;
        end
        
        % Update unassigned tracks
        for i = 1:length(unassignedTracks)
            ind = unassignedTracks(i);
            tracks(ind).age = tracks(ind).age + 1;
            tracks(ind).consecutiveInvisibleCount = ...
                tracks(ind).consecutiveInvisibleCount + 1;
        end
        
        % Delete lost tracks
%         if isempty(tracks)
%             continue
%         end
        
        invisibleForTooLong = 20;
        ageThreshold = 8;
        
        % Compute the fraction of the track's age for which it was visible.
        ages = [tracks(:).age];
        totalVisibleCounts = [tracks(:).totalVisibleCount];
        visibility = totalVisibleCounts ./ ages;
        
        % Find the indices of 'lost' tracks.
        lostInds = (ages < ageThreshold & visibility < 0.6) | ...
            [tracks(:).consecutiveInvisibleCount] >= invisibleForTooLong;
        
        % Delete lost tracks.
        tracks = tracks(~lostInds);
        
        % Create new tracks
        wcentroids = wcentroids(unassignedDetections, :);
        bboxes = bboxes(unassignedDetections, :);
        
        for i = 1:size(wcentroids, 1)
            
            centroid = wcentroids(i,:);
            bbox = bboxes(i, :);
%             wcentroid = pointsToWorld(cameraParams,rotationMatrix,...
%                 translationVector,centroid);
            
            kalmanFilter = configureKalmanFilter('ConstantVelocity', ...
                centroid, [200, 50], [100, 25], 50);
            
            % Create a new track.
            newTrack = struct(...
                'id', nextId, ...
                'bbox', bbox, ...
                'kalmanFilter', kalmanFilter, ...
                'age', 1, ...
                'totalVisibleCount', 1, ...
                'consecutiveInvisibleCount', 0);
            
            % Add it to the array of tracks.
            tracks(end + 1) = newTrack;
            
            % Increment the next id.
            nextId = nextId + 1;
        end
        
        % Display tracking results
        frame = im2uint8(frame);
        mask = uint8(repmat(error, [1, 1, 3])) .* 255;
        
        minVisibleCount = 8;
        if ~isempty(tracks)
              
            % Noisy detections tend to result in short-lived tracks.
            % Only display tracks that have been visible for more than 
            % a minimum number of frames.
            reliableTrackInds = ...
                [tracks(:).totalVisibleCount] > minVisibleCount;
            reliableTracks = tracks(reliableTrackInds);
            
            % Display the objects. If an object has not been detected
            % in this frame, display its predicted bounding box.
            if ~isempty(reliableTracks)
                % Get bounding boxes.
                bboxes = cat(1, reliableTracks.bbox);
                
                % Get ids.
                ids = int32([reliableTracks(:).id]);
                
                % Create labels for objects indicating the ones for 
                % which we display the predicted rather than the actual 
                % location.
                labels = cellstr(int2str(ids'));
                predictedTrackInds = ...
                    [reliableTracks(:).consecutiveInvisibleCount] > 0;
                isPredicted = cell(size(labels));
                isPredicted(predictedTrackInds) = {' predicted'};
                labels = strcat(labels, isPredicted);
                
                % Draw the objects on the frame.
                frame = insertObjectAnnotation(frame, 'rectangle', ...
                    bboxes, labels);
                
                % Draw the objects on the mask.
                mask = insertObjectAnnotation(mask, 'rectangle', ...
                    bboxes, labels);
            end
        end
        
        % Display the mask and the frame.
        

        maskPlayer.step(mask);        
        videoPlayer.step(frame);
        
    pause(0.5) 
    fprintf('%d ', k)
end
