% Template author: Samuel Khzym (khzyms@mcmaster.ca)
% Algorithm implementation: Sathurshan Arulmohan (arulmohs@mcmaster.ca) 
function [track1, track2, track3, sensorsData, cluster1, cluster2, cluster3, centr1, centr2, centr3] = fcn(frontCamDets, frontLeftRadarDetsInfo, frontRightRadarDetsInfo, lidarPointCloudInfo, clockTime)    
    %INPUT
    %sensor data
    % OUTPUT
    %track1 [isActive, xPos, yPos]
    %track2 [isActive, xPos, yPos]
    %track3 [isActive, xPos, yPos]
    %cluster1 [[xPos, yPos]]
    %cluster2 [[xPos, yPos]]
    %cluster3 [[xPos, yPos]]
    %centr1 [xPos, yPos]
    %centr2 [xPos, yPos]
    %centr3 [xPos, yPos]

    %format the data from the sensors
    sensorsData = formatSensorData(frontCamDets, frontLeftRadarDetsInfo, frontRightRadarDetsInfo, lidarPointCloudInfo);

    %run DBSCAN
    %minPoints minimum value is 3
    %minPoints and radius can be altered for different results
    radius = 0.25;
    minPoints = 3;
    [track1, track2, track3, cluster1, cluster2, cluster3, centr1, centr2, centr3] = DBSCAN(radius, minPoints, sensorsData);
end

function sensorsData = formatSensorData(frontCamDets, frontLeftRadarDetsInfo, frontRightRadarDetsInfo, lidarPointCloudInfo)
    %% Format Sensor Data
    numFrontCamDets = frontCamDets.NumDetections;
    frontCamDetsArray = frontCamDets.Detections;
    
    numFrontLeftRadarDets = frontLeftRadarDetsInfo.NumDetections;
    frontLeftRadarDetsArray = frontLeftRadarDetsInfo.Detections;
    
    numFrontRightRadarDets = frontRightRadarDetsInfo.NumDetections;
    frontRightRadarDetsArray = frontRightRadarDetsInfo.Detections;
    
    %printSensorDets(frontCamDets, frontLeftRadarDetsInfo, frontRightRadarDetsInfo, lidarPointCloudInfo, clockTime);
    
    %Get Front camera data [xPos, yPos]
    frontCamera = zeros(numFrontCamDets,2);
    for i = 1:numFrontCamDets
        detection = frontCamDetsArray(i);
        detectionState = detection.Measurement; 
        frontCamera(i,:)=[detectionState(1), detectionState(2)];
    end

    %Get left radar data [xPos, yPos]
    leftRadar = zeros(numFrontLeftRadarDets,2);
    for i = 1:numFrontLeftRadarDets
        detection = frontLeftRadarDetsArray(i);
        detectionState = detection.Measurement; 
        leftRadar(i,:) = [detectionState(1), detectionState(2)];
    end

    %Get right radar data [xPos, yPos]
    rightRadar = zeros(numFrontRightRadarDets, 2);
    for i = 1:numFrontRightRadarDets
        detection = frontRightRadarDetsArray(i);
        detectionState = detection.Measurement; 
        rightRadar(i,:) = [detectionState(1), detectionState(2)];
    end

    %Get lidar data [xPos, yPos]
    lidarDims = size(lidarPointCloudInfo); 
    numValidPoints = 0;
    lidar = zeros(1,2);

    for i = 1:lidarDims(1)
        for j = 1:lidarDims(2)
            if ~(isnan(lidarPointCloudInfo(i, j, 1)) || isnan(lidarPointCloudInfo(i, j, 2)) || isnan(lidarPointCloudInfo(i, j, 3)))
                numValidPoints = numValidPoints + 1;
                lidar = [lidar;[lidarPointCloudInfo(i, j, 1), lidarPointCloudInfo(i, j, 2)]];
            end
        end
    end

    %combine sensors' data into one matrix 
    sensorsData = [frontCamera;leftRadar;rightRadar;lidar(2:end,:)];
end

function [track1, track2, track3, cluster1, cluster2, cluster3, centr1, centr2, centr3] = DBSCAN(radius, minPoints, sensorsData)
    %% Algorithm- DBSCAN
    %INPUT
    % radius = int
    %minPoints = int
    %sensorsData = [[xPos, yPos]]
    %leftOverData = [[xPos, yPos]] 
    %OUTPUT
    %track1 [isActive, xPos, yPos]
    %track2 [isActive, xPos, yPos]
    %track3 [isActive, xPos, yPos]
    %cluster1 [[xPos, yPos]]
    %cluster2 [[xPos, yPos]]
    %cluster3 [[xPos, yPos]]
    %centr1 [xPos, yPos]
    %centr2 [xPos, yPos]
    %centr3 [xPos, yPos]
    
    
    %Find Core Points
    dataDim = size(sensorsData, 1);
    pointTypeData = [sensorsData zeros(dataDim,1)]; %[xPos, yPos, isCorePoint]

    for i = 1: dataDim
        pointTypeData(i,3) = findCorePoint(sensorsData(i,:), sensorsData, radius, minPoints);
    end

    %Sort Data into Core Points and non Core Points
    corePointDataLoc = pointTypeData(:,3) == 1;
    corePointData = pointTypeData(corePointDataLoc, :);
    corePointData(:,3) = []; %remove the isCorePoint column
    

    nonCorePointDataLoc = pointTypeData(:,3) == 0;
    nonCorePointData = pointTypeData(nonCorePointDataLoc, :);
    nonCorePointData(:,3) = [];
    
    %Organize the Corepoints into clusters
    corePointClusters = groupCorePoints(corePointData, radius);
    
    %Classify border points into the clusters
    clusterData = classifyBorderPoints(nonCorePointData, corePointClusters, radius);

    %Assign Tracks 
    clusterNum = unique(clusterData(:,3));
    
    %If there are more than three clusters, find the top three clusters
    %with the most points 
    if size(clusterNum, 1) > 3
        temp = clusterData;

        topCluster = mode(temp(:,3));
        temp(temp == topCluster) = NaN;

        secondCluster = mode(temp(:,3));
        temp(temp == secondCluster) = NaN;

        thirdCluster = mode(temp(:,3));

        clusters = [topCluster; secondCluster; thirdCluster];
    else
        clusters = clusterNum;
    end

    clusters = clusters';
 
    %Assign clusters to tracks. The centroid of the cluster points is used
    %as the position of the tracks
    
    %Sets up Track 1 if needed
    if size(clusters, 2) >= 1
        track1Points = clusterData(clusterData(:,3) == clusters(1), 1:2);
        centr1 = mean(track1Points);
        track1 = [1 centr1(1) centr1(2)];
        cluster1 = track1Points;
    else
        centr1 = [0 0];
        track1 = [0 inf inf];
        cluster1 = [0 0];
    end

    %Sets up Track 2 if needed
    if size(clusters, 2) >= 2
        track2Points = clusterData(clusterData(:,3) == clusters(2), 1:2);
        centr2 = mean(track2Points);
        track2 = [1 centr2(1) centr2(2)];
        cluster2 = track2Points;
    else
        centr2 = [0 0];
        track2 = [0 inf inf];
        cluster2 = [0 0];
    end

    %Sets up Track 3 if needed
    if size(clusters, 2) >= 3
        track3Points = clusterData(clusterData(:,3) == clusters(3), 1:2);
        centr3 = mean(track3Points);
        track3 = [1 centr3(1) centr3(2)];
        cluster3 = track3Points;
    else
        centr3 = [0 0];
        track3 = [0 inf inf];
        cluster3 = [0 0];   
    end

end

%Determines if a point is a Core Point
function isCorePoint = findCorePoint (currentPoint, sensorData, radius, minPoints)
    %INPUT
    % currentPoint = [xPos, yPos]
    %sensorData = [[xPos, yPos]]
    %radius = float
    %minPoint = int
    %OUTPUT
    %isCorePoint = Boolean

    %find the relative distance of all points in the Data to the current point
    relativeDistance = sqrt((sensorData(:,1) - currentPoint(1)).^2 + (sensorData(:,2) - currentPoint(2)).^2);

    %Find the number of neighbouring points within the radius of the core point
    neighbouringPointsLoc = find(relativeDistance <= radius);
    numNeighbours = numel(neighbouringPointsLoc);

    %Determine if current point is a core point
    if (numNeighbours >= minPoints)
        isCorePoint = true;
    else
        isCorePoint = false;
    end
end

%Group core points into clusters
function corePointClusters = groupCorePoints(corePointData, radius)
    %INPUT
    %corePointData = [[xPos, yPos]]
    %radius = float 
    %OUTPUT
    %corePointClusters = %[xPos, yPos, clusterNum]
    
    corePointClusters = []; 
    cluster = []; %[xPos, yPos, clusterNum]
    clusterNumber = 1;

    while size(corePointData, 1) ~= 0
        %Takes whatever is first in corePointData as starting point for cluster
        cluster = [corePointData(1,:) clusterNumber]; %[xPos yPos clusterNum]
        corePointData = corePointData(2:end, :); %remove the point from the dataset so we don't have duplicates
        counter = 1;

        
        while counter <= size(cluster,1)
            %Use the position counter of cluster as a pointer to compare neighbouring core points
            %Use the first two columns of cluster as it contains the xPos and yPos
            currentPoint = cluster(counter,1:2);
            dataDim = size(corePointData, 1);
           
            %Find the relative position of all remaining corePoints to the current point
            relativeDistance = sqrt((corePointData(:,1) - currentPoint(1)).^2 + (corePointData(:,2) - currentPoint(2)).^2);
        
            %Find the locations of neighbouring points in the data 
            neighbouringPointsLoc = relativeDistance <= radius;
        
            %Update cluster with new core points
            newCorePoints = corePointData(neighbouringPointsLoc, :);
            newPoints = [newCorePoints ones(size(newCorePoints,1),1)*clusterNumber]; %[xPos, yPos, clusterNum]
            cluster = [cluster; newPoints];
        
            %Delete the classified core points from the dataset
            corePointData(neighbouringPointsLoc, :) = [];
    
            counter = counter + 1;
        end
    
        %Update CorePointCluster
        corePointClusters = [corePointClusters; cluster]; 
        clusterNumber = clusterNumber + 1;
    end 
end

%Classifies border points into clusters
function clusterData = classifyBorderPoints(nonCorePointData, clusterData, radius)
    %INPUT
    % nonCorePointData = [[xpos, yPos]]
    %clusterData = [[xPos, yPos, clusterNum]]
    %radius = int
    %OUTPUT
    %clusterData = [[xPos, yPos, clusterNum]]
    
    %Loop through each non Core Point
    for i = 1 : size(nonCorePointData, 1)
        currentPoint = nonCorePointData(i, :);

        for j = 1 : size(clusterData, 1)
            %Calculate the distance between the core point and the current point
            corePoint = clusterData(j,1:2);
            relativeDistance1D = sqrt((corePoint(1) - currentPoint(1)).^2 + (corePoint(2) - currentPoint(2)).^2);
        
            %Determine if current point is close enough to core point to be
            %part of the core point's cluster
            if relativeDistance1D <= radius
                clusterNumber = clusterData(j,3);
                currentPointData = [currentPoint clusterNumber];
                clusterData = [clusterData(1:j, :) ; currentPointData; clusterData(j+1:end, :)];
                break
            end
        end
    end
end


%% Helper functions go here
function printSensorDets(frontCamDets, frontLeftRadarDetsInfo, frontRightRadarDetsInfo, lidarPointCloudInfo, clockTime)

    fprintf('\n\n--- SENSOR DETECTIONS, TIMESTEP = %4.2f ---\n', clockTime);

    fprintf("CAMERA DETECTIONS:\n");
    
    for i = 1:frontCamDets.NumDetections
        detection = frontCamDets.Detections(i);
        detectionState = detection.Measurement; %returns a state vector in the format [xPos, yPos, zPos, xVel, yVel, zVel]
        fprintf('CamDet%d - x = %4.3f, y = %4.3f\n', int8(i), detectionState(1), detectionState(2));
    end

    fprintf("LEFT RADAR DETECTIONS:\n");
    
    for i = 1:frontLeftRadarDetsInfo.NumDetections
        detection = frontLeftRadarDetsInfo.Detections(i);
        detectionState = detection.Measurement; %returns a state vector in the format [xPos, yPos, zPos, xVel, yVel, zVel]
        fprintf('LeftRadDet%d - x = %4.3f, y = %4.3f\n', int8(i), detectionState(1), detectionState(2));
    end

    fprintf("RIGHT RADAR DETECTIONS:\n");
    
    for i = 1:frontRightRadarDetsInfo.NumDetections
        detection = frontRightRadarDetsInfo.Detections(i);
        detectionState = detection.Measurement; %returns a state vector in the format [xPos, yPos, zPos, xVel, yVel, zVel]
        fprintf('RightRadDet%d - x = %4.3f, y = %4.3f\n', int8(i), detectionState(1), detectionState(2));
    end

    fprintf("LIDAR CLOUD DETECTIONS:\n");

    % lidarPointCloudInfo is an m x n x 3 array where m and n are vertical
    % and horizontal LIDAR channels (m x n is a point in the point cloud)
    % and it contains a 3-element array in the form [xPos, yPos, zPos]
    % representing the position of the point in the point cloud. m and n
    % depend on the resolution and FOV of the LIDAR. Invalid points or
    % points that are part of the road/ego vehicle have been filtered out
    % and are represented as [NaN, NaN, NaN] in the point cloud.
    lidarDims = size(lidarPointCloudInfo); 
    numValidPoints = 0;

    for i = 1:lidarDims(1)
        for j = 1:lidarDims(2)
            if ~(isnan(lidarPointCloudInfo(i, j, 1)) || isnan(lidarPointCloudInfo(i, j, 2)) || isnan(lidarPointCloudInfo(i, j, 3)))
                numValidPoints = numValidPoints + 1;
            end
        end
    end
    fprintf('Point cloud size - %d x %d x %d\n', int16(lidarDims(1)), int16(lidarDims(2)), int16(lidarDims(3)));
    fprintf('Number of valid points in point cloud is %d\n', int8(numValidPoints));

end
