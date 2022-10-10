function [track1, track2, track3, sensorsData, cluster1, cluster2, cluster3, centr1, centr2, centr3] = DBSCAN(frontCamDets, frontLeftRadarDetsInfo, frontRightRadarDetsInfo, lidarPointCloudInfo, clockTime)
    %% Format Sensor Data
    %Get Front camera data [xPos, yPos]
    frontCamera = zeros(frontCamDets.NumDetections,2);
    for i = 1:frontCamDets.NumDetections
        detection = frontCamDets.Detections(i);
        detectionState = detection.Measurement; 
        frontCamera(i,:)=[detectionState(1), detectionState(2)];
    end

    %Get left radar data [xPos, yPos]
    leftRadar = zeros(frontLeftRadarDetsInfo.NumDetections,2);
    for i = 1:frontLeftRadarDetsInfo.NumDetections
        detection = frontLeftRadarDetsInfo.Detections(i);
        detectionState = detection.Measurement; 
        leftRadar(i,:) = [detectionState(1), detectionState(2)];
    end

    %Get right radar data [xPos, yPos]
    rightRadar = zeros(frontRightRadarDetsInfo.NumDetections, 2);
    for i = 1:frontRightRadarDetsInfo.NumDetections
        detection = frontRightRadarDetsInfo.Detections(i);
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

    %% Algorithm- DBSCAN
    %Find Core Points
    
    %Note: you can adjust the radius and the minPoints values to get better results
    %Note: minPoints must be greater than or equal to 3 
    radius = 0.25;
    minPoints = 3;


    dataDim = size(sensorsData, 1);
    pointTypeData = [sensorsData zeros(dataDim,1)]; %[xPos, yPos, isCorePoint]

    for i = 1: dataDim
        pointTypeData(i,3) = findCorePoint(sensorsData(i,:), sensorsData, dataDim, radius, minPoints);
    end

    %Sort Data into Core Points and non Core Points
    corePointDataLoc = find(pointTypeData(:,3) == 1);
    corePointData = pointTypeData(corePointDataLoc, :);
    corePointData(:,3) = []; %remove the isCorePoint coloum
    

    nonCorePointDataLoc = find(pointTypeData(:,3) == 0);
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
        topCluster = mode(temp);
        topCluster = topCluster(3);
        temp(temp == topCluster) = NaN;
        secondCluster = mode(temp);
        secondCluster = secondCluster(3);
        temp(temp == secondCluster) = NaN;
        thirdCluster = mode(temp);
        thirdCluster = thirdCluster(3);
        clusters = [topCluster; secondCluster; thirdCluster];
    else
        clusters = clusterNum;
    end

    clusters = clusters';
 
    %Assign clusters to tracks. The centroid of the cluster points is used
    %as the position of the tracks
    if size(clusters, 2) >= 1
        track1Points = clusterData(clusterData(:,3) == clusters(1), [1:2]);
        centroid = mean(track1Points);
        track1 = [1 centroid(1) centroid(2)];
        cluster1 = track1Points;
        centr1 = centroid;
    else
        track1 = [0 inf inf];
        cluster1 = [0 0];
        centr1 = [0 0];
    end

    if size(clusters, 2) >= 2
        track2Points = clusterData(clusterData(:,3) == clusters(2), [1:2]);
        centroid = mean(track2Points);
        track2 = [1 centroid(1) centroid(2)];
        cluster2 = track2Points;
        centr2 = centroid;
    else
        track2 = [0 inf inf];
        cluster2 = [0 0];
        centr2 = [0 0];
    end

    if size(clusters, 2) >= 3
        track3Points = clusterData(clusterData(:,3) == clusters(3), [1:2]);
        centroid = mean(track3Points);
        track3 = [1 centroid(1) centroid(2)];
        cluster3 = track3Points;
        centr3 = centroid;
    else
        track3 = [0 inf inf];
        cluster3 = [0 0];
        centr3 = [0 0];
    end

end

%Determines if a point is a Core Point
function isCorePoint = findCorePoint (currentPoint, sensorData, dataDim, radius, minPoints)

    %Find the relative position of all other points to the current point
    relativePosition2D = abs(sensorData - currentPoint);

    %Convert the relative position to relative distance to the current point
    relativeDistance1D = sqrt(relativePosition2D(:,1).^2 + relativePosition2D(:,2).^2);

    %Find the number of neighbouring points within the radius of the core point
    neighbouringPointsLoc = find(relativeDistance1D <= radius);
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
    
    corePointClusters = [];
    cluster = [];
    clusterNumber = 1;

    while size(corePointData, 1) ~= 0
        %Takes whatever is first left in corePointData as starting point for cluster
        cluster = [corePointData(1,:) clusterNumber]; %[xPos yPos clusterNum]
        counter = 1;

        
        while counter <= size(cluster,1)
            %Use the position counter of cluster as the point to compare neighbouring core points
            %Use the first two coloms of cluster as it contains the xPos and yPos
            currentPoint = cluster(counter,1:2);
            dataDim = size(corePointData, 1);
            
            %Find the relative position of all other core points to the current point
            relativePosition2D = abs(corePointData - currentPoint);
        
            %Convert the relative position to relative distance to the current point
            relativeDistance1D = sqrt(relativePosition2D(:,1).^2 + relativePosition2D(:,2).^2);
        
            %Find the locations of neighbouring points in the data 
            neighbouringPointsLoc = relativeDistance1D <= radius;
            
        
            %Update cluster with new core points
            newCorePoints = corePointData(neighbouringPointsLoc, :);
    
            newPoints = [newCorePoints ones(size(newCorePoints,1),1)*clusterNumber];
            
            cluster = [cluster; newPoints];
        
            %Delete the classified core points from the dataset
            index = find(neighbouringPointsLoc);
            corePointData(index, :) = [];
    
            counter = counter + 1;
        end
    
        %Update CorePointCluster (ignore first row of cluster since it contains a duplicate)
        corePointClusters = [corePointClusters; cluster(2:end, :)];
        clusterNumber = clusterNumber + 1;
    end
    
end

%Classifies border points into clusters
function clusterData = classifyBorderPoints(nonCorePointData, clusterData, radius)

    %Loop through each non Core Point
    for i = 1 : size(nonCorePointData, 1)
        currentPoint = nonCorePointData(i, :);

        for j = 1 : size(clusterData, 1)
            %Find the relative position of a core point and current point
            corePoint = clusterData(j,[1:2]);
            relativePosition2D = abs(corePoint - currentPoint);
        
            %Convert the relative position to relative distance to the current point
            relativeDistance1D = sqrt(relativePosition2D(1).^2 + relativePosition2D(2).^2);
        
            %Determine if current point is close enough to core point to be
            %part of the core point's cluster
            if relativeDistance1D <= radius
                clusterNumber = clusterData(j,3);
                currentPointData = [currentPoint clusterNumber];
                clusterData = [clusterData(1:j, :) ; currentPointData; clusterData(j:end, :)];
                break
            end
        end
    end
end
