# DBSCAN
I explore the world of autonomous vehicles by implementing a sensor fusion algorithm with a set of given vehicle data. The MATLAB script file contains an implementation of a DBSCAN.

The demonstration.zip folder contains a screen recording of a simulation of the DBSCAN. The dots on the left panel represents sensor detections and the grey squares represent the tracks determined by the DBSCAN. 

## Outcome
For the most part, the implemented DBSCAN can accurately distinguish clusters and determine the tracks. However, once in a while, the simulation shows that the DBSCAN loses a cluster. There are two reasons for this to occur. 
1. Two clusters come too close to each other, and the DBSCAN can not distinguish the different clusters.
2. The sensors may pick up two different dense data points for the same vehicle. In this DBSCAN implementation, the maximum number of tracks is set to three. As a result, if the DBSCAN determines two tracks for the same vehicle, then there is a possibility to lose track of a different vehicle

## Future Work
In the future, the DBSCAN implementation will have a method to distinguish vehicles even when they are in close proximity to each other. This can be achieved through the optimization of the radius and the minimum number of points parameters. The DBSCAN will also be able to combine two dense data points that represent the same vehicle to reduce redudant tracks. 

