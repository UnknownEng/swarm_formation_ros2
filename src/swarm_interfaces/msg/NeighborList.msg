std_msgs/Header header
uint32 robot_id
NeighborMeasurement[] neighbors
ObstacleMeasurement[] obstacles  # collision geometry (box surfaces, pillars, walls)
ObstacleMeasurement[] landmarks  # landmark centres for localization only
