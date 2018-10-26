Published topics:
1. `/vicon/<object-name>` of type `geometry_msgs/PoseStamped`.
1. `/vicon/markers` of type `sensor_msgs/PointCloud2`.

Parameters:
1. `vicon_host_name`: (required) hostname of Vicon machine.
1. `vicon_object_name`: (optional) only the specified object is published.
1. `enable_objects`: (default: true) publish objects.
1. `enable_markers`: (default: false) publish unlabeled markers. (_Note_: Object tracking mode must be disabled in Vicon Tracker in order to stream marker data)
