lio-sam current AutoRacer entry:
ros2 launch lio_sam autoracer_run.launch.py

required inputs:
- /point_cloud_raw
- /imu/data_raw

notes:
- legacy Wheeltec-based run*.launch.py wrappers have been removed
- GNSS/offline launch wrappers are not provided in the current AutoRacer tree

## Save map
```
ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap
```
```
ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap "{resolution: 0.2, destination: /Downloads/service_LOAM}"
```
