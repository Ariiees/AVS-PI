# ENV SET
If you are using pi, please set the following environment parameter in current terminal.

```
export LIBGL_ALWAYS_SOFTWARE=1
```
# list

Will list recorded data with sensor_id (default is topic name), data_type (image defalut is jpg, lidar default is laz), timestamp in 13 digitals ms and stored path.

```
ros2 run avs retrieve_cli --data image --start 2025-8-20_15-59 --end 2025-8-20_16-00 --list
ros2 run avs retrieve_cli --data lidar --start 2025-8-20_15-59 --end 2025-8-20_16-00 --list
```
# view
Will display all the data in the start to end time range.
```
ros2 run avs retrieve_cli --data lidar --start 2025-8-20_15-59 --end 2025-8-20_16-00 --list
ros2 run avs retrieve_cli --data image --start 2025-8-20_15-59 --end 2025-8-20_16-00 --view
```