## Build
```
g++ -O2 -std=c++17 offload_sender_tcp_cloud_batch.cpp -o offload_sender_tcp_cloud_batch
```
## Run
```
ros2 run avs offload <dst_ip> <dst_port> <start_ts_ns> <end_ts_ns> <topics_csv> <max_records> [runs]
```
