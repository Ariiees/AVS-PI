# udp_offload_send.cpp 
## Usage
```
udp_offload_send --dst_ip <ip> --dst_port <port>
                --topic <sensor_topic> --start <ts_ns> --end <ts_ns>
                [--ssd_root <path>] [--max_records <n>]
                [--mtu_payload <bytes>] [--batch_us <us>]
```
## Behavior
Queries AVS via avs::RetrieveAPI for DataRef in [start,end] and streams payloads over UDP.

Each UDP packet carries request id, packet seq, record count, then per record ts_ns and raw bytes.

Packets are MTU safe by bounding payload size.

## Output
Prints sender summary metrics to stdout at the end.