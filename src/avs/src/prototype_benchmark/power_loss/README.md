# Power Loss Auto-Reboot Runner

Quick start (single automatic experiment run):
```
chmod +x run_1min_reboot_auto.sh
./run_1min_reboot_auto.sh
```

What this does:
- Starts AVS ingestion (3 subscribers) via `ros2 launch` and waits 60s.
- Reboots the system automatically.
- On boot, validates durability (CRC + truncation) and rebuilds `global.sqlite3`, then exits.

Manual/advanced:
```
sudo cp power_loss_autorun.service /etc/systemd/system/
sudo systemctl enable power_loss_autorun.service
chmod +x run_1min_reboot.sh
./run_1min_reboot.sh
```

To stop/disable the resume service:
```
sudo systemctl stop power_loss_autorun.service
sudo systemctl disable power_loss_autorun.service
```

Experiment logic:
- Creates a new run dir under `/home/avs/Log/power_loss/<run_id>/` and records `state.json`.
- Starts `ros2 launch avs avs_store.launch.py` and samples a heartbeat plus latest data mtime/path under `/home/avs/DATA/SSD` (excluding `global.sqlite3`).
- After 60s, issues a reboot and leaves the run marked as active.
- On next boot, the resume service detects the reboot (boot_id change) and performs recovery checks only.
- Metrics are computed as:
  - Power cut interval = boot_time_after_reboot - last_heartbeat_before_reboot.
  - Recovery time = time to complete recovery checks after boot.
  - Data loss window = first_recovered_valid_record - last_durable_record (may be 0 if no new data).
  - Estimated write rate (B/s) from directory size samples before reboot.
  - Estimated lost bytes = write rate * data loss window.
  - Last durable record before crash = last complete chunk end_ts from log (or `.idx` fallback).
  - First recovered valid record after reboot = first complete chunk seen after reboot (from the same log scan).
  - CRC validation summary = prefix CRC32 of the trip log up to the last persisted chunk boundary (from log scan).
  - SQLite recovery = rebuilds missing `end_ts_ns`/`number_of_records` from `.idx` after reboot.
- Writes `power_loss_report.json` and `power_loss_report.txt`, then exits.

Notes:
- The reboot command defaults to `systemctl reboot`. Run as a user with permission.
- Override launch with `--launch-file` and `--launch-args` if needed.
- The resume script sources `/opt/ros/<distro>/setup.bash` and `/home/avs/AVS-PI/install/setup.bash` if present.
- The resume service is safe to leave enabled; it exits if no active run exists.
- Reports land under `/home/avs/Log/power_loss/<run_id>/`.
