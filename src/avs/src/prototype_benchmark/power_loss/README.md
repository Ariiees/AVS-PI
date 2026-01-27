# Power Loss Auto-Reboot Runner

Quick start (single automatic experiment run):
```
chmod +x run_1min_reboot_auto.sh
./run_1min_reboot_auto.sh
```

What this does:
- Starts AVS ingestion (3 subscribers) via `ros2 launch` and waits 60s.
- Reboots the system automatically.
- On boot, resumes, records recovery metrics, then exits.

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
- Starts `ros2 launch avs avs_store.launch.py` and samples a heartbeat plus latest data mtime/path under `/home/avs/DATA/SSD`.
- After 60s, issues a reboot and leaves the run marked as active.
- On next boot, the resume service detects the reboot (boot_id change), restarts ingestion, and waits for new data.
- Metrics are computed as:
  - Power cut interval = boot_time_after_reboot - last_heartbeat_before_reboot.
  - Recovery time = first_data_after_reboot - boot_time_after_reboot.
  - Data loss window = first_data_after_reboot - last_data_before_reboot.
  - Estimated write rate (B/s) from directory size samples before reboot.
  - Estimated lost bytes = write rate * data loss window.
  - Storage health: SMART/NVMe logs before/after reboot + XFS read-only check.
- Writes `power_loss_report.json` and `power_loss_report.txt`, then exits.

Notes:
- The reboot command defaults to `systemctl reboot`. Run as a user with permission.
- Override launch with `--launch-file` and `--launch-args` if needed.
- Storage metrics use `smartctl`, `nvme smart-log`, and `xfs_repair -n` on `/dev/nvme0n1p3` (override with `--device` and `--fs-type`).
- If the filesystem is mounted, the XFS check is skipped and noted in the report.
- The resume service runs as root to capture device metrics.
- The resume script sources `/opt/ros/<distro>/setup.bash` and `/home/avs/AVS-PI/install/setup.bash` if present.
- The resume service is safe to leave enabled; it exits if no active run exists.
- Reports land under `/home/avs/Log/power_loss/<run_id>/`.
