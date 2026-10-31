"""ROS-independent experiment CSV storage."""
import csv
from datetime import datetime, timezone
from pathlib import Path
import yaml

FIELDS = {
    'status': ['state', 'reason', 'error', 'init_samples', 'start_x', 'start_y',
               'goal_x', 'goal_y', 'raw_json'],
    'cmd_vel': ['linear_x', 'linear_y', 'angular_z'],
    'optitrack': ['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'],
    'uwb_position': ['x', 'y', 'valid', 'device_time_ms'],
    'robot_pose': ['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'],
}
COMMON = ['receive_ros_ns', 'receive_monotonic_ns', 'source_stamp_ns', 'frame_id',
          'value_valid']


class LogStore:
    def __init__(self, output_dir, metadata, stale_after_sec=0.6):
        root = Path(output_dir).expanduser()
        root.mkdir(parents=True, exist_ok=True)
        import tempfile
        self.directory = Path(tempfile.mkdtemp(
            prefix=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ_'), dir=root))
        self.metadata = dict(metadata, schema_version=1,
                             created_utc=datetime.now(timezone.utc).isoformat(),
                             parameter_snapshots={})
        self.stale_ns = int(stale_after_sec * 1e9)
        self.files, self.writers, self.latest = {}, {}, {}
        self.segment = 0
        self.previous_a = None
        self.open_csv('events', ['receive_ros_ns', 'receive_monotonic_ns',
                                'segment_id', 'event', 'start_x', 'start_y'])
        for stream, fields in FIELDS.items():
            self.open_csv(stream, COMMON + ['segment_id'] + fields)
        columns = ['receive_ros_ns', 'receive_monotonic_ns', 'segment_id']
        for stream, fields in FIELDS.items():
            columns += [f'{stream}_{key}' for key in
                        ['received', 'age_sec', 'stale'] + COMMON + ['segment_id'] + fields]
        self.open_csv('timeline', columns)
        self.save_metadata()

    def open_csv(self, name, fields):
        handle = (self.directory / f'{name}.csv').open('w', newline='', encoding='utf-8')
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        self.files[name], self.writers[name] = handle, writer

    def save_metadata(self):
        path = self.directory / 'metadata.yaml'
        temporary = path.with_suffix('.tmp')
        temporary.write_text(yaml.safe_dump(self.metadata, allow_unicode=True), encoding='utf-8')
        temporary.replace(path)

    def record(self, stream, values, ros_ns, mono_ns, source_ns=None, frame='', valid=True):
        if stream == 'status' and valid:
            a = (values.get('start_x'), values.get('start_y'))
            a = a if all(v is not None for v in a) else None
            if a != self.previous_a:
                if a is not None:
                    self.segment += 1
                self.writers['events'].writerow(dict(
                    receive_ros_ns=ros_ns, receive_monotonic_ns=mono_ns,
                    segment_id=self.segment, event='start_acquired' if a else 'start_cleared',
                    start_x=a[0] if a else None, start_y=a[1] if a else None))
                self.previous_a = a
        row = dict(values, receive_ros_ns=ros_ns, receive_monotonic_ns=mono_ns,
                   source_stamp_ns=source_ns, frame_id=frame, value_valid=valid,
                   segment_id=self.segment)
        self.writers[stream].writerow(row)
        self.latest[stream] = row

    def timeline(self, ros_ns, mono_ns):
        row = dict(receive_ros_ns=ros_ns, receive_monotonic_ns=mono_ns,
                   segment_id=self.segment)
        for stream in FIELDS:
            latest = self.latest.get(stream)
            row[f'{stream}_received'] = latest is not None
            if latest is not None:
                age_ns = max(0, mono_ns - latest['receive_monotonic_ns'])
                row[f'{stream}_age_sec'] = age_ns / 1e9
                row[f'{stream}_stale'] = age_ns > self.stale_ns
                row.update({f'{stream}_{k}': v for k, v in latest.items()})
        self.writers['timeline'].writerow(row)

    def flush(self):
        for handle in self.files.values():
            handle.flush()

    def close(self):
        for handle in self.files.values():
            handle.close()
