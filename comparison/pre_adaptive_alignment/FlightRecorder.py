"""Buffered flight diagnostics; no simulator access and no imports with side effects."""
import csv
from dataclasses import asdict
import datetime
import hashlib
import json
from pathlib import Path


class FlightRecorder:
    fields = ('t', 'mode', 'distance', 'speed', 'closing_speed', 'fuel', 'damage', 'score_if_arrived',
              'eta', 'full_thrust', 'coast_speed', 'nominal_mode', 'forward_command',
              'delivered_forward_accel', 'thrust_power', 'braking', 'angular_command_norm',
              'tracks', 'confirmed', 'radar_age',
              'rejected', 'ambiguous', 'threat_id', 'ttc', 'cpa_time', 'predicted_miss',
              'clearance', 'uncertainty', 'threat_age', 'baseline_clearance', 'candidates',
              'attitude_error_deg', 'spin_deg_s', 'linear_on', 'angular_on', 'linear_switches',
              'angular_switches', 'coast_fraction', 'fuel_per_dv', 'planning_accel', 'planner_ms',
              'x', 'y', 'z', 'vx', 'vy', 'vz', 'gx', 'gy', 'gz',
              'target_x', 'target_y', 'target_z', 'target_vx', 'target_vy', 'target_vz',
              'desired_ax', 'desired_ay', 'desired_az', 'command_ax', 'command_ay', 'command_az')

    def __init__(self, root, model):
        root = Path(root)
        stamp = datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
        self.directory = root / 'diagnostics' / stamp
        self.directory.mkdir(parents=True, exist_ok=False)
        self.telemetry_file = (self.directory / 'telemetry.csv').open('w', newline='', encoding='utf-8')
        self.radar_file = (self.directory / 'radar.csv').open('w', newline='', encoding='utf-8')
        self.telemetry = csv.DictWriter(self.telemetry_file, fieldnames=self.fields)
        self.telemetry.writeheader()
        self.radar = csv.writer(self.radar_file)
        self.radar.writerow(('received_t', 'measurement_t', 'rel_x', 'rel_y', 'rel_z',
                             'surface_range', 'range_rate', 'diameter'))
        hashes = {name: hashlib.sha256((root / name).read_bytes()).hexdigest()
                  for name in ('Controller.py', 'Guidance.py') if (root / name).exists()}
        (self.directory / 'model.json').write_text(
            json.dumps(dict(model=asdict(model), source_sha256=hashes,
                            units='metres, seconds, kilograms; Phobos axes',
                            evidence='Flight telemetry; predictions are estimates, not debris truth'), indent=2),
            encoding='utf-8')

    def observations(self, now, contacts):
        for contact in contacts:
            self.radar.writerow((now, contact.t_meas, *contact.rel_pos,
                                 contact.surface_range, contact.range_rate, contact.diameter))

    def record(self, values):
        self.telemetry.writerow({key: values.get(key, '') for key in self.fields})
        self.telemetry_file.flush()
        self.radar_file.flush()

    def close(self):
        for stream in (self.telemetry_file, self.radar_file):
            try:
                stream.close()
            except OSError:
                pass                       # diagnostics must not stop control on a full disk
