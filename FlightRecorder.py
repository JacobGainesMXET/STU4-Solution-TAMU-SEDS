"""Buffered flight diagnostics; no simulator access and no imports with side effects."""
import csv
from dataclasses import asdict
import datetime
import hashlib
import json
from pathlib import Path


class FlightRecorder:
    fields = ('t', 'mode', 'no_debris_test', 'radar_observation_practice', 'radar_avoidance_enabled',
              'distance', 'speed', 'closing_speed', 'fuel', 'damage', 'score_if_arrived',
              'eta', 'full_thrust', 'bottom_lift_burn', 'thrust_from_lift_axis_deg',
              'coast_speed', 'transit_reason', 'coast_arrival_s',
              'speed_cutoff_reached', 'velocity_error_deg',
              'guidance_burn_s', 'guidance_burn_plans', 'reburn_advantage',
              'route', 'route_reason', 'cruise_reason', 'launch_burn', 'clear_departure', 'turn_rate_limit_deg_s',
              'pending_route', 'progress', 'distance_coast',
              'route_tracks', 'route_candidates', 'route_eta_s',
              'route_review_t', 'route_ms', 'route_unresolved',
              'route_fuel_kg', 'route_score', 'route_delta_fuel_kg', 'route_burns', 'route_burn_s',
              'route_dv', 'route_threats', 'route_track', 'route_cpa_s', 'route_miss_m', 'route_sigma_m',
              'route_direct_clearance_m', 'route_clearance_m', 'route_selected_miss_m', 'route_selected_sigma_m',
              'route_selected_cpa_s', 'route_angle_deg',
              'route_leg_left_s', 'route_side_x', 'route_side_y', 'route_side_z',
              'flyby_offset_x', 'flyby_offset_y', 'flyby_offset_z',
              'avoidance_strategy', 'avoidance_burn_s', 'forward_avoidance', 'body_lateral_command',
              'nominal_mode', 'forward_command',
              'delivered_forward_accel', 'thrust_power', 'braking', 'angular_command_norm',
              'nose_error_deg', 'alignment_priority', 'angular_accel_limit', 'angular_brake_limit',
              'nose_tolerance_deg', 'quiet_coast', 'coast_entry_s',
              'tracks', 'confirmed', 'radar_age',
              'rejected', 'ambiguous', 'threat_id', 'ttc', 'cpa_time', 'predicted_miss',
              'clearance', 'uncertainty', 'threat_age', 'baseline_clearance', 'hull_gap', 'baseline_hull_gap', 'candidates',
              'attitude_error_deg', 'hull_rotation_deg', 'spin_deg_s', 'linear_on', 'angular_on', 'linear_switches',
              'angular_switches', 'coast_fraction', 'fuel_per_dv', 'planning_accel', 'planner_ms',
              'x', 'y', 'z', 'vx', 'vy', 'vz', 'gx', 'gy', 'gz',
              'nose_x', 'nose_y', 'nose_z',
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
                  for name in ('Controller.py', 'Guidance.py', 'RoutePlanner.py') if (root / name).exists()}
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
