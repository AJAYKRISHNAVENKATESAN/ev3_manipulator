#!/usr/bin/env python3
# """Stage synchronization calibration and adaptive tuning.

# Tracks actual hardware/simulation timings and provides:
# - Per-stage correction factors
# - Dynamic speed scaling
# - Auto-calibration for future runs
# """

# import json
# import os
# from collections import defaultdict
# from pathlib import Path


# CALIBRATION_FILE = "/tmp/stage_calibration.json"


# class StageCalibration:
#     """Track timing data and compute adaptive corrections."""

#     def __init__(self, calibration_file=CALIBRATION_FILE):
#         self.calibration_file = calibration_file
#         self.current_run = {}  # cycle -> { stage -> { hw_ms, sim_ms, checkpoint_data } }
#         self.load_calibration()

#     def load_calibration(self):
#         """Load stored calibration from previous runs."""
#         if os.path.exists(self.calibration_file):
#             try:
#                 with open(self.calibration_file, 'r') as f:
#                     self.history = json.load(f)
#             except (json.JSONDecodeError, IOError):
#                 self.history = {}
#         else:
#             self.history = {}

#     def save_calibration(self):
#         """Save calibration data for next run."""
#         os.makedirs(os.path.dirname(self.calibration_file), exist_ok=True)
#         with open(self.calibration_file, 'w') as f:
#             json.dump(self.history, f, indent=2)

#     def record_stage_timing(self, cycle_id, stage, hardware_ms, sim_ms):
#         """Record actual timing for a stage."""
#         key = f"{cycle_id}_{stage}"
#         self.current_run[key] = {
#             "hardware_ms": hardware_ms,
#             "sim_ms": sim_ms,
#             "delta_ms": sim_ms - hardware_ms,
#         }

#     def record_checkpoint(self, cycle_id, stage, checkpoint_pct, hw_ms, sim_ms):
#         """Record intermediate checkpoint timing."""
#         key = f"{cycle_id}_{stage}_cp{checkpoint_pct}"
#         self.current_run[key] = {
#             "checkpoint": checkpoint_pct,
#             "hardware_ms": hw_ms,
#             "sim_ms": sim_ms,
#             "delta_ms": sim_ms - hw_ms,
#         }

#     def get_speed_factor(self, stage):
#         """
#         Get dynamic speed adjustment for this stage.
        
#         Returns: float
#             < 1.0 = sim is faster than hw, slow it down
#             > 1.0 = sim is slower than hw, speed it up
#         """
#         matching_keys = [
#             k for k in self.current_run.keys()
#             if stage in k and "cp" not in k
#         ]

#         if not matching_keys:
#             return 1.0  # No data, use default

#         total_delta = sum(
#             self.current_run[k].get("delta_ms", 0)
#             for k in matching_keys
#         )
#         mean_delta = total_delta / len(matching_keys)

#         if mean_delta == 0:
#             return 1.0

#         # If sim is 100ms faster, factor should be 0.9 to slow it down.
#         # factor = (requested_time - delta) / requested_time
#         # For mean_delta = 100ms and typical 2000ms action:
#         # factor = (2000 - 100) / 2000 = 0.95
#         return max(0.8, min(1.2, 1.0 - (mean_delta / 2000.0)))

#     def get_corrected_timing(self, stage_name, nominal_time_s):
#         """
#         Get corrected timing for a stage based on historical drift.
        
#         Args:
#             stage_name: e.g. "HOME_AFTER_RED", "PICK_DOWN"
#             nominal_time_s: default timing in seconds
        
#         Returns: float
#             Corrected time in seconds
#         """
#         # Look for matching stages in history
#         pattern_matches = [
#             k for k in self.history.keys()
#             if stage_name in k
#         ]

#         if not pattern_matches:
#             return nominal_time_s

#         # Compute mean correction ratio
#         corrections = []
#         for key in pattern_matches:
#             entry = self.history[key]
#             if "sim_ms" in entry and "hardware_ms" in entry:
#                 hw = entry["hardware_ms"]
#                 sim = entry["sim_ms"]
#                 if hw > 0:
#                     corrections.append(sim / hw)

#         if not corrections:
#             return nominal_time_s

#         mean_ratio = sum(corrections) / len(corrections)
#         corrected_ms = nominal_time_s * 1000 * mean_ratio
#         return corrected_ms / 1000.0

#     def finalize_run(self):
#         """Store current run data in history for future calibration."""
#         for key, data in self.current_run.items():
#             self.history[key] = data
#         self.save_calibration()
#         print(f"[CAL] Calibration saved: {len(self.current_run)} stages tracked")

#     def report(self):
#         """Print calibration report."""
#         if not self.current_run:
#             return

#         print("\n[CAL] === Timing Analysis ===")
#         stages_seen = defaultdict(list)

#         for key, data in sorted(self.current_run.items()):
#             stage = key.rsplit("_", 1)[0]
#             stages_seen[stage].append(data)

#         for stage in sorted(stages_seen.keys()):
#             entries = stages_seen[stage]
#             count = len(entries)
#             mean_hw = sum(e.get("hardware_ms", 0) for e in entries) / count
#             mean_sim = sum(e.get("sim_ms", 0) for e in entries) / count
#             mean_delta = mean_sim - mean_hw

#             ratio = mean_sim / mean_hw if mean_hw > 0 else 1.0

#             status = "FASTER" if mean_delta < 0 else "SLOWER"
#             print(
#                 f"  {stage:30s} | "
#                 f"HW={mean_hw:6.0f}ms | "
#                 f"SIM={mean_sim:6.0f}ms | "
#                 f"Δ={mean_delta:+6.0f}ms ({status}) | "
#                 f"ratio={ratio:.2f}"
#             )


#!/usr/bin/env python3
"""Stage synchronization calibration and adaptive tuning.

Tracks actual hardware/simulation timings and provides:
- Per-stage correction factors
- Dynamic speed scaling
- Auto-calibration for future runs
"""

import json
import os
from collections import defaultdict
from pathlib import Path


CALIBRATION_FILE = "/tmp/stage_calibration.json"


class StageCalibration:
    """Track timing data and compute adaptive corrections."""

    def __init__(self, calibration_file=CALIBRATION_FILE):
        self.calibration_file = calibration_file
        self.current_run = {}  # cycle -> { stage -> { hw_ms, sim_ms, checkpoint_data } }
        self.load_calibration()

    def load_calibration(self):
        """Load stored calibration from previous runs."""
        if os.path.exists(self.calibration_file):
            try:
                with open(self.calibration_file, 'r') as f:
                    self.history = json.load(f)
            except (json.JSONDecodeError, IOError):
                self.history = {}
        else:
            self.history = {}

    def save_calibration(self):
        """Save calibration data for next run."""
        os.makedirs(os.path.dirname(self.calibration_file), exist_ok=True)
        with open(self.calibration_file, 'w') as f:
            json.dump(self.history, f, indent=2)

    def record_stage_timing(self, cycle_id, stage, hardware_ms, sim_ms):
        """Record actual timing for a stage."""
        key = f"{cycle_id}_{stage}"
        self.current_run[key] = {
            "hardware_ms": hardware_ms,
            "sim_ms": sim_ms,
            "delta_ms": sim_ms - hardware_ms,
        }

    def record_checkpoint(self, cycle_id, stage, checkpoint_pct, hw_ms, sim_ms):
        """Record intermediate checkpoint timing."""
        key = f"{cycle_id}_{stage}_cp{checkpoint_pct}"
        self.current_run[key] = {
            "checkpoint": checkpoint_pct,
            "hardware_ms": hw_ms,
            "sim_ms": sim_ms,
            "delta_ms": sim_ms - hw_ms,
        }

    def get_speed_factor(self, stage):
        """
        Get dynamic speed adjustment for this stage.

        Returns: float
            < 1.0 = sim is faster than hw, slow it down
            > 1.0 = sim is slower than hw, speed it up
        """
        matching_keys = [
            k for k in self.current_run.keys()
            if stage in k and "cp" not in k
        ]

        if not matching_keys:
            return 1.0  # No data, use default

        total_delta = sum(
            self.current_run[k].get("delta_ms", 0)
            for k in matching_keys
        )
        mean_delta = total_delta / len(matching_keys)

        if mean_delta == 0:
            return 1.0

        # If sim is 100ms faster, factor should be 0.9 to slow it down.
        # factor = (requested_time - delta) / requested_time
        # For mean_delta = 100ms and typical 2000ms action:
        # factor = (2000 - 100) / 2000 = 0.95
        return max(0.8, min(1.2, 1.0 - (mean_delta / 2000.0)))

    def get_corrected_timing(self, stage_name, nominal_time_s):
        """
        Get the duration the sim should REQUEST for this stage.

        Strategy: request the hardware's measured duration for this stage.
        In the trackable regime the controller finishes at ~time_from_start,
        so requesting hw_ms makes sim_elapsed ~= hw_ms directly. The result
        does NOT depend on the previous sim_ms, so there is nothing to
        oscillate (unlike a sim/hw-ratio correction).

        Args:
            stage_name:    e.g. "HOME_AFTER_RED", "PICK_DOWN"
            nominal_time_s: hand-tuned default in seconds (used as fallback
                            and to bound the correction)

        Returns: float
            Corrected request time in seconds, clamped to
            [0.5 * nominal, 2.0 * nominal] so a bad reading can never
            send the request wildly away from the hand-tuned value.
        """
        # Match stages in history (ignore checkpoint entries).
        matches = [
            k for k in self.history
            if stage_name in k and "cp" not in k
        ]

        hw = [
            self.history[k]["hardware_ms"]
            for k in matches
            if self.history[k].get("hardware_ms", 0) > 0
        ]

        if not hw:
            # No history yet (e.g. first run) -> run at the hand-tuned value.
            return nominal_time_s

        target_s = (sum(hw) / len(hw)) / 1000.0

        lo = 0.5 * nominal_time_s
        hi = 2.0 * nominal_time_s
        return max(lo, min(hi, target_s))

    def finalize_run(self):
        """Store current run data in history for future calibration."""
        for key, data in self.current_run.items():
            self.history[key] = data
        self.save_calibration()
        print(f"[CAL] Calibration saved: {len(self.current_run)} stages tracked")

    def report(self):
        """Print calibration report."""
        if not self.current_run:
            return

        print("\n[CAL] === Timing Analysis ===")
        stages_seen = defaultdict(list)

        for key, data in sorted(self.current_run.items()):
            stage = key.rsplit("_", 1)[0]
            stages_seen[stage].append(data)

        for stage in sorted(stages_seen.keys()):
            entries = stages_seen[stage]
            count = len(entries)
            mean_hw = sum(e.get("hardware_ms", 0) for e in entries) / count
            mean_sim = sum(e.get("sim_ms", 0) for e in entries) / count
            mean_delta = mean_sim - mean_hw

            ratio = mean_sim / mean_hw if mean_hw > 0 else 1.0

            status = "FASTER" if mean_delta < 0 else "SLOWER"
            print(
                f"  {stage:30s} | "
                f"HW={mean_hw:6.0f}ms | "
                f"SIM={mean_sim:6.0f}ms | "
                f"Δ={mean_delta:+6.0f}ms ({status}) | "
                f"ratio={ratio:.2f}"
            )
