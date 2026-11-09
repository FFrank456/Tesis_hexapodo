"""Check the wire layout shared by firmware and Processing without hardware."""
import json
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
source = (ROOT / "main/main.c").read_text(encoding="utf-8")
literal = r'"(?:[^"\\]|\\.)*"'


def format_in(function):
    body = source.split("static void " + function + "(", 1)[1].split("\n}", 1)[0]
    match = re.search(r"printf\(\s*((?:" + literal + r"\s*)+)", body)
    assert match, function
    return "".join(json.loads(s) for s in re.findall(literal, match[1]))


expected = (
    "HEADER,time_ms,adc_raw,adc_voltage,current_raw_a,direction_code,direction_text,"
    "duty_cw_percent,duty_ccw_percent,adc_sample_count,pwm_frequency_hz,adc_sample_rate_hz,"
    "adc_sample_tick,control_mode_code,control_mode_text,current_ref_a,current_error_a,"
    "control_output_percent,duty_percent,pid_kp,pid_ki,pid_kd,pid_integrator,adc_calibrated,"
    "sample_time_us,encoder_count,sequence_id,sequence_index,sequence_start_us,sequence_stage_start_us,"
    "current_valid,current_status,adc_read_us,sample_phase_us,current_valid_count,sample_attempt_count,"
    "sample_missed_count,pwm_frequency_observed_hz,acquisition_rate_observed_hz,adc_rate_observed_hz,"
    "dma_overflows,sensing_version,dma_rejected_window,dma_rejected_quality,dma_phase_validated,"
    "adc_raw_mean,adc_voltage_mean,current_mean_a,dma_block_samples,encoder_version,encoder_time_us,"
    "encoder_relative_count_x4,output_position_rad,output_speed_rad_s,encoder_speed_valid,"
    "encoder_speed_window_us,counts_x4_per_output_rev,encoder_reference_count_x4,encoder_reference_id"
).split(",")
header = format_in("processing_print_header").strip().split(",")
wire = format_in("processing_print_telemetry").strip().split(",")
assert header == expected, "Header layout differs from the v6 consumer contract"
assert len(header) == len(wire) == 59, "Header / DATA field count mismatch"
assert wire[0] == "DATA"
assert all(len(re.findall(r"%[^%]*[a-zA-Z]", token)) == 1 for token in wire[1:])
settings = (ROOT / "main/current_sense.h").read_text(encoding="utf-8")
assert re.search(r"#define\s+SENSE_DATA_VERSION\s+6U", settings)
print("PASS: firmware HEADER/DATA v6 share 59 ordered fields, including encoder SI")
speed = format_in("processing_print_speed").strip().split(",")
assert len(speed) == 13 and speed[:2] == ["SPEED", "1"]
print("PASS: separate SPEED v1 record, 13 fields; existing DATA contract unchanged")
position = format_in("processing_print_position").strip().split(",")
assert len(position) == 16 and position[:2] == ["POSITION", "1"]
print("PASS: separate POSITION v1 record, 16 fields")
