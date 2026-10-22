#!/usr/bin/env python3
"""Host checks for append-only output telemetry in tools/flight_log.py."""
import struct
import importlib.util
import unittest
from pathlib import Path

SPEC = importlib.util.spec_from_file_location('flight_log', Path(__file__).parents[1] / 'tools' / 'flight_log.py')
flight_log = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(flight_log)


def summary(length=69):
    p = bytearray(length)
    struct.pack_into('<IhHh', p, 0, 7, 45, 120, 80)
    p[28] = 1 | (3 << 2)
    if length >= 38:
        struct.pack_into('<h', p, 34, -885)
        p[36], p[37] = 0xa1, 1 | 2 | 4
    if length >= 50:
        struct.pack_into('<hhh', p, 42, 120, 110, 100)
    if length >= 69:
        p[50:54] = bytes((0xff, 2, 2, 6))
        struct.pack_into('<HH', p, 54, 500, 480)
        p[58:60] = bytes((97, 95))
        struct.pack_into('<HHHH', p, 60, 15625, 12, 3, 106)
        p[68] = 2
    return p


class FlightLogToolTest(unittest.TestCase):
    def test_output_telemetry_decodes_with_validity(self):
        row = flight_log.second(summary())
        self.assertEqual(row['output_demod'], 'U85Q-LP 1a09f5c092e8')
        self.assertEqual(row['output_level'], 6)
        self.assertEqual(row['output_standard'], 'PAL')
        self.assertEqual(row['output_coverage_pct'], 96)
        self.assertEqual(row['native_h_lock_pct'], 97)
        self.assertEqual(row['native_h_lines'], 15625)
        self.assertEqual(row['repaired_tips'], 12)
        self.assertEqual(row['longest_h_unlock_ms'], 106)
        self.assertEqual(row['output_observer_gaps'], 2)

    def test_old_record_keeps_output_unknown_and_same_columns(self):
        old = flight_log.second(summary(34))
        new = flight_log.second(summary())
        self.assertIsNone(old['output_demod'])
        self.assertIsNone(old['native_h_lock_pct'])
        self.assertEqual(old.keys(), new.keys())


if __name__ == '__main__':
    unittest.main()
