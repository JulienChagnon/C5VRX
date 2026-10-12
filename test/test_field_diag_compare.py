"""C5VRX-5 by Twotoz and contributors: diagnostic log semantics regression."""
import importlib.util
from pathlib import Path
import tempfile
import unittest

spec = importlib.util.spec_from_file_location("compare", Path(__file__).parents[1] / "tools/field_diag_compare.py")
compare = importlib.util.module_from_spec(spec)
spec.loader.exec_module(compare)

class CompareTest(unittest.TestCase):
    def summary(self, text):
        # A directory, not NamedTemporaryFile: Windows cannot reopen an open temp file by name.
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "field.log"
            path.write_text(text)
            return compare.summary(str(path))

    def test_partial_coverage_and_trace_not_seconds(self):
        r = self.summary("FIELD_DIAG t=1 sync_valid_pct=50 measurement_coverage_pct=50 sync_given_measured_pct=100 gain=70 max=83\n"
                         "FIELD_DIAG t=2 sync_valid_pct=0 measurement_coverage_pct=0 gain=70 max=83\n"
                         "FIELD_TRACE t=1 t_ms=1050 sync_valid_pct=0 measurement_coverage_pct=100\n")
        self.assertEqual(r["seconds"], 2)
        self.assertEqual(r["sync_evidence_ok_pct"], 50)
        self.assertEqual(r["coverage_pct_median"], 25)

    def test_legacy_unknown_coverage(self):
        r = self.summary("FIELD_DIAG t=1 video_pct=100 gain=80 max=83\n")
        self.assertEqual(r["legacy_unknown_coverage_s"], 1)
        self.assertNotEqual(r["sync_evidence_ok_pct"], 100)

    def test_empty(self):
        self.assertEqual(self.summary("")["seconds"], 0)

if __name__ == "__main__":
    unittest.main()
