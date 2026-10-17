#!/usr/bin/env python3
"""Offline tests for check_wmm_current.py (python3 -m unittest, or pytest)."""

import datetime
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import check_wmm_current as wmm  # noqa: E402

PAGE_2025 = """
<h2>WMM2025</h2><p>The WMM2025 is valid until late 2029.</p>
<a href="/wmm/WMM2025COF.zip">WMM2025 coefficients</a>
<a href="/wmm/WMMHR2025.zip">WMMHR2025 (high resolution)</a>
<li>WMM 2020</li><li>WMM 2015v2</li><li>WMM 2015</li><li>WMM 2010</li>
"""
PAGE_2030 = PAGE_2025.replace("<h2>WMM2025</h2>", "<h2>WMM2030</h2><a>WMM2030COF.zip</a>")
PAGE_2025_V2 = PAGE_2025 + "<p>WMM2025v2 released out of cycle</p>"

TODAY = datetime.date(2026, 9, 28)
WMM2025 = (2025.0, (2025, 1))


class CheckTest(unittest.TestCase):

    def test_current_model(self):
        self.assertEqual(wmm.check(*WMM2025, PAGE_2025, TODAY), [])

    def test_newer_model_fails(self):
        problems = wmm.check(*WMM2025, PAGE_2030, datetime.date(2029, 12, 20))
        self.assertEqual(len(problems), 1)
        self.assertIn("WMM2030", problems[0])

    def test_out_of_cycle_version_fails(self):
        problems = wmm.check(*WMM2025, PAGE_2025_V2, TODAY)
        self.assertEqual(len(problems), 1)
        self.assertIn("WMM2025v2", problems[0])

    def test_announcement_far_ahead_is_ignored(self):
        page = PAGE_2025 + "<p>The next model, WMM2030, will be released in December 2029.</p>"
        self.assertEqual(wmm.check(*WMM2025, page, TODAY), [])

    def test_expired_model_fails(self):
        problems = wmm.check(*WMM2025, PAGE_2025, datetime.date(2030, 1, 2))
        self.assertEqual(len(problems), 1)
        self.assertIn("expired", problems[0])

    def test_old_model_on_current_page_fails_twice(self):
        problems = wmm.check(2020.0, (2020, 1), PAGE_2025, TODAY)
        self.assertEqual(len(problems), 2)

    def test_high_resolution_model_is_not_a_newer_model(self):
        self.assertNotIn((2030, 1), wmm.listed_models("WMMHR2030"))

    def test_changed_page_is_an_error(self):
        with self.assertRaises(wmm.CheckError):
            wmm.check(*WMM2025, "<html>moved</html>", TODAY)

    def test_cof_header(self):
        cof = os.path.join(os.path.dirname(os.path.abspath(__file__)), "WMM.COF")
        epoch, key = wmm.read_cof_model(cof)
        self.assertEqual(epoch, float(key[0]))

    def test_model_order(self):
        self.assertLess(wmm.model_key("2015", None), wmm.model_key("2015", "2"))
        self.assertLess(wmm.model_key("2015", "2"), wmm.model_key("2020", None))


if __name__ == "__main__":
    unittest.main()
