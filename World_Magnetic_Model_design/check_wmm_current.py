#!/usr/bin/env python3
"""Check that the World Magnetic Model in WMM.COF is still current.

Fails (exit code 1) if
  * NOAA NCEI lists a newer model than the one in WMM.COF
    (e.g. WMM2030, or an out-of-cycle release like WMM2015v2), or
  * the model in WMM.COF has expired (more than five years after its epoch).
Exit code 2 means the check itself could not be done (network error, or the
NOAA page no longer looks as expected).

Usage (from the repository root):
    python3 World_Magnetic_Model_design/check_wmm_current.py
    python3 World_Magnetic_Model_design/check_wmm_current.py --page-file saved.html --date 2030-01-15

The update procedure is described in World_Magnetic_Model_design/README.md
(replace WMM.COF, rerun make_wmm_coefficients.py).
"""

import argparse
import datetime
import os
import re
import sys
import urllib.request

WMM_PAGE = "https://www.ncei.noaa.gov/products/world-magnetic-model"
DEFAULT_COF = os.path.join(os.path.dirname(os.path.abspath(__file__)), "WMM.COF")
VALIDITY_YEARS = 5

# "WMM2025", "WMM 2025", "WMM-2025", "WMM2015v2", "WMM2025COF.zip";
# not the high resolution model "WMMHR2025"
MODEL_PATTERN = re.compile(r"(?<![A-Za-z])WMM[ -]?((?:19|20)\d{2})(?:v(\d+))?(?!\d)")


class CheckError(Exception):
    """The check could not be done."""


def model_key(year, version):
    """Sort key: WMM2015 < WMM2015v2 < WMM2020."""
    return (int(year), int(version) if version else 1)


def model_name(key):
    year, version = key
    return f"WMM{year}" + (f"v{version}" if version > 1 else "")


def read_cof_model(path):
    """Return (epoch, key) of the model in a WMM.COF file."""
    with open(path) as f:
        fields = f.readline().split()
    if len(fields) < 2:
        raise CheckError(f"{path}: no header line")
    match = MODEL_PATTERN.search(fields[1])
    if not match:
        raise CheckError(f"{path}: unexpected model name {fields[1]!r}")
    return float(fields[0]), model_key(*match.groups())


def listed_models(page):
    """All model versions mentioned on the NOAA page."""
    return {model_key(*m) for m in MODEL_PATTERN.findall(page)}


def fetch_page(url):
    request = urllib.request.Request(url, headers={"User-Agent": "Mozilla/5.0 (larus WMM check)"})
    try:
        with urllib.request.urlopen(request, timeout=60) as response:
            return response.read().decode("utf-8", errors="replace")
    except OSError as error:
        raise CheckError(f"cannot fetch {url}: {error}")


def check(cof_epoch, cof_key, page, today):
    """Return a list of problems (empty if the model is current)."""
    models = listed_models(page)
    # Sanity check: the page must still mention the model we use, otherwise
    # its structure has changed and "no newer model" would mean nothing.
    if not any(year == cof_key[0] for year, _ in models):
        raise CheckError(f"the NOAA page does not mention {model_name(cof_key)} (page changed?)")

    problems = []
    decimal_year = today.year + (today.timetuple().tm_yday - 1) / 365.25
    # A new model is released a few weeks before its epoch (WMM2025: 11/2024).
    # Ignore mentions of models further in the future (announcements).
    newer = sorted(k for k in models if k > cof_key and k[0] <= today.year + 1)
    if newer:
        problems.append(f"a newer model is available: {', '.join(model_name(k) for k in newer)}"
                        f" (WMM.COF contains {model_name(cof_key)})")
    if decimal_year >= cof_epoch + VALIDITY_YEARS:
        problems.append(f"{model_name(cof_key)} expired at {cof_epoch + VALIDITY_YEARS:.1f}"
                        f" (today is {today.isoformat()})")
    return problems


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--cof", default=DEFAULT_COF, help="coefficient file (default: WMM.COF next to this script)")
    parser.add_argument("--page-file", help="read the NOAA page from this file instead of fetching it")
    parser.add_argument("--url", default=WMM_PAGE, help="NOAA page to fetch")
    parser.add_argument("--date", type=datetime.date.fromisoformat, default=datetime.date.today(),
                        help="check as of this date (YYYY-MM-DD, default: today)")
    args = parser.parse_args(argv)

    try:
        cof_epoch, cof_key = read_cof_model(args.cof)
        if args.page_file:
            with open(args.page_file, encoding="utf-8", errors="replace") as f:
                page = f.read()
        else:
            page = fetch_page(args.url)
        problems = check(cof_epoch, cof_key, page, args.date)
    except CheckError as error:
        print(f"ERROR: WMM check could not be done: {error}", file=sys.stderr)
        return 2

    if problems:
        for problem in problems:
            print(f"FAIL: {problem}", file=sys.stderr)
        print("Update WMM.COF as described in World_Magnetic_Model_design/README.md.", file=sys.stderr)
        return 1
    print(f"OK: {model_name(cof_key)} (epoch {cof_epoch:.1f}) is the newest model listed at {args.url}"
          f" and valid until {cof_epoch + VALIDITY_YEARS:.1f}.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
