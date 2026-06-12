#!/usr/bin/env python3
"""Scale all <mass value="..."/> in a URDF by a factor, excluding a named link."""

import sys
import re
import argparse


def main():
    parser = argparse.ArgumentParser(description="Scale URDF mass values")
    parser.add_argument("--scale", type=float, required=True, help="Mass scale factor")
    parser.add_argument("--exclude-link", action="append", default=[], dest="exclude_links",
                        help="Link name to exclude from scaling (repeatable)")
    args = parser.parse_args()

    urdf_text = sys.stdin.read()

    # Split into links, scale mass inside each unless excluded
    link_re = re.compile(r'(<link\s+name="([^"]+)"[^>]*>.*?</link>)', re.DOTALL)
    mass_re = re.compile(r'(<mass\s+[^>]*?value=")([^"]+)(")', re.DOTALL)

    def scale_link(match):
        link_xml = match.group(1)
        link_name = match.group(2)
        if link_name in args.exclude_links:
            return link_xml
        def scale_mass(m):
            old_val = float(m.group(2))
            new_val = old_val * args.scale
            if new_val == 0.0 or abs(new_val) < 1e-9:
                fmt = f"{new_val:.1e}"
            elif abs(new_val) >= 100:
                fmt = f"{new_val:.1f}"
            else:
                fmt = f"{new_val:.6g}"
            return m.group(1) + fmt + m.group(3)
        return mass_re.sub(scale_mass, link_xml)

    urdf_text = link_re.sub(scale_link, urdf_text)
    sys.stdout.write(urdf_text)


if __name__ == "__main__":
    main()

