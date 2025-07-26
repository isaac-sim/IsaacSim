#!/usr/bin/env python3
import argparse
import glob
import json
import os
import subprocess
import sys
import xml.etree.ElementTree as ET
from collections import defaultdict
from datetime import datetime


def load_config(config_path):
    with open(config_path, "r") as f:
        return json.load(f)


def match_patterns(filename, patterns):
    return any(glob.fnmatch.fnmatch(filename, pat) for pat in patterns)


def discover_scripts(test_dir):
    """Discover test scripts in the given directory.

    On Windows, looks for .bat files. On other platforms, looks for .sh files.

    Args:
        test_dir: Directory to search for test scripts.

    Returns:
        List of absolute paths to test scripts.
    """
    if sys.platform.startswith("win"):
        pattern = "*.bat"
    else:
        pattern = "*.sh"
    return [os.path.abspath(f) for f in glob.glob(os.path.join(test_dir, pattern))]


def categorize_and_bucket(scripts, config):
    suites = defaultdict(lambda: defaultdict(list))
    uncategorized = []
    script_to_suite_bucket = {}
    for script in scripts:
        fname = os.path.basename(script)
        assigned = False
        for suite in config["suites"]:
            if suite["include"] and not match_patterns(fname, suite["include"]):
                continue
            if suite.get("exclude") and match_patterns(fname, suite["exclude"]):
                continue
            # In suite, now bucket
            bucketed = False
            for bucket in suite.get("buckets", []):
                if match_patterns(fname, bucket["include"]):
                    suites[suite["name"]][bucket["name"]].append(script)
                    script_to_suite_bucket[script] = (suite["name"], bucket["name"])
                    bucketed = True
                    break
            if not bucketed:
                default_bucket = suite.get("default_bucket", "other")
                suites[suite["name"]][default_bucket].append(script)
                script_to_suite_bucket[script] = (suite["name"], default_bucket)
            assigned = True
            break
        if not assigned:
            uncategorized.append(script)
    return suites, uncategorized, script_to_suite_bucket


def run_scripts(suites, dry_run=False, print_live_output=False):
    results = defaultdict(lambda: defaultdict(list))
    for suite, buckets in suites.items():
        for bucket, scripts in buckets.items():
            for script in scripts:
                if dry_run:
                    results[suite][bucket].append(
                        {"script": script, "status": "dry-run", "exit_code": None, "stdout": "", "stderr": ""}
                    )
                else:
                    try:
                        if print_live_output:
                            print(f"\n===== Running: {script} =====")
                            proc = subprocess.run([script], text=True, shell=True)
                            status = "pass" if proc.returncode == 0 else "fail"
                            # No capture, just mark status
                            results[suite][bucket].append(
                                {
                                    "script": script,
                                    "status": status,
                                    "exit_code": proc.returncode,
                                    "stdout": "",
                                    "stderr": "",
                                }
                            )
                        else:
                            proc = subprocess.run([script], capture_output=True, text=True, shell=True)
                            status = "pass" if proc.returncode == 0 else "fail"
                            results[suite][bucket].append(
                                {
                                    "script": script,
                                    "status": status,
                                    "exit_code": proc.returncode,
                                    "stdout": proc.stdout,
                                    "stderr": proc.stderr,
                                }
                            )
                    except Exception as e:
                        results[suite][bucket].append(
                            {"script": script, "status": "fail", "exit_code": None, "stdout": "", "stderr": str(e)}
                        )
    return results


def generate_junit_report(results, output_path, suite_name, bucket_name):
    testsuite = ET.Element("testsuite", name=f"{suite_name}.{bucket_name}")
    total = 0
    failures = 0
    errors = 0
    for testcase in results[suite_name][bucket_name]:
        tc = ET.SubElement(testsuite, "testcase", name=os.path.basename(testcase["script"]))
        total += 1
        if testcase["status"] == "fail":
            failures += 1
            fail = ET.SubElement(tc, "failure", message=f'Exit code: {testcase["exit_code"]}')
            fail.text = testcase["stderr"] or testcase["stdout"]
        elif testcase["status"] not in ("pass", "dry-run"):
            errors += 1
            err = ET.SubElement(tc, "error", message="Error")
            err.text = testcase["stderr"] or testcase["stdout"]
    testsuite.set("tests", str(total))
    testsuite.set("failures", str(failures))
    testsuite.set("errors", str(errors))
    tree = ET.ElementTree(testsuite)
    tree.write(output_path, encoding="utf-8", xml_declaration=True)


def main():
    parser = argparse.ArgumentParser(description="Run categorized test buckets and generate a jQuery report.")
    default_test_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "./tests"))
    default_config = os.path.join(default_test_dir, "test_config.json")
    parser.add_argument(
        "--config", required=False, default=default_config, help=f"Path to JSON config file (default: {default_config})"
    )
    parser.add_argument(
        "--test-dir",
        required=False,
        default=default_test_dir,
        help=f"Directory containing test scripts (default: {default_test_dir} relative to script)",
    )
    parser.add_argument("--dry-run", action="store_true", help="Only print what would be run")
    parser.add_argument("--output", default="test_report.xml", help="Output JUnit XML report file")
    parser.add_argument("--suite", required=False, default="alltests", help="Suite name to run (default: alltests)")
    parser.add_argument("--bucket", required=False, default="default", help="Bucket name to run (default: default)")
    args = parser.parse_args()

    config = load_config(args.config)
    scripts = discover_scripts(args.test_dir)
    suites, uncategorized, _ = categorize_and_bucket(scripts, config)

    # Check suite and bucket existence
    if args.suite not in suites:
        print(f"Error: Suite '{args.suite}' not found.")
        sys.exit(1)
    if args.bucket not in suites[args.suite]:
        print(f"Error: Bucket '{args.bucket}' not found in suite '{args.suite}'.")
        sys.exit(1)

    # Filter to only the specified suite and bucket
    filtered_suites = {args.suite: {args.bucket: suites[args.suite][args.bucket]}}

    if args.dry_run:
        print("Dry run mode: the following scripts would be run:")
        print(f"Suite: {args.suite}")
        print(f"  Bucket: {args.bucket}")
        for script in suites[args.suite][args.bucket]:
            print(f"    {script}")
    else:
        results = run_scripts(filtered_suites, dry_run=False, print_live_output=True)
        generate_junit_report(results, args.output, args.suite, args.bucket)
        print(f"JUnit report written to {args.output}")


if __name__ == "__main__":
    main()
