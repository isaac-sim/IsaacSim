#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse
import glob
import json
import os
import shlex
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from collections import defaultdict

# Default arguments that are always passed to test scripts
DEFAULT_SCRIPT_ARGS = ["--no-window"]


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


def has_failure_keywords(text, keywords=None):
    """Check if text contains any failure keywords.

    Args:
        text: Text to search for failure keywords.
        keywords: List of keywords to search for. If None, uses default keywords.

    Returns:
        True if any failure keywords are found, False otherwise.
    """
    if keywords is None:
        keywords = ["error", "Error", "ERROR", "Failed", "FAILED"]

    if not text:
        return False

    return any(keyword in text for keyword in keywords)


def run_scripts(suites, dry_run=False, print_live_output=False, failure_keywords=None, script_args=""):
    """Run test scripts and collect results.

    Args:
        suites: Dictionary of test suites and buckets.
        dry_run: If True, only simulate running tests without executing them.
        print_live_output: If True, print test output in real-time instead of capturing it.
        failure_keywords: List of keywords that indicate test failure if found in output.
                         If None, uses default failure keywords.
        script_args: Additional arguments to pass to each test script (in addition to default args).

    Returns:
        Dictionary containing test results organized by suite and bucket.

    Note:
        All test scripts are executed with default arguments: {' '.join(DEFAULT_SCRIPT_ARGS)}
        The script_args parameter adds to these defaults, not replaces them.
    """
    results = defaultdict(lambda: defaultdict(list))
    for suite, buckets in suites.items():
        for bucket, scripts in buckets.items():
            for script in scripts:
                if dry_run:
                    results[suite][bucket].append(
                        {
                            "script": script,
                            "status": "dry-run",
                            "exit_code": None,
                            "stdout": "",
                            "stderr": "",
                            "duration": 0.0,
                        }
                    )
                else:
                    try:
                        # Build command with default arguments plus any additional user arguments
                        all_args = DEFAULT_SCRIPT_ARGS.copy()
                        if script_args:
                            user_args = shlex.split(script_args)
                            all_args.extend(user_args)

                        command = [script] + all_args

                        start_time = time.time()
                        if print_live_output:
                            print(f"\n===== Running: {shlex.join(command)} =====")
                            proc = subprocess.run(command, text=True)
                            end_time = time.time()
                            duration = end_time - start_time
                            # For live output, we can't check keywords since output isn't captured
                            # Only check exit code
                            status = "pass" if proc.returncode == 0 else "fail"
                            results[suite][bucket].append(
                                {
                                    "script": script,
                                    "status": status,
                                    "exit_code": proc.returncode,
                                    "stdout": "",
                                    "stderr": "",
                                    "duration": duration,
                                }
                            )
                        else:
                            proc = subprocess.run(command, capture_output=True, text=True)
                            end_time = time.time()
                            duration = end_time - start_time

                            # Check exit code first
                            status = "pass" if proc.returncode == 0 else "fail"

                            # If test passed based on exit code, check for failure keywords in output
                            if status == "pass" and (
                                has_failure_keywords(proc.stdout, failure_keywords)
                                or has_failure_keywords(proc.stderr, failure_keywords)
                            ):
                                status = "fail"

                            results[suite][bucket].append(
                                {
                                    "script": script,
                                    "status": status,
                                    "exit_code": proc.returncode,
                                    "stdout": proc.stdout,
                                    "stderr": proc.stderr,
                                    "duration": duration,
                                }
                            )
                    except Exception as e:
                        end_time = time.time()
                        duration = end_time - start_time
                        results[suite][bucket].append(
                            {
                                "script": script,
                                "status": "fail",
                                "exit_code": None,
                                "stdout": "",
                                "stderr": str(e),
                                "duration": duration,
                            }
                        )
    return results


def print_timing_summary(results):
    """Print a summary of test execution times."""
    print("\n" + "=" * 80)
    print("TEST EXECUTION TIMING SUMMARY")
    print("=" * 80)

    for suite_name, buckets in results.items():
        print(f"\nSuite: {suite_name}")
        for bucket_name, tests in buckets.items():
            print(f"  Bucket: {bucket_name}")
            total_duration = 0.0

            # Sort tests by duration (longest first)
            sorted_tests = sorted(tests, key=lambda x: x.get("duration", 0.0), reverse=True)

            for test in sorted_tests:
                duration = test.get("duration", 0.0)
                total_duration += duration
                status = test["status"]
                script_name = os.path.basename(test["script"])

                # Format duration
                if duration >= 60:
                    duration_str = f"{duration/60:.1f}m {duration%60:.1f}s"
                else:
                    duration_str = f"{duration:.1f}s"

                # Color coding for status (if terminal supports it)
                status_symbol = "✓" if status == "pass" else "✗" if status == "fail" else "○"

                # Add more detailed failure information
                status_detail = status
                if status == "fail":
                    exit_code = test.get("exit_code")
                    if exit_code == 0:
                        status_detail = "fail (keyword detected)"
                    elif exit_code is not None:
                        status_detail = f"fail (exit code {exit_code})"
                    else:
                        status_detail = "fail (exception)"

                print(f"    {status_symbol} {script_name:<50} {duration_str:>10} ({status_detail})")

            # Print bucket total
            if total_duration >= 60:
                total_str = f"{total_duration/60:.1f}m {total_duration%60:.1f}s"
            else:
                total_str = f"{total_duration:.1f}s"
            print(f"    {'Total:':<52} {total_str:>10}")


def generate_junit_report(results, output_path, suite_name, bucket_name):
    testsuite = ET.Element("testsuite", name=f"{suite_name}.{bucket_name}")
    total = 0
    failures = 0
    errors = 0
    total_time = 0.0

    for testcase in results[suite_name][bucket_name]:
        duration = testcase.get("duration", 0.0)
        total_time += duration

        # Create testcase element with timing information
        tc = ET.SubElement(testsuite, "testcase", name=os.path.basename(testcase["script"]), time=f"{duration:.3f}")
        total += 1

        # Handle different test statuses
        if testcase["status"] == "fail":
            failures += 1
            exit_code = testcase["exit_code"]

            # Determine failure reason
            if exit_code == 0:
                message = "Test failed due to error keywords found in output"
            elif exit_code is not None:
                message = f"Exit code: {exit_code}"
            else:
                message = "Test failed with exception"

            fail = ET.SubElement(tc, "failure", message=message)
            # Include both stderr and stdout in failure message
            output_parts = []
            if testcase["stderr"]:
                output_parts.append(f"STDERR:\n{testcase['stderr']}")
            if testcase["stdout"]:
                output_parts.append(f"STDOUT:\n{testcase['stdout']}")
            fail.text = "\n\n".join(output_parts) if output_parts else "No output captured"

        elif testcase["status"] not in ("pass", "dry-run"):
            errors += 1
            err = ET.SubElement(tc, "error", message="Error")
            # Include both stderr and stdout in error message
            output_parts = []
            if testcase["stderr"]:
                output_parts.append(f"STDERR:\n{testcase['stderr']}")
            if testcase["stdout"]:
                output_parts.append(f"STDOUT:\n{testcase['stdout']}")
            err.text = "\n\n".join(output_parts) if output_parts else "No output captured"

        # Add system-out and system-err elements for all tests (standard JUnit format)
        if testcase["stdout"]:
            system_out = ET.SubElement(tc, "system-out")
            system_out.text = testcase["stdout"]

        if testcase["stderr"]:
            system_err = ET.SubElement(tc, "system-err")
            system_err.text = testcase["stderr"]

    # Set testsuite attributes
    testsuite.set("tests", str(total))
    testsuite.set("failures", str(failures))
    testsuite.set("errors", str(errors))
    testsuite.set("time", f"{total_time:.3f}")

    # Format the XML with proper indentation
    ET.indent(testsuite, space="  ", level=0)
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
    parser.add_argument(
        "--bucket",
        required=False,
        default="default",
        help="Bucket name to run (default: default). Use 'all' to run all buckets in the suite.",
    )
    parser.add_argument(
        "--failure-keywords",
        nargs="*",
        help="Keywords that indicate test failure when found in output (overrides config and defaults). "
        "Example: --failure-keywords error fail exception",
    )
    parser.add_argument(
        "--script-args",
        default="",
        help="Additional arguments to pass to each test script (in addition to default args: "
        f"{' '.join(DEFAULT_SCRIPT_ARGS)}). "
        "Example: --script-args '--headless --timeout 300'",
    )
    args = parser.parse_args()

    config = load_config(args.config)
    scripts = discover_scripts(args.test_dir)
    suites, uncategorized, _ = categorize_and_bucket(scripts, config)

    # Check suite and bucket existence
    if args.suite not in suites:
        print(f"Error: Suite '{args.suite}' not found.")
        sys.exit(1)

    # Handle 'all' bucket option to run all buckets in the suite
    if args.bucket == "all":
        filtered_suites = {args.suite: suites[args.suite]}
    else:
        if args.bucket not in suites[args.suite]:
            print(f"Error: Bucket '{args.bucket}' not found in suite '{args.suite}'.")
            sys.exit(1)
        # Filter to only the specified suite and bucket
        filtered_suites = {args.suite: {args.bucket: suites[args.suite][args.bucket]}}

    if args.dry_run:
        print("Dry run mode: the following scripts would be run:")
        print(f"Suite: {args.suite}")

        # Show default arguments
        print(f"Default script arguments: {' '.join(DEFAULT_SCRIPT_ARGS)}")
        if args.script_args:
            print(f"Additional script arguments: {args.script_args}")

        if args.bucket == "all":
            for bucket_name, scripts in filtered_suites[args.suite].items():
                print(f"  Bucket: {bucket_name}")
                for script in scripts:
                    all_args = DEFAULT_SCRIPT_ARGS.copy()
                    if args.script_args:
                        user_args = shlex.split(args.script_args)
                        all_args.extend(user_args)
                    command = [script] + all_args
                    print(f"    {shlex.join(command)}")
        else:
            print(f"  Bucket: {args.bucket}")
            for script in filtered_suites[args.suite][args.bucket]:
                all_args = DEFAULT_SCRIPT_ARGS.copy()
                if args.script_args:
                    user_args = shlex.split(args.script_args)
                    all_args.extend(user_args)
                command = [script] + all_args
                print(f"    {shlex.join(command)}")
    else:
        # Load failure keywords with priority: CLI args > config > defaults
        failure_keywords = None
        if args.failure_keywords is not None:
            failure_keywords = args.failure_keywords
        elif "failure_keywords" in config:
            failure_keywords = config["failure_keywords"]
        # If failure_keywords is None, has_failure_keywords will use its defaults

        results = run_scripts(
            filtered_suites,
            dry_run=False,
            print_live_output=True,
            failure_keywords=failure_keywords,
            script_args=args.script_args,
        )

        # Print timing summary
        print_timing_summary(results)

        if args.bucket == "all":
            # Generate separate reports for each bucket
            for bucket_name in filtered_suites[args.suite].keys():
                bucket_output = args.output.replace(".xml", f"_{bucket_name}.xml")
                generate_junit_report(results, bucket_output, args.suite, bucket_name)
                print(f"JUnit report for bucket '{bucket_name}' written to {bucket_output}")
        else:
            generate_junit_report(results, args.output, args.suite, args.bucket)
            print(f"JUnit report written to {args.output}")


if __name__ == "__main__":
    main()
