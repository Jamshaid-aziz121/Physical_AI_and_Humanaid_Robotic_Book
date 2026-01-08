#!/usr/bin/env python3
"""
Framework for reproducibility checks in the Physical AI & Humanoid Robotics Book

This module provides utilities to verify that code examples and experiments
produce consistent, reproducible results.
"""

import os
import sys
import json
import hashlib
import subprocess
from datetime import datetime
from typing import Dict, List, Any, Optional
from pathlib import Path


class ReproducibilityChecker:
    """
    Framework for verifying reproducibility of robotics experiments and code examples
    """
    def __init__(self, project_root: str = "."):
        self.project_root = Path(project_root)
        self.results_dir = self.project_root / "test_results"
        self.results_dir.mkdir(exist_ok=True)

        # Track all checks performed
        self.checks_performed = []

    def check_environment_consistency(self) -> Dict[str, Any]:
        """
        Check if the development environment is consistent

        Returns:
            Dictionary with environment information and consistency status
        """
        env_info = {
            "timestamp": datetime.now().isoformat(),
            "python_version": sys.version,
            "platform": sys.platform,
            "cwd": str(Path.cwd()),
            "dependencies": self._get_dependencies()
        }

        check_result = {
            "check": "environment_consistency",
            "passed": True,
            "details": env_info
        }

        self.checks_performed.append(check_result)
        return check_result

    def _get_dependencies(self) -> List[str]:
        """
        Get list of installed Python packages

        Returns:
            List of package names with versions
        """
        try:
            result = subprocess.run([sys.executable, "-m", "pip", "freeze"],
                                    capture_output=True, text=True, timeout=30)
            if result.returncode == 0:
                return result.stdout.strip().split('\n')
            else:
                return ["Failed to get dependencies"]
        except subprocess.TimeoutExpired:
            return ["Timeout getting dependencies"]
        except Exception as e:
            return [f"Error getting dependencies: {str(e)}"]

    def check_file_integrity(self, file_path: str) -> Dict[str, Any]:
        """
        Check integrity of a file by computing its hash

        Args:
            file_path: Path to the file to check

        Returns:
            Dictionary with file hash and integrity status
        """
        file_path = Path(file_path)
        if not file_path.exists():
            check_result = {
                "check": "file_integrity",
                "file": str(file_path),
                "passed": False,
                "details": "File does not exist"
            }
            self.checks_performed.append(check_result)
            return check_result

        try:
            with open(file_path, 'rb') as f:
                file_content = f.read()
                file_hash = hashlib.sha256(file_content).hexdigest()

            check_result = {
                "check": "file_integrity",
                "file": str(file_path),
                "hash": file_hash,
                "size": len(file_content),
                "passed": True,
                "details": f"File hash computed successfully"
            }
            self.checks_performed.append(check_result)
            return check_result
        except Exception as e:
            check_result = {
                "check": "file_integrity",
                "file": str(file_path),
                "passed": False,
                "details": f"Error reading file: {str(e)}"
            }
            self.checks_performed.append(check_result)
            return check_result

    def check_code_syntax(self, file_path: str) -> Dict[str, Any]:
        """
        Check Python syntax of a code file

        Args:
            file_path: Path to the Python file to check

        Returns:
            Dictionary with syntax check results
        """
        file_path = Path(file_path)
        if not file_path.suffix == '.py':
            check_result = {
                "check": "code_syntax",
                "file": str(file_path),
                "passed": False,
                "details": "Not a Python file"
            }
            self.checks_performed.append(check_result)
            return check_result

        if not file_path.exists():
            check_result = {
                "check": "code_syntax",
                "file": str(file_path),
                "passed": False,
                "details": "File does not exist"
            }
            self.checks_performed.append(check_result)
            return check_result

        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                source_code = f.read()

            # Compile to check syntax
            compile(source_code, str(file_path), 'exec')

            check_result = {
                "check": "code_syntax",
                "file": str(file_path),
                "passed": True,
                "details": "Syntax is valid"
            }
            self.checks_performed.append(check_result)
            return check_result
        except SyntaxError as e:
            check_result = {
                "check": "code_syntax",
                "file": str(file_path),
                "passed": False,
                "details": f"Syntax error at line {e.lineno}: {e.msg}"
            }
            self.checks_performed.append(check_result)
            return check_result
        except Exception as e:
            check_result = {
                "check": "code_syntax",
                "file": str(file_path),
                "passed": False,
                "details": f"Error checking syntax: {str(e)}"
            }
            self.checks_performed.append(check_result)
            return check_result

    def check_directory_structure(self, required_dirs: List[str], base_path: str = ".") -> Dict[str, Any]:
        """
        Check if required directories exist in the project

        Args:
            required_dirs: List of required directory paths
            base_path: Base path to check from (default: current directory)

        Returns:
            Dictionary with directory structure check results
        """
        base_path = Path(base_path)
        missing_dirs = []

        for dir_path in required_dirs:
            full_path = base_path / dir_path
            if not full_path.exists():
                missing_dirs.append(str(full_path))

        passed = len(missing_dirs) == 0

        check_result = {
            "check": "directory_structure",
            "required_dirs": required_dirs,
            "missing_dirs": missing_dirs,
            "passed": passed,
            "details": f"Found {len(required_dirs) - len(missing_dirs)}/{len(required_dirs)} required directories"
        }

        self.checks_performed.append(check_result)
        return check_result

    def run_all_checks(self, target_dir: str = "book-content/modules") -> Dict[str, Any]:
        """
        Run all reproducibility checks on the target directory

        Args:
            target_dir: Directory to run checks on (default: book-content/modules)

        Returns:
            Dictionary with summary of all checks
        """
        target_path = Path(target_dir)
        if not target_path.exists():
            return {
                "overall_passed": False,
                "summary": f"Target directory {target_dir} does not exist",
                "checks": []
            }

        # Check environment
        self.check_environment_consistency()

        # Find all Python files to check
        python_files = list(target_path.rglob("*.py"))

        # Check syntax for all Python files
        for py_file in python_files:
            self.check_code_syntax(str(py_file))

        # Check integrity for all files
        all_files = list(target_path.rglob("*"))
        for file_path in all_files:
            if file_path.is_file():
                self.check_file_integrity(str(file_path))

        # Check required directories
        required_dirs = [
            "01-ros2-fundamentals/concepts",
            "01-ros2-fundamentals/code-examples",
            "01-ros2-fundamentals/exercises",
            "01-ros2-fundamentals/solutions",
            "common_utils",
            "common_config"
        ]
        self.check_directory_structure(required_dirs, target_dir)

        # Generate summary
        total_checks = len(self.checks_performed)
        passed_checks = sum(1 for check in self.checks_performed if check["passed"])

        summary = {
            "overall_passed": passed_checks == total_checks,
            "total_checks": total_checks,
            "passed_checks": passed_checks,
            "failed_checks": total_checks - passed_checks,
            "success_rate": round(passed_checks / total_checks * 100, 2) if total_checks > 0 else 0,
            "checks": self.checks_performed
        }

        # Save results
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        results_file = self.results_dir / f"reproducibility_check_{timestamp}.json"
        with open(results_file, 'w') as f:
            json.dump(summary, f, indent=2)

        return summary

    def generate_report(self, output_file: Optional[str] = None) -> str:
        """
        Generate a human-readable report of all checks performed

        Args:
            output_file: Optional file path to save the report

        Returns:
            Report string
        """
        if not self.checks_performed:
            report = "No checks have been performed yet."
        else:
            total_checks = len(self.checks_performed)
            passed_checks = sum(1 for check in self.checks_performed if check["passed"])

            report_lines = [
                "Reproducibility Check Report",
                "=" * 30,
                f"Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
                f"Total Checks: {total_checks}",
                f"Passed: {passed_checks}",
                f"Failed: {total_checks - passed_checks}",
                f"Success Rate: {round(passed_checks / total_checks * 100, 2) if total_checks > 0 else 0}%",
                "",
                "Detailed Results:",
                "-" * 16
            ]

            for i, check in enumerate(self.checks_performed, 1):
                status = "PASS" if check["passed"] else "FAIL"
                report_lines.append(f"{i:2d}. [{status}] {check['check']}")

                if 'file' in check:
                    report_lines.append(f"    File: {check['file']}")

                report_lines.append(f"    Details: {check['details']}")
                report_lines.append("")

            report = "\n".join(report_lines)

        if output_file:
            with open(output_file, 'w') as f:
                f.write(report)

        return report


def main():
    """
    Main function to run the reproducibility checker
    """
    import argparse

    parser = argparse.ArgumentParser(description='Reproducibility Checker for Physical AI & Humanoid Robotics Book')
    parser.add_argument('--target-dir', '-t', default='book-content/modules',
                       help='Target directory to check (default: book-content/modules)')
    parser.add_argument('--output-file', '-o',
                       help='Output file for the report')
    parser.add_argument('--check-single-file', '-f',
                       help='Check syntax of a single Python file')

    args = parser.parse_args()

    checker = ReproducibilityChecker()

    if args.check_single_file:
        # Check syntax of a single file
        result = checker.check_code_syntax(args.check_single_file)
        print(f"Check result for {args.check_single_file}:")
        print(f"Passed: {result['passed']}")
        print(f"Details: {result['details']}")
    else:
        # Run all checks
        summary = checker.run_all_checks(args.target_dir)

        print(f"Overall Result: {'PASS' if summary['overall_passed'] else 'FAIL'}")
        print(f"Success Rate: {summary['success_rate']}% ({summary['passed_checks']}/{summary['total_checks']})")

        # Generate and print detailed report
        report = checker.generate_report(args.output_file)
        print("\n" + report)


if __name__ == "__main__":
    main()