# PlatformIO custom test runner for AUnit.
#
# AUnit (bxparks/AUnit) is not one of PlatformIO's built-in test frameworks, so
# platformio.ini sets `test_framework = custom` and PlatformIO loads this file
# to interpret the test program's serial output. The class must be named
# CustomTestRunner and subclass TestRunnerBase (see platformio.test.runners).
#
# The runner uploads the test program (handled by the base class), then reads
# the board's serial output line by line. AUnit prints one line per test and a
# final summary line; we translate those into PlatformIO TestCase results and
# stop reading once the summary appears.

import re

import click

from platformio.test.result import TestCase, TestStatus
from platformio.test.runners.base import TestRunnerBase
from platformio.util import strip_ansi_codes


class CustomTestRunner(TestRunnerBase):
    # One line per resolved test, e.g.:
    #   Test deviceIo_resetTurnsEverythingOff passed.
    #   Test protocol_streamRoundTrip failed.
    #   Test someTest skipped.
    #   Test someTest timed out.
    TESTCASE_PARSE_RE = re.compile(
        r"^Test\s+(?P<name>\S+)\s+(?P<status>passed|failed|skipped|timed out)\.$"
    )

    # AUnit status word -> PlatformIO status. A timed-out test is a failure.
    STATUS_MAP = {
        "passed": TestStatus.PASSED,
        "failed": TestStatus.FAILED,
        "skipped": TestStatus.SKIPPED,
        "timed out": TestStatus.FAILED,
    }

    def on_testing_line_output(self, line):
        if self.options.verbose:
            click.echo(line, nl=False)

        line = strip_ansi_codes(line or "").strip()
        if not line:
            return

        test_case = self.parse_test_case(line)
        if test_case:
            self.test_suite.add_case(test_case)
            if not self.options.verbose:
                click.echo(test_case.humanize())

        # AUnit prints a single line when the whole run is done, e.g.:
        #   TestRunner summary: 6 passed, 0 failed, 0 skipped, 0 timed out, ...
        # Use it to tell the serial reader the suite has finished.
        if line.startswith("TestRunner summary:"):
            self.test_suite.on_finish()

    def parse_test_case(self, line):
        match = self.TESTCASE_PARSE_RE.search(line)
        if not match:
            return None
        data = match.groupdict()
        return TestCase(
            name=data["name"],
            status=self.STATUS_MAP[data["status"]],
            stdout=line,
        )
