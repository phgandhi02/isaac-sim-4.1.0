# Copyright (c) 2018-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import doctest
import sys
import unittest

from . import _doctest


class StandaloneDocTestCase(unittest.TestCase):
    """Base class for all standalone test cases with doctest support for checking docstrings examples

    .. note::

        Test methods must start with the ``test_`` prefix

    This class add the following methods:

    * ``assertDocTest``: Check that the examples in docstrings pass for a class/module member
    * ``assertDocTests``: Check that the examples in docstrings pass for all class/module's members (names)

    Example:

    .. code-block:: python

        >>> import omni.isaac.doctest
        >>> tester = omni.isaac.doctest.StandaloneDocTestCase()
        >>> tester.__class__.__name__
        'StandaloneDocTestCase'
    """

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self._doctest_checker = _doctest.DocTest()

    def assertDocTest(
        self,
        expr: object,
        msg: str = "",
        flags: int = doctest.NORMALIZE_WHITESPACE | doctest.ELLIPSIS | doctest.FAIL_FAST,
    ) -> None:
        """Check that the examples in docstrings pass for a class/module member

        Args:
            expr: module function or class definition, property or method to check docstrings examples for
            msg (str): custom message to display when failing
            flags (int): doctest's option flags

        Example:

        .. code-block:: python

            >>> from omni.isaac.doctest import StandaloneDocTestCase
            >>>
            >>> tester.assertDocTest(StandaloneDocTestCase)
        """
        result = self._doctest_checker.checkDocTest(expr, flags)
        if not result:
            sys.tracebacklimit = 0  # don't show test file tracing, only doctest
            self.fail("Examples in docstrings failed" if not msg else msg)

    def assertDocTests(
        self,
        expr: object,
        msg: str = "",
        flags: int = doctest.NORMALIZE_WHITESPACE | doctest.ELLIPSIS | doctest.FAIL_FAST,
        order: list[tuple[object, int]] = [],
        exclude: list[object] = [],
        stop_on_failure: bool = False,
    ) -> None:
        """Check that the examples in docstrings pass for all class/module's members (names)

        Args:
            expr: module or class definition to check members' docstrings examples for
            msg (str): custom message to display when failing
            flags (int): doctest's option flags
            order (list[tuple[object, int]]): list of pair (name, index) to modify the examples execution order
            exclude (list[object]): list of class/module names to exclude for testing
            stop_on_failure (bool): stop testing docstrings example at fist encountered failure

        Example:

        .. code-block:: python

            >>> from omni.isaac.doctest import StandaloneDocTestCase
            >>>
            >>> tester.assertDocTests(StandaloneDocTestCase, exclude=[StandaloneDocTestCase.assertDocTests])
            ... # doctest: +NO_CHECK
        """
        objects = self._doctest_checker.get_members(expr, order, exclude, {})
        print(f"class/module members to check: {len(objects)}")
        # test docstrings examples
        failures = 0
        for obj in objects:
            try:
                name = obj.__name__
            except:
                name = obj.fget.__name__
            print(f"  |-- checking: {name}")
            try:
                self.assertDocTest(obj, msg=msg, flags=flags)
            except AssertionError as e:
                failures += 1
                if stop_on_failure:
                    raise e
        if failures:
            self.fail(
                f"Some tests failed: failed ({failures}), successful ({len(objects) - failures}), total ({len(objects)})"
            )
