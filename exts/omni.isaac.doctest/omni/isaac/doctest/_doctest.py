# Copyright (c) 2018-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import doctest
import inspect

NO_CHECK = doctest.register_optionflag("NO_CHECK")
"""When specified, run the example and ignore its output, regardless of whether it has any

Example:

.. code-block:: python

    >>> print(list(range(20)))  # doctest: +NO_CHECK
"""


class _Checker(doctest.OutputChecker):
    """Custom doctest's output checker to support the NO_CHECK option"""

    def check_output(self, want, got, optionflags):
        result = super().check_output(want, got, optionflags)
        if not result:
            if optionflags & NO_CHECK:
                return True
        return result


class DocTest:
    def __init__(self, *args, **kwargs) -> None:
        self._globs = {"__name__": "__main__"}
        self._checker = _Checker()

    def _get_names(self, obj, privates: bool = False) -> list[str]:
        """Get class/module names without including special methods"""
        names = []
        for name, value in inspect.getmembers(obj):
            if inspect.isfunction(value) or inspect.isdatadescriptor(value):
                # ignore special names, e.g., __str__
                if name.startswith("__"):
                    continue
                # ignore private names unless specified
                elif name.startswith("_") and not privates:
                    continue
                # ignore functions not defined in current module
                elif inspect.ismodule(obj) and inspect.getmodule(value) != obj:
                    continue
                # ignore properties/methods not defined in current class
                elif inspect.isclass(obj) and not obj.__dict__.get(name):
                    continue
                names.append(f"{obj.__name__}.{name}")
        # add class name to list
        if inspect.isclass(obj):
            names.insert(0, obj.__name__)
        # order methods
        if "initialize" in names:
            names.remove("initialize")
            names.insert(1, "initialize")
        return sorted(names)

    def get_members(
        self, expr: object, order: list[tuple[object, int]] = [], exclude: list[object] = [], _globals: dict = {}
    ) -> list[object]:
        """Get class/module members (names)

        Args:
            expr: module function or class definition, property or method to check docstrings examples for
            order (list[tuple[object, int]]): list of pair (name, index) to modify the examples execution order
            exclude (list[object]): list of class/module names to exclude for testing
            _globals (dict): current namespace

        Returns:
            list[object]: list of class/module members
        """
        _globals.update({expr.__name__: expr})
        members = [eval(name, _globals) for name in self._get_names(expr)]
        # remove exclude items
        members = [member for member in members if not member in exclude]
        # order names
        for member, index in order:
            if member in members:
                index = len(members) + index if index < 0 else index
                members.insert(index, members.pop(members.index(member)))
        return members

    def checkDocTest(
        self,
        expr: object,
        flags: int = doctest.NORMALIZE_WHITESPACE | doctest.ELLIPSIS | doctest.FAIL_FAST,
    ) -> bool:
        """Check that the examples in docstrings pass for a class/module member

        Args:
            expr: module function or class definition, property or method to check docstrings examples for
            flags (int): doctest's option flags

        Returns:
            bool: whether the test passes or fails
        """
        # implement doctest.run_docstring_examples with execution checking
        testFinder = doctest.DocTestFinder(verbose=False, recurse=False)
        testRunner = doctest.DocTestRunner(checker=self._checker, verbose=False, optionflags=flags)
        for test in testFinder.find(expr, name="module"):
            test.globs = self._globs
            status = testRunner.run(test, clear_globs=False)
            if status.failed:
                return False
        return True
