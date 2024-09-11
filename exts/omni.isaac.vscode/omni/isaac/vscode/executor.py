# Copyright (c) 2018-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import __future__

import contextlib
import dis
import io
import traceback

try:
    from ast import PyCF_ALLOW_TOP_LEVEL_AWAIT
except ImportError:
    PyCF_ALLOW_TOP_LEVEL_AWAIT = 0


class Executor:
    """Execute Python statements or expressions from strings

    Args:
        globals (str): global namespace
        locals (str): local namespace
    """

    def __init__(self, globals: dict = {}, locals: dict = {}) -> None:
        self._globals = globals
        self._locals = locals
        self._compiler_flags = self._get_compiler_flags()
        self._coroutine_flag = self._get_coroutine_flag()

    def _get_compiler_flags(self) -> int:
        """Get current Python version compiler flags"""
        flags = 0
        for value in globals().values():
            try:
                if isinstance(value, __future__._Feature):
                    flags |= value.compiler_flag
            except BaseException:
                pass
        return flags | PyCF_ALLOW_TOP_LEVEL_AWAIT

    def _get_coroutine_flag(self) -> int:
        """Get current Python version coroutine flag"""
        for k, v in dis.COMPILER_FLAG_NAMES.items():
            if v == "COROUTINE":
                return k
        return -1

    async def execute(self, source: str) -> tuple[str, Exception, str]:
        """Execute source in the Python scope

        Args:
            source (str): statement or expression

        Returns:
            tuple[str, Exception, str]: standard output, exception thrown (or None if not thrown), exception trace
        """
        output = io.StringIO()
        try:
            with contextlib.redirect_stdout(output):
                do_exec_step = True
                # try 'eval' first
                try:
                    code = compile(source, "<string>", "eval", flags=self._compiler_flags, dont_inherit=True)
                except SyntaxError:
                    pass
                else:
                    result = eval(code, self._globals, self._locals)
                    do_exec_step = False
                # if 'eval' fails, try 'exec'
                if do_exec_step:
                    code = compile(source, "<string>", "exec", flags=self._compiler_flags, dont_inherit=True)
                    result = eval(code, self._globals, self._locals)
                # await the result if it is a coroutine
                if self._coroutine_flag != -1 and bool(code.co_flags & self._coroutine_flag):
                    result = await result
        except Exception as e:
            return output.getvalue(), e, traceback.format_exc()
        return output.getvalue(), None, ""
