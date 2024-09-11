"""pybind11 omni.kvdb bindings"""
from __future__ import annotations
import omni.kvdb.bindings._kvdb
import typing

__all__ = [
    "IKeyValueDatabase",
    "acquire_kvdb_interface",
    "release_kvdb_interface"
]


class IKeyValueDatabase():
    pass
def acquire_kvdb_interface(plugin_name: str = None, library_path: str = None) -> IKeyValueDatabase:
    pass
def release_kvdb_interface(arg0: IKeyValueDatabase) -> None:
    pass
