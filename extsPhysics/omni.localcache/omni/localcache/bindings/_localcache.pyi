"""pybind11 omni.localcache bindings"""
from __future__ import annotations
import omni.localcache.bindings._localcache
import typing

__all__ = [
    "ILocalCache",
    "acquire_localcache_interface",
    "release_localcache_interface"
]


class ILocalCache():
    pass
def acquire_localcache_interface(plugin_name: str = None, library_path: str = None) -> ILocalCache:
    pass
def release_localcache_interface(arg0: ILocalCache) -> None:
    pass
