import omni.ext
from omni.localcache.bindings import _localcache


class ILocalCacheExtension(omni.ext.IExt):
    def on_startup(self):
        self._localcache = _localcache.acquire_localcache_interface()

    def on_shutdown(self):
        _localcache.release_localcache_interface(self._localcache)
