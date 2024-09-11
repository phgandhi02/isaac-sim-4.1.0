import omni.ext
from omni.kvdb.bindings import _kvdb


class IKeyValueDatabaseExtension(omni.ext.IExt):
    def on_startup(self):
        self._kvdb = _kvdb.acquire_kvdb_interface()

    def on_shutdown(self):
        _kvdb.release_kvdb_interface(self._kvdb)
