import os, sys
import pathlib

import carb

import warp as wp
from warp.build_dll import set_msvc_compiler
import numpy as np

#wp.config.verify_fp = True
#wp.config.verify_cuda = True
#wp.config.print_launches = True
#wp.config.verbose = True


_warp_deps = """
<project toolsVersion="5.6">
  <dependency name="msvc" linkPath="_warp-deps/msvc" tags="non-redist">
    <package name="msvc" version="2019-16.11.17-2" platforms="windows-x86_64" />
  </dependency>
  <dependency name="winsdk" linkPath="_warp-deps/winsdk" tags="non-redist">
    <package name="winsdk" version="10.17763" platforms="windows-x86_64" />
  </dependency>
</project>
"""

# This is an internal hack to check whether the msvc compiler and windows sdk can be found in host-deps.
# It's required to get TeamCity tests to work, because TC may not have proper Visual Studio installations.
def ensure_warp_msvc_compiler():
    if wp.config.host_compiler is None:
        if os.name == "nt":
            carb.log_warn("~!~!~! Host compiler not set, trying host-deps...")

            import re

            cwd = os.getcwd()
            carb.log_warn("~!~!~!   Working directory: %s" % cwd)

            carb.log_warn("~!~!~!   Executable: %s" % sys.argv[0])
            exe_dir = os.path.dirname(os.path.realpath(sys.argv[0]))
            carb.log_warn("~!~!~!   Executable directory: %s" % exe_dir)

            script_dir = os.path.dirname(os.path.realpath(__file__))
            carb.log_warn("~!~!~!   Script directory: %s" % script_dir)

            def parse_version(s, num_components=3):
                ss = s.split(".")
                v = [0] * num_components
                n = min(len(ss), num_components)
                for i in range(n):
                    if ss[i].isdigit():
                        v[i] = int(ss[i])
                    else:
                        break
                return v
            
            def version_gt(v1, v2):
                n = min(len(v1), len(v2))
                return v1[:n] > v2[:n]

            class HostCompilerInfo:
                def __ini__(self, msvc_dir=None, winsdk_dir=None, compiler_path=None, linker_path=None):
                    self.msvc_dir = msvc_dir
                    self.winsdk_dir = winsdk_dir
                    self.compiler_path = compiler_path
                    self.linker_path = linker_path

            def get_compiler_info(host_deps_dir):
                carb.log_warn("~!~!~!")
                carb.log_warn("~!~!~! Trying %s" % host_deps_dir)

                if not os.path.isdir(host_deps_dir):
                    carb.log_warn("~!~!~! *** Directory not found")
                    return None

                # check for msvc and winsdk directories
                msvc_root = os.path.join(host_deps_dir, "msvc")
                winsdk_root = os.path.join(host_deps_dir, "winsdk")
                if not os.path.isdir(msvc_root) or not os.path.isdir(winsdk_root):
                    carb.log_warn("~!~!~! *** Failed to find msvc and winsdk directories")
                    return None

                # find the directory that contains toolchain versions
                toolchain_parent_dir = os.path.join(msvc_root, "VC", "Tools", "MSVC")
                if not os.path.isdir(toolchain_parent_dir):
                    carb.log_warn("~!~!~! *** Failed to find toolchain collection directory")
                    return None

                # pick a toolchain
                toolchain_dir = None
                compiler_path = None
                linker_path = None
                for name in os.listdir(toolchain_parent_dir):
                    _toolchain_dir = os.path.join(toolchain_parent_dir, name)
                    if os.path.isdir(_toolchain_dir) and re.match(r"^[0-9]+\.[0-9]+\.", name):
                        # check the toolchain binary directory
                        _toolchain_bin_dir = os.path.join(_toolchain_dir, "bin", "HostX64", "x64")
                        if not os.path.isdir(_toolchain_bin_dir):
                            continue
                        # check for compiler and linker
                        _compiler_path = os.path.join(_toolchain_bin_dir, "cl.exe")
                        _linker_path = os.path.join(_toolchain_bin_dir, "link.exe")
                        if not os.path.isfile(_compiler_path) or not os.path.isfile(_linker_path):
                            continue
                        # yay
                        info = HostCompilerInfo()
                        info.msvc_dir = _toolchain_dir
                        info.winsdk_dir = winsdk_root
                        info.compiler_path = _compiler_path
                        info.linker_path = _linker_path
                        return info

                return None


            # host deps paths to try
            host_deps_paths = {
                # try relative to cwd, assuming we're running from _build/$platform/$config
                os.path.abspath(os.path.join(cwd, "..", "..", "host-deps")),
                # try relative to cwd, assuming we're running from repo root
                os.path.abspath(os.path.join(cwd, "_build", "host-deps")),
                # try relative to the executable path, assuming _build/$platform/$config/kit/kit[.exe]
                os.path.abspath(os.path.join(exe_dir, "..", "..", "..", "host-deps")),
                # try relative to this script
                os.path.abspath(os.path.join(script_dir, "..", "..", "..", "..", "..", "..", "..", "host-deps")),
            }

            for host_deps_dir in host_deps_paths:
                info = get_compiler_info(host_deps_dir)
                if info is not None:
                    break

            def try_import_packmanapi():
                packman_local_paths = {
                    os.path.abspath(os.path.join(cwd, "..", "..", "..", "tools", "packman")),
                    os.path.abspath(os.path.join(cwd, "tools", "packman")),
                    os.path.abspath(os.path.join(exe_dir, "..", "..", "..", "..", "tools", "packman")),
                }

                for packman_local_dir in packman_local_paths:
                    if os.path.isdir(packman_local_dir):
                        try:
                            sys.path.append(packman_local_dir)
                            import packmanconf
                            packmanconf.init()
                            import packmanapi
                            carb.log_warn("~!~!~!   Using packman local dir: %s" % packman_local_dir)
                            return True
                        except ImportError:
                            ...

                if "PM_PACKAGES_ROOT" in os.environ:
                    pm_module_dir = None
                    pm_root = os.path.normpath(os.environ["PM_PACKAGES_ROOT"])
                    carb.log_warn("~!~!~!   PM_PACKAGES_ROOT: %s" % pm_root)
                    pm_common = os.path.join(pm_root, "packman-common")
                    if os.path.isdir(pm_common):
                        pm_subdirs = os.listdir(pm_common)
                        carb.log_warn("~!~!~!   Packman versions: %s" % str(pm_subdirs))
                        max_ver = [0, 0, 0]
                        for name in pm_subdirs:
                            _pm_subdir = os.path.join(pm_common, name)
                            if os.path.isdir(_pm_subdir):
                                v = parse_version(name)
                                if version_gt(v, max_ver):
                                    max_ver = v
                                    pm_module_dir = _pm_subdir

                        if pm_module_dir is not None:
                            carb.log_warn("~!~!~!   Using packman module dir: %s" % pm_module_dir)
                            sys.path.append(pm_module_dir)
                            import packmanapi
                            return True
                return False

            # if we failed to find existing host-deps, try downloading the deps using packman
            if info is None:
                carb.log_warn("~!~!~!")
                carb.log_warn("~!~!~! Failed to find host-deps directory, trying Packman...")

                if try_import_packmanapi():
                    import packmanapi
                    deps_file = os.path.join(cwd, "_warp-deps.packman.xml")
                    with open(deps_file, 'w') as f:
                        f.write(_warp_deps)

                    packmanapi.pull(deps_file, platform="windows-x86_64")

                    info = get_compiler_info(os.path.join(cwd, "_warp-deps"))
                else:
                    carb.log_warn("~!~!~!   *** Packman not found")

            carb.log_warn("~!~!~!")

            if info is None:
                carb.log_warn("~!~!~! *** Failed to set up host compiler")
                return

            carb.log_warn("~!~!~! Found msvc directory: %s" % info.msvc_dir)
            carb.log_warn("~!~!~! Found host compiler: %s" % info.compiler_path)
            carb.log_warn("~!~!~! Found host linker: %s" % info.linker_path)
            carb.log_warn("~!~!~! Found winsdk directory: %s" % info.winsdk_dir)

            # configure warp compiler
            set_msvc_compiler(msvc_path=info.msvc_dir, sdk_path=info.winsdk_dir)
            wp.config.host_compiler = info.compiler_path


@wp.kernel
def _arange_k(a: wp.array(dtype=wp.int32)):
    tid = wp.tid()
    a[tid] = tid


def arange(n, device="cpu"):
    a = wp.empty(n, dtype=wp.int32, device=device)
    wp.launch(kernel=_arange_k, dim=n, inputs=[a], device=device)
    wp.synchronize()
    return a


@wp.kernel
def _linspace_k(a: wp.array(dtype=wp.float32), offset: wp.float32, step: wp.float32):
    tid = wp.tid()
    a[tid] = offset + float(tid) * step


def linspace(n, start, end, include_end=False, include_start=True, device="cpu"):
    d = n - 1
    if not include_start:
        d += 1
    if not include_end:
        d += 1

    step = (end - start) / d
    if not include_start:
        offset = start + step
    else:
        offset = start

    a = wp.empty(n, dtype=wp.float32, device=device)
    wp.launch(kernel=_linspace_k, dim=n, inputs=[a, offset, step], device=device)
    wp.synchronize()
    return a


@wp.kernel
def _fill_float32_k(a: wp.array(dtype=wp.float32), value: wp.float32):
    tid = wp.tid()
    a[tid] = value


def fill_float32(n, value=0.0, device="cpu"):
    a = wp.empty(n, dtype=wp.float32, device=device)
    wp.launch(kernel=_fill_float32_k, dim=n, inputs=[a, value], device=device)
    wp.synchronize()
    return a


@wp.kernel
def _fill_vec3_k(a: wp.array(dtype=wp.vec3), value: wp.vec3):
    tid = wp.tid()
    a[tid] = value


def fill_vec3(n, value=wp.vec3(0.0, 0.0, 0.0), device="cpu"):
    a = wp.empty(n, dtype=wp.vec3, device=device)
    wp.launch(kernel=_fill_vec3_k, dim=n, inputs=[a, value], device=device)
    wp.synchronize()
    return a


@wp.kernel
def _random_k(seed: int, a: wp.array(dtype=float), lower:float, upper:float):
    tid = wp.tid()
    state = wp.rand_init(seed, tid)
    a[tid] = wp.randf(state, lower, upper)


def random(n, lower=0.0, upper=1.0, device="cpu", seed=42):
    a = wp.zeros(n, dtype=float, device=device)
    wp.launch(kernel=_random_k, dim=n, inputs=[seed, a, lower, upper], device=device)
    wp.synchronize()
    return a


@wp.kernel
def _compute_dof_forces_k(
    pos: wp.array(dtype=float, ndim=2),
    vel: wp.array(dtype=float, ndim=2),
    force: wp.array(dtype=float, ndim=2),
    stiffness: float,
    damping: float,
):
    i, j = wp.tid()
    pos_target = 0.0
    force[i, j] = stiffness * (pos_target - pos[i, j]) - damping * vel[i, j]


def compute_dof_forces(pos, vel, force, stiffness, damping, device="cpu"):
    wp.launch(kernel=_compute_dof_forces_k, dim=force.shape, inputs=[pos, vel, force, stiffness, damping], device=device)
    wp.synchronize()
