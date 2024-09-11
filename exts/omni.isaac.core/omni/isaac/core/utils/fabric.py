import warp as wp


@wp.kernel(enable_backward=False)
def set_view_to_fabric_array(
    fabric_to_view: wp.fabricarray(dtype=wp.uint32), view_to_fabric: wp.array(ndim=1, dtype=wp.uint32)
):
    fabic_idx = wp.tid()
    view_idx = fabric_to_view[fabic_idx]
    view_to_fabric[view_idx] = wp.uint32(fabic_idx)


@wp.kernel(enable_backward=False)
def set_vec3d_array(
    fabric_vals: wp.fabricarray(dtype=wp.vec3d),
    fabric_to_view: wp.fabricarray(dtype=wp.uint32),
    view_to_fabric: wp.array(ndim=1, dtype=wp.uint32),
    new_vals: wp.array(ndim=2),
    view_indices: wp.array(ndim=1, dtype=wp.uint32),
):
    i = wp.tid()
    view_idx = view_indices[i]
    fabric_idx = view_to_fabric[view_idx]
    new_val = new_vals[i]
    fabric_vals[fabric_idx] = wp.vec3d(wp.float64(new_val[0]), wp.float64(new_val[1]), wp.float64(new_val[2]))


@wp.kernel(enable_backward=False)
def get_vec3d_array(
    fabric_vals: wp.fabricarray(dtype=wp.vec3d),
    fabric_to_view: wp.fabricarray(dtype=wp.uint32),
    view_to_fabric: wp.array(ndim=1, dtype=wp.uint32),
    result: wp.array(ndim=2, dtype=wp.float32),
    view_indices: wp.array(ndim=1, dtype=wp.uint32),
):
    i = wp.tid()
    view_idx = view_indices[i]
    fabric_idx = view_to_fabric[view_idx]
    val = fabric_vals[fabric_idx]
    result[view_idx, 0] = wp.float32(val[0])
    result[view_idx, 1] = wp.float32(val[1])
    result[view_idx, 2] = wp.float32(val[2])


@wp.kernel(enable_backward=False)
def set_quatf_array(
    fabric_vals: wp.fabricarray(dtype=wp.quatf),
    fabric_to_view: wp.fabricarray(dtype=wp.uint32),
    view_to_fabric: wp.array(ndim=1, dtype=wp.uint32),
    new_vals: wp.array(ndim=2),
    view_indices: wp.array(ndim=1, dtype=wp.uint32),
):
    i = wp.tid()
    view_idx = view_indices[i]
    fabric_idx = view_to_fabric[view_idx]
    new_val = new_vals[i]
    fabric_vals[fabric_idx] = wp.quatf(
        wp.float32(new_val[1]), wp.float32(new_val[2]), wp.float32(new_val[3]), wp.float32(new_val[0])
    )


@wp.kernel(enable_backward=False)
def get_quatf_array(
    fabric_vals: wp.fabricarray(dtype=wp.quatf),
    fabric_to_view: wp.fabricarray(dtype=wp.uint32),
    view_to_fabric: wp.array(ndim=1, dtype=wp.uint32),
    result: wp.array(ndim=2, dtype=wp.float32),
    view_indices: wp.array(ndim=1, dtype=wp.uint32),
):
    i = wp.tid()
    view_idx = view_indices[i]
    fabric_idx = view_to_fabric[view_idx]
    val = fabric_vals[fabric_idx]
    result[view_idx, 0] = val[3]
    result[view_idx, 1] = val[0]
    result[view_idx, 2] = val[1]
    result[view_idx, 3] = val[2]


@wp.kernel(enable_backward=False)
def arange_k(a: wp.array(dtype=wp.uint32)):
    tid = wp.uint32(wp.tid())
    a[tid] = tid
