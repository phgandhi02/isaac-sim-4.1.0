from omni.physics.tensors import acquire_tensor_api, float32, uint8, uint32
import carb


def create_simulation_view(frontend_name, stage_id=-1):
    tensor_api = acquire_tensor_api()
    if tensor_api is None:
        raise Exception("Failed to acquire tensor API")

    backend = tensor_api.create_simulation_view(stage_id)
    if backend is None:
        raise Exception("Failed to create simulation view backend")

    device_ordinal = backend.device_ordinal

    frontend_id = frontend_name.lower()

    if frontend_id == "numpy" or frontend_id == "np":
        if device_ordinal == -1:
            from .frontend_np import FrontendNumpy

            frontend = FrontendNumpy()
            return SimulationView(backend, frontend)
        else:
            raise Exception("The Numpy frontend cannot be used with GPU pipelines")

    elif frontend_id == "torch" or frontend_id == "pytorch":
        from .frontend_torch import FrontendTorch

        frontend = FrontendTorch(device_ordinal)
        return SimulationView(backend, frontend)

    elif frontend_id == "tensorflow" or frontend_id == "tf":
        from .frontend_tf import FrontendTensorflow

        frontend = FrontendTensorflow(device_ordinal)
        return SimulationView(backend, frontend)

    elif frontend_id == "warp" or frontend_id == "wp":
        from .frontend_warp import FrontendWarp

        frontend = FrontendWarp(device_ordinal)
        return SimulationView(backend, frontend)

    else:
        raise Exception("Unrecognized frontend name '%s'" % frontend_name)


def reset():
    tensor_api = acquire_tensor_api()
    if tensor_api is None:
        raise Exception("Failed to acquire tensor API")
    tensor_api.reset()


class SimulationView:
    def __init__(self, backend, frontend):
        self._backend = backend
        self._frontend = frontend

    @property
    def device(self):
        return self._frontend.device

    @property
    def is_valid(self):
        return self._backend.is_valid
    
    @property
    def device_ordinal(self):
        return self._backend.device_ordinal

    @property
    def cuda_context(self):
        return self._backend.cuda_context
    
    def invalidate(self):
        return self._backend.invalidate()
    
    def set_subspace_roots(self, pattern):
        if not self._backend.set_subspace_roots(pattern):
            raise Exception("Failed to set subspace roots")

    def create_articulation_view(self, pattern):
        return ArticulationView(self._backend.create_articulation_view(pattern), self._frontend)

    def create_rigid_body_view(self, pattern):
        return RigidBodyView(self._backend.create_rigid_body_view(pattern), self._frontend)

    def create_soft_body_view(self, pattern):
        return SoftBodyView(self._backend.create_soft_body_view(pattern), self._frontend)

    def create_soft_body_material_view(self, pattern):
        return SoftBodyMaterialView(self._backend.create_soft_body_material_view(pattern), self._frontend)
    
    def create_rigid_contact_view(self, pattern, filter_patterns=[], max_contact_data_count = 0):
        return RigidContactView(self._backend.create_rigid_contact_view(pattern, filter_patterns, max_contact_data_count), self._frontend)

    def create_sdf_shape_view(self, pattern, num_points):
        return SdfShapeView(self._backend.create_sdf_shape_view(pattern, num_points), self._frontend)

    def create_particle_system_view(self, pattern):
        return ParticleSystemView(self._backend.create_particle_system_view(pattern), self._frontend)

    def create_particle_cloth_view(self, pattern):
        return ParticleClothView(self._backend.create_particle_cloth_view(pattern), self._frontend)

    def create_particle_material_view(self, pattern):
        return ParticleMaterialView(self._backend.create_particle_material_view(pattern), self._frontend)

    def flush(self):
        carb.log_warn("DEPRECATED: flush() no longer is needed and has no effect.")
        return self._backend.flush()

    def clear_forces(self):
        return self._backend.clear_forces()

    def enable_warnings(self, enable):
        return self._backend.enable_warnings(enable)

    def update_articulations_kinematic(self):
        return self._backend.update_articulations_kinematic()

    def check(self):
        return self._backend.check()

    def step(self, dt):
        return self._backend.step(dt)

    def set_gravity(self, gravity):
        return self._backend.set_gravity(gravity)

    def get_gravity(self):
        gravity = carb.Float3()
        res = self._backend.get_gravity(gravity)
        if res:
            return gravity
        else:
            return None 


class ArticulationView:
    def __init__(self, backend, frontend):
        self._backend = backend
        self._frontend = frontend

    @property
    def count(self):
        return self._backend.count

    @property
    def max_links(self):
        return self._backend.max_links

    @property
    def max_dofs(self):
        return self._backend.max_dofs

    @property
    def max_shapes(self):
        return self._backend.max_shapes

    @property
    def max_fixed_tendons(self):
        return self._backend.max_fixed_tendons

    @property
    def is_homogeneous(self):
        return self._backend.is_homogeneous

    @property
    def shared_metatype(self):
        return self._backend.shared_metatype

    @property
    def jacobian_shape(self):
        shape = self._backend.jacobian_shape
        if shape is None:
            raise Exception("Unable to obtain Jacobian shape")
        return shape

    @property
    def mass_matrix_shape(self):
        shape = self._backend.mass_matrix_shape
        if shape is None:
            raise Exception("Unable to obtain Mass Matrix shape")
        return shape

    @property
    def prim_paths(self):
        return self._backend.prim_paths

    @property
    def dof_paths(self):
        return self._backend.dof_paths

    @property
    def link_paths(self):
        return self._backend.link_paths

    def get_metatype(self, arti_idx):
        return self._backend.get_metatype(arti_idx)

    def get_dof_types(self):
        if not hasattr(self, "_dof_types"):
            self._dof_types, self._dof_types_desc = self._frontend.create_tensor((self.count, self.max_dofs), uint8, -1)

        if not self._backend.get_dof_types(self._dof_types_desc):
            raise Exception("Failed to get DOF types from backend")

        return self._dof_types

    def get_dof_motions(self):
        if not hasattr(self, "_dof_motions"):
            self._dof_motions, self._dof_motions_desc = self._frontend.create_tensor((self.count, self.max_dofs), uint8, -1)

        if not self._backend.get_dof_motions(self._dof_motions_desc):
            raise Exception("Failed to get DOF motions from backend")

        return self._dof_motions

    def get_dof_limits(self):
        if not hasattr(self, "_dof_limits"):
            self._dof_limits, self._dof_limits_desc = self._frontend.create_tensor(
                (self.count, self.max_dofs, 2), float32, -1
            )

        if not self._backend.get_dof_limits(self._dof_limits_desc):
            raise Exception("Failed to get DOF limits from backend")

        return self._dof_limits

    def get_drive_types(self):
        if not hasattr(self, "_drive_types"):
            self._drive_types, self._drive_types_desc = self._frontend.create_tensor((self.count, self.max_dofs), uint8, -1)

        if not self._backend.get_drive_types(self._drive_types_desc):
            raise Exception("Failed to get drive types from backend")

        return self._drive_types

    def get_dof_stiffnesses(self):
        if not hasattr(self, "_dof_stiffnesses"):
            self._dof_stiffnesses, self._dof_stiffnesses_desc = self._frontend.create_tensor((self.count, self.max_dofs), float32, -1)

        if not self._backend.get_dof_stiffnesses(self._dof_stiffnesses_desc):
            raise Exception("Failed to get DOF stiffnesses from backend")

        return self._dof_stiffnesses

    def get_dof_dampings(self):
        if not hasattr(self, "_dof_dampings"):
            self._dof_dampings, self._dof_dampings_desc = self._frontend.create_tensor((self.count, self.max_dofs), float32, -1)

        if not self._backend.get_dof_dampings(self._dof_dampings_desc):
            raise Exception("Failed to get DOF dampings from backend")

        return self._dof_dampings

    def get_dof_max_forces(self):
        if not hasattr(self, "_dof_max_forces"):
            self._dof_max_forces, self._dof_max_forces_desc = self._frontend.create_tensor((self.count, self.max_dofs), float32, -1)

        if not self._backend.get_dof_max_forces(self._dof_max_forces_desc):
            raise Exception("Failed to get DOF max forces from backend")

        return self._dof_max_forces

    def get_dof_friction_coefficients(self):
        if not hasattr(self, "_dof_friction_coefficients"):
            self._dof_friction_coefficients, self._dof_friction_coefficients_desc = self._frontend.create_tensor((self.count, self.max_dofs), float32, -1)

        if not self._backend.get_dof_friction_coefficients(self._dof_friction_coefficients_desc):
            raise Exception("Failed to get DOF friction coefficients from backend")

        return self._dof_friction_coefficients

    def get_dof_max_velocities(self):
        if not hasattr(self, "_dof_max_velocities"):
            self._dof_max_velocities, self._dof_max_velocities_desc = self._frontend.create_tensor((self.count, self.max_dofs), float32, -1)

        if not self._backend.get_dof_max_velocities(self._dof_max_velocities_desc):
            raise Exception("Failed to get DOF max velocities from backend")

        return self._dof_max_velocities

    def get_dof_armatures(self):
        if not hasattr(self, "_dof_armatures"):
            self._dof_armatures, self._dof_armatures_desc = self._frontend.create_tensor((self.count, self.max_dofs), float32, -1)

        if not self._backend.get_dof_armatures(self._dof_armatures_desc):
            raise Exception("Failed to get DOF armatures from backend")

        return self._dof_armatures

    def set_dof_limits(self, data, indices):
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_dof_limits(data_desc, indices_desc):
            raise Exception("Failed to set DOF limits in backend")

    def set_dof_stiffnesses(self, data, indices):
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_dof_stiffnesses(data_desc, indices_desc):
            raise Exception("Failed to set DOF stiffnesses in backend")

    def set_dof_dampings(self, data, indices):
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_dof_dampings(data_desc, indices_desc):
            raise Exception("Failed to set DOF dampings in backend")

    def set_dof_max_forces(self, data, indices):
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_dof_max_forces(data_desc, indices_desc):
            raise Exception("Failed to set DOF max forces in backend")

    def set_dof_friction_coefficients(self, data, indices):
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_dof_friction_coefficients(data_desc, indices_desc):
            raise Exception("Failed to set DOF friction coefficients in backend")

    def set_dof_max_velocities(self, data, indices):
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_dof_max_velocities(data_desc, indices_desc):
            raise Exception("Failed to set DOF max velocities in backend")

    def set_dof_armatures(self, data, indices):
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_dof_armatures(data_desc, indices_desc):
            raise Exception("Failed to set DOF armatures in backend")

    def get_link_transforms(self):
        if not hasattr(self, "_link_transforms"):
            self._link_transforms, self._link_transforms_desc = self._frontend.create_tensor(
                (self.count, self.max_links, 7), float32
            )

        if not self._backend.get_link_transforms(self._link_transforms_desc):
            raise Exception("Failed to get link tranforms from backend")

        return self._link_transforms

    def get_link_velocities(self):
        if not hasattr(self, "_link_velocities"):
            self._link_velocities, self._link_velocities_desc = self._frontend.create_tensor(
                (self.count, self.max_links, 6), float32
            )

        if not self._backend.get_link_velocities(self._link_velocities_desc):
            raise Exception("Failed to get link velocities from backend")

        return self._link_velocities
    
    def get_link_accelerations(self):
        if not hasattr(self, "_link_accelerations"):
            self._link_accelerations, self._link_accelerations_desc = self._frontend.create_tensor(
                (self.count, self.max_links, 6), float32
            )

        if not self._backend.get_link_accelerations(self._link_accelerations_desc):
            raise Exception("Failed to get link accelerations from backend")

        return self._link_accelerations
    
    def get_root_transforms(self):
        if not hasattr(self, "_root_transforms"):
            self._root_transforms, self._root_transforms_desc = self._frontend.create_tensor((self.count, 7), float32)

        if not self._backend.get_root_transforms(self._root_transforms_desc):
            raise Exception("Failed to get root link transforms from backend")

        return self._root_transforms

    def get_root_velocities(self):
        if not hasattr(self, "_root_velocities"):
            self._root_velocities, self._root_velocities_desc = self._frontend.create_tensor((self.count, 6), float32)

        if not self._backend.get_root_velocities(self._root_velocities_desc):
            raise Exception("Failed to get root link transforms from backend")

        return self._root_velocities

    def set_root_transforms(self, data, indices):
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_root_transforms(data_desc, indices_desc):
            raise Exception("Failed to set root link transforms in backend")

    def set_root_velocities(self, data, indices):
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_root_velocities(data_desc, indices_desc):
            raise Exception("Failed to set root link velocities in backend")

    def get_dof_positions(self):
        if not hasattr(self, "_dof_positions"):
            self._dof_positions, self._dof_positions_desc = self._frontend.create_tensor(
                (self.count, self.max_dofs), float32
            )

        if not self._backend.get_dof_positions(self._dof_positions_desc):
            raise Exception("Failed to get DOF positions from backend")

        return self._dof_positions

    def get_dof_velocities(self):
        if not hasattr(self, "_dof_velocities"):
            self._dof_velocities, self._dof_velocities_desc = self._frontend.create_tensor(
                (self.count, self.max_dofs), float32
            )

        if not self._backend.get_dof_velocities(self._dof_velocities_desc):
            raise Exception("Failed to get DOF velocities from backend")

        return self._dof_velocities

    def set_dof_positions(self, data, indices):
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_dof_positions(data_desc, indices_desc):
            raise Exception("Failed to set DOF positions in backend")

    def set_dof_velocities(self, data, indices):
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_dof_velocities(data_desc, indices_desc):
            raise Exception("Failed to set DOF velocities in backend")

    def set_dof_actuation_forces(self, data, indices):
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_dof_actuation_forces(data_desc, indices_desc):
            raise Exception("Failed to set DOF actuation forces in backend")

    def set_dof_position_targets(self, data, indices):
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_dof_position_targets(data_desc, indices_desc):
            raise Exception("Failed to set DOF position targets in backend")

    def set_dof_velocity_targets(self, data, indices):
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_dof_velocity_targets(data_desc, indices_desc):
            raise Exception("Failed to set DOF velocity targets in backend")

    def apply_forces_and_torques_at_position(self, force_data, torque_data, position_data, indices, is_global):
        force_data_desc = torque_data_desc = position_data_desc = None
        if(force_data is not None):
            force_data = self._frontend.as_contiguous_float32(force_data)
            force_data_desc = self._frontend.get_tensor_desc(force_data)
        if(torque_data is not None):
            torque_data = self._frontend.as_contiguous_float32(torque_data)
            torque_data_desc = self._frontend.get_tensor_desc(torque_data)
        if(position_data is not None):
            position_data = self._frontend.as_contiguous_float32(position_data)
            position_data_desc = self._frontend.get_tensor_desc(position_data)

        indices = self._frontend.as_contiguous_uint32(indices)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.apply_forces_and_torques_at_position(force_data_desc, torque_data_desc, position_data_desc, indices_desc, is_global):
            raise Exception("Failed to apply forces and torques at pos in backend")
        return True

    def get_dof_position_targets(self):
        if not hasattr(self, "_dof_position_targets"):
            self._dof_position_targets, self._dof_position_targets_desc = self._frontend.create_tensor(
                (self.count, self.max_dofs), float32
            )

        if not self._backend.get_dof_position_targets(self._dof_position_targets_desc):
            raise Exception("Failed to get DOF position targets from backend")

        return self._dof_position_targets

    def get_dof_velocity_targets(self):
        if not hasattr(self, "_dof_velocity_targets"):
            self._dof_velocity_targets, self._dof_velocity_targets_desc = self._frontend.create_tensor(
                (self.count, self.max_dofs), float32
            )

        if not self._backend.get_dof_velocity_targets(self._dof_velocity_targets_desc):
            raise Exception("Failed to get DOF velocity targets from backend")

        return self._dof_velocity_targets

    def get_dof_actuation_forces(self):
        if not hasattr(self, "_dof_actuation_forces"):
            self._dof_actuation_forces, self._dof_actuation_forces_desc = self._frontend.create_tensor(
                (self.count, self.max_dofs), float32
            )

        if not self._backend.get_dof_actuation_forces(self._dof_actuation_forces_desc):
            raise Exception("Failed to get DOF actuation forces from backend")

        return self._dof_actuation_forces

    def get_dof_projected_joint_forces(self):
        if not hasattr(self, "_dof_projected_joint_forces"):
            self._dof_projected_joint_forces, self._dof_projected_joint_forces_desc = self._frontend.create_tensor(
                (self.count, self.max_dofs), float32
            )

        if not self._backend.get_dof_projected_joint_forces(self._dof_projected_joint_forces_desc):
            raise Exception("Failed to get dof projected joint forces from backend")

        return self._dof_projected_joint_forces

    def get_jacobians(self):
        if not hasattr(self, "_jacobians"):
            jshape = self.jacobian_shape
            self._jacobians, self._jacobians_desc = self._frontend.create_tensor(
                # (self.count, jshape[0], jshape[1]), float32
                (self.count, jshape[0] // 6, 6, jshape[1]),
                float32,
            )

        if not self._backend.get_jacobians(self._jacobians_desc):
            raise Exception("Failed to get Jacobians from backend")

        return self._jacobians

    def get_mass_matrices(self):
        if not hasattr(self, "_mass_matrices"):
            shape = self.mass_matrix_shape
            self._mass_matrices, self._mass_matrices_desc = self._frontend.create_tensor(
                # (self.count, shape[0], shape[1]), float32
                (self.count, shape[0], shape[1]),
                float32,
            )

        if not self._backend.get_mass_matrices(self._mass_matrices_desc):
            raise Exception("Failed to get Mass Matrices from backend")

        return self._mass_matrices

    def get_coriolis_and_centrifugal_forces(self):
        if not hasattr(self, "_coriolis_centrifugal"):
            self._coriolis_centrifugal, self._coriolis_centrifugal_desc = self._frontend.create_tensor(
                # (self.count, self.max_dofs), float32
                (self.count, self.max_dofs),
                float32,
            )

        if not self._backend.get_coriolis_and_centrifugal_forces(self._coriolis_centrifugal_desc):
            raise Exception("Failed to get Coriolis and Centrifugal forces from backend")

        return self._coriolis_centrifugal

    def get_generalized_gravity_forces(self):
        if not hasattr(self, "_generalized_gravity"):
            self._generalized_gravity, self._generalized_gravity_desc = self._frontend.create_tensor(
                # (self.count, self.max_dofs), float32
                (self.count, self.max_dofs),
                float32,
            )

        if not self._backend.get_generalized_gravity_forces(self._generalized_gravity_desc):
            raise Exception("Failed to get Generalized Gravity forces from backend")

        return self._generalized_gravity

    def get_link_incoming_joint_force(self):
        if not hasattr(self, "_link_incoming_joint_force"):
            self._link_incoming_joint_force, self._link_incoming_joint_force_desc = self._frontend.create_tensor((self.count, self.max_links, 6), float32)

        if not self._backend.get_link_incoming_joint_force(self._link_incoming_joint_force_desc):
            raise Exception("Failed to get incoming link joint forces from backend")

        return self._link_incoming_joint_force

    def get_masses(self):
        if not hasattr(self, "_masses"):
            self._masses, self._masses_desc = self._frontend.create_tensor((self.count, self.max_links), float32, -1)

        if not self._backend.get_masses(self._masses_desc):
            raise Exception("Failed to get articulation masses from backend")

        return self._masses

    def get_inv_masses(self):
        if not hasattr(self, "_inv_masses"):
            self._inv_masses, self._inv_masses_desc = self._frontend.create_tensor((self.count, self.max_links), float32, -1)

        if not self._backend.get_inv_masses(self._inv_masses_desc):
            raise Exception("Failed to get articulation inv masses from backend")

        return self._inv_masses

    def get_coms(self):
        if not hasattr(self, "_coms"):
            self._coms, self._com_desc = self._frontend.create_tensor((self.count, self.max_links, 7), float32, -1)

        if not self._backend.get_coms(self._com_desc):
            raise Exception("Failed to get articulation coms from backend")

        return self._coms

    def get_inertias(self):
        if not hasattr(self, "_inertias"):
            self._inertias, self._inertias_desc = self._frontend.create_tensor((self.count, self.max_links, 9), float32, -1)

        if not self._backend.get_inertias(self._inertias_desc):
            raise Exception("Failed to get articulation inertias from backend")

        return self._inertias

    def get_inv_inertias(self):
        if not hasattr(self, "_inv_inertias"):
            self._inv_inertias, self._inv_inertias_desc = self._frontend.create_tensor((self.count, self.max_links, 9), float32, -1)

        if not self._backend.get_inv_inertias(self._inv_inertias_desc):
            raise Exception("Failed to get articulation inv inertias from backend")

        return self._inv_inertias

    def get_disable_gravities(self):
        if not hasattr(self, "_disable_gravities"):
            self._disable_gravities, self._disable_gravities_desc = self._frontend.create_tensor((self.count, self.max_links), uint8, -1)

        if not self._backend.get_disable_gravities(self._disable_gravities_desc):
            raise Exception("Failed to get rigid body disable gravities from backend")

        return self._disable_gravities

    def set_masses(self, data, indices):
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_masses(data_desc, indices_desc):
            raise Exception("Failed to set articulation masses in backend")

    def set_coms(self, data, indices):
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_coms(data_desc, indices_desc):
            raise Exception("Failed to set articulation coms in backend")

    def set_inertias(self, data, indices):
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_inertias(data_desc, indices_desc):
            raise Exception("Failed to set articulation inertias in backend")

    def set_disable_gravities(self, data, indices):
        data = self._frontend.as_contiguous_uint8(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_disable_gravities(data_desc, indices_desc):
            raise Exception("Failed to set rigid body disable gravities in backend")

    def get_material_properties(self):
        if not hasattr(self, "_material_properties"):
            self._material_properties, self._material_properties_desc = self._frontend.create_tensor((self.count, self.max_shapes, 3), float32, -1)

        if not self._backend.get_material_properties(self._material_properties_desc):
            raise Exception("Failed to get articulation material properties from backend")

        return self._material_properties

    def get_contact_offsets(self):
        if not hasattr(self, "_contact_offsets"):
            self._contact_offsets, self._contact_offsets_desc = self._frontend.create_tensor((self.count, self.max_shapes), float32, -1)

        if not self._backend.get_contact_offsets(self._contact_offsets_desc):
            raise Exception("Failed to get articulation contact offsets from backend")

        return self._contact_offsets

    def get_rest_offsets(self):
        if not hasattr(self, "_rest_offsets"):
            self._rest_offsets, self._rest_offsets_desc = self._frontend.create_tensor((self.count, self.max_shapes), float32, -1)

        if not self._backend.get_rest_offsets(self._rest_offsets_desc):
            raise Exception("Failed to get articulation rest offsets from backend")

        return self._rest_offsets

    def set_material_properties(self, data, indices):
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_material_properties(data_desc, indices_desc):
            raise Exception("Failed to set articulation material properties in backend")

    def set_contact_offsets(self, data, indices):
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_contact_offsets(data_desc, indices_desc):
            raise Exception("Failed to set articulation contact offsets in backend")

    def set_rest_offsets(self, data, indices):
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_rest_offsets(data_desc, indices_desc):
            raise Exception("Failed to set articulation rest offsets in backend")

    def get_fixed_tendon_stiffnesses(self):
        if not hasattr(self, "fixed_tendon_stiffnesses"):
            self.fixed_tendon_stiffnesses, self._fixed_tendon_stiffnesses_desc = self._frontend.create_tensor((self.count, self.max_fixed_tendons), float32)

        if not self._backend.get_fixed_tendon_stiffnesses(self._fixed_tendon_stiffnesses_desc):
            raise Exception("Failed to get articulation fixed tendon stiffnesses from backend")

        return self.fixed_tendon_stiffnesses

    def get_fixed_tendon_dampings(self):
        if not hasattr(self, "fixed_tendon_dampings"):
            self.fixed_tendon_dampings, self._fixed_tendon_dampings_desc = self._frontend.create_tensor((self.count, self.max_fixed_tendons), float32)

        if not self._backend.get_fixed_tendon_dampings(self._fixed_tendon_dampings_desc):
            raise Exception("Failed to get articulation fixed tendon dampings from backend")

        return self.fixed_tendon_dampings

    def get_fixed_tendon_limit_stiffnesses(self):
        if not hasattr(self, "fixed_tendon_limit_stiffnesses"):
            self.fixed_tendon_limit_stiffnesses, self._fixed_tendon_limit_stiffnesses_desc = self._frontend.create_tensor((self.count, self.max_fixed_tendons), float32)

        if not self._backend.get_fixed_tendon_limit_stiffnesses(self._fixed_tendon_limit_stiffnesses_desc):
            raise Exception("Failed to get articulation fixed tendon limit stiffnesses from backend")

        return self.fixed_tendon_limit_stiffnesses

    def get_fixed_tendon_limits(self):
        if not hasattr(self, "fixed_tendon_limits"):
            self.fixed_tendon_limits, self._fixed_tendon_limits_desc = self._frontend.create_tensor((self.count, self.max_fixed_tendons, 2), float32)

        if not self._backend.get_fixed_tendon_limits(self._fixed_tendon_limits_desc):
            raise Exception("Failed to get articulation fixed tendon limits from backend")

        return self.fixed_tendon_limits

    def get_fixed_tendon_rest_lengths(self):
        if not hasattr(self, "fixed_tendon_rest_lengths"):
            self.fixed_tendon_rest_lengths, self._fixed_tendon_rest_lengths_desc = self._frontend.create_tensor((self.count, self.max_fixed_tendons), float32)

        if not self._backend.get_fixed_tendon_rest_lengths(self._fixed_tendon_rest_lengths_desc):
            raise Exception("Failed to get articulation fixed tendon rest_lengths from backend")

        return self.fixed_tendon_rest_lengths

    def get_fixed_tendon_offsets(self):
        if not hasattr(self, "fixed_tendon_offsets"):
            self.fixed_tendon_offsets, self._fixed_tendon_offsets_desc = self._frontend.create_tensor((self.count, self.max_fixed_tendons), float32)

        if not self._backend.get_fixed_tendon_offsets(self._fixed_tendon_offsets_desc):
            raise Exception("Failed to get articulation fixed tendon offsets from backend")

        return self.fixed_tendon_offsets

    def set_fixed_tendon_properties(self, stiffnesses, dampings, limit_stiffnesses, limits, rest_lengths, offsets, indices):
        stiffnesses = self._frontend.as_contiguous_float32(stiffnesses)
        stiffness_desc = self._frontend.get_tensor_desc(stiffnesses)
        dampings = self._frontend.as_contiguous_float32(dampings)
        damping_desc = self._frontend.get_tensor_desc(dampings)
        limit_stiffnesses = self._frontend.as_contiguous_float32(limit_stiffnesses)
        limit_stiffness_desc = self._frontend.get_tensor_desc(limit_stiffnesses)
        limits = self._frontend.as_contiguous_float32(limits)
        limits_desc = self._frontend.get_tensor_desc(limits)
        rest_lengths = self._frontend.as_contiguous_float32(rest_lengths)
        rest_length_desc = self._frontend.get_tensor_desc(rest_lengths)
        offsets = self._frontend.as_contiguous_float32(offsets)
        offset_desc = self._frontend.get_tensor_desc(offsets)

        indices = self._frontend.as_contiguous_uint32(indices)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_fixed_tendon_properties(stiffness_desc, damping_desc, limit_stiffness_desc, limits_desc, rest_length_desc, offset_desc, indices_desc):
            raise Exception("Failed to set articulation fixed tendon properties in backend")

    def check(self):
        return self._backend.check()


class RigidBodyView:
    def __init__(self, backend, frontend):
        self._backend = backend
        self._frontend = frontend

    @property
    def count(self):
        return self._backend.count

    @property
    def max_shapes(self):
        return self._backend.max_shapes

    @property
    def prim_paths(self):
        return self._backend.prim_paths

    def get_transforms(self):
        if not hasattr(self, "_transforms"):
            self._transforms, self._transforms_desc = self._frontend.create_tensor((self.count, 7), float32)

        if not self._backend.get_transforms(self._transforms_desc):
            raise Exception("Failed to get rigid body transforms from backend")

        return self._transforms

    def get_velocities(self):
        if not hasattr(self, "_velocities"):
            self._velocities, self._velocities_desc = self._frontend.create_tensor((self.count, 6), float32)

        if not self._backend.get_velocities(self._velocities_desc):
            raise Exception("Failed to get rigid body velocities from backend")

        return self._velocities

    def get_accelerations(self):
        if not hasattr(self, "_accelerations"):
            self._accelerations, self._accelerations_desc = self._frontend.create_tensor((self.count, 6), float32)

        if not self._backend.get_accelerations(self._accelerations_desc):
            raise Exception("Failed to get rigid body accelerations from backend")

        return self._accelerations

    def set_kinematic_targets(self, data, indices):
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_kinematic_targets(data_desc, indices_desc):
            raise Exception("Failed to set rigid body kinematic targets in backend")

    def set_dynamic_targets(self, data, indices):
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_dynamic_targets(data_desc, indices_desc):
            raise Exception("Failed to set rigid body dynamic targets in backend")

    def set_transforms(self, data, indices):
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_transforms(data_desc, indices_desc):
            raise Exception("Failed to set rigid body transforms in backend")

    def set_velocities(self, data, indices):
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_velocities(data_desc, indices_desc):
            raise Exception("Failed to set rigid body velocities in backend")

    def apply_forces_and_torques_at_position(self, force_data, torque_data, position_data, indices, is_global):
        force_data_desc = torque_data_desc = position_data_desc = None
        if(force_data is not None):
            force_data = self._frontend.as_contiguous_float32(force_data)
            force_data_desc = self._frontend.get_tensor_desc(force_data)
        if(torque_data is not None):
            torque_data = self._frontend.as_contiguous_float32(torque_data)
            torque_data_desc = self._frontend.get_tensor_desc(torque_data)
        if(position_data is not None):
            position_data = self._frontend.as_contiguous_float32(position_data)
            position_data_desc = self._frontend.get_tensor_desc(position_data)

        indices = self._frontend.as_contiguous_uint32(indices)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.apply_forces_and_torques_at_position(force_data_desc, torque_data_desc, position_data_desc, indices_desc, is_global):
            raise Exception("Failed to apply forces and torques at pos in backend")
        return True

    def apply_forces(self, data, indices, is_global=True):
        if not self.apply_forces_and_torques_at_position(data, None, None, indices, is_global):
            raise Exception("Failed to apply forces in backend")

    def get_masses(self):
        if not hasattr(self, "_masses"):
            self._masses, self._masses_desc = self._frontend.create_tensor((self.count, 1), float32, -1)

        if not self._backend.get_masses(self._masses_desc):
            raise Exception("Failed to get rigid body masses from backend")

        return self._masses

    def get_inv_masses(self):
        if not hasattr(self, "_inv_masses"):
            self._inv_masses, self._inv_masses_desc = self._frontend.create_tensor((self.count, 1), float32, -1)

        if not self._backend.get_inv_masses(self._inv_masses_desc):
            raise Exception("Failed to get rigid body inv masses from backend")

        return self._inv_masses

    def get_coms(self):
        if not hasattr(self, "_coms"):
            self._coms, self._com_desc = self._frontend.create_tensor((self.count, 7), float32, -1)

        if not self._backend.get_coms(self._com_desc):
            raise Exception("Failed to get rigid body coms from backend")

        return self._coms

    def get_inertias(self):
        if not hasattr(self, "_inertias"):
            self._inertias, self._inertias_desc = self._frontend.create_tensor((self.count, 9), float32, -1)

        if not self._backend.get_inertias(self._inertias_desc):
            raise Exception("Failed to get rigid body inertias from backend")

        return self._inertias

    def get_inv_inertias(self):
        if not hasattr(self, "_inv_inertias"):
            self._inv_inertias, self._inv_inertias_desc = self._frontend.create_tensor((self.count, 9), float32, -1)

        if not self._backend.get_inv_inertias(self._inv_inertias_desc):
            raise Exception("Failed to get rigid body inv inertias from backend")

        return self._inv_inertias

    def get_disable_gravities(self):
        if not hasattr(self, "_disable_gravities"):
            self._disable_gravities, self._disable_gravities_desc = self._frontend.create_tensor((self.count, 1), uint8, -1)

        if not self._backend.get_disable_gravities(self._disable_gravities_desc):
            raise Exception("Failed to get rigid body disable gravities from backend")

        return self._disable_gravities

    def get_disable_simulations(self):
        if not hasattr(self, "_disable_simulations"):
            self._disable_simulations, self._disable_simulations_desc = self._frontend.create_tensor((self.count, 1), uint8, -1)

        if not self._backend.get_disable_simulations(self._disable_simulations_desc):
            raise Exception("Failed to get rigid body disable simulations from backend")

        return self._disable_simulations

    def set_masses(self, data, indices):
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_masses(data_desc, indices_desc):
            raise Exception("Failed to set rigid body masses in backend")

    def set_coms(self, data, indices):
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_coms(data_desc, indices_desc):
            raise Exception("Failed to set rigid body coms in backend")

    def set_inertias(self, data, indices):
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_inertias(data_desc, indices_desc):
            raise Exception("Failed to set rigid body inertias in backend")

    def set_disable_gravities(self, data, indices):
        data = self._frontend.as_contiguous_uint8(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_disable_gravities(data_desc, indices_desc):
            raise Exception("Failed to set rigid body disable gravities in backend")

    def set_disable_simulations(self, data, indices):
        data = self._frontend.as_contiguous_uint8(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_disable_simulations(data_desc, indices_desc):
            raise Exception("Failed to set rigid body disable simulations in backend")

    def get_material_properties(self):
        if not hasattr(self, "_material_properties"):
            self._material_properties, self._material_properties_desc = self._frontend.create_tensor((self.count, self.max_shapes, 3), float32, -1)

        if not self._backend.get_material_properties(self._material_properties_desc):
            raise Exception("Failed to get rigid body material properties from backend")

        return self._material_properties

    def get_contact_offsets(self):
        if not hasattr(self, "_contact_offsets"):
            self._contact_offsets, self._contact_offsets_desc = self._frontend.create_tensor((self.count, self.max_shapes), float32, -1)

        if not self._backend.get_contact_offsets(self._contact_offsets_desc):
            raise Exception("Failed to get rigid body contact offsets from backend")

        return self._contact_offsets

    def get_rest_offsets(self):
        if not hasattr(self, "rest_offsets"):
            self.rest_offsets, self._rest_offsets_desc = self._frontend.create_tensor((self.count, self.max_shapes), float32, -1)

        if not self._backend.get_rest_offsets(self._rest_offsets_desc):
            raise Exception("Failed to get rigid body rest offsets from backend")

        return self.rest_offsets

    def set_material_properties(self, data, indices):
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_material_properties(data_desc, indices_desc):
            raise Exception("Failed to set rigid body material properties in backend")

    def set_contact_offsets(self, data, indices):
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_contact_offsets(data_desc, indices_desc):
            raise Exception("Failed to set rigid body contact offsets in backend")

    def set_rest_offsets(self, data, indices):
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_rest_offsets(data_desc, indices_desc):
            raise Exception("Failed to set rigid body rest offsets in backend")

    def check(self):
        return self._backend.check()

class SoftBodyView:
    def __init__(self, backend, frontend):
        self._backend = backend
        self._frontend = frontend

    @property
    def count(self):
        return self._backend.count

    @property
    def max_elements_per_body(self):
        return self._backend.max_elements_per_body
    
    @property
    def max_vertices_per_body(self):
        return self._backend.max_vertices_per_body

    @property
    def max_sim_elements_per_body(self):
        return self._backend.max_sim_elements_per_body
    
    @property
    def max_sim_vertices_per_body(self):
        return self._backend.max_sim_vertices_per_body

    def get_element_stresses(self):
        if not hasattr(self, "_element_stresses"):
            self._element_stresses, self._element_stresses_desc = self._frontend.create_tensor((self.count, self.max_elements_per_body, 9), float32)

        if not self._backend.get_element_stresses(self._element_stresses_desc):
            raise Exception("Failed to get soft body element Cauchy stresses from backend")

        return self._element_stresses

    def get_element_deformation_gradients(self):
        if not hasattr(self, "_element_deformation_gradients"):
            self._element_deformation_gradients, self._element_deformation_gradients_desc = self._frontend.create_tensor((self.count, self.max_elements_per_body, 9), float32)

        if not self._backend.get_element_deformation_gradients(self._element_deformation_gradients_desc):
            raise Exception("Failed to get soft body element deformation gradients from backend")

        return self._element_deformation_gradients

    def get_element_rest_poses(self):
        if not hasattr(self, "_element_rest_poses"):
            self._element_rest_poses, self._element_rest_poses_desc = self._frontend.create_tensor((self.count, self.max_elements_per_body * 9), float32)

        if not self._backend.get_element_rest_poses(self._element_rest_poses_desc):
            raise Exception("Failed to get soft body element rest poses from backend")

        return self._element_rest_poses

    def get_element_rotations(self):
        if not hasattr(self, "_element_rotations"):
            self._element_rotations, self._element_rotations_desc = self._frontend.create_tensor((self.count, self.max_elements_per_body * 4), float32)

        if not self._backend.get_element_rotations(self._element_rotations_desc):
            raise Exception("Failed to get soft body element rotations from backend")

        return self._element_rotations

    def get_nodal_positions(self):
        if not hasattr(self, "_nodal_positions"):
            self._nodal_positions, self._nodal_positions_desc = self._frontend.create_tensor((self.count, self.max_vertices_per_body, 3), float32)

        if not self._backend.get_nodal_positions(self._nodal_positions_desc):
            raise Exception("Failed to get soft body nodal positions from backend")

        return self._nodal_positions

    def get_element_indices(self):
        if not hasattr(self, "_element_indices"):
            self._element_indices, self._element_indices_desc = self._frontend.create_tensor((self.count, self.max_elements_per_body, 4), uint32)

        if not self._backend.get_element_indices(self._element_indices_desc):
            raise Exception("Failed to get soft body simulation mesh element indices from backend")

        return self._element_indices

    ### simulation mesh data
    def get_sim_element_indices(self):
        if not hasattr(self, "_sim_element_indices"):
            self._sim_element_indices, self._sim_element_indices_desc = self._frontend.create_tensor((self.count, self.max_sim_elements_per_body, 4), uint32)

        if not self._backend.get_sim_element_indices(self._sim_element_indices_desc):
            raise Exception("Failed to get soft body simulation mesh element indices from backend")

        return self._sim_element_indices

    def get_sim_element_stresses(self):
        if not hasattr(self, "_sim_element_stresses"):
            self._sim_element_stresses, self._sim_element_stresses_desc = self._frontend.create_tensor((self.count, self.max_sim_elements_per_body, 9), float32)

        if not self._backend.get_sim_element_stresses(self._sim_element_stresses_desc):
            raise Exception("Failed to get soft body simulation element stresses from backend")

        return self._sim_element_stresses

    def get_sim_element_deformation_gradients(self):
        if not hasattr(self, "_sim_element_deformation_gradients"):
            self._sim_element_deformation_gradients, self._sim_element_deformation_gradients_desc = self._frontend.create_tensor((self.count, self.max_sim_elements_per_body, 9), float32)
        if not self._backend.get_sim_element_deformation_gradients(self._sim_element_deformation_gradients_desc):
            raise Exception("Failed to get soft body simulation element deformation gradients from backend")

        return self._sim_element_deformation_gradients

    def get_sim_element_rest_poses(self):
        if not hasattr(self, "_sim_element_rest_poses"):
            self._sim_element_rest_poses, self._sim_element_rest_poses_desc = self._frontend.create_tensor((self.count, self.max_sim_elements_per_body * 9), float32)

        if not self._backend.get_sim_element_rest_poses(self._sim_element_rest_poses_desc):
            raise Exception("Failed to get soft body simulation element rest poses from backend")

        return self._sim_element_rest_poses

    def get_sim_element_rotations(self):
        if not hasattr(self, "_sim_element_rotations"):
            self._sim_element_rotations, self._sim_element_rotations_desc = self._frontend.create_tensor((self.count, self.max_sim_elements_per_body * 4), float32)

        if not self._backend.get_sim_element_rotations(self._sim_element_rotations_desc):
            raise Exception("Failed to get soft body element rotations from backend")

        return self._sim_element_rotations

    def get_sim_nodal_positions(self):
        if not hasattr(self, "_sim_nodal_positions"):
            self._sim_nodal_positions, self._sim_nodal_positions_desc = self._frontend.create_tensor((self.count, self.max_sim_vertices_per_body, 3), float32)

        if not self._backend.get_sim_nodal_positions(self._sim_nodal_positions_desc):
            raise Exception("Failed to get soft body simulation mesh nodal positions from backend")

        return self._sim_nodal_positions

    def set_sim_nodal_positions(self, data, indices):
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_sim_nodal_positions(data_desc, indices_desc):
            raise Exception("Failed to set soft body simulation mesh nodal positions in backend")

    def get_sim_nodal_velocities(self):
        if not hasattr(self, "_sim_nodal_velocities"):
            self._sim_nodal_velocities, self._sim_nodal_velocities_desc = self._frontend.create_tensor((self.count, self.max_sim_vertices_per_body, 3), float32)

        if not self._backend.get_sim_nodal_velocities(self._sim_nodal_velocities_desc):
            raise Exception("Failed to get soft body simulation mesh nodal velocities from backend")

        return self._sim_nodal_velocities

    def set_sim_nodal_velocities(self, data, indices):
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_sim_nodal_velocities(data_desc, indices_desc):
            raise Exception("Failed to set soft body simulation mesh nodal velocities in backend")

    def get_sim_kinematic_targets(self):
        if not hasattr(self, "_sim_kinematic_targets"):
            self._sim_kinematic_targets, self._sim_kinematic_targets_desc = self._frontend.create_tensor((self.count, self.max_sim_vertices_per_body, 4), float32)

        if not self._backend.get_sim_kinematic_targets(self._sim_kinematic_targets_desc):
            raise Exception("Failed to get soft body simulation mesh kinematic targets from backend")

        return self._sim_kinematic_targets
    
    def set_sim_kinematic_targets(self, data, indices):
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_sim_kinematic_targets(data_desc, indices_desc):
            raise Exception("Failed to set soft body simulation mesh kinematic targets in backend")

    def get_transforms(self):
        if not hasattr(self, "_transforms"):
            self._transforms, self._transforms_desc = self._frontend.create_tensor((self.count, 7), float32)

        if not self._backend.get_transforms(self._transforms_desc):
            raise Exception("Failed to get soft body transforms from backend")

        return self._transforms
        
    def check(self):
        return self._backend.check()


class SoftBodyMaterialView:
    def __init__(self, backend, frontend):
        self._backend = backend
        self._frontend = frontend

    @property
    def count(self):
        return self._backend.count

    def get_dynamic_friction(self):
        if not hasattr(self, "_dynamic_friction"):
            self._dynamic_friction, self._dynamic_friction_desc = self._frontend.create_tensor((self.count, 1), float32, -1)
        
        if not self._backend.get_dynamic_friction(self._dynamic_friction_desc):
            raise Exception("Failed to get soft body material dynamic friction from backend")

        return self._dynamic_friction

    def set_dynamic_friction(self, data, indices):
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_dynamic_friction(data_desc, indices_desc):
            raise Exception("Failed to set soft body material dynamic friction in backend")

    def get_damping(self):
        if not hasattr(self, "_damping"):
            self._damping, self._damping_desc = self._frontend.create_tensor((self.count, 1), float32, -1)
        
        if not self._backend.get_damping(self._damping_desc):
            raise Exception("Failed to get soft body material damping from backend")

        return self._damping

    def set_damping(self, data, indices):
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_damping(data_desc, indices_desc):
            raise Exception("Failed to set soft body material damping scale in backend")

    def get_damping_scale(self):
        if not hasattr(self, "_damping_scale"):
            self._damping_scale, self._damping_scale_desc = self._frontend.create_tensor((self.count, 1), float32, -1)
        
        if not self._backend.get_damping_scale(self._damping_scale_desc):
            raise Exception("Failed to get soft body material damping scale from backend")

        return self._damping_scale

    def set_damping_scale(self, data, indices):
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_damping_scale(data_desc, indices_desc):
            raise Exception("Failed to set soft body material damping scale in backend")

    def get_poissons_ratio(self):
        if not hasattr(self, "_poissons_ratio"):
            self._poissons_ratio, self._poissons_desc = self._frontend.create_tensor((self.count, 1), float32, -1)
        
        if not self._backend.get_poissons_ratio(self._poissons_desc):
            raise Exception("Failed to get soft body material poissons ratio from backend")

        return self._poissons_ratio

    def set_poissons_ratio(self, data, indices):
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_poissons_ratio(data_desc, indices_desc):
            raise Exception("Failed to set soft body material poissons ratio in backend")

    def get_youngs_modulus(self):
        if not hasattr(self, "_youngs_modulus"):
            self._youngs_modulus, self._youngs_modulus_desc = self._frontend.create_tensor((self.count, 1), float32, -1)
        
        if not self._backend.get_youngs_modulus(self._youngs_modulus_desc):
            raise Exception("Failed to get soft body material youngs modulus from backend")

        return self._youngs_modulus

    def set_youngs_modulus(self, data, indices):
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_youngs_modulus(data_desc, indices_desc):
            raise Exception("Failed to set soft body material youngs modulus in backend")

    def check(self):
        return self._backend.check()


class RigidContactView:
    def __init__(self, backend, frontend):
        self._backend = backend
        self._frontend = frontend
        self._num_components = 0

    @property
    def sensor_count(self):
        return self._backend.sensor_count

    @property
    def filter_count(self):
        return self._backend.filter_count

    @property
    def max_contact_data_count(self):
        return self._backend.max_contact_data_count

    @property
    def sensor_paths(self):
        return self._backend.sensor_paths

    @property
    def sensor_names(self):
        return self._backend.sensor_names

    def get_net_contact_forces(self, dt):
        if not hasattr(self, "_net_forces"):
            self._net_forces, self._net_forces_desc = self._frontend.create_tensor(
                (self.sensor_count, 3), float32
            )
        if not self._backend.get_net_contact_forces(self._net_forces_desc, dt):
            raise Exception("Failed to get net contact forces from backend")
        else:
            return self._net_forces

    def get_contact_force_matrix(self, dt):
        if self.filter_count == 0:
            raise Exception("Contact force matrix is not available because no filters were specified")
        if not hasattr(self, "_matrix"):
            self._matrix, self._matrix_desc = self._frontend.create_tensor(
                (self.sensor_count, self.filter_count, 3), float32)
        if not self._backend.get_contact_force_matrix(self._matrix_desc, dt):
            raise Exception("Failed to get contact force matrix from backend")
        return self._matrix

    def get_contact_data(self, dt):
        if self.filter_count == 0:
            raise Exception("Contact data is not available because no filters were specified")

        if self.max_contact_data_count == 0:
            raise Exception("No contact data is available because create_rigid_contact_view is initialized with max_contact_data_count = 0")

        if not hasattr(self, "_force_buffer"):
            self._force_buffer, self._force_buffer_desc = self._frontend.create_tensor(
                (self.max_contact_data_count, 1), float32)         
        if not hasattr(self, "_point_buffer"):
            self._point_buffer, self._point_buffer_desc = self._frontend.create_tensor(
                (self.max_contact_data_count, 3), float32)
        if not hasattr(self, "_normal_buffer"):
            self._normal_buffer, self._normal_buffer_desc = self._frontend.create_tensor(
                (self.max_contact_data_count, 3), float32)
        if not hasattr(self, "_separation_buffer"):
            self._separation_buffer, self._separation_buffer_desc = self._frontend.create_tensor(
                (self.max_contact_data_count, 1), float32)  
        if not hasattr(self, "_start_indices_buffer"):
            self._start_indices_buffer, self._start_indices_buffer_desc = self._frontend.create_tensor(
                (self.sensor_count, self.filter_count), uint32)
        if not hasattr(self, "_contact_count_buffer"):
            self._contact_count_buffer, self._contact_count_buffer_desc = self._frontend.create_tensor(
                (self.sensor_count, self.filter_count), uint32)  

        if not self._backend.get_contact_data(self._force_buffer_desc, self._point_buffer_desc, self._normal_buffer_desc, 
                                                self._separation_buffer_desc, self._contact_count_buffer_desc, self._start_indices_buffer_desc, dt):
            raise Exception("Failed to get contact data from backend")

        return self._force_buffer, self._point_buffer, self._normal_buffer, self._separation_buffer, self._contact_count_buffer, self._start_indices_buffer,

    def get_friction_data(self, dt):
        if self.filter_count == 0:
            raise Exception("Friction data is not available because no filters were specified")

        if self.max_contact_data_count == 0:
            raise Exception("No contact data is available because create_rigid_contact_view is initialized with max_contact_data_count = 0")

        if not hasattr(self, "_friction_force_buffer"):
            self._friction_force_buffer, self._friction_force_buffer_desc = self._frontend.create_tensor(
                (self.max_contact_data_count, 3), float32)         
        if not hasattr(self, "_friction_point_buffer"):
            self._friction_point_buffer, self._friction_point_buffer_desc = self._frontend.create_tensor(
                (self.max_contact_data_count, 3), float32)
        if not hasattr(self, "_start_indices_buffer"):
            self._start_indices_buffer, self._start_indices_buffer_desc = self._frontend.create_tensor(
                (self.sensor_count, self.filter_count), uint32)
        if not hasattr(self, "_contact_count_buffer"):
            self._contact_count_buffer, self._contact_count_buffer_desc = self._frontend.create_tensor(
                (self.sensor_count, self.filter_count), uint32)  

        if not self._backend.get_friction_data(self._friction_force_buffer_desc, self._friction_point_buffer_desc, self._contact_count_buffer_desc, self._start_indices_buffer_desc, dt):
            raise Exception("Failed to get friction data from backend")

        return self._friction_force_buffer, self._friction_point_buffer, self._contact_count_buffer, self._start_indices_buffer,

    def check(self):
        return self._backend.check()

class SdfShapeView:
    def __init__(self, backend, frontend):
        self._backend = backend
        self._frontend = frontend

    @property
    def count(self):
        return self._backend.count

    @property
    def max_num_points(self):
        return self._backend.max_num_points

    @property
    def object_paths(self):
        return self._backend.object_paths

    def get_sdf_and_gradients(self, data):
        data = self._frontend.as_contiguous_float32(data)
        data_desc = self._frontend.get_tensor_desc(data)
        if not hasattr(self, "_sdf_and_gradients"):
            self._sdf_and_gradients, self._sdf_and_gradients_desc = self._frontend.create_tensor(
                (self.count, self._backend.max_num_points, 4), float32
            )
        if not self._backend.get_sdf_and_gradients(self._sdf_and_gradients_desc, data_desc):
            raise Exception("Failed to get sdf and gradients from backend")
        else:
            return self._sdf_and_gradients

    def check(self):
        return self._backend.check()

class ParticleSystemView:
    def __init__(self, backend, frontend):
        self._backend = backend
        self._frontend = frontend

    @property
    def count(self):
        return self._backend.count

    def get_solid_rest_offsets(self):
        if not hasattr(self, "_solid_rest_offsets"):
            self._solid_rest_offsets, self._solid_rest_offsets_desc = self._frontend.create_tensor((self.count, 1), float32, -1)
        
        if not self._backend.get_solid_rest_offsets(self._solid_rest_offsets_desc):
            raise Exception("Failed to get particle system solid rest offsets from backend")

        return self._solid_rest_offsets

    def set_solid_rest_offsets(self, data, indices):
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_solid_rest_offsets(data_desc, indices_desc):
            raise Exception("Failed to set particle system solid rest offsets in backend")

    def get_fluid_rest_offsets(self):
        if not hasattr(self, "_fluid_rest_offsets"):
            self._fluid_rest_offsets, self._fluid_rest_offsets_desc = self._frontend.create_tensor((self.count, 1), float32, -1)
        
        if not self._backend.get_fluid_rest_offsets(self._fluid_rest_offsets_desc):
            raise Exception("Failed to get particle system fluid rest offsets from backend")

        return self._fluid_rest_offsets

    def set_fluid_rest_offsets(self, data, indices):
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_fluid_rest_offsets(data_desc, indices_desc):
            raise Exception("Failed to set particle system fluid rest offsets in backend")

    def get_particle_contact_offsets(self):
        if not hasattr(self, "_particle_contact_offsets"):
            self._particle_contact_offsets, self._particle_contact_offsets_desc = self._frontend.create_tensor((self.count, 1), float32, -1)
        
        if not self._backend.get_particle_contact_offsets(self._particle_contact_offsets_desc):
            raise Exception("Failed to get particle system particle contact offsets from backend")

        return self._particle_contact_offsets

    def set_particle_contact_offsets(self, data, indices):
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_particle_contact_offsets(data_desc, indices_desc):
            raise Exception("Failed to set particle system particle contact offsets in backend")

    def get_wind(self):
        if not hasattr(self, "_wind"):
            self._wind, self._wind_desc = self._frontend.create_tensor((self.count, 3), float32, -1)
        
        if not self._backend.get_wind(self._wind_desc):
            raise Exception("Failed to get particle system particle contact offsets from backend")

        return self._wind

    def set_wind(self, data, indices):
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_wind(data_desc, indices_desc):
            raise Exception("Failed to set particle system particle wind in backend")

    def check(self):
        return self._backend.check()


class ParticleClothView:
    def __init__(self, backend, frontend):
        self._backend = backend
        self._frontend = frontend

    @property
    def count(self):
        return self._backend.count

    @property
    def max_particles_per_cloth(self):
        return self._backend.max_particles_per_cloth

    @property
    def max_springs_per_cloth(self):
        return self._backend.max_springs_per_cloth

    def get_positions(self):
        if not hasattr(self, "_positions"):
            self._positions, self._positions_desc = self._frontend.create_tensor((self.count, self.max_particles_per_cloth * 3), float32)
        
        if not self._backend.get_positions(self._positions_desc):
            raise Exception("Failed to get particle cloth positions from backend")

        return self._positions

    def set_positions(self, data, indices):
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_positions(data_desc, indices_desc):
            raise Exception("Failed to set particle cloth positions in backend")

    def get_velocities(self):
        if not hasattr(self, "_velocities"):
            self._velocities, self._velocities_desc = self._frontend.create_tensor((self.count, self.max_particles_per_cloth * 3), float32)

        if not self._backend.get_velocities(self._velocities_desc):
            raise Exception("Failed to get particle cloth velocities from backend")

        return self._velocities

    def set_velocities(self, data, indices):
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_velocities(data_desc, indices_desc):
            raise Exception("Failed to set particle cloth velocities in backend")

    def get_masses(self):
        if not hasattr(self, "_masses"):
            self._masses, self._masses_desc = self._frontend.create_tensor((self.count, self.max_particles_per_cloth), float32)

        if not self._backend.get_masses(self._masses_desc):
            raise Exception("Failed to get particle cloth masses from backend")

        return self._masses

    def set_masses(self, data, indices):
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_masses(data_desc, indices_desc):
            raise Exception("Failed to set particle cloth masses in backend")

    def get_spring_damping(self):
        if not hasattr(self, "_spring_damping"):
            self._spring_damping, self.spring_damping_desc = self._frontend.create_tensor((self.count, self.max_springs_per_cloth), float32)

        if not self._backend.get_spring_damping(self.spring_damping_desc):
            raise Exception("Failed to get particle spring damping from backend")

        return self._spring_damping     

    def set_spring_damping(self, data, indices):
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_spring_damping(data_desc, indices_desc):
            raise Exception("Failed to set particle spring damping in backend")

    def get_spring_stiffness(self):
        if not hasattr(self, "_spring_stiffness"):
            self._spring_stiffness, self.spring_stiffness_desc = self._frontend.create_tensor((self.count, self.max_springs_per_cloth), float32)

        if not self._backend.get_spring_stiffness(self.spring_stiffness_desc):
            raise Exception("Failed to get particle spring stiffness from backend")

        return self._spring_stiffness    

    def set_spring_stiffness(self, data, indices):
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_spring_stiffness(data_desc, indices_desc):
            raise Exception("Failed to set particle spring stiffness in backend")            

    def check(self):
        return self._backend.check()


class ParticleMaterialView:
    def __init__(self, backend, frontend):
        self._backend = backend
        self._frontend = frontend

    @property
    def count(self):
        return self._backend.count

    def get_friction(self):
        if not hasattr(self, "_friction"):
            self._friction, self._friction_desc = self._frontend.create_tensor((self.count, 1), float32, -1)
        
        if not self._backend.get_friction(self._friction_desc):
            raise Exception("Failed to get particle material friction from backend")

        return self._friction

    def set_friction(self, data, indices):
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_friction(data_desc, indices_desc):
            raise Exception("Failed to set particle material friction in backend")

    def get_damping(self):
        if not hasattr(self, "_damping"):
            self._damping, self._damping_desc = self._frontend.create_tensor((self.count, 1), float32, -1)
        
        if not self._backend.get_damping(self._damping_desc):
            raise Exception("Failed to get particle material damping from backend")

        return self._damping

    def set_damping(self, data, indices):
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_damping(data_desc, indices_desc):
            raise Exception("Failed to set particle material damping in backend")

    def get_gravity_scale(self):
        if not hasattr(self, "_gravity_scale"):
            self._gravity_scale, self._gravity_scale_desc = self._frontend.create_tensor((self.count, 1), float32, -1)
        
        if not self._backend.get_gravity_scale(self._gravity_scale_desc):
            raise Exception("Failed to get particle system particle material gravity scale from backend")

        return self._gravity_scale

    def set_gravity_scale(self, data, indices):
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_gravity_scale(data_desc, indices_desc):
            raise Exception("Failed to set particle system particle material gravity scale in backend")

    def get_lift(self):
        if not hasattr(self, "_lift"):
            self._lift, self._lift_desc = self._frontend.create_tensor((self.count, 1), float32, -1)
        
        if not self._backend.get_lift(self._lift_desc):
            raise Exception("Failed to get particle material lift from backend")

        return self._lift

    def set_lift(self, data, indices):
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_lift(data_desc, indices_desc):
            raise Exception("Failed to set particle material lift in backend")

    def get_drag(self):
        if not hasattr(self, "_drag"):
            self._drag, self._drag_desc = self._frontend.create_tensor((self.count, 1), float32, -1)
        
        if not self._backend.get_drag(self._drag_desc):
            raise Exception("Failed to get particle material drag from backend")

        return self._drag

    def set_drag(self, data, indices):
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_drag(data_desc, indices_desc):
            raise Exception("Failed to set particle material drag in backend")

    def check(self):
        return self._backend.check()

