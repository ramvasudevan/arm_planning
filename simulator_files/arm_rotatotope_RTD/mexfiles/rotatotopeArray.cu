/*
Author: Bohao Zhang
Oct. 29 2019

arm_planning mex

a cuda array for a cluster of rotatotopes
*/

#ifndef ROTATOTOPE_ARRAY_CPP
#define ROTATOTOPE_ARRAY_CPP

#include "rotatotopeArray.h"

rotatotopeArray::rotatotopeArray(uint32_t n_links_input, uint32_t n_time_steps_input, uint32_t joint_per_link_input, double* R_input, double* dev_R_input, uint32_t R_unit_length_input, uint8_t* dev_rot_axes_input, double* Z_input, uint32_t Z_width_input, uint32_t Z_length_input, uint32_t reduce_order_input, double* g_k_input) {
	debugMode = false;
	
	n_links = n_links_input;
	n_time_steps = n_time_steps_input;
	joint_per_link = joint_per_link_input;
	dev_R = dev_R_input;
	R_unit_length = R_unit_length_input;
	dev_rot_axes = dev_rot_axes_input;
	reduce_order = reduce_order_input;

	if (n_links > 0) {
		Z = Z_input;
		Z_length = Z_length_input;
		Z_width = Z_width_input;
		Z_unit_length = Z_length / n_links;
		cudaMalloc((void**)&dev_Z, Z_width * Z_length * sizeof(double));
		cudaMemcpy(dev_Z, Z, Z_width * Z_length * sizeof(double), cudaMemcpyHostToDevice);

		c_k = new double[n_links * 2];
		g_k = new double[n_links * 2];

		for (uint32_t joint_id = 0; joint_id < n_links * 2; joint_id++) {
			uint32_t R_id_start = ((joint_id + 1) * n_time_steps - 1) * R_unit_length;
			c_k[joint_id] = R_input[R_id_start * 5 + k_dim];
			g_k[joint_id] = g_k_input[joint_id];
		}

		double* dev_RZ_new;
		cudaMalloc((void**)&dev_RZ, n_links * n_time_steps * reduce_order * Z_width * sizeof(double));
		cudaMalloc((void**)&dev_RZ_new, n_links * n_time_steps * reduce_order * R_unit_length * Z_width * sizeof(double));

		bool *dev_c_idx_new, *dev_k_idx_new;
		cudaMalloc((void**)&dev_c_idx, n_links * n_time_steps * reduce_order * sizeof(bool));
		cudaMemset(dev_c_idx, 0, n_links * n_time_steps * reduce_order * sizeof(bool));
		cudaMalloc((void**)&dev_c_idx_new, n_links * n_time_steps * reduce_order * R_unit_length * sizeof(bool));
		cudaMemset(dev_c_idx_new, 0, n_links * n_time_steps * reduce_order * R_unit_length * sizeof(bool));
		cudaMalloc((void**)&dev_k_idx, n_links * (n_links + 1) * n_time_steps * reduce_order * sizeof(bool));
		cudaMemset(dev_k_idx, 0, n_links * (n_links + 1) * n_time_steps * reduce_order * sizeof(bool));
		cudaMalloc((void**)&dev_k_idx_new, n_links * (n_links + 1) * n_time_steps * reduce_order * R_unit_length * sizeof(bool));
		cudaMemset(dev_k_idx_new, 0, n_links * (n_links + 1) * n_time_steps * reduce_order * R_unit_length * sizeof(bool));

		dim3 grid1(n_links, n_time_steps, 1);
		dim3 block1(reduce_order, Z_width, 1);
		initialize_RZ_kernel << < grid1, block1 >> > (dev_Z, Z_unit_length, reduce_order, dev_RZ, dev_c_idx);

		for (int link = n_links; link > 0; link--) {
			for (int joint_offset = joint_per_link - 1; joint_offset >= 0; joint_offset--) {
				dim3 grid2(link, n_time_steps, 1);
				dim3 block2(reduce_order, R_unit_length, 1);
				multiply_kernel << < grid2, block2 >> > (dev_rot_axes, n_links - link, joint_offset, reduce_order, dev_RZ, dev_R, dev_c_idx, dev_k_idx, dev_RZ_new, dev_c_idx_new, dev_k_idx_new);

				reduce_kernel << < grid2, (reduce_order * R_unit_length) >> > (dev_RZ_new, dev_c_idx_new, dev_k_idx_new, n_links - link, reduce_order, dev_RZ, dev_c_idx, dev_k_idx);
			}
		}

		cudaFree(dev_RZ_new);
		cudaFree(dev_c_idx_new);
		cudaFree(dev_k_idx_new);
	}
	else {
		c_k = nullptr;
		g_k = nullptr;
		Z = nullptr;
		dev_Z = nullptr;
		dev_RZ = nullptr;
		dev_c_idx = nullptr;
		dev_k_idx = nullptr;
	}

	n_pairs = 0;
	self_pairs = nullptr;
	
	dev_RZ_stack = nullptr;
	dev_c_idx_stack = nullptr;
	dev_k_idx_stack = nullptr;
	RZ_length = nullptr;

	n_obstacles = 0;
	A_con = nullptr;
	dev_A_con = nullptr;
	d_con = nullptr;
	dev_d_con = nullptr;
	delta_con = nullptr;
	dev_delta_con = nullptr;
	k_con = nullptr;
	dev_k_con = nullptr;
	k_con_num = nullptr;
	dev_k_con_num = nullptr;
	max_k_con_num = nullptr;

	A_con_self = nullptr;
	dev_A_con_self = nullptr;
	d_con_self = nullptr;
	dev_d_con_self = nullptr;
	delta_con_self = nullptr;
	dev_delta_con_self = nullptr;
	k_con_self = nullptr;
	dev_k_con_self = nullptr;
	k_con_num_self = nullptr;
	dev_k_con_num_self = nullptr;
	max_k_con_num_self = nullptr;

	current_k_opt = nullptr;
	con = nullptr;
	jaco_con = nullptr;
	hess_con = nullptr;
	con_self = nullptr;
	jaco_con_self = nullptr;
	hess_con_self = nullptr;
}

__global__ void initialize_RZ_kernel(double* link_Z, uint32_t link_Z_length, uint32_t reduce_order, double* RZ, bool* c_idx) {
	uint32_t link_id = blockIdx.x;
	uint32_t time_id = blockIdx.y;
	uint32_t n_time_steps = gridDim.y;
	uint32_t z_id = threadIdx.x;
	uint32_t w_id = threadIdx.y;
	uint32_t Z_width = blockDim.y;

	if (z_id < link_Z_length) {
		RZ[((link_id * n_time_steps + time_id) * reduce_order + z_id) * Z_width + w_id] = link_Z[(link_id * link_Z_length + z_id) * Z_width + w_id];
	}
	else {
		RZ[((link_id * n_time_steps + time_id) * reduce_order + z_id) * Z_width + w_id] = 0;
	}

	if (z_id == 0) c_idx[(link_id * n_time_steps + time_id) * reduce_order] = true;
}

__global__ void multiply_kernel(uint8_t* rot_axes, uint32_t link_offset, uint32_t joint_offset, uint32_t reduce_order, double* RZ, double* R, bool* c_idx, bool* k_idx, double* RZ_new, bool* c_idx_new, bool* k_idx_new) {
	uint32_t link_id = blockIdx.x + link_offset;
	uint32_t joint_id = blockIdx.x * 2 + joint_offset;
	uint32_t time_id = blockIdx.y;
	uint32_t n_time_steps = gridDim.y;
	uint32_t z_id = threadIdx.x;
	uint32_t r_id = threadIdx.y;
	uint32_t R_unit_length = blockDim.y;

	uint32_t mul_Z = (link_id * n_time_steps + time_id) * reduce_order + z_id;
	uint32_t mul_R = (joint_id * n_time_steps + time_id) * R_unit_length + r_id;
	uint32_t mul_RZ = ((link_id * n_time_steps + time_id) * reduce_order + z_id) * R_unit_length + r_id;

	uint8_t rot_axis = rot_axes[joint_id];
	bool if_center = (r_id == 0); // true if center, false if not

	if (rot_axis == 1) {
		RZ_new[mul_RZ * 3] = if_center ? RZ[mul_Z * 3] : 0;
		RZ_new[mul_RZ * 3 + 1] = R[mul_R * 5] * RZ[mul_Z * 3 + 1] - R[mul_R * 5 + 1] * RZ[mul_Z * 3 + 2];
		RZ_new[mul_RZ * 3 + 2] = R[mul_R * 5 + 1] * RZ[mul_Z * 3 + 1] + R[mul_R * 5] * RZ[mul_Z * 3 + 2];
	}
	else if (rot_axis == 2) {
		RZ_new[mul_RZ * 3] = R[mul_R * 5] * RZ[mul_Z * 3] + R[mul_R * 5 + 1] * RZ[mul_Z * 3 + 2];
		RZ_new[mul_RZ * 3 + 1] = if_center ? RZ[mul_Z * 3 + 1] : 0;
		RZ_new[mul_RZ * 3 + 2] = R[mul_R * 5] * RZ[mul_Z * 3 + 2] - R[mul_R * 5 + 1] * RZ[mul_Z * 3];
	}
	else {
		RZ_new[mul_RZ * 3] = R[mul_R * 5] * RZ[mul_Z * 3] - R[mul_R * 5 + 1] * RZ[mul_Z * 3 + 1];
		RZ_new[mul_RZ * 3 + 1] = R[mul_R * 5 + 1] * RZ[mul_Z * 3] + R[mul_R * 5] * RZ[mul_Z * 3 + 1];
		RZ_new[mul_RZ * 3 + 2] = if_center ? RZ[mul_Z * 3 + 2] : 0;
	}

	c_idx_new[mul_RZ] = c_idx[mul_Z];

	// update k for this joint
	uint32_t k_id = link_id * (link_id + 1) + joint_id;
	uint32_t mul_k = (k_id * n_time_steps + time_id) * reduce_order * R_unit_length + (z_id * R_unit_length + r_id);
	if (R[mul_R * 5 + k_dim] != 0) {
		k_idx_new[mul_k] = true;
	}
	else {
		k_idx_new[mul_k] = false;
	}

	// update k for previous joints
	for (uint32_t joint_k_id = joint_id + 1; joint_k_id < (link_id + 1) * 2; joint_k_id++) {
		k_id = link_id * (link_id + 1) + joint_k_id;
		uint32_t mul_z = (k_id * n_time_steps + time_id) * reduce_order + z_id;
		mul_k = (k_id * n_time_steps + time_id) * reduce_order * R_unit_length + (z_id * R_unit_length + r_id);
		k_idx_new[mul_k] = k_idx[mul_z];
	}
}

__global__ void reduce_kernel(double* RZ_new, bool* c_idx_new, bool* k_idx_new, uint32_t link_offset, uint32_t reduce_order, double* RZ, bool* c_idx, bool* k_idx) {
	uint32_t link_id = blockIdx.x + link_offset;
	uint32_t time_id = blockIdx.y;
	uint32_t n_time_steps = gridDim.y;
	uint32_t z_id = threadIdx.x;
	uint32_t norm_length = blockDim.x;
	uint32_t mul_Z = (link_id * n_time_steps + time_id) * norm_length + z_id; // we never reduce the center
	__shared__ double RZ_norm[MAX_NORM_SIZE];
	__shared__ uint32_t RZ_id[MAX_NORM_SIZE];

	RZ_norm[z_id] = 0;
	double norm;
	for (uint32_t i = 0; i < 3; i++) {
		norm = RZ_new[mul_Z * 3 + i];
		RZ_norm[z_id] += norm * norm;
	}

	RZ_id[z_id] = z_id;

	__syncthreads();

	uint32_t base = (link_id * n_time_steps + time_id) * norm_length; // indeces offset for RZ_new
	uint32_t k_start = ((link_id * (link_id + 1)) * n_time_steps + time_id) * norm_length;
	uint32_t k_end = (((link_id + 1) * (link_id + 2)) * n_time_steps + time_id) * norm_length;
	uint32_t k_step = n_time_steps * norm_length;

	if (z_id == 0) {
		// choose the vectors whose norm is among (reduce_order - 3) largest
		uint32_t high = norm_length;
		uint32_t low = 1;
		uint32_t k = reduce_order - 3;
		uint32_t i, j;
		while (low < high) {
			i = low;
			j = high - 1;
			double pivot = RZ_norm[low];
			while (i <= j) {
				while (i <= j && RZ_norm[i] >= pivot)
					i++;
				while (i <= j && RZ_norm[j] < pivot)
					j--;
				if (i < j) {
					double temp_double = RZ_norm[i];
					RZ_norm[i] = RZ_norm[j];
					RZ_norm[j] = temp_double;
					uint32_t temp = RZ_id[i];
					RZ_id[i] = RZ_id[j];
					RZ_id[j] = temp;
					i++;
					j--;
				}
			}
			double temp_double = RZ_norm[low];
			RZ_norm[low] = RZ_norm[j];
			RZ_norm[j] = temp_double;
			uint32_t temp = RZ_id[low];
			RZ_id[low] = RZ_id[j];
			RZ_id[j] = temp;

			if (j == k - 1)
				break;
			else if (j < k - 1)
				low = j + 1;
			else
				high = j;
		}
	}

	__syncthreads();

	// at this point, the first (reduce_order - 3) entries in RZ_new are the (reduce_order - 3) largest ones
	// we choose them as entries for RZ after reduction.
	// we compress the rest of the entries to a box with 3 generators

	uint32_t base_ori = (link_id * n_time_steps + time_id) * reduce_order; // indeces offset for RZ
	uint32_t k_start_ori = ((link_id * (link_id + 1)) * n_time_steps + time_id) * reduce_order;
	uint32_t k_end_ori = (((link_id + 1) * (link_id + 2)) * n_time_steps + time_id) * reduce_order;
	uint32_t k_step_ori = n_time_steps * reduce_order;

	if (z_id < reduce_order - 3) { // copy these generators to RZ
		uint32_t sorted_id = RZ_id[z_id];
		c_idx[base_ori + z_id] = c_idx_new[base + sorted_id];

		for (uint32_t h = 0; h < 3; h++) {
			RZ[(base_ori + z_id) * 3 + h] = RZ_new[(base + sorted_id) * 3 + h];
		}

		uint32_t k_pivot = k_start, k_pivot_ori = k_start_ori;
		while (k_pivot != k_end && k_pivot_ori != k_end_ori) {
			k_idx[k_pivot_ori + z_id] = k_idx_new[k_pivot + sorted_id];
			k_pivot += k_step;
			k_pivot_ori += k_step_ori;
		}
	}
	else if (reduce_order - 3 <= z_id && z_id < reduce_order) { // construct a 3-d box for the rest of the generators
		uint32_t box_id = (z_id + 3) - reduce_order;
		double entry_sum = 0;
		for (uint32_t h = reduce_order - 3; h < norm_length; h++) {
			uint32_t sorted_id = RZ_id[h];
			entry_sum += abs(RZ_new[(base + sorted_id) * 3 + box_id]);
		}

		for (uint32_t h = 0; h < 3; h++) {
			if (h == box_id) {
				RZ[(base_ori + z_id) * 3 + h] = entry_sum;
			}
			else {
				RZ[(base_ori + z_id) * 3 + h] = 0;
			}
		}

		c_idx[base_ori + z_id] = false;
		for (uint32_t h = k_start_ori; h < k_end_ori; h += k_step_ori) {
			k_idx[h + z_id] = false;
		}
	}
}

void rotatotopeArray::stack(rotatotopeArray &EEs, rotatotopeArray &base) {
	RZ_stack = new double*[n_links];
	dev_RZ_stack = new double*[n_links];
	c_idx_stack = new bool*[n_links];
	dev_c_idx_stack = new bool*[n_links];
	k_idx_stack = new bool*[n_links];
	dev_k_idx_stack = new bool*[n_links];
	RZ_length = new uint32_t[n_links];

	for (uint32_t link_id = 0; link_id < n_links; link_id++) {
		RZ_length[link_id] = reduce_order + link_id * (EEs.reduce_order - 1) + base.reduce_order - 1;

		RZ_stack[link_id] = nullptr;
		cudaMalloc((void**)&(dev_RZ_stack[link_id]), n_time_steps * RZ_length[link_id] * Z_width * sizeof(double));

		c_idx_stack[link_id] = nullptr;
		cudaMalloc((void**)&(dev_c_idx_stack[link_id]), n_time_steps * RZ_length[link_id] * sizeof(bool));

		k_idx_stack[link_id] = nullptr;
		cudaMalloc((void**)&(dev_k_idx_stack[link_id]), 2 * (link_id + 1) * n_time_steps * RZ_length[link_id] * sizeof(bool));
		cudaMemset(dev_k_idx_stack, 0, 2 * (link_id + 1) * n_time_steps * RZ_length[link_id] * sizeof(bool));

		// copy dev_RZ to dev_RZ_stack
		dim3 grid1(n_time_steps, 1, 1);
		dim3 block1(reduce_order, Z_width, 1);
		copy_kernel << < grid1, block1 >> > (link_id, dev_RZ, dev_c_idx, dev_k_idx, reduce_order, EEs.reduce_order, dev_RZ_stack[link_id], dev_c_idx_stack[link_id], dev_k_idx_stack[link_id]);

		// stack with EE
		for (int EE_id = link_id - 1; EE_id >= 0; EE_id--) {
			dim3 grid2(n_time_steps, 1, 1);
			dim3 block2(EEs.reduce_order, Z_width, 1);
			stack_kernel << < grid2, block2 >> > (link_id, EE_id, EE_id, reduce_order, EEs.reduce_order, dev_RZ_stack[link_id], EEs.dev_RZ, dev_c_idx_stack[link_id], EEs.dev_c_idx, dev_k_idx_stack[link_id], EEs.dev_k_idx);
		}

		// stack with base
		dim3 grid3(n_time_steps, 1, 1);
		dim3 block3(base.reduce_order, Z_width, 1);
		stack_kernel << < grid3, block3 >> > (link_id, 0, link_id, reduce_order, base.reduce_order, dev_RZ_stack[link_id], base.dev_RZ, dev_c_idx_stack[link_id], base.dev_c_idx, dev_k_idx_stack[link_id], base.dev_k_idx);
		
		// origin shift
		origin_shift_kernel <<< n_time_steps, 1 >>> (RZ_length[link_id], dev_RZ_stack[link_id]);
	}

	uint32_t link_id = 0;
	if(debugMode){
		debug_RZ = new double[n_time_steps * RZ_length[link_id] * Z_width];
		cudaMemcpy(debug_RZ, dev_RZ_stack[link_id], n_time_steps * RZ_length[link_id] * Z_width * sizeof(double), cudaMemcpyDeviceToHost);

		debug_c_idx = new bool[n_time_steps * RZ_length[link_id]];
		cudaMemcpy(debug_c_idx, dev_c_idx_stack[link_id], n_time_steps * RZ_length[link_id] * sizeof(bool), cudaMemcpyDeviceToHost);

		debug_k_idx = new bool[2 * (link_id + 1) * n_time_steps * RZ_length[link_id]];
		cudaMemcpy(debug_k_idx, dev_k_idx_stack[link_id], 2 * (link_id + 1) * n_time_steps * RZ_length[link_id] * sizeof(bool), cudaMemcpyDeviceToHost);
	}
	else{
		debug_RZ = nullptr;
		debug_c_idx = nullptr;
		debug_k_idx = nullptr;
	}
}

__global__ void copy_kernel(uint32_t link_id, double* RZ, bool* c_idx, bool* k_idx, uint32_t link_reduce_order, uint32_t point_reduce_order, double* RZ_stack, bool* c_idx_stack, bool* k_idx_stack) {
	uint32_t time_id = blockIdx.x;
	uint32_t n_time_steps = gridDim.x;
	uint32_t Z_id = threadIdx.x;
	uint32_t z_id = threadIdx.y;
	
	uint32_t RZ_length = link_reduce_order + (link_id + 1) * (point_reduce_order - 1);
	uint32_t copy_Z = time_id * RZ_length + Z_id;
	uint32_t copy_k_start = time_id * RZ_length + Z_id;
	uint32_t copy_k_step = n_time_steps * RZ_length;
	uint32_t link_Z = (link_id * n_time_steps + time_id) * link_reduce_order + Z_id;
	uint32_t link_k_start = ((link_id * (link_id + 1)) * n_time_steps + time_id) * link_reduce_order + Z_id;
	uint32_t link_k_end = (((link_id + 1) * (link_id + 2)) * n_time_steps + time_id) * link_reduce_order + Z_id;
	uint32_t link_k_step = n_time_steps * link_reduce_order;

	RZ_stack[copy_Z * 3 + z_id] = RZ[link_Z * 3 + z_id];

	if (z_id == 0) {
		c_idx_stack[copy_Z] = c_idx[link_Z];

		uint32_t copy_k = copy_k_start;
		for (uint32_t link_k = link_k_start; link_k < link_k_end; link_k += link_k_step) {
			k_idx_stack[copy_k] = k_idx[link_k];
			copy_k += copy_k_step;
		}
	}
}

__global__ void stack_kernel(uint32_t link_id, uint32_t EE_id, uint32_t stack_offset, uint32_t link_reduce_order, uint32_t point_reduce_order, double* RZ_stack, double* EE_RZ, bool* c_idx_stack, bool* EE_c_idx, bool* k_idx_stack, bool* EE_k_idx) {
	uint32_t time_id = blockIdx.x;
	uint32_t n_time_steps = gridDim.x;
	uint32_t Z_id = threadIdx.x;
	uint32_t z_id = threadIdx.y;

	uint32_t RZ_length = link_reduce_order + (link_id + 1) * (point_reduce_order - 1);
	uint32_t stack_Z = time_id * RZ_length + Z_id;
	uint32_t stack_k_start = time_id * RZ_length + Z_id;
	uint32_t stack_k_end = (2 * (link_id + 1) * n_time_steps + time_id) * RZ_length + Z_id;
	uint32_t stack_k_step = n_time_steps * RZ_length;
	uint32_t EE_Z = (EE_id * n_time_steps + time_id) * point_reduce_order + Z_id;
	uint32_t EE_k_start = ((EE_id * (EE_id + 1)) * n_time_steps + time_id) * point_reduce_order + Z_id;
	uint32_t EE_k_end = (((EE_id + 1) * (EE_id + 2)) * n_time_steps + time_id) * point_reduce_order + Z_id;
	uint32_t EE_k_step = n_time_steps * point_reduce_order;

	if (Z_id == 0) { // add the center
		RZ_stack[stack_Z * 3 + z_id] += EE_RZ[EE_Z * 3 + z_id];

		if (z_id == 0) {
			c_idx_stack[stack_Z] = true;

			uint32_t EE_k = EE_k_start;
			for (uint32_t stack_k = stack_k_start; stack_k < stack_k_end; stack_k += stack_k_step) {
				if (EE_k < EE_k_end) {
					k_idx_stack[stack_k] |= EE_k_idx[EE_k];
				}
				else {
					break;
				}
				
				EE_k += EE_k_step;
			}
		}
	}
	else { // stack the generators
		uint32_t stack_offset_length = link_reduce_order - 1 + stack_offset * (point_reduce_order - 1);
		RZ_stack[(stack_Z + stack_offset_length) * 3 + z_id] = EE_RZ[EE_Z * 3 + z_id];

		if (z_id == 0) {
			c_idx_stack[(stack_Z + stack_offset_length)] = EE_c_idx[EE_Z];

			uint32_t EE_k = EE_k_start;
			for (uint32_t stack_k = stack_k_start + stack_offset_length; stack_k < stack_k_end + stack_offset_length; stack_k += stack_k_step) {
				if (EE_k < EE_k_end) {
					k_idx_stack[stack_k] = EE_k_idx[EE_k];
				}
				else {
					k_idx_stack[stack_k] = false;
				}

				EE_k += EE_k_step;
			}
		}
	}
}

__global__ void origin_shift_kernel(uint32_t RZ_length, double* RZ_stack){
	uint32_t time_id = blockIdx.x;
	uint32_t stack_Z = time_id * RZ_length;

	RZ_stack[stack_Z * 3    ] += ORIGIN_SHIFT_X;
	RZ_stack[stack_Z * 3 + 1] += ORIGIN_SHIFT_Y;
	RZ_stack[stack_Z * 3 + 2] += ORIGIN_SHIFT_Z;
}

void rotatotopeArray::generate_constraints(uint32_t n_obstacles_in, double* OZ, uint32_t OZ_width, uint32_t OZ_length) {
	// obstacle constraints
	n_obstacles = n_obstacles_in;
	uint32_t OZ_unit_length = OZ_length / n_obstacles;

	double* dev_OZ;
	cudaMalloc((void**)&dev_OZ, OZ_length * OZ_width * sizeof(double));
	cudaMemcpy(dev_OZ, OZ, OZ_length * OZ_width * sizeof(double), cudaMemcpyHostToDevice);

	A_con = new double*[n_links];
	dev_A_con = new double*[n_links];
	d_con = new double*[n_links];
	dev_d_con = new double*[n_links];
	delta_con = new double*[n_links];
	dev_delta_con = new double*[n_links];
	k_con = new bool*[n_links];
	dev_k_con = new bool*[n_links];
	k_con_num = new uint8_t*[n_links];
	dev_k_con_num = new uint8_t*[n_links];
	max_k_con_num = new uint32_t[n_links];

	for (uint32_t link_id = 0; link_id < n_links; link_id++) {
		uint32_t buff_obstacle_length = RZ_length[link_id] + 3;
		uint32_t constraint_length = ((buff_obstacle_length - 1) * (buff_obstacle_length - 2)) / 2;

		// buffer the obstacle by k-independent generators
		k_con[link_id] = new bool[2 * (link_id + 1) * n_time_steps * RZ_length[link_id]];
		cudaMalloc((void**)&(dev_k_con[link_id]), 2 * (link_id + 1) * n_time_steps * RZ_length[link_id] * sizeof(bool));
		k_con_num[link_id] = new uint8_t[n_time_steps];
		cudaMalloc((void**)&(dev_k_con_num[link_id]), n_time_steps * sizeof(uint8_t));

		double* dev_buff_obstacles;
		cudaMalloc((void**)&dev_buff_obstacles, n_obstacles * n_time_steps * buff_obstacle_length * 3 * sizeof(double));
		cudaMemset(dev_buff_obstacles, 0, n_obstacles * n_time_steps * buff_obstacle_length * 3 * sizeof(double));

		double* dev_frs_k_dep_G;
		cudaMalloc((void**)&dev_frs_k_dep_G, n_time_steps * RZ_length[link_id] * 3 * sizeof(double));
		cudaMemset(dev_frs_k_dep_G, 0, n_time_steps * RZ_length[link_id] * 3 * sizeof(double));

		dim3 grid1(n_obstacles, n_time_steps, 1);
		buff_obstacles_kernel << < grid1, RZ_length[link_id] >> > (link_id, RZ_length[link_id], dev_RZ_stack[link_id], dev_c_idx_stack[link_id], dev_k_idx_stack[link_id], dev_OZ, OZ_unit_length, dev_buff_obstacles, dev_frs_k_dep_G, dev_k_con[link_id], dev_k_con_num[link_id]);

		if(debugMode){
			cudaMemcpy(k_con[link_id], dev_k_con[link_id], 2 * (link_id + 1) * n_time_steps * RZ_length[link_id] * sizeof(bool), cudaMemcpyDeviceToHost);
		}
		cudaMemcpy(k_con_num[link_id], dev_k_con_num[link_id], n_time_steps * sizeof(uint8_t), cudaMemcpyDeviceToHost);

		// find the maximum width of A_con for memory allocation
		max_k_con_num[link_id] = 0;
		for (uint32_t i = 0; i < n_time_steps; i++) {
			if (k_con_num[link_id][i] > max_k_con_num[link_id]) {
				max_k_con_num[link_id] = k_con_num[link_id][i];
			}
		}

		// generate obstacles polynomials
		cudaMalloc((void**)&(dev_A_con[link_id]), n_obstacles * n_time_steps * constraint_length * max_k_con_num[link_id] * sizeof(double));
		cudaMalloc((void**)&(dev_d_con[link_id]), n_obstacles * n_time_steps * constraint_length * sizeof(double));
		cudaMalloc((void**)&(dev_delta_con[link_id]), n_obstacles * n_time_steps * constraint_length * sizeof(double));
		
		dim3 grid2(n_obstacles, n_time_steps, 1);
		polytope << < grid2, constraint_length >> > (buff_obstacle_length, RZ_length[link_id], dev_buff_obstacles, dev_frs_k_dep_G, dev_k_con_num[link_id], max_k_con_num[link_id], dev_A_con[link_id], dev_d_con[link_id], dev_delta_con[link_id]);

		if(debugMode){
			A_con[link_id] = new double[n_obstacles * n_time_steps * constraint_length * max_k_con_num[link_id]];
			cudaMemcpy(A_con[link_id], dev_A_con[link_id], n_obstacles * n_time_steps * constraint_length * max_k_con_num[link_id] * sizeof(double), cudaMemcpyDeviceToHost);
			
			d_con[link_id] = new double[n_obstacles * n_time_steps * constraint_length];
			cudaMemcpy(d_con[link_id], dev_d_con[link_id], n_obstacles * n_time_steps * constraint_length * sizeof(double), cudaMemcpyDeviceToHost);
			
			delta_con[link_id] = new double[n_obstacles * n_time_steps * constraint_length];
			cudaMemcpy(delta_con[link_id], dev_delta_con[link_id], n_obstacles * n_time_steps * constraint_length * sizeof(double), cudaMemcpyDeviceToHost);
		}
		else{
			A_con[link_id] = nullptr;
			d_con[link_id] = nullptr;
			delta_con[link_id] = nullptr;
		}

		cudaFree(dev_buff_obstacles);
		cudaFree(dev_frs_k_dep_G);
	}

	cudaFree(dev_OZ);
}

__global__ void buff_obstacles_kernel(uint32_t link_id, uint32_t RZ_length, double* RZ, bool* c_idx, bool* k_idx, double* OZ, uint32_t OZ_unit_length, double* buff_obstacles, double* frs_k_dep_G, bool* k_con, uint8_t* k_con_num) {
	uint32_t obstacle_id = blockIdx.x;
	uint32_t obstacle_base = obstacle_id * OZ_unit_length;
	uint32_t time_id = blockIdx.y;
	uint32_t n_time_steps = gridDim.y;
	uint32_t z_id = threadIdx.x; 
	uint32_t buff_obstacle_length = RZ_length + 3;
	uint32_t RZ_base = time_id * RZ_length;
	uint32_t k_start = time_id * RZ_length;
	uint32_t k_end = (2 * (link_id + 1) * n_time_steps + time_id) * RZ_length;
	uint32_t k_step = n_time_steps * RZ_length;
	uint32_t k_con_num_base = time_id;
	uint32_t buff_base = (obstacle_id * n_time_steps + time_id) * buff_obstacle_length;

	// first, find kc_col
	__shared__ bool kc_info[MAX_RZ_LENGTH];

	kc_info[z_id] = false;
	for (uint32_t i = k_start; i < k_end; i += k_step) {
		if (k_idx[i + z_id] == true) {
			kc_info[z_id] = true;
			break;
		}
	}

	kc_info[z_id] &= c_idx[RZ_base + z_id];

	__syncthreads();

	if (z_id == 0) { // process the original obstacle zonotope
		for (uint32_t i = 0; i < 3; i++) {
			buff_obstacles[buff_base * 3 + i] = OZ[obstacle_base * 3 + i] - RZ[RZ_base * 3 + i];
		}

		for (uint32_t obs_g = 1; obs_g < OZ_unit_length; obs_g++) {
			for (uint32_t i = 0; i < 3; i++) {
				buff_obstacles[(buff_base + obs_g) * 3 + i] = OZ[(obstacle_base + obs_g) * 3 + i];

				// buffer the obstacle, suppose the generators are an eye matrix
				if(i == obs_g - 1) buff_obstacles[(buff_base + obs_g) * 3 + i] += BUFFER_DIST / 2.0;
			}
		}
	}
	else if (z_id == 1) { // find k-dependent generators and complete k_con
		if (obstacle_id == 0) {
			uint8_t k_dep_num = 0;
			for (uint32_t z = 1; z < RZ_length; z++) {
				if (kc_info[z]) {
					for (uint32_t j = k_start; j < k_end; j += k_step) {
						k_con[j + k_dep_num] = k_idx[j + z];
					}

					for (uint32_t i = 0; i < 3; i++) {
						frs_k_dep_G[(RZ_base + k_dep_num) * 3 + i] = RZ[(RZ_base + z) * 3 + i];
					}

					k_dep_num++;
				}
			}

			k_con_num[k_con_num_base] = k_dep_num;
		}
	}
	else if (z_id == 2) { // find k-independent generators and complete buff_obstacles
		uint8_t k_indep_num = OZ_unit_length;
		// add a test here, reduce small generators to be a box
		double reduced_generators[3];
		reduced_generators[0] = 0;
		reduced_generators[1] = 0;
		reduced_generators[2] = 0;

		for (uint32_t z = 1; z < RZ_length; z++) {
			if (!kc_info[z]) {
				double norm = 0;
				for (uint32_t i = 0; i < 3; i++) {
					norm += RZ[(RZ_base + z) * 3 + i] * RZ[(RZ_base + z) * 3 + i];
				}

				if(norm >= TOO_SMALL_POLYTOPE_JUDGE){
					for (uint32_t i = 0; i < 3; i++) {
						buff_obstacles[(buff_base + k_indep_num) * 3 + i] = RZ[(RZ_base + z) * 3 + i];
					}
					k_indep_num++;
				}
				else{
					for (uint32_t i = 0; i < 3; i++) {
						reduced_generators[i] += RZ[(RZ_base + z) * 3 + i];
					}
				}
			}
		}

		for (uint32_t i = 0; i < 3; i++) {
			for (uint32_t j = 0; j < 3; j++){
				if(i == j){
					buff_obstacles[(buff_base + k_indep_num) * 3 + j] = reduced_generators[i];
				}
				else{
					buff_obstacles[(buff_base + k_indep_num) * 3 + j] = 0;
				}
			}
			k_indep_num++;
		}
	}
}

__global__ void polytope(uint32_t buff_obstacle_length, uint32_t k_dep_G_length, double* buff_obstacles, double* frs_k_dep_G, uint8_t* k_con_num, uint32_t A_con_width, double* A_con, double* d_con, double* delta_con) {
	uint32_t obstacle_id = blockIdx.x;
	uint32_t time_id = blockIdx.y;
	uint32_t n_time_steps = gridDim.y;
	double buff_obstacle_size = (double)buff_obstacle_length - 1.0;
	uint32_t constraint_length = (buff_obstacle_length - 1) * (buff_obstacle_length - 2) / 2;
	uint32_t k_con_base = time_id;
	uint32_t k_dep_G_base = k_con_base * k_dep_G_length;
	uint32_t obs_base = (obstacle_id * n_time_steps + time_id) * buff_obstacle_length;
	uint32_t c_id = threadIdx.x;
	uint32_t first = (uint32_t)floor(-0.5*sqrt(4 * buff_obstacle_size * buff_obstacle_size - 4 * buff_obstacle_size - 8.0 * ((double)c_id) + 1.0) + buff_obstacle_size - 0.5);
	uint32_t first_base = (obs_base + first + 1) * 3;
	uint32_t second = c_id + 1 - ((2 * (buff_obstacle_length - 1) - 3 - first) * first) / 2;
	uint32_t second_base = (obs_base + second + 1) * 3;
	uint32_t con_base = (obstacle_id * n_time_steps + time_id) * constraint_length + c_id;

	double A_1 = buff_obstacles[first_base + 1] * buff_obstacles[second_base + 2] - buff_obstacles[first_base + 2] * buff_obstacles[second_base + 1];
	double A_2 = buff_obstacles[first_base + 2] * buff_obstacles[second_base] - buff_obstacles[first_base] * buff_obstacles[second_base + 2];
	double A_3 = buff_obstacles[first_base] * buff_obstacles[second_base + 1] - buff_obstacles[first_base + 1] * buff_obstacles[second_base];
	
	double A_s_q = sqrt(A_1 * A_1 + A_2 * A_2 + A_3 * A_3);
	if(A_s_q > 0){
		A_1 /= A_s_q;
		A_2 /= A_s_q;
		A_3 /= A_s_q;
	}
	else{
		A_1 = 0;
		A_2 = 0;
		A_3 = 0;
	}
	 
	for (uint32_t i = 0; i < k_con_num[k_con_base]; i++) {
		A_con[con_base * A_con_width + i] = A_1 * frs_k_dep_G[(k_dep_G_base + i) * 3] + A_2 * frs_k_dep_G[(k_dep_G_base + i) * 3 + 1] + A_3 * frs_k_dep_G[(k_dep_G_base + i) * 3 + 2];
	}

	double d = A_1 * buff_obstacles[obs_base * 3] + A_2 * buff_obstacles[obs_base * 3 + 1] + A_3 * buff_obstacles[obs_base * 3 + 2];

	double deltaD = 0;
	for (uint32_t i = 1; i < buff_obstacle_length - k_con_num[k_con_base]; i++) {
		deltaD += abs(A_1 * buff_obstacles[(obs_base + i) * 3] + A_2 * buff_obstacles[(obs_base + i) * 3 + 1] + A_3 * buff_obstacles[(obs_base + i) * 3 + 2]);
	}

	if (A_s_q > 0) {
		d_con[con_base] = d;
		delta_con[con_base] = deltaD;
	}
	else {
		d_con[con_base] = 0;
		delta_con[con_base] = A_BIG_NUMBER;
	}
}

void rotatotopeArray::generate_self_constraints(uint32_t n_pairs_input, uint32_t* self_pairs_input){
	n_pairs = n_pairs_input;
	self_pairs = self_pairs_input;
	
	A_con_self = new double*[n_pairs];
	dev_A_con_self = new double*[n_pairs];
	d_con_self = new double*[n_pairs];
	dev_d_con_self = new double*[n_pairs];
	delta_con_self = new double*[n_pairs];
	dev_delta_con_self = new double*[n_pairs];
	k_con_self = new bool*[n_pairs];
	dev_k_con_self = new bool*[n_pairs];
	k_con_num_self = new uint8_t*[n_pairs];
	dev_k_con_num_self = new uint8_t*[n_pairs];
	max_k_con_num_self = new uint32_t[n_pairs];

	for(uint32_t pair_id = 0; pair_id < n_pairs; pair_id++){
		uint32_t R1 = self_pairs[pair_id * 2];
		uint32_t R2 = self_pairs[pair_id * 2 + 1];
		uint32_t R1_length = RZ_length[R1];
		uint32_t R2_length = RZ_length[R2];

		uint32_t gen_zono_length = R2_length;
		uint32_t constraint_length = ((gen_zono_length - 1) * (gen_zono_length - 2)) / 2;
		uint32_t k_dep_length = R1_length;

		// buffer the obstacle by k-independent generators
		k_con_self[pair_id] = new bool[2 * (R2 + 1) * n_time_steps * k_dep_length];
		cudaMalloc((void**)&(dev_k_con_self[pair_id]), 2 * (R2 + 1) * n_time_steps * k_dep_length * sizeof(bool));
		cudaMemset(dev_k_con_self[pair_id], 0, 2 * (R2 + 1) * n_time_steps * k_dep_length * sizeof(bool));
		k_con_num_self[pair_id] = new uint8_t[n_time_steps];
		cudaMalloc((void**)&(dev_k_con_num_self[pair_id]), n_time_steps * sizeof(uint8_t));

		double* dev_gen_zono;
		cudaMalloc((void**)&dev_gen_zono, n_time_steps * gen_zono_length * Z_width * sizeof(double));
		cudaMemset(dev_gen_zono, 0, n_time_steps * gen_zono_length * Z_width * sizeof(double));

		double* dev_k_dep_pt;
		cudaMalloc((void**)&dev_k_dep_pt, n_time_steps * k_dep_length * Z_width * sizeof(double));
		cudaMemset(dev_k_dep_pt, 0, n_time_steps * k_dep_length * Z_width * sizeof(double));

		gen_zono_kernel << < n_time_steps, R2_length >> > (R1, R2, R1_length, R2_length, dev_RZ_stack[R1], dev_c_idx_stack[R1], dev_k_idx_stack[R1], dev_RZ_stack[R2], dev_c_idx_stack[R2], dev_k_idx_stack[R2], dev_gen_zono, dev_k_dep_pt, dev_k_con_self[pair_id], dev_k_con_num_self[pair_id]);

		if(debugMode){
			cudaMemcpy(k_con_self[pair_id], dev_k_con_self[pair_id], 2 * (R2 + 1) * n_time_steps * k_dep_length * sizeof(bool), cudaMemcpyDeviceToHost);
		}
		cudaMemcpy(k_con_num_self[pair_id], dev_k_con_num_self[pair_id], n_time_steps * sizeof(uint8_t), cudaMemcpyDeviceToHost);

		// find the maximum width of A_con for memory allocation
		max_k_con_num_self[pair_id] = 0;
		for (uint32_t i = 0; i < n_time_steps; i++) {
			if (k_con_num_self[pair_id][i] > max_k_con_num_self[pair_id]) {
				max_k_con_num_self[pair_id] = k_con_num_self[pair_id][i];
			}
		}

		if(max_k_con_num_self[pair_id] > k_dep_length){
			mexErrMsgIdAndTxt("MyProg:ConvertString","*** Incorrect size of k dep in self intersection");
		}

		// generate obstacles polynomials
		cudaMalloc((void**)&(dev_A_con_self[pair_id]), n_time_steps * constraint_length * max_k_con_num_self[pair_id] * sizeof(double));
		cudaMalloc((void**)&(dev_d_con_self[pair_id]), n_time_steps * constraint_length * sizeof(double));
		cudaMalloc((void**)&(dev_delta_con_self[pair_id]), n_time_steps * constraint_length * sizeof(double));
		
		dim3 grid2(1, n_time_steps, 1);
		polytope << < grid2, constraint_length >> > (gen_zono_length, k_dep_length, dev_gen_zono, dev_k_dep_pt, dev_k_con_num_self[pair_id], max_k_con_num_self[pair_id], dev_A_con_self[pair_id], dev_d_con_self[pair_id], dev_delta_con_self[pair_id]);

		if(debugMode){
			A_con_self[pair_id] = new double[n_time_steps * constraint_length * max_k_con_num_self[pair_id]];
			cudaMemcpy(A_con_self[pair_id], dev_A_con_self[pair_id], n_time_steps * constraint_length * max_k_con_num_self[pair_id] * sizeof(double), cudaMemcpyDeviceToHost);
			
			d_con_self[pair_id] = new double[n_time_steps * constraint_length];
			cudaMemcpy(d_con_self[pair_id], dev_d_con_self[pair_id], n_time_steps * constraint_length * sizeof(double), cudaMemcpyDeviceToHost);
			
			delta_con_self[pair_id] = new double[n_time_steps * constraint_length];
			cudaMemcpy(delta_con_self[pair_id], dev_delta_con_self[pair_id], n_time_steps * constraint_length * sizeof(double), cudaMemcpyDeviceToHost);
		}
		else{
			A_con_self[pair_id] = nullptr;
			d_con_self[pair_id] = nullptr;
			delta_con_self[pair_id] = nullptr;
		}

		cudaFree(dev_gen_zono);
		cudaFree(dev_k_dep_pt);
	}
}

__global__ void gen_zono_kernel(uint32_t link_id_1, uint32_t link_id_2, uint32_t RZ_length_1, uint32_t RZ_length_2, double* RZ_1, bool* c_idx_1, bool* k_idx_1, double* RZ_2, bool* c_idx_2, bool* k_idx_2, double* gen_zono, double* k_dep_pt, bool* k_con_self, uint8_t* k_con_num_self) {
	uint32_t time_id = blockIdx.x;
	uint32_t n_time_steps = gridDim.x;
	uint32_t z_id = threadIdx.x; 
	uint32_t gen_zono_length = RZ_length_2;
	uint32_t gen_zono_base = time_id * gen_zono_length;
	uint32_t k_con_num_base = time_id;
	uint32_t RZ_base_1 = time_id * RZ_length_1;
	uint32_t k_start_1 = time_id * RZ_length_1;
	uint32_t k_end_1 = (2 * (link_id_1 + 1) * n_time_steps + time_id) * RZ_length_1;
	uint32_t k_step_1 = n_time_steps * RZ_length_1;
	uint32_t RZ_base_2 = time_id * RZ_length_2;
	uint32_t k_start_2 = time_id * RZ_length_2;
	uint32_t k_end_2 = (2 * (link_id_2 + 1) * n_time_steps + time_id) * RZ_length_2;
	uint32_t k_step_2 = n_time_steps * RZ_length_2;
	
	// first, find kc_col for both links in a pair
	__shared__ bool kc_info_1[MAX_RZ_LENGTH];
	__shared__ bool kc_info_2[MAX_RZ_LENGTH];

	if(z_id < RZ_length_1){
		kc_info_1[z_id] = false;
		for (uint32_t i = k_start_1; i < k_end_1; i += k_step_1) {
			if (k_idx_1[i + z_id] == true) {
				kc_info_1[z_id] = true;
				break;
			}
		}
		kc_info_1[z_id] &= c_idx_1[RZ_base_1 + z_id];
	}

	kc_info_2[z_id] = false;
	for (uint32_t i = k_start_2; i < k_end_2; i += k_step_2) {
		if (k_idx_2[i + z_id] == true) {
			kc_info_2[z_id] = true;
			break;
		}
	}
	kc_info_2[z_id] &= c_idx_2[RZ_base_2 + z_id];

	__syncthreads();

	if (z_id == 0) { // process the center
		for (uint32_t i = 0; i < 3; i++) {
			gen_zono[gen_zono_base * 3 + i] = RZ_1[RZ_base_1 * 3 + i] - RZ_2[RZ_base_2 * 3 + i];
		}
	}
	else if (z_id == 1) { // find k-dependent generators and complete k_con
		uint8_t k_dep_num = 0;
		for (uint32_t z = 1; z < RZ_length_1; z++) {
			if (kc_info_1[z]) {
				for (uint32_t j = k_start_1; j < k_end_1; j += k_step_1) {
					k_con_self[j + k_dep_num] = k_idx_1[j + z];
				}

				for (uint32_t i = 0; i < 3; i++) {
					k_dep_pt[(RZ_base_1 + k_dep_num) * 3 + i] = -RZ_1[(RZ_base_1 + z) * 3 + i];
				}

				k_dep_num++;
			}
		}
		for (uint32_t z = 1; z < RZ_length_2; z++) {
			if (kc_info_2[z]) {
				uint32_t kj = k_start_1;
				for (uint32_t j = k_start_2; j < k_end_2; j += k_step_2) {
					k_con_self[kj + k_dep_num] = k_idx_2[j + z];
					kj += k_step_1;
				}

				for (uint32_t i = 0; i < 3; i++) {
					k_dep_pt[(RZ_base_1 + k_dep_num) * 3 + i] = RZ_2[(RZ_base_2 + z) * 3 + i];
				}

				k_dep_num++;
			}
		}

		k_con_num_self[k_con_num_base] = k_dep_num;
	}
	else if (z_id == 2) { // find k-independent generators and complete gen_zono
		uint8_t k_indep_num = 1;
		// add a test here, reduce small generators to be a box
		double reduced_generators[3];
		reduced_generators[0] = 0;
		reduced_generators[1] = 0;
		reduced_generators[2] = 0;

		for (uint32_t z = 1; z < RZ_length_1; z++) {
			if (!kc_info_1[z]) {
				double norm = 0;
				for (uint32_t i = 0; i < 3; i++) {
					norm += RZ_1[(RZ_base_1 + z) * 3 + i] * RZ_1[(RZ_base_1 + z) * 3 + i];
				}

				if(norm >= TOO_SMALL_POLYTOPE_JUDGE){
					for (uint32_t i = 0; i < 3; i++) {
						gen_zono[(gen_zono_base + k_indep_num) * 3 + i] = RZ_1[(RZ_base_1 + z) * 3 + i];
					}
					k_indep_num++;
				}
				else{
					for (uint32_t i = 0; i < 3; i++) {
						reduced_generators[i] += RZ_1[(RZ_base_1 + z) * 3 + i];
					}
				}
			}
		}

		for (uint32_t z = 1; z < RZ_length_2; z++) {
			if (!kc_info_2[z]) {
				double norm = 0;
				for (uint32_t i = 0; i < 3; i++) {
					norm += RZ_2[(RZ_base_2 + z) * 3 + i] * RZ_2[(RZ_base_2 + z) * 3 + i];
				}

				if(norm >= TOO_SMALL_POLYTOPE_JUDGE){
					for (uint32_t i = 0; i < 3; i++) {
						gen_zono[(gen_zono_base + k_indep_num) * 3 + i] = RZ_2[(RZ_base_2 + z) * 3 + i];
					}
					k_indep_num++;
				}
				else{
					for (uint32_t i = 0; i < 3; i++) {
						reduced_generators[i] += RZ_2[(RZ_base_2 + z) * 3 + i];
					}
				}

				if(k_indep_num >= gen_zono_length - 3){
					break;
				}
			}
		}

		for (uint32_t i = 0; i < 3; i++) {
			for (uint32_t j = 0; j < 3; j++){
				if(i == j){
					gen_zono[(gen_zono_base + k_indep_num) * 3 + j] = reduced_generators[i] + BUFFER_DIST;
				}
				else{
					gen_zono[(gen_zono_base + k_indep_num) * 3 + j] = 0;
				}
			}
			k_indep_num++;
		}
	}
}

void rotatotopeArray::evaluate_constraints(double* k_opt) {
	start_t = clock();
	if(con != nullptr){
		delete[] con;
		delete[] jaco_con;
		delete[] hess_con;
	}

	if(con_self != nullptr){
		delete[] con_self;
		delete[] jaco_con_self;
		delete[] hess_con_self;
	}
	
	current_k_opt = k_opt;

	con = new double[n_links * n_obstacles * n_time_steps];
	double* dev_con;
	cudaMalloc((void**)&dev_con, n_links * n_obstacles * n_time_steps * sizeof(double));

	jaco_con = new double[n_links * n_obstacles * n_time_steps * n_links * 2];
	double* dev_jaco_con;
	cudaMalloc((void**)&dev_jaco_con, n_links * n_obstacles * n_time_steps * n_links * 2 * sizeof(double));
	cudaMemset(dev_jaco_con, 0, n_links * n_obstacles * n_time_steps * n_links * 2 * sizeof(double));

	hess_con = new double[n_links * n_obstacles * n_time_steps * n_links * (n_links * 2 - 1)];
	double* dev_hess_con;
	cudaMalloc((void**)&dev_hess_con, n_links * n_obstacles * n_time_steps * n_links * (n_links * 2 - 1) * sizeof(double));
	cudaMemset(dev_hess_con, 0, n_links * n_obstacles * n_time_steps * n_links * (n_links * 2 - 1) * sizeof(double));

	con_self = new double[n_pairs * n_time_steps];
	double* dev_con_self;
	cudaMalloc((void**)&dev_con_self, n_pairs * n_time_steps * sizeof(double));

	jaco_con_self = new double[n_pairs * n_time_steps * n_links * 2];
	double* dev_jaco_con_self;
	cudaMalloc((void**)&dev_jaco_con_self, n_pairs * n_time_steps * n_links * 2 * sizeof(double));
	cudaMemset(dev_jaco_con_self, 0, n_pairs * n_time_steps * n_links * 2 * sizeof(double));

	hess_con_self = new double[n_pairs * n_time_steps * n_links * (n_links * 2 - 1)];
	double* dev_hess_con_self;
	cudaMalloc((void**)&dev_hess_con_self, n_pairs * n_time_steps * n_links * (n_links * 2 - 1) * sizeof(double));
	cudaMemset(dev_hess_con_self, 0, n_pairs * n_time_steps * n_links * (n_links * 2 - 1) * sizeof(double));


	double* lambda = new double[n_links * 2];
	for (uint32_t joint_id = 0; joint_id < n_links * 2; joint_id++) {
		lambda[joint_id] = c_k[joint_id] + k_opt[joint_id] / g_k[joint_id];
	}

	double* dev_lambda;
	cudaMalloc((void**)&dev_lambda, n_links * 2 * sizeof(double));
	cudaMemcpy(dev_lambda, lambda, n_links * 2 * sizeof(double), cudaMemcpyHostToDevice);

	double* dev_g_k;
	cudaMalloc((void**)&dev_g_k, n_links * 2 * sizeof(double));
	cudaMemcpy(dev_g_k, g_k, n_links * 2 * sizeof(double), cudaMemcpyHostToDevice);

	// obstacles constraint evaluation
	for (uint32_t link_id = 0; link_id < n_links; link_id++) {
		uint32_t buff_obstacle_length = RZ_length[link_id] + 3;
		uint32_t constraint_length = ((buff_obstacle_length - 1) * (buff_obstacle_length - 2)) / 2;

		double* dev_con_result; // results of evaluation of constriants
		bool* dev_index_factor; // whether the constraints are positive or negative
		cudaMalloc((void**)&dev_con_result, n_obstacles * n_time_steps * constraint_length * sizeof(double));
		cudaMalloc((void**)&dev_index_factor, n_obstacles * n_time_steps * constraint_length * sizeof(bool));

		dim3 grid1(n_obstacles, n_time_steps, 1);
		dim3 block1(constraint_length, 1, 1);
		evaluate_constraints_kernel << < grid1, block1 >> > (dev_lambda, link_id, RZ_length[link_id], dev_A_con[link_id], max_k_con_num[link_id], dev_d_con[link_id], dev_delta_con[link_id], dev_k_con[link_id], dev_k_con_num[link_id], dev_con_result, dev_index_factor);
		
		dim3 grid2(n_obstacles, n_time_steps, 1);
		dim3 block2((link_id + 1) * 2, (link_id + 1) * 2, 1);
		evaluate_gradient_kernel << < grid2, block2 >> > (dev_con_result, dev_index_factor, link_id, link_id, RZ_length[link_id], constraint_length, dev_lambda, dev_g_k, dev_A_con[link_id], max_k_con_num[link_id], dev_k_con[link_id], dev_k_con_num[link_id], n_links, dev_con, dev_jaco_con, dev_hess_con);

		cudaFree(dev_con_result);
		cudaFree(dev_index_factor);
	}

	// self intersection constraint evaluation
	for (uint32_t pair_id = 0; pair_id < n_pairs; pair_id++) {
		uint32_t R1 = self_pairs[pair_id * 2];
		uint32_t R2 = self_pairs[pair_id * 2 + 1];
		uint32_t R1_length = RZ_length[R1];
		uint32_t R2_length = RZ_length[R2];

		uint32_t gen_zono_length = R2_length;
		uint32_t constraint_length = ((gen_zono_length - 1) * (gen_zono_length - 2)) / 2;

		double* dev_con_result; // results of evaluation of constriants
		bool* dev_index_factor; // whether the constraints are positive or negative
		cudaMalloc((void**)&dev_con_result, n_time_steps * constraint_length * sizeof(double));
		cudaMalloc((void**)&dev_index_factor, n_time_steps * constraint_length * sizeof(bool));

		dim3 grid1(1, n_time_steps, 1);
		dim3 block1(constraint_length, 1, 1);
		evaluate_constraints_kernel << < grid1, block1 >> > (dev_lambda, R2, R1_length, dev_A_con_self[pair_id], max_k_con_num_self[pair_id], dev_d_con_self[pair_id], dev_delta_con_self[pair_id], dev_k_con_self[pair_id], dev_k_con_num_self[pair_id], dev_con_result, dev_index_factor);
		
		dim3 grid2(1, n_time_steps, 1);
		dim3 block2((R2 + 1) * 2, (R2 + 1) * 2, 1);
		evaluate_gradient_kernel << < grid2, block2 >> > (dev_con_result, dev_index_factor, R2, pair_id, R1_length, constraint_length, dev_lambda, dev_g_k, dev_A_con_self[pair_id], max_k_con_num_self[pair_id], dev_k_con_self[pair_id], dev_k_con_num_self[pair_id], n_links, dev_con_self, dev_jaco_con_self, dev_hess_con_self);

		cudaFree(dev_con_result);
		cudaFree(dev_index_factor);
	}

	cudaMemcpy(con, dev_con, n_links * n_obstacles * n_time_steps * sizeof(double), cudaMemcpyDeviceToHost);
	cudaFree(dev_con);

	cudaMemcpy(jaco_con, dev_jaco_con, n_links * n_obstacles * n_time_steps * n_links * 2 * sizeof(double), cudaMemcpyDeviceToHost);
	cudaFree(dev_jaco_con);

	cudaMemcpy(hess_con, dev_hess_con, n_links * n_obstacles * n_time_steps * n_links * (n_links * 2 - 1)  * sizeof(double), cudaMemcpyDeviceToHost);
	cudaFree(dev_hess_con);

	cudaMemcpy(con_self, dev_con_self, n_pairs * n_time_steps * sizeof(double), cudaMemcpyDeviceToHost);
	cudaFree(dev_con_self);

	cudaMemcpy(jaco_con_self, dev_jaco_con_self, n_pairs * n_time_steps * n_links * 2 * sizeof(double), cudaMemcpyDeviceToHost);
	cudaFree(dev_jaco_con_self);

	cudaMemcpy(hess_con_self, dev_hess_con_self, n_pairs * n_time_steps * n_links * (n_links * 2 - 1)  * sizeof(double), cudaMemcpyDeviceToHost);
	cudaFree(dev_hess_con_self);
	
	delete[] lambda;
	cudaFree(dev_lambda);

	cudaFree(dev_g_k);

	end_t = clock();
	if(debugMode){
		mexPrintf("CUDA: constraint evaluation time: %.6f ms\n", 1000.0 * (end_t - start_t) / (double)(CLOCKS_PER_SEC));
	}
}

__global__ void evaluate_constraints_kernel(double* lambda, uint32_t link_id, uint32_t RZ_length, double* A_con, uint32_t A_con_width, double* d_con, double* delta_con, bool* k_con, uint8_t* k_con_num, double* con_result, bool* index_factor) {
	uint32_t obstacle_id = blockIdx.x;
	uint32_t time_id = blockIdx.y;
	uint32_t n_time_steps = gridDim.y;
	uint32_t c_id = threadIdx.x;
	uint32_t constraint_length = blockDim.x;
	uint32_t k_con_num_base = time_id;
	uint32_t con_base = (obstacle_id * n_time_steps + time_id) * constraint_length + c_id;
	uint32_t con_result_base = (obstacle_id * n_time_steps + time_id) * constraint_length + c_id;

	__shared__ double shared_lambda[6];
	__shared__ double lambdas_prod[MAX_K_DEP_SIZE];

	if (c_id < 2 * (link_id + 1)) {
		shared_lambda[c_id] = lambda[c_id];
	}

	__syncthreads();

	if (c_id < k_con_num[k_con_num_base]) {
		double prod = 1.0;
		for (uint32_t j = 0; j < 2 * (link_id + 1); j++) {
			if (k_con[(j * n_time_steps + time_id) * RZ_length + c_id]) {
				prod *= shared_lambda[j];
			}
		}
		lambdas_prod[c_id] = prod;
	}

	__syncthreads();

	if (delta_con[con_base] == A_BIG_NUMBER){
		con_result[con_result_base] = -A_BIG_NUMBER;
		index_factor[con_result_base] = false;
	}
	else{
		double result = 0;
		for (uint32_t p = 0; p < k_con_num[k_con_num_base]; p++){
			result += lambdas_prod[p] * A_con[con_base * A_con_width + p];
		}

		double pos_result =  result - d_con[con_base] - delta_con[con_base];
		double neg_result = -result + d_con[con_base] - delta_con[con_base];

		if(pos_result > neg_result){
			con_result[con_result_base] = pos_result;
			index_factor[con_result_base] = true;
		}
		else{
			con_result[con_result_base] = neg_result;
			index_factor[con_result_base] = false;
		}     
	}
}

__global__ void evaluate_gradient_kernel(double* con_result, bool* index_factor, uint32_t link_id, uint32_t pos_id, uint32_t RZ_length, uint32_t constraint_length, double* lambda, double* g_k, double* A_con, uint32_t A_con_width, bool* k_con, uint8_t* k_con_num, uint32_t n_links, double* con, double* jaco_con, double* hess_con) {
	uint32_t obstacle_id = blockIdx.x;
	uint32_t n_obstacles = gridDim.x;
	uint32_t time_id = blockIdx.y;
	uint32_t n_time_steps = gridDim.y;
	uint32_t joint_id = threadIdx.x;
	uint32_t joint_id_sec = threadIdx.y;
	uint32_t k_con_num_base = time_id;
	
	uint32_t con_result_base = (obstacle_id * n_time_steps + time_id) * constraint_length;
	__shared__ uint32_t max_idx;
	uint32_t valu_con_base = (pos_id * n_obstacles + obstacle_id) * n_time_steps + time_id;
	uint32_t jaco_con_base = ((pos_id * n_obstacles + obstacle_id) * n_time_steps + time_id) * n_links * 2;
	uint32_t hess_con_base = ((pos_id * n_obstacles + obstacle_id) * n_time_steps + time_id) * n_links * (n_links * 2 - 1);

	__shared__ double shared_lambda[6];
	__shared__ double max_index_factor;

	if (joint_id_sec == 0) {
		if(joint_id == 0){
			double maximum = -A_BIG_NUMBER - A_BIG_NUMBER;
			max_idx = 0;
			for (uint32_t i = 0; i < constraint_length; i++) {
				double cur = con_result[con_result_base + i];
				if (maximum < cur) {
					max_idx = con_result_base + i;
					maximum = cur;
				}
			}
			con[valu_con_base] = -maximum + CONSERVATIVE_BUFFER;

			if(index_factor[max_idx]){
				max_index_factor = 1.0;
			}
			else{
				max_index_factor = -1.0;
			}
		}
		else if (joint_id <= 2 * (link_id + 1)) {
			shared_lambda[joint_id - 1] = lambda[joint_id - 1];
		}
	}

	__syncthreads();

	if(joint_id == joint_id_sec){
		double result = 0;
		for (uint32_t p = 0; p < k_con_num[k_con_num_base]; p++) {
			if(k_con[(joint_id * n_time_steps + time_id) * RZ_length + p]){
				double prod = 1.0;
				for (uint32_t j = 0; j < 2 * (link_id + 1); j++) {
					if (j != joint_id && k_con[(j * n_time_steps + time_id) * RZ_length + p]) {
						prod *= shared_lambda[j];
					}
				}

				result += prod * max_index_factor * A_con[max_idx * A_con_width + p];
			}
		}
		
		jaco_con[jaco_con_base + joint_id] = -result / g_k[joint_id];
	}
	else if(joint_id > joint_id_sec){
		double result = 0;
		for (uint32_t p = 0; p < k_con_num[k_con_num_base]; p++) {
			if(k_con[(joint_id * n_time_steps + time_id) * RZ_length + p] && k_con[(joint_id_sec * n_time_steps + time_id) * RZ_length + p]){
				double prod = 1.0;
				for (uint32_t j = 0; j < 2 * (link_id + 1); j++) {
					if (j != joint_id && j != joint_id_sec && k_con[(j * n_time_steps + time_id) * RZ_length + p]) {
						prod *= shared_lambda[j];
					}
				}
				
				result += prod * max_index_factor * A_con[max_idx * A_con_width + p];
			}
		}
		
		uint32_t hess_index = 0;
		for(uint32_t i = 0; i < joint_id_sec; i++){
			hess_index += n_links * 2 - 1 - i;
		}
		hess_con[hess_con_base + joint_id * (joint_id - 1) / 2 + joint_id_sec] = -result / g_k[joint_id] / g_k[joint_id_sec];
	}	
}

rotatotopeArray::~rotatotopeArray() {
	cudaFree(dev_Z);

	if (n_links > 0) {
		cudaFree(dev_RZ);
		cudaFree(dev_c_idx);
		cudaFree(dev_k_idx);
	}
	
	if (c_k != nullptr) {
		delete[] c_k;
		delete[] g_k;
	}

	if (dev_RZ_stack != nullptr) {
		for (uint32_t i = 0; i < n_links; i++) {
			delete[] RZ_stack[i];
		}
		delete[] RZ_stack;

		for (uint32_t i = 0; i < n_links; i++) {
			cudaFree(dev_RZ_stack[i]);
		}
		delete[] dev_RZ_stack;

		for (uint32_t i = 0; i < n_links; i++) {
			delete[] c_idx_stack[i];
		}
		delete[] c_idx_stack;

		for (uint32_t i = 0; i < n_links; i++) {
			cudaFree(dev_c_idx_stack[i]);
		}
		delete[] dev_c_idx_stack;

		for (uint32_t i = 0; i < n_links; i++) {
			delete[] k_idx_stack[i];
		}
		delete[] k_idx_stack;

		for (uint32_t i = 0; i < n_links; i++) {
			cudaFree(dev_k_idx_stack[i]);
		}
		delete[] dev_k_idx_stack;

		delete[] RZ_length;
	}

	if (A_con != nullptr) {
		for (uint32_t i = 0; i < n_links; i++) {
			delete[] A_con[i];
		}
		delete[] A_con;

		for (uint32_t i = 0; i < n_links; i++) {
			cudaFree(dev_A_con[i]);
		}
		delete[] dev_A_con;

		for (uint32_t i = 0; i < n_links; i++) {
			delete[] d_con[i];
		}
		delete[] d_con;

		for (uint32_t i = 0; i < n_links; i++) {
			cudaFree(dev_d_con[i]);
		}
		delete[] dev_d_con;

		for (uint32_t i = 0; i < n_links; i++) {
			delete[] delta_con[i];
		}
		delete[] delta_con;

		for (uint32_t i = 0; i < n_links; i++) {
			cudaFree(dev_delta_con[i]);
		}
		delete[] dev_delta_con;

		for (uint32_t i = 0; i < n_links; i++) {
			delete[] k_con[i];
		}
		delete[] k_con;

		for (uint32_t i = 0; i < n_links; i++) {
			cudaFree(dev_k_con[i]);
		}
		delete[] dev_k_con;

		for (uint32_t i = 0; i < n_links; i++) {
			delete[] k_con_num[i];
		}
		delete[] k_con_num;

		for (uint32_t i = 0; i < n_links; i++) {
			cudaFree(dev_k_con_num[i]);
		}
		delete[] dev_k_con_num;

		delete[] max_k_con_num;
	}

	if(dev_A_con_self != nullptr){
		for (uint32_t i = 0; i < n_pairs; i++) {
			delete[] A_con_self[i];
		}
		delete[] A_con_self;

		for (uint32_t i = 0; i < n_pairs; i++) {
			cudaFree(dev_A_con_self[i]);
		}
		delete[] dev_A_con_self;

		for (uint32_t i = 0; i < n_pairs; i++) {
			delete[] d_con_self[i];
		}
		delete[] d_con_self;

		for (uint32_t i = 0; i < n_pairs; i++) {
			cudaFree(dev_d_con_self[i]);
		}
		delete[] dev_d_con_self;

		for (uint32_t i = 0; i < n_pairs; i++) {
			delete[] delta_con_self[i];
		}
		delete[] delta_con_self;

		for (uint32_t i = 0; i < n_pairs; i++) {
			cudaFree(dev_delta_con_self[i]);
		}
		delete[] dev_delta_con_self;

		for (uint32_t i = 0; i < n_pairs; i++) {
			delete[] k_con_self[i];
		}
		delete[] k_con_self;

		for (uint32_t i = 0; i < n_pairs; i++) {
			cudaFree(dev_k_con_self[i]);
		}
		delete[] dev_k_con_self;

		for (uint32_t i = 0; i < n_pairs; i++) {
			delete[] k_con_num_self[i];
		}
		delete[] k_con_num_self;

		for (uint32_t i = 0; i < n_pairs; i++) {
			cudaFree(dev_k_con_num_self[i]);
		}
		delete[] dev_k_con_num_self;

		delete[] max_k_con_num_self;
	}

	if (debug_RZ != nullptr) {
		delete[] debug_RZ;
		delete[] debug_c_idx;
		delete[] debug_k_idx;
	}

	if (con != nullptr) {
		delete[] con;
		delete[] jaco_con;
		delete[] hess_con;
	}

	if (con_self != nullptr) {
		delete[] con_self;
		delete[] jaco_con_self;
		delete[] hess_con_self;
	}
}

#endif // !ROTATOTOPE_ARRAY_CPPs

