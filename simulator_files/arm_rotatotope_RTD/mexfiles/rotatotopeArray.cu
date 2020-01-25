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

		bool *dev_c_idx_new;
		uint8_t *dev_k_idx_new, *dev_C_idx_new;
		cudaMalloc((void**)&dev_c_idx, n_links * n_time_steps * reduce_order * sizeof(bool));
		cudaMemset(dev_c_idx, 0, n_links * n_time_steps * reduce_order * sizeof(bool));
		cudaMalloc((void**)&dev_c_idx_new, n_links * n_time_steps * reduce_order * R_unit_length * sizeof(bool));
		cudaMemset(dev_c_idx_new, 0, n_links * n_time_steps * reduce_order * R_unit_length * sizeof(bool));
		cudaMalloc((void**)&dev_k_idx, n_links * (n_links + 1) * n_time_steps * reduce_order * sizeof(uint8_t));
		cudaMemset(dev_k_idx, 0, n_links * (n_links + 1) * n_time_steps * reduce_order * sizeof(uint8_t));
		cudaMalloc((void**)&dev_k_idx_new, n_links * (n_links + 1) * n_time_steps * reduce_order * R_unit_length * sizeof(uint8_t));
		cudaMemset(dev_k_idx_new, 0, n_links * (n_links + 1) * n_time_steps * reduce_order * R_unit_length * sizeof(uint8_t));
		cudaMalloc((void**)&dev_C_idx, n_links * (n_links + 1) * n_time_steps * reduce_order * sizeof(uint8_t));
		cudaMemset(dev_C_idx, 0, n_links * (n_links + 1) * n_time_steps * reduce_order * sizeof(uint8_t));
		cudaMalloc((void**)&dev_C_idx_new, n_links * (n_links + 1) * n_time_steps * reduce_order * R_unit_length * sizeof(uint8_t));
		cudaMemset(dev_C_idx_new, 0, n_links * (n_links + 1) * n_time_steps * reduce_order * R_unit_length * sizeof(uint8_t));

		dim3 grid1(n_links, n_time_steps, 1);
		dim3 block1(reduce_order, Z_width, 1);
		initialize_RZ_kernel << < grid1, block1 >> > (dev_Z, Z_unit_length, reduce_order, dev_RZ, dev_c_idx);

		for (int link = n_links; link > 0; link--) {
			for (int joint_offset = joint_per_link - 1; joint_offset >= 0; joint_offset--) {
				dim3 grid2(link, n_time_steps, 1);
				dim3 block2(reduce_order, R_unit_length, 1);
				multiply_kernel << < grid2, block2 >> > (dev_rot_axes, n_links - link, joint_offset, reduce_order, dev_RZ, dev_R, dev_c_idx, dev_k_idx, dev_C_idx, dev_RZ_new, dev_c_idx_new, dev_k_idx_new, dev_C_idx_new);

				reduce_kernel << < grid2, (reduce_order * R_unit_length) >> > (dev_RZ_new, dev_c_idx_new, dev_k_idx_new, dev_C_idx_new, n_links - link, reduce_order, dev_RZ, dev_c_idx, dev_k_idx, dev_C_idx);
			}
		}

		cudaFree(dev_RZ_new);
		cudaFree(dev_c_idx_new);
		cudaFree(dev_k_idx_new);
		cudaFree(dev_C_idx_new);
	}
	else {
		c_k = nullptr;
		g_k = nullptr;
		Z = nullptr;
		dev_Z = nullptr;
		dev_RZ = nullptr;
		dev_c_idx = nullptr;
		dev_k_idx = nullptr;
		dev_C_idx = nullptr;
	}

	n_pairs = 0;
	self_pairs = nullptr;
	
	dev_RZ_stack = nullptr;
	dev_c_idx_stack = nullptr;
	dev_k_idx_stack = nullptr;
	dev_C_idx_stack = nullptr;
	RZ_length = nullptr;

	n_obstacles = 0;
	A = nullptr;
	dev_A = nullptr;

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

__global__ void multiply_kernel(uint8_t* rot_axes, uint32_t link_offset, uint32_t joint_offset, uint32_t reduce_order, double* RZ, double* R, bool* c_idx, uint8_t* k_idx, uint8_t* C_idx, double* RZ_new, bool* c_idx_new, uint8_t* k_idx_new, uint8_t* C_idx_new) {
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

	// update k_idx for this joint
	uint32_t k_id = link_id * (link_id + 1) + joint_id;
	uint32_t mul_k = (k_id * n_time_steps + time_id) * reduce_order * R_unit_length + (z_id * R_unit_length + r_id);
	if (R[mul_R * 5 + k_dim] != 0) {
		k_idx_new[mul_k] = 2;
	}
	else {
		k_idx_new[mul_k] = 1;
	}

	// update k_idx for previous joints
	for (uint32_t joint_k_id = joint_id + 1; joint_k_id < (link_id + 1) * 2; joint_k_id++) {
		k_id = link_id * (link_id + 1) + joint_k_id;
		uint32_t mul_z = (k_id * n_time_steps + time_id) * reduce_order + z_id;
		mul_k = (k_id * n_time_steps + time_id) * reduce_order * R_unit_length + (z_id * R_unit_length + r_id);
		k_idx_new[mul_k] = k_idx[mul_z];
	}

	// update C_idx for this joint
	uint32_t C_id = link_id * (link_id + 1) + joint_id;
	uint32_t mul_C = (C_id * n_time_steps + time_id) * reduce_order * R_unit_length + (z_id * R_unit_length + r_id);
	if (r_id == 0) {
		C_idx_new[mul_C] = 2;
	}
	else {
		C_idx_new[mul_C] = 1;
	}

	// update C_idx for previous joints
	for (uint32_t joint_k_id = joint_id + 1; joint_k_id < (link_id + 1) * 2; joint_k_id++) {
		C_id = link_id * (link_id + 1) + joint_k_id;
		uint32_t mul_z = (C_id * n_time_steps + time_id) * reduce_order + z_id;
		mul_C = (C_id * n_time_steps + time_id) * reduce_order * R_unit_length + (z_id * R_unit_length + r_id);
		C_idx_new[mul_C] = C_idx[mul_z];
	}
}

__global__ void reduce_kernel(double* RZ_new, bool* c_idx_new, uint8_t* k_idx_new, uint8_t* C_idx_new, uint32_t link_offset, uint32_t reduce_order, double* RZ, bool* c_idx, uint8_t* k_idx, uint8_t* C_idx) {
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

		uint32_t C_pivot = k_start, C_pivot_ori = k_start_ori;
		while (C_pivot != k_end && C_pivot_ori != k_end_ori) {
			C_idx[C_pivot_ori + z_id] = C_idx_new[C_pivot + sorted_id];
			C_pivot += k_step;
			C_pivot_ori += k_step_ori;
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
			k_idx[h + z_id] = 1;
		}
		for (uint32_t h = k_start_ori; h < k_end_ori; h += k_step_ori) {
			C_idx[h + z_id] = 1;
		}
	}
}

void rotatotopeArray::stack(rotatotopeArray &EEs, rotatotopeArray &base) {
	RZ_stack = new double*[n_links];
	dev_RZ_stack = new double*[n_links];
	c_idx_stack = new bool*[n_links];
	dev_c_idx_stack = new bool*[n_links];
	k_idx_stack = new uint8_t*[n_links];
	dev_k_idx_stack = new uint8_t*[n_links];
	C_idx_stack = new uint8_t*[n_links];
	dev_C_idx_stack = new uint8_t*[n_links];
	RZ_length = new uint32_t[n_links];

	for (uint32_t link_id = 0; link_id < n_links; link_id++) {
		RZ_length[link_id] = reduce_order + link_id * (EEs.reduce_order - 1) + base.reduce_order - 1;

		RZ_stack[link_id] = nullptr;
		cudaMalloc((void**)&(dev_RZ_stack[link_id]), n_time_steps * RZ_length[link_id] * Z_width * sizeof(double));

		c_idx_stack[link_id] = nullptr;
		cudaMalloc((void**)&(dev_c_idx_stack[link_id]), n_time_steps * RZ_length[link_id] * sizeof(bool));

		k_idx_stack[link_id] = nullptr;
		cudaMalloc((void**)&(dev_k_idx_stack[link_id]), 2 * (link_id + 1) * n_time_steps * RZ_length[link_id] * sizeof(uint8_t));
		cudaMemset(dev_k_idx_stack, 0, 2 * (link_id + 1) * n_time_steps * RZ_length[link_id] * sizeof(uint8_t));

		C_idx_stack[link_id] = nullptr;
		cudaMalloc((void**)&(dev_C_idx_stack[link_id]), 2 * (link_id + 1) * n_time_steps * RZ_length[link_id] * sizeof(uint8_t));
		cudaMemset(dev_C_idx_stack, 0, 2 * (link_id + 1) * n_time_steps * RZ_length[link_id] * sizeof(uint8_t));

		// copy dev_RZ to dev_RZ_stack
		dim3 grid1(n_time_steps, 1, 1);
		dim3 block1(reduce_order, Z_width, 1);
		copy_kernel << < grid1, block1 >> > (link_id, dev_RZ, dev_c_idx, dev_k_idx, dev_C_idx, reduce_order, EEs.reduce_order, dev_RZ_stack[link_id], dev_c_idx_stack[link_id], dev_k_idx_stack[link_id], dev_C_idx_stack[link_id]);

		// stack with EE
		for (int EE_id = link_id - 1; EE_id >= 0; EE_id--) {
			dim3 grid2(n_time_steps, 1, 1);
			dim3 block2(EEs.reduce_order, Z_width, 1);
			stack_kernel << < grid2, block2 >> > (link_id, EE_id, EE_id, reduce_order, EEs.reduce_order, dev_RZ_stack[link_id], EEs.dev_RZ, dev_c_idx_stack[link_id], EEs.dev_c_idx, dev_k_idx_stack[link_id], EEs.dev_k_idx, dev_C_idx_stack[link_id], EEs.dev_C_idx);
		}

		// stack with base
		dim3 grid3(n_time_steps, 1, 1);
		dim3 block3(base.reduce_order, Z_width, 1);
		stack_kernel << < grid3, block3 >> > (link_id, 0, link_id, reduce_order, base.reduce_order, dev_RZ_stack[link_id], base.dev_RZ, dev_c_idx_stack[link_id], base.dev_c_idx, dev_k_idx_stack[link_id], base.dev_k_idx, dev_C_idx_stack[link_id], base.dev_C_idx);
		
		// origin shift
		origin_shift_kernel <<< n_time_steps, 1 >>> (RZ_length[link_id], dev_RZ_stack[link_id]);
	}

	uint32_t link_id = 2;
	if(debugMode){
		debug_RZ = new double[n_time_steps * RZ_length[link_id] * Z_width];
		cudaMemcpy(debug_RZ, dev_RZ_stack[link_id], n_time_steps * RZ_length[link_id] * Z_width * sizeof(double), cudaMemcpyDeviceToHost);

		debug_c_idx = new bool[n_time_steps * RZ_length[link_id]];
		cudaMemcpy(debug_c_idx, dev_c_idx_stack[link_id], n_time_steps * RZ_length[link_id] * sizeof(bool), cudaMemcpyDeviceToHost);

		debug_k_idx = new uint8_t[2 * (link_id + 1) * n_time_steps * RZ_length[link_id]];
		cudaMemcpy(debug_k_idx, dev_k_idx_stack[link_id], 2 * (link_id + 1) * n_time_steps * RZ_length[link_id] * sizeof(uint8_t), cudaMemcpyDeviceToHost);
		
		debug_C_idx = new uint8_t[2 * (link_id + 1) * n_time_steps * RZ_length[link_id]];
		cudaMemcpy(debug_C_idx, dev_C_idx_stack[link_id], 2 * (link_id + 1) * n_time_steps * RZ_length[link_id] * sizeof(uint8_t), cudaMemcpyDeviceToHost);
	}
	else{
		debug_RZ = nullptr;
		debug_c_idx = nullptr;
		debug_k_idx = nullptr;
		debug_C_idx = nullptr;
	}
}

__global__ void copy_kernel(uint32_t link_id, double* RZ, bool* c_idx, uint8_t* k_idx, uint8_t* C_idx, uint32_t link_reduce_order, uint32_t point_reduce_order, double* RZ_stack, bool* c_idx_stack, uint8_t* k_idx_stack, uint8_t* C_idx_stack) {
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

		uint32_t copy_C = copy_k_start;
		for (uint32_t link_C = link_k_start; link_C < link_k_end; link_C += link_k_step) {
			C_idx_stack[copy_C] = C_idx[link_C];
			copy_C += copy_k_step;
		}
	}
}

__global__ void stack_kernel(uint32_t link_id, uint32_t EE_id, uint32_t stack_offset, uint32_t link_reduce_order, uint32_t point_reduce_order, double* RZ_stack, double* EE_RZ, bool* c_idx_stack, bool* EE_c_idx, uint8_t* k_idx_stack, uint8_t* EE_k_idx, uint8_t* C_idx_stack, uint8_t* EE_C_idx) {
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
					k_idx_stack[stack_k] = 0;
				}

				EE_k += EE_k_step;
			}

			uint32_t EE_C = EE_k_start;
			for (uint32_t stack_C = stack_k_start + stack_offset_length; stack_C < stack_k_end + stack_offset_length; stack_C += stack_k_step) {
				if (EE_C < EE_k_end) {
					C_idx_stack[stack_C] = EE_C_idx[EE_C];
				}
				else {
					C_idx_stack[stack_C] = 0;
				}

				EE_C += EE_k_step;
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

void rotatotopeArray::generate_constraints(uint32_t n_obstacles_in, double* OZ_input, uint32_t OZ_width, uint32_t OZ_length) {
	// obstacle constraints
	n_obstacles = n_obstacles_in;

	if(n_obstacles == 0) return;

	OZ_unit_length = OZ_length / n_obstacles;

	OZ = OZ_input;
	cudaMalloc((void**)&dev_OZ, OZ_length * OZ_width * sizeof(double));
	cudaMemcpy(dev_OZ, OZ, OZ_length * OZ_width * sizeof(double), cudaMemcpyHostToDevice);

	A = new double*[n_links];
	dev_A = new double*[n_links];
	for (uint32_t link_id = 0; link_id < n_links; link_id++) {
		uint32_t buff_obstacle_length = RZ_length[link_id] + (OZ_unit_length - 1);
		uint32_t constraint_length = ((buff_obstacle_length - 1) * (buff_obstacle_length - 2)) / 2;

		// generate obstacles polynomials
		cudaMalloc((void**)&(dev_A[link_id]), n_obstacles * n_time_steps * constraint_length * 3 * sizeof(double));
		cudaMemset(dev_A[link_id], 0, n_obstacles * n_time_steps * constraint_length * 3 * sizeof(double));
		
		dim3 grid1(n_obstacles, n_time_steps, 1);
		dim3 block1(constraint_length, 1, 1);
		generate_polytope_normals << < grid1, block1 >> > (buff_obstacle_length, dev_RZ_stack[link_id], dev_OZ, OZ_unit_length, dev_A[link_id]);

		if(debugMode){
			A[link_id] = new double[n_obstacles * n_time_steps * constraint_length * 3];
			cudaMemcpy(A[link_id], dev_A[link_id], n_obstacles * n_time_steps * constraint_length * 3 * sizeof(double), cudaMemcpyDeviceToHost);
		}
		else{
			A[link_id] = nullptr;
		}
	}
}

__global__ void generate_polytope_normals(uint32_t buff_obstacle_length, double* RZ, double* OZ, uint32_t OZ_unit_length, double* A){
	uint32_t obstacle_id = blockIdx.x;
	uint32_t time_id = blockIdx.y;
	uint32_t n_time_steps = gridDim.y;
	uint32_t RZ_length = buff_obstacle_length - (OZ_unit_length - 1);
	uint32_t RZ_base = time_id * RZ_length;
	double   buff_obstacle_size = (double)buff_obstacle_length - 1.0;
	uint32_t obstacle_base = obstacle_id * OZ_unit_length;
	uint32_t c_id = threadIdx.x;
	uint32_t constraint_length = blockDim.x;
	uint32_t first = (uint32_t)floor(-0.5*sqrt(4 * buff_obstacle_size * buff_obstacle_size - 4 * buff_obstacle_size - 8.0 * ((double)c_id) + 1.0) + buff_obstacle_size - 0.5);
	uint32_t first_base = (first + 1) * 3;
	uint32_t second = c_id + 1 - ((2 * (buff_obstacle_length - 1) - 3 - first) * first) / 2;
	uint32_t second_base = (second + 1) * 3;
	uint32_t A_base = (obstacle_id * n_time_steps + time_id) * constraint_length + c_id;
	
	__shared__ double buff_obstacles[MAX_BUFF_OBSTACLE_SIZE];
	__shared__ bool   test[1024];
	__shared__ double obs_center[3];

	if(c_id < RZ_length) { // copy the original FRS
		buff_obstacles[c_id * 3    ] = RZ[(RZ_base + c_id) * 3    ];
		buff_obstacles[c_id * 3 + 1] = RZ[(RZ_base + c_id) * 3 + 1];
		buff_obstacles[c_id * 3 + 2] = RZ[(RZ_base + c_id) * 3 + 2];
	}
	else if(c_id < buff_obstacle_length){ // copy the obstacle generators
		for(uint32_t i = 0; i < 3; i++){
			if(i == c_id - RZ_length){
				buff_obstacles[c_id * 3 + i] = OZ[(obstacle_base + c_id - RZ_length + 1) * 3 + i] + BUFFER_DIST / 2;
			}
			else{
				buff_obstacles[c_id * 3 + i] = 0;
			}
		}
	}
	else if(c_id < buff_obstacle_length + 1){ // copy the obstacle center
		obs_center[0] = OZ[obstacle_base * 3    ];
		obs_center[1] = OZ[obstacle_base * 3 + 1];
		obs_center[2] = OZ[obstacle_base * 3 + 2];
	}

	__syncthreads();
	
	// generate A matrix
	double A_1 = buff_obstacles[first_base + 1] * buff_obstacles[second_base + 2] - buff_obstacles[first_base + 2] * buff_obstacles[second_base + 1];
	double A_2 = buff_obstacles[first_base + 2] * buff_obstacles[second_base    ] - buff_obstacles[first_base    ] * buff_obstacles[second_base + 2];
	double A_3 = buff_obstacles[first_base    ] * buff_obstacles[second_base + 1] - buff_obstacles[first_base + 1] * buff_obstacles[second_base    ];
	
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

	// generate d and deltaD
	double d = A_1 * buff_obstacles[0] + A_2 * buff_obstacles[1] + A_3 * buff_obstacles[2];

	double deltaD = 0;
	for (uint32_t i = 1; i < buff_obstacle_length; i++) {
		deltaD += abs(A_1 * buff_obstacles[i * 3] + A_2 * buff_obstacles[i * 3 + 1] + A_3 * buff_obstacles[i * 3 + 2]);
	}

	// add a test here that throws out unnecessary constraints, test = true for max(h) > 0, false for max(h) <= 0
	double Ax = A_1 * obs_center[0] + A_2 * obs_center[1] + A_3 * obs_center[2];
	double pos_res = Ax - d - deltaD;
	double neg_res = Ax + d - deltaD;

	if(A_s_q > 0){
		test[c_id] = (pos_res > 0) || (neg_res > 0);
	}
	else{
		test[c_id] = false;
	}

	__syncthreads();

	__shared__ bool intersection_possible;

	if(c_id == 0){
		intersection_possible = true;
		for(uint32_t i = 0; i < constraint_length; i++){
			if(test[i]){
				intersection_possible = false;
				break;
			}
		}
	}

	__syncthreads();

	intersection_possible = true;
	if(!intersection_possible){ // intersection is impossible, label A as empty
		if(c_id == 0){
			A[A_base * 3] = A_BIG_NUMBER;
		}
	}
	else{ // intersection is possible, copy the A matrix
		A[A_base * 3    ] = A_1;
		A[A_base * 3 + 1] = A_2;
		A[A_base * 3 + 2] = A_3;
	}
}

void rotatotopeArray::evaluate_constraints(double* k_opt) {
	start_t = clock();
	if(n_obstacles > 0 && con != nullptr){
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

	double* dev_con = nullptr;
	double* dev_jaco_con = nullptr;
	double* dev_hess_con = nullptr;
	if(n_obstacles > 0){
		con = new double[n_links * n_obstacles * n_time_steps];
		cudaMalloc((void**)&dev_con, n_links * n_obstacles * n_time_steps * sizeof(double));

		jaco_con = new double[n_links * n_obstacles * n_time_steps * n_links * 2];
		cudaMalloc((void**)&dev_jaco_con, n_links * n_obstacles * n_time_steps * n_links * 2 * sizeof(double));
		cudaMemset(dev_jaco_con, 0, n_links * n_obstacles * n_time_steps * n_links * 2 * sizeof(double));

		hess_con = new double[n_links * n_obstacles * n_time_steps * n_links * (n_links * 2 - 1)];
		cudaMalloc((void**)&dev_hess_con, n_links * n_obstacles * n_time_steps * n_links * (n_links * 2 - 1) * sizeof(double));
		cudaMemset(dev_hess_con, 0, n_links * n_obstacles * n_time_steps * n_links * (n_links * 2 - 1) * sizeof(double));
	}

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
		lambda[joint_id] = (k_opt[joint_id] - c_k[joint_id]) / g_k[joint_id];
	}

	double* dev_lambda;
	cudaMalloc((void**)&dev_lambda, n_links * 2 * sizeof(double));
	cudaMemcpy(dev_lambda, lambda, n_links * 2 * sizeof(double), cudaMemcpyHostToDevice);

	double* dev_g_k;
	cudaMalloc((void**)&dev_g_k, n_links * 2 * sizeof(double));
	cudaMemcpy(dev_g_k, g_k, n_links * 2 * sizeof(double), cudaMemcpyHostToDevice);

	// obstacles constraint evaluation
	if(n_obstacles > 0){
		for (uint32_t link_id = 0; link_id < n_links; link_id++) {
			uint32_t buff_obstacle_length = RZ_length[link_id] + 3;
			uint32_t constraint_length = ((buff_obstacle_length - 1) * (buff_obstacle_length - 2)) / 2;

			dim3 grid1(n_obstacles, n_time_steps, 1);
			dim3 block1(constraint_length, 1, 1);
			evaluate_sliced_constraints <<< grid1, block1 >>> (link_id, link_id, n_links, RZ_length[link_id], dev_RZ_stack[link_id], dev_c_idx_stack[link_id], dev_k_idx_stack[link_id], dev_C_idx_stack[link_id], dev_lambda, dev_OZ, OZ_unit_length, dev_A[link_id], dev_g_k, dev_con, dev_jaco_con, dev_hess_con);

		}
	}

	// self intersection constraint evaluation
	for (uint32_t pair_id = 0; pair_id < n_pairs; pair_id++) {
		uint32_t R1 = self_pairs[pair_id * 2];
		uint32_t R2 = self_pairs[pair_id * 2 + 1];
		uint32_t R1_length = RZ_length[R1];
		uint32_t R2_length = RZ_length[R2];

		uint32_t gen_zono_length = R2_length;
		uint32_t constraint_length = ((gen_zono_length - 1) * (gen_zono_length - 2)) / 2;
	}

	if(n_obstacles > 0){
		cudaMemcpy(con, dev_con, n_links * n_obstacles * n_time_steps * sizeof(double), cudaMemcpyDeviceToHost);
		cudaFree(dev_con);

		cudaMemcpy(jaco_con, dev_jaco_con, n_links * n_obstacles * n_time_steps * n_links * 2 * sizeof(double), cudaMemcpyDeviceToHost);
		cudaFree(dev_jaco_con);

		cudaMemcpy(hess_con, dev_hess_con, n_links * n_obstacles * n_time_steps * n_links * (n_links * 2 - 1)  * sizeof(double), cudaMemcpyDeviceToHost);
		cudaFree(dev_hess_con);
	}

	cudaMemcpy(con_self, dev_con_self, n_pairs * n_time_steps * sizeof(double), cudaMemcpyDeviceToHost);
	cudaFree(dev_con_self);

	cudaMemcpy(jaco_con_self, dev_jaco_con_self, n_pairs * n_time_steps * n_links * 2 * sizeof(double), cudaMemcpyDeviceToHost);
	cudaFree(dev_jaco_con_self);

	cudaMemcpy(hess_con_self, dev_hess_con_self, n_pairs * n_time_steps * n_links * (n_links * 2 - 1)  * sizeof(double), cudaMemcpyDeviceToHost);
	cudaFree(dev_hess_con_self);
	
	delete[] lambda;
	cudaFree(dev_lambda);

	cudaFree(dev_g_k);

	end_t = clock();mexPrintf("CUDA: constraint evaluation time: %.6f ms\n", 1000.0 * (end_t - start_t) / (double)(CLOCKS_PER_SEC));
	if(debugMode){
		mexPrintf("CUDA: constraint evaluation time: %.6f ms\n", 1000.0 * (end_t - start_t) / (double)(CLOCKS_PER_SEC));
	}
}

__global__ void evaluate_sliced_constraints(uint32_t link_id, uint32_t pos_id, uint32_t n_links, uint32_t RZ_length, double* RZ, bool* c_idx, uint8_t* k_idx, uint8_t* C_idx, double* lambda, double* OZ, uint32_t OZ_unit_length, double* A, double* g_k, double* con, double* jaco_con, double* hess_con){
	uint32_t obstacle_id = blockIdx.x;
	uint32_t n_obstacles = gridDim.x;
	uint32_t time_id = blockIdx.y;
	uint32_t n_time_steps = gridDim.y;
	uint32_t RZ_base = time_id * RZ_length;
	uint32_t buff_obstacle_length = RZ_length + (OZ_unit_length - 1);
	uint32_t c_id = threadIdx.x;
	uint32_t constraint_length = blockDim.x;
	uint32_t obstacle_base = obstacle_id * OZ_unit_length;
	uint32_t A_base = (obstacle_id * n_time_steps + time_id) * constraint_length + c_id;
	uint32_t valu_con_base = (pos_id * n_obstacles + obstacle_id) * n_time_steps + time_id;
	uint32_t jaco_con_base = ((pos_id * n_obstacles + obstacle_id) * n_time_steps + time_id) * n_links * 2;
	uint32_t hess_con_base = ((pos_id * n_obstacles + obstacle_id) * n_time_steps + time_id) * n_links * (n_links * 2 - 1);

	__shared__ bool A_is_empty;
	
	if(c_id == 0){ // confirm whether A is empty
		if (A[A_base * 3] == A_BIG_NUMBER) {
			con[valu_con_base] = -A_BIG_NUMBER;
			A_is_empty = true;
		}
		else{
			A_is_empty = false;
		}
	}

	__syncthreads();

	if(A_is_empty) return;
	
	__shared__ double g_sliced[MAX_BUFF_OBSTACLE_SIZE]; // omit the center, index starts from 1
	__shared__ bool   sliced_to_pt[MAX_RZ_LENGTH];
	__shared__ double shared_lambda[6];
	__shared__ double c_poly[3];
	__shared__ double g_poly[MAX_BUFF_OBSTACLE_SIZE]; // index starts from 0
	__shared__ uint32_t g_poly_num;
	__shared__ double con_res[1024]; // store array Pb here
	__shared__ bool   index_factor[1024]; // true for d+deltaD and false for -d+deltaD

	if(c_id < (link_id + 1) * 2){ // copy the lambda to shared memory
		shared_lambda[c_id] = lambda[c_id];
	}

	__syncthreads();

	if (c_id == 0) {
		c_poly[0] = RZ[RZ_base * 3    ] - OZ[obstacle_base * 3    ];
		c_poly[1] = RZ[RZ_base * 3 + 1] - OZ[obstacle_base * 3 + 1];
		c_poly[2] = RZ[RZ_base * 3 + 2] - OZ[obstacle_base * 3 + 2];
	}
	else if(c_id < RZ_length) { // copy the original FRS
		g_sliced[c_id * 3    ] = RZ[(RZ_base + c_id) * 3    ];
		g_sliced[c_id * 3 + 1] = RZ[(RZ_base + c_id) * 3 + 1];
		g_sliced[c_id * 3 + 2] = RZ[(RZ_base + c_id) * 3 + 2];

		// slice the generators, label the ones sliced to a point
		bool all = true;
		for(uint32_t i = 0; i < 2 * (link_id + 1); i++){
			uint8_t k_idx_res = k_idx[(i * n_time_steps + time_id) * RZ_length + c_id];
			uint8_t C_idx_res = C_idx[(i * n_time_steps + time_id) * RZ_length + c_id];

			if(k_idx_res == 2){
				for(uint32_t j = 0; j < 3; j++){
					g_sliced[c_id * 3 + j] *= shared_lambda[i];
				}
			}

			all &= (k_idx_res != 1) || (C_idx_res != 1);
		}
		
		sliced_to_pt[c_id] = all & c_idx[RZ_base + c_id];
	}

	__syncthreads();

	if(c_id == 0){ // find c_poly
		for(uint32_t i = 1; i < RZ_length; i++){
			if(sliced_to_pt[i]){
				c_poly[0] += g_sliced[i * 3    ];
				c_poly[1] += g_sliced[i * 3 + 1];
				c_poly[2] += g_sliced[i * 3 + 2];
			}
		}
	}
	else if(c_id == 1){ // find g_poly
		g_poly_num = 0;

		for(uint32_t i = 1; i < RZ_length; i++){
			if(!sliced_to_pt[i]){
				g_poly[g_poly_num * 3    ] = g_sliced[i * 3    ];
				g_poly[g_poly_num * 3 + 1] = g_sliced[i * 3 + 1];
				g_poly[g_poly_num * 3 + 2] = g_sliced[i * 3 + 2];
				g_poly_num++;
			}
		}

		// buffer using obstacle
		for(uint32_t i = 1; i < OZ_unit_length; i++){
			for(uint32_t j = 0; j < 3; j++){
				if(i == j + 1){
					g_poly[g_poly_num * 3 + j] = OZ[(obstacle_base + i) * 3 + j] + BUFFER_DIST / 2;
				}
				else{
					g_poly[g_poly_num * 3 + j] = 0;
				}
			}
			g_poly_num++;
		}
	}

	__syncthreads();

	// define Pb Pb = min(d+deltaD; -d+deltaD)
	double A_1 = A[A_base * 3    ];
	double A_2 = A[A_base * 3 + 1];
	double A_3 = A[A_base * 3 + 2];

	// generate d and deltaD
	double d, deltaD;
	if(A_1 == 0 && A_2 == 0 && A_3 == 0){ // invalid constraints
		d = 0;
		deltaD = A_BIG_NUMBER;
	}
	else{ // valid
		d = A_1 * c_poly[0] + A_2 * c_poly[1] + A_3 * c_poly[2];
		deltaD = 0;
		for (uint32_t i = 0; i < g_poly_num; i++) {
			deltaD += abs(A_1 * g_poly[i * 3] + A_2 * g_poly[i * 3 + 1] + A_3 * g_poly[i * 3 + 2]);
		}
	}

	double pos_res =  d + deltaD;
	double neg_res = -d + deltaD;

	if(pos_res > neg_res){
		con_res[c_id] = neg_res;
		index_factor[c_id] = false;
	}
	else{
		con_res[c_id] = pos_res;
		index_factor[c_id] = true;
	}

	__syncthreads();

	if(c_id >= 2 * (link_id + 1) + (link_id + 1) * ((link_id + 1) * 2 - 1)) return; // end the unused threads 
	
	__shared__ uint32_t min_idx;
	__shared__ double   minimum;
	__shared__ double   A_1_min;
	__shared__ double   A_2_min;
	__shared__ double   A_3_min;
	__shared__ double   A_factor;

	if(c_id == 0){ // find all the minimum in Pb, fill in con
		minimum = A_BIG_NUMBER + A_BIG_NUMBER;
		min_idx = 0;

		for(uint32_t i = 0; i < constraint_length; i++){
			if(con_res[i] < minimum){
				minimum = con_res[i];
				min_idx = i;
			}
		}

		con[valu_con_base] = minimum + CONSERVATIVE_BUFFER;

		A_1_min = A[((obstacle_id * n_time_steps + time_id) * constraint_length + min_idx) * 3    ];
		A_2_min = A[((obstacle_id * n_time_steps + time_id) * constraint_length + min_idx) * 3 + 1];
		A_3_min = A[((obstacle_id * n_time_steps + time_id) * constraint_length + min_idx) * 3 + 2];

		if(index_factor[min_idx]){
			A_factor = 1.0;
		}
		else{
			A_factor = -1.0;
		}
	}

	__syncthreads();

	if(c_id < 2 * (link_id + 1)){ // fill in jaco_con
		double result = 0;
		for(uint32_t i = 1; i < RZ_length; i++){
			if(k_idx[(c_id * n_time_steps + time_id) * RZ_length + i] == 2){ // lambda differentiated exists in this term
				if(sliced_to_pt[i]){ // for d, note +-d
					double prod = A_factor * (A_1_min * RZ[(RZ_base + i) * 3] + A_2_min * RZ[(RZ_base + i) * 3 + 1] + A_3_min * RZ[(RZ_base + i) * 3 + 2]);

					for(uint32_t j = 0; j < 2 * (link_id + 1); j++){
						if(j != c_id && k_idx[(j * n_time_steps + time_id) * RZ_length + i] == 2){
							prod *= shared_lambda[j];
						}
					}

					result += prod;
				}
				else{ // for deltaD
					double abs_value = A_1_min * g_sliced[i * 3] + A_2_min * g_sliced[i * 3 + 1] + A_3_min * g_sliced[i * 3 + 2];
					double sign;
					if(abs_value >= 0){
						sign = 1.0;
					}
					else{
						sign = -1.0;
					}
					double prod = sign * (A_1_min * RZ[(RZ_base + i) * 3] + A_2_min * RZ[(RZ_base + i) * 3 + 1] + A_3_min * RZ[(RZ_base + i) * 3 + 2]);

					for(uint32_t j = 0; j < 2 * (link_id + 1); j++){
						if(j != c_id && k_idx[(j * n_time_steps + time_id) * RZ_length + i] == 2){
							prod *= shared_lambda[j];
						}
					}

					result += prod;
				}
			}
		}

		jaco_con[jaco_con_base + c_id] = result / g_k[c_id];
	}
	else if(c_id < 2 * (link_id + 1) + (link_id + 1) * ((link_id + 1) * 2 - 1)){ // fill in hessian
		c_id -= 2 * (link_id + 1);
		uint32_t fir, sec;
		for(fir = 0; fir < 2 * (link_id + 1); fir++){
			if(fir * (fir + 1) / 2 <= c_id && c_id < (fir + 1) * (fir + 2) / 2){
				sec = c_id - fir * (fir + 1) / 2;
				break;
			}
		}
		fir++;

		double result = 0;
		for(uint32_t i = 1; i < RZ_length; i++){
			if(k_idx[(fir * n_time_steps + time_id) * RZ_length + i] == 2 && k_idx[(sec * n_time_steps + time_id) * RZ_length + i] == 2){ // lambda differentiated exists in this term
				if(sliced_to_pt[i]){ // for d, note +-d
					double prod = A_factor * (A_1_min * RZ[(RZ_base + i) * 3] + A_2_min * RZ[(RZ_base + i) * 3 + 1] + A_3_min * RZ[(RZ_base + i) * 3 + 2]);

					for(uint32_t j = 0; j < 2 * (link_id + 1); j++){
						if(j != fir && j != sec && k_idx[(j * n_time_steps + time_id) * RZ_length + i] == 2){
							prod *= shared_lambda[j];
						}
					}

					result += prod;
				}
				else{ // for deltaD
					double abs_value = A_1_min * g_sliced[i * 3] + A_2_min * g_sliced[i * 3 + 1] + A_3_min * g_sliced[i * 3 + 2];
					double sign;
					if(abs_value >= 0){
						sign = 1.0;
					}
					else{
						sign = -1.0;
					}
					double prod = sign * (A_1_min * RZ[(RZ_base + i) * 3] + A_2_min * RZ[(RZ_base + i) * 3 + 1] + A_3_min * RZ[(RZ_base + i) * 3 + 2]);

					for(uint32_t j = 0; j < 2 * (link_id + 1); j++){
						if(j != fir && j != sec && k_idx[(j * n_time_steps + time_id) * RZ_length + i] == 2){
							prod *= shared_lambda[j];
						}
					}

					result += prod;
				}
			}
		}

		hess_con[hess_con_base + c_id] = result / g_k[fir] / g_k[sec];
	}
}

rotatotopeArray::~rotatotopeArray() {
	cudaFree(dev_Z);

	if (n_links > 0) {
		cudaFree(dev_RZ);
		cudaFree(dev_c_idx);
		cudaFree(dev_k_idx);
		cudaFree(dev_C_idx);
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

		for (uint32_t i = 0; i < n_links; i++) {
			delete[] C_idx_stack[i];
		}
		delete[] C_idx_stack;

		for (uint32_t i = 0; i < n_links; i++) {
			cudaFree(dev_C_idx_stack[i]);
		}
		delete[] dev_C_idx_stack;

		delete[] RZ_length;
	}

	if (n_obstacles > 0 && A != nullptr) {
		for (uint32_t i = 0; i < n_links; i++) {
			delete[] A[i];
		}
		delete[] A;

		for (uint32_t i = 0; i < n_links; i++) {
			cudaFree(dev_A[i]);
		}
		delete[] dev_A;

		cudaFree(dev_OZ);
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

