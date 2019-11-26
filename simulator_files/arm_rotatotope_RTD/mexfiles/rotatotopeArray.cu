/*
Author: Bohao Zhang
Oct. 29 2019

arm_planning mex

a cuda array for a cluster of rotatotopes
*/

#ifndef ROTATOTOPE_ARRAY_CPP
#define ROTATOTOPE_ARRAY_CPP

#include "rotatotopeArray.h"

rotatotopeArray::rotatotopeArray(uint32_t n_links_input, uint32_t n_time_steps_input, double* &R_input, double* &dev_R_input, uint32_t R_unit_length_input, uint8_t* &dev_rot_axes_input, double* &Z_input, uint32_t Z_width_input, uint32_t Z_length_input) {
	n_links = n_links_input;
	n_time_steps = n_time_steps_input;
	dev_R = dev_R_input;
	R_unit_length = R_unit_length_input;
	dev_rot_axes = dev_rot_axes_input;

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
			for (uint32_t R_id = R_id_start + 1; R_id < R_id_start + R_unit_length; R_id++) {
				if (R_input[R_id * 5 + k_dim] != 0) {
					g_k[joint_id] = R_input[R_id * 5 + k_dim];
					break;
				}
			}
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
		initialize_RZ_kernel << < grid1, block1 >> > (dev_Z, Z_unit_length, dev_RZ, dev_c_idx);

		for (int link = n_links; link > 0; link--) {
			for (int joint_offset = 1; joint_offset >= 0; joint_offset--) {
				dim3 grid2(link, n_time_steps, 1);
				dim3 block2(reduce_order, R_unit_length, 1);
				multiply_kernel << < grid2, block2 >> > (dev_rot_axes, n_links - link, joint_offset, dev_RZ, dev_R, dev_c_idx, dev_k_idx, dev_RZ_new, dev_c_idx_new, dev_k_idx_new);

				reduce_kernel << < grid2, (reduce_order * R_unit_length) >> > (dev_RZ_new, dev_c_idx_new, dev_k_idx_new, n_links - link, dev_RZ, dev_c_idx, dev_k_idx);
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

	dev_RZ_stack = nullptr;
	dev_c_idx_stack = nullptr;
	dev_k_idx_stack = nullptr;

	n_obstacles = 0;
	A_con = nullptr;
	dev_A_con = nullptr;
	b_con = nullptr;
	dev_b_con = nullptr;
	k_con = nullptr;
	dev_k_con = nullptr;
	k_con_num = nullptr;
	dev_k_con_num = nullptr;
	max_k_con_num = nullptr;
}

__global__ void initialize_RZ_kernel(double* link_Z, uint32_t link_Z_length, double* RZ, bool* c_idx) {
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

__global__ void multiply_kernel(uint8_t* rot_axes, uint32_t link_offset, uint32_t joint_offset, double* RZ, double* R, bool* c_idx, bool* k_idx, double* RZ_new, bool* c_idx_new, bool* k_idx_new) {
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

__global__ void reduce_kernel(double* RZ_new, bool* c_idx_new, bool* k_idx_new, uint32_t link_offset, double* RZ, bool* c_idx, bool* k_idx) {
	uint32_t link_id = blockIdx.x + link_offset;
	uint32_t time_id = blockIdx.y;
	uint32_t n_time_steps = gridDim.y;
	uint32_t z_id = threadIdx.x;
	uint32_t norm_length = blockDim.x;
	uint32_t mul_Z = (link_id * n_time_steps + time_id) * norm_length + z_id; // we never reduce the center
	__shared__ double RZ_norm[norm_size];
	__shared__ uint32_t RZ_id[norm_size];

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

void rotatotopeArray::stack(rotatotopeArray &EEs) {
	dev_RZ_stack = new double*[n_links];
	dev_c_idx_stack = new bool*[n_links];
	dev_k_idx_stack = new bool*[n_links];

	for (uint32_t link_id = 0; link_id < n_links; link_id++) {
		uint32_t RZ_length = ((reduce_order - 1) * (link_id + 1) + 1);
		cudaMalloc((void**)&(dev_RZ_stack[link_id]), n_time_steps * RZ_length * Z_width * sizeof(double));
		cudaMalloc((void**)&(dev_c_idx_stack[link_id]), n_time_steps * RZ_length * sizeof(bool));
		cudaMalloc((void**)&(dev_k_idx_stack[link_id]), 2 * (link_id + 1) * n_time_steps * RZ_length * sizeof(bool));
		cudaMemset(dev_k_idx_stack, 0, 2 * (link_id + 1) * n_time_steps * RZ_length * sizeof(bool));

		// copy dev_RZ to dev_RZ_stack
		dim3 grid1(n_time_steps, 1, 1);
		dim3 block1(reduce_order, Z_width, 1);
		copy_kernel << < grid1, block1 >> > (link_id, dev_RZ, dev_c_idx, dev_k_idx, dev_RZ_stack[link_id], dev_c_idx_stack[link_id], dev_k_idx_stack[link_id]);

		// stack
		for (int EE_id = link_id - 1; EE_id >= 0; EE_id--) {
			dim3 grid2(n_time_steps, 1, 1);
			dim3 block2(reduce_order, Z_width, 1);
			stack_kernel << < grid2, block2 >> > (link_id, EE_id, dev_RZ_stack[link_id], EEs.dev_RZ, dev_c_idx_stack[link_id], EEs.dev_c_idx, dev_k_idx_stack[link_id], EEs.dev_k_idx);
		}

		if (link_id == 2) {
			debug_RZ = new double[n_time_steps * RZ_length * Z_width];
			cudaMemcpy(debug_RZ, dev_RZ_stack[link_id], n_time_steps * RZ_length * Z_width * sizeof(double), cudaMemcpyDeviceToHost);

			debug_c_idx = new bool[n_time_steps * RZ_length];
			cudaMemcpy(debug_c_idx, dev_c_idx_stack[link_id], n_time_steps * RZ_length * sizeof(bool), cudaMemcpyDeviceToHost);

			debug_k_idx = new bool[2 * (link_id + 1) * n_time_steps * RZ_length];
			cudaMemcpy(debug_k_idx, dev_k_idx_stack[link_id], 2 * (link_id + 1) * n_time_steps * RZ_length * sizeof(bool), cudaMemcpyDeviceToHost);
		}
	}
}

__global__ void copy_kernel(uint32_t link_id, double* RZ, bool* c_idx, bool* k_idx, double* RZ_stack, bool* c_idx_stack, bool* k_idx_stack) {
	uint32_t time_id = blockIdx.x;
	uint32_t n_time_steps = gridDim.x;
	uint32_t Z_id = threadIdx.x;
	uint32_t z_id = threadIdx.y;
	
	uint32_t RZ_length = ((reduce_order - 1) * (link_id + 1) + 1);
	uint32_t copy_Z = time_id * RZ_length + Z_id;
	uint32_t copy_k_start = time_id * RZ_length + Z_id;
	//uint32_t copy_k_end = (2 * (link_id + 1) * n_time_steps + time_id) * RZ_length + Z_id;
	uint32_t copy_k_step = n_time_steps * RZ_length;
	uint32_t link_Z = (link_id * n_time_steps + time_id) * reduce_order + Z_id;
	uint32_t link_k_start = ((link_id * (link_id + 1)) * n_time_steps + time_id) * reduce_order + Z_id;
	uint32_t link_k_end = (((link_id + 1) * (link_id + 2)) * n_time_steps + time_id) * reduce_order + Z_id;
	uint32_t link_k_step = n_time_steps * reduce_order;

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

__global__ void stack_kernel(uint32_t link_id, uint32_t EE_id, double* RZ_stack, double* EE_RZ, bool* c_idx_stack, bool* EE_c_idx, bool* k_idx_stack, bool* EE_k_idx) {
	uint32_t time_id = blockIdx.x;
	uint32_t n_time_steps = gridDim.x;
	uint32_t Z_id = threadIdx.x;
	uint32_t z_id = threadIdx.y;

	uint32_t RZ_length = ((reduce_order - 1) * (link_id + 1) + 1);
	uint32_t stack_Z = time_id * RZ_length + Z_id;
	uint32_t stack_k_start = time_id * RZ_length + Z_id;
	uint32_t stack_k_end = (2 * (link_id + 1) * n_time_steps + time_id) * RZ_length + Z_id;
	uint32_t stack_k_step = n_time_steps * RZ_length;
	uint32_t EE_Z = (EE_id * n_time_steps + time_id) * reduce_order + Z_id;
	uint32_t EE_k_start = ((EE_id * (EE_id + 1)) * n_time_steps + time_id) * reduce_order + Z_id;
	uint32_t EE_k_end = (((EE_id + 1) * (EE_id + 2)) * n_time_steps + time_id) * reduce_order + Z_id;
	uint32_t EE_k_step = n_time_steps * reduce_order;

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
		RZ_stack[(stack_Z + (EE_id + 1) * (reduce_order - 1)) * 3 + z_id] = EE_RZ[EE_Z * 3 + z_id];

		if (z_id == 0) {
			c_idx_stack[stack_Z + (EE_id + 1) * (reduce_order - 1)] = EE_c_idx[EE_Z];

			uint32_t EE_k = EE_k_start;
			for (uint32_t stack_k = stack_k_start + (EE_id + 1) * (reduce_order - 1); stack_k < stack_k_end + (EE_id + 1) * (reduce_order - 1); stack_k += stack_k_step) {
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

void rotatotopeArray::generate_constraints(uint32_t n_obstacles_in, double* OZ, uint32_t OZ_width, uint32_t OZ_length) {
	n_obstacles = n_obstacles_in;
	uint32_t OZ_unit_length = OZ_length / n_obstacles;

	double* dev_OZ;
	cudaMalloc((void**)&dev_OZ, OZ_length * OZ_width * sizeof(double));
	cudaMemcpy(dev_OZ, OZ, OZ_length * OZ_width * sizeof(double), cudaMemcpyHostToDevice);

	A_con = new double*[n_links];
	dev_A_con = new double*[n_links];
	b_con = new double*[n_links];
	dev_b_con = new double*[n_links];
	k_con = new bool*[n_links];
	dev_k_con = new bool*[n_links];
	k_con_num = new uint8_t*[n_links];
	dev_k_con_num = new uint8_t*[n_links];
	max_k_con_num = new uint32_t[n_links];

	for (uint32_t link_id = 0; link_id < n_links; link_id++) {
		uint32_t RZ_length = ((reduce_order - 1) * (link_id + 1) + 1);
		uint32_t buff_obstacle_length = RZ_length + 3;
		uint32_t constraint_length = ((buff_obstacle_length - 1) * (buff_obstacle_length - 2)) / 2;

		// buffer the obstacle by k-independent generators
		k_con[link_id] = new bool[2 * (link_id + 1) * n_time_steps * RZ_length];
		cudaMalloc((void**)&(dev_k_con[link_id]), 2 * (link_id + 1) * n_time_steps * RZ_length * sizeof(bool));
		k_con_num[link_id] = new uint8_t[n_time_steps];
		cudaMalloc((void**)&(dev_k_con_num[link_id]), n_time_steps * sizeof(uint8_t));

		double* dev_buff_obstacles;
		cudaMalloc((void**)&dev_buff_obstacles, n_obstacles * n_time_steps * buff_obstacle_length * 3 * sizeof(double));
		cudaMemset(dev_buff_obstacles, 0, n_obstacles * n_time_steps * buff_obstacle_length * 3 * sizeof(double));

		double* dev_frs_k_dep_G;
		cudaMalloc((void**)&dev_frs_k_dep_G, n_time_steps * RZ_length * 3 * sizeof(double));
		cudaMemset(dev_frs_k_dep_G, 0, n_time_steps * RZ_length * 3 * sizeof(double));

		dim3 grid1(n_obstacles, n_time_steps, 1);
		buff_obstacles_kernel << < grid1, RZ_length >> > (link_id, dev_RZ_stack[link_id], dev_c_idx_stack[link_id], dev_k_idx_stack[link_id], dev_OZ, OZ_unit_length, dev_buff_obstacles, dev_frs_k_dep_G, dev_k_con[link_id], dev_k_con_num[link_id]);

		//cudaMemcpy(k_con[link_id], dev_k_con[link_id], 2 * (link_id + 1) * n_time_steps * RZ_length * sizeof(bool), cudaMemcpyDeviceToHost);
		cudaMemcpy(k_con_num[link_id], dev_k_con_num[link_id], n_time_steps * sizeof(uint8_t), cudaMemcpyDeviceToHost);

		// find the maximum width of A_con for memory allocation
		max_k_con_num[link_id] = 0;
		for (uint32_t i = 0; i < n_time_steps; i++) {
			if (k_con_num[link_id][i] > max_k_con_num[link_id]) {
				max_k_con_num[link_id] = k_con_num[link_id][i];
			}
		}

		// generate obstacles polynomials
		A_con[link_id] = new double[n_obstacles * n_time_steps * constraint_length * 2 * max_k_con_num[link_id]];
		cudaMalloc((void**)&(dev_A_con[link_id]), n_obstacles * n_time_steps * constraint_length * 2 * max_k_con_num[link_id] * sizeof(double));

		b_con[link_id] = new double[n_obstacles * n_time_steps * constraint_length * 2];
		cudaMalloc((void**)&(dev_b_con[link_id]), n_obstacles * n_time_steps * constraint_length * 2 * sizeof(double));
		
		dim3 grid2(n_obstacles, constraint_length, 1);
		polytope << < grid2, n_time_steps >> > (link_id, dev_buff_obstacles, dev_frs_k_dep_G, dev_k_con_num[link_id], max_k_con_num[link_id], dev_A_con[link_id], dev_b_con[link_id]);

		//cudaMemcpy(A_con[link_id], dev_A_con[link_id], n_obstacles * n_time_steps * constraint_length * 2 * max_k_con_num[link_id] * sizeof(double), cudaMemcpyDeviceToHost);
		//cudaMemcpy(b_con[link_id], dev_b_con[link_id], n_obstacles * n_time_steps * constraint_length * 2 * sizeof(double), cudaMemcpyDeviceToHost);

		cudaFree(dev_buff_obstacles);
		cudaFree(dev_frs_k_dep_G);
	}

	cudaFree(dev_OZ);
}

__global__ void buff_obstacles_kernel(uint32_t link_id, double* RZ, bool* c_idx, bool* k_idx, double* OZ, uint32_t OZ_unit_length, double* buff_obstacles, double* frs_k_dep_G, bool* k_con, uint8_t* k_con_num) {
	uint32_t obstacle_id = blockIdx.x;
	uint32_t obstacle_base = obstacle_id * OZ_unit_length;
	uint32_t time_id = blockIdx.y;
	uint32_t n_time_steps = gridDim.y;
	uint32_t z_id = threadIdx.x; 
	uint32_t RZ_length = ((reduce_order - 1) * (link_id + 1) + 1);
	uint32_t buff_obstacle_length = RZ_length + 3;
	uint32_t RZ_base = time_id * RZ_length;
	uint32_t k_start = time_id * RZ_length;
	uint32_t k_end = (2 * (link_id + 1) * n_time_steps + time_id) * RZ_length;
	uint32_t k_step = n_time_steps * RZ_length;
	uint32_t k_con_num_base = time_id;
	uint32_t buff_base = (obstacle_id * n_time_steps + time_id) * buff_obstacle_length;

	// first, find kc_col
	__shared__ bool kc_info[max_RZ_length];

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
		for (uint32_t z = 1; z < RZ_length; z++) {
			if (!kc_info[z]) {
				for (uint32_t i = 0; i < 3; i++) {
					buff_obstacles[(buff_base + k_indep_num) * 3 + i] = RZ[(RZ_base + z) * 3 + i];
				}
				k_indep_num++;
			}
		}
	}
}

__global__ void polytope(uint32_t link_id, double* buff_obstacles, double* frs_k_dep_G, uint8_t* k_con_num, uint32_t A_con_width, double* A_con, double* b_con) {
	uint32_t obstacle_id = blockIdx.x;
	uint32_t time_id = threadIdx.x;
	uint32_t n_time_steps = blockDim.x;
	uint32_t RZ_length = ((reduce_order - 1) * (link_id + 1) + 1);
	uint32_t buff_obstacle_length = RZ_length + 3;
	double buff_obstacle_size = (double)buff_obstacle_length - 1.0;
	uint32_t constraint_length = (buff_obstacle_length - 1) * (buff_obstacle_length - 2) / 2;
	uint32_t k_con_base = time_id;
	uint32_t k_dep_G_base = k_con_base * RZ_length;
	uint32_t obs_base = (obstacle_id * n_time_steps + time_id) * buff_obstacle_length;
	uint32_t c_id = blockIdx.y;
	uint32_t first = (uint32_t)floor(-0.5*sqrt(4 * buff_obstacle_size * buff_obstacle_size - 4 * buff_obstacle_size - 8.0 * ((double)c_id) + 1.0) + buff_obstacle_size - 0.5);
	uint32_t first_base = (obs_base + first + 1) * 3;
	uint32_t second = c_id + 1 - ((2 * (buff_obstacle_length - 1) - 3 - first) * first) / 2;
	uint32_t second_base = (obs_base + second + 1) * 3;
	uint32_t con_base = (obstacle_id * n_time_steps + time_id) * constraint_length * 2 + c_id;

	double A_1 = buff_obstacles[first_base + 1] * buff_obstacles[second_base + 2] - buff_obstacles[first_base + 2] * buff_obstacles[second_base + 1];
	double A_2 = buff_obstacles[first_base + 2] * buff_obstacles[second_base] - buff_obstacles[first_base] * buff_obstacles[second_base + 2];
	double A_3 = buff_obstacles[first_base] * buff_obstacles[second_base + 1] - buff_obstacles[first_base + 1] * buff_obstacles[second_base];
	
	if (A_1 != 0 || A_2 != 0 || A_3 != 0) {
		double A_s_q = sqrt(A_1 * A_1 + A_2 * A_2 + A_3 * A_3);
		A_1 /= A_s_q;
		A_2 /= A_s_q;
		A_3 /= A_s_q;
	}

	for (uint32_t i = 0; i < k_con_num[k_con_base]; i++) {
		A_con[con_base * A_con_width + i] = A_1 * frs_k_dep_G[(k_dep_G_base + i) * 3] + A_2 * frs_k_dep_G[(k_dep_G_base + i) * 3 + 1] + A_3 * frs_k_dep_G[(k_dep_G_base + i) * 3 + 2];
		A_con[(con_base + constraint_length) * A_con_width + i] = -A_con[con_base * A_con_width + i];
	}

	double d = A_1 * buff_obstacles[obs_base * 3] + A_2 * buff_obstacles[obs_base * 3 + 1] + A_3 * buff_obstacles[obs_base * 3 + 2];

	double deltaD = 0;
	for (uint32_t i = 1; i < buff_obstacle_length - k_con_num[k_con_base]; i++) {
		deltaD += abs(A_1 * buff_obstacles[(obs_base + i) * 3] + A_2 * buff_obstacles[(obs_base + i) * 3 + 1] + A_3 * buff_obstacles[(obs_base + i) * 3 + 2]);
	}

	if (A_1 != 0 || A_2 != 0 || A_3 != 0) {
		b_con[con_base] = d + deltaD;
		b_con[con_base + constraint_length] = -d + deltaD;
	}
	else {
		b_con[con_base] = A_BIG_NUMBER;
		b_con[con_base + constraint_length] = A_BIG_NUMBER;
	}
}

void rotatotopeArray::evaluate_constraints(double* k_opt, double* &con, double* &grad_con) {
	con = new double [n_links * n_obstacles * n_time_steps];
	double* dev_con;
	cudaMalloc((void**)&dev_con, n_links * n_obstacles * n_time_steps * sizeof(double));

	double* lambda = new double[n_links * 2];
	for (uint32_t joint_id = 0; joint_id < n_links * 2; joint_id++) {
		lambda[joint_id] = c_k[joint_id] + k_opt[joint_id] / g_k[joint_id];
	}

	double* dev_lambda;
	cudaMalloc((void**)&dev_lambda, n_links * 2 * sizeof(double));
	cudaMemcpy(dev_lambda, lambda, n_links * 2 * sizeof(double), cudaMemcpyHostToDevice);

	for (uint32_t link_id = 0; link_id < n_links; link_id++) {
		uint32_t RZ_length = ((reduce_order - 1) * (link_id + 1) + 1);
		uint32_t buff_obstacle_length = RZ_length + 3;
		uint32_t constraint_length = ((buff_obstacle_length - 1) * (buff_obstacle_length - 2));

		double* dev_con_result; // results of evaluation of constriants
		cudaMalloc((void**)&dev_con_result, n_obstacles * n_time_steps * constraint_length * sizeof(double));
		
		dim3 grid1(n_obstacles, constraint_length, 1);
		evaluate_constraints_kernel << < grid1, n_time_steps >> > (dev_lambda, link_id, dev_A_con[link_id], max_k_con_num[link_id], dev_b_con[link_id], dev_k_con[link_id], dev_k_con_num[link_id], dev_con_result);
		
		dim3 block2(n_obstacles, n_time_steps, 1);
		find_max_kernel << < 1, block2 >> > (dev_con_result, link_id, dev_con);

		cudaFree(dev_con_result);
	}

	cudaMemcpy(con, dev_con, n_links * n_obstacles * n_time_steps * sizeof(double), cudaMemcpyDeviceToHost);

	cudaFree(dev_con);

	delete[] lambda;
	cudaFree(dev_lambda);
}

__global__ void evaluate_constraints_kernel(double* lambda, uint32_t link_id, double* A_con, uint32_t A_con_width, double* b_con, bool* k_con, uint8_t* k_con_num, double* con_result) {
	uint32_t obstacle_id = blockIdx.x;
	uint32_t c_id = blockIdx.y;
	uint32_t constraint_length = gridDim.y;
	uint32_t time_id = threadIdx.x;
	uint32_t n_time_steps = blockDim.x;
	uint32_t RZ_length = ((reduce_order - 1) * (link_id + 1) + 1);
	uint32_t k_con_num_base = time_id;
	uint32_t con_base = (obstacle_id * n_time_steps + time_id) * constraint_length + c_id;

	double result = 0;
	for (uint32_t p = 0; p < k_con_num[k_con_num_base]; p++){
		double prod = 1.0;
		for (uint32_t j = 0; j < 2 * (link_id + 1); j++) {
			if (k_con[(j * n_time_steps + time_id) * RZ_length + p]) {
				prod *= lambda[j];
			}
		}

		result += prod * A_con[con_base * A_con_width + p];
	}

	con_result[con_base] = result - b_con[con_base];
}

__global__ void find_max_kernel(double* con_result, uint32_t link_id, double* con) {
	uint32_t obstacle_id = threadIdx.x;
	uint32_t n_obstacles = blockDim.x;
	uint32_t time_id = threadIdx.y;
	uint32_t n_time_steps = blockDim.y;
	
	uint32_t RZ_length = ((reduce_order - 1) * (link_id + 1) + 1);
	uint32_t buff_obstacle_length = RZ_length + 3;
	uint32_t constraint_length = ((buff_obstacle_length - 1) * (buff_obstacle_length - 2));
	uint32_t con_result_base = (obstacle_id * n_time_steps + time_id) * constraint_length;
	uint32_t con_base = (link_id * n_obstacles + obstacle_id) * n_time_steps + time_id;

	double maximum = FLT_MIN;
	for (uint32_t i = 0; i < constraint_length; i++) {
		if (maximum < con_result[con_result_base + i]) {
			maximum = con_result[con_result_base + i];
		}
	}
	
	con[con_base] = -maximum;
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
			cudaFree(dev_RZ_stack[i]);
		}
		delete[] dev_RZ_stack;

		for (uint32_t i = 0; i < n_links; i++) {
			cudaFree(dev_c_idx_stack[i]);
		}
		delete[] dev_c_idx_stack;

		for (uint32_t i = 0; i < n_links; i++) {
			cudaFree(dev_k_idx_stack[i]);
		}
		delete[] dev_k_idx_stack;
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
			delete[] b_con[i];
		}
		delete[] b_con;

		for (uint32_t i = 0; i < n_links; i++) {
			cudaFree(dev_b_con[i]);
		}
		delete[] dev_b_con;

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

	if (debug_RZ != nullptr) {
		delete[] debug_RZ;
		delete[] debug_c_idx;
		delete[] debug_k_idx;
	}

	if (debug != nullptr) {
		delete[] debug;
		delete[] debug_2;
	}
}

#endif // !ROTATOTOPE_ARRAY_CPPs

