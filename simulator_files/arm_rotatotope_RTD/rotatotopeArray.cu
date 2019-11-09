/*
Author: Bohao Zhang
Oct. 29 2019

arm_planning mex

a cuda array for a cluster of rotatotopes
*/

#ifndef ROTATOTOPE_ARRAY_CPP
#define ROTATOTOPE_ARRAY_CPP

#include "rotatotopeArray.h"

rotatotopeArray::rotatotopeArray(uint32_t n_links_input, uint32_t n_time_steps_input, double* &dev_R_input, uint32_t R_unit_length_input, uint8_t* &dev_rot_axes_input, double* &Z_input, uint32_t Z_width_input, uint32_t Z_length_input) {
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

		RZ = new double[n_links * n_time_steps * reduce_order * Z_width]; // compute RZ iteratively
		cudaMalloc((void**)&dev_RZ, n_links * n_time_steps * reduce_order * Z_width * sizeof(double));
		cudaMalloc((void**)&dev_RZ_new, n_links * n_time_steps * reduce_order * R_unit_length * Z_width * sizeof(double));

		c_idx = new bool[n_links * n_time_steps * reduce_order]; // compute c_idx and k_idx iteratively
		k_idx = new bool[n_links * (n_links + 1) * n_time_steps * reduce_order];
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
}

rotatotopeArray::~rotatotopeArray() {
	cudaFree(dev_Z);

	if (n_links > 0) {
		delete[] RZ;
		cudaFree(dev_RZ);
		delete[] c_idx;
		cudaFree(dev_c_idx);
		delete[] k_idx;
		cudaFree(dev_k_idx);
	}
}

void rotatotopeArray::stack(rotatotopeArray &EEs) {
	double* dev_RZ_new;
	cudaMalloc((void**)&dev_RZ_new, n_links * n_time_steps * (reduce_order * 2 - 1) * Z_width * sizeof(double));

	RZ_new = new double[n_links * n_time_steps * (reduce_order * 2 - 1) * Z_width];

	bool *dev_c_idx_new, *dev_k_idx_new;
	cudaMalloc((void**)&dev_c_idx_new, n_links * n_time_steps * (reduce_order * 2 - 1) * sizeof(bool));
	cudaMalloc((void**)&dev_k_idx_new, n_links * (n_links + 1) * n_time_steps * (reduce_order * 2 - 1) * sizeof(bool));
	cudaMemset(dev_k_idx_new, 0, n_links * (n_links + 1) * n_time_steps * (reduce_order * 2 - 1) * sizeof(bool));

	for (int link = EEs.n_links; link > 0; link--) {
		dim3 grid1(link, n_time_steps, 1);
		dim3 block1(reduce_order, Z_width, 1);
		add_kernel << < grid1, block1 >> > (n_links - link, dev_RZ, EEs.dev_RZ, dev_c_idx, EEs.dev_c_idx, dev_k_idx, EEs.dev_k_idx, dev_RZ_new, dev_c_idx_new, dev_k_idx_new);
	
		reduce_kernel << < grid1, (2 * reduce_order - 1) >> > (dev_RZ_new, dev_c_idx_new, dev_k_idx_new, n_links - link, dev_RZ, dev_c_idx, dev_k_idx);
	}

	cudaMemcpy(RZ_new, dev_RZ_new, n_links * n_time_steps * (reduce_order * 2 - 1) * Z_width * sizeof(double), cudaMemcpyDeviceToHost);

	cudaFree(dev_RZ_new);
	cudaFree(dev_c_idx_new);
	cudaFree(dev_k_idx_new);
}

void rotatotopeArray::returnResults() {
	if (n_links > 0) {
		cudaMemcpy(RZ, dev_RZ, n_links * n_time_steps * reduce_order * Z_width * sizeof(double), cudaMemcpyDeviceToHost);
		cudaMemcpy(c_idx, dev_c_idx, n_links * n_time_steps * reduce_order * sizeof(bool), cudaMemcpyDeviceToHost);
		cudaMemcpy(k_idx, dev_k_idx, n_links * (n_links + 1) * n_time_steps * reduce_order * sizeof(bool), cudaMemcpyDeviceToHost);
	}
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

__global__ void add_kernel(uint32_t link_offset, double* link_RZ, double* EE_RZ, bool* link_c_idx, bool* EE_c_idx, bool* link_k_idx, bool* EE_k_idx, double* RZ_new, bool* c_idx_new, bool* k_idx_new) {
	uint32_t link_id = blockIdx.x + link_offset;
	uint32_t time_id = blockIdx.y;
	uint32_t n_time_steps = gridDim.y;
	uint32_t Z_id = threadIdx.x;
	uint32_t z_id = threadIdx.y;

	uint32_t add_link_Z = (link_id * n_time_steps + time_id) * reduce_order + Z_id;
	uint32_t add_EE_Z = (blockIdx.x * n_time_steps + time_id) * reduce_order + Z_id;
	uint32_t add_Z = (link_id * n_time_steps + time_id) * (2 * reduce_order - 1) + Z_id;

	uint32_t EE_k_start = ((blockIdx.x * (blockIdx.x + 1)) * n_time_steps + time_id) * reduce_order + Z_id;
	uint32_t EE_k_end = (((blockIdx.x + 1) * (blockIdx.x + 2)) * n_time_steps + time_id) * reduce_order + Z_id;
	uint32_t link_k_start = ((link_id * (link_id + 1)) * n_time_steps + time_id) * reduce_order + Z_id;
	uint32_t link_k_end = (((link_id + 1) * (link_id + 2)) * n_time_steps + time_id) * reduce_order + Z_id;
	uint32_t k_step = n_time_steps * reduce_order;
	uint32_t add_k_start = ((link_id * (link_id + 1)) * n_time_steps + time_id) * (2 * reduce_order - 1)+ Z_id;
	uint32_t add_k_step = n_time_steps * (2 * reduce_order - 1);

	if (Z_id == 0) { // add the center
		RZ_new[add_Z * 3 + z_id] = link_RZ[add_link_Z * 3 + z_id] + EE_RZ[add_EE_Z * 3 + z_id];

		if (z_id == 0) {
			c_idx_new[add_Z] = true;

			uint32_t add_k = add_k_start, EE_k = EE_k_start;
			for (uint32_t link_k = link_k_start; link_k < link_k_end; link_k += k_step) {
				if (EE_k < EE_k_end) {
					k_idx_new[add_k] = link_k_idx[link_k] | EE_k_idx[EE_k];
				}
				else {
					k_idx_new[add_k] = link_k_idx[link_k];
				}

				add_k += add_k_step;
				EE_k += k_step;
			}
		}
	}
	else { // stack the generators
		RZ_new[add_Z * 3 + z_id] = link_RZ[add_link_Z * 3 + z_id];
		RZ_new[(add_Z + reduce_order - 1) * 3 + z_id] = EE_RZ[add_EE_Z * 3 + z_id];

		if (z_id == 0) {
			c_idx_new[add_Z] = link_c_idx[add_Z];
			c_idx_new[add_Z + reduce_order - 1] = EE_c_idx[add_EE_Z];

			uint32_t add_k = add_k_start, EE_k = EE_k_start;
			for (uint32_t link_k = link_k_start; link_k < link_k_end; link_k += k_step) {
				k_idx_new[add_k] = link_k_idx[link_k];

				if (EE_k < EE_k_end) {
					k_idx_new[add_k + reduce_order - 1] = true;//EE_k_idx[EE_k];
				}
				else {
					k_idx_new[add_k + reduce_order - 1] = false;
				}

				add_k += add_k_step;
				EE_k += k_step;
			}
		}
	}
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
		RZ_new[mul_RZ * 3    ] = if_center ? RZ[mul_Z * 3] : 0;
		RZ_new[mul_RZ * 3 + 1] = R[mul_R * 5] * RZ[mul_Z * 3 + 1] - R[mul_R * 5 + 1] * RZ[mul_Z * 3 + 2];
		RZ_new[mul_RZ * 3 + 2] = R[mul_R * 5 + 1] * RZ[mul_Z * 3 + 1] + R[mul_R * 5] * RZ[mul_Z * 3 + 2];
	}
	else if (rot_axis == 2) {
		RZ_new[mul_RZ * 3    ] = R[mul_R * 5] * RZ[mul_Z * 3] + R[mul_R * 5 + 1] * RZ[mul_Z * 3 + 2];
		RZ_new[mul_RZ * 3 + 1] = if_center ? RZ[mul_Z * 3 + 1] : 0;
		RZ_new[mul_RZ * 3 + 2] = R[mul_R * 5] * RZ[mul_Z * 3 + 2] - R[mul_R * 5 + 1] * RZ[mul_Z * 3];
	}
	else {
		RZ_new[mul_RZ * 3    ] = R[mul_R * 5] * RZ[mul_Z * 3] - R[mul_R * 5 + 1] * RZ[mul_Z * 3 + 1];
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

	RZ_norm[z_id] = 0;
	double norm;
	for (uint32_t i = 0; i < 3; i++) {
		norm = RZ_new[mul_Z * 3 + i];
		RZ_norm[z_id] += norm * norm;
	}

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
				if (i < j)
					swap(RZ_norm, RZ_new, c_idx_new, k_idx_new, base, k_start, k_end, k_step, (i++), (j--));
			}
			swap(RZ_norm, RZ_new, c_idx_new, k_idx_new, base, k_start, k_end, k_step, low, j);

			if (j == k - 1)
				break;
			else if (j < k - 1)
				low = j + 1;
			else
				high = j;
		}
	}
	
	// at this point, the first (reduce_order - 3) entries in RZ_new are the (reduce_order - 3) largest ones
	// we choose them as entries for RZ after reduction.
	// we compress the rest of the entries to a box with 3 generators

	__syncthreads();

	uint32_t base_ori = (link_id * n_time_steps + time_id) * reduce_order; // indeces offset for RZ
	uint32_t k_start_ori = ((link_id * (link_id + 1)) * n_time_steps + time_id) * reduce_order;
	uint32_t k_end_ori = (((link_id + 1) * (link_id + 2)) * n_time_steps + time_id) * reduce_order;
	uint32_t k_step_ori = n_time_steps * reduce_order;

	if (z_id < reduce_order - 3) { // copy these generators to RZ
		c_idx[base_ori + z_id] = c_idx_new[base + z_id];

		for (uint32_t h = 0; h < 3; h++) {
			RZ[(base_ori + z_id) * 3 + h] = RZ_new[(base + z_id) * 3 + h];
		}

		uint32_t k_pivot = k_start, k_pivot_ori = k_start_ori;
		while (k_pivot != k_end && k_pivot_ori != k_end_ori) {
			k_idx[k_pivot_ori + z_id] = k_idx_new[k_pivot + z_id];
			k_pivot += k_step;
			k_pivot_ori += k_step_ori;
		}
	}
	else if (reduce_order - 3 <= z_id && z_id < reduce_order) { // construct a 3-d box for the rest of the generators
		uint32_t box_id = (z_id + 3) - reduce_order;
		double entry_sum = 0;
		for (uint32_t h = (base + reduce_order - 3) * 3 + box_id; h < (base + norm_length) * 3 + box_id; h += 3) {
			entry_sum += RZ_new[h];
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

__device__ void swap(double* RZ_norm, double* RZ_new, bool* c_idx_new, bool* k_idx_new, uint32_t base, uint32_t k_start, uint32_t k_end, uint32_t k_step, uint32_t i, uint32_t j) {
	uint32_t swap_i = base + i;
	uint32_t swap_j = base + j;

	double temp_double = RZ_norm[j];
	RZ_norm[j] = RZ_norm[i];
	RZ_norm[i] = temp_double;

	bool temp_bool = c_idx_new[swap_j];
	c_idx_new[swap_j] = c_idx_new[swap_i];
	c_idx_new[swap_i] = temp_bool;

	for (uint32_t h = 0; h < 3; h++) {
		temp_double = RZ_new[3 * swap_j + h];
		RZ_new[3 * swap_j + h] = RZ_new[3 * swap_i + h];
		RZ_new[3 * swap_i + h] = temp_double;
	}

	for (uint32_t h = k_start; h < k_end; h += k_step) {
		swap_i = h + i;
		swap_j = h + j;
		temp_bool = k_idx_new[swap_j];
		k_idx_new[swap_j] = k_idx_new[swap_i];
		k_idx_new[swap_i] = temp_bool;
	}
}

#endif // !ROTATOTOPE_ARRAY_CPPs

