/*
Author: Bohao Zhang
Oct. 29 2019

arm_planning mex

a cuda array for a cluster of rotatotopes
*/

#ifndef ROTATOTOPE_ARRAY_H
#define ROTATOTOPE_ARRAY_H

#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <ctime>

#define k_dim 2
#define reduce_order 30
#define norm_size 300
#define max_buff_obstacle_size 40
#define max_constraint_size 741

class rotatotopeArray {
public:
	/*
	Instruction:
		constructor of rotatotopeArray
	Requires:
		1. n_links
		2. n_time_steps
		3. dev_R
		4. R_unit_length
		5. dev_rot_axes
		6. Z
		7. Z_width
		8. Z_length
	*/
	rotatotopeArray(uint32_t n_links_input, uint32_t n_time_steps_input, double* &dev_R_input, uint32_t R_unit_length_input, uint8_t* &dev_rot_axes_input, double* &Z_input, uint32_t Z_width_input, uint32_t Z_length_input);

	/*
	Instruction:
		destructor of rotatotopeArray, release the memory
	*/
	~rotatotopeArray();

	/*
	Instruction:
		stack the links
	Requires:
		1. rotatotope array of EEs
	*/
	void stack(rotatotopeArray &EEs);

	/*
	Instruction:
		copy results from gpu to memory
	*/
	void returnResults();

	// number of different zonotopes
	uint32_t n_links;

	// number of time steps
	uint32_t n_time_steps;

	// a pointer to R in gpu
	double* dev_R;
	uint32_t R_unit_length;

	// a pointer to rot_axes in gpu
	uint8_t* dev_rot_axes;

	// the array of zonotopes to rotate
	double* Z;
	double* dev_Z;
	uint32_t Z_length, Z_width, Z_unit_length;

	// the resulting array of rotatotopes
	double* RZ;
	double* dev_RZ;
	double* RZ_new; // for debug
	double* dev_RZ_new;

	// keep track of the center
	bool* c_idx;
	bool* dev_c_idx;
	bool *dev_c_idx_new; 

	// keep track of k-dependent generators
	bool* k_idx;
	bool* dev_k_idx;
	bool *dev_k_idx_new;
};

/*
Instruction:
	initialize RZ with link_Z to prepare for multiplication
Requires:
	1. link_Z
		--> the Z of zonotopes of links
Modifies:
	1. RZ
	2. c_idx
*/
__global__ void initialize_RZ_kernel(double* link_Z, uint32_t link_Z_length, double* RZ, bool* c_idx);

/*
Instruction:
	add two rotatotopes together
Requires:
	1. link_offset
		--> which link should be rotated
	2. link RZ
		--> the Z of zonotopes of links
	3. EE_RZ
		--> the Z of zonotopes of EEs
	4. link_c_idx
	5. EE_c_idx
	6. link_k_idx
	7. EE_k_idx
	8. RZ_new
	9. c_idx_new
	10. k_idx_new
Modifies:
	1. RZ_new
	2. c_idx_new
	3. k_idx_new
*/
__global__ void add_kernel(uint32_t link_offset, double* link_RZ, double* EE_RZ, bool* link_c_idx, bool* EE_c_idx,  bool* link_k_idx, bool* EE_k_idx, double* RZ_new, bool* c_idx_new, bool* k_idx_new);

/*
Instruction:
	multiply a zonotope with a rotatotope
Requires:
	1. rot_axes
	2. link_offset
		--> which link should be rotated
	3. joint_offset
		--> which joint of link should be rotated
	4. RZ
		--> the Z of zonotopes of results from the previous multiplication
	5. R
		--> the Z of zonotopes in trig_FRS
	6. c_idx
	7. k_idx
	8. RZ_new
		--> the Z of zonotopes after rotation
	9. c_idx_new
		--> index of who are multiplied with a center
	10. k_idx_new
		--> index of who are multiplied with a k-dep generator
Modifies:
	1. RZ_new
	2. c_idx_new
	3. k_idx_new
*/
__global__ void multiply_kernel(uint8_t* rot_axes, uint32_t link_offset, uint32_t joint_offset, double* RZ, double* R, bool* c_idx, bool* k_idx, double* RZ_new, bool* c_idx_new, bool* k_idx_new);

/*
Instruction:
	cuda implementation of reduce
Requires:
	1. RZ_new
		--> the temporary array for RZ, will be reduced to RZ
	2. c_idx_new
	3. k_idx_new
	4. link_offset
		--> which link should be reduced
Modifies:
	1. RZ
	2. c_idx
	3. k_idx
*/
__global__ void reduce_kernel(double* RZ_new, bool* c_idx_new, bool* k_idx_new, uint32_t link_offset, double* RZ, bool* c_idx, bool* k_idx);

/*
Instruction:
	swap operations of all related data for reduce()
Requires:
	1. RZ_norm
	2. RZ_new
	3. c_idx_new
	4. k_idx_new
	5. base
		--> base index for swap in RZ_norm, RZ_new, c_idx_new
	6. k_start
	7. k_end
	8. k_step
		-->  index for swap in k_idx_new
	9. i
	10. j
		--> indeces in array to swap
Modifies:
	1. RZ_norm
	2. RZ_new
	3. c_idx_new
	4. k_idx_new
*/
__device__ void swap(double* RZ_norm, double* RZ, bool* c_idx, bool* k_idx, uint32_t base, uint32_t k_start, uint32_t k_end, uint32_t k_step, uint32_t i, uint32_t j);

#endif // !ROTATOTOPE_ARRAY_H