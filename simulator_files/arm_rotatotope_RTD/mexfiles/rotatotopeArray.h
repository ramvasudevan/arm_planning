/*
Author: Bohao Zhang
Oct. 29 2019

arm_planning mex

a cuda array for a cluster of rotatotopes
*/

#ifndef ROTATOTOPE_ARRAY_H
#define ROTATOTOPE_ARRAY_H

#include "mex.h"
#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include "IpTNLP.hpp"
#include "IpIpoptApplication.hpp"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <ctime>
#include <cfloat>
#include <vector>

#define k_dim 2
#define max_norm_size 310
#define max_RZ_length 130
#define ORIGIN_SHIFT_X -0.03265
#define ORIGIN_SHIFT_Y 0.0
#define ORIGIN_SHIFT_Z 0.72601
#define A_BIG_NUMBER 1000000.0
#define BUFFER_DIST 0.1460
#define TOO_SMALL_POLYTOPE_JUDGE 0.000001
#define CONSERVATIVE_BUFFER 0.001

using std::vector;

class rotatotopeArray {
public:
	/*
	Instruction:
		constructor of rotatotopeArray
	Requires:
		1. n_links
		2. n_time_steps
		3. joint_per_link
		4. R
		5. dev_R
		6. R_unit_length
		7. dev_rot_axes
		8. Z
		9. Z_width
		10. Z_length
		11. reduce_order
	*/
	rotatotopeArray(uint32_t n_links_input, uint32_t n_time_steps_input, uint32_t joint_per_link_input, double* R_input, double* dev_R_input, uint32_t R_unit_length_input, uint8_t* dev_rot_axes_input, double* Z_input, uint32_t Z_width_input, uint32_t Z_length_input, uint32_t reduce_order_input);

	/*
	Instruction:
		destructor of rotatotopeArray, release the memory
	*/
	~rotatotopeArray();

	/*
	Instruction:
		stack the links with EE and base
	Requires:
		1. rotatotope array of EEs
		1. rotatotope array of base
	*/
	void stack(rotatotopeArray &EEs, rotatotopeArray &base);

	/*
	Instruction:
		generate the constraints using input obstacles
	Requires:
		1. number of obstacles
		2. zonotope array of obstacles
		3. width of the array above
		4. length of the array above
	*/
	void generate_constraints(uint32_t n_obstacles, double* OZ, uint32_t OZ_width, uint32_t OZ_length);

	/*
	Instruction:
		generate the self intersection constraints
	Requires:
		1. number of pairs that may cause self intersection
		2. the index of links in pairs
	*/
	void generate_self_constraints(uint32_t n_pairs_input, uint32_t* self_pairs_input);

	/*
	Instruction:
		evaluate constraints at k_opt for optimization
	Requires:
		1. k_opt
			--> input of k
	*/
	void evaluate_constraints(double* k_opt);

	bool debugMode;

	// number of different zonotopes
	uint32_t n_links;

	// number of time steps
	uint32_t n_time_steps;

	// number of rotation for each different zonotopes
	uint32_t joint_per_link;

	// number of pairs of self intersection check
	uint32_t n_pairs;

	// self intersection pairs
	uint32_t* self_pairs;

	// a pointer to R in gpu
	double* dev_R;
	uint32_t R_unit_length;

	// a pointer to rot_axes in gpu
	uint8_t* dev_rot_axes;

	// the array of zonotopes to rotate
	double* Z;
	double* dev_Z;
	uint32_t Z_length, Z_width, Z_unit_length;

	// zonotope of k interval
	double* c_k;
	double* g_k;

	// the resulting array of rotatotopes without stacking
	double* dev_RZ;

	// reduce order for each link
	uint32_t reduce_order;

	// keep track of the center
	bool* dev_c_idx;

	// keep track of k-dependent generators
	bool* dev_k_idx;

	// stacking results
	double** RZ_stack;
	double** dev_RZ_stack;

	bool** c_idx_stack;
	bool** dev_c_idx_stack;

	bool** k_idx_stack;
	bool** dev_k_idx_stack;

	uint32_t* RZ_length;

	double* debug_RZ = nullptr;
	bool* debug_c_idx = nullptr;
	bool* debug_k_idx = nullptr;

	// number of obstacles
	uint32_t n_obstacles;

	// constraint polynomials
	double** A_con;
	double** dev_A_con;

	double** d_con;
	double** dev_d_con;

	double** delta_con;
	double** dev_delta_con;

	bool** k_con;
	bool** dev_k_con;

	uint8_t** k_con_num; // size of each k con
	uint8_t** dev_k_con_num; 

	// maximum of k_con in rotatotopes
	uint32_t* max_k_con_num;

	// self intersection check
	vector< vector< vector<double> > > A_con_self_array;
	vector< vector< vector<double> > > d_con_self_array;
	vector< vector< vector<double> > > delta_con_self_array;

	// current constraints info
	double* current_k_opt; // current k
	double* con; // value of constraints at k
	double* jaco_con; // value of jacobian of constraints at k
	double* hess_con; // value of hessian of constraints at k

	// timing
	std::clock_t start_t, end_t; 
};

/*
Instruction:
	initialize RZ with link_Z to prepare for multiplication
Requires:
	1. link_Z
		--> the Z of zonotopes of links
	2. link_Z_length
	3. reduce_order
	4. RZ
	5. c_idx
Modifies:
	1. RZ
	2. c_idx
*/
__global__ void initialize_RZ_kernel(double* link_Z, uint32_t link_Z_length, uint32_t reduce_order, double* RZ, bool* c_idx);

/*
Instruction:
	multiply a zonotope with a rotatotope
Requires:
	1. rot_axes
	2. link_offset
		--> which link should be rotated
	3. joint_offset
		--> which joint of link should be rotated
	4. reduce_order
	5. RZ
		--> the Z of zonotopes of results from the previous multiplication
	6. R
		--> the Z of zonotopes in trig_FRS
	7. c_idx
	8. k_idx
	9. RZ_new
		--> the Z of zonotopes after rotation
	10. c_idx_new
		--> index of who are multiplied with a center
	11. k_idx_new
		--> index of who are multiplied with a k-dep generator
Modifies:
	1. RZ_new
	2. c_idx_new
	3. k_idx_new
*/
__global__ void multiply_kernel(uint8_t* rot_axes, uint32_t link_offset, uint32_t joint_offset, uint32_t reduce_order, double* RZ, double* R, bool* c_idx, bool* k_idx, double* RZ_new, bool* c_idx_new, bool* k_idx_new);

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
	5. reduce_order
	6. RZ
	7. c_idx
	8. k_idx
Modifies:
	1. RZ
	2. c_idx
	3. k_idx
*/
__global__ void reduce_kernel(double* RZ_new, bool* c_idx_new, bool* k_idx_new, uint32_t link_offset, uint32_t reduce_order, double* RZ, bool* c_idx, bool* k_idx);

/*
Instruction:
	copy one rotatotope to another array, used in stacking
Requires:
	1. link_id
		--> which link should be copied
	2. link_RZ
		--> the Z of zonotopes of links
	3. link_c_idx
	4. link_k_idx
	5. link reduce_order
	6. point reduce_order
	7. RZ_stack
		--> for stacking
	8. c_idx_stack
	9. k_idx_stack
Modifies:
	1. RZ_stack
	2. c_idx_stack
	3. k_idx_stack
*/
__global__ void copy_kernel(uint32_t link_id, double* RZ, bool* c_idx, bool* k_idx, uint32_t link_reduce_order, uint32_t point_reduce_order, double* RZ_stack, bool* c_idx_stack, bool* k_idx_stack);

/*
Instruction:
	stack two rotatotopes together
Requires:
	1. link_id
		--> which in links should be stacked
	2. EE_id
		--> which in EE should be stacked
	3. stack_offset
		--> where should EE be stacked
	4. link reduce_order
	5. point reduce_order
	6. RZ_stack
		--> the Z of zonotopes of links
	7. EE_RZ
	8. c_idx_stack
	9. EE_c_idx
	10. k_idx_stack
	11. EE_k_idx
Modifies:
	1. RZ_stack
	2. c_idx_stack
	3. k_idx_stack
*/
__global__ void stack_kernel(uint32_t link_id, uint32_t EE_id, uint32_t stack_offset, uint32_t link_reduce_order, uint32_t point_reduce_order, double* RZ_stack, double* EE_RZ, bool* c_idx_stack, bool* EE_c_idx, bool* k_idx_stack, bool* EE_k_idx);

/*
Instruction:
	shift the origin
Requires:
	1. RZ_length
	2. RZ_stack
Modifies:
	1. RZ_stack
*/
__global__ void origin_shift_kernel(uint32_t RZ_length, double* RZ_stack); 

/*
Instruction:
	buffer the obstacle by k-independent generators
Requires:
	1. link_id
		--> which link is in operation
	2. RZ_length
	3. RZ
	4. c_idx
	5. k_idx
	6. OZ
	7. OZ_unit_length
	8. buff_obstacles
	9. frs_k_dep_G
	10. k_con
	11. k_con_num
Modifies:
	1. buff_obstacles
	2. frs_k_dep_G
	3. k_con
	4. k_con_num
*/
__global__ void buff_obstacles_kernel(uint32_t link_id, uint32_t RZ_length, double* RZ, bool* c_idx, bool* k_idx, double* OZ, uint32_t OZ_unit_length, double* buff_obstacles, double* frs_k_dep_G, bool* k_con, uint8_t* k_con_num);

/*
Instruction:
	generate the polytopes of constraints
Requires:
	1. link_id
		--> which link is in operation
	2. RZ_length
	3. buff_obstacles
	4. frs_k_dep_G
	5. k_con_num
	6. A_con_width = max_k_con_num
	7. A_con
	8. d_con
	9. delta_con
Modifies:
	1. A_con
	2. d_con
	3. delta_con
*/
__global__ void polytope(uint32_t link_id, uint32_t RZ_length, double* buff_obstacles, double* frs_k_dep_G, uint8_t* k_con_num, uint32_t A_con_width, double* A_con, double* d_con, double* delta_con);

/*
Instruction:
	evaluate constraints with lambda
Requires:
	1. lambda
	2. link_id
	3. RZ_length
	4. A_con
	5. A_con_width
	6. d_con
	7. delta_con
	8. k_con
	9. k_con_num
	10. con_result
Modifies:
	1. con_result
*/
__global__ void evaluate_constraints_kernel(double* lambda, uint32_t link_id, uint32_t RZ_length, double* A_con, uint32_t A_con_width, double* d_con, double* delta_con, bool* k_con, uint8_t* k_con_num, double* con_result);

/*
Instruction:
	first find the maximum of the constraints
	evaluate the jacobian and hessian of constraint with that maximum
Requires:
	1. con_result
	2. link_id
	3. RZ_length
	4. lambda
	5. g_k
	6. A_con
	7. k_con
	8. k_con_num
	9. n_links
	10. con
	11. jaco_con
	12. hess_con
Modifies:
	1. con
	2. jaco_con
	3. hess_con
*/
__global__ void evaluate_gradient_kernel(double* con_result, uint32_t link_id, uint32_t RZ_length, double* lambda, double* g_k, double* A_con, uint32_t A_con_width, bool* k_con, uint8_t* k_con_num, uint32_t n_links, double* con, double* jaco_con, double* hess_con);

#endif // !ROTATOTOPE_ARRAY_H