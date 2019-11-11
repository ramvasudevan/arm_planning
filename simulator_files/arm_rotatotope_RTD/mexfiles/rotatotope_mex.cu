/*
Author: Bohao Zhang
Oct. 22 2019

arm_planning mex

This code aims to replace the contructor of the rotatotope
*/


#include "rotatotopeArray.h"

/*
Instruction:
	This is the mex function to replace
	generate_matrices()
	multiply()
	in the constructor of rotatotope
Requires:
	1. n_links
		--> number of links
	2. n_time_steps
		--> number of time steps
	3. trig_FRS{j}(i) . Z
		--> the Z of zonotopes in trig_FRS,
			index: i \in 1 : n_links * 2
				   j \in 1 : n_time_steps
				   trig_FRS(i,j).Z = (i * n_time_steps + j) * 10 : (i * n_time_steps + j + 1) * 10 - 1
			we need trig_FRS(:, 1 : n * 2) for n th link
	4. link_zonotopes{i} . Z
		--> the Z of link zonotopes
			index: i \in 1 : n_links
	5. EE_zonotopes{i} . Z
		--> the Z of link zonotopes
			index: i \in 1 : n_links
Returns:
	1. RZ
	2. c_idx
	3. k_idx
*/
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
	std::clock_t start_t, end_t; // timing

	/*
P0.	process the input
	*/
	if (nrhs != 7) {
		mexErrMsgIdAndTxt("MyProg:ConvertString","Incorrect number of input!");
	}

	uint32_t n_links = (uint32_t)(*mxGetPr(prhs[0]));
	uint32_t n_joints = 2 * n_links;
	uint32_t n_time_steps = (uint32_t)(*mxGetPr(prhs[1]));

	double* R = mxGetPr(prhs[2]);
	uint32_t R_width = (uint32_t)mxGetM(prhs[2]);
	uint32_t R_length = (uint32_t)mxGetN(prhs[2]);

	double* dev_R;
	cudaMalloc((void**)&dev_R, R_width * R_length * sizeof(double));
	cudaMemcpy(dev_R, R, R_width * R_length * sizeof(double), cudaMemcpyHostToDevice);

	double* link_Z = mxGetPr(prhs[3]);
	uint32_t link_Z_width = (uint32_t)mxGetM(prhs[3]);
	uint32_t link_Z_length = (uint32_t)mxGetN(prhs[3]);
	
	double* EE_Z = mxGetPr(prhs[4]);
	uint32_t EE_Z_width = (uint32_t)mxGetM(prhs[4]);
	uint32_t EE_Z_length = (uint32_t)mxGetN(prhs[4]);

	uint32_t n_obstacles = (uint32_t)(*mxGetPr(prhs[5]));

	double* OZ = mxGetPr(prhs[6]);
	uint32_t OZ_width = (uint32_t)mxGetM(prhs[6]);
	uint32_t OZ_length = (uint32_t)mxGetN(prhs[6]);

	start_t = clock();

	/*
P1.	generate all the rotatotopes
	*/
	uint32_t R_unit_length = R_length / (n_joints * n_time_steps); // equivalent with length(obj.R)

	uint8_t rot_axes[6] = { 3, 2, 1, 2, 1, 2 }, *dev_rot_axes; // rot_axes can be directly defined here. no need for mex for now.
	cudaMalloc((void**)&dev_rot_axes, 6 * sizeof(uint8_t));
	cudaMemcpy(dev_rot_axes, rot_axes, 6 * sizeof(uint8_t), cudaMemcpyHostToDevice);

	rotatotopeArray links = rotatotopeArray(n_links, n_time_steps, dev_R, R_unit_length, dev_rot_axes, link_Z, link_Z_width, link_Z_length);
	rotatotopeArray EEs = rotatotopeArray(n_links - 1, n_time_steps, dev_R, R_unit_length, dev_rot_axes, EE_Z, EE_Z_width, EE_Z_length);

	/*
P2.	stack the rotatotopes
	*/
	if (n_links > 1) {
		links.stack(EEs);
	}

	/*
P3.	generate the constraints
	*/
	links.generate_constraints(n_obstacles, OZ, OZ_width, OZ_length);

	end_t = clock();
	mexPrintf("MEX FUNCTION TIME: %.6f\n", (end_t - start_t) / (double)(CLOCKS_PER_SEC));

	/*
P4. handle the output, release the memory
	*/
	
	nlhs = 4;
	plhs[0] = mxCreateNumericMatrix(n_obstacles * n_links * max_constraint_size * 2, n_time_steps * links.max_k_con_num, mxDOUBLE_CLASS, mxREAL);
	double *output1 = mxGetPr(plhs[0]);
	for (uint32_t j = 0; j < n_time_steps; j++) {
		for (uint32_t k = 0; k < links.max_k_con_num; k++) {
			for (uint32_t i = 0; i < n_obstacles * n_links; i++) {
				for (uint32_t p = 0; p < max_constraint_size * 2; p++) {
					output1[((j * links.max_k_con_num + k) * n_obstacles * n_links + i) * max_constraint_size * 2 + p] = links.A_con[((i * n_time_steps + j) * max_constraint_size * 2 + p) * links.max_k_con_num + k];
				}
			}
		}
	}

	plhs[1] = mxCreateNumericMatrix(n_obstacles * n_links * max_constraint_size * 2, n_time_steps, mxDOUBLE_CLASS, mxREAL);
	double *output2 = mxGetPr(plhs[1]);
	for (uint32_t j = 0; j < n_time_steps; j++) {
		for (uint32_t i = 0; i < n_obstacles * n_links; i++) {
			for (uint32_t p = 0; p < max_constraint_size * 2; p++) {
				output2[(j * n_obstacles * n_links + i) * max_constraint_size * 2 + p] = links.b_con[(i * n_time_steps + j) * max_constraint_size * 2 + p];
			}
		}
	}

	plhs[2] = mxCreateLogicalMatrix(n_links * (n_links + 1) * n_time_steps, reduce_order);
	bool *output3 = mxGetLogicals(plhs[2]);
	for (uint32_t i = 0; i < n_links * (n_links + 1) * n_time_steps; i++) {
		for (uint32_t j = 0; j < reduce_order; j++) {
			output3[j * n_links * (n_links + 1) * n_time_steps + i] = links.k_con[i * reduce_order + j];
		}
	}

	plhs[3] = mxCreateNumericMatrix(n_links, n_time_steps, mxDOUBLE_CLASS, mxREAL);
	double *output4 = mxGetPr(plhs[3]);
	for (uint32_t i = 0; i < n_links; i++) {
		for (uint32_t j = 0; j < n_time_steps; j++) {
			output4[j * n_links + i] = links.k_con_num[i * n_time_steps + j];
		}
	}

	/*
	plhs[3] = mxCreateLogicalMatrix(links.n_links * (links.n_links + 1) * links.n_time_steps, 2 * reduce_order - 1);
	bool *output4 = mxGetLogicals(plhs[3]);
	for (uint32_t i = 0; i < links.n_links * (links.n_links + 1) * links.n_time_steps; i++) {
		for (uint32_t j = 0; j < (2 * reduce_order - 1); j++) {
			output4[j * links.n_links * (links.n_links + 1) * links.n_time_steps + i] = links.k_idx_new[i * (2 * reduce_order - 1) + j];
		}
	}
	*/
	
	cudaFree(dev_R);
	cudaFree(dev_rot_axes);


	//delete[] links.k_idx_new; // for debug
}

