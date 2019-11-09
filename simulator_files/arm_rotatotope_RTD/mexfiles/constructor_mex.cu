/*
Author: Bohao Zhang
Oct. 22 2019

arm_planning mex

This code aims to replace the contructor of the rotatotope
*/

#include "mex.h"
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
	if (nrhs != 5) {
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
P3. handle the output, release the memory
	*/
	links.returnResults();

	end_t = clock();
	mexPrintf("MEX FUNCTION TIME: %.6f\n", (end_t - start_t) / (double)(CLOCKS_PER_SEC));

	nlhs = 3;
	
	plhs[0] = mxCreateNumericMatrix(links.n_links * links.n_time_steps * links.Z_width, reduce_order, mxDOUBLE_CLASS, mxREAL);
	double *output1 = mxGetPr(plhs[0]);
	for (uint32_t i = 0; i < links.n_links * links.n_time_steps; i++) {
		for (uint32_t j = 0; j < reduce_order; j++) {
			for (uint32_t k = 0; k < links.Z_width; k++) {
				output1[(j * links.n_links * links.n_time_steps + i) * links.Z_width + k] = links.RZ[(i * reduce_order + j) * links.Z_width + k];
			}
		}
	}

	plhs[1] = mxCreateLogicalMatrix(links.n_links * links.n_time_steps, reduce_order);
	bool *output2 = mxGetLogicals(plhs[1]);
	for (uint32_t i = 0; i < links.n_links * links.n_time_steps; i++) {
		for (uint32_t j = 0; j < reduce_order; j++) {
			output2[j * links.n_links * links.n_time_steps + i] = links.c_idx[i * reduce_order + j];
		}
	}

	plhs[2] = mxCreateLogicalMatrix(links.n_links * (links.n_links + 1) * links.n_time_steps, reduce_order);
	bool *output3 = mxGetLogicals(plhs[2]);
	for (uint32_t i = 0; i < links.n_links * (links.n_links + 1) * links.n_time_steps; i++) {
		for (uint32_t j = 0; j < reduce_order; j++) {
			output3[j * links.n_links * (links.n_links + 1) * links.n_time_steps + i] = links.k_idx[i * reduce_order + j];
		}
	}
	
	plhs[3] = mxCreateLogicalMatrix(links.n_links * (links.n_links + 1) * links.n_time_steps, 2 * reduce_order - 1);
	bool *output4 = mxGetLogicals(plhs[3]);
	for (uint32_t i = 0; i < links.n_links * (links.n_links + 1) * links.n_time_steps; i++) {
		for (uint32_t j = 0; j < (2 * reduce_order - 1); j++) {
			output4[j * links.n_links * (links.n_links + 1) * links.n_time_steps + i] = links.k_idx_new[i * (2 * reduce_order - 1) + j];
		}
	}
	
	cudaFree(dev_R);
	cudaFree(dev_rot_axes);


	delete[] links.k_idx_new; // for debug
}

