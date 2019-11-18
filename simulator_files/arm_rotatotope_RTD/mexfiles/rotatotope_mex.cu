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
	links.stack(EEs);

	/*
P3.	generate the constraints
	*/
	links.generate_constraints(n_obstacles, OZ, OZ_width, OZ_length);

	end_t = clock();
	mexPrintf("MEX FUNCTION TIME: %.6f\n", (end_t - start_t) / (double)(CLOCKS_PER_SEC));

	/*
P4. handle the output, release the memory
	*/
	nlhs = 3;

	mxArray* output1 = mxCreateCellMatrix(1, n_obstacles);
	for (uint32_t i = 0; i < n_obstacles; i++) {
		mxArray* obstacle_i = mxCreateCellMatrix(1, n_links);
		for (uint32_t j = 0; j < n_links; j++) {
			uint32_t RZ_length = ((reduce_order - 1) * (j + 1) + 1);
			uint32_t buff_obstacle_length = RZ_length + 3;
			uint32_t constraint_length = ((buff_obstacle_length - 1) * (buff_obstacle_length - 2)) / 2;
			mxArray* link_j = mxCreateCellMatrix(1, n_time_steps);
			for (uint32_t k = 0; k < n_time_steps; k++) {
				mxArray* time_step_k = mxCreateNumericMatrix(constraint_length * 2, links.k_con_num[j][k], mxDOUBLE_CLASS, mxREAL);
				double *pt = (double*)mxGetData(time_step_k);

				for (uint32_t t = 0; t < links.k_con_num[j][k]; t++) {
					for (uint32_t p = 0; p < constraint_length * 2; p++) {
						pt[t * constraint_length * 2 + p] = links.A_con[j][((i * n_time_steps + k) * constraint_length * 2 + p) * links.max_k_con_num[j] + t];
					}
				}

				mxSetCell(link_j, k, time_step_k);
			}

			mxSetCell(obstacle_i, j, link_j);
		}

		mxSetCell(output1, i, obstacle_i);
	}
	plhs[0] = output1;

	mxArray* output2 = mxCreateCellMatrix(1, n_obstacles);
	for (uint32_t i = 0; i < n_obstacles; i++) {
		mxArray* obstacle_i = mxCreateCellMatrix(1, n_links);
		for (uint32_t j = 0; j < n_links; j++) {
			uint32_t RZ_length = ((reduce_order - 1) * (j + 1) + 1);
			uint32_t buff_obstacle_length = RZ_length + 3;
			uint32_t constraint_length = ((buff_obstacle_length - 1) * (buff_obstacle_length - 2)) / 2;
			mxArray* link_j = mxCreateCellMatrix(1, n_time_steps);
			for (uint32_t k = 0; k < n_time_steps; k++) {
				mxArray* time_step_k = mxCreateNumericMatrix(constraint_length * 2, 1, mxDOUBLE_CLASS, mxREAL);
				double *pt = (double*)mxGetData(time_step_k);

				for (uint32_t p = 0; p < constraint_length * 2; p++) {
					pt[p] = links.b_con[j][(i * n_time_steps + k) * constraint_length * 2 + p];
				}

				mxSetCell(link_j, k, time_step_k);
			}

			mxSetCell(obstacle_i, j, link_j);
		}

		mxSetCell(output2, i, obstacle_i);
	}
	plhs[1] = output2;

	mxArray* output3 = mxCreateCellMatrix(1, n_obstacles);
	for (uint32_t i = 0; i < n_obstacles; i++) {
		mxArray* obstacle_i = mxCreateCellMatrix(1, n_links);
		for (uint32_t j = 0; j < n_links; j++) {
			uint32_t RZ_length = ((reduce_order - 1) * (j + 1) + 1);
			mxArray* link_j = mxCreateCellMatrix(1, n_time_steps);
			for (uint32_t k = 0; k < n_time_steps; k++) {
				mxArray* time_step_k = mxCreateLogicalMatrix(2 * (j + 1), links.k_con_num[j][k]);
				bool *pt = mxGetLogicals(time_step_k);

				for (uint32_t t = 0; t < links.k_con_num[j][k]; t++) {
					for (uint32_t p = 0; p < 2 * (j + 1); p++) {
						pt[t * 2 * (j + 1) + p] = links.k_con[j][(p * n_time_steps + k) * RZ_length + t];
					}
				}

				mxSetCell(link_j, k, time_step_k);
			}

			mxSetCell(obstacle_i, j, link_j);
		}

		mxSetCell(output3, i, obstacle_i);
	}
	plhs[2] = output3;

	/*
	uint32_t link_id = 2;
	uint32_t RZ_length = ((reduce_order - 1) * (link_id + 1) + 1);
	uint32_t buff_obstacle_length = RZ_length + 3;

	plhs[3] = mxCreateNumericMatrix(n_time_steps * links.Z_width, RZ_length, mxDOUBLE_CLASS, mxREAL);
	double *output4 = mxGetPr(plhs[3]);
	for (uint32_t j = 0; j < n_time_steps; j++) {
		for (uint32_t k = 0; k < RZ_length; k++) {
			for (uint32_t p = 0; p < links.Z_width; p++) {
				output4[(k * n_time_steps + j) * links.Z_width + p] = links.debug_RZ[(j * RZ_length + k) * links.Z_width + p];
			}
		}
	}

	plhs[4] = mxCreateLogicalMatrix(n_time_steps, RZ_length);
	bool *output5 = mxGetLogicals(plhs[4]);
	for (uint32_t j = 0; j < n_time_steps; j++) {
		for (uint32_t k = 0; k < RZ_length; k++) {
			output5[k * n_time_steps + j] = links.debug_c_idx[j * RZ_length + k];
		}
	}

	plhs[5] = mxCreateLogicalMatrix(2 * (link_id + 1) * n_time_steps, RZ_length);
	bool *output6 = mxGetLogicals(plhs[5]);
	for (uint32_t j = 0; j < 2 * (link_id + 1) * n_time_steps; j++) {
		for (uint32_t k = 0; k < RZ_length; k++) {
			output6[k * 2 * (link_id + 1) * n_time_steps + j] = links.debug_k_idx[j * RZ_length + k];
		}
	}
	*/
	/*
	plhs[3] = mxCreateNumericMatrix(n_obstacles * n_time_steps * 3, buff_obstacle_length, mxDOUBLE_CLASS, mxREAL);
	double *output4 = mxGetPr(plhs[3]);
	for (uint32_t j = 0; j < n_obstacles * n_time_steps; j++) {
		for (uint32_t k = 0; k < buff_obstacle_length; k++) {
			for (uint32_t p = 0; p < 3; p++) {
				output4[(k * n_obstacles * n_time_steps + j) * 3 + p] = links.debug[(j * buff_obstacle_length + k) * 3 + p];
			}
		}
	}

	plhs[4] = mxCreateNumericMatrix(n_time_steps * 3, RZ_length, mxDOUBLE_CLASS, mxREAL);
	double *output5 = mxGetPr(plhs[4]);
	for (uint32_t j = 0; j < n_time_steps; j++) {
		for (uint32_t k = 0; k < RZ_length; k++) {
			for (uint32_t p = 0; p < 3; p++) {
				output5[(k * n_time_steps + j) * 3 + p] = links.debug_2[(j * RZ_length + k) * 3 + p];
			}
		}
	}
	*/
	
	cudaFree(dev_R);
	cudaFree(dev_rot_axes);
}

