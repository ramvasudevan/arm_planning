/*
Author: Bohao Zhang
Oct. 22 2019

arm_planning mex

This code aims to replace the contructor of the rotatotope
*/

#include "rotatotope_NLP.h"
#include<iostream> 

using namespace Ipopt;

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
	if (nrhs != 9) {
		mexErrMsgIdAndTxt("MyProg:ConvertString","*** Incorrect number of input!");
	}

	uint32_t n_links = 3;
	uint32_t n_joints = n_links * 2;
	uint32_t n_time_steps = 100;

	double* R = mxGetPr(prhs[0]);
	uint32_t R_width = (uint32_t)mxGetM(prhs[0]);
	uint32_t R_length = (uint32_t)mxGetN(prhs[0]);

	double* dev_R;
	cudaMalloc((void**)&dev_R, R_width * R_length * sizeof(double));
	cudaMemcpy(dev_R, R, R_width * R_length * sizeof(double), cudaMemcpyHostToDevice);

	double* link_Z = mxGetPr(prhs[1]);
	uint32_t link_Z_width = (uint32_t)mxGetM(prhs[1]);
	uint32_t link_Z_length = (uint32_t)mxGetN(prhs[1]);
	
	double* EE_Z = mxGetPr(prhs[2]);
	uint32_t EE_Z_width = (uint32_t)mxGetM(prhs[2]);
	uint32_t EE_Z_length = (uint32_t)mxGetN(prhs[2]);

	uint32_t n_obstacles = (uint32_t)(*mxGetPr(prhs[3]));

	double* OZ = mxGetPr(prhs[4]);
	uint32_t OZ_width = (uint32_t)mxGetM(prhs[4]);
	uint32_t OZ_length = (uint32_t)mxGetN(prhs[4]);

	double* k_opt = mxGetPr(prhs[5]);

	double* q = mxGetPr(prhs[6]);

	double* q_dot = mxGetPr(prhs[7]);

	double* q_des = mxGetPr(prhs[8]);

	start_t = clock();

	/*
P1.	generate all the rotatotopes
	*/
	uint32_t R_unit_length = R_length / (n_joints * n_time_steps); // equivalent with length(obj.R)

	uint8_t rot_axes[6] = { 3, 2, 1, 2, 1, 2 }, *dev_rot_axes; // rot_axes can be directly defined here. no need for mex for now.
	cudaMalloc((void**)&dev_rot_axes, 6 * sizeof(uint8_t));
	cudaMemcpy(dev_rot_axes, rot_axes, 6 * sizeof(uint8_t), cudaMemcpyHostToDevice);

	rotatotopeArray links = rotatotopeArray(n_links, n_time_steps, R, dev_R, R_unit_length, dev_rot_axes, link_Z, link_Z_width, link_Z_length);
	rotatotopeArray EEs = rotatotopeArray(n_links - 1, n_time_steps, R, dev_R, R_unit_length, dev_rot_axes, EE_Z, EE_Z_width, EE_Z_length);

	/*
P2.	stack the rotatotopes
	*/
	links.stack(EEs);

	/*
P3.	generate the constraints
	*/
	links.generate_constraints(n_obstacles, OZ, OZ_width, OZ_length);
	end_t = clock();
	mexPrintf("Construct rotatotopes time: %.6f\n", (end_t - start_t) / (double)(CLOCKS_PER_SEC));
	start_t = clock();
	/*
P4.	solve the NLP
	*/

	SmartPtr<rotatotope_NLP> mynlp = new rotatotope_NLP();
	mynlp->set_parameters(&links, q, q_dot, q_des, links.c_k, links.g_k, n_obstacles);

	// Create a new instance of IpoptApplication
    //  (use a SmartPtr, not raw)
    // We are using the factory, since this allows us to compile this
    // example with an Ipopt Windows DLL
    SmartPtr<IpoptApplication> app = IpoptApplicationFactory();

    // Change some options
    // Note: The following choices are only examples, they might not be
    //       suitable for your optimization problem.
	app->Options()->SetNumericValue("tol", 1e-3);
	app->Options()->SetNumericValue("max_cpu_time", 1);
	app->Options()->SetNumericValue("print_level", 0);
    app->Options()->SetStringValue("mu_strategy", "adaptive");
    app->Options()->SetStringValue("output_file", "ipopt.out");
    //app->Options()->SetStringValue("hessian_approximation", "limited-memory");
	//app->Options()->SetStringValue("limited_memory_update_type", "bfgs");
	//app->Options()->SetStringValue("derivative_test", "first-order");
	//app->Options()->SetNumericValue("derivative_test_perturbation", 0.000001);

    // Initialize the IpoptApplication and process the options
    ApplicationReturnStatus status;
    status = app->Initialize();
    if( status != Solve_Succeeded ) {
		mexErrMsgIdAndTxt("MyProg:ConvertString", "*** Error during initialization!");
    }

    // Ask Ipopt to solve the problem
    status = app->OptimizeTNLP(mynlp);

	nlhs = 1;
    if( status == Solve_Succeeded ) {
        plhs[0] = mxCreateNumericMatrix(n_links * 2, 1, mxDOUBLE_CLASS, mxREAL);
		double *output0 = (double*)mxGetData(plhs[0]);
		for (uint32_t i = 0; i < n_links * 2; i++) {
			output0[i] = mynlp->solution[i];
		}
    }
    else {
		plhs[0] = mxCreateNumericMatrix(1, 1, mxINT32_CLASS, mxREAL);
		int *output0 = (int*)mxGetData(plhs[0]);
		*output0 = -12345;
    }

	end_t = clock();
	mexPrintf("IPOPT NLP time: %.6f\n", (end_t - start_t) / (double)(CLOCKS_PER_SEC));

	/*
P5. handle the output, release the memory
	*/
	
	links.evaluate_constraints(k_opt);
	
	plhs[1] = mxCreateNumericMatrix(n_links * n_obstacles * n_time_steps, 1, mxDOUBLE_CLASS, mxREAL);
	double *output0 = (double*)mxGetData(plhs[1]);
	for (uint32_t i = 0; i < n_obstacles; i++) {
		for (uint32_t j = 0; j < n_links; j++) {
			for (uint32_t k = 0; k < n_time_steps; k++) {
				output0[(i * n_links + j) * n_time_steps + k] = links.con[(j * n_obstacles + i) * n_time_steps + k];
			}
		}
	}
	
	plhs[2] = mxCreateNumericMatrix(n_links * 2, n_links * n_obstacles * n_time_steps, mxDOUBLE_CLASS, mxREAL);
	double *output1 = (double*)mxGetData(plhs[2]);
	for (uint32_t i = 0; i < n_obstacles; i++) {
		for (uint32_t j = 0; j < n_links; j++) {
			for (uint32_t k = 0; k < n_time_steps; k++) {
				for (uint32_t p = 0; p < n_links * 2; p++) {
					output1[((i * n_links + j) * n_time_steps + k) * n_links * 2 + p] = links.jaco_con[((j * n_obstacles + i) * n_time_steps + k) * n_links * 2 + p];
				}
			}
		}
	}

	plhs[3] = mxCreateNumericMatrix(n_links * (n_links * 2 - 1), n_links * n_obstacles * n_time_steps, mxDOUBLE_CLASS, mxREAL);
	double *output2 = (double*)mxGetData(plhs[3]);
	for (uint32_t i = 0; i < n_obstacles; i++) {
		for (uint32_t j = 0; j < n_links; j++) {
			for (uint32_t k = 0; k < n_time_steps; k++) {
				for (uint32_t p = 0; p < n_links * (n_links * 2 - 1); p++) {
					output2[((i * n_links + j) * n_time_steps + k) * n_links * (n_links * 2 - 1) + p] = links.hess_con[((j * n_obstacles + i) * n_time_steps + k) * n_links * (n_links * 2 - 1) + p];
				}
			}
		}
	}
	
	
	/*
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
	plhs[1] = output1;

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
	plhs[2] = output2;

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
	plhs[3] = output3;
	*/

	cudaFree(dev_R);
	cudaFree(dev_rot_axes);
}

