/*
Author: Bohao Zhang
Oct. 22 2019

arm_planning mex

This code aims to replace the contructor of the rotatotope

The hyperparameters that are directly hardcoded in this code are:
1. k_dim
	--> which dimension in R is k-dependent
2. origin shift [x,y,z]
	--> origin shift of the robot links
3. buffer distance
	--> the total distance buffered for the obstacles
4. TOO_SMALL_POLYTOPE_JUDGE
	--> A criteria for the square of the 2-norm of the generator
5. CONSERVATIVE_BUFFER
	--> a small offset directly applied to the constraint functions
6. t_plan
7. t_move
8. t_total
9. number of links
10. number of time steps
11. the zonotope of links
12. the zonotope of end effectors
13. the zonotope of base
14. rot_axes
	--> which axis should be rotated around for each joint
15. link / EE reduce order
*/

#include "rotatotope_NLP.h"
#include<iostream> 

using namespace Ipopt;

const bool debugMode = true;

/*
Instruction:
	This is the mex function to replace
	generate_matrices()
	multiply()
	in the constructor of rotatotope
Requires:
	1. trig_FRS{j}(i) . Z
		--> the Z of zonotopes in trig_FRS,
			index: i \in 1 : n_links * 2
				   j \in 1 : n_time_steps
				   trig_FRS(i,j).Z = (i * n_time_steps + j) * 10 : (i * n_time_steps + j + 1) * 10 - 1
			we need trig_FRS(:, 1 : n * 2) for n th link
	2. number of obstacles
	3. zonotopes of obstacles (1 center with 3 generators)
	4. k_opt input for debugging
	5. q
	6. q_dot
	7. q_des
	8. g_k
*/
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
	std::clock_t start_t, end_t; // timing  

	/*
P0.	process the input
	*/
	if (nrhs != 8) {
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

	uint32_t n_obstacles = (uint32_t)(*mxGetPr(prhs[1]));

	double* OZ = mxGetPr(prhs[2]);
	uint32_t OZ_width = (uint32_t)mxGetM(prhs[2]);
	uint32_t OZ_length = (uint32_t)mxGetN(prhs[2]);

	double* k_opt = mxGetPr(prhs[3]);

	double* q = mxGetPr(prhs[4]);

	double* q_dot = mxGetPr(prhs[5]);

	double* q_des = mxGetPr(prhs[6]);

	double* g_k = mxGetPr(prhs[7]);

	double link_Z[18] =  {  0.1778,         0,         0,
							0.1778,         0,         0,
							0.1651,         0,         0,
							0.1651,         0,         0,
							0.1651,         0,         0,
							0.1651,         0,         0};
	uint32_t link_Z_width = 3;
	uint32_t link_Z_length = 6;

	double EE_Z[6] =  {     0.3556,         0,         0,
		                    0.3302,         0,         0};
	uint32_t EE_Z_width = 3;
	uint32_t EE_Z_length = 2;

	double base_Z[3] =  {   0.1206,         0,         0.0825};
	uint32_t base_Z_width = 3;
	uint32_t base_Z_length = 1;

	start_t = clock();
	/*
P1.	generate all the rotatotopes
	*/
	uint32_t R_unit_length = R_length / (n_joints * n_time_steps); // equivalent with length(obj.R)

	uint8_t rot_axes[6] = { 3, 2, 1, 2, 1, 2 }, *dev_rot_axes; // rot_axes can be directly defined here. no need for mex for now.
	cudaMalloc((void**)&dev_rot_axes, 6 * sizeof(uint8_t));
	cudaMemcpy(dev_rot_axes, rot_axes, 6 * sizeof(uint8_t), cudaMemcpyHostToDevice);

	// should promise that link + 3 * point <= 45, so that combination size <= 1024
	uint32_t link_reduce_order = 15;
	uint32_t point_reduce_order = 10;
	
	rotatotopeArray links = rotatotopeArray(n_links, n_time_steps, 2, R, dev_R, R_unit_length, dev_rot_axes, link_Z, link_Z_width, link_Z_length, link_reduce_order, g_k);
	rotatotopeArray EEs = rotatotopeArray(n_links - 1, n_time_steps, 2, R, dev_R, R_unit_length, dev_rot_axes, EE_Z, EE_Z_width, EE_Z_length, point_reduce_order, g_k);
	rotatotopeArray base = rotatotopeArray(n_links - 2, n_time_steps, 1, R, dev_R, R_unit_length, dev_rot_axes, base_Z, base_Z_width, base_Z_length, point_reduce_order, g_k);

	links.debugMode = debugMode;

	/*
P2.	stack the rotatotopes
	*/
	links.stack(EEs, base);

	/*
P3.	generate the constraints
	*/
	links.generate_constraints(n_obstacles, OZ, OZ_width, OZ_length);

	uint32_t n_pairs = 1;
	uint32_t self_pairs[2] = {0, 2}; // the latter one in the pair is always higher
	links.generate_self_constraints(n_pairs, self_pairs);

	end_t = clock();
	mexPrintf("CUDA: Construct Rotatotopes time: %.6f ms\n", 1000.0 * (end_t - start_t) / (double)(CLOCKS_PER_SEC));
	
	/*
P4.	solve the NLP
	*/
	SmartPtr<rotatotope_NLP> mynlp = new rotatotope_NLP();
	mynlp->set_parameters(&links, q, q_dot, q_des, g_k, n_obstacles);

	// Create a new instance of IpoptApplication
    //  (use a SmartPtr, not raw)
    // We are using the factory, since this allows us to compile this
    // example with an Ipopt Windows DLL
    SmartPtr<IpoptApplication> app = IpoptApplicationFactory();

    // Change some options
    // Note: The following choices are only examples, they might not be
    //       suitable for your optimization problem.
	app->Options()->SetNumericValue("tol", 1e-6);
	app->Options()->SetNumericValue("max_cpu_time", 1);
	app->Options()->SetNumericValue("print_level", 0);
    app->Options()->SetStringValue("mu_strategy", "adaptive");
    app->Options()->SetStringValue("output_file", "ipopt.out");
	if(debugMode){
		app->Options()->SetStringValue("derivative_test", "second-order");
		app->Options()->SetNumericValue("derivative_test_perturbation", 0.000001);
	}

    // Initialize the IpoptApplication and process the options
    ApplicationReturnStatus status;
    status = app->Initialize();
    if( status != Solve_Succeeded ) {
		mexErrMsgIdAndTxt("MyProg:ConvertString", "*** Error during initialization!");
    }

    // Ask Ipopt to solve the problem
	//status = app->OptimizeTNLP(mynlp);
	
	//mynlp->try_joint_limits(k_opt);

	nlhs = 1;
	
    if( status == Solve_Succeeded ) {
        plhs[0] = mxCreateNumericMatrix(n_links * 2, 1, mxDOUBLE_CLASS, mxREAL);
		double *output0 = (double*)mxGetData(plhs[0]);
		for (uint32_t i = 0; i < n_links * 2; i++) {
			output0[i] = 0;//mynlp->solution[i];
		}
    }
    else {
		plhs[0] = mxCreateNumericMatrix(1, 1, mxINT32_CLASS, mxREAL);
		int *output0 = (int*)mxGetData(plhs[0]);
		*output0 = -12345;
	}

	//links.evaluate_self_constraints(k_opt);

	end_t = clock();
	mexPrintf("CUDA: IPOPT solve time: %.6f ms\n", 1000.0 * (end_t - start_t) / (double)(CLOCKS_PER_SEC));

	/*
P5. handle the output, release the memory
	*/
	if(debugMode){
		uint32_t link_id = 0;
		uint32_t RZ_length = links.RZ_length[link_id];

		if(links.debug_RZ == nullptr){
			mexErrMsgIdAndTxt("MyProg:ConvertString","*** debug_RZ is empty!");
		}

		mxArray* output1 = mxCreateCellMatrix(1, n_time_steps);
		for (uint32_t k = 0; k < n_time_steps; k++) {
			mxArray* time_step_k = mxCreateNumericMatrix(links.Z_width, RZ_length, mxDOUBLE_CLASS, mxREAL);
			double *pt = (double*)mxGetData(time_step_k);

			for (uint32_t t = 0; t < RZ_length; t++) {
				for (uint32_t p = 0; p < links.Z_width; p++) {
					pt[t * links.Z_width + p] = links.debug_RZ[(k * RZ_length + t) * links.Z_width + p];
				}
			}

			mxSetCell(output1, k, time_step_k);
		}
		plhs[1] = output1;

		mxArray* output2 = mxCreateCellMatrix(1, n_time_steps);
		for (uint32_t k = 0; k < n_time_steps; k++) {
			mxArray* time_step_k = mxCreateLogicalMatrix(1, RZ_length);
			bool *pt = (bool*)mxGetData(time_step_k);

			for (uint32_t t = 0; t < RZ_length; t++) {
				pt[t] = links.debug_c_idx[k * RZ_length + t];
			}

			mxSetCell(output2, k, time_step_k);
		}
		plhs[2] = output2;

		mxArray* output3 = mxCreateCellMatrix(1, n_time_steps);
		for (uint32_t k = 0; k < n_time_steps; k++) {
			mxArray* time_step_k = mxCreateLogicalMatrix(4 * (link_id + 1), RZ_length);
			bool *pt = (bool*)mxGetData(time_step_k);

			for (uint32_t t = 0; t < RZ_length; t++) {
				for (uint32_t p = 0; p < 2 * (link_id + 1); p++) {
					pt[t * 4 * (link_id + 1) + p] = links.debug_k_idx[(p * n_time_steps + k) * RZ_length + t];
				}
			}

			for (uint32_t t = 0; t < RZ_length; t++) {
				for (uint32_t p = 2 * (link_id + 1); p < 4 * (link_id + 1); p++) {
					pt[t * 4 * (link_id + 1) + p] = links.debug_C_idx[((p - 2 * (link_id + 1)) * n_time_steps + k) * RZ_length + t];
				}
			}

			mxSetCell(output3, k, time_step_k);
		}
		plhs[3] = output3;
		
		if(links.A_con[0] == nullptr){
			mexErrMsgIdAndTxt("MyProg:ConvertString","*** A_con is empty!");
		}

		mxArray* output4 = mxCreateCellMatrix(1, n_obstacles);
		for (uint32_t i = 0; i < n_obstacles; i++) {
			mxArray* obstacle_i = mxCreateCellMatrix(1, n_links);
			for (uint32_t j = 0; j < n_links; j++) {
				uint32_t RZ_length = links.RZ_length[j];
				uint32_t buff_obstacle_length = RZ_length + 3;
				uint32_t constraint_length = ((buff_obstacle_length - 1) * (buff_obstacle_length - 2)) / 2;
				mxArray* link_j = mxCreateCellMatrix(1, n_time_steps);
				for (uint32_t k = 0; k < n_time_steps; k++) {
					mxArray* time_step_k = mxCreateNumericMatrix(constraint_length, links.k_con_num[j][k], mxDOUBLE_CLASS, mxREAL);
					double *pt = (double*)mxGetData(time_step_k);

					for (uint32_t t = 0; t < links.k_con_num[j][k]; t++) {
						for (uint32_t p = 0; p < constraint_length; p++) {
							pt[t * constraint_length + p] = links.A_con[j][((i * n_time_steps + k) * constraint_length + p) * links.max_k_con_num[j] + t];
						}
					}

					mxSetCell(link_j, k, time_step_k);
				}

				mxSetCell(obstacle_i, j, link_j);
			}

			mxSetCell(output4, i, obstacle_i);
		}
		plhs[4] = output4;

		mxArray* output5 = mxCreateCellMatrix(1, n_obstacles);
		for (uint32_t i = 0; i < n_obstacles; i++) {
			mxArray* obstacle_i = mxCreateCellMatrix(1, n_links);
			for (uint32_t j = 0; j < n_links; j++) {
				uint32_t RZ_length = links.RZ_length[j];
				uint32_t buff_obstacle_length = RZ_length + 3;
				uint32_t constraint_length = ((buff_obstacle_length - 1) * (buff_obstacle_length - 2)) / 2;
				mxArray* link_j = mxCreateCellMatrix(1, n_time_steps);
				for (uint32_t k = 0; k < n_time_steps; k++) {
					mxArray* time_step_k = mxCreateNumericMatrix(constraint_length, 1, mxDOUBLE_CLASS, mxREAL);
					double *pt = (double*)mxGetData(time_step_k);

					for (uint32_t p = 0; p < constraint_length; p++) {
						pt[p] = links.d_con[j][(i * n_time_steps + k) * constraint_length + p];
					}

					mxSetCell(link_j, k, time_step_k);
				}

				mxSetCell(obstacle_i, j, link_j);
			}

			mxSetCell(output5, i, obstacle_i);
		}
		plhs[5] = output5;

		mxArray* output6 = mxCreateCellMatrix(1, n_obstacles);
		for (uint32_t i = 0; i < n_obstacles; i++) {
			mxArray* obstacle_i = mxCreateCellMatrix(1, n_links);
			for (uint32_t j = 0; j < n_links; j++) {
				uint32_t RZ_length = links.RZ_length[j];
				uint32_t buff_obstacle_length = RZ_length + 3;
				uint32_t constraint_length = ((buff_obstacle_length - 1) * (buff_obstacle_length - 2)) / 2;
				mxArray* link_j = mxCreateCellMatrix(1, n_time_steps);
				for (uint32_t k = 0; k < n_time_steps; k++) {
					mxArray* time_step_k = mxCreateNumericMatrix(constraint_length, 1, mxDOUBLE_CLASS, mxREAL);
					double *pt = (double*)mxGetData(time_step_k);

					for (uint32_t p = 0; p < constraint_length; p++) {
						pt[p] = links.delta_con[j][(i * n_time_steps + k) * constraint_length + p];
					}

					mxSetCell(link_j, k, time_step_k);
				}

				mxSetCell(obstacle_i, j, link_j);
			}

			mxSetCell(output6, i, obstacle_i);
		}
		plhs[6] = output6;

		mxArray* output7 = mxCreateCellMatrix(1, n_obstacles);
		for (uint32_t i = 0; i < n_obstacles; i++) {
			mxArray* obstacle_i = mxCreateCellMatrix(1, n_links);
			for (uint32_t j = 0; j < n_links; j++) {
				uint32_t RZ_length = links.RZ_length[j];
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

			mxSetCell(output7, i, obstacle_i);
		}
		plhs[7] = output7;
		
		links.evaluate_constraints(k_opt);
		
		plhs[8] = mxCreateNumericMatrix((n_links * n_obstacles + n_pairs) * n_time_steps, 1, mxDOUBLE_CLASS, mxREAL);
		double *output8 = (double*)mxGetData(plhs[8]);
		for (uint32_t i = 0; i < n_obstacles; i++) {
			for (uint32_t j = 0; j < n_links; j++) {
				for (uint32_t k = 0; k < n_time_steps; k++) {
					output8[(i * n_links + j) * n_time_steps + k] = links.con[(j * n_obstacles + i) * n_time_steps + k];
				}
			}
		}
		for(uint32_t i = 0; i < n_pairs; i++){
			for (uint32_t j = 0; j < n_time_steps; j++) {
				output8[i * n_time_steps + j + n_obstacles * n_links * n_time_steps] = links.con_self[i * n_time_steps + j];
			}
		}
		
		plhs[9] = mxCreateNumericMatrix(n_links * 2, (n_links * n_obstacles + n_pairs) * n_time_steps, mxDOUBLE_CLASS, mxREAL);
		double *output9 = (double*)mxGetData(plhs[9]);
		for (uint32_t i = 0; i < n_obstacles; i++) {
			for (uint32_t j = 0; j < n_links; j++) {
				for (uint32_t k = 0; k < n_time_steps; k++) {
					for (uint32_t p = 0; p < n_links * 2; p++) {
						output9[((i * n_links + j) * n_time_steps + k) * n_links * 2 + p] = links.jaco_con[((j * n_obstacles + i) * n_time_steps + k) * n_links * 2 + p];
					}
				}
			}
		}
		for (uint32_t i = 0; i < n_pairs; i++) {
			for (uint32_t j = 0; j < n_time_steps; j++) {
				for (uint32_t p = 0; p < n_links * 2; p++) {
					output9[(i * n_time_steps + j + n_obstacles * n_links * n_time_steps) * n_links * 2 + p] = links.jaco_con_self[(i * n_time_steps + j) * n_links * 2 + p];
				}
			}
		}

		plhs[10] = mxCreateNumericMatrix(n_links * (n_links * 2 - 1), (n_links * n_obstacles + n_pairs) * n_time_steps, mxDOUBLE_CLASS, mxREAL);
		double *output10 = (double*)mxGetData(plhs[10]);
		for (uint32_t i = 0; i < n_obstacles; i++) {
			for (uint32_t j = 0; j < n_links; j++) {
				for (uint32_t k = 0; k < n_time_steps; k++) {
					for (uint32_t p = 0; p < n_links * (n_links * 2 - 1); p++) {
						output10[((i * n_links + j) * n_time_steps + k) * n_links * (n_links * 2 - 1) + p] = links.hess_con[((j * n_obstacles + i) * n_time_steps + k) * n_links * (n_links * 2 - 1) + p];
					}
				}
			}
		}
		for (uint32_t i = 0; i < n_pairs; i++) {
			for (uint32_t j = 0; j < n_time_steps; j++) {
				for (uint32_t p = 0; p < n_links * (n_links * 2 - 1); p++) {
					output10[(i * n_time_steps + j + n_obstacles * n_links * n_time_steps) * n_links * (n_links * 2 - 1) + p] = links.hess_con_self[(i * n_time_steps + j) * n_links * (n_links * 2 - 1) + p];
				}
			}
		}

		if(links.A_con_self[0] == nullptr){
			mexErrMsgIdAndTxt("MyProg:ConvertString","*** A_con_self is empty!");
		}

		mxArray* output11 = mxCreateCellMatrix(1, n_pairs);
		for (uint32_t i = 0; i < n_pairs; i++) {
			mxArray* pair_i = mxCreateCellMatrix(1, n_time_steps);
			for (uint32_t k = 0; k < n_time_steps; k++) {
				uint32_t gen_zono_length = links.RZ_length[2];
				uint32_t constraint_length = ((gen_zono_length - 1) * (gen_zono_length - 2)) / 2;
				mxArray* time_step_k = mxCreateNumericMatrix(constraint_length, links.k_con_num_self[i][k], mxDOUBLE_CLASS, mxREAL);
				double *pt = (double*)mxGetData(time_step_k);

				for (uint32_t t = 0; t < links.k_con_num_self[i][k]; t++) {
					for (uint32_t p = 0; p < constraint_length; p++) {
						pt[t * constraint_length + p] = links.A_con_self[i][(k * constraint_length + p) * links.max_k_con_num_self[i] + t];
					}
				}

				mxSetCell(pair_i, k, time_step_k);
			}

			mxSetCell(output11, i, pair_i);
		}
		plhs[11] = output11;

		mxArray* output12 = mxCreateCellMatrix(1, n_pairs);
		for (uint32_t i = 0; i < n_pairs; i++) {
			mxArray* pair_i = mxCreateCellMatrix(1, n_time_steps);
			for (uint32_t k = 0; k < n_time_steps; k++) {
				uint32_t gen_zono_length = links.RZ_length[2];
				uint32_t constraint_length = ((gen_zono_length - 1) * (gen_zono_length - 2)) / 2;
				mxArray* time_step_k = mxCreateNumericMatrix(constraint_length, 1, mxDOUBLE_CLASS, mxREAL);
				double *pt = (double*)mxGetData(time_step_k);

				for (uint32_t p = 0; p < constraint_length; p++) {
					pt[p] = links.d_con_self[i][k * constraint_length + p];
				}

				mxSetCell(pair_i, k, time_step_k);
			}

			mxSetCell(output12, i, pair_i);
		}
		plhs[12] = output12;

		mxArray* output13 = mxCreateCellMatrix(1, n_pairs);
		for (uint32_t i = 0; i < n_pairs; i++) {
			mxArray* pair_i = mxCreateCellMatrix(1, n_time_steps);
			for (uint32_t k = 0; k < n_time_steps; k++) {
				uint32_t gen_zono_length = links.RZ_length[2];
				uint32_t constraint_length = ((gen_zono_length - 1) * (gen_zono_length - 2)) / 2;
				mxArray* time_step_k = mxCreateNumericMatrix(constraint_length, 1, mxDOUBLE_CLASS, mxREAL);
				double *pt = (double*)mxGetData(time_step_k);

				for (uint32_t p = 0; p < constraint_length; p++) {
					pt[p] = links.delta_con_self[i][k * constraint_length + p];
				}

				mxSetCell(pair_i, k, time_step_k);
			}

			mxSetCell(output13, i, pair_i);
		}
		plhs[13] = output13;

		mxArray* output14 = mxCreateCellMatrix(1, n_pairs);
		for (uint32_t i = 0; i < n_pairs; i++) {
			mxArray* pair_i = mxCreateCellMatrix(1, n_time_steps);
			for (uint32_t k = 0; k < n_time_steps; k++) {
				mxArray* time_step_k = mxCreateLogicalMatrix(2 * (2 + 1), links.k_con_num_self[i][k]);
				bool *pt = mxGetLogicals(time_step_k);

				for (uint32_t t = 0; t < links.k_con_num_self[i][k]; t++) {
					for (uint32_t p = 0; p < 2 * (2 + 1); p++) {
						pt[t * 2 * (2 + 1) + p] = links.k_con_self[i][(p * n_time_steps + k) * links.RZ_length[0] + t];
					}
				}

				mxSetCell(pair_i, k, time_step_k);
			}

			mxSetCell(output14, i, pair_i);
		}
		plhs[14] = output14;
	}

	cudaFree(dev_R);
	cudaFree(dev_rot_axes);
}

