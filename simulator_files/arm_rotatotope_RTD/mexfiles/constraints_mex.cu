/*
Author: Bohao Zhang
Oct. 30 2019

arm_planning mex

This code aims to generate constraint for rotatotopes
*/

#include "mex.h"
#include "rotatotopeArray.h"

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
	std::clock_t start_t, end_t; // timing
	/*
P0.	process the input
	*/
	if (nrhs != 5) {
		mexErrMsgIdAndTxt("MyProg:ConvertString", "Incorrect number of input!");
	}

	uint32_t n_links = (uint32_t)(*mxGetPr(prhs[0]));
	uint32_t n_joints = 2 * n_links;
	uint32_t n_time_steps = (uint32_t)(*mxGetPr(prhs[1]));

	double* RZ = mxGetPr(prhs[2]);
	uint32_t RZ_width = (uint32_t)mxGetM(prhs[2]);
	uint32_t RZ_length = (uint32_t)mxGetN(prhs[2]);

	double* c_idx = mxGetPr(prhs[3]);
	uint32_t c_idx_width = (uint32_t)mxGetM(prhs[3]);
	uint32_t c_idx_length = (uint32_t)mxGetN(prhs[3]);

	double* k_idx = mxGetPr(prhs[4]);
	uint32_t k_idx_width = (uint32_t)mxGetM(prhs[4]);
	uint32_t k_idx_length = (uint32_t)mxGetN(prhs[4]);


}

