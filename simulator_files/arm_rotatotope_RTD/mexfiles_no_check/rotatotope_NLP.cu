/*
Author: Bohao Zhang
Dec. 19 2019

arm_planning mex

ipopt nlp for rotatotopes
*/

#ifndef ROTATOTOPE_NLP_CPP
#define ROTATOTOPE_NLP_CPP

#include "rotatotope_NLP.h"

#ifdef __GNUC__
#pragma GCC diagnostic ignored "-Wunused-parameter"
#endif

// constructor
rotatotope_NLP::rotatotope_NLP()
{
    ra_info = nullptr;
    q = nullptr;
    q_dot = nullptr;
    q_des = nullptr;
    n_obstacles = 0;
    q_max = nullptr;
   q_min = nullptr;
   q_dot_max = nullptr;
   q_dot_min = nullptr;
   grad_q_max = nullptr;
   grad_q_min = nullptr;
   grad_q_dot_max = nullptr;
   grad_q_dot_min = nullptr;
    solution = nullptr;
}

// destructor
rotatotope_NLP::~rotatotope_NLP()
{
   delete[] q_max;
   delete[] q_min;
   delete[] q_dot_max;
   delete[] q_dot_min;
   
   delete[] grad_q_max;
   delete[] grad_q_min;
   delete[] grad_q_dot_max;
   delete[] grad_q_dot_min;

   delete[] solution;
}

 // [set_parameters]
 // set needed parameters
 bool rotatotope_NLP::set_parameters(
    rotatotopeArray* ra_input,
    double* q_input,
    double* q_dot_input,
    double* q_des_input,
    double* g_k_input,
    uint32_t n_obstacles_input
 )
 {
    ra_info = ra_input;
    q = q_input;
    q_dot = q_dot_input;
    q_des = q_des_input;
    g_k = g_k_input;
    n_obstacles = n_obstacles_input;
    uint32_t n_angles = ra_info->n_links * 2;
    q_max = new double[n_angles];
   q_min = new double[n_angles];
   q_dot_max = new double[n_angles];
   q_dot_min = new double[n_angles];
   
   grad_q_max = new double[n_angles];
   grad_q_min = new double[n_angles];
   grad_q_dot_max = new double[n_angles];
   grad_q_dot_min = new double[n_angles];
    return true;
 };

// [TNLP_get_nlp_info]
// returns the size of the problem
bool rotatotope_NLP::get_nlp_info(
   Index&          n,
   Index&          m,
   Index&          nnz_jac_g,
   Index&          nnz_h_lag,
   IndexStyleEnum& index_style
)
{
   // The problem described 6 variables, x[0] through x[5] for each joint
   n = ra_info->n_links * 2;

   // number of inequality constraint
   m = ra_info->n_links * n_obstacles * ra_info->n_time_steps + ra_info->n_pairs * ra_info->n_time_steps + ra_info->n_links * 8;

   nnz_jac_g = m * n;
   nnz_h_lag = n * (n + 1) / 2;

   // use the C style indexing (0-based)
   index_style = TNLP::C_STYLE;

   return true;
}
// [TNLP_get_nlp_info]

// [TNLP_get_bounds_info]
// returns the variable bounds
bool rotatotope_NLP::get_bounds_info(
   Index   n,
   Number* x_l,
   Number* x_u,
   Index   m,
   Number* g_l,
   Number* g_u
)
{
   // here, the n and m we gave IPOPT in get_nlp_info are passed back to us.
   // If desired, we could assert to make sure they are what we think they are.
   if(n != ra_info->n_links * 2){
      mexErrMsgIdAndTxt("MyProg:ConvertString", "*** Error wrong value of n in get_bounds_info!");
   }
   if(m != ra_info->n_links * n_obstacles * ra_info->n_time_steps + ra_info->n_pairs * ra_info->n_time_steps + ra_info->n_links * 8){
      mexErrMsgIdAndTxt("MyProg:ConvertString", "*** Error wrong value of m in get_bounds_info!");
   }

   // lower bounds
   for( Index i = 0; i < n; i++ ) {
      x_l[i] = -g_k[i];
   }

   // upper bounds  
   for( Index i = 0; i < n; i++ ) {
      x_u[i] = g_k[i];
   }

   for( Index i = 0; i < m; i++ ) {
      // constraint has a lower bound of inf
      g_l[i] = -2e19;
      // constraint has an upper bound of 0
      g_u[i] = 0;
   }

   return true;
}
// [TNLP_get_bounds_info]

// [TNLP_get_starting_point]
// returns the initial point for the problem
bool rotatotope_NLP::get_starting_point(
   Index   n,
   bool    init_x,
   Number* x,
   bool    init_z,
   Number* z_L,
   Number* z_U,
   Index   m,
   bool    init_lambda,
   Number* lambda
)
{
   // Here, we assume we only have starting values for x, if you code
   // your own NLP, you can provide starting values for the dual variables
   // if you wish
   if(init_x == false || init_z == true || init_lambda == true){
       mexErrMsgIdAndTxt("MyProg:ConvertString", "*** Error wrong value of init in get_starting_point!");
   }

   if(n != ra_info->n_links * 2){
      mexErrMsgIdAndTxt("MyProg:ConvertString", "*** Error wrong value of n in get_starting_point!");
   }

   // initialize to a random point
   std::srand(std::time(NULL));
   for( Index i = 0; i < n; i++ ) {
      x[i] = 2 * g_k[i] * ((double)std::rand() / RAND_MAX) - g_k[i];
   }

   return true;
}
// [TNLP_get_starting_point]

// [TNLP_eval_f]
// returns the value of the objective function
bool rotatotope_NLP::eval_f(
   Index         n,
   const Number* x,
   bool          new_x,
   Number&       obj_value
)
{
   if(n != ra_info->n_links * 2){
      mexErrMsgIdAndTxt("MyProg:ConvertString", "*** Error wrong value of n in eval_f!");
   }

   // q_plan = q + q_dot*P.t_plan + 0.5*k*P.t_plan^2;
   // q_plan_dot = q_dot + k*P.t_plan;
   // obj_value = sum((q_plan + 0.5 * q_plan_dot * (t_final - t_plan)- q_des).^2);
   obj_value = 0; 
   for(Index i = 0; i < ra_info->n_links * 2; i++){
      double q_plan = q[i] + q_dot[i] * t_plan + x[i] * t_plan * t_plan / 2;
      double q_plan_dot = q_dot[i] + x[i] * t_plan;
      double entry = q_plan + 0.5 * q_plan_dot * (t_total - t_plan) - q_des[i];
      obj_value += entry * entry;
   }

   return true;
}
// [TNLP_eval_f]

// [TNLP_eval_grad_f]
// return the gradient of the objective function grad_{x} f(x)
bool rotatotope_NLP::eval_grad_f(
   Index         n,
   const Number* x,
   bool          new_x,
   Number*       grad_f
)
{
   if(n != ra_info->n_links * 2){
      mexErrMsgIdAndTxt("MyProg:ConvertString", "*** Error wrong value of n in eval_grad_f!");
   }

   for(Index i = 0; i < ra_info->n_links * 2; i++){
      double q_plan = q[i] + q_dot[i] * t_plan + x[i] * t_plan * t_plan / 2;
      double q_plan_dot = q_dot[i] + x[i] * t_plan;
      double entry = q_plan + 0.5 * q_plan_dot * (t_total - t_plan) - q_des[i];
      grad_f[i] = t_plan * t_total * entry;
   }

   return true;
}
// [TNLP_eval_grad_f]

// [TNLP_eval_g]
// return the value of the constraints: g(x)
bool rotatotope_NLP::eval_g(
   Index         n,
   const Number* x,
   bool          new_x,
   Index         m,
   Number*       g
)
{
   if(n != ra_info->n_links * 2){
      mexErrMsgIdAndTxt("MyProg:ConvertString", "*** Error wrong value of n in eval_g!");
   }
   if(m != ra_info->n_links * n_obstacles * ra_info->n_time_steps + ra_info->n_pairs * ra_info->n_time_steps + n * 4){
      mexErrMsgIdAndTxt("MyProg:ConvertString", "*** Error wrong value of m in eval_g!");
   }
   
   bool compute_new_constraints = false;
   for(uint32_t i = 0; i < n; i++){
      if(ra_info->current_k_opt[i] != (double)x[i]){
            compute_new_constraints = true;
            break;
      }
   }

   if(compute_new_constraints){
      double* x_double = new double[n];
      for(uint32_t i = 0; i < n; i++){
         x_double[i] = (double)x[i];
      }
      ra_info->evaluate_constraints(x_double);
      delete[] x_double;

      compute_max_min_states(x);
   }

   Index offset = ra_info->n_links * n_obstacles * ra_info->n_time_steps;
   memcpy(g, ra_info->con, offset * sizeof(double));
   memcpy(g + offset, ra_info->con_self, ra_info->n_pairs * ra_info->n_time_steps * sizeof(double));

   offset += ra_info->n_pairs * ra_info->n_time_steps;
   for(Index i = offset; i < offset + n; i++) {
      g[i] = joint_state_limits[i - offset] - q_min[i - offset];
   }
   offset += n;
   for(Index i = offset; i < offset + n; i++) {
      g[i] = -joint_state_limits[i - offset + n] + q_max[i - offset];
   }
   offset += n;
   for(Index i = offset; i < offset + n; i++) {
      g[i] = joint_speed_limits[i - offset] - q_dot_min[i - offset];
   }
   offset += n;
   for(Index i = offset; i < offset + n; i++) {
      g[i] = -joint_speed_limits[i - offset + n] + q_dot_max[i - offset];
   }

   return true;
}
// [TNLP_eval_g]

// [TNLP_eval_jac_g]
// return the structure or values of the Jacobian
bool rotatotope_NLP::eval_jac_g(
   Index         n,
   const Number* x,
   bool          new_x,
   Index         m,
   Index         nele_jac,
   Index*        iRow,
   Index*        jCol,
   Number*       values
)
{
   if(n != ra_info->n_links * 2){
      mexErrMsgIdAndTxt("MyProg:ConvertString", "*** Error wrong value of n in eval_jac_g!");
   }
   if(m != ra_info->n_links * n_obstacles * ra_info->n_time_steps + ra_info->n_pairs * ra_info->n_time_steps + ra_info->n_links * 8){
      mexErrMsgIdAndTxt("MyProg:ConvertString", "*** Error wrong value of m in eval_jac_g!");
   }

   if( values == NULL ) {
      // return the structure of the Jacobian
      // this particular Jacobian is dense
      for(Index i = 0; i < m; i++){
          for(Index j = 0; j < n; j++){
              iRow[i * n + j] = i;
              jCol[i * n + j] = j;
          }
      }
   }
   else {
      bool compute_new_constraints = false;
      for(uint32_t i = 0; i < n; i++){
         if(ra_info->current_k_opt[i] != (double)x[i]){
               compute_new_constraints = true;
               break;
         }
      }
   
      if(compute_new_constraints){
         double* x_double = new double[n];
         for(uint32_t i = 0; i < n; i++){
            x_double[i] = (double)x[i];
         }
         ra_info->evaluate_constraints(x_double);
         delete[] x_double;

         compute_max_min_states(x);
      }
      
      // return the values of the Jacobian of the constraints
      Index offset = ra_info->n_links * n_obstacles * ra_info->n_time_steps;
      memcpy(values, ra_info->jaco_con, offset * n * sizeof(double));
      memcpy(values + offset * n, ra_info->jaco_con_self, ra_info->n_pairs * ra_info->n_time_steps * n * sizeof(double));

      offset += ra_info->n_pairs * ra_info->n_time_steps;
      for(Index i = offset; i < offset + n; i++) {
         for(Index j = 0; j < n; j++){
            if(i - offset == j){
               values[i * n + j] = -grad_q_min[j];
            }  
            else{
               values[i * n + j] = 0;
            }
         }
      }
      offset += n;
      for(Index i = offset; i < offset + n; i++) {
         for(Index j = 0; j < n; j++){
            if(i - offset == j){
               values[i * n + j] = grad_q_max[j];
            }  
            else{
               values[i * n + j] = 0;
            }
         }
      }
      offset += n;
      for(Index i = offset; i < offset + n; i++) {
         for(Index j = 0; j < n; j++){
            if(i - offset == j){
               values[i * n + j] = -grad_q_dot_min[j];
            }  
            else{
               values[i * n + j] = 0;
            }
         }
      }
      offset += n;
      for(Index i = offset; i < offset + n; i++) {
         for(Index j = 0; j < n; j++){
            if(i - offset == j){
               values[i * n + j] = grad_q_dot_max[j];
            }  
            else{
               values[i * n + j] = 0;
            }
         }
      }
   }

   return true;
}
// [TNLP_eval_jac_g]

// [TNLP_eval_h]
//return the structure or values of the Hessian
bool rotatotope_NLP::eval_h(
   Index         n,
   const Number* x,
   bool          new_x,
   Number        obj_factor,
   Index         m,
   const Number* lambda,
   bool          new_lambda,
   Index         nele_hess,
   Index*        iRow,
   Index*        jCol,
   Number*       values
)
{
   if(n != ra_info->n_links * 2){
      mexErrMsgIdAndTxt("MyProg:ConvertString", "*** Error wrong value of n in eval_h!");
   }
   if(m != ra_info->n_links * n_obstacles * ra_info->n_time_steps + ra_info->n_pairs * ra_info->n_time_steps + ra_info->n_links * 8){
      mexErrMsgIdAndTxt("MyProg:ConvertString", "*** Error wrong value of m in eval_h!");
   }

   if (values == NULL) {
      // return the structure. This is a symmetric matrix, fill the lower left
      // triangle only.
  
      // the Hessian for this problem is actually dense
      Index idx = 0;
      for (Index row = 0; row < n; row++) {
        for (Index col = 0; col <= row; col++) {
          iRow[idx] = row; 
          jCol[idx] = col;
          idx++;
        }
      }
      
      if(idx != nele_hess){
         mexErrMsgIdAndTxt("MyProg:ConvertString", "*** Error wrong size of hessian in eval_h!");
      }
    }
    else {
      bool compute_new_constraints = false;
      for(uint32_t i = 0; i < n; i++){
         if(ra_info->current_k_opt[i] != (double)x[i]){
               compute_new_constraints = true;
               break;
         }
      }
   
      if(compute_new_constraints){
         double* x_double = new double[n];
         for(uint32_t i = 0; i < n; i++){
            x_double[i] = (double)x[i];
         }
         ra_info->evaluate_constraints(x_double);
         delete[] x_double;
      }

      Index idx = 0;
      for (Index row = 0; row < n; row++) {
         for (Index col = 0; col <= row; col++) {
            if(row == col){
               values[idx] = obj_factor * t_plan * t_total * t_plan * t_total / 2;
            }
            else{
               values[idx] = 0;
            }
            idx++;
         }
      }

      Index offset = ra_info->n_links * n_obstacles * ra_info->n_time_steps;
      for(Index i = 0; i < offset; i++){
         idx = 0;
         Index hess_idx = 0;
         for (Index row = 0; row < n; row++) {
            for (Index col = 0; col <= row; col++) {
               if(row != col) {
                  values[idx] += lambda[i] * ra_info->hess_con[i * n * (n - 1) / 2 + hess_idx];
                  hess_idx++;
               }
               idx++;
            }
         }
      }

      for(Index i = offset; i < offset + ra_info->n_pairs * ra_info->n_time_steps; i++){
         idx = 0;
         Index hess_idx = 0;
         for (Index row = 0; row < n; row++) {
            for (Index col = 0; col <= row; col++) {
               if(row != col){
                  values[idx] += lambda[i] * ra_info->hess_con_self[(i - offset) * n * (n - 1) / 2 + hess_idx];
                  hess_idx++;
               }
               idx++;
            }
         }
      }

      // joint and speed limit has zero hessian
    }

   return true;
}
// [TNLP_eval_h]

// [TNLP_finalize_solution]
void rotatotope_NLP::finalize_solution(
   SolverReturn               status,
   Index                      n,
   const Number*              x,
   const Number*              z_L,
   const Number*              z_U,
   Index                      m,
   const Number*              g,
   const Number*              lambda,
   Number                     obj_value,
   const IpoptData*           ip_data,
   IpoptCalculatedQuantities* ip_cq
)
{
   // here is where we would store the solution to variables, or write to a file, etc
   // so we could use the solution.

   /*
   // For this example, we write the solution to the console
   mexPrintf("\nSolution of the primal variables, x\n\n");
   for( Index i = 0; i < n; i++ ) {
      mexPrintf( "x[%d] = %f\n", i, x[i]);
   }
   
   mexPrintf("\nSolution of the bound multipliers, z_L and z_U\n");
   for( Index i = 0; i < n; i++ )   
   {
      mexPrintf( "z_L[%d] = %f\n", i, z_L[i]);
   }
   for( Index i = 0; i < n; i++ )
   {
      std::cout << "z_U[" << i << "] = " << z_U[i] << std::endl;
   }

   std::cout << std::endl << std::endl << "Objective value" << std::endl;
   std::cout << "f(x*) = " << obj_value << std::endl;

   std::cout << std::endl << "Final value of the constraints:" << std::endl;
   for( Index i = 0; i < m; i++ )
   {
      std::cout << "g(" << i << ") = " << g[i] << std::endl;
   }
   */

   // store the solution
   solution = new double[n];
   for( Index i = 0; i < n; i++ ) {
      solution[i] = (double)x[i];
   }
}

void rotatotope_NLP::compute_max_min_states(const Number* k) {
   uint32_t n_angles = ra_info->n_links * 2;
   double t_to_stop = t_total - t_move;

   for (uint32_t i = 0; i < n_angles; i++){
      double q_peak = q[i] + q_dot[i] * t_move + k[i] * t_move * t_move * 0.5;
      double q_dot_peak = q_dot[i] + k[i] * t_move;
      double q_ddot_to_stop = -q_dot_peak / t_to_stop;
      double q_stop = q_peak + q_dot_peak * t_to_stop + 0.5 * q_ddot_to_stop * t_to_stop * t_to_stop;
      double t_max_min_to_peak = -q_dot[i] / k[i];

      double q_max_to_peak;
      double q_min_to_peak;
      double q_dot_max_to_peak;
      double q_dot_min_to_peak;
      
      double grad_q_max_to_peak;
      double grad_q_min_to_peak;
      double grad_q_dot_max_to_peak;
      double grad_q_dot_min_to_peak;
      
      double q_max_to_stop;
      double q_min_to_stop;
      double q_dot_max_to_stop;
      double q_dot_min_to_stop;
      
      double grad_q_max_to_stop;
      double grad_q_min_to_stop;
      double grad_q_dot_max_to_stop;
      double grad_q_dot_min_to_stop;

      double q_endpoints_ordered[2];
      double grad_q_endpoints_ordered[2];

      if (q_peak >= q[i]){
         q_endpoints_ordered[0] = q[i]; 
         q_endpoints_ordered[1] = q_peak;
         grad_q_endpoints_ordered[0] = 0; 
         grad_q_endpoints_ordered[1] = 0.5 * t_move * t_move;
      }
      else{
         q_endpoints_ordered[0] = q_peak; 
         q_endpoints_ordered[1] = q[i];
         grad_q_endpoints_ordered[0] = 0.5 * t_move * t_move; 
         grad_q_endpoints_ordered[1] = 0;
      }
      
      if (t_max_min_to_peak > 0 && t_max_min_to_peak < t_move){
         if (k[i] >= 0){
            q_min_to_peak = q[i] + q_dot[i] * t_max_min_to_peak + 0.5 * k[i] * t_max_min_to_peak * t_max_min_to_peak;
            q_max_to_peak  = q_endpoints_ordered[1];
            grad_q_min_to_peak = (0.5 * q_dot[i] * q_dot[i]) / (k[i] * k[i]);
            grad_q_max_to_peak = grad_q_endpoints_ordered[1];
         }
         else{
            q_min_to_peak = q_endpoints_ordered[0];
            q_max_to_peak = q[i] + q_dot[i] * t_max_min_to_peak + 0.5 * k[i] * t_max_min_to_peak * t_max_min_to_peak;
            grad_q_min_to_peak = grad_q_endpoints_ordered[0];
            grad_q_max_to_peak = (0.5 * q_dot[i] * q_dot[i]) / (k[i] * k[i]);
         }
      }
      else{
         q_min_to_peak = q_endpoints_ordered[0];
         q_max_to_peak = q_endpoints_ordered[1];
         
         grad_q_min_to_peak = grad_q_endpoints_ordered[0];
         grad_q_max_to_peak = grad_q_endpoints_ordered[1];
      }
      
      if( q_dot_peak >= q_dot[i]){
         q_dot_min_to_peak = q_dot[i];
         q_dot_max_to_peak = q_dot_peak;
         
         grad_q_dot_min_to_peak = 0;
         grad_q_dot_max_to_peak = t_move;
      }
      else{
         q_dot_min_to_peak = q_dot_peak;
         q_dot_max_to_peak = q_dot[i];
         
         grad_q_dot_min_to_peak = t_move;
         grad_q_dot_max_to_peak = 0;
      }

      if( q_stop >= q_peak){
         q_min_to_stop = q_peak;
         q_max_to_stop = q_stop;
         
         grad_q_min_to_stop = 0.5 * t_move * t_move;
         grad_q_max_to_stop = 0.5 * t_move * t_move + 0.5 * t_move * t_to_stop;
      }
      else{
         q_min_to_stop = q_stop;
         q_max_to_stop = q_peak;
         
         grad_q_min_to_stop = 0.5 * t_move * t_move + 0.5 * t_move * t_to_stop;
         grad_q_max_to_stop = 0.5 * t_move * t_move;
      }
      
      if(q_dot_peak >= 0){
         q_dot_min_to_stop = 0;
         q_dot_max_to_stop = q_dot_peak;
         
         grad_q_dot_min_to_stop = 0;
         grad_q_dot_max_to_stop = t_move;
      }
      else{
         q_dot_min_to_stop = q_dot_peak;
         q_dot_max_to_stop = 0;
         
         grad_q_dot_min_to_stop = t_move;
         grad_q_dot_max_to_stop = 0;
      }
      
      if (q_min_to_peak <= q_min_to_stop){
         q_min[i] = q_min_to_peak;
         grad_q_min[i] = grad_q_min_to_peak;
      }
      else{
         q_min[i] = q_min_to_stop;
         grad_q_min[i] = grad_q_min_to_stop;
      }
      
      if (q_max_to_peak >= q_max_to_stop){
         q_max[i] = q_max_to_peak;
         grad_q_max[i] = grad_q_max_to_peak;
      }
      else{
         q_max[i] = q_max_to_stop;
         grad_q_max[i] = grad_q_max_to_stop;
      }
      
      if (q_dot_min_to_peak <= q_dot_min_to_stop){
         q_dot_min[i] = q_dot_min_to_peak;
         grad_q_dot_min[i] = grad_q_dot_min_to_peak;
      }
      else{
         q_dot_min[i] = q_dot_min_to_stop;
         grad_q_dot_min[i] = grad_q_dot_min_to_stop;
      }
      
      if (q_dot_max_to_peak >= q_dot_max_to_stop){
         q_dot_max[i] = q_dot_max_to_peak;
         grad_q_dot_max[i] = grad_q_dot_max_to_peak;
      }
      else{
         q_dot_max[i] = q_dot_max_to_stop;
         grad_q_dot_max[i] = grad_q_dot_max_to_stop;
      }
   }
}

void rotatotope_NLP::try_joint_limits(double* k){
   compute_max_min_states(k);
 
   mexPrintf("values:\n");
   for(Index i = 0; i < 6; i++) {
      mexPrintf("%f\n", joint_state_limits[i] - q_min[i]);
   }
   for(Index i = 0; i < 6; i++) {
      mexPrintf("%f\n", -joint_state_limits[i + 6] + q_max[i]);
   }
   for(Index i = 0; i < 6; i++) {
      mexPrintf("%f\n", joint_speed_limits[i] - q_dot_min[i]);
   }
   for(Index i = 0; i < 6; i++) {
      mexPrintf("%f\n", -joint_speed_limits[i + 6] + q_dot_max[i]);
   }
   
   for(Index i = 0; i < 6; i++) {
      for(Index j = 0; j < 6; j++){
         if(i == j){
            std::cout<<-grad_q_min[j]<<" ";
         }  
         else{
            std::cout<<0<<" ";
         }
      }
      std::cout<<std::endl;
   }
   for(Index i = 0; i < 6; i++) {
      for(Index j = 0; j < 6; j++){
         if(i == j){
            std::cout<<grad_q_max[j]<<" ";
         }  
         else{
            std::cout<<0<<" ";
         }
      }
      std::cout<<std::endl;
   }
   for(Index i = 0; i < 6; i++) {
      for(Index j = 0; j < 6; j++){
         if(i == j){
            std::cout<<-grad_q_dot_min[j]<<" ";
         }  
         else{
            std::cout<<0<<" ";
         }
      }
      std::cout<<std::endl;
   }
   for(Index i = 0; i < 6; i++) {
      for(Index j = 0; j < 6; j++){
         if(i == j){
            std::cout<<grad_q_dot_max[j]<<" ";
         }  
         else{
            std::cout<<0<<" ";
         }
      }
      std::cout<<std::endl;
   }

}

#endif