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
}

// destructor
rotatotope_NLP::~rotatotope_NLP()
{ }

 // [set_parameters]
 // set needed parameters
 bool rotatotope_NLP::set_parameters(
    rotatotopeArray* ra_input,
    double* q_input,
    double* q_dot_input,
    double* q_des_input,
    uint32_t n_obstacles_input
 )
 {
    ra_info = ra_input;
    q = q_input;
    q_dot = q_dot_input;
    q_des = q_des_input;
    n_obstacles = n_obstacles_input;
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
   m = ra_info->n_links * n_obstacles * ra_info->n_time_steps;

   // in this example the jacobian is dense and contains 8 nonzeros
   nnz_jac_g = 2; //? 

   // the Hessian is also dense and has 4 total nonzeros, but we
   // only need the lower left corner (since it is symmetric)
   nnz_h_lag = 3; // ?

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
   assert(n == ra_info->n_links * 2);
   assert(m == ra_info->n_links * n_obstacles * ra_info->n_time_steps);

   // the variables have lower bounds of 0 ?
   for( Index i = 0; i < n; i++ )
   {
      x_l[i] = 0.0;
   }

   // the variables have upper bounds of 1 ? 
   for( Index i = 0; i < n; i++ )
   {
      x_u[i] = 1.0;
   }

   // the first constraint g1 has a lower bound of inf
   g_l[0] = -2e19;
   // the first constraint g1 has an upper bound of 0
   g_u[0] = 0;

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
   assert(init_x == true);
   assert(init_z == false);
   assert(init_lambda == false);

   // initialize to the given starting point
   x[0] = 0.5;
   x[1] = 0.5;
   x[2] = 0.5;
   x[3] = 0.5;
   x[4] = 0.5;
   x[5] = 0.5;

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
   assert(n == ra_info->n_links * 2);

   // q_plan = q_0 + q_dot_0*P.t_plan + (1/2)*k*P.t_plan^2;
   // obj_value = sum((q_plan - q_des).^2);
   obj_value = 0; 
   for(uint32_t i = 0; i < ra_info->n_links * 2; i++){
       double entry = q[i] + q_dot[i] * t_plan + x[i] * t_plan * t_plan / 2 - q_des[i];
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
   assert(n == ra_info->n_links * 2);

   for(uint32_t i = 0; i < ra_info->n_links * 2; i++){
        double entry = q[i] + q_dot[i] * t_plan + x[i] * t_plan * t_plan / 2 - q_des[i];
        grad_f[i] = t_plan * t_plan * entry;
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
   assert(n == ra_info->n_links * 2);
   assert(m == ra_info->n_links * n_obstacles * ra_info->n_time_steps);

   bool compute_new_constraints;
   if(ra_info->current_k_opt != nullptr){
        compute_new_constraints = false;
        for(uint32_t i = 0; i < ra_info->n_links * 2; i++){
            if(ra_info->current_k_opt[i] != x[i]){
                compute_new_constraints = true;
                break;
            }
        }
    }
    else{
        compute_new_constraints = true;
    }

    if(compute_new_constraints){
        double* x_double = new double[ra_info->n_links * 2];
        for(uint32_t i = 0; i < ra_info->n_links * 2; i++){
            x_double[i] = x[i];
        }
        ra_info->evaluate_constraints(x_double);
        delete[] x_double;
    }

   for(uint32_t i = 0; i < ra_info->n_links * n_obstacles * ra_info->n_time_steps; i++){
        g[i] = ra_info->con[i];
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
    assert(n == ra_info->n_links * 2);
    assert(m == ra_info->n_links * n_obstacles * ra_info->n_time_steps);

    bool compute_new_constraints;
    if(ra_info->current_k_opt != nullptr){
         compute_new_constraints = false;
         for(uint32_t i = 0; i < ra_info->n_links * 2; i++){
             if(ra_info->current_k_opt[i] != x[i]){
                 compute_new_constraints = true;
                 break;
             }
         }
     }
     else{
         compute_new_constraints = true;
     }
 
     if(compute_new_constraints){
         double* x_double = new double[ra_info->n_links * 2];
         for(uint32_t i = 0; i < ra_info->n_links * 2; i++){
             x_double[i] = x[i];
         }
         ra_info->evaluate_constraints(x_double);
         delete[] x_double;
     }

   if( values == NULL )
   {
      // return the structure of the Jacobian

      // this particular Jacobian is dense
      for(Index i = 0; i < m; i++){
          for(Index j = 0; j < n; j++){
              iRow[i * n + j] = i;
              jCol[i * n + j] = j;
          }
      }
   }
   else
   {
      // return the values of the Jacobian of the constraints

      for(Index i = 0; i < m; i++){
          for(Index j = 0; j < n; j++){
              values[i * n + j] = ra_info->grad_con[i * n + j];
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
    assert(n == ra_info->n_links * 2);
    assert(m == ra_info->n_links * n_obstacles * ra_info->n_time_steps);

   if( values == NULL )
   {
      // return the structure. This is a symmetric matrix, fill the lower left
      // triangle only.

      // the hessian for this problem is actually dense
      Index idx = 0;
      for( Index row = 0; row < 2; row++ )
      {
         for( Index col = 0; col <= row; col++ )
         {
            iRow[idx] = row;
            jCol[idx] = col;
            idx++;
         }
      }

      assert(idx == nele_hess);
   }
   else
   {
      // return the values. This is a symmetric matrix, fill the lower left
      // triangle only

      // fill the objective portion
      values[0] = obj_factor * 2; // 0,0

      values[1] = 0.;     // 1,0
      values[2] = obj_factor * 2; // 1,1

      // values[3] = obj_factor * (x[3]);     // 2,0
      // values[4] = 0.;                      // 2,1
      // values[5] = 0.;                      // 2,2

      // values[6] = obj_factor * (2 * x[0] + x[1] + x[2]); // 3,0
      // values[7] = obj_factor * (x[0]);                   // 3,1
      // values[8] = obj_factor * (x[0]);                   // 3,2
      // values[9] = 0.;                                    // 3,3

      // // add the portion for the first constraint
      // values[1] += lambda[0] * (x[2] * x[3]); // 1,0

      // values[3] += lambda[0] * (x[1] * x[3]); // 2,0
      // values[4] += lambda[0] * (x[0] * x[3]); // 2,1

      // values[6] += lambda[0] * (x[1] * x[2]); // 3,0
      // values[7] += lambda[0] * (x[0] * x[2]); // 3,1
      // values[8] += lambda[0] * (x[0] * x[1]); // 3,2

      // // add the portion for the second constraint
      // values[0] += lambda[1] * 2; // 0,0

      // values[2] += lambda[1] * 2; // 1,1

      // values[5] += lambda[1] * 2; // 2,2

      // values[9] += lambda[1] * 2; // 3,3
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

   // For this example, we write the solution to the console
   std::cout << std::endl << std::endl << "Solution of the primal variables, x" << std::endl;
   for( Index i = 0; i < n; i++ )
   {
      std::cout << "x[" << i << "] = " << x[i] << std::endl;
   }

   std::cout << std::endl << std::endl << "Solution of the bound multipliers, z_L and z_U" << std::endl;
   for( Index i = 0; i < n; i++ )
   {
      std::cout << "z_L[" << i << "] = " << z_L[i] << std::endl;
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
}

#endif