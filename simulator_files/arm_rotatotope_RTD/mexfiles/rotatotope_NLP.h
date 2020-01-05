/*
Author: Bohao Zhang
Dec. 19 2019

arm_planning mex

ipopt nlp for rotatotopes
*/

#ifndef ROTATOTOPE_NLP_H
#define ROTATOTOPE_NLP_H

#include "rotatotopeArray.h"

#define t_plan 1.0
#define t_total 1.0
#define t_move 0.5

using namespace Ipopt;

class rotatotope_NLP: public TNLP
{
public:
   /** Default constructor */
   rotatotope_NLP();

   /** Default destructor */
   virtual ~rotatotope_NLP();

    // [set_parameters]
    bool set_parameters(
       rotatotopeArray* ra_input,
       double* q_input,
       double* q_dot_input,
       double* q_des_input,
       double* c_k_input,
       double* g_k_input,
       uint32_t n_obstacles_input
    );


   /**@name Overloaded from TNLP */
   //@{
   /** Method to return some info about the NLP */
   virtual bool get_nlp_info(
      Index&          n,
      Index&          m,
      Index&          nnz_jac_g,
      Index&          nnz_h_lag,
      IndexStyleEnum& index_style
   );

   /** Method to return the bounds for my problem */
   virtual bool get_bounds_info(
      Index   n,
      Number* x_l,
      Number* x_u,
      Index   m,
      Number* g_l,
      Number* g_u
   );

   /** Method to return the starting point for the algorithm */
   virtual bool get_starting_point(
      Index   n,
      bool    init_x,
      Number* x,
      bool    init_z,
      Number* z_L,
      Number* z_U,
      Index   m,
      bool    init_lambda,
      Number* lambda
   );

   /** Method to return the objective value */
   virtual bool eval_f(
      Index         n,
      const Number* x,
      bool          new_x,
      Number&       obj_value
   );

   /** Method to return the gradient of the objective */
   virtual bool eval_grad_f(
      Index         n,
      const Number* x,
      bool          new_x,
      Number*       grad_f
   );

   /** Method to return the constraint residuals */
   virtual bool eval_g(
      Index         n,
      const Number* x,
      bool          new_x,
      Index         m,
      Number*       g
   );

   /** Method to return:
    *   1) The structure of the jacobian (if "values" is NULL)
    *   2) The values of the jacobian (if "values" is not NULL)
    */
   virtual bool eval_jac_g(
      Index         n,
      const Number* x,
      bool          new_x,
      Index         m,
      Index         nele_jac,
      Index*        iRow,
      Index*        jCol,
      Number*       values
   );

   /** Method to return:
    *   1) The structure of the hessian of the lagrangian (if "values" is NULL)
    *   2) The values of the hessian of the lagrangian (if "values" is not NULL)
    */
   virtual bool eval_h(
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
   );

   /** This method is called when the algorithm is complete so the TNLP can store/write the solution */
   virtual void finalize_solution(
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
   );
   //@}

   void compute_max_min_states(const Number* k, double* &q_min, double* &q_max, double* &q_dot_min, double* &q_dot_max, double* &grad_q_min, double* &grad_q_max, double* &grad_q_dot_min, double* &grad_q_dot_max);

   double* solution;

private:
   /**@name Methods to block default compiler methods.
    *
    * The compiler automatically generates the following three methods.
    *  Since the default compiler implementation is generally not what
    *  you want (for all but the most simple classes), we usually
    *  put the declarations of these methods in the private section
    *  and never implement them. This prevents the compiler from
    *  implementing an incorrect "default" behavior without us
    *  knowing. (See Scott Meyers book, "Effective C++")
    */
   //@{
   rotatotope_NLP(
      const rotatotope_NLP&
   );

   rotatotope_NLP& operator=(
      const rotatotope_NLP&
   );

   rotatotopeArray* ra_info;

   double* q;

   double* q_dot;

   double* q_des;

   double* c_k;

   double* g_k;

   uint32_t n_obstacles;

   double* q_max;
   double* q_min;
   double* q_dot_max;
   double* q_dot_min;
   
   double* grad_q_max;
   double* grad_q_min;
   double* grad_q_dot_max;
   double* grad_q_dot_min;

   //@}
};

#endif