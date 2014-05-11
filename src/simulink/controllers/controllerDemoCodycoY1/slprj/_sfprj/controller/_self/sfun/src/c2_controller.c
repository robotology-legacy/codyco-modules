/* Include files */

#include "blascompat32.h"
#include "controller_sfun.h"
#include "c2_controller.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "controller_sfun_debug_macros.h"

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static const char * c2_debug_family_names[51] = { "PINV_TOL", "gravAcc", "n",
  "St", "m", "grav", "xDcom", "xDDcomStar", "Pr", "A", "pinvA", "HwDotDes", "Jc",
  "JcDqD", "HDotDes", "handsDesiredWrenches", "feetDesiredWrenches", "Minv",
  "JcMinv", "PInv_JcMinvSt", "N0", "desiredExternalWrench",
  "measuredExternalWrench", "tau1", "LinearizedDynamics", "tau0", "nargin",
  "nargout", "demoState", "q", "qInit", "qD", "measuredExternalForces", "M", "h",
  "g", "H", "PosRightFoot", "feetJacobians", "handsJacobians", "feetJdqd",
  "handsJddq", "xcom", "Jcom", "Desired_x_dx_ddx_CoM", "kH", "kImp",
  "IntErrorCoM", "tau", "CoMError", "normCoMError" };

static const char * c2_b_debug_family_names[4] = { "nargin", "nargout", "w", "S"
};

/* Function Declarations */
static void initialize_c2_controller(SFc2_controllerInstanceStruct
  *chartInstance);
static void initialize_params_c2_controller(SFc2_controllerInstanceStruct
  *chartInstance);
static void enable_c2_controller(SFc2_controllerInstanceStruct *chartInstance);
static void disable_c2_controller(SFc2_controllerInstanceStruct *chartInstance);
static void c2_update_debugger_state_c2_controller(SFc2_controllerInstanceStruct
  *chartInstance);
static const mxArray *get_sim_state_c2_controller(SFc2_controllerInstanceStruct *
  chartInstance);
static void set_sim_state_c2_controller(SFc2_controllerInstanceStruct
  *chartInstance, const mxArray *c2_st);
static void finalize_c2_controller(SFc2_controllerInstanceStruct *chartInstance);
static void sf_c2_controller(SFc2_controllerInstanceStruct *chartInstance);
static void c2_chartstep_c2_controller(SFc2_controllerInstanceStruct
  *chartInstance);
static void initSimStructsc2_controller(SFc2_controllerInstanceStruct
  *chartInstance);
static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber);
static const mxArray *c2_sf_marshallOut(void *chartInstanceVoid, void *c2_inData);
static real_T c2_emlrt_marshallIn(SFc2_controllerInstanceStruct *chartInstance,
  const mxArray *c2_normCoMError, const char_T *c2_identifier);
static real_T c2_b_emlrt_marshallIn(SFc2_controllerInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_b_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_c_emlrt_marshallIn(SFc2_controllerInstanceStruct *chartInstance,
  const mxArray *c2_CoMError, const char_T *c2_identifier, real_T c2_y[3]);
static void c2_d_emlrt_marshallIn(SFc2_controllerInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[3]);
static void c2_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_c_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_e_emlrt_marshallIn(SFc2_controllerInstanceStruct *chartInstance,
  const mxArray *c2_tau, const char_T *c2_identifier, real_T c2_y[25]);
static void c2_f_emlrt_marshallIn(SFc2_controllerInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[25]);
static void c2_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_d_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static const mxArray *c2_e_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static const mxArray *c2_f_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static const mxArray *c2_g_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static const mxArray *c2_h_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static const mxArray *c2_i_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static const mxArray *c2_j_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static const mxArray *c2_k_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static const mxArray *c2_l_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static const mxArray *c2_m_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static const mxArray *c2_n_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_g_emlrt_marshallIn(SFc2_controllerInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[18]);
static void c2_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_o_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_h_emlrt_marshallIn(SFc2_controllerInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[625]);
static void c2_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_p_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_i_emlrt_marshallIn(SFc2_controllerInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[450]);
static void c2_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_q_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_j_emlrt_marshallIn(SFc2_controllerInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[558]);
static void c2_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static void c2_k_emlrt_marshallIn(SFc2_controllerInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[961]);
static void c2_h_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_r_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_l_emlrt_marshallIn(SFc2_controllerInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[12]);
static void c2_i_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static void c2_m_emlrt_marshallIn(SFc2_controllerInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[6]);
static void c2_j_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_s_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_n_emlrt_marshallIn(SFc2_controllerInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[72]);
static void c2_k_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_t_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_o_emlrt_marshallIn(SFc2_controllerInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[72]);
static void c2_l_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_u_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_p_emlrt_marshallIn(SFc2_controllerInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[9]);
static void c2_m_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static void c2_info_helper(c2_ResolvedFunctionInfo c2_info[268]);
static void c2_b_info_helper(c2_ResolvedFunctionInfo c2_info[268]);
static void c2_c_info_helper(c2_ResolvedFunctionInfo c2_info[268]);
static void c2_d_info_helper(c2_ResolvedFunctionInfo c2_info[268]);
static void c2_e_info_helper(c2_ResolvedFunctionInfo c2_info[268]);
static void c2_isVariableSizing(SFc2_controllerInstanceStruct *chartInstance);
static void c2_eml_scalar_eg(SFc2_controllerInstanceStruct *chartInstance);
static void c2_check_forloop_overflow_error(SFc2_controllerInstanceStruct
  *chartInstance);
static void c2_b_eml_scalar_eg(SFc2_controllerInstanceStruct *chartInstance);
static boolean_T c2_eml_use_refblas(SFc2_controllerInstanceStruct *chartInstance);
static void c2_eye(SFc2_controllerInstanceStruct *chartInstance, real_T c2_I[9]);
static void c2_Sf(SFc2_controllerInstanceStruct *chartInstance, real_T c2_w[3],
                  real_T c2_S[9]);
static void c2_c_eml_scalar_eg(SFc2_controllerInstanceStruct *chartInstance);
static boolean_T c2_isfinite(SFc2_controllerInstanceStruct *chartInstance,
  real_T c2_x);
static void c2_eml_error(SFc2_controllerInstanceStruct *chartInstance);
static real_T c2_abs(SFc2_controllerInstanceStruct *chartInstance, real_T c2_x);
static void c2_realmin(SFc2_controllerInstanceStruct *chartInstance);
static void c2_pinv(SFc2_controllerInstanceStruct *chartInstance, real_T c2_A[72],
                    real_T c2_X[72]);
static void c2_eml_xgesvd(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_A[72], real_T c2_U[72], real_T c2_S[6], real_T c2_V[36]);
static real_T c2_eml_xnrm2(SFc2_controllerInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_x[72], int32_T c2_ix0);
static void c2_b_check_forloop_overflow_error(SFc2_controllerInstanceStruct
  *chartInstance, boolean_T c2_overflow);
static real_T c2_eml_div(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_x, real_T c2_y);
static void c2_eml_xscal(SFc2_controllerInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_a, real_T c2_x[72], int32_T c2_ix0, real_T c2_b_x[72]);
static real_T c2_eml_xdotc(SFc2_controllerInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_x[72], int32_T c2_ix0, real_T c2_y[72], int32_T c2_iy0);
static void c2_eml_xaxpy(SFc2_controllerInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_a, int32_T c2_ix0, real_T c2_y[72], int32_T c2_iy0, real_T
  c2_b_y[72]);
static real_T c2_b_eml_xnrm2(SFc2_controllerInstanceStruct *chartInstance,
  int32_T c2_n, real_T c2_x[6], int32_T c2_ix0);
static void c2_b_eml_xscal(SFc2_controllerInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_a, real_T c2_x[6], int32_T c2_ix0, real_T c2_b_x[6]);
static void c2_b_eml_xaxpy(SFc2_controllerInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_a, real_T c2_x[72], int32_T c2_ix0, real_T c2_y[12], int32_T
  c2_iy0, real_T c2_b_y[12]);
static void c2_c_eml_xaxpy(SFc2_controllerInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_a, real_T c2_x[12], int32_T c2_ix0, real_T c2_y[72], int32_T
  c2_iy0, real_T c2_b_y[72]);
static real_T c2_b_eml_xdotc(SFc2_controllerInstanceStruct *chartInstance,
  int32_T c2_n, real_T c2_x[36], int32_T c2_ix0, real_T c2_y[36], int32_T c2_iy0);
static void c2_d_eml_xaxpy(SFc2_controllerInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_a, int32_T c2_ix0, real_T c2_y[36], int32_T c2_iy0, real_T
  c2_b_y[36]);
static void c2_d_eml_scalar_eg(SFc2_controllerInstanceStruct *chartInstance);
static void c2_c_eml_xscal(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_a, real_T c2_x[36], int32_T c2_ix0, real_T c2_b_x[36]);
static void c2_eps(SFc2_controllerInstanceStruct *chartInstance);
static void c2_b_eml_error(SFc2_controllerInstanceStruct *chartInstance);
static real_T c2_sqrt(SFc2_controllerInstanceStruct *chartInstance, real_T c2_x);
static void c2_c_eml_error(SFc2_controllerInstanceStruct *chartInstance);
static void c2_eml_xrotg(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_a, real_T c2_b, real_T *c2_b_a, real_T *c2_b_b, real_T *c2_c, real_T *c2_s);
static void c2_eml_xrot(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_x[36], int32_T c2_ix0, int32_T c2_iy0, real_T c2_c, real_T c2_s, real_T
  c2_b_x[36]);
static void c2_b_eml_xrot(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_x[72], int32_T c2_ix0, int32_T c2_iy0, real_T c2_c, real_T c2_s, real_T
  c2_b_x[72]);
static void c2_eml_xswap(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_x[36], int32_T c2_ix0, int32_T c2_iy0, real_T c2_b_x[36]);
static void c2_b_eml_xswap(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_x[72], int32_T c2_ix0, int32_T c2_iy0, real_T c2_b_x[72]);
static void c2_eml_xgemm(SFc2_controllerInstanceStruct *chartInstance, int32_T
  c2_k, real_T c2_A[36], real_T c2_B[72], real_T c2_C[72], real_T c2_b_C[72]);
static void c2_e_eml_scalar_eg(SFc2_controllerInstanceStruct *chartInstance);
static void c2_inv(SFc2_controllerInstanceStruct *chartInstance, real_T c2_x[961],
                   real_T c2_y[961]);
static void c2_invNxN(SFc2_controllerInstanceStruct *chartInstance, real_T c2_x
                      [961], real_T c2_y[961]);
static void c2_eml_matlab_zgetrf(SFc2_controllerInstanceStruct *chartInstance,
  real_T c2_A[961], real_T c2_b_A[961], int32_T c2_ipiv[31], int32_T *c2_info);
static void c2_c_eml_xswap(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_x[961], int32_T c2_ix0, int32_T c2_iy0, real_T c2_b_x[961]);
static void c2_below_threshold(SFc2_controllerInstanceStruct *chartInstance);
static void c2_eml_xger(SFc2_controllerInstanceStruct *chartInstance, int32_T
  c2_m, int32_T c2_n, real_T c2_alpha1, int32_T c2_ix0, int32_T c2_iy0, real_T
  c2_A[961], int32_T c2_ia0, real_T c2_b_A[961]);
static void c2_eml_ipiv2perm(SFc2_controllerInstanceStruct *chartInstance,
  int32_T c2_ipiv[31], int32_T c2_perm[31]);
static void c2_f_eml_scalar_eg(SFc2_controllerInstanceStruct *chartInstance);
static void c2_eml_xtrsm(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_A[961], real_T c2_B[961], real_T c2_b_B[961]);
static real_T c2_norm(SFc2_controllerInstanceStruct *chartInstance, real_T c2_x
                      [961]);
static void c2_eml_warning(SFc2_controllerInstanceStruct *chartInstance);
static void c2_b_eml_warning(SFc2_controllerInstanceStruct *chartInstance,
  char_T c2_varargin_2[14]);
static void c2_g_eml_scalar_eg(SFc2_controllerInstanceStruct *chartInstance);
static void c2_b_eml_xgemm(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_A[558], real_T c2_B[961], real_T c2_C[558], real_T c2_b_C[558]);
static void c2_h_eml_scalar_eg(SFc2_controllerInstanceStruct *chartInstance);
static void c2_c_eml_xgemm(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_A[558], real_T c2_B[775], real_T c2_C[450], real_T c2_b_C[450]);
static void c2_b_pinv(SFc2_controllerInstanceStruct *chartInstance, real_T c2_A
                      [450], real_T c2_tol, real_T c2_X[450]);
static void c2_i_eml_scalar_eg(SFc2_controllerInstanceStruct *chartInstance);
static void c2_svd(SFc2_controllerInstanceStruct *chartInstance, real_T c2_A[450],
                   real_T c2_U[450], real_T c2_S[324], real_T c2_V[324]);
static void c2_b_eml_xgesvd(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_A[450], real_T c2_U[450], real_T c2_S[18], real_T c2_V[324]);
static real_T c2_c_eml_xnrm2(SFc2_controllerInstanceStruct *chartInstance,
  int32_T c2_n, real_T c2_x[450], int32_T c2_ix0);
static void c2_d_eml_xscal(SFc2_controllerInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_a, real_T c2_x[450], int32_T c2_ix0, real_T c2_b_x[450]);
static real_T c2_c_eml_xdotc(SFc2_controllerInstanceStruct *chartInstance,
  int32_T c2_n, real_T c2_x[450], int32_T c2_ix0, real_T c2_y[450], int32_T
  c2_iy0);
static void c2_e_eml_xaxpy(SFc2_controllerInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_a, int32_T c2_ix0, real_T c2_y[450], int32_T c2_iy0, real_T
  c2_b_y[450]);
static real_T c2_d_eml_xnrm2(SFc2_controllerInstanceStruct *chartInstance,
  int32_T c2_n, real_T c2_x[18], int32_T c2_ix0);
static void c2_e_eml_xscal(SFc2_controllerInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_a, real_T c2_x[18], int32_T c2_ix0, real_T c2_b_x[18]);
static void c2_f_eml_xaxpy(SFc2_controllerInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_a, real_T c2_x[450], int32_T c2_ix0, real_T c2_y[25], int32_T
  c2_iy0, real_T c2_b_y[25]);
static void c2_g_eml_xaxpy(SFc2_controllerInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_a, real_T c2_x[25], int32_T c2_ix0, real_T c2_y[450], int32_T
  c2_iy0, real_T c2_b_y[450]);
static real_T c2_d_eml_xdotc(SFc2_controllerInstanceStruct *chartInstance,
  int32_T c2_n, real_T c2_x[324], int32_T c2_ix0, real_T c2_y[324], int32_T
  c2_iy0);
static void c2_h_eml_xaxpy(SFc2_controllerInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_a, int32_T c2_ix0, real_T c2_y[324], int32_T c2_iy0, real_T
  c2_b_y[324]);
static void c2_j_eml_scalar_eg(SFc2_controllerInstanceStruct *chartInstance);
static void c2_f_eml_xscal(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_a, real_T c2_x[324], int32_T c2_ix0, real_T c2_b_x[324]);
static void c2_c_eml_xrot(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_x[324], int32_T c2_ix0, int32_T c2_iy0, real_T c2_c, real_T c2_s, real_T
  c2_b_x[324]);
static void c2_d_eml_xrot(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_x[450], int32_T c2_ix0, int32_T c2_iy0, real_T c2_c, real_T c2_s, real_T
  c2_b_x[450]);
static void c2_d_eml_xswap(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_x[324], int32_T c2_ix0, int32_T c2_iy0, real_T c2_b_x[324]);
static void c2_e_eml_xswap(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_x[450], int32_T c2_ix0, int32_T c2_iy0, real_T c2_b_x[450]);
static void c2_d_eml_xgemm(SFc2_controllerInstanceStruct *chartInstance, int32_T
  c2_k, real_T c2_A[324], real_T c2_B[450], real_T c2_C[450], real_T c2_b_C[450]);
static void c2_b_below_threshold(SFc2_controllerInstanceStruct *chartInstance);
static void c2_b_eye(SFc2_controllerInstanceStruct *chartInstance, real_T c2_I
                     [625]);
static void c2_k_eml_scalar_eg(SFc2_controllerInstanceStruct *chartInstance);
static void c2_e_eml_xgemm(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_A[450], real_T c2_B[558], real_T c2_C[775], real_T c2_b_C[775]);
static void c2_l_eml_scalar_eg(SFc2_controllerInstanceStruct *chartInstance);
static void c2_f_eml_xgemm(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_A[775], real_T c2_B[775], real_T c2_C[625], real_T c2_b_C[625]);
static void c2_m_eml_scalar_eg(SFc2_controllerInstanceStruct *chartInstance);
static void c2_g_eml_xgemm(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_A[558], real_T c2_B[31], real_T c2_C[18], real_T c2_b_C[18]);
static void c2_n_eml_scalar_eg(SFc2_controllerInstanceStruct *chartInstance);
static void c2_h_eml_xgemm(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_A[558], real_T c2_B[558], real_T c2_C[324], real_T c2_b_C[324]);
static void c2_o_eml_scalar_eg(SFc2_controllerInstanceStruct *chartInstance);
static void c2_p_eml_scalar_eg(SFc2_controllerInstanceStruct *chartInstance);
static void c2_q_eml_scalar_eg(SFc2_controllerInstanceStruct *chartInstance);
static void c2_i_eml_xgemm(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_A[625], real_T c2_B[25], real_T c2_C[25], real_T c2_b_C[25]);
static real_T c2_b_norm(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_x[3]);
static void c2_q_emlrt_marshallIn(SFc2_controllerInstanceStruct *chartInstance,
  const mxArray *c2_sprintf, const char_T *c2_identifier, char_T c2_y[14]);
static void c2_r_emlrt_marshallIn(SFc2_controllerInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, char_T c2_y[14]);
static const mxArray *c2_v_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static int32_T c2_s_emlrt_marshallIn(SFc2_controllerInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_n_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static uint8_T c2_t_emlrt_marshallIn(SFc2_controllerInstanceStruct
  *chartInstance, const mxArray *c2_b_is_active_c2_controller, const char_T
  *c2_identifier);
static uint8_T c2_u_emlrt_marshallIn(SFc2_controllerInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_g_eml_xscal(SFc2_controllerInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_a, real_T c2_x[72], int32_T c2_ix0);
static void c2_i_eml_xaxpy(SFc2_controllerInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_a, int32_T c2_ix0, real_T c2_y[72], int32_T c2_iy0);
static void c2_h_eml_xscal(SFc2_controllerInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_a, real_T c2_x[6], int32_T c2_ix0);
static void c2_j_eml_xaxpy(SFc2_controllerInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_a, real_T c2_x[72], int32_T c2_ix0, real_T c2_y[12], int32_T
  c2_iy0);
static void c2_k_eml_xaxpy(SFc2_controllerInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_a, real_T c2_x[12], int32_T c2_ix0, real_T c2_y[72], int32_T
  c2_iy0);
static void c2_l_eml_xaxpy(SFc2_controllerInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_a, int32_T c2_ix0, real_T c2_y[36], int32_T c2_iy0);
static void c2_i_eml_xscal(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_a, real_T c2_x[36], int32_T c2_ix0);
static void c2_b_sqrt(SFc2_controllerInstanceStruct *chartInstance, real_T *c2_x);
static void c2_b_eml_xrotg(SFc2_controllerInstanceStruct *chartInstance, real_T *
  c2_a, real_T *c2_b, real_T *c2_c, real_T *c2_s);
static void c2_e_eml_xrot(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_x[36], int32_T c2_ix0, int32_T c2_iy0, real_T c2_c, real_T c2_s);
static void c2_f_eml_xrot(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_x[72], int32_T c2_ix0, int32_T c2_iy0, real_T c2_c, real_T c2_s);
static void c2_f_eml_xswap(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_x[36], int32_T c2_ix0, int32_T c2_iy0);
static void c2_g_eml_xswap(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_x[72], int32_T c2_ix0, int32_T c2_iy0);
static void c2_j_eml_xgemm(SFc2_controllerInstanceStruct *chartInstance, int32_T
  c2_k, real_T c2_A[36], real_T c2_B[72], real_T c2_C[72]);
static void c2_b_eml_matlab_zgetrf(SFc2_controllerInstanceStruct *chartInstance,
  real_T c2_A[961], int32_T c2_ipiv[31], int32_T *c2_info);
static void c2_h_eml_xswap(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_x[961], int32_T c2_ix0, int32_T c2_iy0);
static void c2_b_eml_xger(SFc2_controllerInstanceStruct *chartInstance, int32_T
  c2_m, int32_T c2_n, real_T c2_alpha1, int32_T c2_ix0, int32_T c2_iy0, real_T
  c2_A[961], int32_T c2_ia0);
static void c2_b_eml_xtrsm(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_A[961], real_T c2_B[961]);
static void c2_k_eml_xgemm(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_A[558], real_T c2_B[961], real_T c2_C[558]);
static void c2_l_eml_xgemm(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_A[558], real_T c2_B[775], real_T c2_C[450]);
static void c2_j_eml_xscal(SFc2_controllerInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_a, real_T c2_x[450], int32_T c2_ix0);
static void c2_m_eml_xaxpy(SFc2_controllerInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_a, int32_T c2_ix0, real_T c2_y[450], int32_T c2_iy0);
static void c2_k_eml_xscal(SFc2_controllerInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_a, real_T c2_x[18], int32_T c2_ix0);
static void c2_n_eml_xaxpy(SFc2_controllerInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_a, real_T c2_x[450], int32_T c2_ix0, real_T c2_y[25], int32_T
  c2_iy0);
static void c2_o_eml_xaxpy(SFc2_controllerInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_a, real_T c2_x[25], int32_T c2_ix0, real_T c2_y[450], int32_T
  c2_iy0);
static void c2_p_eml_xaxpy(SFc2_controllerInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_a, int32_T c2_ix0, real_T c2_y[324], int32_T c2_iy0);
static void c2_l_eml_xscal(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_a, real_T c2_x[324], int32_T c2_ix0);
static void c2_g_eml_xrot(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_x[324], int32_T c2_ix0, int32_T c2_iy0, real_T c2_c, real_T c2_s);
static void c2_h_eml_xrot(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_x[450], int32_T c2_ix0, int32_T c2_iy0, real_T c2_c, real_T c2_s);
static void c2_i_eml_xswap(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_x[324], int32_T c2_ix0, int32_T c2_iy0);
static void c2_j_eml_xswap(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_x[450], int32_T c2_ix0, int32_T c2_iy0);
static void c2_m_eml_xgemm(SFc2_controllerInstanceStruct *chartInstance, int32_T
  c2_k, real_T c2_A[324], real_T c2_B[450], real_T c2_C[450]);
static void c2_n_eml_xgemm(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_A[450], real_T c2_B[558], real_T c2_C[775]);
static void c2_o_eml_xgemm(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_A[775], real_T c2_B[775], real_T c2_C[625]);
static void c2_p_eml_xgemm(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_A[558], real_T c2_B[31], real_T c2_C[18]);
static void c2_q_eml_xgemm(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_A[558], real_T c2_B[558], real_T c2_C[324]);
static void c2_r_eml_xgemm(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_A[625], real_T c2_B[25], real_T c2_C[25]);
static void init_dsm_address_info(SFc2_controllerInstanceStruct *chartInstance);

/* Function Definitions */
static void initialize_c2_controller(SFc2_controllerInstanceStruct
  *chartInstance)
{
  chartInstance->c2_sfEvent = CALL_EVENT;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c2_is_active_c2_controller = 0U;
}

static void initialize_params_c2_controller(SFc2_controllerInstanceStruct
  *chartInstance)
{
}

static void enable_c2_controller(SFc2_controllerInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c2_controller(SFc2_controllerInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c2_update_debugger_state_c2_controller(SFc2_controllerInstanceStruct
  *chartInstance)
{
}

static const mxArray *get_sim_state_c2_controller(SFc2_controllerInstanceStruct *
  chartInstance)
{
  const mxArray *c2_st;
  const mxArray *c2_y = NULL;
  int32_T c2_i0;
  real_T c2_u[3];
  const mxArray *c2_b_y = NULL;
  real_T c2_hoistedGlobal;
  real_T c2_b_u;
  const mxArray *c2_c_y = NULL;
  int32_T c2_i1;
  real_T c2_c_u[25];
  const mxArray *c2_d_y = NULL;
  uint8_T c2_b_hoistedGlobal;
  uint8_T c2_d_u;
  const mxArray *c2_e_y = NULL;
  real_T *c2_normCoMError;
  real_T (*c2_tau)[25];
  real_T (*c2_CoMError)[3];
  c2_normCoMError = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
  c2_CoMError = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 2);
  c2_tau = (real_T (*)[25])ssGetOutputPortSignal(chartInstance->S, 1);
  c2_st = NULL;
  c2_st = NULL;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_createcellarray(4), FALSE);
  for (c2_i0 = 0; c2_i0 < 3; c2_i0++) {
    c2_u[c2_i0] = (*c2_CoMError)[c2_i0];
  }

  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 3), FALSE);
  sf_mex_setcell(c2_y, 0, c2_b_y);
  c2_hoistedGlobal = *c2_normCoMError;
  c2_b_u = c2_hoistedGlobal;
  c2_c_y = NULL;
  sf_mex_assign(&c2_c_y, sf_mex_create("y", &c2_b_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c2_y, 1, c2_c_y);
  for (c2_i1 = 0; c2_i1 < 25; c2_i1++) {
    c2_c_u[c2_i1] = (*c2_tau)[c2_i1];
  }

  c2_d_y = NULL;
  sf_mex_assign(&c2_d_y, sf_mex_create("y", c2_c_u, 0, 0U, 1U, 0U, 1, 25), FALSE);
  sf_mex_setcell(c2_y, 2, c2_d_y);
  c2_b_hoistedGlobal = chartInstance->c2_is_active_c2_controller;
  c2_d_u = c2_b_hoistedGlobal;
  c2_e_y = NULL;
  sf_mex_assign(&c2_e_y, sf_mex_create("y", &c2_d_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c2_y, 3, c2_e_y);
  sf_mex_assign(&c2_st, c2_y, FALSE);
  return c2_st;
}

static void set_sim_state_c2_controller(SFc2_controllerInstanceStruct
  *chartInstance, const mxArray *c2_st)
{
  const mxArray *c2_u;
  real_T c2_dv0[3];
  int32_T c2_i2;
  real_T c2_dv1[25];
  int32_T c2_i3;
  real_T *c2_normCoMError;
  real_T (*c2_CoMError)[3];
  real_T (*c2_tau)[25];
  c2_normCoMError = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
  c2_CoMError = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 2);
  c2_tau = (real_T (*)[25])ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c2_doneDoubleBufferReInit = TRUE;
  c2_u = sf_mex_dup(c2_st);
  c2_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 0)),
                        "CoMError", c2_dv0);
  for (c2_i2 = 0; c2_i2 < 3; c2_i2++) {
    (*c2_CoMError)[c2_i2] = c2_dv0[c2_i2];
  }

  *c2_normCoMError = c2_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c2_u, 1)), "normCoMError");
  c2_e_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 2)),
                        "tau", c2_dv1);
  for (c2_i3 = 0; c2_i3 < 25; c2_i3++) {
    (*c2_tau)[c2_i3] = c2_dv1[c2_i3];
  }

  chartInstance->c2_is_active_c2_controller = c2_t_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 3)),
     "is_active_c2_controller");
  sf_mex_destroy(&c2_u);
  c2_update_debugger_state_c2_controller(chartInstance);
  sf_mex_destroy(&c2_st);
}

static void finalize_c2_controller(SFc2_controllerInstanceStruct *chartInstance)
{
}

static void sf_c2_controller(SFc2_controllerInstanceStruct *chartInstance)
{
  int32_T c2_i4;
  int32_T c2_i5;
  int32_T c2_i6;
  int32_T c2_i7;
  int32_T c2_i8;
  int32_T c2_i9;
  int32_T c2_i10;
  int32_T c2_i11;
  int32_T c2_i12;
  int32_T c2_i13;
  int32_T c2_i14;
  int32_T c2_i15;
  int32_T c2_i16;
  int32_T c2_i17;
  int32_T c2_i18;
  int32_T c2_i19;
  int32_T c2_i20;
  int32_T c2_i21;
  int32_T c2_i22;
  int32_T c2_i23;
  int32_T c2_i24;
  real_T *c2_demoState;
  real_T *c2_normCoMError;
  real_T (*c2_IntErrorCoM)[3];
  real_T (*c2_CoMError)[3];
  real_T (*c2_kImp)[25];
  real_T (*c2_kH)[4];
  real_T (*c2_Desired_x_dx_ddx_CoM)[9];
  real_T (*c2_Jcom)[186];
  real_T (*c2_xcom)[3];
  real_T (*c2_handsJddq)[12];
  real_T (*c2_feetJdqd)[12];
  real_T (*c2_tau)[25];
  real_T (*c2_handsJacobians)[372];
  real_T (*c2_feetJacobians)[372];
  real_T (*c2_PosRightFoot)[7];
  real_T (*c2_H)[6];
  real_T (*c2_g)[31];
  real_T (*c2_h)[31];
  real_T (*c2_M)[961];
  real_T (*c2_measuredExternalForces)[18];
  real_T (*c2_qD)[31];
  real_T (*c2_qInit)[25];
  real_T (*c2_q)[25];
  c2_normCoMError = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
  c2_IntErrorCoM = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 19);
  c2_CoMError = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 2);
  c2_kImp = (real_T (*)[25])ssGetInputPortSignal(chartInstance->S, 18);
  c2_kH = (real_T (*)[4])ssGetInputPortSignal(chartInstance->S, 17);
  c2_Desired_x_dx_ddx_CoM = (real_T (*)[9])ssGetInputPortSignal(chartInstance->S,
    16);
  c2_Jcom = (real_T (*)[186])ssGetInputPortSignal(chartInstance->S, 15);
  c2_xcom = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 14);
  c2_handsJddq = (real_T (*)[12])ssGetInputPortSignal(chartInstance->S, 13);
  c2_feetJdqd = (real_T (*)[12])ssGetInputPortSignal(chartInstance->S, 12);
  c2_tau = (real_T (*)[25])ssGetOutputPortSignal(chartInstance->S, 1);
  c2_handsJacobians = (real_T (*)[372])ssGetInputPortSignal(chartInstance->S, 11);
  c2_feetJacobians = (real_T (*)[372])ssGetInputPortSignal(chartInstance->S, 10);
  c2_PosRightFoot = (real_T (*)[7])ssGetInputPortSignal(chartInstance->S, 9);
  c2_H = (real_T (*)[6])ssGetInputPortSignal(chartInstance->S, 8);
  c2_g = (real_T (*)[31])ssGetInputPortSignal(chartInstance->S, 7);
  c2_h = (real_T (*)[31])ssGetInputPortSignal(chartInstance->S, 6);
  c2_M = (real_T (*)[961])ssGetInputPortSignal(chartInstance->S, 5);
  c2_measuredExternalForces = (real_T (*)[18])ssGetInputPortSignal
    (chartInstance->S, 4);
  c2_qD = (real_T (*)[31])ssGetInputPortSignal(chartInstance->S, 3);
  c2_qInit = (real_T (*)[25])ssGetInputPortSignal(chartInstance->S, 2);
  c2_q = (real_T (*)[25])ssGetInputPortSignal(chartInstance->S, 1);
  c2_demoState = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 1U, chartInstance->c2_sfEvent);
  _SFD_DATA_RANGE_CHECK(*c2_demoState, 0U);
  for (c2_i4 = 0; c2_i4 < 25; c2_i4++) {
    _SFD_DATA_RANGE_CHECK((*c2_q)[c2_i4], 1U);
  }

  for (c2_i5 = 0; c2_i5 < 25; c2_i5++) {
    _SFD_DATA_RANGE_CHECK((*c2_qInit)[c2_i5], 2U);
  }

  for (c2_i6 = 0; c2_i6 < 31; c2_i6++) {
    _SFD_DATA_RANGE_CHECK((*c2_qD)[c2_i6], 3U);
  }

  for (c2_i7 = 0; c2_i7 < 18; c2_i7++) {
    _SFD_DATA_RANGE_CHECK((*c2_measuredExternalForces)[c2_i7], 4U);
  }

  for (c2_i8 = 0; c2_i8 < 961; c2_i8++) {
    _SFD_DATA_RANGE_CHECK((*c2_M)[c2_i8], 5U);
  }

  for (c2_i9 = 0; c2_i9 < 31; c2_i9++) {
    _SFD_DATA_RANGE_CHECK((*c2_h)[c2_i9], 6U);
  }

  for (c2_i10 = 0; c2_i10 < 31; c2_i10++) {
    _SFD_DATA_RANGE_CHECK((*c2_g)[c2_i10], 7U);
  }

  for (c2_i11 = 0; c2_i11 < 6; c2_i11++) {
    _SFD_DATA_RANGE_CHECK((*c2_H)[c2_i11], 8U);
  }

  for (c2_i12 = 0; c2_i12 < 7; c2_i12++) {
    _SFD_DATA_RANGE_CHECK((*c2_PosRightFoot)[c2_i12], 9U);
  }

  for (c2_i13 = 0; c2_i13 < 372; c2_i13++) {
    _SFD_DATA_RANGE_CHECK((*c2_feetJacobians)[c2_i13], 10U);
  }

  for (c2_i14 = 0; c2_i14 < 372; c2_i14++) {
    _SFD_DATA_RANGE_CHECK((*c2_handsJacobians)[c2_i14], 11U);
  }

  for (c2_i15 = 0; c2_i15 < 25; c2_i15++) {
    _SFD_DATA_RANGE_CHECK((*c2_tau)[c2_i15], 12U);
  }

  for (c2_i16 = 0; c2_i16 < 12; c2_i16++) {
    _SFD_DATA_RANGE_CHECK((*c2_feetJdqd)[c2_i16], 13U);
  }

  for (c2_i17 = 0; c2_i17 < 12; c2_i17++) {
    _SFD_DATA_RANGE_CHECK((*c2_handsJddq)[c2_i17], 14U);
  }

  for (c2_i18 = 0; c2_i18 < 3; c2_i18++) {
    _SFD_DATA_RANGE_CHECK((*c2_xcom)[c2_i18], 15U);
  }

  for (c2_i19 = 0; c2_i19 < 186; c2_i19++) {
    _SFD_DATA_RANGE_CHECK((*c2_Jcom)[c2_i19], 16U);
  }

  for (c2_i20 = 0; c2_i20 < 9; c2_i20++) {
    _SFD_DATA_RANGE_CHECK((*c2_Desired_x_dx_ddx_CoM)[c2_i20], 17U);
  }

  for (c2_i21 = 0; c2_i21 < 4; c2_i21++) {
    _SFD_DATA_RANGE_CHECK((*c2_kH)[c2_i21], 18U);
  }

  for (c2_i22 = 0; c2_i22 < 25; c2_i22++) {
    _SFD_DATA_RANGE_CHECK((*c2_kImp)[c2_i22], 19U);
  }

  for (c2_i23 = 0; c2_i23 < 3; c2_i23++) {
    _SFD_DATA_RANGE_CHECK((*c2_CoMError)[c2_i23], 20U);
  }

  for (c2_i24 = 0; c2_i24 < 3; c2_i24++) {
    _SFD_DATA_RANGE_CHECK((*c2_IntErrorCoM)[c2_i24], 21U);
  }

  _SFD_DATA_RANGE_CHECK(*c2_normCoMError, 22U);
  chartInstance->c2_sfEvent = CALL_EVENT;
  c2_chartstep_c2_controller(chartInstance);
  sf_debug_check_for_state_inconsistency(_controllerMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void c2_chartstep_c2_controller(SFc2_controllerInstanceStruct
  *chartInstance)
{
  real_T c2_hoistedGlobal;
  real_T c2_demoState;
  int32_T c2_i25;
  real_T c2_q[25];
  int32_T c2_i26;
  real_T c2_qInit[25];
  int32_T c2_i27;
  real_T c2_qD[31];
  int32_T c2_i28;
  real_T c2_measuredExternalForces[18];
  int32_T c2_i29;
  static real_T c2_M[961];
  int32_T c2_i30;
  real_T c2_h[31];
  int32_T c2_i31;
  real_T c2_g[31];
  int32_T c2_i32;
  real_T c2_H[6];
  int32_T c2_i33;
  real_T c2_PosRightFoot[7];
  int32_T c2_i34;
  real_T c2_feetJacobians[372];
  int32_T c2_i35;
  real_T c2_handsJacobians[372];
  int32_T c2_i36;
  real_T c2_feetJdqd[12];
  int32_T c2_i37;
  real_T c2_handsJddq[12];
  int32_T c2_i38;
  real_T c2_xcom[3];
  int32_T c2_i39;
  real_T c2_Jcom[186];
  int32_T c2_i40;
  real_T c2_Desired_x_dx_ddx_CoM[9];
  int32_T c2_i41;
  real_T c2_kH[4];
  int32_T c2_i42;
  real_T c2_kImp[25];
  int32_T c2_i43;
  real_T c2_IntErrorCoM[3];
  uint32_T c2_debug_family_var_map[51];
  real_T c2_PINV_TOL;
  real_T c2_gravAcc;
  real_T c2_n;
  static real_T c2_St[775];
  real_T c2_m;
  real_T c2_grav[6];
  real_T c2_xDcom[3];
  real_T c2_xDDcomStar[3];
  real_T c2_Pr[3];
  real_T c2_A[72];
  real_T c2_pinvA[72];
  real_T c2_HwDotDes[3];
  real_T c2_Jc[558];
  real_T c2_JcDqD[18];
  real_T c2_HDotDes[6];
  real_T c2_handsDesiredWrenches[6];
  real_T c2_feetDesiredWrenches[12];
  static real_T c2_Minv[961];
  real_T c2_JcMinv[558];
  real_T c2_PInv_JcMinvSt[450];
  real_T c2_N0[625];
  real_T c2_desiredExternalWrench[18];
  real_T c2_measuredExternalWrench[18];
  real_T c2_tau1[25];
  real_T c2_LinearizedDynamics[25];
  real_T c2_tau0[25];
  real_T c2_nargin = 20.0;
  real_T c2_nargout = 3.0;
  real_T c2_tau[25];
  real_T c2_CoMError[3];
  real_T c2_normCoMError;
  int32_T c2_i44;
  static real_T c2_b[775] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  real_T c2_a;
  real_T c2_y;
  int32_T c2_i45;
  int32_T c2_i46;
  int32_T c2_i47;
  int32_T c2_i48;
  int32_T c2_i49;
  int32_T c2_i50;
  real_T c2_b_a[93];
  int32_T c2_i51;
  real_T c2_b_b[31];
  int32_T c2_i52;
  int32_T c2_i53;
  int32_T c2_i54;
  real_T c2_C[3];
  int32_T c2_i55;
  int32_T c2_i56;
  int32_T c2_i57;
  int32_T c2_i58;
  int32_T c2_i59;
  int32_T c2_i60;
  real_T c2_c_a;
  int32_T c2_i61;
  int32_T c2_i62;
  real_T c2_d_a;
  int32_T c2_i63;
  real_T c2_c_b[3];
  int32_T c2_i64;
  real_T c2_e_a;
  int32_T c2_i65;
  real_T c2_d_b[3];
  int32_T c2_i66;
  int32_T c2_i67;
  int32_T c2_i68;
  real_T c2_dv2[9];
  real_T c2_dv3[9];
  int32_T c2_i69;
  real_T c2_b_xcom[3];
  real_T c2_dv4[9];
  real_T c2_dv5[9];
  int32_T c2_i70;
  real_T c2_b_Pr[3];
  real_T c2_dv6[9];
  real_T c2_dv7[9];
  int32_T c2_i71;
  int32_T c2_i72;
  int32_T c2_i73;
  int32_T c2_i74;
  int32_T c2_i75;
  int32_T c2_i76;
  int32_T c2_i77;
  int32_T c2_i78;
  int32_T c2_i79;
  int32_T c2_i80;
  int32_T c2_i81;
  int32_T c2_i82;
  int32_T c2_i83;
  int32_T c2_i84;
  int32_T c2_i85;
  int32_T c2_i86;
  int32_T c2_i87;
  int32_T c2_i88;
  int32_T c2_i89;
  int32_T c2_i90;
  int32_T c2_i91;
  int32_T c2_i92;
  int32_T c2_i93;
  int32_T c2_i94;
  int32_T c2_i95;
  int32_T c2_i96;
  int32_T c2_i97;
  int32_T c2_i98;
  int32_T c2_i99;
  int32_T c2_i100;
  int32_T c2_i101;
  real_T c2_b_A[72];
  real_T c2_dv8[72];
  int32_T c2_i102;
  int32_T c2_i103;
  int32_T c2_i104;
  int32_T c2_i105;
  int32_T c2_i106;
  int32_T c2_i107;
  int32_T c2_i108;
  int32_T c2_i109;
  int32_T c2_i110;
  int32_T c2_i111;
  int32_T c2_i112;
  int32_T c2_i113;
  int32_T c2_i114;
  int32_T c2_i115;
  int32_T c2_i116;
  int32_T c2_i117;
  int32_T c2_i118;
  real_T c2_f_a;
  int32_T c2_i119;
  int32_T c2_i120;
  real_T c2_g_a;
  int32_T c2_i121;
  int32_T c2_i122;
  int32_T c2_i123;
  int32_T c2_i124;
  int32_T c2_i125;
  int32_T c2_i126;
  real_T c2_h_a[72];
  int32_T c2_i127;
  real_T c2_e_b[6];
  int32_T c2_i128;
  int32_T c2_i129;
  int32_T c2_i130;
  real_T c2_b_C[12];
  int32_T c2_i131;
  int32_T c2_i132;
  int32_T c2_i133;
  int32_T c2_i134;
  int32_T c2_i135;
  int32_T c2_i136;
  int32_T c2_i137;
  static real_T c2_b_M[961];
  static real_T c2_dv9[961];
  int32_T c2_i138;
  int32_T c2_i139;
  real_T c2_i_a[558];
  int32_T c2_i140;
  static real_T c2_f_b[961];
  int32_T c2_i141;
  int32_T c2_i142;
  int32_T c2_i143;
  real_T c2_dv10[558];
  int32_T c2_i144;
  static real_T c2_dv11[961];
  int32_T c2_i145;
  real_T c2_dv12[558];
  int32_T c2_i146;
  static real_T c2_dv13[961];
  int32_T c2_i147;
  int32_T c2_i148;
  real_T c2_b_y[450];
  int32_T c2_i149;
  real_T c2_j_a[558];
  int32_T c2_i150;
  real_T c2_g_b[775];
  int32_T c2_i151;
  real_T c2_c_y[450];
  real_T c2_dv14[450];
  int32_T c2_i152;
  int32_T c2_i153;
  real_T c2_k_a[450];
  int32_T c2_i154;
  int32_T c2_i155;
  real_T c2_d_y[775];
  int32_T c2_i156;
  real_T c2_l_a[450];
  int32_T c2_i157;
  real_T c2_m_a[558];
  int32_T c2_i158;
  real_T c2_e_y[625];
  int32_T c2_i159;
  real_T c2_f_y[775];
  int32_T c2_i160;
  real_T c2_h_b[775];
  real_T c2_dv15[625];
  int32_T c2_i161;
  int32_T c2_i162;
  int32_T c2_i163;
  int32_T c2_i164;
  int32_T c2_i165;
  int32_T c2_i166;
  int32_T c2_i167;
  real_T c2_g_y[18];
  int32_T c2_i168;
  real_T c2_n_a[558];
  int32_T c2_i169;
  real_T c2_i_b[31];
  int32_T c2_i170;
  int32_T c2_i171;
  int32_T c2_i172;
  int32_T c2_i173;
  int32_T c2_i174;
  real_T c2_j_b[558];
  int32_T c2_i175;
  real_T c2_h_y[324];
  int32_T c2_i176;
  real_T c2_o_a[558];
  int32_T c2_i177;
  real_T c2_k_b[558];
  int32_T c2_i178;
  real_T c2_l_b[18];
  int32_T c2_i179;
  real_T c2_i_y[18];
  int32_T c2_i180;
  int32_T c2_i181;
  int32_T c2_i182;
  int32_T c2_i183;
  int32_T c2_i184;
  int32_T c2_i185;
  int32_T c2_i186;
  real_T c2_c_C[25];
  int32_T c2_i187;
  int32_T c2_i188;
  int32_T c2_i189;
  int32_T c2_i190;
  int32_T c2_i191;
  int32_T c2_i192;
  int32_T c2_i193;
  int32_T c2_i194;
  int32_T c2_i195;
  int32_T c2_i196;
  int32_T c2_i197;
  int32_T c2_i198;
  int32_T c2_i199;
  int32_T c2_i200;
  int32_T c2_i201;
  int32_T c2_i202;
  int32_T c2_i203;
  int32_T c2_i204;
  real_T c2_m_b[25];
  int32_T c2_i205;
  int32_T c2_i206;
  real_T c2_j_y[625];
  int32_T c2_i207;
  real_T c2_n_b[25];
  int32_T c2_i208;
  int32_T c2_i209;
  int32_T c2_i210;
  real_T c2_b_CoMError[3];
  int32_T c2_i211;
  int32_T c2_i212;
  real_T *c2_b_demoState;
  real_T *c2_b_normCoMError;
  real_T (*c2_b_tau)[25];
  real_T (*c2_c_CoMError)[3];
  real_T (*c2_b_IntErrorCoM)[3];
  real_T (*c2_b_kImp)[25];
  real_T (*c2_b_kH)[4];
  real_T (*c2_b_Desired_x_dx_ddx_CoM)[9];
  real_T (*c2_b_Jcom)[186];
  real_T (*c2_c_xcom)[3];
  real_T (*c2_b_handsJddq)[12];
  real_T (*c2_b_feetJdqd)[12];
  real_T (*c2_b_handsJacobians)[372];
  real_T (*c2_b_feetJacobians)[372];
  real_T (*c2_b_PosRightFoot)[7];
  real_T (*c2_b_H)[6];
  real_T (*c2_b_g)[31];
  real_T (*c2_b_h)[31];
  real_T (*c2_c_M)[961];
  real_T (*c2_b_measuredExternalForces)[18];
  real_T (*c2_b_qD)[31];
  real_T (*c2_b_qInit)[25];
  real_T (*c2_b_q)[25];
  c2_b_normCoMError = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
  c2_b_IntErrorCoM = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 19);
  c2_c_CoMError = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 2);
  c2_b_kImp = (real_T (*)[25])ssGetInputPortSignal(chartInstance->S, 18);
  c2_b_kH = (real_T (*)[4])ssGetInputPortSignal(chartInstance->S, 17);
  c2_b_Desired_x_dx_ddx_CoM = (real_T (*)[9])ssGetInputPortSignal
    (chartInstance->S, 16);
  c2_b_Jcom = (real_T (*)[186])ssGetInputPortSignal(chartInstance->S, 15);
  c2_c_xcom = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 14);
  c2_b_handsJddq = (real_T (*)[12])ssGetInputPortSignal(chartInstance->S, 13);
  c2_b_feetJdqd = (real_T (*)[12])ssGetInputPortSignal(chartInstance->S, 12);
  c2_b_tau = (real_T (*)[25])ssGetOutputPortSignal(chartInstance->S, 1);
  c2_b_handsJacobians = (real_T (*)[372])ssGetInputPortSignal(chartInstance->S,
    11);
  c2_b_feetJacobians = (real_T (*)[372])ssGetInputPortSignal(chartInstance->S,
    10);
  c2_b_PosRightFoot = (real_T (*)[7])ssGetInputPortSignal(chartInstance->S, 9);
  c2_b_H = (real_T (*)[6])ssGetInputPortSignal(chartInstance->S, 8);
  c2_b_g = (real_T (*)[31])ssGetInputPortSignal(chartInstance->S, 7);
  c2_b_h = (real_T (*)[31])ssGetInputPortSignal(chartInstance->S, 6);
  c2_c_M = (real_T (*)[961])ssGetInputPortSignal(chartInstance->S, 5);
  c2_b_measuredExternalForces = (real_T (*)[18])ssGetInputPortSignal
    (chartInstance->S, 4);
  c2_b_qD = (real_T (*)[31])ssGetInputPortSignal(chartInstance->S, 3);
  c2_b_qInit = (real_T (*)[25])ssGetInputPortSignal(chartInstance->S, 2);
  c2_b_q = (real_T (*)[25])ssGetInputPortSignal(chartInstance->S, 1);
  c2_b_demoState = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 1U, chartInstance->c2_sfEvent);
  c2_hoistedGlobal = *c2_b_demoState;
  c2_demoState = c2_hoistedGlobal;
  for (c2_i25 = 0; c2_i25 < 25; c2_i25++) {
    c2_q[c2_i25] = (*c2_b_q)[c2_i25];
  }

  for (c2_i26 = 0; c2_i26 < 25; c2_i26++) {
    c2_qInit[c2_i26] = (*c2_b_qInit)[c2_i26];
  }

  for (c2_i27 = 0; c2_i27 < 31; c2_i27++) {
    c2_qD[c2_i27] = (*c2_b_qD)[c2_i27];
  }

  for (c2_i28 = 0; c2_i28 < 18; c2_i28++) {
    c2_measuredExternalForces[c2_i28] = (*c2_b_measuredExternalForces)[c2_i28];
  }

  for (c2_i29 = 0; c2_i29 < 961; c2_i29++) {
    c2_M[c2_i29] = (*c2_c_M)[c2_i29];
  }

  for (c2_i30 = 0; c2_i30 < 31; c2_i30++) {
    c2_h[c2_i30] = (*c2_b_h)[c2_i30];
  }

  for (c2_i31 = 0; c2_i31 < 31; c2_i31++) {
    c2_g[c2_i31] = (*c2_b_g)[c2_i31];
  }

  for (c2_i32 = 0; c2_i32 < 6; c2_i32++) {
    c2_H[c2_i32] = (*c2_b_H)[c2_i32];
  }

  for (c2_i33 = 0; c2_i33 < 7; c2_i33++) {
    c2_PosRightFoot[c2_i33] = (*c2_b_PosRightFoot)[c2_i33];
  }

  for (c2_i34 = 0; c2_i34 < 372; c2_i34++) {
    c2_feetJacobians[c2_i34] = (*c2_b_feetJacobians)[c2_i34];
  }

  for (c2_i35 = 0; c2_i35 < 372; c2_i35++) {
    c2_handsJacobians[c2_i35] = (*c2_b_handsJacobians)[c2_i35];
  }

  for (c2_i36 = 0; c2_i36 < 12; c2_i36++) {
    c2_feetJdqd[c2_i36] = (*c2_b_feetJdqd)[c2_i36];
  }

  for (c2_i37 = 0; c2_i37 < 12; c2_i37++) {
    c2_handsJddq[c2_i37] = (*c2_b_handsJddq)[c2_i37];
  }

  for (c2_i38 = 0; c2_i38 < 3; c2_i38++) {
    c2_xcom[c2_i38] = (*c2_c_xcom)[c2_i38];
  }

  for (c2_i39 = 0; c2_i39 < 186; c2_i39++) {
    c2_Jcom[c2_i39] = (*c2_b_Jcom)[c2_i39];
  }

  for (c2_i40 = 0; c2_i40 < 9; c2_i40++) {
    c2_Desired_x_dx_ddx_CoM[c2_i40] = (*c2_b_Desired_x_dx_ddx_CoM)[c2_i40];
  }

  for (c2_i41 = 0; c2_i41 < 4; c2_i41++) {
    c2_kH[c2_i41] = (*c2_b_kH)[c2_i41];
  }

  for (c2_i42 = 0; c2_i42 < 25; c2_i42++) {
    c2_kImp[c2_i42] = (*c2_b_kImp)[c2_i42];
  }

  for (c2_i43 = 0; c2_i43 < 3; c2_i43++) {
    c2_IntErrorCoM[c2_i43] = (*c2_b_IntErrorCoM)[c2_i43];
  }

  sf_debug_symbol_scope_push_eml(0U, 51U, 51U, c2_debug_family_names,
    c2_debug_family_var_map);
  sf_debug_symbol_scope_add_eml(&c2_PINV_TOL, 0U, c2_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(&c2_gravAcc, 1U, c2_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(&c2_n, 2U, c2_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(c2_St, 3U, c2_u_sf_marshallOut);
  sf_debug_symbol_scope_add_eml_importable(&c2_m, 4U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_grav, 5U, c2_j_sf_marshallOut,
    c2_j_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_xDcom, 6U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_xDDcomStar, 7U,
    c2_b_sf_marshallOut, c2_b_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_Pr, 8U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_A, 9U, c2_t_sf_marshallOut,
    c2_l_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_pinvA, 10U, c2_s_sf_marshallOut,
    c2_k_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_HwDotDes, 11U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_Jc, 12U, c2_q_sf_marshallOut,
    c2_g_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_JcDqD, 13U, c2_n_sf_marshallOut,
    c2_d_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_HDotDes, 14U, c2_j_sf_marshallOut,
    c2_j_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_handsDesiredWrenches, 15U,
    c2_j_sf_marshallOut, c2_j_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_feetDesiredWrenches, 16U,
    c2_r_sf_marshallOut, c2_i_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_Minv, 17U, c2_l_sf_marshallOut,
    c2_h_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_JcMinv, 18U, c2_q_sf_marshallOut,
    c2_g_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_PInv_JcMinvSt, 19U,
    c2_p_sf_marshallOut, c2_f_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_N0, 20U, c2_o_sf_marshallOut,
    c2_e_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_desiredExternalWrench, 21U,
    c2_n_sf_marshallOut, c2_d_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_measuredExternalWrench, 22U,
    c2_n_sf_marshallOut, c2_d_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_tau1, 23U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_LinearizedDynamics, 24U,
    c2_c_sf_marshallOut, c2_c_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_tau0, 25U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c2_nargin, 26U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c2_nargout, 27U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  sf_debug_symbol_scope_add_eml(&c2_demoState, 28U, c2_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(c2_q, 29U, c2_c_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(c2_qInit, 30U, c2_c_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(c2_qD, 31U, c2_k_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(c2_measuredExternalForces, 32U,
    c2_m_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(c2_M, 33U, c2_l_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(c2_h, 34U, c2_k_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(c2_g, 35U, c2_k_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(c2_H, 36U, c2_j_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(c2_PosRightFoot, 37U, c2_i_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(c2_feetJacobians, 38U, c2_h_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(c2_handsJacobians, 39U, c2_h_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(c2_feetJdqd, 40U, c2_g_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(c2_handsJddq, 41U, c2_g_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(c2_xcom, 42U, c2_b_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(c2_Jcom, 43U, c2_f_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(c2_Desired_x_dx_ddx_CoM, 44U,
    c2_e_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(c2_kH, 45U, c2_d_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(c2_kImp, 46U, c2_c_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(c2_IntErrorCoM, 47U, c2_b_sf_marshallOut);
  sf_debug_symbol_scope_add_eml_importable(c2_tau, 48U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_CoMError, 49U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c2_normCoMError, 50U,
    c2_sf_marshallOut, c2_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 4);
  c2_PINV_TOL = 1.0E-5;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 5);
  c2_gravAcc = 9.81;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 7);
  c2_n = 25.0;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 8);
  for (c2_i44 = 0; c2_i44 < 775; c2_i44++) {
    c2_St[c2_i44] = c2_b[c2_i44];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 10);
  c2_m = c2_M[0];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 11);
  c2_a = -c2_m;
  c2_y = c2_a * 9.81;
  for (c2_i45 = 0; c2_i45 < 2; c2_i45++) {
    c2_grav[c2_i45] = 0.0;
  }

  c2_grav[2] = c2_y;
  for (c2_i46 = 0; c2_i46 < 3; c2_i46++) {
    c2_grav[c2_i46 + 3] = 0.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 16);
  c2_i47 = 0;
  c2_i48 = 0;
  for (c2_i49 = 0; c2_i49 < 31; c2_i49++) {
    for (c2_i50 = 0; c2_i50 < 3; c2_i50++) {
      c2_b_a[c2_i50 + c2_i47] = c2_Jcom[c2_i50 + c2_i48];
    }

    c2_i47 += 3;
    c2_i48 += 6;
  }

  for (c2_i51 = 0; c2_i51 < 31; c2_i51++) {
    c2_b_b[c2_i51] = c2_qD[c2_i51];
  }

  c2_b_eml_scalar_eg(chartInstance);
  c2_b_eml_scalar_eg(chartInstance);
  for (c2_i52 = 0; c2_i52 < 3; c2_i52++) {
    c2_xDcom[c2_i52] = 0.0;
  }

  for (c2_i53 = 0; c2_i53 < 3; c2_i53++) {
    c2_xDcom[c2_i53] = 0.0;
  }

  for (c2_i54 = 0; c2_i54 < 3; c2_i54++) {
    c2_C[c2_i54] = c2_xDcom[c2_i54];
  }

  for (c2_i55 = 0; c2_i55 < 3; c2_i55++) {
    c2_xDcom[c2_i55] = c2_C[c2_i55];
  }

  for (c2_i56 = 0; c2_i56 < 3; c2_i56++) {
    c2_C[c2_i56] = c2_xDcom[c2_i56];
  }

  for (c2_i57 = 0; c2_i57 < 3; c2_i57++) {
    c2_xDcom[c2_i57] = c2_C[c2_i57];
  }

  for (c2_i58 = 0; c2_i58 < 3; c2_i58++) {
    c2_xDcom[c2_i58] = 0.0;
    c2_i59 = 0;
    for (c2_i60 = 0; c2_i60 < 31; c2_i60++) {
      c2_xDcom[c2_i58] += c2_b_a[c2_i59 + c2_i58] * c2_b_b[c2_i60];
      c2_i59 += 3;
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 17);
  c2_c_a = c2_kH[0];
  for (c2_i61 = 0; c2_i61 < 3; c2_i61++) {
    c2_C[c2_i61] = c2_xcom[c2_i61] - c2_Desired_x_dx_ddx_CoM[c2_i61];
  }

  for (c2_i62 = 0; c2_i62 < 3; c2_i62++) {
    c2_C[c2_i62] *= c2_c_a;
  }

  c2_d_a = c2_kH[1];
  for (c2_i63 = 0; c2_i63 < 3; c2_i63++) {
    c2_c_b[c2_i63] = c2_IntErrorCoM[c2_i63];
  }

  for (c2_i64 = 0; c2_i64 < 3; c2_i64++) {
    c2_c_b[c2_i64] *= c2_d_a;
  }

  c2_e_a = c2_kH[2];
  for (c2_i65 = 0; c2_i65 < 3; c2_i65++) {
    c2_d_b[c2_i65] = c2_xDcom[c2_i65] - c2_Desired_x_dx_ddx_CoM[c2_i65 + 3];
  }

  for (c2_i66 = 0; c2_i66 < 3; c2_i66++) {
    c2_d_b[c2_i66] *= c2_e_a;
  }

  for (c2_i67 = 0; c2_i67 < 3; c2_i67++) {
    c2_xDDcomStar[c2_i67] = ((c2_Desired_x_dx_ddx_CoM[c2_i67 + 6] - c2_C[c2_i67])
      - c2_c_b[c2_i67]) - c2_d_b[c2_i67];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 19);
  for (c2_i68 = 0; c2_i68 < 3; c2_i68++) {
    c2_Pr[c2_i68] = c2_PosRightFoot[c2_i68] - c2_xcom[c2_i68];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 21);
  c2_eye(chartInstance, c2_dv2);
  c2_eye(chartInstance, c2_dv3);
  for (c2_i69 = 0; c2_i69 < 3; c2_i69++) {
    c2_b_xcom[c2_i69] = c2_xcom[c2_i69];
  }

  c2_Sf(chartInstance, c2_b_xcom, c2_dv4);
  c2_eye(chartInstance, c2_dv5);
  for (c2_i70 = 0; c2_i70 < 3; c2_i70++) {
    c2_b_Pr[c2_i70] = c2_Pr[c2_i70];
  }

  c2_Sf(chartInstance, c2_b_Pr, c2_dv6);
  c2_eye(chartInstance, c2_dv7);
  c2_i71 = 0;
  c2_i72 = 0;
  for (c2_i73 = 0; c2_i73 < 3; c2_i73++) {
    for (c2_i74 = 0; c2_i74 < 3; c2_i74++) {
      c2_A[c2_i74 + c2_i71] = c2_dv2[c2_i74 + c2_i72];
    }

    c2_i71 += 6;
    c2_i72 += 3;
  }

  c2_i75 = 0;
  for (c2_i76 = 0; c2_i76 < 3; c2_i76++) {
    for (c2_i77 = 0; c2_i77 < 3; c2_i77++) {
      c2_A[(c2_i77 + c2_i75) + 18] = 0.0;
    }

    c2_i75 += 6;
  }

  c2_i78 = 0;
  c2_i79 = 0;
  for (c2_i80 = 0; c2_i80 < 3; c2_i80++) {
    for (c2_i81 = 0; c2_i81 < 3; c2_i81++) {
      c2_A[(c2_i81 + c2_i78) + 36] = c2_dv3[c2_i81 + c2_i79];
    }

    c2_i78 += 6;
    c2_i79 += 3;
  }

  c2_i82 = 0;
  for (c2_i83 = 0; c2_i83 < 3; c2_i83++) {
    for (c2_i84 = 0; c2_i84 < 3; c2_i84++) {
      c2_A[(c2_i84 + c2_i82) + 54] = 0.0;
    }

    c2_i82 += 6;
  }

  c2_i85 = 0;
  c2_i86 = 0;
  for (c2_i87 = 0; c2_i87 < 3; c2_i87++) {
    for (c2_i88 = 0; c2_i88 < 3; c2_i88++) {
      c2_A[(c2_i88 + c2_i85) + 3] = -c2_dv4[c2_i88 + c2_i86];
    }

    c2_i85 += 6;
    c2_i86 += 3;
  }

  c2_i89 = 0;
  c2_i90 = 0;
  for (c2_i91 = 0; c2_i91 < 3; c2_i91++) {
    for (c2_i92 = 0; c2_i92 < 3; c2_i92++) {
      c2_A[(c2_i92 + c2_i89) + 21] = c2_dv5[c2_i92 + c2_i90];
    }

    c2_i89 += 6;
    c2_i90 += 3;
  }

  c2_i93 = 0;
  c2_i94 = 0;
  for (c2_i95 = 0; c2_i95 < 3; c2_i95++) {
    for (c2_i96 = 0; c2_i96 < 3; c2_i96++) {
      c2_A[(c2_i96 + c2_i93) + 39] = c2_dv6[c2_i96 + c2_i94];
    }

    c2_i93 += 6;
    c2_i94 += 3;
  }

  c2_i97 = 0;
  c2_i98 = 0;
  for (c2_i99 = 0; c2_i99 < 3; c2_i99++) {
    for (c2_i100 = 0; c2_i100 < 3; c2_i100++) {
      c2_A[(c2_i100 + c2_i97) + 57] = c2_dv7[c2_i100 + c2_i98];
    }

    c2_i97 += 6;
    c2_i98 += 3;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 23);
  for (c2_i101 = 0; c2_i101 < 72; c2_i101++) {
    c2_b_A[c2_i101] = c2_A[c2_i101];
  }

  c2_pinv(chartInstance, c2_b_A, c2_dv8);
  for (c2_i102 = 0; c2_i102 < 72; c2_i102++) {
    c2_pinvA[c2_i102] = c2_dv8[c2_i102];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 26);
  for (c2_i103 = 0; c2_i103 < 3; c2_i103++) {
    c2_HwDotDes[c2_i103] = c2_H[c2_i103 + 3];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 27);
  c2_i104 = 0;
  c2_i105 = 0;
  for (c2_i106 = 0; c2_i106 < 31; c2_i106++) {
    for (c2_i107 = 0; c2_i107 < 12; c2_i107++) {
      c2_Jc[c2_i107 + c2_i104] = c2_feetJacobians[c2_i107 + c2_i105];
    }

    c2_i104 += 18;
    c2_i105 += 12;
  }

  c2_i108 = 0;
  c2_i109 = 0;
  for (c2_i110 = 0; c2_i110 < 31; c2_i110++) {
    for (c2_i111 = 0; c2_i111 < 3; c2_i111++) {
      c2_Jc[(c2_i111 + c2_i108) + 12] = c2_handsJacobians[c2_i111 + c2_i109];
    }

    c2_i108 += 18;
    c2_i109 += 12;
  }

  c2_i112 = 0;
  c2_i113 = 0;
  for (c2_i114 = 0; c2_i114 < 31; c2_i114++) {
    for (c2_i115 = 0; c2_i115 < 3; c2_i115++) {
      c2_Jc[(c2_i115 + c2_i112) + 15] = c2_handsJacobians[(c2_i115 + c2_i113) +
        6];
    }

    c2_i112 += 18;
    c2_i113 += 12;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 28);
  for (c2_i116 = 0; c2_i116 < 12; c2_i116++) {
    c2_JcDqD[c2_i116] = c2_feetJdqd[c2_i116];
  }

  for (c2_i117 = 0; c2_i117 < 3; c2_i117++) {
    c2_JcDqD[c2_i117 + 12] = c2_handsJddq[c2_i117];
  }

  for (c2_i118 = 0; c2_i118 < 3; c2_i118++) {
    c2_JcDqD[c2_i118 + 15] = c2_handsJddq[c2_i118 + 6];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 32);
  c2_f_a = c2_m;
  for (c2_i119 = 0; c2_i119 < 3; c2_i119++) {
    c2_C[c2_i119] = c2_xDDcomStar[c2_i119];
  }

  for (c2_i120 = 0; c2_i120 < 3; c2_i120++) {
    c2_C[c2_i120] *= c2_f_a;
  }

  c2_g_a = -c2_kH[3];
  for (c2_i121 = 0; c2_i121 < 3; c2_i121++) {
    c2_c_b[c2_i121] = c2_HwDotDes[c2_i121];
  }

  for (c2_i122 = 0; c2_i122 < 3; c2_i122++) {
    c2_c_b[c2_i122] *= c2_g_a;
  }

  for (c2_i123 = 0; c2_i123 < 3; c2_i123++) {
    c2_HDotDes[c2_i123] = c2_C[c2_i123];
  }

  for (c2_i124 = 0; c2_i124 < 3; c2_i124++) {
    c2_HDotDes[c2_i124 + 3] = c2_c_b[c2_i124];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 35);
  for (c2_i125 = 0; c2_i125 < 6; c2_i125++) {
    c2_handsDesiredWrenches[c2_i125] = 0.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 37);
  CV_EML_IF(0, 1, 0, c2_demoState == 1.0);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 40);
  CV_EML_IF(0, 1, 1, c2_demoState == 2.0);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 43);
  CV_EML_IF(0, 1, 2, c2_demoState == 3.0);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 46);
  CV_EML_IF(0, 1, 3, c2_demoState == 4.0);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 50);
  for (c2_i126 = 0; c2_i126 < 72; c2_i126++) {
    c2_h_a[c2_i126] = c2_pinvA[c2_i126];
  }

  for (c2_i127 = 0; c2_i127 < 6; c2_i127++) {
    c2_e_b[c2_i127] = (c2_HDotDes[c2_i127] - c2_grav[c2_i127]) -
      c2_handsDesiredWrenches[c2_i127];
  }

  c2_e_eml_scalar_eg(chartInstance);
  c2_e_eml_scalar_eg(chartInstance);
  for (c2_i128 = 0; c2_i128 < 12; c2_i128++) {
    c2_feetDesiredWrenches[c2_i128] = 0.0;
  }

  for (c2_i129 = 0; c2_i129 < 12; c2_i129++) {
    c2_feetDesiredWrenches[c2_i129] = 0.0;
  }

  for (c2_i130 = 0; c2_i130 < 12; c2_i130++) {
    c2_b_C[c2_i130] = c2_feetDesiredWrenches[c2_i130];
  }

  for (c2_i131 = 0; c2_i131 < 12; c2_i131++) {
    c2_feetDesiredWrenches[c2_i131] = c2_b_C[c2_i131];
  }

  for (c2_i132 = 0; c2_i132 < 12; c2_i132++) {
    c2_b_C[c2_i132] = c2_feetDesiredWrenches[c2_i132];
  }

  for (c2_i133 = 0; c2_i133 < 12; c2_i133++) {
    c2_feetDesiredWrenches[c2_i133] = c2_b_C[c2_i133];
  }

  for (c2_i134 = 0; c2_i134 < 12; c2_i134++) {
    c2_feetDesiredWrenches[c2_i134] = 0.0;
    c2_i135 = 0;
    for (c2_i136 = 0; c2_i136 < 6; c2_i136++) {
      c2_feetDesiredWrenches[c2_i134] += c2_h_a[c2_i135 + c2_i134] *
        c2_e_b[c2_i136];
      c2_i135 += 12;
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 53);
  for (c2_i137 = 0; c2_i137 < 961; c2_i137++) {
    c2_b_M[c2_i137] = c2_M[c2_i137];
  }

  c2_inv(chartInstance, c2_b_M, c2_dv9);
  for (c2_i138 = 0; c2_i138 < 961; c2_i138++) {
    c2_Minv[c2_i138] = c2_dv9[c2_i138];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 54);
  for (c2_i139 = 0; c2_i139 < 558; c2_i139++) {
    c2_i_a[c2_i139] = c2_Jc[c2_i139];
  }

  for (c2_i140 = 0; c2_i140 < 961; c2_i140++) {
    c2_f_b[c2_i140] = c2_Minv[c2_i140];
  }

  c2_g_eml_scalar_eg(chartInstance);
  c2_g_eml_scalar_eg(chartInstance);
  for (c2_i141 = 0; c2_i141 < 558; c2_i141++) {
    c2_JcMinv[c2_i141] = 0.0;
  }

  for (c2_i142 = 0; c2_i142 < 558; c2_i142++) {
    c2_JcMinv[c2_i142] = 0.0;
  }

  for (c2_i143 = 0; c2_i143 < 558; c2_i143++) {
    c2_dv10[c2_i143] = c2_i_a[c2_i143];
  }

  for (c2_i144 = 0; c2_i144 < 961; c2_i144++) {
    c2_dv11[c2_i144] = c2_f_b[c2_i144];
  }

  for (c2_i145 = 0; c2_i145 < 558; c2_i145++) {
    c2_dv12[c2_i145] = c2_dv10[c2_i145];
  }

  for (c2_i146 = 0; c2_i146 < 961; c2_i146++) {
    c2_dv13[c2_i146] = c2_dv11[c2_i146];
  }

  c2_k_eml_xgemm(chartInstance, c2_dv12, c2_dv13, c2_JcMinv);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 55);
  for (c2_i147 = 0; c2_i147 < 558; c2_i147++) {
    c2_i_a[c2_i147] = c2_JcMinv[c2_i147];
  }

  c2_h_eml_scalar_eg(chartInstance);
  c2_h_eml_scalar_eg(chartInstance);
  for (c2_i148 = 0; c2_i148 < 450; c2_i148++) {
    c2_b_y[c2_i148] = 0.0;
  }

  for (c2_i149 = 0; c2_i149 < 558; c2_i149++) {
    c2_j_a[c2_i149] = c2_i_a[c2_i149];
  }

  for (c2_i150 = 0; c2_i150 < 775; c2_i150++) {
    c2_g_b[c2_i150] = c2_b[c2_i150];
  }

  c2_l_eml_xgemm(chartInstance, c2_j_a, c2_g_b, c2_b_y);
  for (c2_i151 = 0; c2_i151 < 450; c2_i151++) {
    c2_c_y[c2_i151] = c2_b_y[c2_i151];
  }

  c2_b_pinv(chartInstance, c2_c_y, 1.0E-5, c2_dv14);
  for (c2_i152 = 0; c2_i152 < 450; c2_i152++) {
    c2_PInv_JcMinvSt[c2_i152] = c2_dv14[c2_i152];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 56);
  for (c2_i153 = 0; c2_i153 < 450; c2_i153++) {
    c2_k_a[c2_i153] = c2_PInv_JcMinvSt[c2_i153];
  }

  for (c2_i154 = 0; c2_i154 < 558; c2_i154++) {
    c2_i_a[c2_i154] = c2_JcMinv[c2_i154];
  }

  c2_k_eml_scalar_eg(chartInstance);
  c2_k_eml_scalar_eg(chartInstance);
  for (c2_i155 = 0; c2_i155 < 775; c2_i155++) {
    c2_d_y[c2_i155] = 0.0;
  }

  for (c2_i156 = 0; c2_i156 < 450; c2_i156++) {
    c2_l_a[c2_i156] = c2_k_a[c2_i156];
  }

  for (c2_i157 = 0; c2_i157 < 558; c2_i157++) {
    c2_m_a[c2_i157] = c2_i_a[c2_i157];
  }

  c2_n_eml_xgemm(chartInstance, c2_l_a, c2_m_a, c2_d_y);
  c2_l_eml_scalar_eg(chartInstance);
  c2_l_eml_scalar_eg(chartInstance);
  for (c2_i158 = 0; c2_i158 < 625; c2_i158++) {
    c2_e_y[c2_i158] = 0.0;
  }

  for (c2_i159 = 0; c2_i159 < 775; c2_i159++) {
    c2_f_y[c2_i159] = c2_d_y[c2_i159];
  }

  for (c2_i160 = 0; c2_i160 < 775; c2_i160++) {
    c2_h_b[c2_i160] = c2_b[c2_i160];
  }

  c2_o_eml_xgemm(chartInstance, c2_f_y, c2_h_b, c2_e_y);
  c2_b_eye(chartInstance, c2_dv15);
  for (c2_i161 = 0; c2_i161 < 625; c2_i161++) {
    c2_N0[c2_i161] = c2_dv15[c2_i161] - c2_e_y[c2_i161];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 58);
  for (c2_i162 = 0; c2_i162 < 12; c2_i162++) {
    c2_desiredExternalWrench[c2_i162] = c2_feetDesiredWrenches[c2_i162];
  }

  for (c2_i163 = 0; c2_i163 < 6; c2_i163++) {
    c2_desiredExternalWrench[c2_i163 + 12] = 0.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 60);
  for (c2_i164 = 0; c2_i164 < 18; c2_i164++) {
    c2_measuredExternalWrench[c2_i164] = c2_desiredExternalWrench[c2_i164];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 62);
  for (c2_i165 = 0; c2_i165 < 558; c2_i165++) {
    c2_i_a[c2_i165] = c2_JcMinv[c2_i165];
  }

  for (c2_i166 = 0; c2_i166 < 31; c2_i166++) {
    c2_b_b[c2_i166] = c2_h[c2_i166];
  }

  c2_m_eml_scalar_eg(chartInstance);
  c2_m_eml_scalar_eg(chartInstance);
  for (c2_i167 = 0; c2_i167 < 18; c2_i167++) {
    c2_g_y[c2_i167] = 0.0;
  }

  for (c2_i168 = 0; c2_i168 < 558; c2_i168++) {
    c2_n_a[c2_i168] = c2_i_a[c2_i168];
  }

  for (c2_i169 = 0; c2_i169 < 31; c2_i169++) {
    c2_i_b[c2_i169] = c2_b_b[c2_i169];
  }

  c2_p_eml_xgemm(chartInstance, c2_n_a, c2_i_b, c2_g_y);
  for (c2_i170 = 0; c2_i170 < 558; c2_i170++) {
    c2_i_a[c2_i170] = c2_JcMinv[c2_i170];
  }

  c2_i171 = 0;
  for (c2_i172 = 0; c2_i172 < 18; c2_i172++) {
    c2_i173 = 0;
    for (c2_i174 = 0; c2_i174 < 31; c2_i174++) {
      c2_j_b[c2_i174 + c2_i171] = c2_Jc[c2_i173 + c2_i172];
      c2_i173 += 18;
    }

    c2_i171 += 31;
  }

  c2_n_eml_scalar_eg(chartInstance);
  c2_n_eml_scalar_eg(chartInstance);
  for (c2_i175 = 0; c2_i175 < 324; c2_i175++) {
    c2_h_y[c2_i175] = 0.0;
  }

  for (c2_i176 = 0; c2_i176 < 558; c2_i176++) {
    c2_o_a[c2_i176] = c2_i_a[c2_i176];
  }

  for (c2_i177 = 0; c2_i177 < 558; c2_i177++) {
    c2_k_b[c2_i177] = c2_j_b[c2_i177];
  }

  c2_q_eml_xgemm(chartInstance, c2_o_a, c2_k_b, c2_h_y);
  for (c2_i178 = 0; c2_i178 < 18; c2_i178++) {
    c2_l_b[c2_i178] = c2_desiredExternalWrench[c2_i178];
  }

  c2_o_eml_scalar_eg(chartInstance);
  c2_o_eml_scalar_eg(chartInstance);
  for (c2_i179 = 0; c2_i179 < 18; c2_i179++) {
    c2_i_y[c2_i179] = 0.0;
    c2_i180 = 0;
    for (c2_i181 = 0; c2_i181 < 18; c2_i181++) {
      c2_i_y[c2_i179] += c2_h_y[c2_i180 + c2_i179] * c2_l_b[c2_i181];
      c2_i180 += 18;
    }
  }

  for (c2_i182 = 0; c2_i182 < 450; c2_i182++) {
    c2_k_a[c2_i182] = c2_PInv_JcMinvSt[c2_i182];
  }

  for (c2_i183 = 0; c2_i183 < 18; c2_i183++) {
    c2_g_y[c2_i183] = (c2_g_y[c2_i183] - c2_JcDqD[c2_i183]) - c2_i_y[c2_i183];
  }

  c2_p_eml_scalar_eg(chartInstance);
  c2_p_eml_scalar_eg(chartInstance);
  for (c2_i184 = 0; c2_i184 < 25; c2_i184++) {
    c2_tau1[c2_i184] = 0.0;
  }

  for (c2_i185 = 0; c2_i185 < 25; c2_i185++) {
    c2_tau1[c2_i185] = 0.0;
  }

  for (c2_i186 = 0; c2_i186 < 25; c2_i186++) {
    c2_c_C[c2_i186] = c2_tau1[c2_i186];
  }

  for (c2_i187 = 0; c2_i187 < 25; c2_i187++) {
    c2_tau1[c2_i187] = c2_c_C[c2_i187];
  }

  for (c2_i188 = 0; c2_i188 < 25; c2_i188++) {
    c2_c_C[c2_i188] = c2_tau1[c2_i188];
  }

  for (c2_i189 = 0; c2_i189 < 25; c2_i189++) {
    c2_tau1[c2_i189] = c2_c_C[c2_i189];
  }

  for (c2_i190 = 0; c2_i190 < 25; c2_i190++) {
    c2_tau1[c2_i190] = 0.0;
    c2_i191 = 0;
    for (c2_i192 = 0; c2_i192 < 18; c2_i192++) {
      c2_tau1[c2_i190] += c2_k_a[c2_i191 + c2_i190] * c2_g_y[c2_i192];
      c2_i191 += 25;
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 63);
  c2_i193 = 0;
  for (c2_i194 = 0; c2_i194 < 18; c2_i194++) {
    c2_i195 = 0;
    for (c2_i196 = 0; c2_i196 < 25; c2_i196++) {
      c2_k_a[c2_i196 + c2_i193] = c2_Jc[(c2_i195 + c2_i194) + 108];
      c2_i195 += 18;
    }

    c2_i193 += 25;
  }

  for (c2_i197 = 0; c2_i197 < 18; c2_i197++) {
    c2_l_b[c2_i197] = c2_measuredExternalWrench[c2_i197];
  }

  c2_p_eml_scalar_eg(chartInstance);
  c2_p_eml_scalar_eg(chartInstance);
  for (c2_i198 = 0; c2_i198 < 25; c2_i198++) {
    c2_c_C[c2_i198] = 0.0;
    c2_i199 = 0;
    for (c2_i200 = 0; c2_i200 < 18; c2_i200++) {
      c2_c_C[c2_i198] += c2_k_a[c2_i199 + c2_i198] * c2_l_b[c2_i200];
      c2_i199 += 25;
    }
  }

  for (c2_i201 = 0; c2_i201 < 25; c2_i201++) {
    c2_LinearizedDynamics[c2_i201] = (c2_g[c2_i201 + 6] - c2_c_C[c2_i201]) -
      c2_kImp[c2_i201] * (c2_q[c2_i201] - c2_qInit[c2_i201]);
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 65);
  for (c2_i202 = 0; c2_i202 < 25; c2_i202++) {
    c2_tau0[c2_i202] = c2_LinearizedDynamics[c2_i202];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 67);
  for (c2_i203 = 0; c2_i203 < 625; c2_i203++) {
    c2_e_y[c2_i203] = c2_N0[c2_i203];
  }

  for (c2_i204 = 0; c2_i204 < 25; c2_i204++) {
    c2_m_b[c2_i204] = c2_tau0[c2_i204];
  }

  c2_q_eml_scalar_eg(chartInstance);
  c2_q_eml_scalar_eg(chartInstance);
  for (c2_i205 = 0; c2_i205 < 25; c2_i205++) {
    c2_c_C[c2_i205] = 0.0;
  }

  for (c2_i206 = 0; c2_i206 < 625; c2_i206++) {
    c2_j_y[c2_i206] = c2_e_y[c2_i206];
  }

  for (c2_i207 = 0; c2_i207 < 25; c2_i207++) {
    c2_n_b[c2_i207] = c2_m_b[c2_i207];
  }

  c2_r_eml_xgemm(chartInstance, c2_j_y, c2_n_b, c2_c_C);
  for (c2_i208 = 0; c2_i208 < 25; c2_i208++) {
    c2_tau[c2_i208] = c2_tau1[c2_i208] + c2_c_C[c2_i208];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 71);
  for (c2_i209 = 0; c2_i209 < 3; c2_i209++) {
    c2_CoMError[c2_i209] = c2_xcom[c2_i209] - c2_Desired_x_dx_ddx_CoM[c2_i209];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 72);
  for (c2_i210 = 0; c2_i210 < 3; c2_i210++) {
    c2_b_CoMError[c2_i210] = c2_CoMError[c2_i210];
  }

  c2_normCoMError = c2_b_norm(chartInstance, c2_b_CoMError);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, -72);
  sf_debug_symbol_scope_pop();
  for (c2_i211 = 0; c2_i211 < 25; c2_i211++) {
    (*c2_b_tau)[c2_i211] = c2_tau[c2_i211];
  }

  for (c2_i212 = 0; c2_i212 < 3; c2_i212++) {
    (*c2_c_CoMError)[c2_i212] = c2_CoMError[c2_i212];
  }

  *c2_b_normCoMError = c2_normCoMError;
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 1U, chartInstance->c2_sfEvent);
}

static void initSimStructsc2_controller(SFc2_controllerInstanceStruct
  *chartInstance)
{
}

static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber)
{
  _SFD_SCRIPT_TRANSLATION(c2_chartNumber, 0U, sf_debug_get_script_id(
    "/home/daniele/src/codyco/src/simulink/controllers/controllerDemoCodycoY1/Sf.m"));
}

static const mxArray *c2_sf_marshallOut(void *chartInstanceVoid, void *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  real_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_controllerInstanceStruct *chartInstance;
  chartInstance = (SFc2_controllerInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(real_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static real_T c2_emlrt_marshallIn(SFc2_controllerInstanceStruct *chartInstance,
  const mxArray *c2_normCoMError, const char_T *c2_identifier)
{
  real_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_normCoMError),
    &c2_thisId);
  sf_mex_destroy(&c2_normCoMError);
  return c2_y;
}

static real_T c2_b_emlrt_marshallIn(SFc2_controllerInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  real_T c2_y;
  real_T c2_d0;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_d0, 1, 0, 0U, 0, 0U, 0);
  c2_y = c2_d0;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_normCoMError;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y;
  SFc2_controllerInstanceStruct *chartInstance;
  chartInstance = (SFc2_controllerInstanceStruct *)chartInstanceVoid;
  c2_normCoMError = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_normCoMError),
    &c2_thisId);
  sf_mex_destroy(&c2_normCoMError);
  *(real_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_b_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i213;
  real_T c2_b_inData[3];
  int32_T c2_i214;
  real_T c2_u[3];
  const mxArray *c2_y = NULL;
  SFc2_controllerInstanceStruct *chartInstance;
  chartInstance = (SFc2_controllerInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i213 = 0; c2_i213 < 3; c2_i213++) {
    c2_b_inData[c2_i213] = (*(real_T (*)[3])c2_inData)[c2_i213];
  }

  for (c2_i214 = 0; c2_i214 < 3; c2_i214++) {
    c2_u[c2_i214] = c2_b_inData[c2_i214];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 3), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_c_emlrt_marshallIn(SFc2_controllerInstanceStruct *chartInstance,
  const mxArray *c2_CoMError, const char_T *c2_identifier, real_T c2_y[3])
{
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_CoMError), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_CoMError);
}

static void c2_d_emlrt_marshallIn(SFc2_controllerInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[3])
{
  real_T c2_dv16[3];
  int32_T c2_i215;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv16, 1, 0, 0U, 1, 0U, 1, 3);
  for (c2_i215 = 0; c2_i215 < 3; c2_i215++) {
    c2_y[c2_i215] = c2_dv16[c2_i215];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_CoMError;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[3];
  int32_T c2_i216;
  SFc2_controllerInstanceStruct *chartInstance;
  chartInstance = (SFc2_controllerInstanceStruct *)chartInstanceVoid;
  c2_CoMError = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_CoMError), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_CoMError);
  for (c2_i216 = 0; c2_i216 < 3; c2_i216++) {
    (*(real_T (*)[3])c2_outData)[c2_i216] = c2_y[c2_i216];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_c_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i217;
  real_T c2_b_inData[25];
  int32_T c2_i218;
  real_T c2_u[25];
  const mxArray *c2_y = NULL;
  SFc2_controllerInstanceStruct *chartInstance;
  chartInstance = (SFc2_controllerInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i217 = 0; c2_i217 < 25; c2_i217++) {
    c2_b_inData[c2_i217] = (*(real_T (*)[25])c2_inData)[c2_i217];
  }

  for (c2_i218 = 0; c2_i218 < 25; c2_i218++) {
    c2_u[c2_i218] = c2_b_inData[c2_i218];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 25), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_e_emlrt_marshallIn(SFc2_controllerInstanceStruct *chartInstance,
  const mxArray *c2_tau, const char_T *c2_identifier, real_T c2_y[25])
{
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_tau), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_tau);
}

static void c2_f_emlrt_marshallIn(SFc2_controllerInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[25])
{
  real_T c2_dv17[25];
  int32_T c2_i219;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv17, 1, 0, 0U, 1, 0U, 1, 25);
  for (c2_i219 = 0; c2_i219 < 25; c2_i219++) {
    c2_y[c2_i219] = c2_dv17[c2_i219];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_tau;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[25];
  int32_T c2_i220;
  SFc2_controllerInstanceStruct *chartInstance;
  chartInstance = (SFc2_controllerInstanceStruct *)chartInstanceVoid;
  c2_tau = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_tau), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_tau);
  for (c2_i220 = 0; c2_i220 < 25; c2_i220++) {
    (*(real_T (*)[25])c2_outData)[c2_i220] = c2_y[c2_i220];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_d_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i221;
  real_T c2_b_inData[4];
  int32_T c2_i222;
  real_T c2_u[4];
  const mxArray *c2_y = NULL;
  SFc2_controllerInstanceStruct *chartInstance;
  chartInstance = (SFc2_controllerInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i221 = 0; c2_i221 < 4; c2_i221++) {
    c2_b_inData[c2_i221] = (*(real_T (*)[4])c2_inData)[c2_i221];
  }

  for (c2_i222 = 0; c2_i222 < 4; c2_i222++) {
    c2_u[c2_i222] = c2_b_inData[c2_i222];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 4), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static const mxArray *c2_e_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i223;
  int32_T c2_i224;
  int32_T c2_i225;
  real_T c2_b_inData[9];
  int32_T c2_i226;
  int32_T c2_i227;
  int32_T c2_i228;
  real_T c2_u[9];
  const mxArray *c2_y = NULL;
  SFc2_controllerInstanceStruct *chartInstance;
  chartInstance = (SFc2_controllerInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i223 = 0;
  for (c2_i224 = 0; c2_i224 < 3; c2_i224++) {
    for (c2_i225 = 0; c2_i225 < 3; c2_i225++) {
      c2_b_inData[c2_i225 + c2_i223] = (*(real_T (*)[9])c2_inData)[c2_i225 +
        c2_i223];
    }

    c2_i223 += 3;
  }

  c2_i226 = 0;
  for (c2_i227 = 0; c2_i227 < 3; c2_i227++) {
    for (c2_i228 = 0; c2_i228 < 3; c2_i228++) {
      c2_u[c2_i228 + c2_i226] = c2_b_inData[c2_i228 + c2_i226];
    }

    c2_i226 += 3;
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 3, 3), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static const mxArray *c2_f_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i229;
  int32_T c2_i230;
  int32_T c2_i231;
  real_T c2_b_inData[186];
  int32_T c2_i232;
  int32_T c2_i233;
  int32_T c2_i234;
  real_T c2_u[186];
  const mxArray *c2_y = NULL;
  SFc2_controllerInstanceStruct *chartInstance;
  chartInstance = (SFc2_controllerInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i229 = 0;
  for (c2_i230 = 0; c2_i230 < 31; c2_i230++) {
    for (c2_i231 = 0; c2_i231 < 6; c2_i231++) {
      c2_b_inData[c2_i231 + c2_i229] = (*(real_T (*)[186])c2_inData)[c2_i231 +
        c2_i229];
    }

    c2_i229 += 6;
  }

  c2_i232 = 0;
  for (c2_i233 = 0; c2_i233 < 31; c2_i233++) {
    for (c2_i234 = 0; c2_i234 < 6; c2_i234++) {
      c2_u[c2_i234 + c2_i232] = c2_b_inData[c2_i234 + c2_i232];
    }

    c2_i232 += 6;
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 6, 31), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static const mxArray *c2_g_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i235;
  real_T c2_b_inData[12];
  int32_T c2_i236;
  real_T c2_u[12];
  const mxArray *c2_y = NULL;
  SFc2_controllerInstanceStruct *chartInstance;
  chartInstance = (SFc2_controllerInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i235 = 0; c2_i235 < 12; c2_i235++) {
    c2_b_inData[c2_i235] = (*(real_T (*)[12])c2_inData)[c2_i235];
  }

  for (c2_i236 = 0; c2_i236 < 12; c2_i236++) {
    c2_u[c2_i236] = c2_b_inData[c2_i236];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 12, 1), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static const mxArray *c2_h_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i237;
  int32_T c2_i238;
  int32_T c2_i239;
  real_T c2_b_inData[372];
  int32_T c2_i240;
  int32_T c2_i241;
  int32_T c2_i242;
  real_T c2_u[372];
  const mxArray *c2_y = NULL;
  SFc2_controllerInstanceStruct *chartInstance;
  chartInstance = (SFc2_controllerInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i237 = 0;
  for (c2_i238 = 0; c2_i238 < 31; c2_i238++) {
    for (c2_i239 = 0; c2_i239 < 12; c2_i239++) {
      c2_b_inData[c2_i239 + c2_i237] = (*(real_T (*)[372])c2_inData)[c2_i239 +
        c2_i237];
    }

    c2_i237 += 12;
  }

  c2_i240 = 0;
  for (c2_i241 = 0; c2_i241 < 31; c2_i241++) {
    for (c2_i242 = 0; c2_i242 < 12; c2_i242++) {
      c2_u[c2_i242 + c2_i240] = c2_b_inData[c2_i242 + c2_i240];
    }

    c2_i240 += 12;
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 12, 31), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static const mxArray *c2_i_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i243;
  real_T c2_b_inData[7];
  int32_T c2_i244;
  real_T c2_u[7];
  const mxArray *c2_y = NULL;
  SFc2_controllerInstanceStruct *chartInstance;
  chartInstance = (SFc2_controllerInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i243 = 0; c2_i243 < 7; c2_i243++) {
    c2_b_inData[c2_i243] = (*(real_T (*)[7])c2_inData)[c2_i243];
  }

  for (c2_i244 = 0; c2_i244 < 7; c2_i244++) {
    c2_u[c2_i244] = c2_b_inData[c2_i244];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 7), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static const mxArray *c2_j_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i245;
  real_T c2_b_inData[6];
  int32_T c2_i246;
  real_T c2_u[6];
  const mxArray *c2_y = NULL;
  SFc2_controllerInstanceStruct *chartInstance;
  chartInstance = (SFc2_controllerInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i245 = 0; c2_i245 < 6; c2_i245++) {
    c2_b_inData[c2_i245] = (*(real_T (*)[6])c2_inData)[c2_i245];
  }

  for (c2_i246 = 0; c2_i246 < 6; c2_i246++) {
    c2_u[c2_i246] = c2_b_inData[c2_i246];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 6), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static const mxArray *c2_k_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i247;
  real_T c2_b_inData[31];
  int32_T c2_i248;
  real_T c2_u[31];
  const mxArray *c2_y = NULL;
  SFc2_controllerInstanceStruct *chartInstance;
  chartInstance = (SFc2_controllerInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i247 = 0; c2_i247 < 31; c2_i247++) {
    c2_b_inData[c2_i247] = (*(real_T (*)[31])c2_inData)[c2_i247];
  }

  for (c2_i248 = 0; c2_i248 < 31; c2_i248++) {
    c2_u[c2_i248] = c2_b_inData[c2_i248];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 31), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static const mxArray *c2_l_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i249;
  int32_T c2_i250;
  int32_T c2_i251;
  real_T c2_b_inData[961];
  int32_T c2_i252;
  int32_T c2_i253;
  int32_T c2_i254;
  real_T c2_u[961];
  const mxArray *c2_y = NULL;
  SFc2_controllerInstanceStruct *chartInstance;
  chartInstance = (SFc2_controllerInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i249 = 0;
  for (c2_i250 = 0; c2_i250 < 31; c2_i250++) {
    for (c2_i251 = 0; c2_i251 < 31; c2_i251++) {
      c2_b_inData[c2_i251 + c2_i249] = (*(real_T (*)[961])c2_inData)[c2_i251 +
        c2_i249];
    }

    c2_i249 += 31;
  }

  c2_i252 = 0;
  for (c2_i253 = 0; c2_i253 < 31; c2_i253++) {
    for (c2_i254 = 0; c2_i254 < 31; c2_i254++) {
      c2_u[c2_i254 + c2_i252] = c2_b_inData[c2_i254 + c2_i252];
    }

    c2_i252 += 31;
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 31, 31), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static const mxArray *c2_m_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i255;
  real_T c2_b_inData[18];
  int32_T c2_i256;
  real_T c2_u[18];
  const mxArray *c2_y = NULL;
  SFc2_controllerInstanceStruct *chartInstance;
  chartInstance = (SFc2_controllerInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i255 = 0; c2_i255 < 18; c2_i255++) {
    c2_b_inData[c2_i255] = (*(real_T (*)[18])c2_inData)[c2_i255];
  }

  for (c2_i256 = 0; c2_i256 < 18; c2_i256++) {
    c2_u[c2_i256] = c2_b_inData[c2_i256];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 18, 1), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static const mxArray *c2_n_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i257;
  real_T c2_b_inData[18];
  int32_T c2_i258;
  real_T c2_u[18];
  const mxArray *c2_y = NULL;
  SFc2_controllerInstanceStruct *chartInstance;
  chartInstance = (SFc2_controllerInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i257 = 0; c2_i257 < 18; c2_i257++) {
    c2_b_inData[c2_i257] = (*(real_T (*)[18])c2_inData)[c2_i257];
  }

  for (c2_i258 = 0; c2_i258 < 18; c2_i258++) {
    c2_u[c2_i258] = c2_b_inData[c2_i258];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 18), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_g_emlrt_marshallIn(SFc2_controllerInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[18])
{
  real_T c2_dv18[18];
  int32_T c2_i259;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv18, 1, 0, 0U, 1, 0U, 1, 18);
  for (c2_i259 = 0; c2_i259 < 18; c2_i259++) {
    c2_y[c2_i259] = c2_dv18[c2_i259];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_measuredExternalWrench;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[18];
  int32_T c2_i260;
  SFc2_controllerInstanceStruct *chartInstance;
  chartInstance = (SFc2_controllerInstanceStruct *)chartInstanceVoid;
  c2_measuredExternalWrench = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_g_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_measuredExternalWrench),
                        &c2_thisId, c2_y);
  sf_mex_destroy(&c2_measuredExternalWrench);
  for (c2_i260 = 0; c2_i260 < 18; c2_i260++) {
    (*(real_T (*)[18])c2_outData)[c2_i260] = c2_y[c2_i260];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_o_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i261;
  int32_T c2_i262;
  int32_T c2_i263;
  real_T c2_b_inData[625];
  int32_T c2_i264;
  int32_T c2_i265;
  int32_T c2_i266;
  real_T c2_u[625];
  const mxArray *c2_y = NULL;
  SFc2_controllerInstanceStruct *chartInstance;
  chartInstance = (SFc2_controllerInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i261 = 0;
  for (c2_i262 = 0; c2_i262 < 25; c2_i262++) {
    for (c2_i263 = 0; c2_i263 < 25; c2_i263++) {
      c2_b_inData[c2_i263 + c2_i261] = (*(real_T (*)[625])c2_inData)[c2_i263 +
        c2_i261];
    }

    c2_i261 += 25;
  }

  c2_i264 = 0;
  for (c2_i265 = 0; c2_i265 < 25; c2_i265++) {
    for (c2_i266 = 0; c2_i266 < 25; c2_i266++) {
      c2_u[c2_i266 + c2_i264] = c2_b_inData[c2_i266 + c2_i264];
    }

    c2_i264 += 25;
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 25, 25), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_h_emlrt_marshallIn(SFc2_controllerInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[625])
{
  real_T c2_dv19[625];
  int32_T c2_i267;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv19, 1, 0, 0U, 1, 0U, 2, 25,
                25);
  for (c2_i267 = 0; c2_i267 < 625; c2_i267++) {
    c2_y[c2_i267] = c2_dv19[c2_i267];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_N0;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[625];
  int32_T c2_i268;
  int32_T c2_i269;
  int32_T c2_i270;
  SFc2_controllerInstanceStruct *chartInstance;
  chartInstance = (SFc2_controllerInstanceStruct *)chartInstanceVoid;
  c2_N0 = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_N0), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_N0);
  c2_i268 = 0;
  for (c2_i269 = 0; c2_i269 < 25; c2_i269++) {
    for (c2_i270 = 0; c2_i270 < 25; c2_i270++) {
      (*(real_T (*)[625])c2_outData)[c2_i270 + c2_i268] = c2_y[c2_i270 + c2_i268];
    }

    c2_i268 += 25;
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_p_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i271;
  int32_T c2_i272;
  int32_T c2_i273;
  real_T c2_b_inData[450];
  int32_T c2_i274;
  int32_T c2_i275;
  int32_T c2_i276;
  real_T c2_u[450];
  const mxArray *c2_y = NULL;
  SFc2_controllerInstanceStruct *chartInstance;
  chartInstance = (SFc2_controllerInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i271 = 0;
  for (c2_i272 = 0; c2_i272 < 18; c2_i272++) {
    for (c2_i273 = 0; c2_i273 < 25; c2_i273++) {
      c2_b_inData[c2_i273 + c2_i271] = (*(real_T (*)[450])c2_inData)[c2_i273 +
        c2_i271];
    }

    c2_i271 += 25;
  }

  c2_i274 = 0;
  for (c2_i275 = 0; c2_i275 < 18; c2_i275++) {
    for (c2_i276 = 0; c2_i276 < 25; c2_i276++) {
      c2_u[c2_i276 + c2_i274] = c2_b_inData[c2_i276 + c2_i274];
    }

    c2_i274 += 25;
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 25, 18), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_i_emlrt_marshallIn(SFc2_controllerInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[450])
{
  real_T c2_dv20[450];
  int32_T c2_i277;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv20, 1, 0, 0U, 1, 0U, 2, 25,
                18);
  for (c2_i277 = 0; c2_i277 < 450; c2_i277++) {
    c2_y[c2_i277] = c2_dv20[c2_i277];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_PInv_JcMinvSt;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[450];
  int32_T c2_i278;
  int32_T c2_i279;
  int32_T c2_i280;
  SFc2_controllerInstanceStruct *chartInstance;
  chartInstance = (SFc2_controllerInstanceStruct *)chartInstanceVoid;
  c2_PInv_JcMinvSt = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_i_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_PInv_JcMinvSt), &c2_thisId,
                        c2_y);
  sf_mex_destroy(&c2_PInv_JcMinvSt);
  c2_i278 = 0;
  for (c2_i279 = 0; c2_i279 < 18; c2_i279++) {
    for (c2_i280 = 0; c2_i280 < 25; c2_i280++) {
      (*(real_T (*)[450])c2_outData)[c2_i280 + c2_i278] = c2_y[c2_i280 + c2_i278];
    }

    c2_i278 += 25;
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_q_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i281;
  int32_T c2_i282;
  int32_T c2_i283;
  real_T c2_b_inData[558];
  int32_T c2_i284;
  int32_T c2_i285;
  int32_T c2_i286;
  real_T c2_u[558];
  const mxArray *c2_y = NULL;
  SFc2_controllerInstanceStruct *chartInstance;
  chartInstance = (SFc2_controllerInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i281 = 0;
  for (c2_i282 = 0; c2_i282 < 31; c2_i282++) {
    for (c2_i283 = 0; c2_i283 < 18; c2_i283++) {
      c2_b_inData[c2_i283 + c2_i281] = (*(real_T (*)[558])c2_inData)[c2_i283 +
        c2_i281];
    }

    c2_i281 += 18;
  }

  c2_i284 = 0;
  for (c2_i285 = 0; c2_i285 < 31; c2_i285++) {
    for (c2_i286 = 0; c2_i286 < 18; c2_i286++) {
      c2_u[c2_i286 + c2_i284] = c2_b_inData[c2_i286 + c2_i284];
    }

    c2_i284 += 18;
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 18, 31), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_j_emlrt_marshallIn(SFc2_controllerInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[558])
{
  real_T c2_dv21[558];
  int32_T c2_i287;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv21, 1, 0, 0U, 1, 0U, 2, 18,
                31);
  for (c2_i287 = 0; c2_i287 < 558; c2_i287++) {
    c2_y[c2_i287] = c2_dv21[c2_i287];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_JcMinv;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[558];
  int32_T c2_i288;
  int32_T c2_i289;
  int32_T c2_i290;
  SFc2_controllerInstanceStruct *chartInstance;
  chartInstance = (SFc2_controllerInstanceStruct *)chartInstanceVoid;
  c2_JcMinv = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_j_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_JcMinv), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_JcMinv);
  c2_i288 = 0;
  for (c2_i289 = 0; c2_i289 < 31; c2_i289++) {
    for (c2_i290 = 0; c2_i290 < 18; c2_i290++) {
      (*(real_T (*)[558])c2_outData)[c2_i290 + c2_i288] = c2_y[c2_i290 + c2_i288];
    }

    c2_i288 += 18;
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static void c2_k_emlrt_marshallIn(SFc2_controllerInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[961])
{
  real_T c2_dv22[961];
  int32_T c2_i291;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv22, 1, 0, 0U, 1, 0U, 2, 31,
                31);
  for (c2_i291 = 0; c2_i291 < 961; c2_i291++) {
    c2_y[c2_i291] = c2_dv22[c2_i291];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_h_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_Minv;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[961];
  int32_T c2_i292;
  int32_T c2_i293;
  int32_T c2_i294;
  SFc2_controllerInstanceStruct *chartInstance;
  chartInstance = (SFc2_controllerInstanceStruct *)chartInstanceVoid;
  c2_Minv = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_k_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_Minv), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_Minv);
  c2_i292 = 0;
  for (c2_i293 = 0; c2_i293 < 31; c2_i293++) {
    for (c2_i294 = 0; c2_i294 < 31; c2_i294++) {
      (*(real_T (*)[961])c2_outData)[c2_i294 + c2_i292] = c2_y[c2_i294 + c2_i292];
    }

    c2_i292 += 31;
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_r_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i295;
  real_T c2_b_inData[12];
  int32_T c2_i296;
  real_T c2_u[12];
  const mxArray *c2_y = NULL;
  SFc2_controllerInstanceStruct *chartInstance;
  chartInstance = (SFc2_controllerInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i295 = 0; c2_i295 < 12; c2_i295++) {
    c2_b_inData[c2_i295] = (*(real_T (*)[12])c2_inData)[c2_i295];
  }

  for (c2_i296 = 0; c2_i296 < 12; c2_i296++) {
    c2_u[c2_i296] = c2_b_inData[c2_i296];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 12), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_l_emlrt_marshallIn(SFc2_controllerInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[12])
{
  real_T c2_dv23[12];
  int32_T c2_i297;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv23, 1, 0, 0U, 1, 0U, 1, 12);
  for (c2_i297 = 0; c2_i297 < 12; c2_i297++) {
    c2_y[c2_i297] = c2_dv23[c2_i297];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_i_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_feetDesiredWrenches;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[12];
  int32_T c2_i298;
  SFc2_controllerInstanceStruct *chartInstance;
  chartInstance = (SFc2_controllerInstanceStruct *)chartInstanceVoid;
  c2_feetDesiredWrenches = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_l_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_feetDesiredWrenches),
                        &c2_thisId, c2_y);
  sf_mex_destroy(&c2_feetDesiredWrenches);
  for (c2_i298 = 0; c2_i298 < 12; c2_i298++) {
    (*(real_T (*)[12])c2_outData)[c2_i298] = c2_y[c2_i298];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static void c2_m_emlrt_marshallIn(SFc2_controllerInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[6])
{
  real_T c2_dv24[6];
  int32_T c2_i299;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv24, 1, 0, 0U, 1, 0U, 1, 6);
  for (c2_i299 = 0; c2_i299 < 6; c2_i299++) {
    c2_y[c2_i299] = c2_dv24[c2_i299];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_j_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_handsDesiredWrenches;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[6];
  int32_T c2_i300;
  SFc2_controllerInstanceStruct *chartInstance;
  chartInstance = (SFc2_controllerInstanceStruct *)chartInstanceVoid;
  c2_handsDesiredWrenches = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_m_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_handsDesiredWrenches),
                        &c2_thisId, c2_y);
  sf_mex_destroy(&c2_handsDesiredWrenches);
  for (c2_i300 = 0; c2_i300 < 6; c2_i300++) {
    (*(real_T (*)[6])c2_outData)[c2_i300] = c2_y[c2_i300];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_s_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i301;
  int32_T c2_i302;
  int32_T c2_i303;
  real_T c2_b_inData[72];
  int32_T c2_i304;
  int32_T c2_i305;
  int32_T c2_i306;
  real_T c2_u[72];
  const mxArray *c2_y = NULL;
  SFc2_controllerInstanceStruct *chartInstance;
  chartInstance = (SFc2_controllerInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i301 = 0;
  for (c2_i302 = 0; c2_i302 < 6; c2_i302++) {
    for (c2_i303 = 0; c2_i303 < 12; c2_i303++) {
      c2_b_inData[c2_i303 + c2_i301] = (*(real_T (*)[72])c2_inData)[c2_i303 +
        c2_i301];
    }

    c2_i301 += 12;
  }

  c2_i304 = 0;
  for (c2_i305 = 0; c2_i305 < 6; c2_i305++) {
    for (c2_i306 = 0; c2_i306 < 12; c2_i306++) {
      c2_u[c2_i306 + c2_i304] = c2_b_inData[c2_i306 + c2_i304];
    }

    c2_i304 += 12;
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 12, 6), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_n_emlrt_marshallIn(SFc2_controllerInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[72])
{
  real_T c2_dv25[72];
  int32_T c2_i307;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv25, 1, 0, 0U, 1, 0U, 2, 12,
                6);
  for (c2_i307 = 0; c2_i307 < 72; c2_i307++) {
    c2_y[c2_i307] = c2_dv25[c2_i307];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_k_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_pinvA;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[72];
  int32_T c2_i308;
  int32_T c2_i309;
  int32_T c2_i310;
  SFc2_controllerInstanceStruct *chartInstance;
  chartInstance = (SFc2_controllerInstanceStruct *)chartInstanceVoid;
  c2_pinvA = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_n_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_pinvA), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_pinvA);
  c2_i308 = 0;
  for (c2_i309 = 0; c2_i309 < 6; c2_i309++) {
    for (c2_i310 = 0; c2_i310 < 12; c2_i310++) {
      (*(real_T (*)[72])c2_outData)[c2_i310 + c2_i308] = c2_y[c2_i310 + c2_i308];
    }

    c2_i308 += 12;
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_t_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i311;
  int32_T c2_i312;
  int32_T c2_i313;
  real_T c2_b_inData[72];
  int32_T c2_i314;
  int32_T c2_i315;
  int32_T c2_i316;
  real_T c2_u[72];
  const mxArray *c2_y = NULL;
  SFc2_controllerInstanceStruct *chartInstance;
  chartInstance = (SFc2_controllerInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i311 = 0;
  for (c2_i312 = 0; c2_i312 < 12; c2_i312++) {
    for (c2_i313 = 0; c2_i313 < 6; c2_i313++) {
      c2_b_inData[c2_i313 + c2_i311] = (*(real_T (*)[72])c2_inData)[c2_i313 +
        c2_i311];
    }

    c2_i311 += 6;
  }

  c2_i314 = 0;
  for (c2_i315 = 0; c2_i315 < 12; c2_i315++) {
    for (c2_i316 = 0; c2_i316 < 6; c2_i316++) {
      c2_u[c2_i316 + c2_i314] = c2_b_inData[c2_i316 + c2_i314];
    }

    c2_i314 += 6;
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 6, 12), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_o_emlrt_marshallIn(SFc2_controllerInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[72])
{
  real_T c2_dv26[72];
  int32_T c2_i317;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv26, 1, 0, 0U, 1, 0U, 2, 6,
                12);
  for (c2_i317 = 0; c2_i317 < 72; c2_i317++) {
    c2_y[c2_i317] = c2_dv26[c2_i317];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_l_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_A;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[72];
  int32_T c2_i318;
  int32_T c2_i319;
  int32_T c2_i320;
  SFc2_controllerInstanceStruct *chartInstance;
  chartInstance = (SFc2_controllerInstanceStruct *)chartInstanceVoid;
  c2_A = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_o_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_A), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_A);
  c2_i318 = 0;
  for (c2_i319 = 0; c2_i319 < 12; c2_i319++) {
    for (c2_i320 = 0; c2_i320 < 6; c2_i320++) {
      (*(real_T (*)[72])c2_outData)[c2_i320 + c2_i318] = c2_y[c2_i320 + c2_i318];
    }

    c2_i318 += 6;
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_u_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i321;
  int32_T c2_i322;
  int32_T c2_i323;
  real_T c2_b_inData[775];
  int32_T c2_i324;
  int32_T c2_i325;
  int32_T c2_i326;
  real_T c2_u[775];
  const mxArray *c2_y = NULL;
  SFc2_controllerInstanceStruct *chartInstance;
  chartInstance = (SFc2_controllerInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i321 = 0;
  for (c2_i322 = 0; c2_i322 < 25; c2_i322++) {
    for (c2_i323 = 0; c2_i323 < 31; c2_i323++) {
      c2_b_inData[c2_i323 + c2_i321] = (*(real_T (*)[775])c2_inData)[c2_i323 +
        c2_i321];
    }

    c2_i321 += 31;
  }

  c2_i324 = 0;
  for (c2_i325 = 0; c2_i325 < 25; c2_i325++) {
    for (c2_i326 = 0; c2_i326 < 31; c2_i326++) {
      c2_u[c2_i326 + c2_i324] = c2_b_inData[c2_i326 + c2_i324];
    }

    c2_i324 += 31;
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 31, 25), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_p_emlrt_marshallIn(SFc2_controllerInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[9])
{
  real_T c2_dv27[9];
  int32_T c2_i327;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv27, 1, 0, 0U, 1, 0U, 2, 3, 3);
  for (c2_i327 = 0; c2_i327 < 9; c2_i327++) {
    c2_y[c2_i327] = c2_dv27[c2_i327];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_m_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_S;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[9];
  int32_T c2_i328;
  int32_T c2_i329;
  int32_T c2_i330;
  SFc2_controllerInstanceStruct *chartInstance;
  chartInstance = (SFc2_controllerInstanceStruct *)chartInstanceVoid;
  c2_S = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_p_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_S), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_S);
  c2_i328 = 0;
  for (c2_i329 = 0; c2_i329 < 3; c2_i329++) {
    for (c2_i330 = 0; c2_i330 < 3; c2_i330++) {
      (*(real_T (*)[9])c2_outData)[c2_i330 + c2_i328] = c2_y[c2_i330 + c2_i328];
    }

    c2_i328 += 3;
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

const mxArray *sf_c2_controller_get_eml_resolved_functions_info(void)
{
  const mxArray *c2_nameCaptureInfo;
  c2_ResolvedFunctionInfo c2_info[268];
  const mxArray *c2_m0 = NULL;
  int32_T c2_i331;
  c2_ResolvedFunctionInfo *c2_r0;
  c2_nameCaptureInfo = NULL;
  c2_nameCaptureInfo = NULL;
  c2_info_helper(c2_info);
  c2_b_info_helper(c2_info);
  c2_c_info_helper(c2_info);
  c2_d_info_helper(c2_info);
  c2_e_info_helper(c2_info);
  sf_mex_assign(&c2_m0, sf_mex_createstruct("nameCaptureInfo", 1, 268), FALSE);
  for (c2_i331 = 0; c2_i331 < 268; c2_i331++) {
    c2_r0 = &c2_info[c2_i331];
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c2_r0->context)), "context", "nameCaptureInfo",
                    c2_i331);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->name, 15, 0U,
      0U, 0U, 2, 1, strlen(c2_r0->name)), "name", "nameCaptureInfo", c2_i331);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->dominantType,
      15, 0U, 0U, 0U, 2, 1, strlen(c2_r0->dominantType)), "dominantType",
                    "nameCaptureInfo", c2_i331);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->resolved, 15,
      0U, 0U, 0U, 2, 1, strlen(c2_r0->resolved)), "resolved", "nameCaptureInfo",
                    c2_i331);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", &c2_r0->fileTimeLo,
      7, 0U, 0U, 0U, 0), "fileTimeLo", "nameCaptureInfo", c2_i331);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", &c2_r0->fileTimeHi,
      7, 0U, 0U, 0U, 0), "fileTimeHi", "nameCaptureInfo", c2_i331);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", &c2_r0->mFileTimeLo,
      7, 0U, 0U, 0U, 0), "mFileTimeLo", "nameCaptureInfo", c2_i331);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", &c2_r0->mFileTimeHi,
      7, 0U, 0U, 0U, 0), "mFileTimeHi", "nameCaptureInfo", c2_i331);
  }

  sf_mex_assign(&c2_nameCaptureInfo, c2_m0, FALSE);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c2_nameCaptureInfo);
  return c2_nameCaptureInfo;
}

static void c2_info_helper(c2_ResolvedFunctionInfo c2_info[268])
{
  c2_info[0].context = "";
  c2_info[0].name = "eye";
  c2_info[0].dominantType = "double";
  c2_info[0].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m";
  c2_info[0].fileTimeLo = 1286818688U;
  c2_info[0].fileTimeHi = 0U;
  c2_info[0].mFileTimeLo = 0U;
  c2_info[0].mFileTimeHi = 0U;
  c2_info[1].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c2_info[1].name = "eml_assert_valid_size_arg";
  c2_info[1].dominantType = "double";
  c2_info[1].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m";
  c2_info[1].fileTimeLo = 1286818694U;
  c2_info[1].fileTimeHi = 0U;
  c2_info[1].mFileTimeLo = 0U;
  c2_info[1].mFileTimeHi = 0U;
  c2_info[2].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!isintegral";
  c2_info[2].name = "isinf";
  c2_info[2].dominantType = "double";
  c2_info[2].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isinf.m";
  c2_info[2].fileTimeLo = 1286818760U;
  c2_info[2].fileTimeHi = 0U;
  c2_info[2].mFileTimeLo = 0U;
  c2_info[2].mFileTimeHi = 0U;
  c2_info[3].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!numel_for_size";
  c2_info[3].name = "mtimes";
  c2_info[3].dominantType = "double";
  c2_info[3].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[3].fileTimeLo = 1289519692U;
  c2_info[3].fileTimeHi = 0U;
  c2_info[3].mFileTimeLo = 0U;
  c2_info[3].mFileTimeHi = 0U;
  c2_info[4].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m";
  c2_info[4].name = "eml_index_class";
  c2_info[4].dominantType = "";
  c2_info[4].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[4].fileTimeLo = 1323170578U;
  c2_info[4].fileTimeHi = 0U;
  c2_info[4].mFileTimeLo = 0U;
  c2_info[4].mFileTimeHi = 0U;
  c2_info[5].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m";
  c2_info[5].name = "intmax";
  c2_info[5].dominantType = "char";
  c2_info[5].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c2_info[5].fileTimeLo = 1311255316U;
  c2_info[5].fileTimeHi = 0U;
  c2_info[5].mFileTimeLo = 0U;
  c2_info[5].mFileTimeHi = 0U;
  c2_info[6].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c2_info[6].name = "eml_is_float_class";
  c2_info[6].dominantType = "char";
  c2_info[6].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_is_float_class.m";
  c2_info[6].fileTimeLo = 1286818782U;
  c2_info[6].fileTimeHi = 0U;
  c2_info[6].mFileTimeLo = 0U;
  c2_info[6].mFileTimeHi = 0U;
  c2_info[7].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c2_info[7].name = "min";
  c2_info[7].dominantType = "double";
  c2_info[7].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c2_info[7].fileTimeLo = 1311255318U;
  c2_info[7].fileTimeHi = 0U;
  c2_info[7].mFileTimeLo = 0U;
  c2_info[7].mFileTimeHi = 0U;
  c2_info[8].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c2_info[8].name = "eml_min_or_max";
  c2_info[8].dominantType = "char";
  c2_info[8].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m";
  c2_info[8].fileTimeLo = 1334071490U;
  c2_info[8].fileTimeHi = 0U;
  c2_info[8].mFileTimeLo = 0U;
  c2_info[8].mFileTimeHi = 0U;
  c2_info[9].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c2_info[9].name = "eml_scalar_eg";
  c2_info[9].dominantType = "double";
  c2_info[9].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[9].fileTimeLo = 1286818796U;
  c2_info[9].fileTimeHi = 0U;
  c2_info[9].mFileTimeLo = 0U;
  c2_info[9].mFileTimeHi = 0U;
  c2_info[10].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c2_info[10].name = "eml_scalexp_alloc";
  c2_info[10].dominantType = "double";
  c2_info[10].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c2_info[10].fileTimeLo = 1330608434U;
  c2_info[10].fileTimeHi = 0U;
  c2_info[10].mFileTimeLo = 0U;
  c2_info[10].mFileTimeHi = 0U;
  c2_info[11].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c2_info[11].name = "eml_index_class";
  c2_info[11].dominantType = "";
  c2_info[11].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[11].fileTimeLo = 1323170578U;
  c2_info[11].fileTimeHi = 0U;
  c2_info[11].mFileTimeLo = 0U;
  c2_info[11].mFileTimeHi = 0U;
  c2_info[12].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum";
  c2_info[12].name = "eml_scalar_eg";
  c2_info[12].dominantType = "double";
  c2_info[12].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[12].fileTimeLo = 1286818796U;
  c2_info[12].fileTimeHi = 0U;
  c2_info[12].mFileTimeLo = 0U;
  c2_info[12].mFileTimeHi = 0U;
  c2_info[13].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c2_info[13].name = "eml_index_class";
  c2_info[13].dominantType = "";
  c2_info[13].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[13].fileTimeLo = 1323170578U;
  c2_info[13].fileTimeHi = 0U;
  c2_info[13].mFileTimeLo = 0U;
  c2_info[13].mFileTimeHi = 0U;
  c2_info[14].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c2_info[14].name = "eml_int_forloop_overflow_check";
  c2_info[14].dominantType = "";
  c2_info[14].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[14].fileTimeLo = 1332168672U;
  c2_info[14].fileTimeHi = 0U;
  c2_info[14].mFileTimeLo = 0U;
  c2_info[14].mFileTimeHi = 0U;
  c2_info[15].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper";
  c2_info[15].name = "intmax";
  c2_info[15].dominantType = "char";
  c2_info[15].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c2_info[15].fileTimeLo = 1311255316U;
  c2_info[15].fileTimeHi = 0U;
  c2_info[15].mFileTimeLo = 0U;
  c2_info[15].mFileTimeHi = 0U;
  c2_info[16].context = "";
  c2_info[16].name = "mtimes";
  c2_info[16].dominantType = "double";
  c2_info[16].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[16].fileTimeLo = 1289519692U;
  c2_info[16].fileTimeHi = 0U;
  c2_info[16].mFileTimeLo = 0U;
  c2_info[16].mFileTimeHi = 0U;
  c2_info[17].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[17].name = "eml_index_class";
  c2_info[17].dominantType = "";
  c2_info[17].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[17].fileTimeLo = 1323170578U;
  c2_info[17].fileTimeHi = 0U;
  c2_info[17].mFileTimeLo = 0U;
  c2_info[17].mFileTimeHi = 0U;
  c2_info[18].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[18].name = "eml_scalar_eg";
  c2_info[18].dominantType = "double";
  c2_info[18].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[18].fileTimeLo = 1286818796U;
  c2_info[18].fileTimeHi = 0U;
  c2_info[18].mFileTimeLo = 0U;
  c2_info[18].mFileTimeHi = 0U;
  c2_info[19].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[19].name = "eml_xgemm";
  c2_info[19].dominantType = "char";
  c2_info[19].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m";
  c2_info[19].fileTimeLo = 1299076772U;
  c2_info[19].fileTimeHi = 0U;
  c2_info[19].mFileTimeLo = 0U;
  c2_info[19].mFileTimeHi = 0U;
  c2_info[20].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m";
  c2_info[20].name = "eml_blas_inline";
  c2_info[20].dominantType = "";
  c2_info[20].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c2_info[20].fileTimeLo = 1299076768U;
  c2_info[20].fileTimeHi = 0U;
  c2_info[20].mFileTimeLo = 0U;
  c2_info[20].mFileTimeHi = 0U;
  c2_info[21].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m!below_threshold";
  c2_info[21].name = "mtimes";
  c2_info[21].dominantType = "double";
  c2_info[21].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[21].fileTimeLo = 1289519692U;
  c2_info[21].fileTimeHi = 0U;
  c2_info[21].mFileTimeLo = 0U;
  c2_info[21].mFileTimeHi = 0U;
  c2_info[22].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c2_info[22].name = "eml_index_class";
  c2_info[22].dominantType = "";
  c2_info[22].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[22].fileTimeLo = 1323170578U;
  c2_info[22].fileTimeHi = 0U;
  c2_info[22].mFileTimeLo = 0U;
  c2_info[22].mFileTimeHi = 0U;
  c2_info[23].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c2_info[23].name = "eml_scalar_eg";
  c2_info[23].dominantType = "double";
  c2_info[23].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[23].fileTimeLo = 1286818796U;
  c2_info[23].fileTimeHi = 0U;
  c2_info[23].mFileTimeLo = 0U;
  c2_info[23].mFileTimeHi = 0U;
  c2_info[24].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c2_info[24].name = "eml_refblas_xgemm";
  c2_info[24].dominantType = "char";
  c2_info[24].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemm.m";
  c2_info[24].fileTimeLo = 1299076774U;
  c2_info[24].fileTimeHi = 0U;
  c2_info[24].mFileTimeLo = 0U;
  c2_info[24].mFileTimeHi = 0U;
  c2_info[25].context = "";
  c2_info[25].name = "Sf";
  c2_info[25].dominantType = "double";
  c2_info[25].resolved =
    "[E]/home/daniele/src/codyco/src/simulink/controllers/controllerDemoCodycoY1/Sf.m";
  c2_info[25].fileTimeLo = 1397737223U;
  c2_info[25].fileTimeHi = 0U;
  c2_info[25].mFileTimeLo = 0U;
  c2_info[25].mFileTimeHi = 0U;
  c2_info[26].context = "";
  c2_info[26].name = "pinv";
  c2_info[26].dominantType = "double";
  c2_info[26].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m";
  c2_info[26].fileTimeLo = 1286818828U;
  c2_info[26].fileTimeHi = 0U;
  c2_info[26].mFileTimeLo = 0U;
  c2_info[26].mFileTimeHi = 0U;
  c2_info[27].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv";
  c2_info[27].name = "eml_index_class";
  c2_info[27].dominantType = "";
  c2_info[27].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[27].fileTimeLo = 1323170578U;
  c2_info[27].fileTimeHi = 0U;
  c2_info[27].mFileTimeLo = 0U;
  c2_info[27].mFileTimeHi = 0U;
  c2_info[28].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv";
  c2_info[28].name = "eml_scalar_eg";
  c2_info[28].dominantType = "double";
  c2_info[28].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[28].fileTimeLo = 1286818796U;
  c2_info[28].fileTimeHi = 0U;
  c2_info[28].mFileTimeLo = 0U;
  c2_info[28].mFileTimeHi = 0U;
  c2_info[29].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv";
  c2_info[29].name = "svd";
  c2_info[29].dominantType = "double";
  c2_info[29].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/svd.m";
  c2_info[29].fileTimeLo = 1286818832U;
  c2_info[29].fileTimeHi = 0U;
  c2_info[29].mFileTimeLo = 0U;
  c2_info[29].mFileTimeHi = 0U;
  c2_info[30].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/svd.m";
  c2_info[30].name = "eml_index_class";
  c2_info[30].dominantType = "";
  c2_info[30].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[30].fileTimeLo = 1323170578U;
  c2_info[30].fileTimeHi = 0U;
  c2_info[30].mFileTimeLo = 0U;
  c2_info[30].mFileTimeHi = 0U;
  c2_info[31].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/svd.m";
  c2_info[31].name = "eml_int_forloop_overflow_check";
  c2_info[31].dominantType = "";
  c2_info[31].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[31].fileTimeLo = 1332168672U;
  c2_info[31].fileTimeHi = 0U;
  c2_info[31].mFileTimeLo = 0U;
  c2_info[31].mFileTimeHi = 0U;
  c2_info[32].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/svd.m";
  c2_info[32].name = "isfinite";
  c2_info[32].dominantType = "double";
  c2_info[32].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isfinite.m";
  c2_info[32].fileTimeLo = 1286818758U;
  c2_info[32].fileTimeHi = 0U;
  c2_info[32].mFileTimeLo = 0U;
  c2_info[32].mFileTimeHi = 0U;
  c2_info[33].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isfinite.m";
  c2_info[33].name = "isinf";
  c2_info[33].dominantType = "double";
  c2_info[33].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isinf.m";
  c2_info[33].fileTimeLo = 1286818760U;
  c2_info[33].fileTimeHi = 0U;
  c2_info[33].mFileTimeLo = 0U;
  c2_info[33].mFileTimeHi = 0U;
  c2_info[34].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isfinite.m";
  c2_info[34].name = "isnan";
  c2_info[34].dominantType = "double";
  c2_info[34].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m";
  c2_info[34].fileTimeLo = 1286818760U;
  c2_info[34].fileTimeHi = 0U;
  c2_info[34].mFileTimeLo = 0U;
  c2_info[34].mFileTimeHi = 0U;
  c2_info[35].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/svd.m";
  c2_info[35].name = "eml_error";
  c2_info[35].dominantType = "char";
  c2_info[35].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m";
  c2_info[35].fileTimeLo = 1305318000U;
  c2_info[35].fileTimeHi = 0U;
  c2_info[35].mFileTimeLo = 0U;
  c2_info[35].mFileTimeHi = 0U;
  c2_info[36].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/svd.m";
  c2_info[36].name = "eml_xgesvd";
  c2_info[36].dominantType = "double";
  c2_info[36].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/eml_xgesvd.m";
  c2_info[36].fileTimeLo = 1286818806U;
  c2_info[36].fileTimeHi = 0U;
  c2_info[36].mFileTimeLo = 0U;
  c2_info[36].mFileTimeHi = 0U;
  c2_info[37].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/eml_xgesvd.m";
  c2_info[37].name = "eml_lapack_xgesvd";
  c2_info[37].dominantType = "double";
  c2_info[37].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/internal/eml_lapack_xgesvd.m";
  c2_info[37].fileTimeLo = 1286818810U;
  c2_info[37].fileTimeHi = 0U;
  c2_info[37].mFileTimeLo = 0U;
  c2_info[37].mFileTimeHi = 0U;
  c2_info[38].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/internal/eml_lapack_xgesvd.m";
  c2_info[38].name = "eml_matlab_zsvdc";
  c2_info[38].dominantType = "double";
  c2_info[38].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c2_info[38].fileTimeLo = 1295284866U;
  c2_info[38].fileTimeHi = 0U;
  c2_info[38].mFileTimeLo = 0U;
  c2_info[38].mFileTimeHi = 0U;
  c2_info[39].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c2_info[39].name = "eml_index_class";
  c2_info[39].dominantType = "";
  c2_info[39].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[39].fileTimeLo = 1323170578U;
  c2_info[39].fileTimeHi = 0U;
  c2_info[39].mFileTimeLo = 0U;
  c2_info[39].mFileTimeHi = 0U;
  c2_info[40].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c2_info[40].name = "eml_scalar_eg";
  c2_info[40].dominantType = "double";
  c2_info[40].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[40].fileTimeLo = 1286818796U;
  c2_info[40].fileTimeHi = 0U;
  c2_info[40].mFileTimeLo = 0U;
  c2_info[40].mFileTimeHi = 0U;
  c2_info[41].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c2_info[41].name = "eml_index_plus";
  c2_info[41].dominantType = "double";
  c2_info[41].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[41].fileTimeLo = 1286818778U;
  c2_info[41].fileTimeHi = 0U;
  c2_info[41].mFileTimeLo = 0U;
  c2_info[41].mFileTimeHi = 0U;
  c2_info[42].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[42].name = "eml_index_class";
  c2_info[42].dominantType = "";
  c2_info[42].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[42].fileTimeLo = 1323170578U;
  c2_info[42].fileTimeHi = 0U;
  c2_info[42].mFileTimeLo = 0U;
  c2_info[42].mFileTimeHi = 0U;
  c2_info[43].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c2_info[43].name = "min";
  c2_info[43].dominantType = "coder.internal.indexInt";
  c2_info[43].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c2_info[43].fileTimeLo = 1311255318U;
  c2_info[43].fileTimeHi = 0U;
  c2_info[43].mFileTimeLo = 0U;
  c2_info[43].mFileTimeHi = 0U;
  c2_info[44].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c2_info[44].name = "eml_scalar_eg";
  c2_info[44].dominantType = "coder.internal.indexInt";
  c2_info[44].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[44].fileTimeLo = 1286818796U;
  c2_info[44].fileTimeHi = 0U;
  c2_info[44].mFileTimeLo = 0U;
  c2_info[44].mFileTimeHi = 0U;
  c2_info[45].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c2_info[45].name = "eml_scalexp_alloc";
  c2_info[45].dominantType = "coder.internal.indexInt";
  c2_info[45].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c2_info[45].fileTimeLo = 1330608434U;
  c2_info[45].fileTimeHi = 0U;
  c2_info[45].mFileTimeLo = 0U;
  c2_info[45].mFileTimeHi = 0U;
  c2_info[46].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum";
  c2_info[46].name = "eml_scalar_eg";
  c2_info[46].dominantType = "coder.internal.indexInt";
  c2_info[46].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[46].fileTimeLo = 1286818796U;
  c2_info[46].fileTimeHi = 0U;
  c2_info[46].mFileTimeLo = 0U;
  c2_info[46].mFileTimeHi = 0U;
  c2_info[47].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c2_info[47].name = "max";
  c2_info[47].dominantType = "double";
  c2_info[47].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/max.m";
  c2_info[47].fileTimeLo = 1311255316U;
  c2_info[47].fileTimeHi = 0U;
  c2_info[47].mFileTimeLo = 0U;
  c2_info[47].mFileTimeHi = 0U;
  c2_info[48].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/max.m";
  c2_info[48].name = "eml_min_or_max";
  c2_info[48].dominantType = "char";
  c2_info[48].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m";
  c2_info[48].fileTimeLo = 1334071490U;
  c2_info[48].fileTimeHi = 0U;
  c2_info[48].mFileTimeLo = 0U;
  c2_info[48].mFileTimeHi = 0U;
  c2_info[49].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum";
  c2_info[49].name = "eml_relop";
  c2_info[49].dominantType = "function_handle";
  c2_info[49].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_relop.m";
  c2_info[49].fileTimeLo = 1326727998U;
  c2_info[49].fileTimeHi = 0U;
  c2_info[49].mFileTimeLo = 0U;
  c2_info[49].mFileTimeHi = 0U;
  c2_info[50].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_relop.m";
  c2_info[50].name = "coder.internal.indexIntRelop";
  c2_info[50].dominantType = "";
  c2_info[50].resolved =
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexIntRelop.m";
  c2_info[50].fileTimeLo = 1326728322U;
  c2_info[50].fileTimeHi = 0U;
  c2_info[50].mFileTimeLo = 0U;
  c2_info[50].mFileTimeHi = 0U;
  c2_info[51].context =
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexIntRelop.m!float_class_contains_indexIntClass";
  c2_info[51].name = "eml_float_model";
  c2_info[51].dominantType = "char";
  c2_info[51].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m";
  c2_info[51].fileTimeLo = 1326727996U;
  c2_info[51].fileTimeHi = 0U;
  c2_info[51].mFileTimeLo = 0U;
  c2_info[51].mFileTimeHi = 0U;
  c2_info[52].context =
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexIntRelop.m!is_signed_indexIntClass";
  c2_info[52].name = "intmin";
  c2_info[52].dominantType = "char";
  c2_info[52].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m";
  c2_info[52].fileTimeLo = 1311255318U;
  c2_info[52].fileTimeHi = 0U;
  c2_info[52].mFileTimeLo = 0U;
  c2_info[52].mFileTimeHi = 0U;
  c2_info[53].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum";
  c2_info[53].name = "isnan";
  c2_info[53].dominantType = "coder.internal.indexInt";
  c2_info[53].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m";
  c2_info[53].fileTimeLo = 1286818760U;
  c2_info[53].fileTimeHi = 0U;
  c2_info[53].mFileTimeLo = 0U;
  c2_info[53].mFileTimeHi = 0U;
  c2_info[54].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c2_info[54].name = "eml_index_minus";
  c2_info[54].dominantType = "double";
  c2_info[54].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c2_info[54].fileTimeLo = 1286818778U;
  c2_info[54].fileTimeHi = 0U;
  c2_info[54].mFileTimeLo = 0U;
  c2_info[54].mFileTimeHi = 0U;
  c2_info[55].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c2_info[55].name = "eml_index_class";
  c2_info[55].dominantType = "";
  c2_info[55].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[55].fileTimeLo = 1323170578U;
  c2_info[55].fileTimeHi = 0U;
  c2_info[55].mFileTimeLo = 0U;
  c2_info[55].mFileTimeHi = 0U;
  c2_info[56].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c2_info[56].name = "max";
  c2_info[56].dominantType = "coder.internal.indexInt";
  c2_info[56].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/max.m";
  c2_info[56].fileTimeLo = 1311255316U;
  c2_info[56].fileTimeHi = 0U;
  c2_info[56].mFileTimeLo = 0U;
  c2_info[56].mFileTimeHi = 0U;
  c2_info[57].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c2_info[57].name = "eml_int_forloop_overflow_check";
  c2_info[57].dominantType = "";
  c2_info[57].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[57].fileTimeLo = 1332168672U;
  c2_info[57].fileTimeHi = 0U;
  c2_info[57].mFileTimeLo = 0U;
  c2_info[57].mFileTimeHi = 0U;
  c2_info[58].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c2_info[58].name = "eml_index_times";
  c2_info[58].dominantType = "coder.internal.indexInt";
  c2_info[58].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c2_info[58].fileTimeLo = 1286818780U;
  c2_info[58].fileTimeHi = 0U;
  c2_info[58].mFileTimeLo = 0U;
  c2_info[58].mFileTimeHi = 0U;
  c2_info[59].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c2_info[59].name = "eml_index_class";
  c2_info[59].dominantType = "";
  c2_info[59].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[59].fileTimeLo = 1323170578U;
  c2_info[59].fileTimeHi = 0U;
  c2_info[59].mFileTimeLo = 0U;
  c2_info[59].mFileTimeHi = 0U;
  c2_info[60].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c2_info[60].name = "eml_index_plus";
  c2_info[60].dominantType = "coder.internal.indexInt";
  c2_info[60].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[60].fileTimeLo = 1286818778U;
  c2_info[60].fileTimeHi = 0U;
  c2_info[60].mFileTimeLo = 0U;
  c2_info[60].mFileTimeHi = 0U;
  c2_info[61].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c2_info[61].name = "eml_index_minus";
  c2_info[61].dominantType = "coder.internal.indexInt";
  c2_info[61].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c2_info[61].fileTimeLo = 1286818778U;
  c2_info[61].fileTimeHi = 0U;
  c2_info[61].mFileTimeLo = 0U;
  c2_info[61].mFileTimeHi = 0U;
  c2_info[62].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c2_info[62].name = "eml_xnrm2";
  c2_info[62].dominantType = "double";
  c2_info[62].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xnrm2.m";
  c2_info[62].fileTimeLo = 1299076776U;
  c2_info[62].fileTimeHi = 0U;
  c2_info[62].mFileTimeLo = 0U;
  c2_info[62].mFileTimeHi = 0U;
  c2_info[63].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xnrm2.m";
  c2_info[63].name = "eml_blas_inline";
  c2_info[63].dominantType = "";
  c2_info[63].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c2_info[63].fileTimeLo = 1299076768U;
  c2_info[63].fileTimeHi = 0U;
  c2_info[63].mFileTimeLo = 0U;
  c2_info[63].mFileTimeHi = 0U;
}

static void c2_b_info_helper(c2_ResolvedFunctionInfo c2_info[268])
{
  c2_info[64].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xnrm2.m!below_threshold";
  c2_info[64].name = "length";
  c2_info[64].dominantType = "double";
  c2_info[64].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m";
  c2_info[64].fileTimeLo = 1303146206U;
  c2_info[64].fileTimeHi = 0U;
  c2_info[64].mFileTimeLo = 0U;
  c2_info[64].mFileTimeHi = 0U;
  c2_info[65].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m!intlength";
  c2_info[65].name = "eml_index_class";
  c2_info[65].dominantType = "";
  c2_info[65].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[65].fileTimeLo = 1323170578U;
  c2_info[65].fileTimeHi = 0U;
  c2_info[65].mFileTimeLo = 0U;
  c2_info[65].mFileTimeHi = 0U;
  c2_info[66].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xnrm2.m";
  c2_info[66].name = "eml_index_class";
  c2_info[66].dominantType = "";
  c2_info[66].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[66].fileTimeLo = 1323170578U;
  c2_info[66].fileTimeHi = 0U;
  c2_info[66].mFileTimeLo = 0U;
  c2_info[66].mFileTimeHi = 0U;
  c2_info[67].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xnrm2.m";
  c2_info[67].name = "eml_refblas_xnrm2";
  c2_info[67].dominantType = "double";
  c2_info[67].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c2_info[67].fileTimeLo = 1299076784U;
  c2_info[67].fileTimeHi = 0U;
  c2_info[67].mFileTimeLo = 0U;
  c2_info[67].mFileTimeHi = 0U;
  c2_info[68].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c2_info[68].name = "abs";
  c2_info[68].dominantType = "double";
  c2_info[68].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c2_info[68].fileTimeLo = 1286818694U;
  c2_info[68].fileTimeHi = 0U;
  c2_info[68].mFileTimeLo = 0U;
  c2_info[68].mFileTimeHi = 0U;
  c2_info[69].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c2_info[69].name = "eml_scalar_abs";
  c2_info[69].dominantType = "double";
  c2_info[69].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  c2_info[69].fileTimeLo = 1286818712U;
  c2_info[69].fileTimeHi = 0U;
  c2_info[69].mFileTimeLo = 0U;
  c2_info[69].mFileTimeHi = 0U;
  c2_info[70].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c2_info[70].name = "realmin";
  c2_info[70].dominantType = "char";
  c2_info[70].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m";
  c2_info[70].fileTimeLo = 1307651242U;
  c2_info[70].fileTimeHi = 0U;
  c2_info[70].mFileTimeLo = 0U;
  c2_info[70].mFileTimeHi = 0U;
  c2_info[71].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m";
  c2_info[71].name = "eml_realmin";
  c2_info[71].dominantType = "char";
  c2_info[71].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_realmin.m";
  c2_info[71].fileTimeLo = 1307651244U;
  c2_info[71].fileTimeHi = 0U;
  c2_info[71].mFileTimeLo = 0U;
  c2_info[71].mFileTimeHi = 0U;
  c2_info[72].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_realmin.m";
  c2_info[72].name = "eml_float_model";
  c2_info[72].dominantType = "char";
  c2_info[72].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m";
  c2_info[72].fileTimeLo = 1326727996U;
  c2_info[72].fileTimeHi = 0U;
  c2_info[72].mFileTimeLo = 0U;
  c2_info[72].mFileTimeHi = 0U;
  c2_info[73].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c2_info[73].name = "eml_index_class";
  c2_info[73].dominantType = "";
  c2_info[73].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[73].fileTimeLo = 1323170578U;
  c2_info[73].fileTimeHi = 0U;
  c2_info[73].mFileTimeLo = 0U;
  c2_info[73].mFileTimeHi = 0U;
  c2_info[74].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c2_info[74].name = "eml_index_minus";
  c2_info[74].dominantType = "double";
  c2_info[74].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c2_info[74].fileTimeLo = 1286818778U;
  c2_info[74].fileTimeHi = 0U;
  c2_info[74].mFileTimeLo = 0U;
  c2_info[74].mFileTimeHi = 0U;
  c2_info[75].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c2_info[75].name = "eml_index_times";
  c2_info[75].dominantType = "coder.internal.indexInt";
  c2_info[75].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c2_info[75].fileTimeLo = 1286818780U;
  c2_info[75].fileTimeHi = 0U;
  c2_info[75].mFileTimeLo = 0U;
  c2_info[75].mFileTimeHi = 0U;
  c2_info[76].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c2_info[76].name = "eml_index_plus";
  c2_info[76].dominantType = "coder.internal.indexInt";
  c2_info[76].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[76].fileTimeLo = 1286818778U;
  c2_info[76].fileTimeHi = 0U;
  c2_info[76].mFileTimeLo = 0U;
  c2_info[76].mFileTimeHi = 0U;
  c2_info[77].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c2_info[77].name = "eml_int_forloop_overflow_check";
  c2_info[77].dominantType = "";
  c2_info[77].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[77].fileTimeLo = 1332168672U;
  c2_info[77].fileTimeHi = 0U;
  c2_info[77].mFileTimeLo = 0U;
  c2_info[77].mFileTimeHi = 0U;
  c2_info[78].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c2_info[78].name = "eml_div";
  c2_info[78].dominantType = "double";
  c2_info[78].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c2_info[78].fileTimeLo = 1313347810U;
  c2_info[78].fileTimeHi = 0U;
  c2_info[78].mFileTimeLo = 0U;
  c2_info[78].mFileTimeHi = 0U;
  c2_info[79].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c2_info[79].name = "eml_xscal";
  c2_info[79].dominantType = "double";
  c2_info[79].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xscal.m";
  c2_info[79].fileTimeLo = 1299076776U;
  c2_info[79].fileTimeHi = 0U;
  c2_info[79].mFileTimeLo = 0U;
  c2_info[79].mFileTimeHi = 0U;
  c2_info[80].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xscal.m";
  c2_info[80].name = "eml_blas_inline";
  c2_info[80].dominantType = "";
  c2_info[80].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c2_info[80].fileTimeLo = 1299076768U;
  c2_info[80].fileTimeHi = 0U;
  c2_info[80].mFileTimeLo = 0U;
  c2_info[80].mFileTimeHi = 0U;
  c2_info[81].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xscal.m!below_threshold";
  c2_info[81].name = "length";
  c2_info[81].dominantType = "double";
  c2_info[81].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m";
  c2_info[81].fileTimeLo = 1303146206U;
  c2_info[81].fileTimeHi = 0U;
  c2_info[81].mFileTimeLo = 0U;
  c2_info[81].mFileTimeHi = 0U;
  c2_info[82].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xscal.m";
  c2_info[82].name = "eml_index_class";
  c2_info[82].dominantType = "";
  c2_info[82].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[82].fileTimeLo = 1323170578U;
  c2_info[82].fileTimeHi = 0U;
  c2_info[82].mFileTimeLo = 0U;
  c2_info[82].mFileTimeHi = 0U;
  c2_info[83].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xscal.m";
  c2_info[83].name = "eml_scalar_eg";
  c2_info[83].dominantType = "double";
  c2_info[83].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[83].fileTimeLo = 1286818796U;
  c2_info[83].fileTimeHi = 0U;
  c2_info[83].mFileTimeLo = 0U;
  c2_info[83].mFileTimeHi = 0U;
  c2_info[84].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xscal.m";
  c2_info[84].name = "eml_refblas_xscal";
  c2_info[84].dominantType = "double";
  c2_info[84].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xscal.m";
  c2_info[84].fileTimeLo = 1299076784U;
  c2_info[84].fileTimeHi = 0U;
  c2_info[84].mFileTimeLo = 0U;
  c2_info[84].mFileTimeHi = 0U;
  c2_info[85].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xscal.m";
  c2_info[85].name = "eml_index_class";
  c2_info[85].dominantType = "";
  c2_info[85].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[85].fileTimeLo = 1323170578U;
  c2_info[85].fileTimeHi = 0U;
  c2_info[85].mFileTimeLo = 0U;
  c2_info[85].mFileTimeHi = 0U;
  c2_info[86].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xscal.m";
  c2_info[86].name = "eml_index_minus";
  c2_info[86].dominantType = "double";
  c2_info[86].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c2_info[86].fileTimeLo = 1286818778U;
  c2_info[86].fileTimeHi = 0U;
  c2_info[86].mFileTimeLo = 0U;
  c2_info[86].mFileTimeHi = 0U;
  c2_info[87].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xscal.m";
  c2_info[87].name = "eml_index_times";
  c2_info[87].dominantType = "coder.internal.indexInt";
  c2_info[87].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c2_info[87].fileTimeLo = 1286818780U;
  c2_info[87].fileTimeHi = 0U;
  c2_info[87].mFileTimeLo = 0U;
  c2_info[87].mFileTimeHi = 0U;
  c2_info[88].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xscal.m";
  c2_info[88].name = "eml_index_plus";
  c2_info[88].dominantType = "coder.internal.indexInt";
  c2_info[88].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[88].fileTimeLo = 1286818778U;
  c2_info[88].fileTimeHi = 0U;
  c2_info[88].mFileTimeLo = 0U;
  c2_info[88].mFileTimeHi = 0U;
  c2_info[89].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xscal.m";
  c2_info[89].name = "eml_int_forloop_overflow_check";
  c2_info[89].dominantType = "";
  c2_info[89].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[89].fileTimeLo = 1332168672U;
  c2_info[89].fileTimeHi = 0U;
  c2_info[89].mFileTimeLo = 0U;
  c2_info[89].mFileTimeHi = 0U;
  c2_info[90].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c2_info[90].name = "eml_xdotc";
  c2_info[90].dominantType = "double";
  c2_info[90].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdotc.m";
  c2_info[90].fileTimeLo = 1299076772U;
  c2_info[90].fileTimeHi = 0U;
  c2_info[90].mFileTimeLo = 0U;
  c2_info[90].mFileTimeHi = 0U;
  c2_info[91].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdotc.m";
  c2_info[91].name = "eml_blas_inline";
  c2_info[91].dominantType = "";
  c2_info[91].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c2_info[91].fileTimeLo = 1299076768U;
  c2_info[91].fileTimeHi = 0U;
  c2_info[91].mFileTimeLo = 0U;
  c2_info[91].mFileTimeHi = 0U;
  c2_info[92].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdotc.m";
  c2_info[92].name = "eml_xdot";
  c2_info[92].dominantType = "double";
  c2_info[92].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdot.m";
  c2_info[92].fileTimeLo = 1299076772U;
  c2_info[92].fileTimeHi = 0U;
  c2_info[92].mFileTimeLo = 0U;
  c2_info[92].mFileTimeHi = 0U;
  c2_info[93].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdot.m";
  c2_info[93].name = "eml_blas_inline";
  c2_info[93].dominantType = "";
  c2_info[93].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c2_info[93].fileTimeLo = 1299076768U;
  c2_info[93].fileTimeHi = 0U;
  c2_info[93].mFileTimeLo = 0U;
  c2_info[93].mFileTimeHi = 0U;
  c2_info[94].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xdot.m!below_threshold";
  c2_info[94].name = "length";
  c2_info[94].dominantType = "double";
  c2_info[94].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m";
  c2_info[94].fileTimeLo = 1303146206U;
  c2_info[94].fileTimeHi = 0U;
  c2_info[94].mFileTimeLo = 0U;
  c2_info[94].mFileTimeHi = 0U;
  c2_info[95].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xdot.m";
  c2_info[95].name = "eml_index_class";
  c2_info[95].dominantType = "";
  c2_info[95].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[95].fileTimeLo = 1323170578U;
  c2_info[95].fileTimeHi = 0U;
  c2_info[95].mFileTimeLo = 0U;
  c2_info[95].mFileTimeHi = 0U;
  c2_info[96].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xdot.m";
  c2_info[96].name = "eml_refblas_xdot";
  c2_info[96].dominantType = "double";
  c2_info[96].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdot.m";
  c2_info[96].fileTimeLo = 1299076772U;
  c2_info[96].fileTimeHi = 0U;
  c2_info[96].mFileTimeLo = 0U;
  c2_info[96].mFileTimeHi = 0U;
  c2_info[97].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdot.m";
  c2_info[97].name = "eml_refblas_xdotx";
  c2_info[97].dominantType = "char";
  c2_info[97].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m";
  c2_info[97].fileTimeLo = 1299076774U;
  c2_info[97].fileTimeHi = 0U;
  c2_info[97].mFileTimeLo = 0U;
  c2_info[97].mFileTimeHi = 0U;
  c2_info[98].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m";
  c2_info[98].name = "eml_scalar_eg";
  c2_info[98].dominantType = "double";
  c2_info[98].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[98].fileTimeLo = 1286818796U;
  c2_info[98].fileTimeHi = 0U;
  c2_info[98].mFileTimeLo = 0U;
  c2_info[98].mFileTimeHi = 0U;
  c2_info[99].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m";
  c2_info[99].name = "eml_index_class";
  c2_info[99].dominantType = "";
  c2_info[99].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[99].fileTimeLo = 1323170578U;
  c2_info[99].fileTimeHi = 0U;
  c2_info[99].mFileTimeLo = 0U;
  c2_info[99].mFileTimeHi = 0U;
  c2_info[100].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m";
  c2_info[100].name = "eml_int_forloop_overflow_check";
  c2_info[100].dominantType = "";
  c2_info[100].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[100].fileTimeLo = 1332168672U;
  c2_info[100].fileTimeHi = 0U;
  c2_info[100].mFileTimeLo = 0U;
  c2_info[100].mFileTimeHi = 0U;
  c2_info[101].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m";
  c2_info[101].name = "eml_index_plus";
  c2_info[101].dominantType = "coder.internal.indexInt";
  c2_info[101].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[101].fileTimeLo = 1286818778U;
  c2_info[101].fileTimeHi = 0U;
  c2_info[101].mFileTimeLo = 0U;
  c2_info[101].mFileTimeHi = 0U;
  c2_info[102].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c2_info[102].name = "eml_xaxpy";
  c2_info[102].dominantType = "double";
  c2_info[102].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xaxpy.m";
  c2_info[102].fileTimeLo = 1299076770U;
  c2_info[102].fileTimeHi = 0U;
  c2_info[102].mFileTimeLo = 0U;
  c2_info[102].mFileTimeHi = 0U;
  c2_info[103].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xaxpy.m";
  c2_info[103].name = "eml_blas_inline";
  c2_info[103].dominantType = "";
  c2_info[103].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c2_info[103].fileTimeLo = 1299076768U;
  c2_info[103].fileTimeHi = 0U;
  c2_info[103].mFileTimeLo = 0U;
  c2_info[103].mFileTimeHi = 0U;
  c2_info[104].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xaxpy.m!below_threshold";
  c2_info[104].name = "length";
  c2_info[104].dominantType = "double";
  c2_info[104].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m";
  c2_info[104].fileTimeLo = 1303146206U;
  c2_info[104].fileTimeHi = 0U;
  c2_info[104].mFileTimeLo = 0U;
  c2_info[104].mFileTimeHi = 0U;
  c2_info[105].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xaxpy.m";
  c2_info[105].name = "eml_index_class";
  c2_info[105].dominantType = "";
  c2_info[105].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[105].fileTimeLo = 1323170578U;
  c2_info[105].fileTimeHi = 0U;
  c2_info[105].mFileTimeLo = 0U;
  c2_info[105].mFileTimeHi = 0U;
  c2_info[106].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xaxpy.m";
  c2_info[106].name = "eml_scalar_eg";
  c2_info[106].dominantType = "double";
  c2_info[106].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[106].fileTimeLo = 1286818796U;
  c2_info[106].fileTimeHi = 0U;
  c2_info[106].mFileTimeLo = 0U;
  c2_info[106].mFileTimeHi = 0U;
  c2_info[107].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xaxpy.m";
  c2_info[107].name = "eml_refblas_xaxpy";
  c2_info[107].dominantType = "double";
  c2_info[107].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xaxpy.m";
  c2_info[107].fileTimeLo = 1299076772U;
  c2_info[107].fileTimeHi = 0U;
  c2_info[107].mFileTimeLo = 0U;
  c2_info[107].mFileTimeHi = 0U;
  c2_info[108].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xaxpy.m";
  c2_info[108].name = "eml_index_class";
  c2_info[108].dominantType = "";
  c2_info[108].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[108].fileTimeLo = 1323170578U;
  c2_info[108].fileTimeHi = 0U;
  c2_info[108].mFileTimeLo = 0U;
  c2_info[108].mFileTimeHi = 0U;
  c2_info[109].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xaxpy.m";
  c2_info[109].name = "eml_isa_uint";
  c2_info[109].dominantType = "coder.internal.indexInt";
  c2_info[109].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isa_uint.m";
  c2_info[109].fileTimeLo = 1286818784U;
  c2_info[109].fileTimeHi = 0U;
  c2_info[109].mFileTimeLo = 0U;
  c2_info[109].mFileTimeHi = 0U;
  c2_info[110].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xaxpy.m";
  c2_info[110].name = "eml_index_minus";
  c2_info[110].dominantType = "double";
  c2_info[110].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c2_info[110].fileTimeLo = 1286818778U;
  c2_info[110].fileTimeHi = 0U;
  c2_info[110].mFileTimeLo = 0U;
  c2_info[110].mFileTimeHi = 0U;
  c2_info[111].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xaxpy.m";
  c2_info[111].name = "eml_int_forloop_overflow_check";
  c2_info[111].dominantType = "";
  c2_info[111].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[111].fileTimeLo = 1332168672U;
  c2_info[111].fileTimeHi = 0U;
  c2_info[111].mFileTimeLo = 0U;
  c2_info[111].mFileTimeHi = 0U;
  c2_info[112].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xaxpy.m";
  c2_info[112].name = "eml_index_plus";
  c2_info[112].dominantType = "double";
  c2_info[112].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[112].fileTimeLo = 1286818778U;
  c2_info[112].fileTimeHi = 0U;
  c2_info[112].mFileTimeLo = 0U;
  c2_info[112].mFileTimeHi = 0U;
  c2_info[113].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xaxpy.m";
  c2_info[113].name = "eml_index_plus";
  c2_info[113].dominantType = "coder.internal.indexInt";
  c2_info[113].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[113].fileTimeLo = 1286818778U;
  c2_info[113].fileTimeHi = 0U;
  c2_info[113].mFileTimeLo = 0U;
  c2_info[113].mFileTimeHi = 0U;
  c2_info[114].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper";
  c2_info[114].name = "intmin";
  c2_info[114].dominantType = "char";
  c2_info[114].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m";
  c2_info[114].fileTimeLo = 1311255318U;
  c2_info[114].fileTimeHi = 0U;
  c2_info[114].mFileTimeLo = 0U;
  c2_info[114].mFileTimeHi = 0U;
  c2_info[115].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c2_info[115].name = "abs";
  c2_info[115].dominantType = "double";
  c2_info[115].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c2_info[115].fileTimeLo = 1286818694U;
  c2_info[115].fileTimeHi = 0U;
  c2_info[115].mFileTimeLo = 0U;
  c2_info[115].mFileTimeHi = 0U;
  c2_info[116].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c2_info[116].name = "mtimes";
  c2_info[116].dominantType = "double";
  c2_info[116].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[116].fileTimeLo = 1289519692U;
  c2_info[116].fileTimeHi = 0U;
  c2_info[116].mFileTimeLo = 0U;
  c2_info[116].mFileTimeHi = 0U;
  c2_info[117].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c2_info[117].name = "realmin";
  c2_info[117].dominantType = "char";
  c2_info[117].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m";
  c2_info[117].fileTimeLo = 1307651242U;
  c2_info[117].fileTimeHi = 0U;
  c2_info[117].mFileTimeLo = 0U;
  c2_info[117].mFileTimeHi = 0U;
  c2_info[118].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c2_info[118].name = "eps";
  c2_info[118].dominantType = "char";
  c2_info[118].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c2_info[118].fileTimeLo = 1326727996U;
  c2_info[118].fileTimeHi = 0U;
  c2_info[118].mFileTimeLo = 0U;
  c2_info[118].mFileTimeHi = 0U;
  c2_info[119].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c2_info[119].name = "eml_is_float_class";
  c2_info[119].dominantType = "char";
  c2_info[119].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_is_float_class.m";
  c2_info[119].fileTimeLo = 1286818782U;
  c2_info[119].fileTimeHi = 0U;
  c2_info[119].mFileTimeLo = 0U;
  c2_info[119].mFileTimeHi = 0U;
  c2_info[120].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c2_info[120].name = "eml_eps";
  c2_info[120].dominantType = "char";
  c2_info[120].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_eps.m";
  c2_info[120].fileTimeLo = 1326727996U;
  c2_info[120].fileTimeHi = 0U;
  c2_info[120].mFileTimeLo = 0U;
  c2_info[120].mFileTimeHi = 0U;
  c2_info[121].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_eps.m";
  c2_info[121].name = "eml_float_model";
  c2_info[121].dominantType = "char";
  c2_info[121].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m";
  c2_info[121].fileTimeLo = 1326727996U;
  c2_info[121].fileTimeHi = 0U;
  c2_info[121].mFileTimeLo = 0U;
  c2_info[121].mFileTimeHi = 0U;
  c2_info[122].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c2_info[122].name = "eml_error";
  c2_info[122].dominantType = "char";
  c2_info[122].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m";
  c2_info[122].fileTimeLo = 1305318000U;
  c2_info[122].fileTimeHi = 0U;
  c2_info[122].mFileTimeLo = 0U;
  c2_info[122].mFileTimeHi = 0U;
  c2_info[123].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum";
  c2_info[123].name = "eml_const_nonsingleton_dim";
  c2_info[123].dominantType = "double";
  c2_info[123].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_const_nonsingleton_dim.m";
  c2_info[123].fileTimeLo = 1286818696U;
  c2_info[123].fileTimeHi = 0U;
  c2_info[123].mFileTimeLo = 0U;
  c2_info[123].mFileTimeHi = 0U;
  c2_info[124].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum";
  c2_info[124].name = "eml_scalar_eg";
  c2_info[124].dominantType = "double";
  c2_info[124].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[124].fileTimeLo = 1286818796U;
  c2_info[124].fileTimeHi = 0U;
  c2_info[124].mFileTimeLo = 0U;
  c2_info[124].mFileTimeHi = 0U;
  c2_info[125].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum";
  c2_info[125].name = "eml_index_class";
  c2_info[125].dominantType = "";
  c2_info[125].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[125].fileTimeLo = 1323170578U;
  c2_info[125].fileTimeHi = 0U;
  c2_info[125].mFileTimeLo = 0U;
  c2_info[125].mFileTimeHi = 0U;
  c2_info[126].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum_sub";
  c2_info[126].name = "eml_index_class";
  c2_info[126].dominantType = "";
  c2_info[126].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[126].fileTimeLo = 1323170578U;
  c2_info[126].fileTimeHi = 0U;
  c2_info[126].mFileTimeLo = 0U;
  c2_info[126].mFileTimeHi = 0U;
  c2_info[127].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum_sub";
  c2_info[127].name = "isnan";
  c2_info[127].dominantType = "double";
  c2_info[127].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m";
  c2_info[127].fileTimeLo = 1286818760U;
  c2_info[127].fileTimeHi = 0U;
  c2_info[127].mFileTimeLo = 0U;
  c2_info[127].mFileTimeHi = 0U;
}

static void c2_c_info_helper(c2_ResolvedFunctionInfo c2_info[268])
{
  c2_info[128].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum_sub";
  c2_info[128].name = "eml_index_plus";
  c2_info[128].dominantType = "coder.internal.indexInt";
  c2_info[128].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[128].fileTimeLo = 1286818778U;
  c2_info[128].fileTimeHi = 0U;
  c2_info[128].mFileTimeLo = 0U;
  c2_info[128].mFileTimeHi = 0U;
  c2_info[129].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum_sub";
  c2_info[129].name = "eml_int_forloop_overflow_check";
  c2_info[129].dominantType = "";
  c2_info[129].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[129].fileTimeLo = 1332168672U;
  c2_info[129].fileTimeHi = 0U;
  c2_info[129].mFileTimeLo = 0U;
  c2_info[129].mFileTimeHi = 0U;
  c2_info[130].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum_sub";
  c2_info[130].name = "eml_relop";
  c2_info[130].dominantType = "function_handle";
  c2_info[130].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_relop.m";
  c2_info[130].fileTimeLo = 1326727998U;
  c2_info[130].fileTimeHi = 0U;
  c2_info[130].mFileTimeLo = 0U;
  c2_info[130].mFileTimeHi = 0U;
  c2_info[131].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c2_info[131].name = "sqrt";
  c2_info[131].dominantType = "double";
  c2_info[131].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m";
  c2_info[131].fileTimeLo = 1286818752U;
  c2_info[131].fileTimeHi = 0U;
  c2_info[131].mFileTimeLo = 0U;
  c2_info[131].mFileTimeHi = 0U;
  c2_info[132].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m";
  c2_info[132].name = "eml_error";
  c2_info[132].dominantType = "char";
  c2_info[132].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m";
  c2_info[132].fileTimeLo = 1305318000U;
  c2_info[132].fileTimeHi = 0U;
  c2_info[132].mFileTimeLo = 0U;
  c2_info[132].mFileTimeHi = 0U;
  c2_info[133].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m";
  c2_info[133].name = "eml_scalar_sqrt";
  c2_info[133].dominantType = "double";
  c2_info[133].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sqrt.m";
  c2_info[133].fileTimeLo = 1286818738U;
  c2_info[133].fileTimeHi = 0U;
  c2_info[133].mFileTimeLo = 0U;
  c2_info[133].mFileTimeHi = 0U;
  c2_info[134].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c2_info[134].name = "eml_xrotg";
  c2_info[134].dominantType = "double";
  c2_info[134].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xrotg.m";
  c2_info[134].fileTimeLo = 1299076776U;
  c2_info[134].fileTimeHi = 0U;
  c2_info[134].mFileTimeLo = 0U;
  c2_info[134].mFileTimeHi = 0U;
  c2_info[135].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xrotg.m";
  c2_info[135].name = "eml_blas_inline";
  c2_info[135].dominantType = "";
  c2_info[135].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c2_info[135].fileTimeLo = 1299076768U;
  c2_info[135].fileTimeHi = 0U;
  c2_info[135].mFileTimeLo = 0U;
  c2_info[135].mFileTimeHi = 0U;
  c2_info[136].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xrotg.m";
  c2_info[136].name = "eml_refblas_xrotg";
  c2_info[136].dominantType = "double";
  c2_info[136].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xrotg.m";
  c2_info[136].fileTimeLo = 1299076784U;
  c2_info[136].fileTimeHi = 0U;
  c2_info[136].mFileTimeLo = 0U;
  c2_info[136].mFileTimeHi = 0U;
  c2_info[137].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xrotg.m";
  c2_info[137].name = "abs";
  c2_info[137].dominantType = "double";
  c2_info[137].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c2_info[137].fileTimeLo = 1286818694U;
  c2_info[137].fileTimeHi = 0U;
  c2_info[137].mFileTimeLo = 0U;
  c2_info[137].mFileTimeHi = 0U;
  c2_info[138].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xrotg.m";
  c2_info[138].name = "mrdivide";
  c2_info[138].dominantType = "double";
  c2_info[138].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c2_info[138].fileTimeLo = 1342810944U;
  c2_info[138].fileTimeHi = 0U;
  c2_info[138].mFileTimeLo = 1319729966U;
  c2_info[138].mFileTimeHi = 0U;
  c2_info[139].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c2_info[139].name = "rdivide";
  c2_info[139].dominantType = "double";
  c2_info[139].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c2_info[139].fileTimeLo = 1286818844U;
  c2_info[139].fileTimeHi = 0U;
  c2_info[139].mFileTimeLo = 0U;
  c2_info[139].mFileTimeHi = 0U;
  c2_info[140].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c2_info[140].name = "eml_div";
  c2_info[140].dominantType = "double";
  c2_info[140].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c2_info[140].fileTimeLo = 1313347810U;
  c2_info[140].fileTimeHi = 0U;
  c2_info[140].mFileTimeLo = 0U;
  c2_info[140].mFileTimeHi = 0U;
  c2_info[141].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xrotg.m";
  c2_info[141].name = "sqrt";
  c2_info[141].dominantType = "double";
  c2_info[141].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m";
  c2_info[141].fileTimeLo = 1286818752U;
  c2_info[141].fileTimeHi = 0U;
  c2_info[141].mFileTimeLo = 0U;
  c2_info[141].mFileTimeHi = 0U;
  c2_info[142].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xrotg.m!eml_ceval_xrotg";
  c2_info[142].name = "eml_scalar_eg";
  c2_info[142].dominantType = "double";
  c2_info[142].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[142].fileTimeLo = 1286818796U;
  c2_info[142].fileTimeHi = 0U;
  c2_info[142].mFileTimeLo = 0U;
  c2_info[142].mFileTimeHi = 0U;
  c2_info[143].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c2_info[143].name = "eml_xrot";
  c2_info[143].dominantType = "double";
  c2_info[143].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xrot.m";
  c2_info[143].fileTimeLo = 1299076776U;
  c2_info[143].fileTimeHi = 0U;
  c2_info[143].mFileTimeLo = 0U;
  c2_info[143].mFileTimeHi = 0U;
  c2_info[144].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xrot.m";
  c2_info[144].name = "eml_blas_inline";
  c2_info[144].dominantType = "";
  c2_info[144].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c2_info[144].fileTimeLo = 1299076768U;
  c2_info[144].fileTimeHi = 0U;
  c2_info[144].mFileTimeLo = 0U;
  c2_info[144].mFileTimeHi = 0U;
  c2_info[145].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xrot.m";
  c2_info[145].name = "eml_index_class";
  c2_info[145].dominantType = "";
  c2_info[145].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[145].fileTimeLo = 1323170578U;
  c2_info[145].fileTimeHi = 0U;
  c2_info[145].mFileTimeLo = 0U;
  c2_info[145].mFileTimeHi = 0U;
  c2_info[146].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xrot.m";
  c2_info[146].name = "eml_scalar_eg";
  c2_info[146].dominantType = "double";
  c2_info[146].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[146].fileTimeLo = 1286818796U;
  c2_info[146].fileTimeHi = 0U;
  c2_info[146].mFileTimeLo = 0U;
  c2_info[146].mFileTimeHi = 0U;
  c2_info[147].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xrot.m";
  c2_info[147].name = "eml_refblas_xrot";
  c2_info[147].dominantType = "double";
  c2_info[147].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xrot.m";
  c2_info[147].fileTimeLo = 1299076784U;
  c2_info[147].fileTimeHi = 0U;
  c2_info[147].mFileTimeLo = 0U;
  c2_info[147].mFileTimeHi = 0U;
  c2_info[148].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xrot.m";
  c2_info[148].name = "eml_index_class";
  c2_info[148].dominantType = "";
  c2_info[148].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[148].fileTimeLo = 1323170578U;
  c2_info[148].fileTimeHi = 0U;
  c2_info[148].mFileTimeLo = 0U;
  c2_info[148].mFileTimeHi = 0U;
  c2_info[149].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xrot.m";
  c2_info[149].name = "eml_int_forloop_overflow_check";
  c2_info[149].dominantType = "";
  c2_info[149].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[149].fileTimeLo = 1332168672U;
  c2_info[149].fileTimeHi = 0U;
  c2_info[149].mFileTimeLo = 0U;
  c2_info[149].mFileTimeHi = 0U;
  c2_info[150].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xrot.m";
  c2_info[150].name = "mtimes";
  c2_info[150].dominantType = "double";
  c2_info[150].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[150].fileTimeLo = 1289519692U;
  c2_info[150].fileTimeHi = 0U;
  c2_info[150].mFileTimeLo = 0U;
  c2_info[150].mFileTimeHi = 0U;
  c2_info[151].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xrot.m";
  c2_info[151].name = "eml_index_plus";
  c2_info[151].dominantType = "coder.internal.indexInt";
  c2_info[151].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[151].fileTimeLo = 1286818778U;
  c2_info[151].fileTimeHi = 0U;
  c2_info[151].mFileTimeLo = 0U;
  c2_info[151].mFileTimeHi = 0U;
  c2_info[152].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c2_info[152].name = "eml_xswap";
  c2_info[152].dominantType = "double";
  c2_info[152].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xswap.m";
  c2_info[152].fileTimeLo = 1299076778U;
  c2_info[152].fileTimeHi = 0U;
  c2_info[152].mFileTimeLo = 0U;
  c2_info[152].mFileTimeHi = 0U;
  c2_info[153].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xswap.m";
  c2_info[153].name = "eml_blas_inline";
  c2_info[153].dominantType = "";
  c2_info[153].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c2_info[153].fileTimeLo = 1299076768U;
  c2_info[153].fileTimeHi = 0U;
  c2_info[153].mFileTimeLo = 0U;
  c2_info[153].mFileTimeHi = 0U;
  c2_info[154].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xswap.m";
  c2_info[154].name = "eml_index_class";
  c2_info[154].dominantType = "";
  c2_info[154].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[154].fileTimeLo = 1323170578U;
  c2_info[154].fileTimeHi = 0U;
  c2_info[154].mFileTimeLo = 0U;
  c2_info[154].mFileTimeHi = 0U;
  c2_info[155].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xswap.m";
  c2_info[155].name = "eml_refblas_xswap";
  c2_info[155].dominantType = "double";
  c2_info[155].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c2_info[155].fileTimeLo = 1299076786U;
  c2_info[155].fileTimeHi = 0U;
  c2_info[155].mFileTimeLo = 0U;
  c2_info[155].mFileTimeHi = 0U;
  c2_info[156].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c2_info[156].name = "eml_index_class";
  c2_info[156].dominantType = "";
  c2_info[156].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[156].fileTimeLo = 1323170578U;
  c2_info[156].fileTimeHi = 0U;
  c2_info[156].mFileTimeLo = 0U;
  c2_info[156].mFileTimeHi = 0U;
  c2_info[157].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c2_info[157].name = "abs";
  c2_info[157].dominantType = "coder.internal.indexInt";
  c2_info[157].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c2_info[157].fileTimeLo = 1286818694U;
  c2_info[157].fileTimeHi = 0U;
  c2_info[157].mFileTimeLo = 0U;
  c2_info[157].mFileTimeHi = 0U;
  c2_info[158].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c2_info[158].name = "eml_scalar_abs";
  c2_info[158].dominantType = "coder.internal.indexInt";
  c2_info[158].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  c2_info[158].fileTimeLo = 1286818712U;
  c2_info[158].fileTimeHi = 0U;
  c2_info[158].mFileTimeLo = 0U;
  c2_info[158].mFileTimeHi = 0U;
  c2_info[159].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c2_info[159].name = "eml_int_forloop_overflow_check";
  c2_info[159].dominantType = "";
  c2_info[159].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[159].fileTimeLo = 1332168672U;
  c2_info[159].fileTimeHi = 0U;
  c2_info[159].mFileTimeLo = 0U;
  c2_info[159].mFileTimeHi = 0U;
  c2_info[160].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c2_info[160].name = "eml_index_plus";
  c2_info[160].dominantType = "coder.internal.indexInt";
  c2_info[160].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[160].fileTimeLo = 1286818778U;
  c2_info[160].fileTimeHi = 0U;
  c2_info[160].mFileTimeLo = 0U;
  c2_info[160].mFileTimeHi = 0U;
  c2_info[161].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv";
  c2_info[161].name = "eml_int_forloop_overflow_check";
  c2_info[161].dominantType = "";
  c2_info[161].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[161].fileTimeLo = 1332168672U;
  c2_info[161].fileTimeHi = 0U;
  c2_info[161].mFileTimeLo = 0U;
  c2_info[161].mFileTimeHi = 0U;
  c2_info[162].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv";
  c2_info[162].name = "eml_index_plus";
  c2_info[162].dominantType = "double";
  c2_info[162].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[162].fileTimeLo = 1286818778U;
  c2_info[162].fileTimeHi = 0U;
  c2_info[162].mFileTimeLo = 0U;
  c2_info[162].mFileTimeHi = 0U;
  c2_info[163].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv";
  c2_info[163].name = "eml_div";
  c2_info[163].dominantType = "double";
  c2_info[163].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c2_info[163].fileTimeLo = 1313347810U;
  c2_info[163].fileTimeHi = 0U;
  c2_info[163].mFileTimeLo = 0U;
  c2_info[163].mFileTimeHi = 0U;
  c2_info[164].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv";
  c2_info[164].name = "eml_xscal";
  c2_info[164].dominantType = "double";
  c2_info[164].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xscal.m";
  c2_info[164].fileTimeLo = 1299076776U;
  c2_info[164].fileTimeHi = 0U;
  c2_info[164].mFileTimeLo = 0U;
  c2_info[164].mFileTimeHi = 0U;
  c2_info[165].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv";
  c2_info[165].name = "eml_index_plus";
  c2_info[165].dominantType = "coder.internal.indexInt";
  c2_info[165].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[165].fileTimeLo = 1286818778U;
  c2_info[165].fileTimeHi = 0U;
  c2_info[165].mFileTimeLo = 0U;
  c2_info[165].mFileTimeHi = 0U;
  c2_info[166].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv";
  c2_info[166].name = "eml_xgemm";
  c2_info[166].dominantType = "char";
  c2_info[166].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m";
  c2_info[166].fileTimeLo = 1299076772U;
  c2_info[166].fileTimeHi = 0U;
  c2_info[166].mFileTimeLo = 0U;
  c2_info[166].mFileTimeHi = 0U;
  c2_info[167].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m!below_threshold";
  c2_info[167].name = "min";
  c2_info[167].dominantType = "double";
  c2_info[167].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c2_info[167].fileTimeLo = 1311255318U;
  c2_info[167].fileTimeHi = 0U;
  c2_info[167].mFileTimeLo = 0U;
  c2_info[167].mFileTimeHi = 0U;
  c2_info[168].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemm.m";
  c2_info[168].name = "eml_index_minus";
  c2_info[168].dominantType = "double";
  c2_info[168].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c2_info[168].fileTimeLo = 1286818778U;
  c2_info[168].fileTimeHi = 0U;
  c2_info[168].mFileTimeLo = 0U;
  c2_info[168].mFileTimeHi = 0U;
  c2_info[169].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemm.m";
  c2_info[169].name = "eml_index_class";
  c2_info[169].dominantType = "";
  c2_info[169].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[169].fileTimeLo = 1323170578U;
  c2_info[169].fileTimeHi = 0U;
  c2_info[169].mFileTimeLo = 0U;
  c2_info[169].mFileTimeHi = 0U;
  c2_info[170].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemm.m";
  c2_info[170].name = "eml_scalar_eg";
  c2_info[170].dominantType = "double";
  c2_info[170].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[170].fileTimeLo = 1286818796U;
  c2_info[170].fileTimeHi = 0U;
  c2_info[170].mFileTimeLo = 0U;
  c2_info[170].mFileTimeHi = 0U;
  c2_info[171].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemm.m";
  c2_info[171].name = "eml_index_times";
  c2_info[171].dominantType = "coder.internal.indexInt";
  c2_info[171].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c2_info[171].fileTimeLo = 1286818780U;
  c2_info[171].fileTimeHi = 0U;
  c2_info[171].mFileTimeLo = 0U;
  c2_info[171].mFileTimeHi = 0U;
  c2_info[172].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemm.m";
  c2_info[172].name = "eml_index_plus";
  c2_info[172].dominantType = "coder.internal.indexInt";
  c2_info[172].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[172].fileTimeLo = 1286818778U;
  c2_info[172].fileTimeHi = 0U;
  c2_info[172].mFileTimeLo = 0U;
  c2_info[172].mFileTimeHi = 0U;
  c2_info[173].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemm.m";
  c2_info[173].name = "eml_int_forloop_overflow_check";
  c2_info[173].dominantType = "";
  c2_info[173].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[173].fileTimeLo = 1332168672U;
  c2_info[173].fileTimeHi = 0U;
  c2_info[173].mFileTimeLo = 0U;
  c2_info[173].mFileTimeHi = 0U;
  c2_info[174].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemm.m";
  c2_info[174].name = "eml_index_plus";
  c2_info[174].dominantType = "double";
  c2_info[174].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[174].fileTimeLo = 1286818778U;
  c2_info[174].fileTimeHi = 0U;
  c2_info[174].mFileTimeLo = 0U;
  c2_info[174].mFileTimeHi = 0U;
  c2_info[175].context = "";
  c2_info[175].name = "inv";
  c2_info[175].dominantType = "double";
  c2_info[175].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m";
  c2_info[175].fileTimeLo = 1305318000U;
  c2_info[175].fileTimeHi = 0U;
  c2_info[175].mFileTimeLo = 0U;
  c2_info[175].mFileTimeHi = 0U;
  c2_info[176].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!invNxN";
  c2_info[176].name = "eml_index_class";
  c2_info[176].dominantType = "";
  c2_info[176].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[176].fileTimeLo = 1323170578U;
  c2_info[176].fileTimeHi = 0U;
  c2_info[176].mFileTimeLo = 0U;
  c2_info[176].mFileTimeHi = 0U;
  c2_info[177].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!invNxN";
  c2_info[177].name = "eml_xgetrf";
  c2_info[177].dominantType = "double";
  c2_info[177].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/eml_xgetrf.m";
  c2_info[177].fileTimeLo = 1286818806U;
  c2_info[177].fileTimeHi = 0U;
  c2_info[177].mFileTimeLo = 0U;
  c2_info[177].mFileTimeHi = 0U;
  c2_info[178].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/eml_xgetrf.m";
  c2_info[178].name = "eml_lapack_xgetrf";
  c2_info[178].dominantType = "double";
  c2_info[178].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/internal/eml_lapack_xgetrf.m";
  c2_info[178].fileTimeLo = 1286818810U;
  c2_info[178].fileTimeHi = 0U;
  c2_info[178].mFileTimeLo = 0U;
  c2_info[178].mFileTimeHi = 0U;
  c2_info[179].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/internal/eml_lapack_xgetrf.m";
  c2_info[179].name = "eml_matlab_zgetrf";
  c2_info[179].dominantType = "double";
  c2_info[179].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[179].fileTimeLo = 1302688994U;
  c2_info[179].fileTimeHi = 0U;
  c2_info[179].mFileTimeLo = 0U;
  c2_info[179].mFileTimeHi = 0U;
  c2_info[180].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[180].name = "realmin";
  c2_info[180].dominantType = "char";
  c2_info[180].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m";
  c2_info[180].fileTimeLo = 1307651242U;
  c2_info[180].fileTimeHi = 0U;
  c2_info[180].mFileTimeLo = 0U;
  c2_info[180].mFileTimeHi = 0U;
  c2_info[181].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[181].name = "eps";
  c2_info[181].dominantType = "char";
  c2_info[181].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c2_info[181].fileTimeLo = 1326727996U;
  c2_info[181].fileTimeHi = 0U;
  c2_info[181].mFileTimeLo = 0U;
  c2_info[181].mFileTimeHi = 0U;
  c2_info[182].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[182].name = "min";
  c2_info[182].dominantType = "coder.internal.indexInt";
  c2_info[182].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c2_info[182].fileTimeLo = 1311255318U;
  c2_info[182].fileTimeHi = 0U;
  c2_info[182].mFileTimeLo = 0U;
  c2_info[182].mFileTimeHi = 0U;
  c2_info[183].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[183].name = "colon";
  c2_info[183].dominantType = "double";
  c2_info[183].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m";
  c2_info[183].fileTimeLo = 1311255318U;
  c2_info[183].fileTimeHi = 0U;
  c2_info[183].mFileTimeLo = 0U;
  c2_info[183].mFileTimeHi = 0U;
  c2_info[184].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m";
  c2_info[184].name = "floor";
  c2_info[184].dominantType = "double";
  c2_info[184].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  c2_info[184].fileTimeLo = 1286818742U;
  c2_info[184].fileTimeHi = 0U;
  c2_info[184].mFileTimeLo = 0U;
  c2_info[184].mFileTimeHi = 0U;
  c2_info[185].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  c2_info[185].name = "eml_scalar_floor";
  c2_info[185].dominantType = "double";
  c2_info[185].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m";
  c2_info[185].fileTimeLo = 1286818726U;
  c2_info[185].fileTimeHi = 0U;
  c2_info[185].mFileTimeLo = 0U;
  c2_info[185].mFileTimeHi = 0U;
  c2_info[186].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!checkrange";
  c2_info[186].name = "intmin";
  c2_info[186].dominantType = "char";
  c2_info[186].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m";
  c2_info[186].fileTimeLo = 1311255318U;
  c2_info[186].fileTimeHi = 0U;
  c2_info[186].mFileTimeLo = 0U;
  c2_info[186].mFileTimeHi = 0U;
  c2_info[187].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!checkrange";
  c2_info[187].name = "intmax";
  c2_info[187].dominantType = "char";
  c2_info[187].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c2_info[187].fileTimeLo = 1311255316U;
  c2_info[187].fileTimeHi = 0U;
  c2_info[187].mFileTimeLo = 0U;
  c2_info[187].mFileTimeHi = 0U;
  c2_info[188].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_integer_colon_dispatcher";
  c2_info[188].name = "intmin";
  c2_info[188].dominantType = "char";
  c2_info[188].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m";
  c2_info[188].fileTimeLo = 1311255318U;
  c2_info[188].fileTimeHi = 0U;
  c2_info[188].mFileTimeLo = 0U;
  c2_info[188].mFileTimeHi = 0U;
  c2_info[189].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_integer_colon_dispatcher";
  c2_info[189].name = "intmax";
  c2_info[189].dominantType = "char";
  c2_info[189].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c2_info[189].fileTimeLo = 1311255316U;
  c2_info[189].fileTimeHi = 0U;
  c2_info[189].mFileTimeLo = 0U;
  c2_info[189].mFileTimeHi = 0U;
  c2_info[190].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_integer_colon_dispatcher";
  c2_info[190].name = "eml_isa_uint";
  c2_info[190].dominantType = "coder.internal.indexInt";
  c2_info[190].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isa_uint.m";
  c2_info[190].fileTimeLo = 1286818784U;
  c2_info[190].fileTimeHi = 0U;
  c2_info[190].mFileTimeLo = 0U;
  c2_info[190].mFileTimeHi = 0U;
  c2_info[191].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c2_info[191].name = "eml_unsigned_class";
  c2_info[191].dominantType = "char";
  c2_info[191].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_unsigned_class.m";
  c2_info[191].fileTimeLo = 1323170580U;
  c2_info[191].fileTimeHi = 0U;
  c2_info[191].mFileTimeLo = 0U;
  c2_info[191].mFileTimeHi = 0U;
}

static void c2_d_info_helper(c2_ResolvedFunctionInfo c2_info[268])
{
  c2_info[192].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_unsigned_class.m";
  c2_info[192].name = "eml_index_class";
  c2_info[192].dominantType = "";
  c2_info[192].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[192].fileTimeLo = 1323170578U;
  c2_info[192].fileTimeHi = 0U;
  c2_info[192].mFileTimeLo = 0U;
  c2_info[192].mFileTimeHi = 0U;
  c2_info[193].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c2_info[193].name = "eml_index_class";
  c2_info[193].dominantType = "";
  c2_info[193].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[193].fileTimeLo = 1323170578U;
  c2_info[193].fileTimeHi = 0U;
  c2_info[193].mFileTimeLo = 0U;
  c2_info[193].mFileTimeHi = 0U;
  c2_info[194].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c2_info[194].name = "intmax";
  c2_info[194].dominantType = "char";
  c2_info[194].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c2_info[194].fileTimeLo = 1311255316U;
  c2_info[194].fileTimeHi = 0U;
  c2_info[194].mFileTimeLo = 0U;
  c2_info[194].mFileTimeHi = 0U;
  c2_info[195].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c2_info[195].name = "eml_isa_uint";
  c2_info[195].dominantType = "coder.internal.indexInt";
  c2_info[195].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isa_uint.m";
  c2_info[195].fileTimeLo = 1286818784U;
  c2_info[195].fileTimeHi = 0U;
  c2_info[195].mFileTimeLo = 0U;
  c2_info[195].mFileTimeHi = 0U;
  c2_info[196].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c2_info[196].name = "eml_index_plus";
  c2_info[196].dominantType = "double";
  c2_info[196].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[196].fileTimeLo = 1286818778U;
  c2_info[196].fileTimeHi = 0U;
  c2_info[196].mFileTimeLo = 0U;
  c2_info[196].mFileTimeHi = 0U;
  c2_info[197].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_signed_integer_colon";
  c2_info[197].name = "eml_int_forloop_overflow_check";
  c2_info[197].dominantType = "";
  c2_info[197].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[197].fileTimeLo = 1332168672U;
  c2_info[197].fileTimeHi = 0U;
  c2_info[197].mFileTimeLo = 0U;
  c2_info[197].mFileTimeHi = 0U;
  c2_info[198].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[198].name = "eml_index_class";
  c2_info[198].dominantType = "";
  c2_info[198].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[198].fileTimeLo = 1323170578U;
  c2_info[198].fileTimeHi = 0U;
  c2_info[198].mFileTimeLo = 0U;
  c2_info[198].mFileTimeHi = 0U;
  c2_info[199].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[199].name = "eml_index_plus";
  c2_info[199].dominantType = "double";
  c2_info[199].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[199].fileTimeLo = 1286818778U;
  c2_info[199].fileTimeHi = 0U;
  c2_info[199].mFileTimeLo = 0U;
  c2_info[199].mFileTimeHi = 0U;
  c2_info[200].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[200].name = "eml_int_forloop_overflow_check";
  c2_info[200].dominantType = "";
  c2_info[200].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[200].fileTimeLo = 1332168672U;
  c2_info[200].fileTimeHi = 0U;
  c2_info[200].mFileTimeLo = 0U;
  c2_info[200].mFileTimeHi = 0U;
  c2_info[201].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[201].name = "eml_index_minus";
  c2_info[201].dominantType = "double";
  c2_info[201].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c2_info[201].fileTimeLo = 1286818778U;
  c2_info[201].fileTimeHi = 0U;
  c2_info[201].mFileTimeLo = 0U;
  c2_info[201].mFileTimeHi = 0U;
  c2_info[202].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[202].name = "eml_index_minus";
  c2_info[202].dominantType = "coder.internal.indexInt";
  c2_info[202].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c2_info[202].fileTimeLo = 1286818778U;
  c2_info[202].fileTimeHi = 0U;
  c2_info[202].mFileTimeLo = 0U;
  c2_info[202].mFileTimeHi = 0U;
  c2_info[203].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[203].name = "eml_index_times";
  c2_info[203].dominantType = "coder.internal.indexInt";
  c2_info[203].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c2_info[203].fileTimeLo = 1286818780U;
  c2_info[203].fileTimeHi = 0U;
  c2_info[203].mFileTimeLo = 0U;
  c2_info[203].mFileTimeHi = 0U;
  c2_info[204].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[204].name = "eml_index_plus";
  c2_info[204].dominantType = "coder.internal.indexInt";
  c2_info[204].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[204].fileTimeLo = 1286818778U;
  c2_info[204].fileTimeHi = 0U;
  c2_info[204].mFileTimeLo = 0U;
  c2_info[204].mFileTimeHi = 0U;
  c2_info[205].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[205].name = "eml_ixamax";
  c2_info[205].dominantType = "double";
  c2_info[205].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_ixamax.m";
  c2_info[205].fileTimeLo = 1299076770U;
  c2_info[205].fileTimeHi = 0U;
  c2_info[205].mFileTimeLo = 0U;
  c2_info[205].mFileTimeHi = 0U;
  c2_info[206].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_ixamax.m";
  c2_info[206].name = "eml_blas_inline";
  c2_info[206].dominantType = "";
  c2_info[206].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c2_info[206].fileTimeLo = 1299076768U;
  c2_info[206].fileTimeHi = 0U;
  c2_info[206].mFileTimeLo = 0U;
  c2_info[206].mFileTimeHi = 0U;
  c2_info[207].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_ixamax.m!below_threshold";
  c2_info[207].name = "length";
  c2_info[207].dominantType = "double";
  c2_info[207].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m";
  c2_info[207].fileTimeLo = 1303146206U;
  c2_info[207].fileTimeHi = 0U;
  c2_info[207].mFileTimeLo = 0U;
  c2_info[207].mFileTimeHi = 0U;
  c2_info[208].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_ixamax.m";
  c2_info[208].name = "eml_index_class";
  c2_info[208].dominantType = "";
  c2_info[208].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[208].fileTimeLo = 1323170578U;
  c2_info[208].fileTimeHi = 0U;
  c2_info[208].mFileTimeLo = 0U;
  c2_info[208].mFileTimeHi = 0U;
  c2_info[209].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_ixamax.m";
  c2_info[209].name = "eml_refblas_ixamax";
  c2_info[209].dominantType = "double";
  c2_info[209].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c2_info[209].fileTimeLo = 1299076770U;
  c2_info[209].fileTimeHi = 0U;
  c2_info[209].mFileTimeLo = 0U;
  c2_info[209].mFileTimeHi = 0U;
  c2_info[210].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c2_info[210].name = "eml_index_class";
  c2_info[210].dominantType = "";
  c2_info[210].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[210].fileTimeLo = 1323170578U;
  c2_info[210].fileTimeHi = 0U;
  c2_info[210].mFileTimeLo = 0U;
  c2_info[210].mFileTimeHi = 0U;
  c2_info[211].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c2_info[211].name = "eml_xcabs1";
  c2_info[211].dominantType = "double";
  c2_info[211].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xcabs1.m";
  c2_info[211].fileTimeLo = 1286818706U;
  c2_info[211].fileTimeHi = 0U;
  c2_info[211].mFileTimeLo = 0U;
  c2_info[211].mFileTimeHi = 0U;
  c2_info[212].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xcabs1.m";
  c2_info[212].name = "abs";
  c2_info[212].dominantType = "double";
  c2_info[212].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c2_info[212].fileTimeLo = 1286818694U;
  c2_info[212].fileTimeHi = 0U;
  c2_info[212].mFileTimeLo = 0U;
  c2_info[212].mFileTimeHi = 0U;
  c2_info[213].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c2_info[213].name = "eml_int_forloop_overflow_check";
  c2_info[213].dominantType = "";
  c2_info[213].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[213].fileTimeLo = 1332168672U;
  c2_info[213].fileTimeHi = 0U;
  c2_info[213].mFileTimeLo = 0U;
  c2_info[213].mFileTimeHi = 0U;
  c2_info[214].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c2_info[214].name = "eml_index_plus";
  c2_info[214].dominantType = "coder.internal.indexInt";
  c2_info[214].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[214].fileTimeLo = 1286818778U;
  c2_info[214].fileTimeHi = 0U;
  c2_info[214].mFileTimeLo = 0U;
  c2_info[214].mFileTimeHi = 0U;
  c2_info[215].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[215].name = "eml_xswap";
  c2_info[215].dominantType = "double";
  c2_info[215].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xswap.m";
  c2_info[215].fileTimeLo = 1299076778U;
  c2_info[215].fileTimeHi = 0U;
  c2_info[215].mFileTimeLo = 0U;
  c2_info[215].mFileTimeHi = 0U;
  c2_info[216].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[216].name = "eml_div";
  c2_info[216].dominantType = "double";
  c2_info[216].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c2_info[216].fileTimeLo = 1313347810U;
  c2_info[216].fileTimeHi = 0U;
  c2_info[216].mFileTimeLo = 0U;
  c2_info[216].mFileTimeHi = 0U;
  c2_info[217].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[217].name = "eml_xgeru";
  c2_info[217].dominantType = "double";
  c2_info[217].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgeru.m";
  c2_info[217].fileTimeLo = 1299076774U;
  c2_info[217].fileTimeHi = 0U;
  c2_info[217].mFileTimeLo = 0U;
  c2_info[217].mFileTimeHi = 0U;
  c2_info[218].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgeru.m";
  c2_info[218].name = "eml_blas_inline";
  c2_info[218].dominantType = "";
  c2_info[218].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c2_info[218].fileTimeLo = 1299076768U;
  c2_info[218].fileTimeHi = 0U;
  c2_info[218].mFileTimeLo = 0U;
  c2_info[218].mFileTimeHi = 0U;
  c2_info[219].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgeru.m";
  c2_info[219].name = "eml_xger";
  c2_info[219].dominantType = "double";
  c2_info[219].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xger.m";
  c2_info[219].fileTimeLo = 1299076774U;
  c2_info[219].fileTimeHi = 0U;
  c2_info[219].mFileTimeLo = 0U;
  c2_info[219].mFileTimeHi = 0U;
  c2_info[220].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xger.m";
  c2_info[220].name = "eml_blas_inline";
  c2_info[220].dominantType = "";
  c2_info[220].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c2_info[220].fileTimeLo = 1299076768U;
  c2_info[220].fileTimeHi = 0U;
  c2_info[220].mFileTimeLo = 0U;
  c2_info[220].mFileTimeHi = 0U;
  c2_info[221].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m!below_threshold";
  c2_info[221].name = "intmax";
  c2_info[221].dominantType = "char";
  c2_info[221].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c2_info[221].fileTimeLo = 1311255316U;
  c2_info[221].fileTimeHi = 0U;
  c2_info[221].mFileTimeLo = 0U;
  c2_info[221].mFileTimeHi = 0U;
  c2_info[222].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m!below_threshold";
  c2_info[222].name = "min";
  c2_info[222].dominantType = "double";
  c2_info[222].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c2_info[222].fileTimeLo = 1311255318U;
  c2_info[222].fileTimeHi = 0U;
  c2_info[222].mFileTimeLo = 0U;
  c2_info[222].mFileTimeHi = 0U;
  c2_info[223].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m!below_threshold";
  c2_info[223].name = "mtimes";
  c2_info[223].dominantType = "double";
  c2_info[223].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[223].fileTimeLo = 1289519692U;
  c2_info[223].fileTimeHi = 0U;
  c2_info[223].mFileTimeLo = 0U;
  c2_info[223].mFileTimeHi = 0U;
  c2_info[224].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m";
  c2_info[224].name = "eml_index_class";
  c2_info[224].dominantType = "";
  c2_info[224].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[224].fileTimeLo = 1323170578U;
  c2_info[224].fileTimeHi = 0U;
  c2_info[224].mFileTimeLo = 0U;
  c2_info[224].mFileTimeHi = 0U;
  c2_info[225].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m";
  c2_info[225].name = "eml_refblas_xger";
  c2_info[225].dominantType = "double";
  c2_info[225].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xger.m";
  c2_info[225].fileTimeLo = 1299076776U;
  c2_info[225].fileTimeHi = 0U;
  c2_info[225].mFileTimeLo = 0U;
  c2_info[225].mFileTimeHi = 0U;
  c2_info[226].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xger.m";
  c2_info[226].name = "eml_refblas_xgerx";
  c2_info[226].dominantType = "char";
  c2_info[226].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c2_info[226].fileTimeLo = 1299076778U;
  c2_info[226].fileTimeHi = 0U;
  c2_info[226].mFileTimeLo = 0U;
  c2_info[226].mFileTimeHi = 0U;
  c2_info[227].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c2_info[227].name = "eml_index_class";
  c2_info[227].dominantType = "";
  c2_info[227].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[227].fileTimeLo = 1323170578U;
  c2_info[227].fileTimeHi = 0U;
  c2_info[227].mFileTimeLo = 0U;
  c2_info[227].mFileTimeHi = 0U;
  c2_info[228].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c2_info[228].name = "abs";
  c2_info[228].dominantType = "coder.internal.indexInt";
  c2_info[228].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c2_info[228].fileTimeLo = 1286818694U;
  c2_info[228].fileTimeHi = 0U;
  c2_info[228].mFileTimeLo = 0U;
  c2_info[228].mFileTimeHi = 0U;
  c2_info[229].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c2_info[229].name = "eml_index_minus";
  c2_info[229].dominantType = "double";
  c2_info[229].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c2_info[229].fileTimeLo = 1286818778U;
  c2_info[229].fileTimeHi = 0U;
  c2_info[229].mFileTimeLo = 0U;
  c2_info[229].mFileTimeHi = 0U;
  c2_info[230].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c2_info[230].name = "eml_int_forloop_overflow_check";
  c2_info[230].dominantType = "";
  c2_info[230].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[230].fileTimeLo = 1332168672U;
  c2_info[230].fileTimeHi = 0U;
  c2_info[230].mFileTimeLo = 0U;
  c2_info[230].mFileTimeHi = 0U;
  c2_info[231].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c2_info[231].name = "eml_index_plus";
  c2_info[231].dominantType = "double";
  c2_info[231].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[231].fileTimeLo = 1286818778U;
  c2_info[231].fileTimeHi = 0U;
  c2_info[231].mFileTimeLo = 0U;
  c2_info[231].mFileTimeHi = 0U;
  c2_info[232].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c2_info[232].name = "eml_index_plus";
  c2_info[232].dominantType = "coder.internal.indexInt";
  c2_info[232].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[232].fileTimeLo = 1286818778U;
  c2_info[232].fileTimeHi = 0U;
  c2_info[232].mFileTimeLo = 0U;
  c2_info[232].mFileTimeHi = 0U;
  c2_info[233].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!invNxN";
  c2_info[233].name = "eml_ipiv2perm";
  c2_info[233].dominantType = "coder.internal.indexInt";
  c2_info[233].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_ipiv2perm.m";
  c2_info[233].fileTimeLo = 1286818782U;
  c2_info[233].fileTimeHi = 0U;
  c2_info[233].mFileTimeLo = 0U;
  c2_info[233].mFileTimeHi = 0U;
  c2_info[234].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_ipiv2perm.m";
  c2_info[234].name = "colon";
  c2_info[234].dominantType = "double";
  c2_info[234].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m";
  c2_info[234].fileTimeLo = 1311255318U;
  c2_info[234].fileTimeHi = 0U;
  c2_info[234].mFileTimeLo = 0U;
  c2_info[234].mFileTimeHi = 0U;
  c2_info[235].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_ipiv2perm.m";
  c2_info[235].name = "eml_index_class";
  c2_info[235].dominantType = "";
  c2_info[235].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[235].fileTimeLo = 1323170578U;
  c2_info[235].fileTimeHi = 0U;
  c2_info[235].mFileTimeLo = 0U;
  c2_info[235].mFileTimeHi = 0U;
  c2_info[236].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_ipiv2perm.m";
  c2_info[236].name = "coder.internal.indexIntRelop";
  c2_info[236].dominantType = "";
  c2_info[236].resolved =
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexIntRelop.m";
  c2_info[236].fileTimeLo = 1326728322U;
  c2_info[236].fileTimeHi = 0U;
  c2_info[236].mFileTimeLo = 0U;
  c2_info[236].mFileTimeHi = 0U;
  c2_info[237].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!invNxN";
  c2_info[237].name = "eml_int_forloop_overflow_check";
  c2_info[237].dominantType = "";
  c2_info[237].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[237].fileTimeLo = 1332168672U;
  c2_info[237].fileTimeHi = 0U;
  c2_info[237].mFileTimeLo = 0U;
  c2_info[237].mFileTimeHi = 0U;
  c2_info[238].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!invNxN";
  c2_info[238].name = "eml_index_plus";
  c2_info[238].dominantType = "double";
  c2_info[238].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[238].fileTimeLo = 1286818778U;
  c2_info[238].fileTimeHi = 0U;
  c2_info[238].mFileTimeLo = 0U;
  c2_info[238].mFileTimeHi = 0U;
  c2_info[239].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!invNxN";
  c2_info[239].name = "mtimes";
  c2_info[239].dominantType = "double";
  c2_info[239].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[239].fileTimeLo = 1289519692U;
  c2_info[239].fileTimeHi = 0U;
  c2_info[239].mFileTimeLo = 0U;
  c2_info[239].mFileTimeHi = 0U;
  c2_info[240].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!invNxN";
  c2_info[240].name = "eml_scalar_eg";
  c2_info[240].dominantType = "double";
  c2_info[240].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[240].fileTimeLo = 1286818796U;
  c2_info[240].fileTimeHi = 0U;
  c2_info[240].mFileTimeLo = 0U;
  c2_info[240].mFileTimeHi = 0U;
  c2_info[241].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!invNxN";
  c2_info[241].name = "eml_xtrsm";
  c2_info[241].dominantType = "char";
  c2_info[241].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xtrsm.m";
  c2_info[241].fileTimeLo = 1299076778U;
  c2_info[241].fileTimeHi = 0U;
  c2_info[241].mFileTimeLo = 0U;
  c2_info[241].mFileTimeHi = 0U;
  c2_info[242].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xtrsm.m";
  c2_info[242].name = "eml_blas_inline";
  c2_info[242].dominantType = "";
  c2_info[242].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c2_info[242].fileTimeLo = 1299076768U;
  c2_info[242].fileTimeHi = 0U;
  c2_info[242].mFileTimeLo = 0U;
  c2_info[242].mFileTimeHi = 0U;
  c2_info[243].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xtrsm.m!below_threshold";
  c2_info[243].name = "mtimes";
  c2_info[243].dominantType = "double";
  c2_info[243].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[243].fileTimeLo = 1289519692U;
  c2_info[243].fileTimeHi = 0U;
  c2_info[243].mFileTimeLo = 0U;
  c2_info[243].mFileTimeHi = 0U;
  c2_info[244].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xtrsm.m";
  c2_info[244].name = "eml_index_class";
  c2_info[244].dominantType = "";
  c2_info[244].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[244].fileTimeLo = 1323170578U;
  c2_info[244].fileTimeHi = 0U;
  c2_info[244].mFileTimeLo = 0U;
  c2_info[244].mFileTimeHi = 0U;
  c2_info[245].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xtrsm.m";
  c2_info[245].name = "eml_scalar_eg";
  c2_info[245].dominantType = "double";
  c2_info[245].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[245].fileTimeLo = 1286818796U;
  c2_info[245].fileTimeHi = 0U;
  c2_info[245].mFileTimeLo = 0U;
  c2_info[245].mFileTimeHi = 0U;
  c2_info[246].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xtrsm.m";
  c2_info[246].name = "eml_refblas_xtrsm";
  c2_info[246].dominantType = "char";
  c2_info[246].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c2_info[246].fileTimeLo = 1299076786U;
  c2_info[246].fileTimeHi = 0U;
  c2_info[246].mFileTimeLo = 0U;
  c2_info[246].mFileTimeHi = 0U;
  c2_info[247].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c2_info[247].name = "eml_scalar_eg";
  c2_info[247].dominantType = "double";
  c2_info[247].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[247].fileTimeLo = 1286818796U;
  c2_info[247].fileTimeHi = 0U;
  c2_info[247].mFileTimeLo = 0U;
  c2_info[247].mFileTimeHi = 0U;
  c2_info[248].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c2_info[248].name = "eml_index_minus";
  c2_info[248].dominantType = "double";
  c2_info[248].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c2_info[248].fileTimeLo = 1286818778U;
  c2_info[248].fileTimeHi = 0U;
  c2_info[248].mFileTimeLo = 0U;
  c2_info[248].mFileTimeHi = 0U;
  c2_info[249].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c2_info[249].name = "eml_index_class";
  c2_info[249].dominantType = "";
  c2_info[249].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[249].fileTimeLo = 1323170578U;
  c2_info[249].fileTimeHi = 0U;
  c2_info[249].mFileTimeLo = 0U;
  c2_info[249].mFileTimeHi = 0U;
  c2_info[250].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c2_info[250].name = "eml_int_forloop_overflow_check";
  c2_info[250].dominantType = "";
  c2_info[250].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[250].fileTimeLo = 1332168672U;
  c2_info[250].fileTimeHi = 0U;
  c2_info[250].mFileTimeLo = 0U;
  c2_info[250].mFileTimeHi = 0U;
  c2_info[251].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c2_info[251].name = "eml_index_times";
  c2_info[251].dominantType = "coder.internal.indexInt";
  c2_info[251].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c2_info[251].fileTimeLo = 1286818780U;
  c2_info[251].fileTimeHi = 0U;
  c2_info[251].mFileTimeLo = 0U;
  c2_info[251].mFileTimeHi = 0U;
  c2_info[252].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c2_info[252].name = "eml_index_plus";
  c2_info[252].dominantType = "coder.internal.indexInt";
  c2_info[252].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[252].fileTimeLo = 1286818778U;
  c2_info[252].fileTimeHi = 0U;
  c2_info[252].mFileTimeLo = 0U;
  c2_info[252].mFileTimeHi = 0U;
  c2_info[253].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c2_info[253].name = "eml_div";
  c2_info[253].dominantType = "double";
  c2_info[253].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c2_info[253].fileTimeLo = 1313347810U;
  c2_info[253].fileTimeHi = 0U;
  c2_info[253].mFileTimeLo = 0U;
  c2_info[253].mFileTimeHi = 0U;
  c2_info[254].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!checkcond";
  c2_info[254].name = "norm";
  c2_info[254].dominantType = "double";
  c2_info[254].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m";
  c2_info[254].fileTimeLo = 1336522094U;
  c2_info[254].fileTimeHi = 0U;
  c2_info[254].mFileTimeLo = 0U;
  c2_info[254].mFileTimeHi = 0U;
  c2_info[255].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m!mat1norm";
  c2_info[255].name = "abs";
  c2_info[255].dominantType = "double";
  c2_info[255].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c2_info[255].fileTimeLo = 1286818694U;
  c2_info[255].fileTimeHi = 0U;
  c2_info[255].mFileTimeLo = 0U;
  c2_info[255].mFileTimeHi = 0U;
}

static void c2_e_info_helper(c2_ResolvedFunctionInfo c2_info[268])
{
  c2_info[256].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m!mat1norm";
  c2_info[256].name = "isnan";
  c2_info[256].dominantType = "double";
  c2_info[256].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m";
  c2_info[256].fileTimeLo = 1286818760U;
  c2_info[256].fileTimeHi = 0U;
  c2_info[256].mFileTimeLo = 0U;
  c2_info[256].mFileTimeHi = 0U;
  c2_info[257].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m!mat1norm";
  c2_info[257].name = "eml_guarded_nan";
  c2_info[257].dominantType = "char";
  c2_info[257].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_guarded_nan.m";
  c2_info[257].fileTimeLo = 1286818776U;
  c2_info[257].fileTimeHi = 0U;
  c2_info[257].mFileTimeLo = 0U;
  c2_info[257].mFileTimeHi = 0U;
  c2_info[258].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_guarded_nan.m";
  c2_info[258].name = "eml_is_float_class";
  c2_info[258].dominantType = "char";
  c2_info[258].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_is_float_class.m";
  c2_info[258].fileTimeLo = 1286818782U;
  c2_info[258].fileTimeHi = 0U;
  c2_info[258].mFileTimeLo = 0U;
  c2_info[258].mFileTimeHi = 0U;
  c2_info[259].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!checkcond";
  c2_info[259].name = "mtimes";
  c2_info[259].dominantType = "double";
  c2_info[259].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[259].fileTimeLo = 1289519692U;
  c2_info[259].fileTimeHi = 0U;
  c2_info[259].mFileTimeLo = 0U;
  c2_info[259].mFileTimeHi = 0U;
  c2_info[260].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!checkcond";
  c2_info[260].name = "eml_warning";
  c2_info[260].dominantType = "char";
  c2_info[260].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_warning.m";
  c2_info[260].fileTimeLo = 1286818802U;
  c2_info[260].fileTimeHi = 0U;
  c2_info[260].mFileTimeLo = 0U;
  c2_info[260].mFileTimeHi = 0U;
  c2_info[261].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!checkcond";
  c2_info[261].name = "isnan";
  c2_info[261].dominantType = "double";
  c2_info[261].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m";
  c2_info[261].fileTimeLo = 1286818760U;
  c2_info[261].fileTimeHi = 0U;
  c2_info[261].mFileTimeLo = 0U;
  c2_info[261].mFileTimeHi = 0U;
  c2_info[262].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!checkcond";
  c2_info[262].name = "eps";
  c2_info[262].dominantType = "char";
  c2_info[262].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c2_info[262].fileTimeLo = 1326727996U;
  c2_info[262].fileTimeHi = 0U;
  c2_info[262].mFileTimeLo = 0U;
  c2_info[262].mFileTimeHi = 0U;
  c2_info[263].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!checkcond";
  c2_info[263].name = "eml_flt2str";
  c2_info[263].dominantType = "double";
  c2_info[263].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_flt2str.m";
  c2_info[263].fileTimeLo = 1309451196U;
  c2_info[263].fileTimeHi = 0U;
  c2_info[263].mFileTimeLo = 0U;
  c2_info[263].mFileTimeHi = 0U;
  c2_info[264].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_flt2str.m";
  c2_info[264].name = "char";
  c2_info[264].dominantType = "double";
  c2_info[264].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/strfun/char.m";
  c2_info[264].fileTimeLo = 1319729968U;
  c2_info[264].fileTimeHi = 0U;
  c2_info[264].mFileTimeLo = 0U;
  c2_info[264].mFileTimeHi = 0U;
  c2_info[265].context = "";
  c2_info[265].name = "norm";
  c2_info[265].dominantType = "double";
  c2_info[265].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m";
  c2_info[265].fileTimeLo = 1336522094U;
  c2_info[265].fileTimeHi = 0U;
  c2_info[265].mFileTimeLo = 0U;
  c2_info[265].mFileTimeHi = 0U;
  c2_info[266].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m!genpnorm";
  c2_info[266].name = "eml_index_class";
  c2_info[266].dominantType = "";
  c2_info[266].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[266].fileTimeLo = 1323170578U;
  c2_info[266].fileTimeHi = 0U;
  c2_info[266].mFileTimeLo = 0U;
  c2_info[266].mFileTimeHi = 0U;
  c2_info[267].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m!genpnorm";
  c2_info[267].name = "eml_xnrm2";
  c2_info[267].dominantType = "double";
  c2_info[267].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xnrm2.m";
  c2_info[267].fileTimeLo = 1299076776U;
  c2_info[267].fileTimeHi = 0U;
  c2_info[267].mFileTimeLo = 0U;
  c2_info[267].mFileTimeHi = 0U;
}

static void c2_isVariableSizing(SFc2_controllerInstanceStruct *chartInstance)
{
}

static void c2_eml_scalar_eg(SFc2_controllerInstanceStruct *chartInstance)
{
}

static void c2_check_forloop_overflow_error(SFc2_controllerInstanceStruct
  *chartInstance)
{
}

static void c2_b_eml_scalar_eg(SFc2_controllerInstanceStruct *chartInstance)
{
}

static boolean_T c2_eml_use_refblas(SFc2_controllerInstanceStruct *chartInstance)
{
  return FALSE;
}

static void c2_eye(SFc2_controllerInstanceStruct *chartInstance, real_T c2_I[9])
{
  int32_T c2_i332;
  int32_T c2_i;
  int32_T c2_b_i;
  c2_isVariableSizing(chartInstance);
  for (c2_i332 = 0; c2_i332 < 9; c2_i332++) {
    c2_I[c2_i332] = 0.0;
  }

  c2_check_forloop_overflow_error(chartInstance);
  for (c2_i = 1; c2_i < 4; c2_i++) {
    c2_b_i = c2_i;
    c2_I[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_b_i), 1, 3, 1, 0) + 3 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
            (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_i), 1, 3, 2, 0) - 1)) -
      1] = 1.0;
  }
}

static void c2_Sf(SFc2_controllerInstanceStruct *chartInstance, real_T c2_w[3],
                  real_T c2_S[9])
{
  uint32_T c2_debug_family_var_map[4];
  real_T c2_nargin = 1.0;
  real_T c2_nargout = 1.0;
  sf_debug_symbol_scope_push_eml(0U, 4U, 4U, c2_b_debug_family_names,
    c2_debug_family_var_map);
  sf_debug_symbol_scope_add_eml_importable(&c2_nargin, 0U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c2_nargout, 1U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_w, 2U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_S, 3U, c2_e_sf_marshallOut,
    c2_m_sf_marshallIn);
  CV_SCRIPT_FCN(0, 0);
  _SFD_SCRIPT_CALL(0U, chartInstance->c2_sfEvent, 4);
  c2_S[0] = 0.0;
  c2_S[3] = -c2_w[2];
  c2_S[6] = c2_w[1];
  c2_S[1] = c2_w[2];
  c2_S[4] = 0.0;
  c2_S[7] = -c2_w[0];
  c2_S[2] = -c2_w[1];
  c2_S[5] = c2_w[0];
  c2_S[8] = 0.0;
  _SFD_SCRIPT_CALL(0U, chartInstance->c2_sfEvent, -4);
  sf_debug_symbol_scope_pop();
}

static void c2_c_eml_scalar_eg(SFc2_controllerInstanceStruct *chartInstance)
{
}

static boolean_T c2_isfinite(SFc2_controllerInstanceStruct *chartInstance,
  real_T c2_x)
{
  real_T c2_b_x;
  boolean_T c2_b_b;
  boolean_T c2_b0;
  real_T c2_c_x;
  boolean_T c2_c_b;
  boolean_T c2_b1;
  c2_b_x = c2_x;
  c2_b_b = muDoubleScalarIsInf(c2_b_x);
  c2_b0 = !c2_b_b;
  c2_c_x = c2_x;
  c2_c_b = muDoubleScalarIsNaN(c2_c_x);
  c2_b1 = !c2_c_b;
  return c2_b0 && c2_b1;
}

static void c2_eml_error(SFc2_controllerInstanceStruct *chartInstance)
{
  int32_T c2_i333;
  static char_T c2_varargin_1[33] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A',
    'T', 'L', 'A', 'B', ':', 's', 'v', 'd', '_', 'm', 'a', 't', 'r', 'i', 'x',
    'W', 'i', 't', 'h', 'N', 'a', 'N', 'I', 'n', 'f' };

  char_T c2_u[33];
  const mxArray *c2_y = NULL;
  for (c2_i333 = 0; c2_i333 < 33; c2_i333++) {
    c2_u[c2_i333] = c2_varargin_1[c2_i333];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 10, 0U, 1U, 0U, 2, 1, 33), FALSE);
  sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 1U, 14,
    c2_y));
}

static real_T c2_abs(SFc2_controllerInstanceStruct *chartInstance, real_T c2_x)
{
  real_T c2_b_x;
  c2_b_x = c2_x;
  return muDoubleScalarAbs(c2_b_x);
}

static void c2_realmin(SFc2_controllerInstanceStruct *chartInstance)
{
}

static void c2_pinv(SFc2_controllerInstanceStruct *chartInstance, real_T c2_A[72],
                    real_T c2_X[72])
{
  int32_T c2_i334;
  int32_T c2_i335;
  int32_T c2_i336;
  int32_T c2_i337;
  real_T c2_U[72];
  int32_T c2_i338;
  real_T c2_b_X[72];
  int32_T c2_k;
  int32_T c2_b_k;
  real_T c2_x;
  real_T c2_b_x;
  boolean_T c2_b;
  boolean_T c2_b2;
  real_T c2_c_x;
  boolean_T c2_b_b;
  boolean_T c2_b3;
  boolean_T c2_c_b;
  int32_T c2_i339;
  real_T c2_b_U[72];
  real_T c2_V[36];
  real_T c2_s[6];
  int32_T c2_i340;
  real_T c2_S[36];
  int32_T c2_c_k;
  real_T c2_d_k;
  int32_T c2_r;
  int32_T c2_e_k;
  int32_T c2_f_k;
  int32_T c2_a;
  int32_T c2_vcol;
  int32_T c2_b_r;
  int32_T c2_d_b;
  int32_T c2_e_b;
  boolean_T c2_overflow;
  int32_T c2_j;
  int32_T c2_b_j;
  real_T c2_y;
  real_T c2_z;
  int32_T c2_b_a;
  int32_T c2_i341;
  real_T c2_b_V[36];
  int32_T c2_i342;
  real_T c2_c_U[72];
  int32_T c2_i343;
  int32_T c2_i344;
  int32_T c2_i345;
  int32_T c2_i346;
  boolean_T exitg1;
  c2_i334 = 0;
  for (c2_i335 = 0; c2_i335 < 6; c2_i335++) {
    c2_i336 = 0;
    for (c2_i337 = 0; c2_i337 < 12; c2_i337++) {
      c2_U[c2_i337 + c2_i334] = c2_A[c2_i336 + c2_i335];
      c2_i336 += 6;
    }

    c2_i334 += 12;
  }

  for (c2_i338 = 0; c2_i338 < 72; c2_i338++) {
    c2_b_X[c2_i338] = 0.0;
  }

  c2_check_forloop_overflow_error(chartInstance);
  for (c2_k = 1; c2_k < 73; c2_k++) {
    c2_b_k = c2_k;
    c2_x = c2_U[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c2_b_k), 1, 72, 1, 0) - 1];
    c2_b_x = c2_x;
    c2_b = muDoubleScalarIsInf(c2_b_x);
    c2_b2 = !c2_b;
    c2_c_x = c2_x;
    c2_b_b = muDoubleScalarIsNaN(c2_c_x);
    c2_b3 = !c2_b_b;
    c2_c_b = (c2_b2 && c2_b3);
    if (!c2_c_b) {
      c2_eml_error(chartInstance);
    }
  }

  for (c2_i339 = 0; c2_i339 < 72; c2_i339++) {
    c2_b_U[c2_i339] = c2_U[c2_i339];
  }

  c2_eml_xgesvd(chartInstance, c2_b_U, c2_U, c2_s, c2_V);
  for (c2_i340 = 0; c2_i340 < 36; c2_i340++) {
    c2_S[c2_i340] = 0.0;
  }

  for (c2_c_k = 0; c2_c_k < 6; c2_c_k++) {
    c2_d_k = 1.0 + (real_T)c2_c_k;
    c2_S[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", c2_d_k),
           1, 6, 1, 0) + 6 * (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", c2_d_k), 1, 6, 2, 0) - 1)) - 1] =
      c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      c2_d_k), 1, 6, 1, 0) - 1];
  }

  c2_r = 0;
  c2_check_forloop_overflow_error(chartInstance);
  c2_e_k = 1;
  exitg1 = FALSE;
  while ((exitg1 == FALSE) && (c2_e_k < 7)) {
    c2_f_k = c2_e_k;
    if (!(c2_S[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_f_k), 1, 6, 1, 0) + 6 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
            (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_f_k), 1, 6, 2, 0) - 1)) -
          1] > 1.0E-5)) {
      exitg1 = TRUE;
    } else {
      c2_a = c2_r + 1;
      c2_r = c2_a;
      c2_e_k++;
    }
  }

  if (c2_r > 0) {
    c2_vcol = 1;
    c2_b_r = c2_r;
    c2_d_b = c2_b_r;
    c2_e_b = c2_d_b;
    if (1 > c2_e_b) {
      c2_overflow = FALSE;
    } else {
      c2_overflow = (c2_e_b > 2147483646);
    }

    c2_b_check_forloop_overflow_error(chartInstance, c2_overflow);
    for (c2_j = 1; c2_j <= c2_b_r; c2_j++) {
      c2_b_j = c2_j;
      c2_y = c2_S[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
        "", (real_T)c2_b_j), 1, 6, 1, 0) + 6 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
                     (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_j), 1, 6, 2, 0)
        - 1)) - 1];
      c2_z = 1.0 / c2_y;
      c2_i_eml_xscal(chartInstance, c2_z, c2_V, c2_vcol);
      c2_b_a = c2_vcol + 6;
      c2_vcol = c2_b_a;
    }

    for (c2_i341 = 0; c2_i341 < 36; c2_i341++) {
      c2_b_V[c2_i341] = c2_V[c2_i341];
    }

    for (c2_i342 = 0; c2_i342 < 72; c2_i342++) {
      c2_c_U[c2_i342] = c2_U[c2_i342];
    }

    c2_j_eml_xgemm(chartInstance, c2_r, c2_b_V, c2_c_U, c2_b_X);
  }

  c2_i343 = 0;
  for (c2_i344 = 0; c2_i344 < 6; c2_i344++) {
    c2_i345 = 0;
    for (c2_i346 = 0; c2_i346 < 12; c2_i346++) {
      c2_X[c2_i346 + c2_i343] = c2_b_X[c2_i345 + c2_i344];
      c2_i345 += 6;
    }

    c2_i343 += 12;
  }
}

static void c2_eml_xgesvd(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_A[72], real_T c2_U[72], real_T c2_S[6], real_T c2_V[36])
{
  int32_T c2_i347;
  real_T c2_b_A[72];
  int32_T c2_i348;
  real_T c2_s[6];
  int32_T c2_i349;
  real_T c2_e[6];
  int32_T c2_i350;
  real_T c2_work[12];
  int32_T c2_i351;
  int32_T c2_i352;
  real_T c2_Vf[36];
  int32_T c2_q;
  int32_T c2_b_q;
  int32_T c2_a;
  int32_T c2_qp1;
  int32_T c2_b_a;
  int32_T c2_qm1;
  int32_T c2_b;
  int32_T c2_c;
  int32_T c2_c_a;
  int32_T c2_b_b;
  int32_T c2_qq;
  int32_T c2_c_b;
  int32_T c2_nmq;
  int32_T c2_d_a;
  int32_T c2_nmqp1;
  int32_T c2_i353;
  real_T c2_c_A[72];
  real_T c2_nrm;
  real_T c2_absx;
  real_T c2_d;
  real_T c2_y;
  real_T c2_d1;
  int32_T c2_b_qp1;
  boolean_T c2_overflow;
  int32_T c2_jj;
  int32_T c2_b_jj;
  int32_T c2_e_a;
  int32_T c2_b_c;
  int32_T c2_d_b;
  int32_T c2_c_c;
  int32_T c2_f_a;
  int32_T c2_e_b;
  int32_T c2_qjj;
  int32_T c2_i354;
  real_T c2_d_A[72];
  int32_T c2_i355;
  real_T c2_e_A[72];
  real_T c2_t;
  int32_T c2_c_q;
  boolean_T c2_b_overflow;
  int32_T c2_ii;
  int32_T c2_b_ii;
  int32_T c2_f_b;
  int32_T c2_pmq;
  int32_T c2_i356;
  real_T c2_b_e[6];
  real_T c2_b_absx;
  real_T c2_b_d;
  real_T c2_b_y;
  real_T c2_d2;
  int32_T c2_c_qp1;
  boolean_T c2_c_overflow;
  int32_T c2_c_ii;
  int32_T c2_d_qp1;
  boolean_T c2_d_overflow;
  int32_T c2_c_jj;
  int32_T c2_g_a;
  int32_T c2_d_c;
  int32_T c2_g_b;
  int32_T c2_e_c;
  int32_T c2_h_a;
  int32_T c2_h_b;
  int32_T c2_qp1jj;
  int32_T c2_i357;
  real_T c2_f_A[72];
  int32_T c2_e_qp1;
  boolean_T c2_e_overflow;
  int32_T c2_d_jj;
  int32_T c2_i_a;
  int32_T c2_f_c;
  int32_T c2_i_b;
  int32_T c2_g_c;
  int32_T c2_j_a;
  int32_T c2_j_b;
  int32_T c2_i358;
  real_T c2_b_work[12];
  int32_T c2_f_qp1;
  boolean_T c2_f_overflow;
  int32_T c2_d_ii;
  int32_T c2_m;
  int32_T c2_d_q;
  int32_T c2_k_a;
  int32_T c2_k_b;
  int32_T c2_l_a;
  int32_T c2_m_a;
  int32_T c2_h_c;
  int32_T c2_l_b;
  int32_T c2_i_c;
  int32_T c2_n_a;
  int32_T c2_m_b;
  int32_T c2_g_qp1;
  boolean_T c2_g_overflow;
  int32_T c2_e_jj;
  int32_T c2_o_a;
  int32_T c2_j_c;
  int32_T c2_n_b;
  int32_T c2_k_c;
  int32_T c2_p_a;
  int32_T c2_o_b;
  int32_T c2_i359;
  real_T c2_b_U[72];
  int32_T c2_i360;
  real_T c2_c_U[72];
  int32_T c2_e_q;
  boolean_T c2_h_overflow;
  int32_T c2_e_ii;
  int32_T c2_q_a;
  int32_T c2_i361;
  int32_T c2_p_b;
  int32_T c2_q_b;
  boolean_T c2_i_overflow;
  int32_T c2_f_ii;
  int32_T c2_g_ii;
  int32_T c2_f_q;
  int32_T c2_r_a;
  int32_T c2_r_b;
  int32_T c2_s_a;
  int32_T c2_l_c;
  int32_T c2_s_b;
  int32_T c2_m_c;
  int32_T c2_t_a;
  int32_T c2_t_b;
  int32_T c2_qp1q;
  int32_T c2_h_qp1;
  boolean_T c2_j_overflow;
  int32_T c2_f_jj;
  int32_T c2_u_a;
  int32_T c2_n_c;
  int32_T c2_u_b;
  int32_T c2_o_c;
  int32_T c2_v_a;
  int32_T c2_v_b;
  int32_T c2_i362;
  real_T c2_b_Vf[36];
  int32_T c2_i363;
  real_T c2_c_Vf[36];
  int32_T c2_h_ii;
  int32_T c2_g_q;
  real_T c2_rt;
  real_T c2_r;
  int32_T c2_w_a;
  int32_T c2_p_c;
  int32_T c2_w_b;
  int32_T c2_q_c;
  int32_T c2_x_b;
  int32_T c2_colq;
  int32_T c2_i364;
  int32_T c2_x_a;
  int32_T c2_r_c;
  int32_T c2_y_a;
  int32_T c2_s_c;
  real_T c2_ab_a;
  real_T c2_y_b;
  real_T c2_c_y;
  int32_T c2_ab_b;
  int32_T c2_t_c;
  int32_T c2_bb_b;
  int32_T c2_colqp1;
  real_T c2_iter;
  real_T c2_tiny;
  real_T c2_snorm;
  int32_T c2_i_ii;
  real_T c2_varargin_1;
  real_T c2_varargin_2;
  real_T c2_b_varargin_2;
  real_T c2_varargin_3;
  real_T c2_x;
  real_T c2_d_y;
  real_T c2_b_x;
  real_T c2_e_y;
  real_T c2_xk;
  real_T c2_yk;
  real_T c2_c_x;
  real_T c2_f_y;
  real_T c2_maxval;
  real_T c2_b_varargin_1;
  real_T c2_c_varargin_2;
  real_T c2_d_varargin_2;
  real_T c2_b_varargin_3;
  real_T c2_d_x;
  real_T c2_g_y;
  real_T c2_e_x;
  real_T c2_h_y;
  real_T c2_b_xk;
  real_T c2_b_yk;
  real_T c2_f_x;
  real_T c2_i_y;
  int32_T c2_bb_a;
  int32_T c2_cb_a;
  int32_T c2_i365;
  boolean_T c2_k_overflow;
  int32_T c2_j_ii;
  int32_T c2_db_a;
  int32_T c2_u_c;
  real_T c2_test0;
  real_T c2_ztest0;
  real_T c2_cb_b;
  real_T c2_j_y;
  real_T c2_db_b;
  real_T c2_k_y;
  int32_T c2_eb_a;
  int32_T c2_v_c;
  real_T c2_kase;
  int32_T c2_qs;
  int32_T c2_b_m;
  int32_T c2_h_q;
  int32_T c2_fb_a;
  int32_T c2_eb_b;
  int32_T c2_gb_a;
  int32_T c2_fb_b;
  boolean_T c2_l_overflow;
  int32_T c2_k_ii;
  real_T c2_test;
  int32_T c2_hb_a;
  int32_T c2_w_c;
  int32_T c2_ib_a;
  int32_T c2_x_c;
  real_T c2_ztest;
  real_T c2_gb_b;
  real_T c2_l_y;
  int32_T c2_jb_a;
  int32_T c2_kb_a;
  int32_T c2_y_c;
  real_T c2_f;
  int32_T c2_lb_a;
  int32_T c2_ab_c;
  int32_T c2_mb_a;
  int32_T c2_i366;
  int32_T c2_i_q;
  int32_T c2_nb_a;
  int32_T c2_hb_b;
  int32_T c2_ob_a;
  int32_T c2_ib_b;
  boolean_T c2_m_overflow;
  int32_T c2_k;
  int32_T c2_b_k;
  real_T c2_t1;
  real_T c2_b_t1;
  real_T c2_b_f;
  real_T c2_sn;
  real_T c2_cs;
  real_T c2_b_cs;
  real_T c2_b_sn;
  int32_T c2_pb_a;
  int32_T c2_km1;
  real_T c2_qb_a;
  real_T c2_jb_b;
  real_T c2_rb_a;
  real_T c2_kb_b;
  real_T c2_m_y;
  int32_T c2_sb_a;
  int32_T c2_bb_c;
  int32_T c2_lb_b;
  int32_T c2_cb_c;
  int32_T c2_mb_b;
  int32_T c2_colk;
  int32_T c2_tb_a;
  int32_T c2_db_c;
  int32_T c2_nb_b;
  int32_T c2_eb_c;
  int32_T c2_ob_b;
  int32_T c2_colm;
  int32_T c2_ub_a;
  int32_T c2_j_q;
  int32_T c2_c_m;
  int32_T c2_vb_a;
  int32_T c2_pb_b;
  int32_T c2_wb_a;
  int32_T c2_qb_b;
  boolean_T c2_n_overflow;
  int32_T c2_c_k;
  real_T c2_c_t1;
  real_T c2_unusedU0;
  real_T c2_c_sn;
  real_T c2_c_cs;
  real_T c2_xb_a;
  real_T c2_rb_b;
  real_T c2_yb_a;
  real_T c2_sb_b;
  real_T c2_n_y;
  int32_T c2_ac_a;
  int32_T c2_fb_c;
  int32_T c2_tb_b;
  int32_T c2_gb_c;
  int32_T c2_ub_b;
  int32_T c2_bc_a;
  int32_T c2_hb_c;
  int32_T c2_vb_b;
  int32_T c2_ib_c;
  int32_T c2_wb_b;
  int32_T c2_colqm1;
  int32_T c2_cc_a;
  int32_T c2_mm1;
  real_T c2_d3;
  real_T c2_d4;
  real_T c2_d5;
  real_T c2_d6;
  real_T c2_d7;
  real_T c2_c_varargin_1[5];
  int32_T c2_ixstart;
  real_T c2_mtmp;
  real_T c2_g_x;
  boolean_T c2_xb_b;
  int32_T c2_ix;
  int32_T c2_b_ix;
  real_T c2_h_x;
  boolean_T c2_yb_b;
  int32_T c2_dc_a;
  int32_T c2_i367;
  boolean_T c2_o_overflow;
  int32_T c2_c_ix;
  real_T c2_ec_a;
  real_T c2_ac_b;
  boolean_T c2_p;
  real_T c2_b_mtmp;
  real_T c2_scale;
  real_T c2_sm;
  real_T c2_smm1;
  real_T c2_emm1;
  real_T c2_sqds;
  real_T c2_eqds;
  real_T c2_fc_a;
  real_T c2_bc_b;
  real_T c2_o_y;
  real_T c2_gc_a;
  real_T c2_cc_b;
  real_T c2_p_y;
  real_T c2_dc_b;
  real_T c2_hc_a;
  real_T c2_ec_b;
  real_T c2_jb_c;
  real_T c2_ic_a;
  real_T c2_fc_b;
  real_T c2_shift;
  real_T c2_jc_a;
  real_T c2_gc_b;
  real_T c2_q_y;
  real_T c2_kc_a;
  real_T c2_hc_b;
  real_T c2_r_y;
  real_T c2_lc_a;
  real_T c2_ic_b;
  real_T c2_g;
  int32_T c2_k_q;
  int32_T c2_b_mm1;
  int32_T c2_mc_a;
  int32_T c2_jc_b;
  int32_T c2_nc_a;
  int32_T c2_kc_b;
  boolean_T c2_p_overflow;
  int32_T c2_d_k;
  int32_T c2_oc_a;
  int32_T c2_pc_a;
  int32_T c2_kp1;
  real_T c2_c_f;
  real_T c2_unusedU1;
  real_T c2_d_sn;
  real_T c2_d_cs;
  real_T c2_qc_a;
  real_T c2_lc_b;
  real_T c2_s_y;
  real_T c2_rc_a;
  real_T c2_mc_b;
  real_T c2_t_y;
  real_T c2_sc_a;
  real_T c2_nc_b;
  real_T c2_u_y;
  real_T c2_tc_a;
  real_T c2_oc_b;
  real_T c2_v_y;
  real_T c2_uc_a;
  real_T c2_pc_b;
  real_T c2_vc_a;
  real_T c2_qc_b;
  real_T c2_w_y;
  int32_T c2_wc_a;
  int32_T c2_kb_c;
  int32_T c2_rc_b;
  int32_T c2_lb_c;
  int32_T c2_sc_b;
  int32_T c2_tc_b;
  int32_T c2_mb_c;
  int32_T c2_uc_b;
  int32_T c2_colkp1;
  real_T c2_d_f;
  real_T c2_unusedU2;
  real_T c2_e_sn;
  real_T c2_e_cs;
  real_T c2_xc_a;
  real_T c2_vc_b;
  real_T c2_x_y;
  real_T c2_yc_a;
  real_T c2_wc_b;
  real_T c2_y_y;
  real_T c2_ad_a;
  real_T c2_xc_b;
  real_T c2_ab_y;
  real_T c2_bd_a;
  real_T c2_yc_b;
  real_T c2_bb_y;
  real_T c2_cd_a;
  real_T c2_ad_b;
  real_T c2_dd_a;
  real_T c2_bd_b;
  real_T c2_cb_y;
  int32_T c2_ed_a;
  int32_T c2_nb_c;
  int32_T c2_cd_b;
  int32_T c2_ob_c;
  int32_T c2_dd_b;
  int32_T c2_ed_b;
  int32_T c2_pb_c;
  int32_T c2_fd_b;
  int32_T c2_fd_a;
  int32_T c2_qb_c;
  int32_T c2_e_k;
  int32_T c2_j;
  int32_T c2_b_j;
  int32_T c2_i;
  int32_T c2_b_i;
  int32_T c2_rb_c;
  int32_T c2_gd_a;
  int32_T c2_sb_c;
  int32_T c2_gd_b;
  int32_T c2_hd_b;
  int32_T c2_hd_a;
  int32_T c2_id_a;
  int32_T c2_tb_c;
  int32_T c2_jd_a;
  int32_T c2_ub_c;
  int32_T c2_id_b;
  int32_T c2_jd_b;
  int32_T c2_vb_c;
  int32_T c2_kd_b;
  int32_T c2_ld_b;
  int32_T c2_wb_c;
  int32_T c2_kd_a;
  int32_T c2_xb_c;
  int32_T c2_md_b;
  int32_T c2_nd_b;
  int32_T c2_yb_c;
  int32_T c2_od_b;
  int32_T c2_pd_b;
  int32_T c2_ld_a;
  real_T c2_d8;
  boolean_T guard1 = FALSE;
  boolean_T guard2 = FALSE;
  boolean_T guard3 = FALSE;
  boolean_T guard4 = FALSE;
  boolean_T exitg1;
  boolean_T exitg2;
  boolean_T exitg3;
  boolean_T exitg4;
  boolean_T exitg5;
  boolean_T guard11 = FALSE;
  for (c2_i347 = 0; c2_i347 < 72; c2_i347++) {
    c2_b_A[c2_i347] = c2_A[c2_i347];
  }

  c2_c_eml_scalar_eg(chartInstance);
  for (c2_i348 = 0; c2_i348 < 6; c2_i348++) {
    c2_s[c2_i348] = 0.0;
  }

  for (c2_i349 = 0; c2_i349 < 6; c2_i349++) {
    c2_e[c2_i349] = 0.0;
  }

  for (c2_i350 = 0; c2_i350 < 12; c2_i350++) {
    c2_work[c2_i350] = 0.0;
  }

  for (c2_i351 = 0; c2_i351 < 72; c2_i351++) {
    c2_U[c2_i351] = 0.0;
  }

  for (c2_i352 = 0; c2_i352 < 36; c2_i352++) {
    c2_Vf[c2_i352] = 0.0;
  }

  c2_check_forloop_overflow_error(chartInstance);
  for (c2_q = 1; c2_q < 7; c2_q++) {
    c2_b_q = c2_q;
    c2_a = c2_b_q + 1;
    c2_qp1 = c2_a;
    c2_b_a = c2_b_q - 1;
    c2_qm1 = c2_b_a;
    c2_b = c2_qm1;
    c2_c = 12 * c2_b;
    c2_c_a = c2_b_q;
    c2_b_b = c2_c;
    c2_qq = c2_c_a + c2_b_b;
    c2_c_b = c2_b_q;
    c2_nmq = 12 - c2_c_b;
    c2_d_a = c2_nmq + 1;
    c2_nmqp1 = c2_d_a;
    if (c2_b_q <= 6) {
      for (c2_i353 = 0; c2_i353 < 72; c2_i353++) {
        c2_c_A[c2_i353] = c2_b_A[c2_i353];
      }

      c2_nrm = c2_eml_xnrm2(chartInstance, c2_nmqp1, c2_c_A, c2_qq);
      if (c2_nrm > 0.0) {
        c2_absx = c2_nrm;
        c2_d = c2_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c2_qq), 1, 72, 1, 0) - 1];
        if (c2_d < 0.0) {
          c2_y = -c2_absx;
        } else {
          c2_y = c2_absx;
        }

        c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_b_q), 1, 6, 1, 0) - 1] = c2_y;
        c2_d1 = c2_eml_div(chartInstance, 1.0, c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK(
          "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_q), 1, 6, 1, 0) - 1]);
        c2_g_eml_xscal(chartInstance, c2_nmqp1, c2_d1, c2_b_A, c2_qq);
        c2_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_qq), 1, 72, 1, 0) - 1] = c2_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK
          ("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_qq), 1, 72, 1, 0) - 1]
          + 1.0;
        c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_b_q), 1, 6, 1, 0) - 1] = -c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK(
          "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_q), 1, 6, 1, 0) - 1];
      } else {
        c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_b_q), 1, 6, 1, 0) - 1] = 0.0;
      }
    }

    c2_b_qp1 = c2_qp1;
    c2_overflow = FALSE;
    c2_b_check_forloop_overflow_error(chartInstance, c2_overflow);
    for (c2_jj = c2_b_qp1; c2_jj < 7; c2_jj++) {
      c2_b_jj = c2_jj;
      c2_e_a = c2_b_jj - 1;
      c2_b_c = c2_e_a;
      c2_d_b = c2_b_c;
      c2_c_c = 12 * c2_d_b;
      c2_f_a = c2_b_q;
      c2_e_b = c2_c_c;
      c2_qjj = c2_f_a + c2_e_b;
      if (c2_b_q <= 6) {
        if (c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c2_b_q), 1, 6, 1, 0) - 1] != 0.0) {
          for (c2_i354 = 0; c2_i354 < 72; c2_i354++) {
            c2_d_A[c2_i354] = c2_b_A[c2_i354];
          }

          for (c2_i355 = 0; c2_i355 < 72; c2_i355++) {
            c2_e_A[c2_i355] = c2_b_A[c2_i355];
          }

          c2_t = c2_eml_xdotc(chartInstance, c2_nmqp1, c2_d_A, c2_qq, c2_e_A,
                              c2_qjj);
          c2_t = -c2_eml_div(chartInstance, c2_t, c2_b_A
                             [(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c2_b_q), 1, 12, 1, 0) + 12 *
                               (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c2_b_q), 1, 6, 2, 0) - 1)) - 1]);
          c2_i_eml_xaxpy(chartInstance, c2_nmqp1, c2_t, c2_qq, c2_b_A, c2_qjj);
        }
      }

      c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c2_b_jj), 1, 6, 1, 0) - 1] = c2_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK(
        "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_qjj), 1, 72, 1, 0) - 1];
    }

    if (c2_b_q <= 6) {
      c2_c_q = c2_b_q;
      c2_b_overflow = FALSE;
      c2_b_check_forloop_overflow_error(chartInstance, c2_b_overflow);
      for (c2_ii = c2_c_q; c2_ii < 13; c2_ii++) {
        c2_b_ii = c2_ii;
        c2_U[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                (real_T)c2_b_ii), 1, 12, 1, 0) + 12 *
              (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                 (real_T)c2_b_q), 1, 6, 2, 0) - 1)) - 1] = c2_b_A
          [(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c2_b_ii), 1, 12, 1, 0) + 12 * (_SFD_EML_ARRAY_BOUNDS_CHECK
             ("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_q), 1, 6, 2, 0) -
             1)) - 1];
      }
    }

    if (c2_b_q <= 4) {
      c2_f_b = c2_b_q;
      c2_pmq = 6 - c2_f_b;
      for (c2_i356 = 0; c2_i356 < 6; c2_i356++) {
        c2_b_e[c2_i356] = c2_e[c2_i356];
      }

      c2_nrm = c2_b_eml_xnrm2(chartInstance, c2_pmq, c2_b_e, c2_qp1);
      if (c2_nrm == 0.0) {
        c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_b_q), 1, 6, 1, 0) - 1] = 0.0;
      } else {
        c2_b_absx = c2_nrm;
        c2_b_d = c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c2_qp1), 1, 6, 1, 0) - 1];
        if (c2_b_d < 0.0) {
          c2_b_y = -c2_b_absx;
        } else {
          c2_b_y = c2_b_absx;
        }

        c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_b_q), 1, 6, 1, 0) - 1] = c2_b_y;
        c2_d2 = c2_eml_div(chartInstance, 1.0, c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK(
          "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_q), 1, 6, 1, 0) - 1]);
        c2_h_eml_xscal(chartInstance, c2_pmq, c2_d2, c2_e, c2_qp1);
        c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_qp1), 1, 6, 1, 0) - 1] = c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK(
          "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_qp1), 1, 6, 1, 0) - 1]
          + 1.0;
      }

      c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c2_b_q), 1, 6, 1, 0) - 1] = -c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("",
        (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_q), 1, 6, 1, 0) - 1];
      if (c2_qp1 <= 12) {
        if (c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c2_b_q), 1, 6, 1, 0) - 1] != 0.0) {
          c2_c_qp1 = c2_qp1;
          c2_c_overflow = FALSE;
          c2_b_check_forloop_overflow_error(chartInstance, c2_c_overflow);
          for (c2_c_ii = c2_c_qp1; c2_c_ii < 13; c2_c_ii++) {
            c2_b_ii = c2_c_ii;
            c2_work[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
              "", (real_T)c2_b_ii), 1, 12, 1, 0) - 1] = 0.0;
          }

          c2_d_qp1 = c2_qp1;
          c2_d_overflow = FALSE;
          c2_b_check_forloop_overflow_error(chartInstance, c2_d_overflow);
          for (c2_c_jj = c2_d_qp1; c2_c_jj < 7; c2_c_jj++) {
            c2_b_jj = c2_c_jj;
            c2_g_a = c2_b_jj - 1;
            c2_d_c = c2_g_a;
            c2_g_b = c2_d_c;
            c2_e_c = 12 * c2_g_b;
            c2_h_a = c2_qp1;
            c2_h_b = c2_e_c;
            c2_qp1jj = c2_h_a + c2_h_b;
            for (c2_i357 = 0; c2_i357 < 72; c2_i357++) {
              c2_f_A[c2_i357] = c2_b_A[c2_i357];
            }

            c2_j_eml_xaxpy(chartInstance, c2_nmq,
                           c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
              _SFD_INTEGER_CHECK("", (real_T)c2_b_jj), 1, 6, 1, 0) - 1], c2_f_A,
                           c2_qp1jj, c2_work, c2_qp1);
          }

          c2_e_qp1 = c2_qp1;
          c2_e_overflow = FALSE;
          c2_b_check_forloop_overflow_error(chartInstance, c2_e_overflow);
          for (c2_d_jj = c2_e_qp1; c2_d_jj < 7; c2_d_jj++) {
            c2_b_jj = c2_d_jj;
            c2_t = c2_eml_div(chartInstance, -c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK(
              "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_jj), 1, 6, 1, 0)
                              - 1], c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("",
              (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_qp1), 1, 6, 1, 0) - 1]);
            c2_i_a = c2_b_jj - 1;
            c2_f_c = c2_i_a;
            c2_i_b = c2_f_c;
            c2_g_c = 12 * c2_i_b;
            c2_j_a = c2_qp1;
            c2_j_b = c2_g_c;
            c2_qp1jj = c2_j_a + c2_j_b;
            for (c2_i358 = 0; c2_i358 < 12; c2_i358++) {
              c2_b_work[c2_i358] = c2_work[c2_i358];
            }

            c2_k_eml_xaxpy(chartInstance, c2_nmq, c2_t, c2_b_work, c2_qp1,
                           c2_b_A, c2_qp1jj);
          }
        }
      }

      c2_f_qp1 = c2_qp1;
      c2_f_overflow = FALSE;
      c2_b_check_forloop_overflow_error(chartInstance, c2_f_overflow);
      for (c2_d_ii = c2_f_qp1; c2_d_ii < 7; c2_d_ii++) {
        c2_b_ii = c2_d_ii;
        c2_Vf[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                 (real_T)c2_b_ii), 1, 6, 1, 0) + 6 *
               (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                  (real_T)c2_b_q), 1, 6, 2, 0) - 1)) - 1] =
          c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_b_ii), 1, 6, 1, 0) - 1];
      }
    }
  }

  c2_m = 6;
  c2_e[4] = c2_b_A[64];
  c2_e[5] = 0.0;
  c2_check_forloop_overflow_error(chartInstance);
  for (c2_d_q = 6; c2_d_q > 0; c2_d_q--) {
    c2_b_q = c2_d_q;
    c2_k_a = c2_b_q + 1;
    c2_qp1 = c2_k_a;
    c2_k_b = c2_b_q;
    c2_nmq = 12 - c2_k_b;
    c2_l_a = c2_nmq + 1;
    c2_nmqp1 = c2_l_a;
    c2_m_a = c2_b_q - 1;
    c2_h_c = c2_m_a;
    c2_l_b = c2_h_c;
    c2_i_c = 12 * c2_l_b;
    c2_n_a = c2_b_q;
    c2_m_b = c2_i_c;
    c2_qq = c2_n_a + c2_m_b;
    if (c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_b_q), 1, 6, 1, 0) - 1] != 0.0) {
      c2_g_qp1 = c2_qp1;
      c2_g_overflow = FALSE;
      c2_b_check_forloop_overflow_error(chartInstance, c2_g_overflow);
      for (c2_e_jj = c2_g_qp1; c2_e_jj < 7; c2_e_jj++) {
        c2_b_jj = c2_e_jj;
        c2_o_a = c2_b_jj - 1;
        c2_j_c = c2_o_a;
        c2_n_b = c2_j_c;
        c2_k_c = 12 * c2_n_b;
        c2_p_a = c2_b_q;
        c2_o_b = c2_k_c;
        c2_qjj = c2_p_a + c2_o_b;
        for (c2_i359 = 0; c2_i359 < 72; c2_i359++) {
          c2_b_U[c2_i359] = c2_U[c2_i359];
        }

        for (c2_i360 = 0; c2_i360 < 72; c2_i360++) {
          c2_c_U[c2_i360] = c2_U[c2_i360];
        }

        c2_t = c2_eml_xdotc(chartInstance, c2_nmqp1, c2_b_U, c2_qq, c2_c_U,
                            c2_qjj);
        c2_t = -c2_eml_div(chartInstance, c2_t, c2_U[_SFD_EML_ARRAY_BOUNDS_CHECK
                           ("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_qq),
                            1, 72, 1, 0) - 1]);
        c2_i_eml_xaxpy(chartInstance, c2_nmqp1, c2_t, c2_qq, c2_U, c2_qjj);
      }

      c2_e_q = c2_b_q;
      c2_h_overflow = FALSE;
      c2_b_check_forloop_overflow_error(chartInstance, c2_h_overflow);
      for (c2_e_ii = c2_e_q; c2_e_ii < 13; c2_e_ii++) {
        c2_b_ii = c2_e_ii;
        c2_U[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                (real_T)c2_b_ii), 1, 12, 1, 0) + 12 *
              (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                 (real_T)c2_b_q), 1, 6, 2, 0) - 1)) - 1] = -c2_U
          [(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c2_b_ii), 1, 12, 1, 0) + 12 * (_SFD_EML_ARRAY_BOUNDS_CHECK
             ("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_q), 1, 6, 2, 0) -
             1)) - 1];
      }

      c2_U[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c2_qq), 1, 72, 1, 0) - 1] = c2_U[_SFD_EML_ARRAY_BOUNDS_CHECK("",
        (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_qq), 1, 72, 1, 0) - 1] + 1.0;
      c2_q_a = c2_b_q - 1;
      c2_i361 = c2_q_a;
      c2_p_b = c2_i361;
      c2_q_b = c2_p_b;
      if (1 > c2_q_b) {
        c2_i_overflow = FALSE;
      } else {
        c2_i_overflow = (c2_q_b > 2147483646);
      }

      c2_b_check_forloop_overflow_error(chartInstance, c2_i_overflow);
      for (c2_f_ii = 1; c2_f_ii <= c2_i361; c2_f_ii++) {
        c2_b_ii = c2_f_ii;
        c2_U[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                (real_T)c2_b_ii), 1, 12, 1, 0) + 12 *
              (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                 (real_T)c2_b_q), 1, 6, 2, 0) - 1)) - 1] = 0.0;
      }
    } else {
      c2_b_check_forloop_overflow_error(chartInstance, FALSE);
      for (c2_g_ii = 1; c2_g_ii < 13; c2_g_ii++) {
        c2_b_ii = c2_g_ii;
        c2_U[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                (real_T)c2_b_ii), 1, 12, 1, 0) + 12 *
              (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                 (real_T)c2_b_q), 1, 6, 2, 0) - 1)) - 1] = 0.0;
      }

      c2_U[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c2_qq), 1, 72, 1, 0) - 1] = 1.0;
    }
  }

  c2_check_forloop_overflow_error(chartInstance);
  for (c2_f_q = 6; c2_f_q > 0; c2_f_q--) {
    c2_b_q = c2_f_q;
    if (c2_b_q <= 4) {
      if (c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_b_q), 1, 6, 1, 0) - 1] != 0.0) {
        c2_r_a = c2_b_q + 1;
        c2_qp1 = c2_r_a;
        c2_r_b = c2_b_q;
        c2_pmq = 6 - c2_r_b;
        c2_s_a = c2_b_q - 1;
        c2_l_c = c2_s_a;
        c2_s_b = c2_l_c;
        c2_m_c = 6 * c2_s_b;
        c2_t_a = c2_qp1;
        c2_t_b = c2_m_c;
        c2_qp1q = c2_t_a + c2_t_b;
        c2_h_qp1 = c2_qp1;
        c2_j_overflow = FALSE;
        c2_b_check_forloop_overflow_error(chartInstance, c2_j_overflow);
        for (c2_f_jj = c2_h_qp1; c2_f_jj < 7; c2_f_jj++) {
          c2_b_jj = c2_f_jj;
          c2_u_a = c2_b_jj - 1;
          c2_n_c = c2_u_a;
          c2_u_b = c2_n_c;
          c2_o_c = 6 * c2_u_b;
          c2_v_a = c2_qp1;
          c2_v_b = c2_o_c;
          c2_qp1jj = c2_v_a + c2_v_b;
          for (c2_i362 = 0; c2_i362 < 36; c2_i362++) {
            c2_b_Vf[c2_i362] = c2_Vf[c2_i362];
          }

          for (c2_i363 = 0; c2_i363 < 36; c2_i363++) {
            c2_c_Vf[c2_i363] = c2_Vf[c2_i363];
          }

          c2_t = c2_b_eml_xdotc(chartInstance, c2_pmq, c2_b_Vf, c2_qp1q, c2_c_Vf,
                                c2_qp1jj);
          c2_t = -c2_eml_div(chartInstance, c2_t,
                             c2_Vf[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c2_qp1q), 1, 36, 1, 0) - 1]);
          c2_l_eml_xaxpy(chartInstance, c2_pmq, c2_t, c2_qp1q, c2_Vf, c2_qp1jj);
        }
      }
    }

    c2_check_forloop_overflow_error(chartInstance);
    for (c2_h_ii = 1; c2_h_ii < 7; c2_h_ii++) {
      c2_b_ii = c2_h_ii;
      c2_Vf[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
               (real_T)c2_b_ii), 1, 6, 1, 0) + 6 * (_SFD_EML_ARRAY_BOUNDS_CHECK(
               "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_q), 1, 6, 2, 0)
              - 1)) - 1] = 0.0;
    }

    c2_Vf[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
             (real_T)c2_b_q), 1, 6, 1, 0) + 6 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
             (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_q), 1, 6, 2, 0) - 1))
      - 1] = 1.0;
  }

  c2_check_forloop_overflow_error(chartInstance);
  for (c2_g_q = 1; c2_g_q < 7; c2_g_q++) {
    c2_b_q = c2_g_q;
    if (c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_b_q), 1, 6, 1, 0) - 1] != 0.0) {
      c2_rt = c2_abs(chartInstance, c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
        (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_q), 1, 6, 1, 0) - 1]);
      c2_r = c2_eml_div(chartInstance, c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
        (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_q), 1, 6, 1, 0) - 1], c2_rt);
      c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c2_b_q), 1, 6, 1, 0) - 1] = c2_rt;
      if (c2_b_q < 6) {
        c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_b_q), 1, 6, 1, 0) - 1] = c2_eml_div(chartInstance,
          c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_b_q), 1, 6, 1, 0) - 1], c2_r);
      }

      if (c2_b_q <= 12) {
        c2_w_a = c2_b_q - 1;
        c2_p_c = c2_w_a;
        c2_w_b = c2_p_c;
        c2_q_c = 12 * c2_w_b;
        c2_x_b = c2_q_c + 1;
        c2_colq = c2_x_b;
        c2_i364 = 12;
        c2_g_eml_xscal(chartInstance, c2_i364, c2_r, c2_U, c2_colq);
      }
    }

    if (c2_b_q < 6) {
      if (c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_b_q), 1, 6, 1, 0) - 1] != 0.0) {
        c2_rt = c2_abs(chartInstance, c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_q), 1, 6, 1, 0) - 1]);
        c2_r = c2_eml_div(chartInstance, c2_rt, c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK
                          ("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_q),
                           1, 6, 1, 0) - 1]);
        c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_b_q), 1, 6, 1, 0) - 1] = c2_rt;
        c2_x_a = c2_b_q + 1;
        c2_r_c = c2_x_a;
        c2_y_a = c2_b_q + 1;
        c2_s_c = c2_y_a;
        c2_ab_a = c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c2_s_c), 1, 6, 1, 0) - 1];
        c2_y_b = c2_r;
        c2_c_y = c2_ab_a * c2_y_b;
        c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_r_c), 1, 6, 1, 0) - 1] = c2_c_y;
        c2_ab_b = c2_b_q;
        c2_t_c = 6 * c2_ab_b;
        c2_bb_b = c2_t_c + 1;
        c2_colqp1 = c2_bb_b;
        c2_i_eml_xscal(chartInstance, c2_r, c2_Vf, c2_colqp1);
      }
    }
  }

  c2_iter = 0.0;
  c2_realmin(chartInstance);
  c2_eps(chartInstance);
  c2_tiny = c2_eml_div(chartInstance, 2.2250738585072014E-308,
                       2.2204460492503131E-16);
  c2_snorm = 0.0;
  c2_check_forloop_overflow_error(chartInstance);
  for (c2_i_ii = 1; c2_i_ii < 7; c2_i_ii++) {
    c2_b_ii = c2_i_ii;
    c2_varargin_1 = c2_abs(chartInstance, c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
      (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_ii), 1, 6, 1, 0) - 1]);
    c2_varargin_2 = c2_abs(chartInstance, c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("",
      (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_ii), 1, 6, 1, 0) - 1]);
    c2_b_varargin_2 = c2_varargin_1;
    c2_varargin_3 = c2_varargin_2;
    c2_x = c2_b_varargin_2;
    c2_d_y = c2_varargin_3;
    c2_b_x = c2_x;
    c2_e_y = c2_d_y;
    c2_eml_scalar_eg(chartInstance);
    c2_xk = c2_b_x;
    c2_yk = c2_e_y;
    c2_c_x = c2_xk;
    c2_f_y = c2_yk;
    c2_eml_scalar_eg(chartInstance);
    c2_maxval = muDoubleScalarMax(c2_c_x, c2_f_y);
    c2_b_varargin_1 = c2_snorm;
    c2_c_varargin_2 = c2_maxval;
    c2_d_varargin_2 = c2_b_varargin_1;
    c2_b_varargin_3 = c2_c_varargin_2;
    c2_d_x = c2_d_varargin_2;
    c2_g_y = c2_b_varargin_3;
    c2_e_x = c2_d_x;
    c2_h_y = c2_g_y;
    c2_eml_scalar_eg(chartInstance);
    c2_b_xk = c2_e_x;
    c2_b_yk = c2_h_y;
    c2_f_x = c2_b_xk;
    c2_i_y = c2_b_yk;
    c2_eml_scalar_eg(chartInstance);
    c2_snorm = muDoubleScalarMax(c2_f_x, c2_i_y);
  }

  exitg1 = FALSE;
  while ((exitg1 == FALSE) && (c2_m > 0)) {
    if (c2_iter >= 75.0) {
      c2_b_eml_error(chartInstance);
      exitg1 = TRUE;
    } else {
      c2_bb_a = c2_m - 1;
      c2_b_q = c2_bb_a;
      c2_cb_a = c2_m - 1;
      c2_i365 = c2_cb_a;
      c2_k_overflow = FALSE;
      c2_b_check_forloop_overflow_error(chartInstance, c2_k_overflow);
      c2_j_ii = c2_i365;
      guard3 = FALSE;
      guard4 = FALSE;
      exitg5 = FALSE;
      while ((exitg5 == FALSE) && (c2_j_ii > -1)) {
        c2_b_ii = c2_j_ii;
        c2_b_q = c2_b_ii;
        if (c2_b_ii == 0) {
          exitg5 = TRUE;
        } else {
          c2_db_a = c2_b_ii + 1;
          c2_u_c = c2_db_a;
          c2_test0 = c2_abs(chartInstance, c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
                             (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_ii), 1,
            6, 1, 0) - 1]) + c2_abs(chartInstance,
            c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                                      (real_T)c2_u_c), 1, 6, 1, 0) - 1]);
          c2_ztest0 = c2_abs(chartInstance, c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("",
                              (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_ii),
            1, 6, 1, 0) - 1]);
          c2_eps(chartInstance);
          c2_cb_b = c2_test0;
          c2_j_y = 2.2204460492503131E-16 * c2_cb_b;
          if (c2_ztest0 <= c2_j_y) {
            guard4 = TRUE;
            exitg5 = TRUE;
          } else if (c2_ztest0 <= c2_tiny) {
            guard4 = TRUE;
            exitg5 = TRUE;
          } else {
            guard11 = FALSE;
            if (c2_iter > 20.0) {
              c2_eps(chartInstance);
              c2_db_b = c2_snorm;
              c2_k_y = 2.2204460492503131E-16 * c2_db_b;
              if (c2_ztest0 <= c2_k_y) {
                guard3 = TRUE;
                exitg5 = TRUE;
              } else {
                guard11 = TRUE;
              }
            } else {
              guard11 = TRUE;
            }

            if (guard11 == TRUE) {
              c2_j_ii--;
              guard3 = FALSE;
              guard4 = FALSE;
            }
          }
        }
      }

      if (guard4 == TRUE) {
        guard3 = TRUE;
      }

      if (guard3 == TRUE) {
        c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_b_ii), 1, 6, 1, 0) - 1] = 0.0;
      }

      c2_eb_a = c2_m - 1;
      c2_v_c = c2_eb_a;
      if (c2_b_q == c2_v_c) {
        c2_kase = 4.0;
      } else {
        c2_qs = c2_m;
        c2_b_m = c2_m;
        c2_h_q = c2_b_q;
        c2_fb_a = c2_b_m;
        c2_eb_b = c2_h_q;
        c2_gb_a = c2_fb_a;
        c2_fb_b = c2_eb_b;
        if (c2_gb_a < c2_fb_b) {
          c2_l_overflow = FALSE;
        } else {
          c2_l_overflow = (c2_fb_b < -2147483647);
        }

        c2_b_check_forloop_overflow_error(chartInstance, c2_l_overflow);
        c2_k_ii = c2_b_m;
        guard2 = FALSE;
        exitg4 = FALSE;
        while ((exitg4 == FALSE) && (c2_k_ii >= c2_h_q)) {
          c2_b_ii = c2_k_ii;
          c2_qs = c2_b_ii;
          if (c2_b_ii == c2_b_q) {
            exitg4 = TRUE;
          } else {
            c2_test = 0.0;
            if (c2_b_ii < c2_m) {
              c2_test = c2_abs(chartInstance, c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK(
                "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_ii), 1, 6, 1, 0)
                               - 1]);
            }

            c2_hb_a = c2_b_q + 1;
            c2_w_c = c2_hb_a;
            if (c2_b_ii > c2_w_c) {
              c2_ib_a = c2_b_ii - 1;
              c2_x_c = c2_ib_a;
              c2_test += c2_abs(chartInstance, c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK(
                "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_x_c), 1, 6, 1, 0)
                                - 1]);
            }

            c2_ztest = c2_abs(chartInstance, c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
                               (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_ii),
              1, 6, 1, 0) - 1]);
            c2_eps(chartInstance);
            c2_gb_b = c2_test;
            c2_l_y = 2.2204460492503131E-16 * c2_gb_b;
            if (c2_ztest <= c2_l_y) {
              guard2 = TRUE;
              exitg4 = TRUE;
            } else if (c2_ztest <= c2_tiny) {
              guard2 = TRUE;
              exitg4 = TRUE;
            } else {
              c2_k_ii--;
              guard2 = FALSE;
            }
          }
        }

        if (guard2 == TRUE) {
          c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_b_ii), 1, 6, 1, 0) - 1] = 0.0;
        }

        if (c2_qs == c2_b_q) {
          c2_kase = 3.0;
        } else if (c2_qs == c2_m) {
          c2_kase = 1.0;
        } else {
          c2_kase = 2.0;
          c2_b_q = c2_qs;
        }
      }

      c2_jb_a = c2_b_q + 1;
      c2_b_q = c2_jb_a;
      switch ((int32_T)_SFD_INTEGER_CHECK("", c2_kase)) {
       case 1:
        c2_kb_a = c2_m - 1;
        c2_y_c = c2_kb_a;
        c2_f = c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
          "", (real_T)c2_y_c), 1, 6, 1, 0) - 1];
        c2_lb_a = c2_m - 1;
        c2_ab_c = c2_lb_a;
        c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_ab_c), 1, 6, 1, 0) - 1] = 0.0;
        c2_mb_a = c2_m - 1;
        c2_i366 = c2_mb_a;
        c2_i_q = c2_b_q;
        c2_nb_a = c2_i366;
        c2_hb_b = c2_i_q;
        c2_ob_a = c2_nb_a;
        c2_ib_b = c2_hb_b;
        if (c2_ob_a < c2_ib_b) {
          c2_m_overflow = FALSE;
        } else {
          c2_m_overflow = (c2_ib_b < -2147483647);
        }

        c2_b_check_forloop_overflow_error(chartInstance, c2_m_overflow);
        for (c2_k = c2_i366; c2_k >= c2_i_q; c2_k--) {
          c2_b_k = c2_k;
          c2_t1 = c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c2_b_k), 1, 6, 1, 0) - 1];
          c2_b_t1 = c2_t1;
          c2_b_f = c2_f;
          c2_b_eml_xrotg(chartInstance, &c2_b_t1, &c2_b_f, &c2_cs, &c2_sn);
          c2_t1 = c2_b_t1;
          c2_f = c2_b_f;
          c2_b_cs = c2_cs;
          c2_b_sn = c2_sn;
          c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_b_k), 1, 6, 1, 0) - 1] = c2_t1;
          if (c2_b_k > c2_b_q) {
            c2_pb_a = c2_b_k - 1;
            c2_km1 = c2_pb_a;
            c2_qb_a = -c2_b_sn;
            c2_jb_b = c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
              _SFD_INTEGER_CHECK("", (real_T)c2_km1), 1, 6, 1, 0) - 1];
            c2_f = c2_qb_a * c2_jb_b;
            c2_rb_a = c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
              _SFD_INTEGER_CHECK("", (real_T)c2_km1), 1, 6, 1, 0) - 1];
            c2_kb_b = c2_b_cs;
            c2_m_y = c2_rb_a * c2_kb_b;
            c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c2_km1), 1, 6, 1, 0) - 1] = c2_m_y;
          }

          c2_sb_a = c2_b_k - 1;
          c2_bb_c = c2_sb_a;
          c2_lb_b = c2_bb_c;
          c2_cb_c = 6 * c2_lb_b;
          c2_mb_b = c2_cb_c + 1;
          c2_colk = c2_mb_b;
          c2_tb_a = c2_m - 1;
          c2_db_c = c2_tb_a;
          c2_nb_b = c2_db_c;
          c2_eb_c = 6 * c2_nb_b;
          c2_ob_b = c2_eb_c + 1;
          c2_colm = c2_ob_b;
          c2_e_eml_xrot(chartInstance, c2_Vf, c2_colk, c2_colm, c2_b_cs, c2_b_sn);
        }
        break;

       case 2:
        c2_ub_a = c2_b_q - 1;
        c2_qm1 = c2_ub_a;
        c2_f = c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
          "", (real_T)c2_qm1), 1, 6, 1, 0) - 1];
        c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_qm1), 1, 6, 1, 0) - 1] = 0.0;
        c2_j_q = c2_b_q;
        c2_c_m = c2_m;
        c2_vb_a = c2_j_q;
        c2_pb_b = c2_c_m;
        c2_wb_a = c2_vb_a;
        c2_qb_b = c2_pb_b;
        if (c2_wb_a > c2_qb_b) {
          c2_n_overflow = FALSE;
        } else {
          c2_n_overflow = (c2_qb_b > 2147483646);
        }

        c2_b_check_forloop_overflow_error(chartInstance, c2_n_overflow);
        for (c2_c_k = c2_j_q; c2_c_k <= c2_c_m; c2_c_k++) {
          c2_b_k = c2_c_k;
          c2_t1 = c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c2_b_k), 1, 6, 1, 0) - 1];
          c2_c_t1 = c2_t1;
          c2_unusedU0 = c2_f;
          c2_b_eml_xrotg(chartInstance, &c2_c_t1, &c2_unusedU0, &c2_c_cs,
                         &c2_c_sn);
          c2_t1 = c2_c_t1;
          c2_b_cs = c2_c_cs;
          c2_b_sn = c2_c_sn;
          c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_b_k), 1, 6, 1, 0) - 1] = c2_t1;
          c2_xb_a = -c2_b_sn;
          c2_rb_b = c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c2_b_k), 1, 6, 1, 0) - 1];
          c2_f = c2_xb_a * c2_rb_b;
          c2_yb_a = c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c2_b_k), 1, 6, 1, 0) - 1];
          c2_sb_b = c2_b_cs;
          c2_n_y = c2_yb_a * c2_sb_b;
          c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_b_k), 1, 6, 1, 0) - 1] = c2_n_y;
          c2_ac_a = c2_b_k - 1;
          c2_fb_c = c2_ac_a;
          c2_tb_b = c2_fb_c;
          c2_gb_c = 12 * c2_tb_b;
          c2_ub_b = c2_gb_c + 1;
          c2_colk = c2_ub_b;
          c2_bc_a = c2_qm1 - 1;
          c2_hb_c = c2_bc_a;
          c2_vb_b = c2_hb_c;
          c2_ib_c = 12 * c2_vb_b;
          c2_wb_b = c2_ib_c + 1;
          c2_colqm1 = c2_wb_b;
          c2_f_eml_xrot(chartInstance, c2_U, c2_colk, c2_colqm1, c2_b_cs,
                        c2_b_sn);
        }
        break;

       case 3:
        c2_cc_a = c2_m - 1;
        c2_mm1 = c2_cc_a;
        c2_d3 = c2_abs(chartInstance, c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_m), 1, 6, 1, 0) - 1]);
        c2_d4 = c2_abs(chartInstance, c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_mm1), 1, 6, 1, 0) - 1]);
        c2_d5 = c2_abs(chartInstance, c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_mm1), 1, 6, 1, 0) - 1]);
        c2_d6 = c2_abs(chartInstance, c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_q), 1, 6, 1, 0) - 1]);
        c2_d7 = c2_abs(chartInstance, c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_q), 1, 6, 1, 0) - 1]);
        c2_c_varargin_1[0] = c2_d3;
        c2_c_varargin_1[1] = c2_d4;
        c2_c_varargin_1[2] = c2_d5;
        c2_c_varargin_1[3] = c2_d6;
        c2_c_varargin_1[4] = c2_d7;
        c2_ixstart = 1;
        c2_mtmp = c2_c_varargin_1[0];
        c2_g_x = c2_mtmp;
        c2_xb_b = muDoubleScalarIsNaN(c2_g_x);
        if (c2_xb_b) {
          c2_b_check_forloop_overflow_error(chartInstance, FALSE);
          c2_ix = 2;
          exitg2 = FALSE;
          while ((exitg2 == FALSE) && (c2_ix < 6)) {
            c2_b_ix = c2_ix;
            c2_ixstart = c2_b_ix;
            c2_h_x = c2_c_varargin_1[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
              _SFD_INTEGER_CHECK("", (real_T)c2_b_ix), 1, 5, 1, 0) - 1];
            c2_yb_b = muDoubleScalarIsNaN(c2_h_x);
            if (!c2_yb_b) {
              c2_mtmp = c2_c_varargin_1[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
                _SFD_INTEGER_CHECK("", (real_T)c2_b_ix), 1, 5, 1, 0) - 1];
              exitg2 = TRUE;
            } else {
              c2_ix++;
            }
          }
        }

        if (c2_ixstart < 5) {
          c2_dc_a = c2_ixstart + 1;
          c2_i367 = c2_dc_a;
          c2_o_overflow = FALSE;
          c2_b_check_forloop_overflow_error(chartInstance, c2_o_overflow);
          for (c2_c_ix = c2_i367; c2_c_ix < 6; c2_c_ix++) {
            c2_b_ix = c2_c_ix;
            c2_ec_a = c2_c_varargin_1[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
              _SFD_INTEGER_CHECK("", (real_T)c2_b_ix), 1, 5, 1, 0) - 1];
            c2_ac_b = c2_mtmp;
            c2_p = (c2_ec_a > c2_ac_b);
            if (c2_p) {
              c2_mtmp = c2_c_varargin_1[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
                _SFD_INTEGER_CHECK("", (real_T)c2_b_ix), 1, 5, 1, 0) - 1];
            }
          }
        }

        c2_b_mtmp = c2_mtmp;
        c2_scale = c2_b_mtmp;
        c2_sm = c2_eml_div(chartInstance, c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_m), 1, 6, 1, 0) - 1],
                           c2_scale);
        c2_smm1 = c2_eml_div(chartInstance, c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
                              (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_mm1), 1,
          6, 1, 0) - 1], c2_scale);
        c2_emm1 = c2_eml_div(chartInstance, c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("",
                              (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_mm1), 1,
          6, 1, 0) - 1], c2_scale);
        c2_sqds = c2_eml_div(chartInstance, c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
                              (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_q), 1,
          6, 1, 0) - 1], c2_scale);
        c2_eqds = c2_eml_div(chartInstance, c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("",
                              (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_q), 1,
          6, 1, 0) - 1], c2_scale);
        c2_fc_a = c2_smm1 + c2_sm;
        c2_bc_b = c2_smm1 - c2_sm;
        c2_o_y = c2_fc_a * c2_bc_b;
        c2_gc_a = c2_emm1;
        c2_cc_b = c2_emm1;
        c2_p_y = c2_gc_a * c2_cc_b;
        c2_dc_b = c2_eml_div(chartInstance, c2_o_y + c2_p_y, 2.0);
        c2_hc_a = c2_sm;
        c2_ec_b = c2_emm1;
        c2_jb_c = c2_hc_a * c2_ec_b;
        c2_ic_a = c2_jb_c;
        c2_fc_b = c2_jb_c;
        c2_jb_c = c2_ic_a * c2_fc_b;
        c2_shift = 0.0;
        guard1 = FALSE;
        if (c2_dc_b != 0.0) {
          guard1 = TRUE;
        } else {
          if (c2_jb_c != 0.0) {
            guard1 = TRUE;
          }
        }

        if (guard1 == TRUE) {
          c2_jc_a = c2_dc_b;
          c2_gc_b = c2_dc_b;
          c2_q_y = c2_jc_a * c2_gc_b;
          c2_shift = c2_q_y + c2_jb_c;
          c2_b_sqrt(chartInstance, &c2_shift);
          if (c2_dc_b < 0.0) {
            c2_shift = -c2_shift;
          }

          c2_shift = c2_eml_div(chartInstance, c2_jb_c, c2_dc_b + c2_shift);
        }

        c2_kc_a = c2_sqds + c2_sm;
        c2_hc_b = c2_sqds - c2_sm;
        c2_r_y = c2_kc_a * c2_hc_b;
        c2_f = c2_r_y + c2_shift;
        c2_lc_a = c2_sqds;
        c2_ic_b = c2_eqds;
        c2_g = c2_lc_a * c2_ic_b;
        c2_k_q = c2_b_q;
        c2_b_mm1 = c2_mm1;
        c2_mc_a = c2_k_q;
        c2_jc_b = c2_b_mm1;
        c2_nc_a = c2_mc_a;
        c2_kc_b = c2_jc_b;
        if (c2_nc_a > c2_kc_b) {
          c2_p_overflow = FALSE;
        } else {
          c2_p_overflow = (c2_kc_b > 2147483646);
        }

        c2_b_check_forloop_overflow_error(chartInstance, c2_p_overflow);
        for (c2_d_k = c2_k_q; c2_d_k <= c2_b_mm1; c2_d_k++) {
          c2_b_k = c2_d_k;
          c2_oc_a = c2_b_k - 1;
          c2_km1 = c2_oc_a;
          c2_pc_a = c2_b_k + 1;
          c2_kp1 = c2_pc_a;
          c2_c_f = c2_f;
          c2_unusedU1 = c2_g;
          c2_b_eml_xrotg(chartInstance, &c2_c_f, &c2_unusedU1, &c2_d_cs,
                         &c2_d_sn);
          c2_f = c2_c_f;
          c2_b_cs = c2_d_cs;
          c2_b_sn = c2_d_sn;
          if (c2_b_k > c2_b_q) {
            c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c2_km1), 1, 6, 1, 0) - 1] = c2_f;
          }

          c2_qc_a = c2_b_cs;
          c2_lc_b = c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c2_b_k), 1, 6, 1, 0) - 1];
          c2_s_y = c2_qc_a * c2_lc_b;
          c2_rc_a = c2_b_sn;
          c2_mc_b = c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c2_b_k), 1, 6, 1, 0) - 1];
          c2_t_y = c2_rc_a * c2_mc_b;
          c2_f = c2_s_y + c2_t_y;
          c2_sc_a = c2_b_cs;
          c2_nc_b = c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c2_b_k), 1, 6, 1, 0) - 1];
          c2_u_y = c2_sc_a * c2_nc_b;
          c2_tc_a = c2_b_sn;
          c2_oc_b = c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c2_b_k), 1, 6, 1, 0) - 1];
          c2_v_y = c2_tc_a * c2_oc_b;
          c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_b_k), 1, 6, 1, 0) - 1] = c2_u_y - c2_v_y;
          c2_uc_a = c2_b_sn;
          c2_pc_b = c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c2_kp1), 1, 6, 1, 0) - 1];
          c2_g = c2_uc_a * c2_pc_b;
          c2_vc_a = c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c2_kp1), 1, 6, 1, 0) - 1];
          c2_qc_b = c2_b_cs;
          c2_w_y = c2_vc_a * c2_qc_b;
          c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_kp1), 1, 6, 1, 0) - 1] = c2_w_y;
          c2_wc_a = c2_b_k - 1;
          c2_kb_c = c2_wc_a;
          c2_rc_b = c2_kb_c;
          c2_lb_c = 6 * c2_rc_b;
          c2_sc_b = c2_lb_c + 1;
          c2_colk = c2_sc_b;
          c2_tc_b = c2_b_k;
          c2_mb_c = 6 * c2_tc_b;
          c2_uc_b = c2_mb_c + 1;
          c2_colkp1 = c2_uc_b;
          c2_e_eml_xrot(chartInstance, c2_Vf, c2_colk, c2_colkp1, c2_b_cs,
                        c2_b_sn);
          c2_d_f = c2_f;
          c2_unusedU2 = c2_g;
          c2_b_eml_xrotg(chartInstance, &c2_d_f, &c2_unusedU2, &c2_e_cs,
                         &c2_e_sn);
          c2_f = c2_d_f;
          c2_b_cs = c2_e_cs;
          c2_b_sn = c2_e_sn;
          c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_b_k), 1, 6, 1, 0) - 1] = c2_f;
          c2_xc_a = c2_b_cs;
          c2_vc_b = c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c2_b_k), 1, 6, 1, 0) - 1];
          c2_x_y = c2_xc_a * c2_vc_b;
          c2_yc_a = c2_b_sn;
          c2_wc_b = c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c2_kp1), 1, 6, 1, 0) - 1];
          c2_y_y = c2_yc_a * c2_wc_b;
          c2_f = c2_x_y + c2_y_y;
          c2_ad_a = -c2_b_sn;
          c2_xc_b = c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c2_b_k), 1, 6, 1, 0) - 1];
          c2_ab_y = c2_ad_a * c2_xc_b;
          c2_bd_a = c2_b_cs;
          c2_yc_b = c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c2_kp1), 1, 6, 1, 0) - 1];
          c2_bb_y = c2_bd_a * c2_yc_b;
          c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_kp1), 1, 6, 1, 0) - 1] = c2_ab_y + c2_bb_y;
          c2_cd_a = c2_b_sn;
          c2_ad_b = c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c2_kp1), 1, 6, 1, 0) - 1];
          c2_g = c2_cd_a * c2_ad_b;
          c2_dd_a = c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c2_kp1), 1, 6, 1, 0) - 1];
          c2_bd_b = c2_b_cs;
          c2_cb_y = c2_dd_a * c2_bd_b;
          c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_kp1), 1, 6, 1, 0) - 1] = c2_cb_y;
          if (c2_b_k < 12) {
            c2_ed_a = c2_b_k - 1;
            c2_nb_c = c2_ed_a;
            c2_cd_b = c2_nb_c;
            c2_ob_c = 12 * c2_cd_b;
            c2_dd_b = c2_ob_c + 1;
            c2_colk = c2_dd_b;
            c2_ed_b = c2_b_k;
            c2_pb_c = 12 * c2_ed_b;
            c2_fd_b = c2_pb_c + 1;
            c2_colkp1 = c2_fd_b;
            c2_f_eml_xrot(chartInstance, c2_U, c2_colk, c2_colkp1, c2_b_cs,
                          c2_b_sn);
          }
        }

        c2_fd_a = c2_m - 1;
        c2_qb_c = c2_fd_a;
        c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_qb_c), 1, 6, 1, 0) - 1] = c2_f;
        c2_iter++;
        break;

       default:
        if (c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c2_b_q), 1, 6, 1, 0) - 1] < 0.0) {
          c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_b_q), 1, 6, 1, 0) - 1] =
            -c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_b_q), 1, 6, 1, 0) - 1];
          c2_gd_a = c2_b_q - 1;
          c2_rb_c = c2_gd_a;
          c2_gd_b = c2_rb_c;
          c2_sb_c = 6 * c2_gd_b;
          c2_hd_b = c2_sb_c + 1;
          c2_colq = c2_hd_b;
          c2_d_eml_scalar_eg(chartInstance);
          c2_d8 = -1.0;
          c2_i_eml_xscal(chartInstance, c2_d8, c2_Vf, c2_colq);
        }

        c2_hd_a = c2_b_q + 1;
        c2_qp1 = c2_hd_a;
        exitg3 = FALSE;
        while ((exitg3 == FALSE) && (c2_b_q < 6)) {
          if (c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
                "", (real_T)c2_b_q), 1, 6, 1, 0) - 1] <
              c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
                "", (real_T)c2_qp1), 1, 6, 1, 0) - 1]) {
            c2_rt = c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
              _SFD_INTEGER_CHECK("", (real_T)c2_b_q), 1, 6, 1, 0) - 1];
            c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c2_b_q), 1, 6, 1, 0) - 1] =
              c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
              "", (real_T)c2_qp1), 1, 6, 1, 0) - 1];
            c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c2_qp1), 1, 6, 1, 0) - 1] = c2_rt;
            if (c2_b_q < 6) {
              c2_jd_a = c2_b_q - 1;
              c2_tb_c = c2_jd_a;
              c2_id_b = c2_tb_c;
              c2_ub_c = 6 * c2_id_b;
              c2_jd_b = c2_ub_c + 1;
              c2_colq = c2_jd_b;
              c2_kd_b = c2_b_q;
              c2_vb_c = 6 * c2_kd_b;
              c2_ld_b = c2_vb_c + 1;
              c2_colqp1 = c2_ld_b;
              c2_f_eml_xswap(chartInstance, c2_Vf, c2_colq, c2_colqp1);
            }

            if (c2_b_q < 12) {
              c2_kd_a = c2_b_q - 1;
              c2_wb_c = c2_kd_a;
              c2_md_b = c2_wb_c;
              c2_xb_c = 12 * c2_md_b;
              c2_nd_b = c2_xb_c + 1;
              c2_colq = c2_nd_b;
              c2_od_b = c2_b_q;
              c2_yb_c = 12 * c2_od_b;
              c2_pd_b = c2_yb_c + 1;
              c2_colqp1 = c2_pd_b;
              c2_g_eml_xswap(chartInstance, c2_U, c2_colq, c2_colqp1);
            }

            c2_b_q = c2_qp1;
            c2_ld_a = c2_b_q + 1;
            c2_qp1 = c2_ld_a;
          } else {
            exitg3 = TRUE;
          }
        }

        c2_iter = 0.0;
        c2_id_a = c2_m - 1;
        c2_m = c2_id_a;
        break;
      }
    }
  }

  c2_check_forloop_overflow_error(chartInstance);
  for (c2_e_k = 1; c2_e_k < 7; c2_e_k++) {
    c2_b_k = c2_e_k;
    c2_S[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c2_b_k), 1, 6, 1, 0) - 1] = c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
      _SFD_INTEGER_CHECK("", (real_T)c2_b_k), 1, 6, 1, 0) - 1];
  }

  c2_check_forloop_overflow_error(chartInstance);
  for (c2_j = 1; c2_j < 7; c2_j++) {
    c2_b_j = c2_j;
    c2_check_forloop_overflow_error(chartInstance);
    for (c2_i = 1; c2_i < 7; c2_i++) {
      c2_b_i = c2_i;
      c2_V[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c2_b_i), 1, 6, 1, 0) + 6 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
              (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_j), 1, 6, 2, 0) - 1))
        - 1] = c2_Vf[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c2_b_i), 1, 6, 1, 0) + 6 *
                      (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c2_b_j), 1, 6, 2, 0) - 1)) - 1];
    }
  }
}

static real_T c2_eml_xnrm2(SFc2_controllerInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_x[72], int32_T c2_ix0)
{
  real_T c2_y;
  int32_T c2_b_n;
  int32_T c2_b_ix0;
  int32_T c2_c_n;
  int32_T c2_c_ix0;
  real_T c2_b_x;
  real_T c2_c_x;
  real_T c2_scale;
  int32_T c2_kstart;
  int32_T c2_a;
  int32_T c2_c;
  int32_T c2_b_a;
  int32_T c2_b_c;
  int32_T c2_c_a;
  int32_T c2_b;
  int32_T c2_kend;
  int32_T c2_b_kstart;
  int32_T c2_b_kend;
  int32_T c2_d_a;
  int32_T c2_b_b;
  int32_T c2_e_a;
  int32_T c2_c_b;
  boolean_T c2_overflow;
  int32_T c2_k;
  int32_T c2_b_k;
  real_T c2_d_x;
  real_T c2_e_x;
  real_T c2_absxk;
  real_T c2_t;
  c2_b_n = c2_n;
  c2_b_ix0 = c2_ix0;
  c2_c_n = c2_b_n;
  c2_c_ix0 = c2_b_ix0;
  c2_y = 0.0;
  if (c2_c_n < 1) {
  } else if (c2_c_n == 1) {
    c2_b_x = c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c2_c_ix0), 1, 72, 1, 0) - 1];
    c2_c_x = c2_b_x;
    c2_y = muDoubleScalarAbs(c2_c_x);
  } else {
    c2_realmin(chartInstance);
    c2_scale = 2.2250738585072014E-308;
    c2_kstart = c2_c_ix0;
    c2_a = c2_c_n;
    c2_c = c2_a;
    c2_b_a = c2_c - 1;
    c2_b_c = c2_b_a;
    c2_c_a = c2_kstart;
    c2_b = c2_b_c;
    c2_kend = c2_c_a + c2_b;
    c2_b_kstart = c2_kstart;
    c2_b_kend = c2_kend;
    c2_d_a = c2_b_kstart;
    c2_b_b = c2_b_kend;
    c2_e_a = c2_d_a;
    c2_c_b = c2_b_b;
    if (c2_e_a > c2_c_b) {
      c2_overflow = FALSE;
    } else {
      c2_overflow = (c2_c_b > 2147483646);
    }

    c2_b_check_forloop_overflow_error(chartInstance, c2_overflow);
    for (c2_k = c2_b_kstart; c2_k <= c2_b_kend; c2_k++) {
      c2_b_k = c2_k;
      c2_d_x = c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
        "", (real_T)c2_b_k), 1, 72, 1, 0) - 1];
      c2_e_x = c2_d_x;
      c2_absxk = muDoubleScalarAbs(c2_e_x);
      if (c2_absxk > c2_scale) {
        c2_t = c2_scale / c2_absxk;
        c2_y = 1.0 + c2_y * c2_t * c2_t;
        c2_scale = c2_absxk;
      } else {
        c2_t = c2_absxk / c2_scale;
        c2_y += c2_t * c2_t;
      }
    }

    c2_y = c2_scale * muDoubleScalarSqrt(c2_y);
  }

  return c2_y;
}

static void c2_b_check_forloop_overflow_error(SFc2_controllerInstanceStruct
  *chartInstance, boolean_T c2_overflow)
{
  int32_T c2_i368;
  static char_T c2_cv0[34] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'i', 'n', 't', '_', 'f', 'o', 'r', 'l', 'o', 'o', 'p',
    '_', 'o', 'v', 'e', 'r', 'f', 'l', 'o', 'w' };

  char_T c2_u[34];
  const mxArray *c2_y = NULL;
  int32_T c2_i369;
  static char_T c2_cv1[23] = { 'c', 'o', 'd', 'e', 'r', '.', 'i', 'n', 't', 'e',
    'r', 'n', 'a', 'l', '.', 'i', 'n', 'd', 'e', 'x', 'I', 'n', 't' };

  char_T c2_b_u[23];
  const mxArray *c2_b_y = NULL;
  if (!c2_overflow) {
  } else {
    for (c2_i368 = 0; c2_i368 < 34; c2_i368++) {
      c2_u[c2_i368] = c2_cv0[c2_i368];
    }

    c2_y = NULL;
    sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 10, 0U, 1U, 0U, 2, 1, 34),
                  FALSE);
    for (c2_i369 = 0; c2_i369 < 23; c2_i369++) {
      c2_b_u[c2_i369] = c2_cv1[c2_i369];
    }

    c2_b_y = NULL;
    sf_mex_assign(&c2_b_y, sf_mex_create("y", c2_b_u, 10, 0U, 1U, 0U, 2, 1, 23),
                  FALSE);
    sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 2U,
      14, c2_y, 14, c2_b_y));
  }
}

static real_T c2_eml_div(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_x, real_T c2_y)
{
  return c2_x / c2_y;
}

static void c2_eml_xscal(SFc2_controllerInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_a, real_T c2_x[72], int32_T c2_ix0, real_T c2_b_x[72])
{
  int32_T c2_i370;
  for (c2_i370 = 0; c2_i370 < 72; c2_i370++) {
    c2_b_x[c2_i370] = c2_x[c2_i370];
  }

  c2_g_eml_xscal(chartInstance, c2_n, c2_a, c2_b_x, c2_ix0);
}

static real_T c2_eml_xdotc(SFc2_controllerInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_x[72], int32_T c2_ix0, real_T c2_y[72], int32_T c2_iy0)
{
  real_T c2_d;
  int32_T c2_b_n;
  int32_T c2_b_ix0;
  int32_T c2_b_iy0;
  int32_T c2_c_n;
  int32_T c2_c_ix0;
  int32_T c2_c_iy0;
  int32_T c2_d_n;
  int32_T c2_d_ix0;
  int32_T c2_d_iy0;
  int32_T c2_e_n;
  int32_T c2_e_ix0;
  int32_T c2_e_iy0;
  int32_T c2_ix;
  int32_T c2_iy;
  int32_T c2_f_n;
  int32_T c2_b;
  int32_T c2_b_b;
  boolean_T c2_overflow;
  int32_T c2_k;
  int32_T c2_a;
  int32_T c2_b_a;
  c2_b_n = c2_n;
  c2_b_ix0 = c2_ix0;
  c2_b_iy0 = c2_iy0;
  c2_c_n = c2_b_n;
  c2_c_ix0 = c2_b_ix0;
  c2_c_iy0 = c2_b_iy0;
  c2_d_n = c2_c_n;
  c2_d_ix0 = c2_c_ix0;
  c2_d_iy0 = c2_c_iy0;
  c2_e_n = c2_d_n;
  c2_e_ix0 = c2_d_ix0;
  c2_e_iy0 = c2_d_iy0;
  c2_d = 0.0;
  if (c2_e_n < 1) {
  } else {
    c2_ix = c2_e_ix0;
    c2_iy = c2_e_iy0;
    c2_f_n = c2_e_n;
    c2_b = c2_f_n;
    c2_b_b = c2_b;
    if (1 > c2_b_b) {
      c2_overflow = FALSE;
    } else {
      c2_overflow = (c2_b_b > 2147483646);
    }

    c2_b_check_forloop_overflow_error(chartInstance, c2_overflow);
    for (c2_k = 1; c2_k <= c2_f_n; c2_k++) {
      c2_d += c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
        "", (real_T)c2_ix), 1, 72, 1, 0) - 1] * c2_y[_SFD_EML_ARRAY_BOUNDS_CHECK
        ("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_iy), 1, 72, 1, 0) - 1];
      c2_a = c2_ix + 1;
      c2_ix = c2_a;
      c2_b_a = c2_iy + 1;
      c2_iy = c2_b_a;
    }
  }

  return c2_d;
}

static void c2_eml_xaxpy(SFc2_controllerInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_a, int32_T c2_ix0, real_T c2_y[72], int32_T c2_iy0, real_T
  c2_b_y[72])
{
  int32_T c2_i371;
  for (c2_i371 = 0; c2_i371 < 72; c2_i371++) {
    c2_b_y[c2_i371] = c2_y[c2_i371];
  }

  c2_i_eml_xaxpy(chartInstance, c2_n, c2_a, c2_ix0, c2_b_y, c2_iy0);
}

static real_T c2_b_eml_xnrm2(SFc2_controllerInstanceStruct *chartInstance,
  int32_T c2_n, real_T c2_x[6], int32_T c2_ix0)
{
  real_T c2_y;
  int32_T c2_b_n;
  int32_T c2_b_ix0;
  int32_T c2_c_n;
  int32_T c2_c_ix0;
  real_T c2_b_x;
  real_T c2_c_x;
  real_T c2_scale;
  int32_T c2_kstart;
  int32_T c2_a;
  int32_T c2_c;
  int32_T c2_b_a;
  int32_T c2_b_c;
  int32_T c2_c_a;
  int32_T c2_b;
  int32_T c2_kend;
  int32_T c2_b_kstart;
  int32_T c2_b_kend;
  int32_T c2_d_a;
  int32_T c2_b_b;
  int32_T c2_e_a;
  int32_T c2_c_b;
  boolean_T c2_overflow;
  int32_T c2_k;
  int32_T c2_b_k;
  real_T c2_d_x;
  real_T c2_e_x;
  real_T c2_absxk;
  real_T c2_t;
  c2_b_n = c2_n;
  c2_b_ix0 = c2_ix0;
  c2_c_n = c2_b_n;
  c2_c_ix0 = c2_b_ix0;
  c2_y = 0.0;
  if (c2_c_n < 1) {
  } else if (c2_c_n == 1) {
    c2_b_x = c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c2_c_ix0), 1, 6, 1, 0) - 1];
    c2_c_x = c2_b_x;
    c2_y = muDoubleScalarAbs(c2_c_x);
  } else {
    c2_realmin(chartInstance);
    c2_scale = 2.2250738585072014E-308;
    c2_kstart = c2_c_ix0;
    c2_a = c2_c_n;
    c2_c = c2_a;
    c2_b_a = c2_c - 1;
    c2_b_c = c2_b_a;
    c2_c_a = c2_kstart;
    c2_b = c2_b_c;
    c2_kend = c2_c_a + c2_b;
    c2_b_kstart = c2_kstart;
    c2_b_kend = c2_kend;
    c2_d_a = c2_b_kstart;
    c2_b_b = c2_b_kend;
    c2_e_a = c2_d_a;
    c2_c_b = c2_b_b;
    if (c2_e_a > c2_c_b) {
      c2_overflow = FALSE;
    } else {
      c2_overflow = (c2_c_b > 2147483646);
    }

    c2_b_check_forloop_overflow_error(chartInstance, c2_overflow);
    for (c2_k = c2_b_kstart; c2_k <= c2_b_kend; c2_k++) {
      c2_b_k = c2_k;
      c2_d_x = c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
        "", (real_T)c2_b_k), 1, 6, 1, 0) - 1];
      c2_e_x = c2_d_x;
      c2_absxk = muDoubleScalarAbs(c2_e_x);
      if (c2_absxk > c2_scale) {
        c2_t = c2_scale / c2_absxk;
        c2_y = 1.0 + c2_y * c2_t * c2_t;
        c2_scale = c2_absxk;
      } else {
        c2_t = c2_absxk / c2_scale;
        c2_y += c2_t * c2_t;
      }
    }

    c2_y = c2_scale * muDoubleScalarSqrt(c2_y);
  }

  return c2_y;
}

static void c2_b_eml_xscal(SFc2_controllerInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_a, real_T c2_x[6], int32_T c2_ix0, real_T c2_b_x[6])
{
  int32_T c2_i372;
  for (c2_i372 = 0; c2_i372 < 6; c2_i372++) {
    c2_b_x[c2_i372] = c2_x[c2_i372];
  }

  c2_h_eml_xscal(chartInstance, c2_n, c2_a, c2_b_x, c2_ix0);
}

static void c2_b_eml_xaxpy(SFc2_controllerInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_a, real_T c2_x[72], int32_T c2_ix0, real_T c2_y[12], int32_T
  c2_iy0, real_T c2_b_y[12])
{
  int32_T c2_i373;
  int32_T c2_i374;
  real_T c2_b_x[72];
  for (c2_i373 = 0; c2_i373 < 12; c2_i373++) {
    c2_b_y[c2_i373] = c2_y[c2_i373];
  }

  for (c2_i374 = 0; c2_i374 < 72; c2_i374++) {
    c2_b_x[c2_i374] = c2_x[c2_i374];
  }

  c2_j_eml_xaxpy(chartInstance, c2_n, c2_a, c2_b_x, c2_ix0, c2_b_y, c2_iy0);
}

static void c2_c_eml_xaxpy(SFc2_controllerInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_a, real_T c2_x[12], int32_T c2_ix0, real_T c2_y[72], int32_T
  c2_iy0, real_T c2_b_y[72])
{
  int32_T c2_i375;
  int32_T c2_i376;
  real_T c2_b_x[12];
  for (c2_i375 = 0; c2_i375 < 72; c2_i375++) {
    c2_b_y[c2_i375] = c2_y[c2_i375];
  }

  for (c2_i376 = 0; c2_i376 < 12; c2_i376++) {
    c2_b_x[c2_i376] = c2_x[c2_i376];
  }

  c2_k_eml_xaxpy(chartInstance, c2_n, c2_a, c2_b_x, c2_ix0, c2_b_y, c2_iy0);
}

static real_T c2_b_eml_xdotc(SFc2_controllerInstanceStruct *chartInstance,
  int32_T c2_n, real_T c2_x[36], int32_T c2_ix0, real_T c2_y[36], int32_T c2_iy0)
{
  real_T c2_d;
  int32_T c2_b_n;
  int32_T c2_b_ix0;
  int32_T c2_b_iy0;
  int32_T c2_c_n;
  int32_T c2_c_ix0;
  int32_T c2_c_iy0;
  int32_T c2_d_n;
  int32_T c2_d_ix0;
  int32_T c2_d_iy0;
  int32_T c2_e_n;
  int32_T c2_e_ix0;
  int32_T c2_e_iy0;
  int32_T c2_ix;
  int32_T c2_iy;
  int32_T c2_f_n;
  int32_T c2_b;
  int32_T c2_b_b;
  boolean_T c2_overflow;
  int32_T c2_k;
  int32_T c2_a;
  int32_T c2_b_a;
  c2_b_n = c2_n;
  c2_b_ix0 = c2_ix0;
  c2_b_iy0 = c2_iy0;
  c2_c_n = c2_b_n;
  c2_c_ix0 = c2_b_ix0;
  c2_c_iy0 = c2_b_iy0;
  c2_d_n = c2_c_n;
  c2_d_ix0 = c2_c_ix0;
  c2_d_iy0 = c2_c_iy0;
  c2_e_n = c2_d_n;
  c2_e_ix0 = c2_d_ix0;
  c2_e_iy0 = c2_d_iy0;
  c2_d = 0.0;
  if (c2_e_n < 1) {
  } else {
    c2_ix = c2_e_ix0;
    c2_iy = c2_e_iy0;
    c2_f_n = c2_e_n;
    c2_b = c2_f_n;
    c2_b_b = c2_b;
    if (1 > c2_b_b) {
      c2_overflow = FALSE;
    } else {
      c2_overflow = (c2_b_b > 2147483646);
    }

    c2_b_check_forloop_overflow_error(chartInstance, c2_overflow);
    for (c2_k = 1; c2_k <= c2_f_n; c2_k++) {
      c2_d += c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
        "", (real_T)c2_ix), 1, 36, 1, 0) - 1] * c2_y[_SFD_EML_ARRAY_BOUNDS_CHECK
        ("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_iy), 1, 36, 1, 0) - 1];
      c2_a = c2_ix + 1;
      c2_ix = c2_a;
      c2_b_a = c2_iy + 1;
      c2_iy = c2_b_a;
    }
  }

  return c2_d;
}

static void c2_d_eml_xaxpy(SFc2_controllerInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_a, int32_T c2_ix0, real_T c2_y[36], int32_T c2_iy0, real_T
  c2_b_y[36])
{
  int32_T c2_i377;
  for (c2_i377 = 0; c2_i377 < 36; c2_i377++) {
    c2_b_y[c2_i377] = c2_y[c2_i377];
  }

  c2_l_eml_xaxpy(chartInstance, c2_n, c2_a, c2_ix0, c2_b_y, c2_iy0);
}

static void c2_d_eml_scalar_eg(SFc2_controllerInstanceStruct *chartInstance)
{
}

static void c2_c_eml_xscal(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_a, real_T c2_x[36], int32_T c2_ix0, real_T c2_b_x[36])
{
  int32_T c2_i378;
  for (c2_i378 = 0; c2_i378 < 36; c2_i378++) {
    c2_b_x[c2_i378] = c2_x[c2_i378];
  }

  c2_i_eml_xscal(chartInstance, c2_a, c2_b_x, c2_ix0);
}

static void c2_eps(SFc2_controllerInstanceStruct *chartInstance)
{
}

static void c2_b_eml_error(SFc2_controllerInstanceStruct *chartInstance)
{
  int32_T c2_i379;
  static char_T c2_varargin_1[30] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A',
    'T', 'L', 'A', 'B', ':', 's', 'v', 'd', '_', 'N', 'o', 'C', 'o', 'n', 'v',
    'e', 'r', 'g', 'e', 'n', 'c', 'e' };

  char_T c2_u[30];
  const mxArray *c2_y = NULL;
  for (c2_i379 = 0; c2_i379 < 30; c2_i379++) {
    c2_u[c2_i379] = c2_varargin_1[c2_i379];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 10, 0U, 1U, 0U, 2, 1, 30), FALSE);
  sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 1U, 14,
    c2_y));
}

static real_T c2_sqrt(SFc2_controllerInstanceStruct *chartInstance, real_T c2_x)
{
  real_T c2_b_x;
  c2_b_x = c2_x;
  c2_b_sqrt(chartInstance, &c2_b_x);
  return c2_b_x;
}

static void c2_c_eml_error(SFc2_controllerInstanceStruct *chartInstance)
{
  int32_T c2_i380;
  static char_T c2_varargin_1[30] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o',
    'o', 'l', 'b', 'o', 'x', ':', 's', 'q', 'r', 't', '_', 'd', 'o', 'm', 'a',
    'i', 'n', 'E', 'r', 'r', 'o', 'r' };

  char_T c2_u[30];
  const mxArray *c2_y = NULL;
  for (c2_i380 = 0; c2_i380 < 30; c2_i380++) {
    c2_u[c2_i380] = c2_varargin_1[c2_i380];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 10, 0U, 1U, 0U, 2, 1, 30), FALSE);
  sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 1U, 14,
    c2_y));
}

static void c2_eml_xrotg(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_a, real_T c2_b, real_T *c2_b_a, real_T *c2_b_b, real_T *c2_c, real_T *c2_s)
{
  *c2_b_a = c2_a;
  *c2_b_b = c2_b;
  c2_b_eml_xrotg(chartInstance, c2_b_a, c2_b_b, c2_c, c2_s);
}

static void c2_eml_xrot(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_x[36], int32_T c2_ix0, int32_T c2_iy0, real_T c2_c, real_T c2_s, real_T
  c2_b_x[36])
{
  int32_T c2_i381;
  for (c2_i381 = 0; c2_i381 < 36; c2_i381++) {
    c2_b_x[c2_i381] = c2_x[c2_i381];
  }

  c2_e_eml_xrot(chartInstance, c2_b_x, c2_ix0, c2_iy0, c2_c, c2_s);
}

static void c2_b_eml_xrot(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_x[72], int32_T c2_ix0, int32_T c2_iy0, real_T c2_c, real_T c2_s, real_T
  c2_b_x[72])
{
  int32_T c2_i382;
  for (c2_i382 = 0; c2_i382 < 72; c2_i382++) {
    c2_b_x[c2_i382] = c2_x[c2_i382];
  }

  c2_f_eml_xrot(chartInstance, c2_b_x, c2_ix0, c2_iy0, c2_c, c2_s);
}

static void c2_eml_xswap(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_x[36], int32_T c2_ix0, int32_T c2_iy0, real_T c2_b_x[36])
{
  int32_T c2_i383;
  for (c2_i383 = 0; c2_i383 < 36; c2_i383++) {
    c2_b_x[c2_i383] = c2_x[c2_i383];
  }

  c2_f_eml_xswap(chartInstance, c2_b_x, c2_ix0, c2_iy0);
}

static void c2_b_eml_xswap(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_x[72], int32_T c2_ix0, int32_T c2_iy0, real_T c2_b_x[72])
{
  int32_T c2_i384;
  for (c2_i384 = 0; c2_i384 < 72; c2_i384++) {
    c2_b_x[c2_i384] = c2_x[c2_i384];
  }

  c2_g_eml_xswap(chartInstance, c2_b_x, c2_ix0, c2_iy0);
}

static void c2_eml_xgemm(SFc2_controllerInstanceStruct *chartInstance, int32_T
  c2_k, real_T c2_A[36], real_T c2_B[72], real_T c2_C[72], real_T c2_b_C[72])
{
  int32_T c2_i385;
  int32_T c2_i386;
  real_T c2_b_A[36];
  int32_T c2_i387;
  real_T c2_b_B[72];
  for (c2_i385 = 0; c2_i385 < 72; c2_i385++) {
    c2_b_C[c2_i385] = c2_C[c2_i385];
  }

  for (c2_i386 = 0; c2_i386 < 36; c2_i386++) {
    c2_b_A[c2_i386] = c2_A[c2_i386];
  }

  for (c2_i387 = 0; c2_i387 < 72; c2_i387++) {
    c2_b_B[c2_i387] = c2_B[c2_i387];
  }

  c2_j_eml_xgemm(chartInstance, c2_k, c2_b_A, c2_b_B, c2_b_C);
}

static void c2_e_eml_scalar_eg(SFc2_controllerInstanceStruct *chartInstance)
{
}

static void c2_inv(SFc2_controllerInstanceStruct *chartInstance, real_T c2_x[961],
                   real_T c2_y[961])
{
  int32_T c2_i388;
  real_T c2_b_x[961];
  int32_T c2_i389;
  real_T c2_c_x[961];
  real_T c2_n1x;
  int32_T c2_i390;
  real_T c2_b_y[961];
  real_T c2_n1xinv;
  real_T c2_a;
  real_T c2_b;
  real_T c2_c_y;
  real_T c2_rc;
  real_T c2_d_x;
  boolean_T c2_b_b;
  real_T c2_e_x;
  int32_T c2_i391;
  static char_T c2_cv2[8] = { '%', '%', '%', 'd', '.', '%', 'd', 'e' };

  char_T c2_u[8];
  const mxArray *c2_d_y = NULL;
  real_T c2_b_u;
  const mxArray *c2_e_y = NULL;
  real_T c2_c_u;
  const mxArray *c2_f_y = NULL;
  real_T c2_d_u;
  const mxArray *c2_g_y = NULL;
  char_T c2_str[14];
  int32_T c2_i392;
  char_T c2_b_str[14];
  boolean_T guard1 = FALSE;
  boolean_T guard2 = FALSE;
  boolean_T guard3 = FALSE;
  for (c2_i388 = 0; c2_i388 < 961; c2_i388++) {
    c2_b_x[c2_i388] = c2_x[c2_i388];
  }

  c2_invNxN(chartInstance, c2_b_x, c2_y);
  for (c2_i389 = 0; c2_i389 < 961; c2_i389++) {
    c2_c_x[c2_i389] = c2_x[c2_i389];
  }

  c2_n1x = c2_norm(chartInstance, c2_c_x);
  for (c2_i390 = 0; c2_i390 < 961; c2_i390++) {
    c2_b_y[c2_i390] = c2_y[c2_i390];
  }

  c2_n1xinv = c2_norm(chartInstance, c2_b_y);
  c2_a = c2_n1x;
  c2_b = c2_n1xinv;
  c2_c_y = c2_a * c2_b;
  c2_rc = 1.0 / c2_c_y;
  guard1 = FALSE;
  guard2 = FALSE;
  if (c2_n1x == 0.0) {
    guard2 = TRUE;
  } else if (c2_n1xinv == 0.0) {
    guard2 = TRUE;
  } else if (c2_rc == 0.0) {
    guard1 = TRUE;
  } else {
    c2_d_x = c2_rc;
    c2_b_b = muDoubleScalarIsNaN(c2_d_x);
    guard3 = FALSE;
    if (c2_b_b) {
      guard3 = TRUE;
    } else {
      c2_eps(chartInstance);
      if (c2_rc < 2.2204460492503131E-16) {
        guard3 = TRUE;
      }
    }

    if (guard3 == TRUE) {
      c2_e_x = c2_rc;
      for (c2_i391 = 0; c2_i391 < 8; c2_i391++) {
        c2_u[c2_i391] = c2_cv2[c2_i391];
      }

      c2_d_y = NULL;
      sf_mex_assign(&c2_d_y, sf_mex_create("y", c2_u, 10, 0U, 1U, 0U, 2, 1, 8),
                    FALSE);
      c2_b_u = 14.0;
      c2_e_y = NULL;
      sf_mex_assign(&c2_e_y, sf_mex_create("y", &c2_b_u, 0, 0U, 0U, 0U, 0),
                    FALSE);
      c2_c_u = 6.0;
      c2_f_y = NULL;
      sf_mex_assign(&c2_f_y, sf_mex_create("y", &c2_c_u, 0, 0U, 0U, 0U, 0),
                    FALSE);
      c2_d_u = c2_e_x;
      c2_g_y = NULL;
      sf_mex_assign(&c2_g_y, sf_mex_create("y", &c2_d_u, 0, 0U, 0U, 0U, 0),
                    FALSE);
      c2_q_emlrt_marshallIn(chartInstance, sf_mex_call_debug("sprintf", 1U, 2U,
        14, sf_mex_call_debug("sprintf", 1U, 3U, 14, c2_d_y, 14, c2_e_y, 14,
        c2_f_y), 14, c2_g_y), "sprintf", c2_str);
      for (c2_i392 = 0; c2_i392 < 14; c2_i392++) {
        c2_b_str[c2_i392] = c2_str[c2_i392];
      }

      c2_b_eml_warning(chartInstance, c2_b_str);
    }
  }

  if (guard2 == TRUE) {
    guard1 = TRUE;
  }

  if (guard1 == TRUE) {
    c2_eml_warning(chartInstance);
  }
}

static void c2_invNxN(SFc2_controllerInstanceStruct *chartInstance, real_T c2_x
                      [961], real_T c2_y[961])
{
  int32_T c2_i393;
  int32_T c2_info;
  int32_T c2_ipiv[31];
  int32_T c2_i394;
  int32_T c2_b_ipiv[31];
  int32_T c2_k;
  int32_T c2_b_k;
  int32_T c2_c;
  int32_T c2_c_k;
  boolean_T c2_overflow;
  int32_T c2_j;
  int32_T c2_b_j;
  int32_T c2_a;
  int32_T c2_i395;
  boolean_T c2_b_overflow;
  int32_T c2_i;
  int32_T c2_b_i;
  real_T c2_b_a;
  real_T c2_b;
  real_T c2_b_y;
  int32_T c2_i396;
  real_T c2_b_x[961];
  for (c2_i393 = 0; c2_i393 < 961; c2_i393++) {
    c2_y[c2_i393] = 0.0;
  }

  c2_b_eml_matlab_zgetrf(chartInstance, c2_x, c2_ipiv, &c2_info);
  for (c2_i394 = 0; c2_i394 < 31; c2_i394++) {
    c2_b_ipiv[c2_i394] = c2_ipiv[c2_i394];
  }

  c2_eml_ipiv2perm(chartInstance, c2_b_ipiv, c2_ipiv);
  c2_b_check_forloop_overflow_error(chartInstance, FALSE);
  for (c2_k = 1; c2_k < 32; c2_k++) {
    c2_b_k = c2_k;
    c2_c = c2_ipiv[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
      "", (real_T)c2_b_k), 1, 31, 1, 0) - 1];
    c2_y[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_b_k), 1, 31, 1, 0) + 31 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
            (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_c), 1, 31, 2, 0) - 1)) -
      1] = 1.0;
    c2_c_k = c2_b_k;
    c2_overflow = FALSE;
    c2_b_check_forloop_overflow_error(chartInstance, c2_overflow);
    for (c2_j = c2_c_k; c2_j < 32; c2_j++) {
      c2_b_j = c2_j;
      if (c2_y[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
             (real_T)c2_b_j), 1, 31, 1, 0) + 31 * (_SFD_EML_ARRAY_BOUNDS_CHECK(
             "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_c), 1, 31, 2, 0) - 1))
          - 1] != 0.0) {
        c2_a = c2_b_j + 1;
        c2_i395 = c2_a;
        c2_b_overflow = FALSE;
        c2_b_check_forloop_overflow_error(chartInstance, c2_b_overflow);
        for (c2_i = c2_i395; c2_i < 32; c2_i++) {
          c2_b_i = c2_i;
          c2_b_a = c2_y[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c2_b_j), 1, 31, 1, 0) + 31 *
                         (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c2_c), 1, 31, 2, 0) - 1)) - 1];
          c2_b = c2_x[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c2_b_i), 1, 31, 1, 0) + 31 *
                       (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c2_b_j), 1, 31, 2, 0) - 1)) - 1];
          c2_b_y = c2_b_a * c2_b;
          c2_y[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                  (real_T)c2_b_i), 1, 31, 1, 0) + 31 *
                (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                   (real_T)c2_c), 1, 31, 2, 0) - 1)) - 1] = c2_y
            [(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                (real_T)c2_b_i), 1, 31, 1, 0) + 31 *
              (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                 (real_T)c2_c), 1, 31, 2, 0) - 1)) - 1] - c2_b_y;
        }
      }
    }
  }

  c2_f_eml_scalar_eg(chartInstance);
  for (c2_i396 = 0; c2_i396 < 961; c2_i396++) {
    c2_b_x[c2_i396] = c2_x[c2_i396];
  }

  c2_b_eml_xtrsm(chartInstance, c2_b_x, c2_y);
}

static void c2_eml_matlab_zgetrf(SFc2_controllerInstanceStruct *chartInstance,
  real_T c2_A[961], real_T c2_b_A[961], int32_T c2_ipiv[31], int32_T *c2_info)
{
  int32_T c2_i397;
  for (c2_i397 = 0; c2_i397 < 961; c2_i397++) {
    c2_b_A[c2_i397] = c2_A[c2_i397];
  }

  c2_b_eml_matlab_zgetrf(chartInstance, c2_b_A, c2_ipiv, c2_info);
}

static void c2_c_eml_xswap(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_x[961], int32_T c2_ix0, int32_T c2_iy0, real_T c2_b_x[961])
{
  int32_T c2_i398;
  for (c2_i398 = 0; c2_i398 < 961; c2_i398++) {
    c2_b_x[c2_i398] = c2_x[c2_i398];
  }

  c2_h_eml_xswap(chartInstance, c2_b_x, c2_ix0, c2_iy0);
}

static void c2_below_threshold(SFc2_controllerInstanceStruct *chartInstance)
{
}

static void c2_eml_xger(SFc2_controllerInstanceStruct *chartInstance, int32_T
  c2_m, int32_T c2_n, real_T c2_alpha1, int32_T c2_ix0, int32_T c2_iy0, real_T
  c2_A[961], int32_T c2_ia0, real_T c2_b_A[961])
{
  int32_T c2_i399;
  for (c2_i399 = 0; c2_i399 < 961; c2_i399++) {
    c2_b_A[c2_i399] = c2_A[c2_i399];
  }

  c2_b_eml_xger(chartInstance, c2_m, c2_n, c2_alpha1, c2_ix0, c2_iy0, c2_b_A,
                c2_ia0);
}

static void c2_eml_ipiv2perm(SFc2_controllerInstanceStruct *chartInstance,
  int32_T c2_ipiv[31], int32_T c2_perm[31])
{
  int32_T c2_i400;
  int32_T c2_k;
  real_T c2_b_k;
  int32_T c2_ipk;
  int32_T c2_a;
  real_T c2_b;
  int32_T c2_b_a;
  real_T c2_b_b;
  int32_T c2_idx;
  real_T c2_flt;
  boolean_T c2_p;
  int32_T c2_pipk;
  for (c2_i400 = 0; c2_i400 < 31; c2_i400++) {
    c2_perm[c2_i400] = 1 + c2_i400;
  }

  for (c2_k = 0; c2_k < 30; c2_k++) {
    c2_b_k = 1.0 + (real_T)c2_k;
    c2_ipk = c2_ipiv[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK
      ("", c2_b_k), 1, 31, 1, 0) - 1];
    c2_a = c2_ipk;
    c2_b = c2_b_k;
    c2_b_a = c2_a;
    c2_b_b = c2_b;
    c2_idx = c2_b_a;
    c2_flt = c2_b_b;
    c2_p = ((real_T)c2_idx > c2_flt);
    if (c2_p) {
      c2_pipk = c2_perm[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c2_ipk), 1, 31, 1, 0) - 1];
      c2_perm[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c2_ipk), 1, 31, 1, 0) - 1] = c2_perm[_SFD_EML_ARRAY_BOUNDS_CHECK
        ("", (int32_T)_SFD_INTEGER_CHECK("", c2_b_k), 1, 31, 1, 0) - 1];
      c2_perm[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        c2_b_k), 1, 31, 1, 0) - 1] = c2_pipk;
    }
  }
}

static void c2_f_eml_scalar_eg(SFc2_controllerInstanceStruct *chartInstance)
{
}

static void c2_eml_xtrsm(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_A[961], real_T c2_B[961], real_T c2_b_B[961])
{
  int32_T c2_i401;
  int32_T c2_i402;
  real_T c2_b_A[961];
  for (c2_i401 = 0; c2_i401 < 961; c2_i401++) {
    c2_b_B[c2_i401] = c2_B[c2_i401];
  }

  for (c2_i402 = 0; c2_i402 < 961; c2_i402++) {
    c2_b_A[c2_i402] = c2_A[c2_i402];
  }

  c2_b_eml_xtrsm(chartInstance, c2_b_A, c2_b_B);
}

static real_T c2_norm(SFc2_controllerInstanceStruct *chartInstance, real_T c2_x
                      [961])
{
  real_T c2_y;
  int32_T c2_j;
  real_T c2_b_j;
  real_T c2_s;
  int32_T c2_i;
  real_T c2_b_i;
  real_T c2_b_x;
  real_T c2_c_x;
  real_T c2_b_y;
  real_T c2_d_x;
  boolean_T c2_b;
  boolean_T exitg1;
  c2_y = 0.0;
  c2_j = 0;
  exitg1 = FALSE;
  while ((exitg1 == FALSE) && (c2_j < 31)) {
    c2_b_j = 1.0 + (real_T)c2_j;
    c2_s = 0.0;
    for (c2_i = 0; c2_i < 31; c2_i++) {
      c2_b_i = 1.0 + (real_T)c2_i;
      c2_b_x = c2_x[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK
                      ("", c2_b_i), 1, 31, 1, 0) + 31 *
                     (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", c2_b_j), 1, 31, 2, 0) - 1)) - 1];
      c2_c_x = c2_b_x;
      c2_b_y = muDoubleScalarAbs(c2_c_x);
      c2_s += c2_b_y;
    }

    c2_d_x = c2_s;
    c2_b = muDoubleScalarIsNaN(c2_d_x);
    if (c2_b) {
      c2_y = rtNaN;
      exitg1 = TRUE;
    } else {
      if (c2_s > c2_y) {
        c2_y = c2_s;
      }

      c2_j++;
    }
  }

  return c2_y;
}

static void c2_eml_warning(SFc2_controllerInstanceStruct *chartInstance)
{
  int32_T c2_i403;
  static char_T c2_varargin_1[27] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A',
    'T', 'L', 'A', 'B', ':', 's', 'i', 'n', 'g', 'u', 'l', 'a', 'r', 'M', 'a',
    't', 'r', 'i', 'x' };

  char_T c2_u[27];
  const mxArray *c2_y = NULL;
  for (c2_i403 = 0; c2_i403 < 27; c2_i403++) {
    c2_u[c2_i403] = c2_varargin_1[c2_i403];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 10, 0U, 1U, 0U, 2, 1, 27), FALSE);
  sf_mex_call_debug("warning", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 1U,
    14, c2_y));
}

static void c2_b_eml_warning(SFc2_controllerInstanceStruct *chartInstance,
  char_T c2_varargin_2[14])
{
  int32_T c2_i404;
  static char_T c2_varargin_1[33] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A',
    'T', 'L', 'A', 'B', ':', 'i', 'l', 'l', 'C', 'o', 'n', 'd', 'i', 't', 'i',
    'o', 'n', 'e', 'd', 'M', 'a', 't', 'r', 'i', 'x' };

  char_T c2_u[33];
  const mxArray *c2_y = NULL;
  int32_T c2_i405;
  char_T c2_b_u[14];
  const mxArray *c2_b_y = NULL;
  for (c2_i404 = 0; c2_i404 < 33; c2_i404++) {
    c2_u[c2_i404] = c2_varargin_1[c2_i404];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 10, 0U, 1U, 0U, 2, 1, 33), FALSE);
  for (c2_i405 = 0; c2_i405 < 14; c2_i405++) {
    c2_b_u[c2_i405] = c2_varargin_2[c2_i405];
  }

  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", c2_b_u, 10, 0U, 1U, 0U, 2, 1, 14),
                FALSE);
  sf_mex_call_debug("warning", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 2U,
    14, c2_y, 14, c2_b_y));
}

static void c2_g_eml_scalar_eg(SFc2_controllerInstanceStruct *chartInstance)
{
}

static void c2_b_eml_xgemm(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_A[558], real_T c2_B[961], real_T c2_C[558], real_T c2_b_C[558])
{
  int32_T c2_i406;
  int32_T c2_i407;
  real_T c2_b_A[558];
  int32_T c2_i408;
  real_T c2_b_B[961];
  for (c2_i406 = 0; c2_i406 < 558; c2_i406++) {
    c2_b_C[c2_i406] = c2_C[c2_i406];
  }

  for (c2_i407 = 0; c2_i407 < 558; c2_i407++) {
    c2_b_A[c2_i407] = c2_A[c2_i407];
  }

  for (c2_i408 = 0; c2_i408 < 961; c2_i408++) {
    c2_b_B[c2_i408] = c2_B[c2_i408];
  }

  c2_k_eml_xgemm(chartInstance, c2_b_A, c2_b_B, c2_b_C);
}

static void c2_h_eml_scalar_eg(SFc2_controllerInstanceStruct *chartInstance)
{
}

static void c2_c_eml_xgemm(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_A[558], real_T c2_B[775], real_T c2_C[450], real_T c2_b_C[450])
{
  int32_T c2_i409;
  int32_T c2_i410;
  real_T c2_b_A[558];
  int32_T c2_i411;
  real_T c2_b_B[775];
  for (c2_i409 = 0; c2_i409 < 450; c2_i409++) {
    c2_b_C[c2_i409] = c2_C[c2_i409];
  }

  for (c2_i410 = 0; c2_i410 < 558; c2_i410++) {
    c2_b_A[c2_i410] = c2_A[c2_i410];
  }

  for (c2_i411 = 0; c2_i411 < 775; c2_i411++) {
    c2_b_B[c2_i411] = c2_B[c2_i411];
  }

  c2_l_eml_xgemm(chartInstance, c2_b_A, c2_b_B, c2_b_C);
}

static void c2_b_pinv(SFc2_controllerInstanceStruct *chartInstance, real_T c2_A
                      [450], real_T c2_tol, real_T c2_X[450])
{
  int32_T c2_i412;
  int32_T c2_i413;
  int32_T c2_i414;
  int32_T c2_i415;
  real_T c2_U[450];
  real_T c2_b_tol;
  int32_T c2_i416;
  real_T c2_b_X[450];
  int32_T c2_i417;
  real_T c2_b_U[450];
  real_T c2_V[324];
  real_T c2_S[324];
  int32_T c2_r;
  int32_T c2_k;
  int32_T c2_b_k;
  int32_T c2_a;
  int32_T c2_vcol;
  int32_T c2_b_r;
  int32_T c2_b;
  int32_T c2_b_b;
  boolean_T c2_overflow;
  int32_T c2_j;
  int32_T c2_b_j;
  real_T c2_d9;
  int32_T c2_b_a;
  int32_T c2_i418;
  real_T c2_b_V[324];
  int32_T c2_i419;
  real_T c2_c_U[450];
  int32_T c2_i420;
  int32_T c2_i421;
  int32_T c2_i422;
  int32_T c2_i423;
  boolean_T exitg1;
  c2_i412 = 0;
  for (c2_i413 = 0; c2_i413 < 18; c2_i413++) {
    c2_i414 = 0;
    for (c2_i415 = 0; c2_i415 < 25; c2_i415++) {
      c2_U[c2_i415 + c2_i412] = c2_A[c2_i414 + c2_i413];
      c2_i414 += 18;
    }

    c2_i412 += 25;
  }

  c2_b_tol = c2_tol;
  c2_i_eml_scalar_eg(chartInstance);
  for (c2_i416 = 0; c2_i416 < 450; c2_i416++) {
    c2_b_X[c2_i416] = 0.0;
  }

  for (c2_i417 = 0; c2_i417 < 450; c2_i417++) {
    c2_b_U[c2_i417] = c2_U[c2_i417];
  }

  c2_svd(chartInstance, c2_b_U, c2_U, c2_S, c2_V);
  c2_r = 0;
  c2_b_check_forloop_overflow_error(chartInstance, FALSE);
  c2_k = 1;
  exitg1 = FALSE;
  while ((exitg1 == FALSE) && (c2_k < 19)) {
    c2_b_k = c2_k;
    if (!(c2_S[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_b_k), 1, 18, 1, 0) + 18 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
            (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_k), 1, 18, 2, 0) - 1))
          - 1] > c2_b_tol)) {
      exitg1 = TRUE;
    } else {
      c2_a = c2_r + 1;
      c2_r = c2_a;
      c2_k++;
    }
  }

  if (c2_r > 0) {
    c2_vcol = 1;
    c2_b_r = c2_r;
    c2_b = c2_b_r;
    c2_b_b = c2_b;
    if (1 > c2_b_b) {
      c2_overflow = FALSE;
    } else {
      c2_overflow = (c2_b_b > 2147483646);
    }

    c2_b_check_forloop_overflow_error(chartInstance, c2_overflow);
    for (c2_j = 1; c2_j <= c2_b_r; c2_j++) {
      c2_b_j = c2_j;
      c2_d9 = c2_eml_div(chartInstance, 1.0, c2_S[(_SFD_EML_ARRAY_BOUNDS_CHECK(
        "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_j), 1, 18, 1, 0) + 18 *
                          (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c2_b_j), 1, 18, 2, 0) - 1)) - 1]);
      c2_l_eml_xscal(chartInstance, c2_d9, c2_V, c2_vcol);
      c2_b_a = c2_vcol + 18;
      c2_vcol = c2_b_a;
    }

    for (c2_i418 = 0; c2_i418 < 324; c2_i418++) {
      c2_b_V[c2_i418] = c2_V[c2_i418];
    }

    for (c2_i419 = 0; c2_i419 < 450; c2_i419++) {
      c2_c_U[c2_i419] = c2_U[c2_i419];
    }

    c2_m_eml_xgemm(chartInstance, c2_r, c2_b_V, c2_c_U, c2_b_X);
  }

  c2_i420 = 0;
  for (c2_i421 = 0; c2_i421 < 18; c2_i421++) {
    c2_i422 = 0;
    for (c2_i423 = 0; c2_i423 < 25; c2_i423++) {
      c2_X[c2_i423 + c2_i420] = c2_b_X[c2_i422 + c2_i421];
      c2_i422 += 18;
    }

    c2_i420 += 25;
  }
}

static void c2_i_eml_scalar_eg(SFc2_controllerInstanceStruct *chartInstance)
{
}

static void c2_svd(SFc2_controllerInstanceStruct *chartInstance, real_T c2_A[450],
                   real_T c2_U[450], real_T c2_S[324], real_T c2_V[324])
{
  int32_T c2_k;
  int32_T c2_b_k;
  int32_T c2_i424;
  real_T c2_b_A[450];
  real_T c2_s[18];
  int32_T c2_i425;
  int32_T c2_c_k;
  real_T c2_d_k;
  c2_b_check_forloop_overflow_error(chartInstance, FALSE);
  for (c2_k = 1; c2_k < 451; c2_k++) {
    c2_b_k = c2_k;
    if (!c2_isfinite(chartInstance, c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_k), 1, 450, 1, 0) - 1]))
    {
      c2_eml_error(chartInstance);
    }
  }

  for (c2_i424 = 0; c2_i424 < 450; c2_i424++) {
    c2_b_A[c2_i424] = c2_A[c2_i424];
  }

  c2_b_eml_xgesvd(chartInstance, c2_b_A, c2_U, c2_s, c2_V);
  for (c2_i425 = 0; c2_i425 < 324; c2_i425++) {
    c2_S[c2_i425] = 0.0;
  }

  for (c2_c_k = 0; c2_c_k < 18; c2_c_k++) {
    c2_d_k = 1.0 + (real_T)c2_c_k;
    c2_S[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", c2_d_k),
           1, 18, 1, 0) + 18 * (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", c2_d_k), 1, 18, 2, 0) - 1)) - 1] =
      c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      c2_d_k), 1, 18, 1, 0) - 1];
  }
}

static void c2_b_eml_xgesvd(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_A[450], real_T c2_U[450], real_T c2_S[18], real_T c2_V[324])
{
  int32_T c2_i426;
  real_T c2_b_A[450];
  int32_T c2_i427;
  real_T c2_s[18];
  int32_T c2_i428;
  real_T c2_e[18];
  int32_T c2_i429;
  real_T c2_work[25];
  int32_T c2_i430;
  int32_T c2_i431;
  real_T c2_Vf[324];
  int32_T c2_q;
  int32_T c2_b_q;
  int32_T c2_a;
  int32_T c2_qp1;
  int32_T c2_b_a;
  int32_T c2_qm1;
  int32_T c2_b;
  int32_T c2_c;
  int32_T c2_c_a;
  int32_T c2_b_b;
  int32_T c2_qq;
  int32_T c2_c_b;
  int32_T c2_nmq;
  int32_T c2_d_a;
  int32_T c2_nmqp1;
  int32_T c2_i432;
  real_T c2_c_A[450];
  real_T c2_nrm;
  real_T c2_absx;
  real_T c2_d;
  real_T c2_y;
  real_T c2_d10;
  int32_T c2_b_qp1;
  boolean_T c2_overflow;
  int32_T c2_jj;
  int32_T c2_b_jj;
  int32_T c2_e_a;
  int32_T c2_b_c;
  int32_T c2_d_b;
  int32_T c2_c_c;
  int32_T c2_f_a;
  int32_T c2_e_b;
  int32_T c2_qjj;
  int32_T c2_i433;
  real_T c2_d_A[450];
  int32_T c2_i434;
  real_T c2_e_A[450];
  real_T c2_t;
  int32_T c2_c_q;
  boolean_T c2_b_overflow;
  int32_T c2_ii;
  int32_T c2_b_ii;
  int32_T c2_f_b;
  int32_T c2_pmq;
  int32_T c2_i435;
  real_T c2_b_e[18];
  real_T c2_b_absx;
  real_T c2_b_d;
  real_T c2_b_y;
  real_T c2_d11;
  int32_T c2_c_qp1;
  boolean_T c2_c_overflow;
  int32_T c2_c_ii;
  int32_T c2_d_qp1;
  boolean_T c2_d_overflow;
  int32_T c2_c_jj;
  int32_T c2_g_a;
  int32_T c2_d_c;
  int32_T c2_g_b;
  int32_T c2_e_c;
  int32_T c2_h_a;
  int32_T c2_h_b;
  int32_T c2_qp1jj;
  int32_T c2_i436;
  real_T c2_f_A[450];
  int32_T c2_e_qp1;
  boolean_T c2_e_overflow;
  int32_T c2_d_jj;
  int32_T c2_i_a;
  int32_T c2_f_c;
  int32_T c2_i_b;
  int32_T c2_g_c;
  int32_T c2_j_a;
  int32_T c2_j_b;
  int32_T c2_i437;
  real_T c2_b_work[25];
  int32_T c2_f_qp1;
  boolean_T c2_f_overflow;
  int32_T c2_d_ii;
  int32_T c2_m;
  int32_T c2_d_q;
  int32_T c2_k_a;
  int32_T c2_k_b;
  int32_T c2_l_a;
  int32_T c2_m_a;
  int32_T c2_h_c;
  int32_T c2_l_b;
  int32_T c2_i_c;
  int32_T c2_n_a;
  int32_T c2_m_b;
  int32_T c2_g_qp1;
  boolean_T c2_g_overflow;
  int32_T c2_e_jj;
  int32_T c2_o_a;
  int32_T c2_j_c;
  int32_T c2_n_b;
  int32_T c2_k_c;
  int32_T c2_p_a;
  int32_T c2_o_b;
  int32_T c2_i438;
  real_T c2_b_U[450];
  int32_T c2_i439;
  real_T c2_c_U[450];
  int32_T c2_e_q;
  boolean_T c2_h_overflow;
  int32_T c2_e_ii;
  int32_T c2_q_a;
  int32_T c2_i440;
  int32_T c2_p_b;
  int32_T c2_q_b;
  boolean_T c2_i_overflow;
  int32_T c2_f_ii;
  int32_T c2_g_ii;
  int32_T c2_f_q;
  int32_T c2_r_a;
  int32_T c2_r_b;
  int32_T c2_s_a;
  int32_T c2_l_c;
  int32_T c2_s_b;
  int32_T c2_m_c;
  int32_T c2_t_a;
  int32_T c2_t_b;
  int32_T c2_qp1q;
  int32_T c2_h_qp1;
  boolean_T c2_j_overflow;
  int32_T c2_f_jj;
  int32_T c2_u_a;
  int32_T c2_n_c;
  int32_T c2_u_b;
  int32_T c2_o_c;
  int32_T c2_v_a;
  int32_T c2_v_b;
  int32_T c2_i441;
  real_T c2_b_Vf[324];
  int32_T c2_i442;
  real_T c2_c_Vf[324];
  int32_T c2_h_ii;
  int32_T c2_g_q;
  real_T c2_rt;
  real_T c2_r;
  int32_T c2_w_a;
  int32_T c2_p_c;
  int32_T c2_w_b;
  int32_T c2_q_c;
  int32_T c2_x_b;
  int32_T c2_colq;
  int32_T c2_i443;
  int32_T c2_x_a;
  int32_T c2_r_c;
  int32_T c2_y_a;
  int32_T c2_s_c;
  real_T c2_ab_a;
  real_T c2_y_b;
  real_T c2_c_y;
  int32_T c2_ab_b;
  int32_T c2_t_c;
  int32_T c2_bb_b;
  int32_T c2_colqp1;
  real_T c2_iter;
  real_T c2_tiny;
  real_T c2_snorm;
  int32_T c2_i_ii;
  real_T c2_varargin_1;
  real_T c2_varargin_2;
  real_T c2_b_varargin_2;
  real_T c2_varargin_3;
  real_T c2_x;
  real_T c2_d_y;
  real_T c2_b_x;
  real_T c2_e_y;
  real_T c2_xk;
  real_T c2_yk;
  real_T c2_c_x;
  real_T c2_f_y;
  real_T c2_maxval;
  real_T c2_b_varargin_1;
  real_T c2_c_varargin_2;
  real_T c2_d_varargin_2;
  real_T c2_b_varargin_3;
  real_T c2_d_x;
  real_T c2_g_y;
  real_T c2_e_x;
  real_T c2_h_y;
  real_T c2_b_xk;
  real_T c2_b_yk;
  real_T c2_f_x;
  real_T c2_i_y;
  int32_T c2_bb_a;
  int32_T c2_cb_a;
  int32_T c2_i444;
  boolean_T c2_k_overflow;
  int32_T c2_j_ii;
  int32_T c2_db_a;
  int32_T c2_u_c;
  real_T c2_test0;
  real_T c2_ztest0;
  real_T c2_cb_b;
  real_T c2_j_y;
  real_T c2_db_b;
  real_T c2_k_y;
  int32_T c2_eb_a;
  int32_T c2_v_c;
  real_T c2_kase;
  int32_T c2_qs;
  int32_T c2_b_m;
  int32_T c2_h_q;
  int32_T c2_fb_a;
  int32_T c2_eb_b;
  int32_T c2_gb_a;
  int32_T c2_fb_b;
  boolean_T c2_l_overflow;
  int32_T c2_k_ii;
  real_T c2_test;
  int32_T c2_hb_a;
  int32_T c2_w_c;
  int32_T c2_ib_a;
  int32_T c2_x_c;
  real_T c2_ztest;
  real_T c2_gb_b;
  real_T c2_l_y;
  int32_T c2_jb_a;
  int32_T c2_kb_a;
  int32_T c2_y_c;
  real_T c2_f;
  int32_T c2_lb_a;
  int32_T c2_ab_c;
  int32_T c2_mb_a;
  int32_T c2_i445;
  int32_T c2_i_q;
  int32_T c2_nb_a;
  int32_T c2_hb_b;
  int32_T c2_ob_a;
  int32_T c2_ib_b;
  boolean_T c2_m_overflow;
  int32_T c2_k;
  int32_T c2_b_k;
  real_T c2_t1;
  real_T c2_b_t1;
  real_T c2_b_f;
  real_T c2_sn;
  real_T c2_cs;
  real_T c2_b_cs;
  real_T c2_b_sn;
  int32_T c2_pb_a;
  int32_T c2_km1;
  real_T c2_qb_a;
  real_T c2_jb_b;
  real_T c2_rb_a;
  real_T c2_kb_b;
  real_T c2_m_y;
  int32_T c2_sb_a;
  int32_T c2_bb_c;
  int32_T c2_lb_b;
  int32_T c2_cb_c;
  int32_T c2_mb_b;
  int32_T c2_colk;
  int32_T c2_tb_a;
  int32_T c2_db_c;
  int32_T c2_nb_b;
  int32_T c2_eb_c;
  int32_T c2_ob_b;
  int32_T c2_colm;
  int32_T c2_ub_a;
  int32_T c2_j_q;
  int32_T c2_c_m;
  int32_T c2_vb_a;
  int32_T c2_pb_b;
  int32_T c2_wb_a;
  int32_T c2_qb_b;
  boolean_T c2_n_overflow;
  int32_T c2_c_k;
  real_T c2_c_t1;
  real_T c2_unusedU0;
  real_T c2_c_sn;
  real_T c2_c_cs;
  real_T c2_xb_a;
  real_T c2_rb_b;
  real_T c2_yb_a;
  real_T c2_sb_b;
  real_T c2_n_y;
  int32_T c2_ac_a;
  int32_T c2_fb_c;
  int32_T c2_tb_b;
  int32_T c2_gb_c;
  int32_T c2_ub_b;
  int32_T c2_bc_a;
  int32_T c2_hb_c;
  int32_T c2_vb_b;
  int32_T c2_ib_c;
  int32_T c2_wb_b;
  int32_T c2_colqm1;
  int32_T c2_cc_a;
  int32_T c2_mm1;
  real_T c2_d12;
  real_T c2_d13;
  real_T c2_d14;
  real_T c2_d15;
  real_T c2_d16;
  real_T c2_c_varargin_1[5];
  int32_T c2_ixstart;
  real_T c2_mtmp;
  real_T c2_g_x;
  boolean_T c2_xb_b;
  int32_T c2_ix;
  int32_T c2_b_ix;
  real_T c2_h_x;
  boolean_T c2_yb_b;
  int32_T c2_dc_a;
  int32_T c2_i446;
  boolean_T c2_o_overflow;
  int32_T c2_c_ix;
  real_T c2_ec_a;
  real_T c2_ac_b;
  boolean_T c2_p;
  real_T c2_b_mtmp;
  real_T c2_scale;
  real_T c2_sm;
  real_T c2_smm1;
  real_T c2_emm1;
  real_T c2_sqds;
  real_T c2_eqds;
  real_T c2_fc_a;
  real_T c2_bc_b;
  real_T c2_o_y;
  real_T c2_gc_a;
  real_T c2_cc_b;
  real_T c2_p_y;
  real_T c2_dc_b;
  real_T c2_hc_a;
  real_T c2_ec_b;
  real_T c2_jb_c;
  real_T c2_ic_a;
  real_T c2_fc_b;
  real_T c2_shift;
  real_T c2_jc_a;
  real_T c2_gc_b;
  real_T c2_q_y;
  real_T c2_kc_a;
  real_T c2_hc_b;
  real_T c2_r_y;
  real_T c2_lc_a;
  real_T c2_ic_b;
  real_T c2_g;
  int32_T c2_k_q;
  int32_T c2_b_mm1;
  int32_T c2_mc_a;
  int32_T c2_jc_b;
  int32_T c2_nc_a;
  int32_T c2_kc_b;
  boolean_T c2_p_overflow;
  int32_T c2_d_k;
  int32_T c2_oc_a;
  int32_T c2_pc_a;
  int32_T c2_kp1;
  real_T c2_c_f;
  real_T c2_unusedU1;
  real_T c2_d_sn;
  real_T c2_d_cs;
  real_T c2_qc_a;
  real_T c2_lc_b;
  real_T c2_s_y;
  real_T c2_rc_a;
  real_T c2_mc_b;
  real_T c2_t_y;
  real_T c2_sc_a;
  real_T c2_nc_b;
  real_T c2_u_y;
  real_T c2_tc_a;
  real_T c2_oc_b;
  real_T c2_v_y;
  real_T c2_uc_a;
  real_T c2_pc_b;
  real_T c2_vc_a;
  real_T c2_qc_b;
  real_T c2_w_y;
  int32_T c2_wc_a;
  int32_T c2_kb_c;
  int32_T c2_rc_b;
  int32_T c2_lb_c;
  int32_T c2_sc_b;
  int32_T c2_tc_b;
  int32_T c2_mb_c;
  int32_T c2_uc_b;
  int32_T c2_colkp1;
  real_T c2_d_f;
  real_T c2_unusedU2;
  real_T c2_e_sn;
  real_T c2_e_cs;
  real_T c2_xc_a;
  real_T c2_vc_b;
  real_T c2_x_y;
  real_T c2_yc_a;
  real_T c2_wc_b;
  real_T c2_y_y;
  real_T c2_ad_a;
  real_T c2_xc_b;
  real_T c2_ab_y;
  real_T c2_bd_a;
  real_T c2_yc_b;
  real_T c2_bb_y;
  real_T c2_cd_a;
  real_T c2_ad_b;
  real_T c2_dd_a;
  real_T c2_bd_b;
  real_T c2_cb_y;
  int32_T c2_ed_a;
  int32_T c2_nb_c;
  int32_T c2_cd_b;
  int32_T c2_ob_c;
  int32_T c2_dd_b;
  int32_T c2_ed_b;
  int32_T c2_pb_c;
  int32_T c2_fd_b;
  int32_T c2_fd_a;
  int32_T c2_qb_c;
  int32_T c2_e_k;
  int32_T c2_j;
  int32_T c2_b_j;
  int32_T c2_i;
  int32_T c2_b_i;
  int32_T c2_rb_c;
  int32_T c2_gd_a;
  int32_T c2_sb_c;
  int32_T c2_gd_b;
  int32_T c2_hd_b;
  int32_T c2_hd_a;
  int32_T c2_id_a;
  int32_T c2_tb_c;
  int32_T c2_jd_a;
  int32_T c2_ub_c;
  int32_T c2_id_b;
  int32_T c2_jd_b;
  int32_T c2_vb_c;
  int32_T c2_kd_b;
  int32_T c2_ld_b;
  int32_T c2_wb_c;
  int32_T c2_kd_a;
  int32_T c2_xb_c;
  int32_T c2_md_b;
  int32_T c2_nd_b;
  int32_T c2_yb_c;
  int32_T c2_od_b;
  int32_T c2_pd_b;
  int32_T c2_ld_a;
  real_T c2_d17;
  boolean_T guard1 = FALSE;
  boolean_T guard2 = FALSE;
  boolean_T guard3 = FALSE;
  boolean_T guard4 = FALSE;
  boolean_T exitg1;
  boolean_T exitg2;
  boolean_T exitg3;
  boolean_T exitg4;
  boolean_T exitg5;
  boolean_T guard11 = FALSE;
  for (c2_i426 = 0; c2_i426 < 450; c2_i426++) {
    c2_b_A[c2_i426] = c2_A[c2_i426];
  }

  c2_i_eml_scalar_eg(chartInstance);
  for (c2_i427 = 0; c2_i427 < 18; c2_i427++) {
    c2_s[c2_i427] = 0.0;
  }

  for (c2_i428 = 0; c2_i428 < 18; c2_i428++) {
    c2_e[c2_i428] = 0.0;
  }

  for (c2_i429 = 0; c2_i429 < 25; c2_i429++) {
    c2_work[c2_i429] = 0.0;
  }

  for (c2_i430 = 0; c2_i430 < 450; c2_i430++) {
    c2_U[c2_i430] = 0.0;
  }

  for (c2_i431 = 0; c2_i431 < 324; c2_i431++) {
    c2_Vf[c2_i431] = 0.0;
  }

  c2_b_check_forloop_overflow_error(chartInstance, FALSE);
  for (c2_q = 1; c2_q < 19; c2_q++) {
    c2_b_q = c2_q;
    c2_a = c2_b_q + 1;
    c2_qp1 = c2_a;
    c2_b_a = c2_b_q - 1;
    c2_qm1 = c2_b_a;
    c2_b = c2_qm1;
    c2_c = 25 * c2_b;
    c2_c_a = c2_b_q;
    c2_b_b = c2_c;
    c2_qq = c2_c_a + c2_b_b;
    c2_c_b = c2_b_q;
    c2_nmq = 25 - c2_c_b;
    c2_d_a = c2_nmq + 1;
    c2_nmqp1 = c2_d_a;
    if (c2_b_q <= 18) {
      for (c2_i432 = 0; c2_i432 < 450; c2_i432++) {
        c2_c_A[c2_i432] = c2_b_A[c2_i432];
      }

      c2_nrm = c2_c_eml_xnrm2(chartInstance, c2_nmqp1, c2_c_A, c2_qq);
      if (c2_nrm > 0.0) {
        c2_absx = c2_nrm;
        c2_d = c2_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c2_qq), 1, 450, 1, 0) - 1];
        if (c2_d < 0.0) {
          c2_y = -c2_absx;
        } else {
          c2_y = c2_absx;
        }

        c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_b_q), 1, 18, 1, 0) - 1] = c2_y;
        c2_d10 = c2_eml_div(chartInstance, 1.0, c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK
                            ("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_q),
                             1, 18, 1, 0) - 1]);
        c2_j_eml_xscal(chartInstance, c2_nmqp1, c2_d10, c2_b_A, c2_qq);
        c2_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_qq), 1, 450, 1, 0) - 1] =
          c2_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_qq), 1, 450, 1, 0) - 1] + 1.0;
        c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_b_q), 1, 18, 1, 0) - 1] = -c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK
          ("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_q), 1, 18, 1, 0) - 1];
      } else {
        c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_b_q), 1, 18, 1, 0) - 1] = 0.0;
      }
    }

    c2_b_qp1 = c2_qp1;
    c2_overflow = FALSE;
    c2_b_check_forloop_overflow_error(chartInstance, c2_overflow);
    for (c2_jj = c2_b_qp1; c2_jj < 19; c2_jj++) {
      c2_b_jj = c2_jj;
      c2_e_a = c2_b_jj - 1;
      c2_b_c = c2_e_a;
      c2_d_b = c2_b_c;
      c2_c_c = 25 * c2_d_b;
      c2_f_a = c2_b_q;
      c2_e_b = c2_c_c;
      c2_qjj = c2_f_a + c2_e_b;
      if (c2_b_q <= 18) {
        if (c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c2_b_q), 1, 18, 1, 0) - 1] != 0.0) {
          for (c2_i433 = 0; c2_i433 < 450; c2_i433++) {
            c2_d_A[c2_i433] = c2_b_A[c2_i433];
          }

          for (c2_i434 = 0; c2_i434 < 450; c2_i434++) {
            c2_e_A[c2_i434] = c2_b_A[c2_i434];
          }

          c2_t = c2_c_eml_xdotc(chartInstance, c2_nmqp1, c2_d_A, c2_qq, c2_e_A,
                                c2_qjj);
          c2_t = -c2_eml_div(chartInstance, c2_t, c2_b_A
                             [(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c2_b_q), 1, 25, 1, 0) + 25 *
                               (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c2_b_q), 1, 18, 2, 0) - 1)) - 1]);
          c2_m_eml_xaxpy(chartInstance, c2_nmqp1, c2_t, c2_qq, c2_b_A, c2_qjj);
        }
      }

      c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c2_b_jj), 1, 18, 1, 0) - 1] = c2_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK
        ("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_qjj), 1, 450, 1, 0) - 1];
    }

    if (c2_b_q <= 18) {
      c2_c_q = c2_b_q;
      c2_b_overflow = FALSE;
      c2_b_check_forloop_overflow_error(chartInstance, c2_b_overflow);
      for (c2_ii = c2_c_q; c2_ii < 26; c2_ii++) {
        c2_b_ii = c2_ii;
        c2_U[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                (real_T)c2_b_ii), 1, 25, 1, 0) + 25 *
              (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                 (real_T)c2_b_q), 1, 18, 2, 0) - 1)) - 1] = c2_b_A
          [(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c2_b_ii), 1, 25, 1, 0) + 25 * (_SFD_EML_ARRAY_BOUNDS_CHECK
             ("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_q), 1, 18, 2, 0)
             - 1)) - 1];
      }
    }

    if (c2_b_q <= 16) {
      c2_f_b = c2_b_q;
      c2_pmq = 18 - c2_f_b;
      for (c2_i435 = 0; c2_i435 < 18; c2_i435++) {
        c2_b_e[c2_i435] = c2_e[c2_i435];
      }

      c2_nrm = c2_d_eml_xnrm2(chartInstance, c2_pmq, c2_b_e, c2_qp1);
      if (c2_nrm == 0.0) {
        c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_b_q), 1, 18, 1, 0) - 1] = 0.0;
      } else {
        c2_b_absx = c2_nrm;
        c2_b_d = c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c2_qp1), 1, 18, 1, 0) - 1];
        if (c2_b_d < 0.0) {
          c2_b_y = -c2_b_absx;
        } else {
          c2_b_y = c2_b_absx;
        }

        c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_b_q), 1, 18, 1, 0) - 1] = c2_b_y;
        c2_d11 = c2_eml_div(chartInstance, 1.0, c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK
                            ("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_q),
                             1, 18, 1, 0) - 1]);
        c2_k_eml_xscal(chartInstance, c2_pmq, c2_d11, c2_e, c2_qp1);
        c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_qp1), 1, 18, 1, 0) - 1] = c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK(
          "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_qp1), 1, 18, 1, 0) - 1]
          + 1.0;
      }

      c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c2_b_q), 1, 18, 1, 0) - 1] = -c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK(
        "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_q), 1, 18, 1, 0) - 1];
      if (c2_qp1 <= 25) {
        if (c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c2_b_q), 1, 18, 1, 0) - 1] != 0.0) {
          c2_c_qp1 = c2_qp1;
          c2_c_overflow = FALSE;
          c2_b_check_forloop_overflow_error(chartInstance, c2_c_overflow);
          for (c2_c_ii = c2_c_qp1; c2_c_ii < 26; c2_c_ii++) {
            c2_b_ii = c2_c_ii;
            c2_work[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
              "", (real_T)c2_b_ii), 1, 25, 1, 0) - 1] = 0.0;
          }

          c2_d_qp1 = c2_qp1;
          c2_d_overflow = FALSE;
          c2_b_check_forloop_overflow_error(chartInstance, c2_d_overflow);
          for (c2_c_jj = c2_d_qp1; c2_c_jj < 19; c2_c_jj++) {
            c2_b_jj = c2_c_jj;
            c2_g_a = c2_b_jj - 1;
            c2_d_c = c2_g_a;
            c2_g_b = c2_d_c;
            c2_e_c = 25 * c2_g_b;
            c2_h_a = c2_qp1;
            c2_h_b = c2_e_c;
            c2_qp1jj = c2_h_a + c2_h_b;
            for (c2_i436 = 0; c2_i436 < 450; c2_i436++) {
              c2_f_A[c2_i436] = c2_b_A[c2_i436];
            }

            c2_n_eml_xaxpy(chartInstance, c2_nmq,
                           c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
              _SFD_INTEGER_CHECK("", (real_T)c2_b_jj), 1, 18, 1, 0) - 1], c2_f_A,
                           c2_qp1jj, c2_work, c2_qp1);
          }

          c2_e_qp1 = c2_qp1;
          c2_e_overflow = FALSE;
          c2_b_check_forloop_overflow_error(chartInstance, c2_e_overflow);
          for (c2_d_jj = c2_e_qp1; c2_d_jj < 19; c2_d_jj++) {
            c2_b_jj = c2_d_jj;
            c2_t = c2_eml_div(chartInstance, -c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK(
              "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_jj), 1, 18, 1, 0)
                              - 1], c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("",
              (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_qp1), 1, 18, 1, 0) - 1]);
            c2_i_a = c2_b_jj - 1;
            c2_f_c = c2_i_a;
            c2_i_b = c2_f_c;
            c2_g_c = 25 * c2_i_b;
            c2_j_a = c2_qp1;
            c2_j_b = c2_g_c;
            c2_qp1jj = c2_j_a + c2_j_b;
            for (c2_i437 = 0; c2_i437 < 25; c2_i437++) {
              c2_b_work[c2_i437] = c2_work[c2_i437];
            }

            c2_o_eml_xaxpy(chartInstance, c2_nmq, c2_t, c2_b_work, c2_qp1,
                           c2_b_A, c2_qp1jj);
          }
        }
      }

      c2_f_qp1 = c2_qp1;
      c2_f_overflow = FALSE;
      c2_b_check_forloop_overflow_error(chartInstance, c2_f_overflow);
      for (c2_d_ii = c2_f_qp1; c2_d_ii < 19; c2_d_ii++) {
        c2_b_ii = c2_d_ii;
        c2_Vf[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                 (real_T)c2_b_ii), 1, 18, 1, 0) + 18 *
               (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                  (real_T)c2_b_q), 1, 18, 2, 0) - 1)) - 1] =
          c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_b_ii), 1, 18, 1, 0) - 1];
      }
    }
  }

  c2_m = 18;
  c2_e[16] = c2_b_A[441];
  c2_e[17] = 0.0;
  c2_b_check_forloop_overflow_error(chartInstance, FALSE);
  for (c2_d_q = 18; c2_d_q > 0; c2_d_q--) {
    c2_b_q = c2_d_q;
    c2_k_a = c2_b_q + 1;
    c2_qp1 = c2_k_a;
    c2_k_b = c2_b_q;
    c2_nmq = 25 - c2_k_b;
    c2_l_a = c2_nmq + 1;
    c2_nmqp1 = c2_l_a;
    c2_m_a = c2_b_q - 1;
    c2_h_c = c2_m_a;
    c2_l_b = c2_h_c;
    c2_i_c = 25 * c2_l_b;
    c2_n_a = c2_b_q;
    c2_m_b = c2_i_c;
    c2_qq = c2_n_a + c2_m_b;
    if (c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_b_q), 1, 18, 1, 0) - 1] != 0.0) {
      c2_g_qp1 = c2_qp1;
      c2_g_overflow = FALSE;
      c2_b_check_forloop_overflow_error(chartInstance, c2_g_overflow);
      for (c2_e_jj = c2_g_qp1; c2_e_jj < 19; c2_e_jj++) {
        c2_b_jj = c2_e_jj;
        c2_o_a = c2_b_jj - 1;
        c2_j_c = c2_o_a;
        c2_n_b = c2_j_c;
        c2_k_c = 25 * c2_n_b;
        c2_p_a = c2_b_q;
        c2_o_b = c2_k_c;
        c2_qjj = c2_p_a + c2_o_b;
        for (c2_i438 = 0; c2_i438 < 450; c2_i438++) {
          c2_b_U[c2_i438] = c2_U[c2_i438];
        }

        for (c2_i439 = 0; c2_i439 < 450; c2_i439++) {
          c2_c_U[c2_i439] = c2_U[c2_i439];
        }

        c2_t = c2_c_eml_xdotc(chartInstance, c2_nmqp1, c2_b_U, c2_qq, c2_c_U,
                              c2_qjj);
        c2_t = -c2_eml_div(chartInstance, c2_t, c2_U[_SFD_EML_ARRAY_BOUNDS_CHECK
                           ("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_qq),
                            1, 450, 1, 0) - 1]);
        c2_m_eml_xaxpy(chartInstance, c2_nmqp1, c2_t, c2_qq, c2_U, c2_qjj);
      }

      c2_e_q = c2_b_q;
      c2_h_overflow = FALSE;
      c2_b_check_forloop_overflow_error(chartInstance, c2_h_overflow);
      for (c2_e_ii = c2_e_q; c2_e_ii < 26; c2_e_ii++) {
        c2_b_ii = c2_e_ii;
        c2_U[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                (real_T)c2_b_ii), 1, 25, 1, 0) + 25 *
              (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                 (real_T)c2_b_q), 1, 18, 2, 0) - 1)) - 1] = -c2_U
          [(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c2_b_ii), 1, 25, 1, 0) + 25 * (_SFD_EML_ARRAY_BOUNDS_CHECK
             ("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_q), 1, 18, 2, 0)
             - 1)) - 1];
      }

      c2_U[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c2_qq), 1, 450, 1, 0) - 1] = c2_U[_SFD_EML_ARRAY_BOUNDS_CHECK("",
        (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_qq), 1, 450, 1, 0) - 1] + 1.0;
      c2_q_a = c2_b_q - 1;
      c2_i440 = c2_q_a;
      c2_p_b = c2_i440;
      c2_q_b = c2_p_b;
      if (1 > c2_q_b) {
        c2_i_overflow = FALSE;
      } else {
        c2_i_overflow = (c2_q_b > 2147483646);
      }

      c2_b_check_forloop_overflow_error(chartInstance, c2_i_overflow);
      for (c2_f_ii = 1; c2_f_ii <= c2_i440; c2_f_ii++) {
        c2_b_ii = c2_f_ii;
        c2_U[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                (real_T)c2_b_ii), 1, 25, 1, 0) + 25 *
              (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                 (real_T)c2_b_q), 1, 18, 2, 0) - 1)) - 1] = 0.0;
      }
    } else {
      c2_check_forloop_overflow_error(chartInstance);
      for (c2_g_ii = 1; c2_g_ii < 26; c2_g_ii++) {
        c2_b_ii = c2_g_ii;
        c2_U[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                (real_T)c2_b_ii), 1, 25, 1, 0) + 25 *
              (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                 (real_T)c2_b_q), 1, 18, 2, 0) - 1)) - 1] = 0.0;
      }

      c2_U[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c2_qq), 1, 450, 1, 0) - 1] = 1.0;
    }
  }

  c2_b_check_forloop_overflow_error(chartInstance, FALSE);
  for (c2_f_q = 18; c2_f_q > 0; c2_f_q--) {
    c2_b_q = c2_f_q;
    if (c2_b_q <= 16) {
      if (c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_b_q), 1, 18, 1, 0) - 1] != 0.0) {
        c2_r_a = c2_b_q + 1;
        c2_qp1 = c2_r_a;
        c2_r_b = c2_b_q;
        c2_pmq = 18 - c2_r_b;
        c2_s_a = c2_b_q - 1;
        c2_l_c = c2_s_a;
        c2_s_b = c2_l_c;
        c2_m_c = 18 * c2_s_b;
        c2_t_a = c2_qp1;
        c2_t_b = c2_m_c;
        c2_qp1q = c2_t_a + c2_t_b;
        c2_h_qp1 = c2_qp1;
        c2_j_overflow = FALSE;
        c2_b_check_forloop_overflow_error(chartInstance, c2_j_overflow);
        for (c2_f_jj = c2_h_qp1; c2_f_jj < 19; c2_f_jj++) {
          c2_b_jj = c2_f_jj;
          c2_u_a = c2_b_jj - 1;
          c2_n_c = c2_u_a;
          c2_u_b = c2_n_c;
          c2_o_c = 18 * c2_u_b;
          c2_v_a = c2_qp1;
          c2_v_b = c2_o_c;
          c2_qp1jj = c2_v_a + c2_v_b;
          for (c2_i441 = 0; c2_i441 < 324; c2_i441++) {
            c2_b_Vf[c2_i441] = c2_Vf[c2_i441];
          }

          for (c2_i442 = 0; c2_i442 < 324; c2_i442++) {
            c2_c_Vf[c2_i442] = c2_Vf[c2_i442];
          }

          c2_t = c2_d_eml_xdotc(chartInstance, c2_pmq, c2_b_Vf, c2_qp1q, c2_c_Vf,
                                c2_qp1jj);
          c2_t = -c2_eml_div(chartInstance, c2_t,
                             c2_Vf[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c2_qp1q), 1, 324, 1, 0) - 1]);
          c2_p_eml_xaxpy(chartInstance, c2_pmq, c2_t, c2_qp1q, c2_Vf, c2_qp1jj);
        }
      }
    }

    c2_b_check_forloop_overflow_error(chartInstance, FALSE);
    for (c2_h_ii = 1; c2_h_ii < 19; c2_h_ii++) {
      c2_b_ii = c2_h_ii;
      c2_Vf[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
               (real_T)c2_b_ii), 1, 18, 1, 0) + 18 *
             (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                (real_T)c2_b_q), 1, 18, 2, 0) - 1)) - 1] = 0.0;
    }

    c2_Vf[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
             (real_T)c2_b_q), 1, 18, 1, 0) + 18 * (_SFD_EML_ARRAY_BOUNDS_CHECK(
             "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_q), 1, 18, 2, 0) -
            1)) - 1] = 1.0;
  }

  c2_b_check_forloop_overflow_error(chartInstance, FALSE);
  for (c2_g_q = 1; c2_g_q < 19; c2_g_q++) {
    c2_b_q = c2_g_q;
    if (c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_b_q), 1, 18, 1, 0) - 1] != 0.0) {
      c2_rt = c2_abs(chartInstance, c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
        (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_q), 1, 18, 1, 0) - 1]);
      c2_r = c2_eml_div(chartInstance, c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
        (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_q), 1, 18, 1, 0) - 1],
                        c2_rt);
      c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c2_b_q), 1, 18, 1, 0) - 1] = c2_rt;
      if (c2_b_q < 18) {
        c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_b_q), 1, 18, 1, 0) - 1] = c2_eml_div(chartInstance,
          c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_b_q), 1, 18, 1, 0) - 1], c2_r);
      }

      if (c2_b_q <= 25) {
        c2_w_a = c2_b_q - 1;
        c2_p_c = c2_w_a;
        c2_w_b = c2_p_c;
        c2_q_c = 25 * c2_w_b;
        c2_x_b = c2_q_c + 1;
        c2_colq = c2_x_b;
        c2_i443 = 25;
        c2_j_eml_xscal(chartInstance, c2_i443, c2_r, c2_U, c2_colq);
      }
    }

    if (c2_b_q < 18) {
      if (c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_b_q), 1, 18, 1, 0) - 1] != 0.0) {
        c2_rt = c2_abs(chartInstance, c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_q), 1, 18, 1, 0) - 1]);
        c2_r = c2_eml_div(chartInstance, c2_rt, c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK
                          ("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_q),
                           1, 18, 1, 0) - 1]);
        c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_b_q), 1, 18, 1, 0) - 1] = c2_rt;
        c2_x_a = c2_b_q + 1;
        c2_r_c = c2_x_a;
        c2_y_a = c2_b_q + 1;
        c2_s_c = c2_y_a;
        c2_ab_a = c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c2_s_c), 1, 18, 1, 0) - 1];
        c2_y_b = c2_r;
        c2_c_y = c2_ab_a * c2_y_b;
        c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_r_c), 1, 18, 1, 0) - 1] = c2_c_y;
        c2_ab_b = c2_b_q;
        c2_t_c = 18 * c2_ab_b;
        c2_bb_b = c2_t_c + 1;
        c2_colqp1 = c2_bb_b;
        c2_l_eml_xscal(chartInstance, c2_r, c2_Vf, c2_colqp1);
      }
    }
  }

  c2_iter = 0.0;
  c2_realmin(chartInstance);
  c2_eps(chartInstance);
  c2_tiny = c2_eml_div(chartInstance, 2.2250738585072014E-308,
                       2.2204460492503131E-16);
  c2_snorm = 0.0;
  c2_b_check_forloop_overflow_error(chartInstance, FALSE);
  for (c2_i_ii = 1; c2_i_ii < 19; c2_i_ii++) {
    c2_b_ii = c2_i_ii;
    c2_varargin_1 = c2_abs(chartInstance, c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
      (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_ii), 1, 18, 1, 0) - 1]);
    c2_varargin_2 = c2_abs(chartInstance, c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("",
      (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_ii), 1, 18, 1, 0) - 1]);
    c2_b_varargin_2 = c2_varargin_1;
    c2_varargin_3 = c2_varargin_2;
    c2_x = c2_b_varargin_2;
    c2_d_y = c2_varargin_3;
    c2_b_x = c2_x;
    c2_e_y = c2_d_y;
    c2_eml_scalar_eg(chartInstance);
    c2_xk = c2_b_x;
    c2_yk = c2_e_y;
    c2_c_x = c2_xk;
    c2_f_y = c2_yk;
    c2_eml_scalar_eg(chartInstance);
    c2_maxval = muDoubleScalarMax(c2_c_x, c2_f_y);
    c2_b_varargin_1 = c2_snorm;
    c2_c_varargin_2 = c2_maxval;
    c2_d_varargin_2 = c2_b_varargin_1;
    c2_b_varargin_3 = c2_c_varargin_2;
    c2_d_x = c2_d_varargin_2;
    c2_g_y = c2_b_varargin_3;
    c2_e_x = c2_d_x;
    c2_h_y = c2_g_y;
    c2_eml_scalar_eg(chartInstance);
    c2_b_xk = c2_e_x;
    c2_b_yk = c2_h_y;
    c2_f_x = c2_b_xk;
    c2_i_y = c2_b_yk;
    c2_eml_scalar_eg(chartInstance);
    c2_snorm = muDoubleScalarMax(c2_f_x, c2_i_y);
  }

  exitg1 = FALSE;
  while ((exitg1 == FALSE) && (c2_m > 0)) {
    if (c2_iter >= 75.0) {
      c2_b_eml_error(chartInstance);
      exitg1 = TRUE;
    } else {
      c2_bb_a = c2_m - 1;
      c2_b_q = c2_bb_a;
      c2_cb_a = c2_m - 1;
      c2_i444 = c2_cb_a;
      c2_k_overflow = FALSE;
      c2_b_check_forloop_overflow_error(chartInstance, c2_k_overflow);
      c2_j_ii = c2_i444;
      guard3 = FALSE;
      guard4 = FALSE;
      exitg5 = FALSE;
      while ((exitg5 == FALSE) && (c2_j_ii > -1)) {
        c2_b_ii = c2_j_ii;
        c2_b_q = c2_b_ii;
        if (c2_b_ii == 0) {
          exitg5 = TRUE;
        } else {
          c2_db_a = c2_b_ii + 1;
          c2_u_c = c2_db_a;
          c2_test0 = c2_abs(chartInstance, c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
                             (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_ii), 1,
            18, 1, 0) - 1]) + c2_abs(chartInstance,
            c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                                       (real_T)c2_u_c), 1, 18, 1, 0) - 1]);
          c2_ztest0 = c2_abs(chartInstance, c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("",
                              (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_ii),
            1, 18, 1, 0) - 1]);
          c2_eps(chartInstance);
          c2_cb_b = c2_test0;
          c2_j_y = 2.2204460492503131E-16 * c2_cb_b;
          if (c2_ztest0 <= c2_j_y) {
            guard4 = TRUE;
            exitg5 = TRUE;
          } else if (c2_ztest0 <= c2_tiny) {
            guard4 = TRUE;
            exitg5 = TRUE;
          } else {
            guard11 = FALSE;
            if (c2_iter > 20.0) {
              c2_eps(chartInstance);
              c2_db_b = c2_snorm;
              c2_k_y = 2.2204460492503131E-16 * c2_db_b;
              if (c2_ztest0 <= c2_k_y) {
                guard3 = TRUE;
                exitg5 = TRUE;
              } else {
                guard11 = TRUE;
              }
            } else {
              guard11 = TRUE;
            }

            if (guard11 == TRUE) {
              c2_j_ii--;
              guard3 = FALSE;
              guard4 = FALSE;
            }
          }
        }
      }

      if (guard4 == TRUE) {
        guard3 = TRUE;
      }

      if (guard3 == TRUE) {
        c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_b_ii), 1, 18, 1, 0) - 1] = 0.0;
      }

      c2_eb_a = c2_m - 1;
      c2_v_c = c2_eb_a;
      if (c2_b_q == c2_v_c) {
        c2_kase = 4.0;
      } else {
        c2_qs = c2_m;
        c2_b_m = c2_m;
        c2_h_q = c2_b_q;
        c2_fb_a = c2_b_m;
        c2_eb_b = c2_h_q;
        c2_gb_a = c2_fb_a;
        c2_fb_b = c2_eb_b;
        if (c2_gb_a < c2_fb_b) {
          c2_l_overflow = FALSE;
        } else {
          c2_l_overflow = (c2_fb_b < -2147483647);
        }

        c2_b_check_forloop_overflow_error(chartInstance, c2_l_overflow);
        c2_k_ii = c2_b_m;
        guard2 = FALSE;
        exitg4 = FALSE;
        while ((exitg4 == FALSE) && (c2_k_ii >= c2_h_q)) {
          c2_b_ii = c2_k_ii;
          c2_qs = c2_b_ii;
          if (c2_b_ii == c2_b_q) {
            exitg4 = TRUE;
          } else {
            c2_test = 0.0;
            if (c2_b_ii < c2_m) {
              c2_test = c2_abs(chartInstance, c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK(
                "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_ii), 1, 18, 1,
                0) - 1]);
            }

            c2_hb_a = c2_b_q + 1;
            c2_w_c = c2_hb_a;
            if (c2_b_ii > c2_w_c) {
              c2_ib_a = c2_b_ii - 1;
              c2_x_c = c2_ib_a;
              c2_test += c2_abs(chartInstance, c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK(
                "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_x_c), 1, 18, 1, 0)
                                - 1]);
            }

            c2_ztest = c2_abs(chartInstance, c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
                               (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_ii),
              1, 18, 1, 0) - 1]);
            c2_eps(chartInstance);
            c2_gb_b = c2_test;
            c2_l_y = 2.2204460492503131E-16 * c2_gb_b;
            if (c2_ztest <= c2_l_y) {
              guard2 = TRUE;
              exitg4 = TRUE;
            } else if (c2_ztest <= c2_tiny) {
              guard2 = TRUE;
              exitg4 = TRUE;
            } else {
              c2_k_ii--;
              guard2 = FALSE;
            }
          }
        }

        if (guard2 == TRUE) {
          c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_b_ii), 1, 18, 1, 0) - 1] = 0.0;
        }

        if (c2_qs == c2_b_q) {
          c2_kase = 3.0;
        } else if (c2_qs == c2_m) {
          c2_kase = 1.0;
        } else {
          c2_kase = 2.0;
          c2_b_q = c2_qs;
        }
      }

      c2_jb_a = c2_b_q + 1;
      c2_b_q = c2_jb_a;
      switch ((int32_T)_SFD_INTEGER_CHECK("", c2_kase)) {
       case 1:
        c2_kb_a = c2_m - 1;
        c2_y_c = c2_kb_a;
        c2_f = c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
          "", (real_T)c2_y_c), 1, 18, 1, 0) - 1];
        c2_lb_a = c2_m - 1;
        c2_ab_c = c2_lb_a;
        c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_ab_c), 1, 18, 1, 0) - 1] = 0.0;
        c2_mb_a = c2_m - 1;
        c2_i445 = c2_mb_a;
        c2_i_q = c2_b_q;
        c2_nb_a = c2_i445;
        c2_hb_b = c2_i_q;
        c2_ob_a = c2_nb_a;
        c2_ib_b = c2_hb_b;
        if (c2_ob_a < c2_ib_b) {
          c2_m_overflow = FALSE;
        } else {
          c2_m_overflow = (c2_ib_b < -2147483647);
        }

        c2_b_check_forloop_overflow_error(chartInstance, c2_m_overflow);
        for (c2_k = c2_i445; c2_k >= c2_i_q; c2_k--) {
          c2_b_k = c2_k;
          c2_t1 = c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c2_b_k), 1, 18, 1, 0) - 1];
          c2_b_t1 = c2_t1;
          c2_b_f = c2_f;
          c2_b_eml_xrotg(chartInstance, &c2_b_t1, &c2_b_f, &c2_cs, &c2_sn);
          c2_t1 = c2_b_t1;
          c2_f = c2_b_f;
          c2_b_cs = c2_cs;
          c2_b_sn = c2_sn;
          c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_b_k), 1, 18, 1, 0) - 1] = c2_t1;
          if (c2_b_k > c2_b_q) {
            c2_pb_a = c2_b_k - 1;
            c2_km1 = c2_pb_a;
            c2_qb_a = -c2_b_sn;
            c2_jb_b = c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
              _SFD_INTEGER_CHECK("", (real_T)c2_km1), 1, 18, 1, 0) - 1];
            c2_f = c2_qb_a * c2_jb_b;
            c2_rb_a = c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
              _SFD_INTEGER_CHECK("", (real_T)c2_km1), 1, 18, 1, 0) - 1];
            c2_kb_b = c2_b_cs;
            c2_m_y = c2_rb_a * c2_kb_b;
            c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c2_km1), 1, 18, 1, 0) - 1] = c2_m_y;
          }

          c2_sb_a = c2_b_k - 1;
          c2_bb_c = c2_sb_a;
          c2_lb_b = c2_bb_c;
          c2_cb_c = 18 * c2_lb_b;
          c2_mb_b = c2_cb_c + 1;
          c2_colk = c2_mb_b;
          c2_tb_a = c2_m - 1;
          c2_db_c = c2_tb_a;
          c2_nb_b = c2_db_c;
          c2_eb_c = 18 * c2_nb_b;
          c2_ob_b = c2_eb_c + 1;
          c2_colm = c2_ob_b;
          c2_g_eml_xrot(chartInstance, c2_Vf, c2_colk, c2_colm, c2_b_cs, c2_b_sn);
        }
        break;

       case 2:
        c2_ub_a = c2_b_q - 1;
        c2_qm1 = c2_ub_a;
        c2_f = c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
          "", (real_T)c2_qm1), 1, 18, 1, 0) - 1];
        c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_qm1), 1, 18, 1, 0) - 1] = 0.0;
        c2_j_q = c2_b_q;
        c2_c_m = c2_m;
        c2_vb_a = c2_j_q;
        c2_pb_b = c2_c_m;
        c2_wb_a = c2_vb_a;
        c2_qb_b = c2_pb_b;
        if (c2_wb_a > c2_qb_b) {
          c2_n_overflow = FALSE;
        } else {
          c2_n_overflow = (c2_qb_b > 2147483646);
        }

        c2_b_check_forloop_overflow_error(chartInstance, c2_n_overflow);
        for (c2_c_k = c2_j_q; c2_c_k <= c2_c_m; c2_c_k++) {
          c2_b_k = c2_c_k;
          c2_t1 = c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c2_b_k), 1, 18, 1, 0) - 1];
          c2_c_t1 = c2_t1;
          c2_unusedU0 = c2_f;
          c2_b_eml_xrotg(chartInstance, &c2_c_t1, &c2_unusedU0, &c2_c_cs,
                         &c2_c_sn);
          c2_t1 = c2_c_t1;
          c2_b_cs = c2_c_cs;
          c2_b_sn = c2_c_sn;
          c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_b_k), 1, 18, 1, 0) - 1] = c2_t1;
          c2_xb_a = -c2_b_sn;
          c2_rb_b = c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c2_b_k), 1, 18, 1, 0) - 1];
          c2_f = c2_xb_a * c2_rb_b;
          c2_yb_a = c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c2_b_k), 1, 18, 1, 0) - 1];
          c2_sb_b = c2_b_cs;
          c2_n_y = c2_yb_a * c2_sb_b;
          c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_b_k), 1, 18, 1, 0) - 1] = c2_n_y;
          c2_ac_a = c2_b_k - 1;
          c2_fb_c = c2_ac_a;
          c2_tb_b = c2_fb_c;
          c2_gb_c = 25 * c2_tb_b;
          c2_ub_b = c2_gb_c + 1;
          c2_colk = c2_ub_b;
          c2_bc_a = c2_qm1 - 1;
          c2_hb_c = c2_bc_a;
          c2_vb_b = c2_hb_c;
          c2_ib_c = 25 * c2_vb_b;
          c2_wb_b = c2_ib_c + 1;
          c2_colqm1 = c2_wb_b;
          c2_h_eml_xrot(chartInstance, c2_U, c2_colk, c2_colqm1, c2_b_cs,
                        c2_b_sn);
        }
        break;

       case 3:
        c2_cc_a = c2_m - 1;
        c2_mm1 = c2_cc_a;
        c2_d12 = c2_abs(chartInstance, c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_m), 1, 18, 1, 0) - 1]);
        c2_d13 = c2_abs(chartInstance, c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_mm1), 1, 18, 1, 0) - 1]);
        c2_d14 = c2_abs(chartInstance, c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_mm1), 1, 18, 1, 0) - 1]);
        c2_d15 = c2_abs(chartInstance, c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_q), 1, 18, 1, 0) - 1]);
        c2_d16 = c2_abs(chartInstance, c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_q), 1, 18, 1, 0) - 1]);
        c2_c_varargin_1[0] = c2_d12;
        c2_c_varargin_1[1] = c2_d13;
        c2_c_varargin_1[2] = c2_d14;
        c2_c_varargin_1[3] = c2_d15;
        c2_c_varargin_1[4] = c2_d16;
        c2_ixstart = 1;
        c2_mtmp = c2_c_varargin_1[0];
        c2_g_x = c2_mtmp;
        c2_xb_b = muDoubleScalarIsNaN(c2_g_x);
        if (c2_xb_b) {
          c2_b_check_forloop_overflow_error(chartInstance, FALSE);
          c2_ix = 2;
          exitg2 = FALSE;
          while ((exitg2 == FALSE) && (c2_ix < 6)) {
            c2_b_ix = c2_ix;
            c2_ixstart = c2_b_ix;
            c2_h_x = c2_c_varargin_1[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
              _SFD_INTEGER_CHECK("", (real_T)c2_b_ix), 1, 5, 1, 0) - 1];
            c2_yb_b = muDoubleScalarIsNaN(c2_h_x);
            if (!c2_yb_b) {
              c2_mtmp = c2_c_varargin_1[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
                _SFD_INTEGER_CHECK("", (real_T)c2_b_ix), 1, 5, 1, 0) - 1];
              exitg2 = TRUE;
            } else {
              c2_ix++;
            }
          }
        }

        if (c2_ixstart < 5) {
          c2_dc_a = c2_ixstart + 1;
          c2_i446 = c2_dc_a;
          c2_o_overflow = FALSE;
          c2_b_check_forloop_overflow_error(chartInstance, c2_o_overflow);
          for (c2_c_ix = c2_i446; c2_c_ix < 6; c2_c_ix++) {
            c2_b_ix = c2_c_ix;
            c2_ec_a = c2_c_varargin_1[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
              _SFD_INTEGER_CHECK("", (real_T)c2_b_ix), 1, 5, 1, 0) - 1];
            c2_ac_b = c2_mtmp;
            c2_p = (c2_ec_a > c2_ac_b);
            if (c2_p) {
              c2_mtmp = c2_c_varargin_1[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
                _SFD_INTEGER_CHECK("", (real_T)c2_b_ix), 1, 5, 1, 0) - 1];
            }
          }
        }

        c2_b_mtmp = c2_mtmp;
        c2_scale = c2_b_mtmp;
        c2_sm = c2_eml_div(chartInstance, c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_m), 1, 18, 1, 0) - 1],
                           c2_scale);
        c2_smm1 = c2_eml_div(chartInstance, c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
                              (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_mm1), 1,
          18, 1, 0) - 1], c2_scale);
        c2_emm1 = c2_eml_div(chartInstance, c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("",
                              (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_mm1), 1,
          18, 1, 0) - 1], c2_scale);
        c2_sqds = c2_eml_div(chartInstance, c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
                              (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_q), 1,
          18, 1, 0) - 1], c2_scale);
        c2_eqds = c2_eml_div(chartInstance, c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("",
                              (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_q), 1,
          18, 1, 0) - 1], c2_scale);
        c2_fc_a = c2_smm1 + c2_sm;
        c2_bc_b = c2_smm1 - c2_sm;
        c2_o_y = c2_fc_a * c2_bc_b;
        c2_gc_a = c2_emm1;
        c2_cc_b = c2_emm1;
        c2_p_y = c2_gc_a * c2_cc_b;
        c2_dc_b = c2_eml_div(chartInstance, c2_o_y + c2_p_y, 2.0);
        c2_hc_a = c2_sm;
        c2_ec_b = c2_emm1;
        c2_jb_c = c2_hc_a * c2_ec_b;
        c2_ic_a = c2_jb_c;
        c2_fc_b = c2_jb_c;
        c2_jb_c = c2_ic_a * c2_fc_b;
        c2_shift = 0.0;
        guard1 = FALSE;
        if (c2_dc_b != 0.0) {
          guard1 = TRUE;
        } else {
          if (c2_jb_c != 0.0) {
            guard1 = TRUE;
          }
        }

        if (guard1 == TRUE) {
          c2_jc_a = c2_dc_b;
          c2_gc_b = c2_dc_b;
          c2_q_y = c2_jc_a * c2_gc_b;
          c2_shift = c2_q_y + c2_jb_c;
          c2_b_sqrt(chartInstance, &c2_shift);
          if (c2_dc_b < 0.0) {
            c2_shift = -c2_shift;
          }

          c2_shift = c2_eml_div(chartInstance, c2_jb_c, c2_dc_b + c2_shift);
        }

        c2_kc_a = c2_sqds + c2_sm;
        c2_hc_b = c2_sqds - c2_sm;
        c2_r_y = c2_kc_a * c2_hc_b;
        c2_f = c2_r_y + c2_shift;
        c2_lc_a = c2_sqds;
        c2_ic_b = c2_eqds;
        c2_g = c2_lc_a * c2_ic_b;
        c2_k_q = c2_b_q;
        c2_b_mm1 = c2_mm1;
        c2_mc_a = c2_k_q;
        c2_jc_b = c2_b_mm1;
        c2_nc_a = c2_mc_a;
        c2_kc_b = c2_jc_b;
        if (c2_nc_a > c2_kc_b) {
          c2_p_overflow = FALSE;
        } else {
          c2_p_overflow = (c2_kc_b > 2147483646);
        }

        c2_b_check_forloop_overflow_error(chartInstance, c2_p_overflow);
        for (c2_d_k = c2_k_q; c2_d_k <= c2_b_mm1; c2_d_k++) {
          c2_b_k = c2_d_k;
          c2_oc_a = c2_b_k - 1;
          c2_km1 = c2_oc_a;
          c2_pc_a = c2_b_k + 1;
          c2_kp1 = c2_pc_a;
          c2_c_f = c2_f;
          c2_unusedU1 = c2_g;
          c2_b_eml_xrotg(chartInstance, &c2_c_f, &c2_unusedU1, &c2_d_cs,
                         &c2_d_sn);
          c2_f = c2_c_f;
          c2_b_cs = c2_d_cs;
          c2_b_sn = c2_d_sn;
          if (c2_b_k > c2_b_q) {
            c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c2_km1), 1, 18, 1, 0) - 1] = c2_f;
          }

          c2_qc_a = c2_b_cs;
          c2_lc_b = c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c2_b_k), 1, 18, 1, 0) - 1];
          c2_s_y = c2_qc_a * c2_lc_b;
          c2_rc_a = c2_b_sn;
          c2_mc_b = c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c2_b_k), 1, 18, 1, 0) - 1];
          c2_t_y = c2_rc_a * c2_mc_b;
          c2_f = c2_s_y + c2_t_y;
          c2_sc_a = c2_b_cs;
          c2_nc_b = c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c2_b_k), 1, 18, 1, 0) - 1];
          c2_u_y = c2_sc_a * c2_nc_b;
          c2_tc_a = c2_b_sn;
          c2_oc_b = c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c2_b_k), 1, 18, 1, 0) - 1];
          c2_v_y = c2_tc_a * c2_oc_b;
          c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_b_k), 1, 18, 1, 0) - 1] = c2_u_y - c2_v_y;
          c2_uc_a = c2_b_sn;
          c2_pc_b = c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c2_kp1), 1, 18, 1, 0) - 1];
          c2_g = c2_uc_a * c2_pc_b;
          c2_vc_a = c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c2_kp1), 1, 18, 1, 0) - 1];
          c2_qc_b = c2_b_cs;
          c2_w_y = c2_vc_a * c2_qc_b;
          c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_kp1), 1, 18, 1, 0) - 1] = c2_w_y;
          c2_wc_a = c2_b_k - 1;
          c2_kb_c = c2_wc_a;
          c2_rc_b = c2_kb_c;
          c2_lb_c = 18 * c2_rc_b;
          c2_sc_b = c2_lb_c + 1;
          c2_colk = c2_sc_b;
          c2_tc_b = c2_b_k;
          c2_mb_c = 18 * c2_tc_b;
          c2_uc_b = c2_mb_c + 1;
          c2_colkp1 = c2_uc_b;
          c2_g_eml_xrot(chartInstance, c2_Vf, c2_colk, c2_colkp1, c2_b_cs,
                        c2_b_sn);
          c2_d_f = c2_f;
          c2_unusedU2 = c2_g;
          c2_b_eml_xrotg(chartInstance, &c2_d_f, &c2_unusedU2, &c2_e_cs,
                         &c2_e_sn);
          c2_f = c2_d_f;
          c2_b_cs = c2_e_cs;
          c2_b_sn = c2_e_sn;
          c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_b_k), 1, 18, 1, 0) - 1] = c2_f;
          c2_xc_a = c2_b_cs;
          c2_vc_b = c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c2_b_k), 1, 18, 1, 0) - 1];
          c2_x_y = c2_xc_a * c2_vc_b;
          c2_yc_a = c2_b_sn;
          c2_wc_b = c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c2_kp1), 1, 18, 1, 0) - 1];
          c2_y_y = c2_yc_a * c2_wc_b;
          c2_f = c2_x_y + c2_y_y;
          c2_ad_a = -c2_b_sn;
          c2_xc_b = c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c2_b_k), 1, 18, 1, 0) - 1];
          c2_ab_y = c2_ad_a * c2_xc_b;
          c2_bd_a = c2_b_cs;
          c2_yc_b = c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c2_kp1), 1, 18, 1, 0) - 1];
          c2_bb_y = c2_bd_a * c2_yc_b;
          c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_kp1), 1, 18, 1, 0) - 1] = c2_ab_y + c2_bb_y;
          c2_cd_a = c2_b_sn;
          c2_ad_b = c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c2_kp1), 1, 18, 1, 0) - 1];
          c2_g = c2_cd_a * c2_ad_b;
          c2_dd_a = c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c2_kp1), 1, 18, 1, 0) - 1];
          c2_bd_b = c2_b_cs;
          c2_cb_y = c2_dd_a * c2_bd_b;
          c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_kp1), 1, 18, 1, 0) - 1] = c2_cb_y;
          if (c2_b_k < 25) {
            c2_ed_a = c2_b_k - 1;
            c2_nb_c = c2_ed_a;
            c2_cd_b = c2_nb_c;
            c2_ob_c = 25 * c2_cd_b;
            c2_dd_b = c2_ob_c + 1;
            c2_colk = c2_dd_b;
            c2_ed_b = c2_b_k;
            c2_pb_c = 25 * c2_ed_b;
            c2_fd_b = c2_pb_c + 1;
            c2_colkp1 = c2_fd_b;
            c2_h_eml_xrot(chartInstance, c2_U, c2_colk, c2_colkp1, c2_b_cs,
                          c2_b_sn);
          }
        }

        c2_fd_a = c2_m - 1;
        c2_qb_c = c2_fd_a;
        c2_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_qb_c), 1, 18, 1, 0) - 1] = c2_f;
        c2_iter++;
        break;

       default:
        if (c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c2_b_q), 1, 18, 1, 0) - 1] < 0.0) {
          c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_b_q), 1, 18, 1, 0) - 1] =
            -c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_b_q), 1, 18, 1, 0) - 1];
          c2_gd_a = c2_b_q - 1;
          c2_rb_c = c2_gd_a;
          c2_gd_b = c2_rb_c;
          c2_sb_c = 18 * c2_gd_b;
          c2_hd_b = c2_sb_c + 1;
          c2_colq = c2_hd_b;
          c2_j_eml_scalar_eg(chartInstance);
          c2_d17 = -1.0;
          c2_l_eml_xscal(chartInstance, c2_d17, c2_Vf, c2_colq);
        }

        c2_hd_a = c2_b_q + 1;
        c2_qp1 = c2_hd_a;
        exitg3 = FALSE;
        while ((exitg3 == FALSE) && (c2_b_q < 18)) {
          if (c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
                "", (real_T)c2_b_q), 1, 18, 1, 0) - 1] <
              c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
                "", (real_T)c2_qp1), 1, 18, 1, 0) - 1]) {
            c2_rt = c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
              _SFD_INTEGER_CHECK("", (real_T)c2_b_q), 1, 18, 1, 0) - 1];
            c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c2_b_q), 1, 18, 1, 0) - 1] =
              c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
              "", (real_T)c2_qp1), 1, 18, 1, 0) - 1];
            c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c2_qp1), 1, 18, 1, 0) - 1] = c2_rt;
            if (c2_b_q < 18) {
              c2_jd_a = c2_b_q - 1;
              c2_tb_c = c2_jd_a;
              c2_id_b = c2_tb_c;
              c2_ub_c = 18 * c2_id_b;
              c2_jd_b = c2_ub_c + 1;
              c2_colq = c2_jd_b;
              c2_kd_b = c2_b_q;
              c2_vb_c = 18 * c2_kd_b;
              c2_ld_b = c2_vb_c + 1;
              c2_colqp1 = c2_ld_b;
              c2_i_eml_xswap(chartInstance, c2_Vf, c2_colq, c2_colqp1);
            }

            if (c2_b_q < 25) {
              c2_kd_a = c2_b_q - 1;
              c2_wb_c = c2_kd_a;
              c2_md_b = c2_wb_c;
              c2_xb_c = 25 * c2_md_b;
              c2_nd_b = c2_xb_c + 1;
              c2_colq = c2_nd_b;
              c2_od_b = c2_b_q;
              c2_yb_c = 25 * c2_od_b;
              c2_pd_b = c2_yb_c + 1;
              c2_colqp1 = c2_pd_b;
              c2_j_eml_xswap(chartInstance, c2_U, c2_colq, c2_colqp1);
            }

            c2_b_q = c2_qp1;
            c2_ld_a = c2_b_q + 1;
            c2_qp1 = c2_ld_a;
          } else {
            exitg3 = TRUE;
          }
        }

        c2_iter = 0.0;
        c2_id_a = c2_m - 1;
        c2_m = c2_id_a;
        break;
      }
    }
  }

  c2_b_check_forloop_overflow_error(chartInstance, FALSE);
  for (c2_e_k = 1; c2_e_k < 19; c2_e_k++) {
    c2_b_k = c2_e_k;
    c2_S[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c2_b_k), 1, 18, 1, 0) - 1] = c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
      _SFD_INTEGER_CHECK("", (real_T)c2_b_k), 1, 18, 1, 0) - 1];
  }

  c2_b_check_forloop_overflow_error(chartInstance, FALSE);
  for (c2_j = 1; c2_j < 19; c2_j++) {
    c2_b_j = c2_j;
    c2_b_check_forloop_overflow_error(chartInstance, FALSE);
    for (c2_i = 1; c2_i < 19; c2_i++) {
      c2_b_i = c2_i;
      c2_V[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c2_b_i), 1, 18, 1, 0) + 18 * (_SFD_EML_ARRAY_BOUNDS_CHECK(
              "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_j), 1, 18, 2, 0)
             - 1)) - 1] = c2_Vf[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c2_b_i), 1, 18, 1, 0) + 18 *
        (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
        c2_b_j), 1, 18, 2, 0) - 1)) - 1];
    }
  }
}

static real_T c2_c_eml_xnrm2(SFc2_controllerInstanceStruct *chartInstance,
  int32_T c2_n, real_T c2_x[450], int32_T c2_ix0)
{
  real_T c2_y;
  int32_T c2_b_n;
  int32_T c2_b_ix0;
  int32_T c2_c_n;
  int32_T c2_c_ix0;
  real_T c2_b_x;
  real_T c2_c_x;
  real_T c2_scale;
  int32_T c2_kstart;
  int32_T c2_a;
  int32_T c2_c;
  int32_T c2_b_a;
  int32_T c2_b_c;
  int32_T c2_c_a;
  int32_T c2_b;
  int32_T c2_kend;
  int32_T c2_b_kstart;
  int32_T c2_b_kend;
  int32_T c2_d_a;
  int32_T c2_b_b;
  int32_T c2_e_a;
  int32_T c2_c_b;
  boolean_T c2_overflow;
  int32_T c2_k;
  int32_T c2_b_k;
  real_T c2_d_x;
  real_T c2_e_x;
  real_T c2_absxk;
  real_T c2_t;
  c2_b_n = c2_n;
  c2_b_ix0 = c2_ix0;
  c2_c_n = c2_b_n;
  c2_c_ix0 = c2_b_ix0;
  c2_y = 0.0;
  if (c2_c_n < 1) {
  } else if (c2_c_n == 1) {
    c2_b_x = c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c2_c_ix0), 1, 450, 1, 0) - 1];
    c2_c_x = c2_b_x;
    c2_y = muDoubleScalarAbs(c2_c_x);
  } else {
    c2_realmin(chartInstance);
    c2_scale = 2.2250738585072014E-308;
    c2_kstart = c2_c_ix0;
    c2_a = c2_c_n;
    c2_c = c2_a;
    c2_b_a = c2_c - 1;
    c2_b_c = c2_b_a;
    c2_c_a = c2_kstart;
    c2_b = c2_b_c;
    c2_kend = c2_c_a + c2_b;
    c2_b_kstart = c2_kstart;
    c2_b_kend = c2_kend;
    c2_d_a = c2_b_kstart;
    c2_b_b = c2_b_kend;
    c2_e_a = c2_d_a;
    c2_c_b = c2_b_b;
    if (c2_e_a > c2_c_b) {
      c2_overflow = FALSE;
    } else {
      c2_overflow = (c2_c_b > 2147483646);
    }

    c2_b_check_forloop_overflow_error(chartInstance, c2_overflow);
    for (c2_k = c2_b_kstart; c2_k <= c2_b_kend; c2_k++) {
      c2_b_k = c2_k;
      c2_d_x = c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
        "", (real_T)c2_b_k), 1, 450, 1, 0) - 1];
      c2_e_x = c2_d_x;
      c2_absxk = muDoubleScalarAbs(c2_e_x);
      if (c2_absxk > c2_scale) {
        c2_t = c2_scale / c2_absxk;
        c2_y = 1.0 + c2_y * c2_t * c2_t;
        c2_scale = c2_absxk;
      } else {
        c2_t = c2_absxk / c2_scale;
        c2_y += c2_t * c2_t;
      }
    }

    c2_y = c2_scale * muDoubleScalarSqrt(c2_y);
  }

  return c2_y;
}

static void c2_d_eml_xscal(SFc2_controllerInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_a, real_T c2_x[450], int32_T c2_ix0, real_T c2_b_x[450])
{
  int32_T c2_i447;
  for (c2_i447 = 0; c2_i447 < 450; c2_i447++) {
    c2_b_x[c2_i447] = c2_x[c2_i447];
  }

  c2_j_eml_xscal(chartInstance, c2_n, c2_a, c2_b_x, c2_ix0);
}

static real_T c2_c_eml_xdotc(SFc2_controllerInstanceStruct *chartInstance,
  int32_T c2_n, real_T c2_x[450], int32_T c2_ix0, real_T c2_y[450], int32_T
  c2_iy0)
{
  real_T c2_d;
  int32_T c2_b_n;
  int32_T c2_b_ix0;
  int32_T c2_b_iy0;
  int32_T c2_c_n;
  int32_T c2_c_ix0;
  int32_T c2_c_iy0;
  int32_T c2_d_n;
  int32_T c2_d_ix0;
  int32_T c2_d_iy0;
  int32_T c2_e_n;
  int32_T c2_e_ix0;
  int32_T c2_e_iy0;
  int32_T c2_ix;
  int32_T c2_iy;
  int32_T c2_f_n;
  int32_T c2_b;
  int32_T c2_b_b;
  boolean_T c2_overflow;
  int32_T c2_k;
  int32_T c2_a;
  int32_T c2_b_a;
  c2_b_n = c2_n;
  c2_b_ix0 = c2_ix0;
  c2_b_iy0 = c2_iy0;
  c2_c_n = c2_b_n;
  c2_c_ix0 = c2_b_ix0;
  c2_c_iy0 = c2_b_iy0;
  c2_d_n = c2_c_n;
  c2_d_ix0 = c2_c_ix0;
  c2_d_iy0 = c2_c_iy0;
  c2_e_n = c2_d_n;
  c2_e_ix0 = c2_d_ix0;
  c2_e_iy0 = c2_d_iy0;
  c2_d = 0.0;
  if (c2_e_n < 1) {
  } else {
    c2_ix = c2_e_ix0;
    c2_iy = c2_e_iy0;
    c2_f_n = c2_e_n;
    c2_b = c2_f_n;
    c2_b_b = c2_b;
    if (1 > c2_b_b) {
      c2_overflow = FALSE;
    } else {
      c2_overflow = (c2_b_b > 2147483646);
    }

    c2_b_check_forloop_overflow_error(chartInstance, c2_overflow);
    for (c2_k = 1; c2_k <= c2_f_n; c2_k++) {
      c2_d += c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
        "", (real_T)c2_ix), 1, 450, 1, 0) - 1] *
        c2_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c2_iy), 1, 450, 1, 0) - 1];
      c2_a = c2_ix + 1;
      c2_ix = c2_a;
      c2_b_a = c2_iy + 1;
      c2_iy = c2_b_a;
    }
  }

  return c2_d;
}

static void c2_e_eml_xaxpy(SFc2_controllerInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_a, int32_T c2_ix0, real_T c2_y[450], int32_T c2_iy0, real_T
  c2_b_y[450])
{
  int32_T c2_i448;
  for (c2_i448 = 0; c2_i448 < 450; c2_i448++) {
    c2_b_y[c2_i448] = c2_y[c2_i448];
  }

  c2_m_eml_xaxpy(chartInstance, c2_n, c2_a, c2_ix0, c2_b_y, c2_iy0);
}

static real_T c2_d_eml_xnrm2(SFc2_controllerInstanceStruct *chartInstance,
  int32_T c2_n, real_T c2_x[18], int32_T c2_ix0)
{
  real_T c2_y;
  int32_T c2_b_n;
  int32_T c2_b_ix0;
  int32_T c2_c_n;
  int32_T c2_c_ix0;
  real_T c2_b_x;
  real_T c2_c_x;
  real_T c2_scale;
  int32_T c2_kstart;
  int32_T c2_a;
  int32_T c2_c;
  int32_T c2_b_a;
  int32_T c2_b_c;
  int32_T c2_c_a;
  int32_T c2_b;
  int32_T c2_kend;
  int32_T c2_b_kstart;
  int32_T c2_b_kend;
  int32_T c2_d_a;
  int32_T c2_b_b;
  int32_T c2_e_a;
  int32_T c2_c_b;
  boolean_T c2_overflow;
  int32_T c2_k;
  int32_T c2_b_k;
  real_T c2_d_x;
  real_T c2_e_x;
  real_T c2_absxk;
  real_T c2_t;
  c2_b_n = c2_n;
  c2_b_ix0 = c2_ix0;
  c2_c_n = c2_b_n;
  c2_c_ix0 = c2_b_ix0;
  c2_y = 0.0;
  if (c2_c_n < 1) {
  } else if (c2_c_n == 1) {
    c2_b_x = c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c2_c_ix0), 1, 18, 1, 0) - 1];
    c2_c_x = c2_b_x;
    c2_y = muDoubleScalarAbs(c2_c_x);
  } else {
    c2_realmin(chartInstance);
    c2_scale = 2.2250738585072014E-308;
    c2_kstart = c2_c_ix0;
    c2_a = c2_c_n;
    c2_c = c2_a;
    c2_b_a = c2_c - 1;
    c2_b_c = c2_b_a;
    c2_c_a = c2_kstart;
    c2_b = c2_b_c;
    c2_kend = c2_c_a + c2_b;
    c2_b_kstart = c2_kstart;
    c2_b_kend = c2_kend;
    c2_d_a = c2_b_kstart;
    c2_b_b = c2_b_kend;
    c2_e_a = c2_d_a;
    c2_c_b = c2_b_b;
    if (c2_e_a > c2_c_b) {
      c2_overflow = FALSE;
    } else {
      c2_overflow = (c2_c_b > 2147483646);
    }

    c2_b_check_forloop_overflow_error(chartInstance, c2_overflow);
    for (c2_k = c2_b_kstart; c2_k <= c2_b_kend; c2_k++) {
      c2_b_k = c2_k;
      c2_d_x = c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
        "", (real_T)c2_b_k), 1, 18, 1, 0) - 1];
      c2_e_x = c2_d_x;
      c2_absxk = muDoubleScalarAbs(c2_e_x);
      if (c2_absxk > c2_scale) {
        c2_t = c2_scale / c2_absxk;
        c2_y = 1.0 + c2_y * c2_t * c2_t;
        c2_scale = c2_absxk;
      } else {
        c2_t = c2_absxk / c2_scale;
        c2_y += c2_t * c2_t;
      }
    }

    c2_y = c2_scale * muDoubleScalarSqrt(c2_y);
  }

  return c2_y;
}

static void c2_e_eml_xscal(SFc2_controllerInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_a, real_T c2_x[18], int32_T c2_ix0, real_T c2_b_x[18])
{
  int32_T c2_i449;
  for (c2_i449 = 0; c2_i449 < 18; c2_i449++) {
    c2_b_x[c2_i449] = c2_x[c2_i449];
  }

  c2_k_eml_xscal(chartInstance, c2_n, c2_a, c2_b_x, c2_ix0);
}

static void c2_f_eml_xaxpy(SFc2_controllerInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_a, real_T c2_x[450], int32_T c2_ix0, real_T c2_y[25], int32_T
  c2_iy0, real_T c2_b_y[25])
{
  int32_T c2_i450;
  int32_T c2_i451;
  real_T c2_b_x[450];
  for (c2_i450 = 0; c2_i450 < 25; c2_i450++) {
    c2_b_y[c2_i450] = c2_y[c2_i450];
  }

  for (c2_i451 = 0; c2_i451 < 450; c2_i451++) {
    c2_b_x[c2_i451] = c2_x[c2_i451];
  }

  c2_n_eml_xaxpy(chartInstance, c2_n, c2_a, c2_b_x, c2_ix0, c2_b_y, c2_iy0);
}

static void c2_g_eml_xaxpy(SFc2_controllerInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_a, real_T c2_x[25], int32_T c2_ix0, real_T c2_y[450], int32_T
  c2_iy0, real_T c2_b_y[450])
{
  int32_T c2_i452;
  int32_T c2_i453;
  real_T c2_b_x[25];
  for (c2_i452 = 0; c2_i452 < 450; c2_i452++) {
    c2_b_y[c2_i452] = c2_y[c2_i452];
  }

  for (c2_i453 = 0; c2_i453 < 25; c2_i453++) {
    c2_b_x[c2_i453] = c2_x[c2_i453];
  }

  c2_o_eml_xaxpy(chartInstance, c2_n, c2_a, c2_b_x, c2_ix0, c2_b_y, c2_iy0);
}

static real_T c2_d_eml_xdotc(SFc2_controllerInstanceStruct *chartInstance,
  int32_T c2_n, real_T c2_x[324], int32_T c2_ix0, real_T c2_y[324], int32_T
  c2_iy0)
{
  real_T c2_d;
  int32_T c2_b_n;
  int32_T c2_b_ix0;
  int32_T c2_b_iy0;
  int32_T c2_c_n;
  int32_T c2_c_ix0;
  int32_T c2_c_iy0;
  int32_T c2_d_n;
  int32_T c2_d_ix0;
  int32_T c2_d_iy0;
  int32_T c2_e_n;
  int32_T c2_e_ix0;
  int32_T c2_e_iy0;
  int32_T c2_ix;
  int32_T c2_iy;
  int32_T c2_f_n;
  int32_T c2_b;
  int32_T c2_b_b;
  boolean_T c2_overflow;
  int32_T c2_k;
  int32_T c2_a;
  int32_T c2_b_a;
  c2_b_n = c2_n;
  c2_b_ix0 = c2_ix0;
  c2_b_iy0 = c2_iy0;
  c2_c_n = c2_b_n;
  c2_c_ix0 = c2_b_ix0;
  c2_c_iy0 = c2_b_iy0;
  c2_d_n = c2_c_n;
  c2_d_ix0 = c2_c_ix0;
  c2_d_iy0 = c2_c_iy0;
  c2_e_n = c2_d_n;
  c2_e_ix0 = c2_d_ix0;
  c2_e_iy0 = c2_d_iy0;
  c2_d = 0.0;
  if (c2_e_n < 1) {
  } else {
    c2_ix = c2_e_ix0;
    c2_iy = c2_e_iy0;
    c2_f_n = c2_e_n;
    c2_b = c2_f_n;
    c2_b_b = c2_b;
    if (1 > c2_b_b) {
      c2_overflow = FALSE;
    } else {
      c2_overflow = (c2_b_b > 2147483646);
    }

    c2_b_check_forloop_overflow_error(chartInstance, c2_overflow);
    for (c2_k = 1; c2_k <= c2_f_n; c2_k++) {
      c2_d += c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
        "", (real_T)c2_ix), 1, 324, 1, 0) - 1] *
        c2_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c2_iy), 1, 324, 1, 0) - 1];
      c2_a = c2_ix + 1;
      c2_ix = c2_a;
      c2_b_a = c2_iy + 1;
      c2_iy = c2_b_a;
    }
  }

  return c2_d;
}

static void c2_h_eml_xaxpy(SFc2_controllerInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_a, int32_T c2_ix0, real_T c2_y[324], int32_T c2_iy0, real_T
  c2_b_y[324])
{
  int32_T c2_i454;
  for (c2_i454 = 0; c2_i454 < 324; c2_i454++) {
    c2_b_y[c2_i454] = c2_y[c2_i454];
  }

  c2_p_eml_xaxpy(chartInstance, c2_n, c2_a, c2_ix0, c2_b_y, c2_iy0);
}

static void c2_j_eml_scalar_eg(SFc2_controllerInstanceStruct *chartInstance)
{
}

static void c2_f_eml_xscal(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_a, real_T c2_x[324], int32_T c2_ix0, real_T c2_b_x[324])
{
  int32_T c2_i455;
  for (c2_i455 = 0; c2_i455 < 324; c2_i455++) {
    c2_b_x[c2_i455] = c2_x[c2_i455];
  }

  c2_l_eml_xscal(chartInstance, c2_a, c2_b_x, c2_ix0);
}

static void c2_c_eml_xrot(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_x[324], int32_T c2_ix0, int32_T c2_iy0, real_T c2_c, real_T c2_s, real_T
  c2_b_x[324])
{
  int32_T c2_i456;
  for (c2_i456 = 0; c2_i456 < 324; c2_i456++) {
    c2_b_x[c2_i456] = c2_x[c2_i456];
  }

  c2_g_eml_xrot(chartInstance, c2_b_x, c2_ix0, c2_iy0, c2_c, c2_s);
}

static void c2_d_eml_xrot(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_x[450], int32_T c2_ix0, int32_T c2_iy0, real_T c2_c, real_T c2_s, real_T
  c2_b_x[450])
{
  int32_T c2_i457;
  for (c2_i457 = 0; c2_i457 < 450; c2_i457++) {
    c2_b_x[c2_i457] = c2_x[c2_i457];
  }

  c2_h_eml_xrot(chartInstance, c2_b_x, c2_ix0, c2_iy0, c2_c, c2_s);
}

static void c2_d_eml_xswap(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_x[324], int32_T c2_ix0, int32_T c2_iy0, real_T c2_b_x[324])
{
  int32_T c2_i458;
  for (c2_i458 = 0; c2_i458 < 324; c2_i458++) {
    c2_b_x[c2_i458] = c2_x[c2_i458];
  }

  c2_i_eml_xswap(chartInstance, c2_b_x, c2_ix0, c2_iy0);
}

static void c2_e_eml_xswap(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_x[450], int32_T c2_ix0, int32_T c2_iy0, real_T c2_b_x[450])
{
  int32_T c2_i459;
  for (c2_i459 = 0; c2_i459 < 450; c2_i459++) {
    c2_b_x[c2_i459] = c2_x[c2_i459];
  }

  c2_j_eml_xswap(chartInstance, c2_b_x, c2_ix0, c2_iy0);
}

static void c2_d_eml_xgemm(SFc2_controllerInstanceStruct *chartInstance, int32_T
  c2_k, real_T c2_A[324], real_T c2_B[450], real_T c2_C[450], real_T c2_b_C[450])
{
  int32_T c2_i460;
  int32_T c2_i461;
  real_T c2_b_A[324];
  int32_T c2_i462;
  real_T c2_b_B[450];
  for (c2_i460 = 0; c2_i460 < 450; c2_i460++) {
    c2_b_C[c2_i460] = c2_C[c2_i460];
  }

  for (c2_i461 = 0; c2_i461 < 324; c2_i461++) {
    c2_b_A[c2_i461] = c2_A[c2_i461];
  }

  for (c2_i462 = 0; c2_i462 < 450; c2_i462++) {
    c2_b_B[c2_i462] = c2_B[c2_i462];
  }

  c2_m_eml_xgemm(chartInstance, c2_k, c2_b_A, c2_b_B, c2_b_C);
}

static void c2_b_below_threshold(SFc2_controllerInstanceStruct *chartInstance)
{
}

static void c2_b_eye(SFc2_controllerInstanceStruct *chartInstance, real_T c2_I
                     [625])
{
  int32_T c2_i463;
  int32_T c2_i;
  int32_T c2_b_i;
  c2_isVariableSizing(chartInstance);
  for (c2_i463 = 0; c2_i463 < 625; c2_i463++) {
    c2_I[c2_i463] = 0.0;
  }

  c2_check_forloop_overflow_error(chartInstance);
  for (c2_i = 1; c2_i < 26; c2_i++) {
    c2_b_i = c2_i;
    c2_I[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_b_i), 1, 25, 1, 0) + 25 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
            (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_i), 1, 25, 2, 0) - 1))
      - 1] = 1.0;
  }
}

static void c2_k_eml_scalar_eg(SFc2_controllerInstanceStruct *chartInstance)
{
}

static void c2_e_eml_xgemm(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_A[450], real_T c2_B[558], real_T c2_C[775], real_T c2_b_C[775])
{
  int32_T c2_i464;
  int32_T c2_i465;
  real_T c2_b_A[450];
  int32_T c2_i466;
  real_T c2_b_B[558];
  for (c2_i464 = 0; c2_i464 < 775; c2_i464++) {
    c2_b_C[c2_i464] = c2_C[c2_i464];
  }

  for (c2_i465 = 0; c2_i465 < 450; c2_i465++) {
    c2_b_A[c2_i465] = c2_A[c2_i465];
  }

  for (c2_i466 = 0; c2_i466 < 558; c2_i466++) {
    c2_b_B[c2_i466] = c2_B[c2_i466];
  }

  c2_n_eml_xgemm(chartInstance, c2_b_A, c2_b_B, c2_b_C);
}

static void c2_l_eml_scalar_eg(SFc2_controllerInstanceStruct *chartInstance)
{
}

static void c2_f_eml_xgemm(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_A[775], real_T c2_B[775], real_T c2_C[625], real_T c2_b_C[625])
{
  int32_T c2_i467;
  int32_T c2_i468;
  real_T c2_b_A[775];
  int32_T c2_i469;
  real_T c2_b_B[775];
  for (c2_i467 = 0; c2_i467 < 625; c2_i467++) {
    c2_b_C[c2_i467] = c2_C[c2_i467];
  }

  for (c2_i468 = 0; c2_i468 < 775; c2_i468++) {
    c2_b_A[c2_i468] = c2_A[c2_i468];
  }

  for (c2_i469 = 0; c2_i469 < 775; c2_i469++) {
    c2_b_B[c2_i469] = c2_B[c2_i469];
  }

  c2_o_eml_xgemm(chartInstance, c2_b_A, c2_b_B, c2_b_C);
}

static void c2_m_eml_scalar_eg(SFc2_controllerInstanceStruct *chartInstance)
{
}

static void c2_g_eml_xgemm(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_A[558], real_T c2_B[31], real_T c2_C[18], real_T c2_b_C[18])
{
  int32_T c2_i470;
  int32_T c2_i471;
  real_T c2_b_A[558];
  int32_T c2_i472;
  real_T c2_b_B[31];
  for (c2_i470 = 0; c2_i470 < 18; c2_i470++) {
    c2_b_C[c2_i470] = c2_C[c2_i470];
  }

  for (c2_i471 = 0; c2_i471 < 558; c2_i471++) {
    c2_b_A[c2_i471] = c2_A[c2_i471];
  }

  for (c2_i472 = 0; c2_i472 < 31; c2_i472++) {
    c2_b_B[c2_i472] = c2_B[c2_i472];
  }

  c2_p_eml_xgemm(chartInstance, c2_b_A, c2_b_B, c2_b_C);
}

static void c2_n_eml_scalar_eg(SFc2_controllerInstanceStruct *chartInstance)
{
}

static void c2_h_eml_xgemm(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_A[558], real_T c2_B[558], real_T c2_C[324], real_T c2_b_C[324])
{
  int32_T c2_i473;
  int32_T c2_i474;
  real_T c2_b_A[558];
  int32_T c2_i475;
  real_T c2_b_B[558];
  for (c2_i473 = 0; c2_i473 < 324; c2_i473++) {
    c2_b_C[c2_i473] = c2_C[c2_i473];
  }

  for (c2_i474 = 0; c2_i474 < 558; c2_i474++) {
    c2_b_A[c2_i474] = c2_A[c2_i474];
  }

  for (c2_i475 = 0; c2_i475 < 558; c2_i475++) {
    c2_b_B[c2_i475] = c2_B[c2_i475];
  }

  c2_q_eml_xgemm(chartInstance, c2_b_A, c2_b_B, c2_b_C);
}

static void c2_o_eml_scalar_eg(SFc2_controllerInstanceStruct *chartInstance)
{
}

static void c2_p_eml_scalar_eg(SFc2_controllerInstanceStruct *chartInstance)
{
}

static void c2_q_eml_scalar_eg(SFc2_controllerInstanceStruct *chartInstance)
{
}

static void c2_i_eml_xgemm(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_A[625], real_T c2_B[25], real_T c2_C[25], real_T c2_b_C[25])
{
  int32_T c2_i476;
  int32_T c2_i477;
  real_T c2_b_A[625];
  int32_T c2_i478;
  real_T c2_b_B[25];
  for (c2_i476 = 0; c2_i476 < 25; c2_i476++) {
    c2_b_C[c2_i476] = c2_C[c2_i476];
  }

  for (c2_i477 = 0; c2_i477 < 625; c2_i477++) {
    c2_b_A[c2_i477] = c2_A[c2_i477];
  }

  for (c2_i478 = 0; c2_i478 < 25; c2_i478++) {
    c2_b_B[c2_i478] = c2_B[c2_i478];
  }

  c2_r_eml_xgemm(chartInstance, c2_b_A, c2_b_B, c2_b_C);
}

static real_T c2_b_norm(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_x[3])
{
  real_T c2_y;
  real_T c2_scale;
  int32_T c2_k;
  int32_T c2_b_k;
  real_T c2_b_x;
  real_T c2_c_x;
  real_T c2_absxk;
  real_T c2_t;
  c2_y = 0.0;
  c2_realmin(chartInstance);
  c2_scale = 2.2250738585072014E-308;
  c2_check_forloop_overflow_error(chartInstance);
  for (c2_k = 1; c2_k < 4; c2_k++) {
    c2_b_k = c2_k;
    c2_b_x = c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c2_b_k), 1, 3, 1, 0) - 1];
    c2_c_x = c2_b_x;
    c2_absxk = muDoubleScalarAbs(c2_c_x);
    if (c2_absxk > c2_scale) {
      c2_t = c2_scale / c2_absxk;
      c2_y = 1.0 + c2_y * c2_t * c2_t;
      c2_scale = c2_absxk;
    } else {
      c2_t = c2_absxk / c2_scale;
      c2_y += c2_t * c2_t;
    }
  }

  return c2_scale * muDoubleScalarSqrt(c2_y);
}

static void c2_q_emlrt_marshallIn(SFc2_controllerInstanceStruct *chartInstance,
  const mxArray *c2_sprintf, const char_T *c2_identifier, char_T c2_y[14])
{
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_r_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_sprintf), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_sprintf);
}

static void c2_r_emlrt_marshallIn(SFc2_controllerInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, char_T c2_y[14])
{
  char_T c2_cv3[14];
  int32_T c2_i479;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_cv3, 1, 10, 0U, 1, 0U, 2, 1,
                14);
  for (c2_i479 = 0; c2_i479 < 14; c2_i479++) {
    c2_y[c2_i479] = c2_cv3[c2_i479];
  }

  sf_mex_destroy(&c2_u);
}

static const mxArray *c2_v_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_controllerInstanceStruct *chartInstance;
  chartInstance = (SFc2_controllerInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(int32_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static int32_T c2_s_emlrt_marshallIn(SFc2_controllerInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  int32_T c2_y;
  int32_T c2_i480;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_i480, 1, 6, 0U, 0, 0U, 0);
  c2_y = c2_i480;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_n_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_sfEvent;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  int32_T c2_y;
  SFc2_controllerInstanceStruct *chartInstance;
  chartInstance = (SFc2_controllerInstanceStruct *)chartInstanceVoid;
  c2_b_sfEvent = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_s_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_sfEvent),
    &c2_thisId);
  sf_mex_destroy(&c2_b_sfEvent);
  *(int32_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static uint8_T c2_t_emlrt_marshallIn(SFc2_controllerInstanceStruct
  *chartInstance, const mxArray *c2_b_is_active_c2_controller, const char_T
  *c2_identifier)
{
  uint8_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_u_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c2_b_is_active_c2_controller), &c2_thisId);
  sf_mex_destroy(&c2_b_is_active_c2_controller);
  return c2_y;
}

static uint8_T c2_u_emlrt_marshallIn(SFc2_controllerInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  uint8_T c2_y;
  uint8_T c2_u0;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_u0, 1, 3, 0U, 0, 0U, 0);
  c2_y = c2_u0;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_g_eml_xscal(SFc2_controllerInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_a, real_T c2_x[72], int32_T c2_ix0)
{
  int32_T c2_b_n;
  real_T c2_b_a;
  int32_T c2_b_ix0;
  int32_T c2_c_n;
  real_T c2_c_a;
  int32_T c2_c_ix0;
  int32_T c2_d_ix0;
  int32_T c2_d_a;
  int32_T c2_c;
  int32_T c2_b;
  int32_T c2_b_c;
  int32_T c2_e_a;
  int32_T c2_b_b;
  int32_T c2_i481;
  int32_T c2_f_a;
  int32_T c2_c_b;
  int32_T c2_g_a;
  int32_T c2_d_b;
  boolean_T c2_overflow;
  int32_T c2_k;
  int32_T c2_b_k;
  c2_b_n = c2_n;
  c2_b_a = c2_a;
  c2_b_ix0 = c2_ix0;
  c2_c_n = c2_b_n;
  c2_c_a = c2_b_a;
  c2_c_ix0 = c2_b_ix0;
  c2_d_ix0 = c2_c_ix0;
  c2_d_a = c2_c_n;
  c2_c = c2_d_a;
  c2_b = c2_c - 1;
  c2_b_c = c2_b;
  c2_e_a = c2_c_ix0;
  c2_b_b = c2_b_c;
  c2_i481 = c2_e_a + c2_b_b;
  c2_f_a = c2_d_ix0;
  c2_c_b = c2_i481;
  c2_g_a = c2_f_a;
  c2_d_b = c2_c_b;
  if (c2_g_a > c2_d_b) {
    c2_overflow = FALSE;
  } else {
    c2_overflow = (c2_d_b > 2147483646);
  }

  c2_b_check_forloop_overflow_error(chartInstance, c2_overflow);
  for (c2_k = c2_d_ix0; c2_k <= c2_i481; c2_k++) {
    c2_b_k = c2_k;
    c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c2_b_k), 1, 72, 1, 0) - 1] = c2_c_a * c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("",
      (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_k), 1, 72, 1, 0) - 1];
  }
}

static void c2_i_eml_xaxpy(SFc2_controllerInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_a, int32_T c2_ix0, real_T c2_y[72], int32_T c2_iy0)
{
  int32_T c2_b_n;
  real_T c2_b_a;
  int32_T c2_b_ix0;
  int32_T c2_b_iy0;
  int32_T c2_c_n;
  real_T c2_c_a;
  int32_T c2_c_ix0;
  int32_T c2_c_iy0;
  int32_T c2_d_a;
  int32_T c2_ix;
  int32_T c2_e_a;
  int32_T c2_iy;
  int32_T c2_f_a;
  int32_T c2_i482;
  int32_T c2_b;
  int32_T c2_b_b;
  boolean_T c2_overflow;
  int32_T c2_k;
  int32_T c2_g_a;
  int32_T c2_c;
  int32_T c2_h_a;
  int32_T c2_b_c;
  int32_T c2_i_a;
  int32_T c2_c_c;
  int32_T c2_j_a;
  int32_T c2_k_a;
  c2_b_n = c2_n;
  c2_b_a = c2_a;
  c2_b_ix0 = c2_ix0;
  c2_b_iy0 = c2_iy0;
  c2_c_n = c2_b_n;
  c2_c_a = c2_b_a;
  c2_c_ix0 = c2_b_ix0;
  c2_c_iy0 = c2_b_iy0;
  if (c2_c_n < 1) {
  } else if (c2_c_a == 0.0) {
  } else {
    c2_d_a = c2_c_ix0 - 1;
    c2_ix = c2_d_a;
    c2_e_a = c2_c_iy0 - 1;
    c2_iy = c2_e_a;
    c2_f_a = c2_c_n - 1;
    c2_i482 = c2_f_a;
    c2_b = c2_i482;
    c2_b_b = c2_b;
    if (0 > c2_b_b) {
      c2_overflow = FALSE;
    } else {
      c2_overflow = (c2_b_b > 2147483646);
    }

    c2_b_check_forloop_overflow_error(chartInstance, c2_overflow);
    for (c2_k = 0; c2_k <= c2_i482; c2_k++) {
      c2_g_a = c2_iy;
      c2_c = c2_g_a;
      c2_h_a = c2_iy;
      c2_b_c = c2_h_a;
      c2_i_a = c2_ix;
      c2_c_c = c2_i_a;
      c2_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c2_c + 1)), 1, 72, 1, 0) - 1] =
        c2_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c2_b_c + 1)), 1, 72, 1, 0) - 1] + c2_c_a *
        c2_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c2_c_c + 1)), 1, 72, 1, 0) - 1];
      c2_j_a = c2_ix + 1;
      c2_ix = c2_j_a;
      c2_k_a = c2_iy + 1;
      c2_iy = c2_k_a;
    }
  }
}

static void c2_h_eml_xscal(SFc2_controllerInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_a, real_T c2_x[6], int32_T c2_ix0)
{
  int32_T c2_b_n;
  real_T c2_b_a;
  int32_T c2_b_ix0;
  int32_T c2_c_n;
  real_T c2_c_a;
  int32_T c2_c_ix0;
  int32_T c2_d_ix0;
  int32_T c2_d_a;
  int32_T c2_c;
  int32_T c2_b;
  int32_T c2_b_c;
  int32_T c2_e_a;
  int32_T c2_b_b;
  int32_T c2_i483;
  int32_T c2_f_a;
  int32_T c2_c_b;
  int32_T c2_g_a;
  int32_T c2_d_b;
  boolean_T c2_overflow;
  int32_T c2_k;
  int32_T c2_b_k;
  c2_b_n = c2_n;
  c2_b_a = c2_a;
  c2_b_ix0 = c2_ix0;
  c2_c_n = c2_b_n;
  c2_c_a = c2_b_a;
  c2_c_ix0 = c2_b_ix0;
  c2_d_ix0 = c2_c_ix0;
  c2_d_a = c2_c_n;
  c2_c = c2_d_a;
  c2_b = c2_c - 1;
  c2_b_c = c2_b;
  c2_e_a = c2_c_ix0;
  c2_b_b = c2_b_c;
  c2_i483 = c2_e_a + c2_b_b;
  c2_f_a = c2_d_ix0;
  c2_c_b = c2_i483;
  c2_g_a = c2_f_a;
  c2_d_b = c2_c_b;
  if (c2_g_a > c2_d_b) {
    c2_overflow = FALSE;
  } else {
    c2_overflow = (c2_d_b > 2147483646);
  }

  c2_b_check_forloop_overflow_error(chartInstance, c2_overflow);
  for (c2_k = c2_d_ix0; c2_k <= c2_i483; c2_k++) {
    c2_b_k = c2_k;
    c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c2_b_k), 1, 6, 1, 0) - 1] = c2_c_a * c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("",
      (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_k), 1, 6, 1, 0) - 1];
  }
}

static void c2_j_eml_xaxpy(SFc2_controllerInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_a, real_T c2_x[72], int32_T c2_ix0, real_T c2_y[12], int32_T
  c2_iy0)
{
  int32_T c2_b_n;
  real_T c2_b_a;
  int32_T c2_b_ix0;
  int32_T c2_b_iy0;
  int32_T c2_c_n;
  real_T c2_c_a;
  int32_T c2_c_ix0;
  int32_T c2_c_iy0;
  int32_T c2_d_a;
  int32_T c2_ix;
  int32_T c2_e_a;
  int32_T c2_iy;
  int32_T c2_f_a;
  int32_T c2_i484;
  int32_T c2_b;
  int32_T c2_b_b;
  boolean_T c2_overflow;
  int32_T c2_k;
  int32_T c2_g_a;
  int32_T c2_c;
  int32_T c2_h_a;
  int32_T c2_b_c;
  int32_T c2_i_a;
  int32_T c2_c_c;
  int32_T c2_j_a;
  int32_T c2_k_a;
  c2_b_n = c2_n;
  c2_b_a = c2_a;
  c2_b_ix0 = c2_ix0;
  c2_b_iy0 = c2_iy0;
  c2_c_n = c2_b_n;
  c2_c_a = c2_b_a;
  c2_c_ix0 = c2_b_ix0;
  c2_c_iy0 = c2_b_iy0;
  if (c2_c_n < 1) {
  } else if (c2_c_a == 0.0) {
  } else {
    c2_d_a = c2_c_ix0 - 1;
    c2_ix = c2_d_a;
    c2_e_a = c2_c_iy0 - 1;
    c2_iy = c2_e_a;
    c2_f_a = c2_c_n - 1;
    c2_i484 = c2_f_a;
    c2_b = c2_i484;
    c2_b_b = c2_b;
    if (0 > c2_b_b) {
      c2_overflow = FALSE;
    } else {
      c2_overflow = (c2_b_b > 2147483646);
    }

    c2_b_check_forloop_overflow_error(chartInstance, c2_overflow);
    for (c2_k = 0; c2_k <= c2_i484; c2_k++) {
      c2_g_a = c2_iy;
      c2_c = c2_g_a;
      c2_h_a = c2_iy;
      c2_b_c = c2_h_a;
      c2_i_a = c2_ix;
      c2_c_c = c2_i_a;
      c2_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c2_c + 1)), 1, 12, 1, 0) - 1] =
        c2_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c2_b_c + 1)), 1, 12, 1, 0) - 1] + c2_c_a *
        c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c2_c_c + 1)), 1, 72, 1, 0) - 1];
      c2_j_a = c2_ix + 1;
      c2_ix = c2_j_a;
      c2_k_a = c2_iy + 1;
      c2_iy = c2_k_a;
    }
  }
}

static void c2_k_eml_xaxpy(SFc2_controllerInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_a, real_T c2_x[12], int32_T c2_ix0, real_T c2_y[72], int32_T
  c2_iy0)
{
  int32_T c2_b_n;
  real_T c2_b_a;
  int32_T c2_b_ix0;
  int32_T c2_b_iy0;
  int32_T c2_c_n;
  real_T c2_c_a;
  int32_T c2_c_ix0;
  int32_T c2_c_iy0;
  int32_T c2_d_a;
  int32_T c2_ix;
  int32_T c2_e_a;
  int32_T c2_iy;
  int32_T c2_f_a;
  int32_T c2_i485;
  int32_T c2_b;
  int32_T c2_b_b;
  boolean_T c2_overflow;
  int32_T c2_k;
  int32_T c2_g_a;
  int32_T c2_c;
  int32_T c2_h_a;
  int32_T c2_b_c;
  int32_T c2_i_a;
  int32_T c2_c_c;
  int32_T c2_j_a;
  int32_T c2_k_a;
  c2_b_n = c2_n;
  c2_b_a = c2_a;
  c2_b_ix0 = c2_ix0;
  c2_b_iy0 = c2_iy0;
  c2_c_n = c2_b_n;
  c2_c_a = c2_b_a;
  c2_c_ix0 = c2_b_ix0;
  c2_c_iy0 = c2_b_iy0;
  if (c2_c_n < 1) {
  } else if (c2_c_a == 0.0) {
  } else {
    c2_d_a = c2_c_ix0 - 1;
    c2_ix = c2_d_a;
    c2_e_a = c2_c_iy0 - 1;
    c2_iy = c2_e_a;
    c2_f_a = c2_c_n - 1;
    c2_i485 = c2_f_a;
    c2_b = c2_i485;
    c2_b_b = c2_b;
    if (0 > c2_b_b) {
      c2_overflow = FALSE;
    } else {
      c2_overflow = (c2_b_b > 2147483646);
    }

    c2_b_check_forloop_overflow_error(chartInstance, c2_overflow);
    for (c2_k = 0; c2_k <= c2_i485; c2_k++) {
      c2_g_a = c2_iy;
      c2_c = c2_g_a;
      c2_h_a = c2_iy;
      c2_b_c = c2_h_a;
      c2_i_a = c2_ix;
      c2_c_c = c2_i_a;
      c2_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c2_c + 1)), 1, 72, 1, 0) - 1] =
        c2_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c2_b_c + 1)), 1, 72, 1, 0) - 1] + c2_c_a *
        c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c2_c_c + 1)), 1, 12, 1, 0) - 1];
      c2_j_a = c2_ix + 1;
      c2_ix = c2_j_a;
      c2_k_a = c2_iy + 1;
      c2_iy = c2_k_a;
    }
  }
}

static void c2_l_eml_xaxpy(SFc2_controllerInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_a, int32_T c2_ix0, real_T c2_y[36], int32_T c2_iy0)
{
  int32_T c2_b_n;
  real_T c2_b_a;
  int32_T c2_b_ix0;
  int32_T c2_b_iy0;
  int32_T c2_c_n;
  real_T c2_c_a;
  int32_T c2_c_ix0;
  int32_T c2_c_iy0;
  int32_T c2_d_a;
  int32_T c2_ix;
  int32_T c2_e_a;
  int32_T c2_iy;
  int32_T c2_f_a;
  int32_T c2_i486;
  int32_T c2_b;
  int32_T c2_b_b;
  boolean_T c2_overflow;
  int32_T c2_k;
  int32_T c2_g_a;
  int32_T c2_c;
  int32_T c2_h_a;
  int32_T c2_b_c;
  int32_T c2_i_a;
  int32_T c2_c_c;
  int32_T c2_j_a;
  int32_T c2_k_a;
  c2_b_n = c2_n;
  c2_b_a = c2_a;
  c2_b_ix0 = c2_ix0;
  c2_b_iy0 = c2_iy0;
  c2_c_n = c2_b_n;
  c2_c_a = c2_b_a;
  c2_c_ix0 = c2_b_ix0;
  c2_c_iy0 = c2_b_iy0;
  if (c2_c_n < 1) {
  } else if (c2_c_a == 0.0) {
  } else {
    c2_d_a = c2_c_ix0 - 1;
    c2_ix = c2_d_a;
    c2_e_a = c2_c_iy0 - 1;
    c2_iy = c2_e_a;
    c2_f_a = c2_c_n - 1;
    c2_i486 = c2_f_a;
    c2_b = c2_i486;
    c2_b_b = c2_b;
    if (0 > c2_b_b) {
      c2_overflow = FALSE;
    } else {
      c2_overflow = (c2_b_b > 2147483646);
    }

    c2_b_check_forloop_overflow_error(chartInstance, c2_overflow);
    for (c2_k = 0; c2_k <= c2_i486; c2_k++) {
      c2_g_a = c2_iy;
      c2_c = c2_g_a;
      c2_h_a = c2_iy;
      c2_b_c = c2_h_a;
      c2_i_a = c2_ix;
      c2_c_c = c2_i_a;
      c2_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c2_c + 1)), 1, 36, 1, 0) - 1] =
        c2_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c2_b_c + 1)), 1, 36, 1, 0) - 1] + c2_c_a *
        c2_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c2_c_c + 1)), 1, 36, 1, 0) - 1];
      c2_j_a = c2_ix + 1;
      c2_ix = c2_j_a;
      c2_k_a = c2_iy + 1;
      c2_iy = c2_k_a;
    }
  }
}

static void c2_i_eml_xscal(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_a, real_T c2_x[36], int32_T c2_ix0)
{
  real_T c2_b_a;
  int32_T c2_b_ix0;
  real_T c2_c_a;
  int32_T c2_c_ix0;
  int32_T c2_d_ix0;
  int32_T c2_d_a;
  int32_T c2_i487;
  int32_T c2_e_a;
  int32_T c2_b;
  int32_T c2_f_a;
  int32_T c2_b_b;
  boolean_T c2_overflow;
  int32_T c2_k;
  int32_T c2_b_k;
  c2_b_a = c2_a;
  c2_b_ix0 = c2_ix0;
  c2_c_a = c2_b_a;
  c2_c_ix0 = c2_b_ix0;
  c2_d_ix0 = c2_c_ix0;
  c2_d_a = c2_c_ix0 + 5;
  c2_i487 = c2_d_a;
  c2_e_a = c2_d_ix0;
  c2_b = c2_i487;
  c2_f_a = c2_e_a;
  c2_b_b = c2_b;
  if (c2_f_a > c2_b_b) {
    c2_overflow = FALSE;
  } else {
    c2_overflow = (c2_b_b > 2147483646);
  }

  c2_b_check_forloop_overflow_error(chartInstance, c2_overflow);
  for (c2_k = c2_d_ix0; c2_k <= c2_i487; c2_k++) {
    c2_b_k = c2_k;
    c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c2_b_k), 1, 36, 1, 0) - 1] = c2_c_a * c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("",
      (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_k), 1, 36, 1, 0) - 1];
  }
}

static void c2_b_sqrt(SFc2_controllerInstanceStruct *chartInstance, real_T *c2_x)
{
  if (*c2_x < 0.0) {
    c2_c_eml_error(chartInstance);
  }

  *c2_x = muDoubleScalarSqrt(*c2_x);
}

static void c2_b_eml_xrotg(SFc2_controllerInstanceStruct *chartInstance, real_T *
  c2_a, real_T *c2_b, real_T *c2_c, real_T *c2_s)
{
  real_T c2_b_a;
  real_T c2_b_b;
  real_T c2_c_b;
  real_T c2_c_a;
  real_T c2_d_a;
  real_T c2_d_b;
  real_T c2_e_b;
  real_T c2_e_a;
  real_T c2_b_c;
  real_T c2_b_s;
  real_T c2_c_c;
  real_T c2_c_s;
  c2_b_a = *c2_a;
  c2_b_b = *c2_b;
  c2_c_b = c2_b_b;
  c2_c_a = c2_b_a;
  c2_d_a = c2_c_a;
  c2_d_b = c2_c_b;
  c2_e_b = c2_d_b;
  c2_e_a = c2_d_a;
  c2_b_c = 0.0;
  c2_b_s = 0.0;
  drotg32(&c2_e_a, &c2_e_b, &c2_b_c, &c2_b_s);
  c2_c_a = c2_e_a;
  c2_c_b = c2_e_b;
  c2_c_c = c2_b_c;
  c2_c_s = c2_b_s;
  *c2_a = c2_c_a;
  *c2_b = c2_c_b;
  *c2_c = c2_c_c;
  *c2_s = c2_c_s;
}

static void c2_e_eml_xrot(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_x[36], int32_T c2_ix0, int32_T c2_iy0, real_T c2_c, real_T c2_s)
{
  int32_T c2_b_ix0;
  int32_T c2_b_iy0;
  real_T c2_b_c;
  real_T c2_b_s;
  int32_T c2_c_ix0;
  int32_T c2_c_iy0;
  real_T c2_c_c;
  real_T c2_c_s;
  int32_T c2_ix;
  int32_T c2_iy;
  int32_T c2_k;
  real_T c2_a;
  real_T c2_b;
  real_T c2_y;
  real_T c2_b_a;
  real_T c2_b_b;
  real_T c2_b_y;
  real_T c2_temp;
  real_T c2_c_a;
  real_T c2_c_b;
  real_T c2_c_y;
  real_T c2_d_a;
  real_T c2_d_b;
  real_T c2_d_y;
  int32_T c2_e_a;
  int32_T c2_f_a;
  c2_b_ix0 = c2_ix0;
  c2_b_iy0 = c2_iy0;
  c2_b_c = c2_c;
  c2_b_s = c2_s;
  c2_c_ix0 = c2_b_ix0;
  c2_c_iy0 = c2_b_iy0;
  c2_c_c = c2_b_c;
  c2_c_s = c2_b_s;
  c2_ix = c2_c_ix0;
  c2_iy = c2_c_iy0;
  c2_check_forloop_overflow_error(chartInstance);
  for (c2_k = 1; c2_k < 7; c2_k++) {
    c2_a = c2_c_c;
    c2_b = c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c2_ix), 1, 36, 1, 0) - 1];
    c2_y = c2_a * c2_b;
    c2_b_a = c2_c_s;
    c2_b_b = c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c2_iy), 1, 36, 1, 0) - 1];
    c2_b_y = c2_b_a * c2_b_b;
    c2_temp = c2_y + c2_b_y;
    c2_c_a = c2_c_c;
    c2_c_b = c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c2_iy), 1, 36, 1, 0) - 1];
    c2_c_y = c2_c_a * c2_c_b;
    c2_d_a = c2_c_s;
    c2_d_b = c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c2_ix), 1, 36, 1, 0) - 1];
    c2_d_y = c2_d_a * c2_d_b;
    c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c2_iy), 1, 36, 1, 0) - 1] = c2_c_y - c2_d_y;
    c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c2_ix), 1, 36, 1, 0) - 1] = c2_temp;
    c2_e_a = c2_iy + 1;
    c2_iy = c2_e_a;
    c2_f_a = c2_ix + 1;
    c2_ix = c2_f_a;
  }
}

static void c2_f_eml_xrot(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_x[72], int32_T c2_ix0, int32_T c2_iy0, real_T c2_c, real_T c2_s)
{
  int32_T c2_b_ix0;
  int32_T c2_b_iy0;
  real_T c2_b_c;
  real_T c2_b_s;
  int32_T c2_c_ix0;
  int32_T c2_c_iy0;
  real_T c2_c_c;
  real_T c2_c_s;
  int32_T c2_ix;
  int32_T c2_iy;
  int32_T c2_k;
  real_T c2_a;
  real_T c2_b;
  real_T c2_y;
  real_T c2_b_a;
  real_T c2_b_b;
  real_T c2_b_y;
  real_T c2_temp;
  real_T c2_c_a;
  real_T c2_c_b;
  real_T c2_c_y;
  real_T c2_d_a;
  real_T c2_d_b;
  real_T c2_d_y;
  int32_T c2_e_a;
  int32_T c2_f_a;
  c2_b_ix0 = c2_ix0;
  c2_b_iy0 = c2_iy0;
  c2_b_c = c2_c;
  c2_b_s = c2_s;
  c2_c_ix0 = c2_b_ix0;
  c2_c_iy0 = c2_b_iy0;
  c2_c_c = c2_b_c;
  c2_c_s = c2_b_s;
  c2_ix = c2_c_ix0;
  c2_iy = c2_c_iy0;
  c2_b_check_forloop_overflow_error(chartInstance, FALSE);
  for (c2_k = 1; c2_k < 13; c2_k++) {
    c2_a = c2_c_c;
    c2_b = c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c2_ix), 1, 72, 1, 0) - 1];
    c2_y = c2_a * c2_b;
    c2_b_a = c2_c_s;
    c2_b_b = c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c2_iy), 1, 72, 1, 0) - 1];
    c2_b_y = c2_b_a * c2_b_b;
    c2_temp = c2_y + c2_b_y;
    c2_c_a = c2_c_c;
    c2_c_b = c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c2_iy), 1, 72, 1, 0) - 1];
    c2_c_y = c2_c_a * c2_c_b;
    c2_d_a = c2_c_s;
    c2_d_b = c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c2_ix), 1, 72, 1, 0) - 1];
    c2_d_y = c2_d_a * c2_d_b;
    c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c2_iy), 1, 72, 1, 0) - 1] = c2_c_y - c2_d_y;
    c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c2_ix), 1, 72, 1, 0) - 1] = c2_temp;
    c2_e_a = c2_iy + 1;
    c2_iy = c2_e_a;
    c2_f_a = c2_ix + 1;
    c2_ix = c2_f_a;
  }
}

static void c2_f_eml_xswap(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_x[36], int32_T c2_ix0, int32_T c2_iy0)
{
  int32_T c2_b_ix0;
  int32_T c2_b_iy0;
  int32_T c2_c_ix0;
  int32_T c2_c_iy0;
  int32_T c2_ix;
  int32_T c2_iy;
  int32_T c2_k;
  real_T c2_temp;
  int32_T c2_a;
  int32_T c2_b_a;
  c2_b_ix0 = c2_ix0;
  c2_b_iy0 = c2_iy0;
  c2_c_ix0 = c2_b_ix0;
  c2_c_iy0 = c2_b_iy0;
  c2_ix = c2_c_ix0;
  c2_iy = c2_c_iy0;
  c2_check_forloop_overflow_error(chartInstance);
  for (c2_k = 1; c2_k < 7; c2_k++) {
    c2_temp = c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
      "", (real_T)c2_ix), 1, 36, 1, 0) - 1];
    c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c2_ix), 1, 36, 1, 0) - 1] = c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
      _SFD_INTEGER_CHECK("", (real_T)c2_iy), 1, 36, 1, 0) - 1];
    c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c2_iy), 1, 36, 1, 0) - 1] = c2_temp;
    c2_a = c2_ix + 1;
    c2_ix = c2_a;
    c2_b_a = c2_iy + 1;
    c2_iy = c2_b_a;
  }
}

static void c2_g_eml_xswap(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_x[72], int32_T c2_ix0, int32_T c2_iy0)
{
  int32_T c2_b_ix0;
  int32_T c2_b_iy0;
  int32_T c2_c_ix0;
  int32_T c2_c_iy0;
  int32_T c2_ix;
  int32_T c2_iy;
  int32_T c2_k;
  real_T c2_temp;
  int32_T c2_a;
  int32_T c2_b_a;
  c2_b_ix0 = c2_ix0;
  c2_b_iy0 = c2_iy0;
  c2_c_ix0 = c2_b_ix0;
  c2_c_iy0 = c2_b_iy0;
  c2_ix = c2_c_ix0;
  c2_iy = c2_c_iy0;
  c2_b_check_forloop_overflow_error(chartInstance, FALSE);
  for (c2_k = 1; c2_k < 13; c2_k++) {
    c2_temp = c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
      "", (real_T)c2_ix), 1, 72, 1, 0) - 1];
    c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c2_ix), 1, 72, 1, 0) - 1] = c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
      _SFD_INTEGER_CHECK("", (real_T)c2_iy), 1, 72, 1, 0) - 1];
    c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c2_iy), 1, 72, 1, 0) - 1] = c2_temp;
    c2_a = c2_ix + 1;
    c2_ix = c2_a;
    c2_b_a = c2_iy + 1;
    c2_iy = c2_b_a;
  }
}

static void c2_j_eml_xgemm(SFc2_controllerInstanceStruct *chartInstance, int32_T
  c2_k, real_T c2_A[36], real_T c2_B[72], real_T c2_C[72])
{
  int32_T c2_b_k;
  int32_T c2_c_k;
  int32_T c2_a;
  int32_T c2_km1;
  int32_T c2_cr;
  int32_T c2_b_cr;
  int32_T c2_b_a;
  int32_T c2_i488;
  int32_T c2_c_a;
  int32_T c2_i489;
  int32_T c2_d_a;
  int32_T c2_b;
  int32_T c2_e_a;
  int32_T c2_b_b;
  boolean_T c2_overflow;
  int32_T c2_ic;
  int32_T c2_b_ic;
  int32_T c2_br;
  int32_T c2_c_cr;
  int32_T c2_ar;
  int32_T c2_f_a;
  int32_T c2_b_br;
  int32_T c2_c_b;
  int32_T c2_c;
  int32_T c2_g_a;
  int32_T c2_d_b;
  int32_T c2_i490;
  int32_T c2_h_a;
  int32_T c2_e_b;
  int32_T c2_i_a;
  int32_T c2_f_b;
  boolean_T c2_b_overflow;
  int32_T c2_ib;
  int32_T c2_b_ib;
  real_T c2_temp;
  int32_T c2_ia;
  int32_T c2_j_a;
  int32_T c2_i491;
  int32_T c2_k_a;
  int32_T c2_i492;
  int32_T c2_l_a;
  int32_T c2_g_b;
  int32_T c2_m_a;
  int32_T c2_h_b;
  boolean_T c2_c_overflow;
  int32_T c2_c_ic;
  int32_T c2_n_a;
  int32_T c2_o_a;
  c2_b_k = c2_k;
  c2_c_k = c2_b_k;
  c2_a = c2_c_k;
  c2_km1 = c2_a;
  c2_check_forloop_overflow_error(chartInstance);
  for (c2_cr = 0; c2_cr < 67; c2_cr += 6) {
    c2_b_cr = c2_cr;
    c2_b_a = c2_b_cr + 1;
    c2_i488 = c2_b_a;
    c2_c_a = c2_b_cr + 6;
    c2_i489 = c2_c_a;
    c2_d_a = c2_i488;
    c2_b = c2_i489;
    c2_e_a = c2_d_a;
    c2_b_b = c2_b;
    if (c2_e_a > c2_b_b) {
      c2_overflow = FALSE;
    } else {
      c2_overflow = (c2_b_b > 2147483646);
    }

    c2_b_check_forloop_overflow_error(chartInstance, c2_overflow);
    for (c2_ic = c2_i488; c2_ic <= c2_i489; c2_ic++) {
      c2_b_ic = c2_ic;
      c2_C[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c2_b_ic), 1, 72, 1, 0) - 1] = 0.0;
    }
  }

  c2_br = 0;
  c2_check_forloop_overflow_error(chartInstance);
  for (c2_c_cr = 0; c2_c_cr < 67; c2_c_cr += 6) {
    c2_b_cr = c2_c_cr;
    c2_ar = 0;
    c2_f_a = c2_br + 1;
    c2_br = c2_f_a;
    c2_b_br = c2_br;
    c2_c_b = c2_km1 - 1;
    c2_c = 12 * c2_c_b;
    c2_g_a = c2_br;
    c2_d_b = c2_c;
    c2_i490 = c2_g_a + c2_d_b;
    c2_h_a = c2_b_br;
    c2_e_b = c2_i490;
    c2_i_a = c2_h_a;
    c2_f_b = c2_e_b;
    if (c2_i_a > c2_f_b) {
      c2_b_overflow = FALSE;
    } else {
      c2_b_overflow = (c2_f_b > 2147483635);
    }

    c2_b_check_forloop_overflow_error(chartInstance, c2_b_overflow);
    for (c2_ib = c2_b_br; c2_ib <= c2_i490; c2_ib += 12) {
      c2_b_ib = c2_ib;
      if (c2_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_b_ib), 1, 72, 1, 0) - 1] != 0.0) {
        c2_temp = c2_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c2_b_ib), 1, 72, 1, 0) - 1];
        c2_ia = c2_ar;
        c2_j_a = c2_b_cr + 1;
        c2_i491 = c2_j_a;
        c2_k_a = c2_b_cr + 6;
        c2_i492 = c2_k_a;
        c2_l_a = c2_i491;
        c2_g_b = c2_i492;
        c2_m_a = c2_l_a;
        c2_h_b = c2_g_b;
        if (c2_m_a > c2_h_b) {
          c2_c_overflow = FALSE;
        } else {
          c2_c_overflow = (c2_h_b > 2147483646);
        }

        c2_b_check_forloop_overflow_error(chartInstance, c2_c_overflow);
        for (c2_c_ic = c2_i491; c2_c_ic <= c2_i492; c2_c_ic++) {
          c2_b_ic = c2_c_ic;
          c2_n_a = c2_ia + 1;
          c2_ia = c2_n_a;
          c2_C[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_b_ic), 1, 72, 1, 0) - 1] =
            c2_C[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_b_ic), 1, 72, 1, 0) - 1] + c2_temp *
            c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_ia), 1, 36, 1, 0) - 1];
        }
      }

      c2_o_a = c2_ar + 6;
      c2_ar = c2_o_a;
    }
  }
}

static void c2_b_eml_matlab_zgetrf(SFc2_controllerInstanceStruct *chartInstance,
  real_T c2_A[961], int32_T c2_ipiv[31], int32_T *c2_info)
{
  int32_T c2_i493;
  int32_T c2_j;
  int32_T c2_b_j;
  int32_T c2_a;
  int32_T c2_jm1;
  int32_T c2_b;
  int32_T c2_mmj;
  int32_T c2_b_a;
  int32_T c2_c;
  int32_T c2_b_b;
  int32_T c2_jj;
  int32_T c2_c_a;
  int32_T c2_jp1j;
  int32_T c2_d_a;
  int32_T c2_b_c;
  int32_T c2_n;
  int32_T c2_ix0;
  int32_T c2_b_n;
  int32_T c2_b_ix0;
  int32_T c2_c_n;
  int32_T c2_c_ix0;
  int32_T c2_idxmax;
  int32_T c2_ix;
  real_T c2_x;
  real_T c2_b_x;
  real_T c2_c_x;
  real_T c2_y;
  real_T c2_d_x;
  real_T c2_e_x;
  real_T c2_b_y;
  real_T c2_smax;
  int32_T c2_d_n;
  int32_T c2_c_b;
  int32_T c2_d_b;
  boolean_T c2_overflow;
  int32_T c2_k;
  int32_T c2_b_k;
  int32_T c2_e_a;
  real_T c2_f_x;
  real_T c2_g_x;
  real_T c2_h_x;
  real_T c2_c_y;
  real_T c2_i_x;
  real_T c2_j_x;
  real_T c2_d_y;
  real_T c2_s;
  int32_T c2_f_a;
  int32_T c2_jpiv_offset;
  int32_T c2_g_a;
  int32_T c2_e_b;
  int32_T c2_jpiv;
  int32_T c2_h_a;
  int32_T c2_f_b;
  int32_T c2_c_c;
  int32_T c2_g_b;
  int32_T c2_jrow;
  int32_T c2_i_a;
  int32_T c2_h_b;
  int32_T c2_jprow;
  int32_T c2_b_jp1j;
  int32_T c2_j_a;
  int32_T c2_d_c;
  int32_T c2_k_a;
  int32_T c2_i_b;
  int32_T c2_i494;
  int32_T c2_l_a;
  int32_T c2_j_b;
  int32_T c2_m_a;
  int32_T c2_k_b;
  boolean_T c2_b_overflow;
  int32_T c2_i;
  int32_T c2_b_i;
  real_T c2_k_x;
  real_T c2_e_y;
  real_T c2_z;
  int32_T c2_l_b;
  int32_T c2_e_c;
  int32_T c2_n_a;
  int32_T c2_f_c;
  int32_T c2_o_a;
  int32_T c2_g_c;
  int32_T c2_m;
  int32_T c2_e_n;
  int32_T c2_d_ix0;
  int32_T c2_iy0;
  int32_T c2_ia0;
  real_T c2_d18;
  c2_realmin(chartInstance);
  c2_eps(chartInstance);
  for (c2_i493 = 0; c2_i493 < 31; c2_i493++) {
    c2_ipiv[c2_i493] = 1 + c2_i493;
  }

  *c2_info = 0;
  c2_b_check_forloop_overflow_error(chartInstance, FALSE);
  for (c2_j = 1; c2_j < 31; c2_j++) {
    c2_b_j = c2_j;
    c2_a = c2_b_j - 1;
    c2_jm1 = c2_a;
    c2_b = c2_b_j;
    c2_mmj = 31 - c2_b;
    c2_b_a = c2_jm1;
    c2_c = c2_b_a << 5;
    c2_b_b = c2_c + 1;
    c2_jj = c2_b_b;
    c2_c_a = c2_jj + 1;
    c2_jp1j = c2_c_a;
    c2_d_a = c2_mmj;
    c2_b_c = c2_d_a;
    c2_n = c2_b_c + 1;
    c2_ix0 = c2_jj;
    c2_b_n = c2_n;
    c2_b_ix0 = c2_ix0;
    c2_c_n = c2_b_n;
    c2_c_ix0 = c2_b_ix0;
    if (c2_c_n < 1) {
      c2_idxmax = 0;
    } else {
      c2_idxmax = 1;
      if (c2_c_n > 1) {
        c2_ix = c2_c_ix0;
        c2_x = c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
          "", (real_T)c2_ix), 1, 961, 1, 0) - 1];
        c2_b_x = c2_x;
        c2_c_x = c2_b_x;
        c2_y = muDoubleScalarAbs(c2_c_x);
        c2_d_x = 0.0;
        c2_e_x = c2_d_x;
        c2_b_y = muDoubleScalarAbs(c2_e_x);
        c2_smax = c2_y + c2_b_y;
        c2_d_n = c2_c_n;
        c2_c_b = c2_d_n;
        c2_d_b = c2_c_b;
        if (2 > c2_d_b) {
          c2_overflow = FALSE;
        } else {
          c2_overflow = (c2_d_b > 2147483646);
        }

        c2_b_check_forloop_overflow_error(chartInstance, c2_overflow);
        for (c2_k = 2; c2_k <= c2_d_n; c2_k++) {
          c2_b_k = c2_k;
          c2_e_a = c2_ix + 1;
          c2_ix = c2_e_a;
          c2_f_x = c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c2_ix), 1, 961, 1, 0) - 1];
          c2_g_x = c2_f_x;
          c2_h_x = c2_g_x;
          c2_c_y = muDoubleScalarAbs(c2_h_x);
          c2_i_x = 0.0;
          c2_j_x = c2_i_x;
          c2_d_y = muDoubleScalarAbs(c2_j_x);
          c2_s = c2_c_y + c2_d_y;
          if (c2_s > c2_smax) {
            c2_idxmax = c2_b_k;
            c2_smax = c2_s;
          }
        }
      }
    }

    c2_f_a = c2_idxmax - 1;
    c2_jpiv_offset = c2_f_a;
    c2_g_a = c2_jj;
    c2_e_b = c2_jpiv_offset;
    c2_jpiv = c2_g_a + c2_e_b;
    if (c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_jpiv), 1, 961, 1, 0) - 1] != 0.0) {
      if (c2_jpiv_offset != 0) {
        c2_h_a = c2_b_j;
        c2_f_b = c2_jpiv_offset;
        c2_c_c = c2_h_a + c2_f_b;
        c2_ipiv[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_b_j), 1, 31, 1, 0) - 1] = c2_c_c;
        c2_g_b = c2_jm1 + 1;
        c2_jrow = c2_g_b;
        c2_i_a = c2_jrow;
        c2_h_b = c2_jpiv_offset;
        c2_jprow = c2_i_a + c2_h_b;
        c2_h_eml_xswap(chartInstance, c2_A, c2_jrow, c2_jprow);
      }

      c2_b_jp1j = c2_jp1j;
      c2_j_a = c2_mmj;
      c2_d_c = c2_j_a;
      c2_k_a = c2_jp1j;
      c2_i_b = c2_d_c - 1;
      c2_i494 = c2_k_a + c2_i_b;
      c2_l_a = c2_b_jp1j;
      c2_j_b = c2_i494;
      c2_m_a = c2_l_a;
      c2_k_b = c2_j_b;
      if (c2_m_a > c2_k_b) {
        c2_b_overflow = FALSE;
      } else {
        c2_b_overflow = (c2_k_b > 2147483646);
      }

      c2_b_check_forloop_overflow_error(chartInstance, c2_b_overflow);
      for (c2_i = c2_b_jp1j; c2_i <= c2_i494; c2_i++) {
        c2_b_i = c2_i;
        c2_k_x = c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c2_b_i), 1, 961, 1, 0) - 1];
        c2_e_y = c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c2_jj), 1, 961, 1, 0) - 1];
        c2_z = c2_k_x / c2_e_y;
        c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_b_i), 1, 961, 1, 0) - 1] = c2_z;
      }
    } else {
      *c2_info = c2_b_j;
    }

    c2_l_b = c2_b_j;
    c2_e_c = 31 - c2_l_b;
    c2_n_a = c2_jj;
    c2_f_c = c2_n_a;
    c2_o_a = c2_jj;
    c2_g_c = c2_o_a;
    c2_m = c2_mmj;
    c2_e_n = c2_e_c;
    c2_d_ix0 = c2_jp1j;
    c2_iy0 = c2_f_c + 31;
    c2_ia0 = c2_g_c + 32;
    c2_d18 = -1.0;
    c2_b_eml_xger(chartInstance, c2_m, c2_e_n, c2_d18, c2_d_ix0, c2_iy0, c2_A,
                  c2_ia0);
  }

  if (*c2_info == 0) {
    if (!(c2_A[960] != 0.0)) {
      *c2_info = 31;
    }
  }
}

static void c2_h_eml_xswap(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_x[961], int32_T c2_ix0, int32_T c2_iy0)
{
  int32_T c2_b_ix0;
  int32_T c2_b_iy0;
  int32_T c2_c_ix0;
  int32_T c2_c_iy0;
  int32_T c2_ix;
  int32_T c2_iy;
  int32_T c2_k;
  real_T c2_temp;
  int32_T c2_a;
  int32_T c2_b_a;
  c2_b_ix0 = c2_ix0;
  c2_b_iy0 = c2_iy0;
  if (c2_eml_use_refblas(chartInstance)) {
  } else {
    c2_below_threshold(chartInstance);
  }

  c2_c_ix0 = c2_b_ix0;
  c2_c_iy0 = c2_b_iy0;
  c2_ix = c2_c_ix0;
  c2_iy = c2_c_iy0;
  c2_b_check_forloop_overflow_error(chartInstance, FALSE);
  for (c2_k = 1; c2_k < 32; c2_k++) {
    c2_temp = c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
      "", (real_T)c2_ix), 1, 961, 1, 0) - 1];
    c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c2_ix), 1, 961, 1, 0) - 1] = c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
      _SFD_INTEGER_CHECK("", (real_T)c2_iy), 1, 961, 1, 0) - 1];
    c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c2_iy), 1, 961, 1, 0) - 1] = c2_temp;
    c2_a = c2_ix + 31;
    c2_ix = c2_a;
    c2_b_a = c2_iy + 31;
    c2_iy = c2_b_a;
  }
}

static void c2_b_eml_xger(SFc2_controllerInstanceStruct *chartInstance, int32_T
  c2_m, int32_T c2_n, real_T c2_alpha1, int32_T c2_ix0, int32_T c2_iy0, real_T
  c2_A[961], int32_T c2_ia0)
{
  int32_T c2_b_m;
  int32_T c2_b_n;
  real_T c2_b_alpha1;
  int32_T c2_b_ix0;
  int32_T c2_b_iy0;
  int32_T c2_b_ia0;
  int32_T c2_c_m;
  int32_T c2_c_n;
  real_T c2_c_alpha1;
  int32_T c2_c_ix0;
  int32_T c2_incx;
  int32_T c2_c_iy0;
  int32_T c2_incy;
  int32_T c2_c_ia0;
  int32_T c2_lda;
  c2_b_m = c2_m;
  c2_b_n = c2_n;
  c2_b_alpha1 = c2_alpha1;
  c2_b_ix0 = c2_ix0;
  c2_b_iy0 = c2_iy0;
  c2_b_ia0 = c2_ia0;
  if (c2_b_m < 1) {
  } else if (c2_b_n < 1) {
  } else {
    c2_c_m = c2_b_m;
    c2_c_n = c2_b_n;
    c2_c_alpha1 = c2_b_alpha1;
    c2_c_ix0 = c2_b_ix0;
    c2_incx = 1;
    c2_c_iy0 = c2_b_iy0;
    c2_incy = 31;
    c2_c_ia0 = c2_b_ia0;
    c2_lda = 31;
    dger32(&c2_c_m, &c2_c_n, &c2_c_alpha1, &c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("",
            (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_c_ix0), 1, 961, 1, 0) - 1],
           &c2_incx, &c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c2_c_iy0), 1, 961, 1, 0) - 1],
           &c2_incy, &c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c2_c_ia0), 1, 961, 1, 0) - 1],
           &c2_lda);
  }
}

static void c2_b_eml_xtrsm(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_A[961], real_T c2_B[961])
{
  int32_T c2_m;
  int32_T c2_n;
  real_T c2_alpha1;
  int32_T c2_lda;
  int32_T c2_ldb;
  char_T c2_DIAGA;
  char_T c2_TRANSA;
  char_T c2_UPLO;
  char_T c2_SIDE;
  c2_m = 31;
  c2_n = 31;
  c2_alpha1 = 1.0;
  c2_lda = 31;
  c2_ldb = 31;
  c2_DIAGA = 'N';
  c2_TRANSA = 'N';
  c2_UPLO = 'U';
  c2_SIDE = 'L';
  dtrsm32(&c2_SIDE, &c2_UPLO, &c2_TRANSA, &c2_DIAGA, &c2_m, &c2_n, &c2_alpha1,
          &c2_A[0], &c2_lda, &c2_B[0], &c2_ldb);
}

static void c2_k_eml_xgemm(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_A[558], real_T c2_B[961], real_T c2_C[558])
{
  int32_T c2_m;
  int32_T c2_n;
  int32_T c2_k;
  real_T c2_alpha1;
  int32_T c2_lda;
  int32_T c2_ldb;
  real_T c2_beta1;
  int32_T c2_ldc;
  char_T c2_TRANSB;
  char_T c2_TRANSA;
  c2_m = 18;
  c2_n = 31;
  c2_k = 31;
  c2_alpha1 = 1.0;
  c2_lda = 18;
  c2_ldb = 31;
  c2_beta1 = 0.0;
  c2_ldc = 18;
  c2_TRANSB = 'N';
  c2_TRANSA = 'N';
  dgemm32(&c2_TRANSA, &c2_TRANSB, &c2_m, &c2_n, &c2_k, &c2_alpha1, &c2_A[0],
          &c2_lda, &c2_B[0], &c2_ldb, &c2_beta1, &c2_C[0], &c2_ldc);
}

static void c2_l_eml_xgemm(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_A[558], real_T c2_B[775], real_T c2_C[450])
{
  int32_T c2_m;
  int32_T c2_n;
  int32_T c2_k;
  real_T c2_alpha1;
  int32_T c2_lda;
  int32_T c2_ldb;
  real_T c2_beta1;
  int32_T c2_ldc;
  char_T c2_TRANSB;
  char_T c2_TRANSA;
  c2_m = 18;
  c2_n = 25;
  c2_k = 31;
  c2_alpha1 = 1.0;
  c2_lda = 18;
  c2_ldb = 31;
  c2_beta1 = 0.0;
  c2_ldc = 18;
  c2_TRANSB = 'N';
  c2_TRANSA = 'N';
  dgemm32(&c2_TRANSA, &c2_TRANSB, &c2_m, &c2_n, &c2_k, &c2_alpha1, &c2_A[0],
          &c2_lda, &c2_B[0], &c2_ldb, &c2_beta1, &c2_C[0], &c2_ldc);
}

static void c2_j_eml_xscal(SFc2_controllerInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_a, real_T c2_x[450], int32_T c2_ix0)
{
  int32_T c2_b_n;
  real_T c2_b_a;
  int32_T c2_b_ix0;
  int32_T c2_c_n;
  real_T c2_c_a;
  int32_T c2_c_ix0;
  int32_T c2_d_ix0;
  int32_T c2_d_a;
  int32_T c2_c;
  int32_T c2_b;
  int32_T c2_b_c;
  int32_T c2_e_a;
  int32_T c2_b_b;
  int32_T c2_i495;
  int32_T c2_f_a;
  int32_T c2_c_b;
  int32_T c2_g_a;
  int32_T c2_d_b;
  boolean_T c2_overflow;
  int32_T c2_k;
  int32_T c2_b_k;
  c2_b_n = c2_n;
  c2_b_a = c2_a;
  c2_b_ix0 = c2_ix0;
  c2_c_n = c2_b_n;
  c2_c_a = c2_b_a;
  c2_c_ix0 = c2_b_ix0;
  c2_d_ix0 = c2_c_ix0;
  c2_d_a = c2_c_n;
  c2_c = c2_d_a;
  c2_b = c2_c - 1;
  c2_b_c = c2_b;
  c2_e_a = c2_c_ix0;
  c2_b_b = c2_b_c;
  c2_i495 = c2_e_a + c2_b_b;
  c2_f_a = c2_d_ix0;
  c2_c_b = c2_i495;
  c2_g_a = c2_f_a;
  c2_d_b = c2_c_b;
  if (c2_g_a > c2_d_b) {
    c2_overflow = FALSE;
  } else {
    c2_overflow = (c2_d_b > 2147483646);
  }

  c2_b_check_forloop_overflow_error(chartInstance, c2_overflow);
  for (c2_k = c2_d_ix0; c2_k <= c2_i495; c2_k++) {
    c2_b_k = c2_k;
    c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c2_b_k), 1, 450, 1, 0) - 1] = c2_c_a * c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("",
      (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_k), 1, 450, 1, 0) - 1];
  }
}

static void c2_m_eml_xaxpy(SFc2_controllerInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_a, int32_T c2_ix0, real_T c2_y[450], int32_T c2_iy0)
{
  int32_T c2_b_n;
  real_T c2_b_a;
  int32_T c2_b_ix0;
  int32_T c2_b_iy0;
  int32_T c2_c_n;
  real_T c2_c_a;
  int32_T c2_c_ix0;
  int32_T c2_c_iy0;
  int32_T c2_d_a;
  int32_T c2_ix;
  int32_T c2_e_a;
  int32_T c2_iy;
  int32_T c2_f_a;
  int32_T c2_i496;
  int32_T c2_b;
  int32_T c2_b_b;
  boolean_T c2_overflow;
  int32_T c2_k;
  int32_T c2_g_a;
  int32_T c2_c;
  int32_T c2_h_a;
  int32_T c2_b_c;
  int32_T c2_i_a;
  int32_T c2_c_c;
  int32_T c2_j_a;
  int32_T c2_k_a;
  c2_b_n = c2_n;
  c2_b_a = c2_a;
  c2_b_ix0 = c2_ix0;
  c2_b_iy0 = c2_iy0;
  c2_c_n = c2_b_n;
  c2_c_a = c2_b_a;
  c2_c_ix0 = c2_b_ix0;
  c2_c_iy0 = c2_b_iy0;
  if (c2_c_n < 1) {
  } else if (c2_c_a == 0.0) {
  } else {
    c2_d_a = c2_c_ix0 - 1;
    c2_ix = c2_d_a;
    c2_e_a = c2_c_iy0 - 1;
    c2_iy = c2_e_a;
    c2_f_a = c2_c_n - 1;
    c2_i496 = c2_f_a;
    c2_b = c2_i496;
    c2_b_b = c2_b;
    if (0 > c2_b_b) {
      c2_overflow = FALSE;
    } else {
      c2_overflow = (c2_b_b > 2147483646);
    }

    c2_b_check_forloop_overflow_error(chartInstance, c2_overflow);
    for (c2_k = 0; c2_k <= c2_i496; c2_k++) {
      c2_g_a = c2_iy;
      c2_c = c2_g_a;
      c2_h_a = c2_iy;
      c2_b_c = c2_h_a;
      c2_i_a = c2_ix;
      c2_c_c = c2_i_a;
      c2_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c2_c + 1)), 1, 450, 1, 0) - 1] =
        c2_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c2_b_c + 1)), 1, 450, 1, 0) - 1] + c2_c_a *
        c2_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c2_c_c + 1)), 1, 450, 1, 0) - 1];
      c2_j_a = c2_ix + 1;
      c2_ix = c2_j_a;
      c2_k_a = c2_iy + 1;
      c2_iy = c2_k_a;
    }
  }
}

static void c2_k_eml_xscal(SFc2_controllerInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_a, real_T c2_x[18], int32_T c2_ix0)
{
  int32_T c2_b_n;
  real_T c2_b_a;
  int32_T c2_b_ix0;
  int32_T c2_c_n;
  real_T c2_c_a;
  int32_T c2_c_ix0;
  int32_T c2_d_ix0;
  int32_T c2_d_a;
  int32_T c2_c;
  int32_T c2_b;
  int32_T c2_b_c;
  int32_T c2_e_a;
  int32_T c2_b_b;
  int32_T c2_i497;
  int32_T c2_f_a;
  int32_T c2_c_b;
  int32_T c2_g_a;
  int32_T c2_d_b;
  boolean_T c2_overflow;
  int32_T c2_k;
  int32_T c2_b_k;
  c2_b_n = c2_n;
  c2_b_a = c2_a;
  c2_b_ix0 = c2_ix0;
  c2_c_n = c2_b_n;
  c2_c_a = c2_b_a;
  c2_c_ix0 = c2_b_ix0;
  c2_d_ix0 = c2_c_ix0;
  c2_d_a = c2_c_n;
  c2_c = c2_d_a;
  c2_b = c2_c - 1;
  c2_b_c = c2_b;
  c2_e_a = c2_c_ix0;
  c2_b_b = c2_b_c;
  c2_i497 = c2_e_a + c2_b_b;
  c2_f_a = c2_d_ix0;
  c2_c_b = c2_i497;
  c2_g_a = c2_f_a;
  c2_d_b = c2_c_b;
  if (c2_g_a > c2_d_b) {
    c2_overflow = FALSE;
  } else {
    c2_overflow = (c2_d_b > 2147483646);
  }

  c2_b_check_forloop_overflow_error(chartInstance, c2_overflow);
  for (c2_k = c2_d_ix0; c2_k <= c2_i497; c2_k++) {
    c2_b_k = c2_k;
    c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c2_b_k), 1, 18, 1, 0) - 1] = c2_c_a * c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("",
      (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_k), 1, 18, 1, 0) - 1];
  }
}

static void c2_n_eml_xaxpy(SFc2_controllerInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_a, real_T c2_x[450], int32_T c2_ix0, real_T c2_y[25], int32_T
  c2_iy0)
{
  int32_T c2_b_n;
  real_T c2_b_a;
  int32_T c2_b_ix0;
  int32_T c2_b_iy0;
  int32_T c2_c_n;
  real_T c2_c_a;
  int32_T c2_c_ix0;
  int32_T c2_c_iy0;
  int32_T c2_d_a;
  int32_T c2_ix;
  int32_T c2_e_a;
  int32_T c2_iy;
  int32_T c2_f_a;
  int32_T c2_i498;
  int32_T c2_b;
  int32_T c2_b_b;
  boolean_T c2_overflow;
  int32_T c2_k;
  int32_T c2_g_a;
  int32_T c2_c;
  int32_T c2_h_a;
  int32_T c2_b_c;
  int32_T c2_i_a;
  int32_T c2_c_c;
  int32_T c2_j_a;
  int32_T c2_k_a;
  c2_b_n = c2_n;
  c2_b_a = c2_a;
  c2_b_ix0 = c2_ix0;
  c2_b_iy0 = c2_iy0;
  c2_c_n = c2_b_n;
  c2_c_a = c2_b_a;
  c2_c_ix0 = c2_b_ix0;
  c2_c_iy0 = c2_b_iy0;
  if (c2_c_n < 1) {
  } else if (c2_c_a == 0.0) {
  } else {
    c2_d_a = c2_c_ix0 - 1;
    c2_ix = c2_d_a;
    c2_e_a = c2_c_iy0 - 1;
    c2_iy = c2_e_a;
    c2_f_a = c2_c_n - 1;
    c2_i498 = c2_f_a;
    c2_b = c2_i498;
    c2_b_b = c2_b;
    if (0 > c2_b_b) {
      c2_overflow = FALSE;
    } else {
      c2_overflow = (c2_b_b > 2147483646);
    }

    c2_b_check_forloop_overflow_error(chartInstance, c2_overflow);
    for (c2_k = 0; c2_k <= c2_i498; c2_k++) {
      c2_g_a = c2_iy;
      c2_c = c2_g_a;
      c2_h_a = c2_iy;
      c2_b_c = c2_h_a;
      c2_i_a = c2_ix;
      c2_c_c = c2_i_a;
      c2_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c2_c + 1)), 1, 25, 1, 0) - 1] =
        c2_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c2_b_c + 1)), 1, 25, 1, 0) - 1] + c2_c_a *
        c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c2_c_c + 1)), 1, 450, 1, 0) - 1];
      c2_j_a = c2_ix + 1;
      c2_ix = c2_j_a;
      c2_k_a = c2_iy + 1;
      c2_iy = c2_k_a;
    }
  }
}

static void c2_o_eml_xaxpy(SFc2_controllerInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_a, real_T c2_x[25], int32_T c2_ix0, real_T c2_y[450], int32_T
  c2_iy0)
{
  int32_T c2_b_n;
  real_T c2_b_a;
  int32_T c2_b_ix0;
  int32_T c2_b_iy0;
  int32_T c2_c_n;
  real_T c2_c_a;
  int32_T c2_c_ix0;
  int32_T c2_c_iy0;
  int32_T c2_d_a;
  int32_T c2_ix;
  int32_T c2_e_a;
  int32_T c2_iy;
  int32_T c2_f_a;
  int32_T c2_i499;
  int32_T c2_b;
  int32_T c2_b_b;
  boolean_T c2_overflow;
  int32_T c2_k;
  int32_T c2_g_a;
  int32_T c2_c;
  int32_T c2_h_a;
  int32_T c2_b_c;
  int32_T c2_i_a;
  int32_T c2_c_c;
  int32_T c2_j_a;
  int32_T c2_k_a;
  c2_b_n = c2_n;
  c2_b_a = c2_a;
  c2_b_ix0 = c2_ix0;
  c2_b_iy0 = c2_iy0;
  c2_c_n = c2_b_n;
  c2_c_a = c2_b_a;
  c2_c_ix0 = c2_b_ix0;
  c2_c_iy0 = c2_b_iy0;
  if (c2_c_n < 1) {
  } else if (c2_c_a == 0.0) {
  } else {
    c2_d_a = c2_c_ix0 - 1;
    c2_ix = c2_d_a;
    c2_e_a = c2_c_iy0 - 1;
    c2_iy = c2_e_a;
    c2_f_a = c2_c_n - 1;
    c2_i499 = c2_f_a;
    c2_b = c2_i499;
    c2_b_b = c2_b;
    if (0 > c2_b_b) {
      c2_overflow = FALSE;
    } else {
      c2_overflow = (c2_b_b > 2147483646);
    }

    c2_b_check_forloop_overflow_error(chartInstance, c2_overflow);
    for (c2_k = 0; c2_k <= c2_i499; c2_k++) {
      c2_g_a = c2_iy;
      c2_c = c2_g_a;
      c2_h_a = c2_iy;
      c2_b_c = c2_h_a;
      c2_i_a = c2_ix;
      c2_c_c = c2_i_a;
      c2_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c2_c + 1)), 1, 450, 1, 0) - 1] =
        c2_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c2_b_c + 1)), 1, 450, 1, 0) - 1] + c2_c_a *
        c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c2_c_c + 1)), 1, 25, 1, 0) - 1];
      c2_j_a = c2_ix + 1;
      c2_ix = c2_j_a;
      c2_k_a = c2_iy + 1;
      c2_iy = c2_k_a;
    }
  }
}

static void c2_p_eml_xaxpy(SFc2_controllerInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_a, int32_T c2_ix0, real_T c2_y[324], int32_T c2_iy0)
{
  int32_T c2_b_n;
  real_T c2_b_a;
  int32_T c2_b_ix0;
  int32_T c2_b_iy0;
  int32_T c2_c_n;
  real_T c2_c_a;
  int32_T c2_c_ix0;
  int32_T c2_c_iy0;
  int32_T c2_d_a;
  int32_T c2_ix;
  int32_T c2_e_a;
  int32_T c2_iy;
  int32_T c2_f_a;
  int32_T c2_i500;
  int32_T c2_b;
  int32_T c2_b_b;
  boolean_T c2_overflow;
  int32_T c2_k;
  int32_T c2_g_a;
  int32_T c2_c;
  int32_T c2_h_a;
  int32_T c2_b_c;
  int32_T c2_i_a;
  int32_T c2_c_c;
  int32_T c2_j_a;
  int32_T c2_k_a;
  c2_b_n = c2_n;
  c2_b_a = c2_a;
  c2_b_ix0 = c2_ix0;
  c2_b_iy0 = c2_iy0;
  c2_c_n = c2_b_n;
  c2_c_a = c2_b_a;
  c2_c_ix0 = c2_b_ix0;
  c2_c_iy0 = c2_b_iy0;
  if (c2_c_n < 1) {
  } else if (c2_c_a == 0.0) {
  } else {
    c2_d_a = c2_c_ix0 - 1;
    c2_ix = c2_d_a;
    c2_e_a = c2_c_iy0 - 1;
    c2_iy = c2_e_a;
    c2_f_a = c2_c_n - 1;
    c2_i500 = c2_f_a;
    c2_b = c2_i500;
    c2_b_b = c2_b;
    if (0 > c2_b_b) {
      c2_overflow = FALSE;
    } else {
      c2_overflow = (c2_b_b > 2147483646);
    }

    c2_b_check_forloop_overflow_error(chartInstance, c2_overflow);
    for (c2_k = 0; c2_k <= c2_i500; c2_k++) {
      c2_g_a = c2_iy;
      c2_c = c2_g_a;
      c2_h_a = c2_iy;
      c2_b_c = c2_h_a;
      c2_i_a = c2_ix;
      c2_c_c = c2_i_a;
      c2_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c2_c + 1)), 1, 324, 1, 0) - 1] =
        c2_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c2_b_c + 1)), 1, 324, 1, 0) - 1] + c2_c_a *
        c2_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c2_c_c + 1)), 1, 324, 1, 0) - 1];
      c2_j_a = c2_ix + 1;
      c2_ix = c2_j_a;
      c2_k_a = c2_iy + 1;
      c2_iy = c2_k_a;
    }
  }
}

static void c2_l_eml_xscal(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_a, real_T c2_x[324], int32_T c2_ix0)
{
  real_T c2_b_a;
  int32_T c2_b_ix0;
  real_T c2_c_a;
  int32_T c2_c_ix0;
  int32_T c2_d_ix0;
  int32_T c2_d_a;
  int32_T c2_i501;
  int32_T c2_e_a;
  int32_T c2_b;
  int32_T c2_f_a;
  int32_T c2_b_b;
  boolean_T c2_overflow;
  int32_T c2_k;
  int32_T c2_b_k;
  c2_b_a = c2_a;
  c2_b_ix0 = c2_ix0;
  c2_c_a = c2_b_a;
  c2_c_ix0 = c2_b_ix0;
  c2_d_ix0 = c2_c_ix0;
  c2_d_a = c2_c_ix0 + 17;
  c2_i501 = c2_d_a;
  c2_e_a = c2_d_ix0;
  c2_b = c2_i501;
  c2_f_a = c2_e_a;
  c2_b_b = c2_b;
  if (c2_f_a > c2_b_b) {
    c2_overflow = FALSE;
  } else {
    c2_overflow = (c2_b_b > 2147483646);
  }

  c2_b_check_forloop_overflow_error(chartInstance, c2_overflow);
  for (c2_k = c2_d_ix0; c2_k <= c2_i501; c2_k++) {
    c2_b_k = c2_k;
    c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c2_b_k), 1, 324, 1, 0) - 1] = c2_c_a * c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("",
      (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_k), 1, 324, 1, 0) - 1];
  }
}

static void c2_g_eml_xrot(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_x[324], int32_T c2_ix0, int32_T c2_iy0, real_T c2_c, real_T c2_s)
{
  int32_T c2_b_ix0;
  int32_T c2_b_iy0;
  real_T c2_b_c;
  real_T c2_b_s;
  int32_T c2_c_ix0;
  int32_T c2_c_iy0;
  real_T c2_c_c;
  real_T c2_c_s;
  int32_T c2_ix;
  int32_T c2_iy;
  int32_T c2_k;
  real_T c2_a;
  real_T c2_b;
  real_T c2_y;
  real_T c2_b_a;
  real_T c2_b_b;
  real_T c2_b_y;
  real_T c2_temp;
  real_T c2_c_a;
  real_T c2_c_b;
  real_T c2_c_y;
  real_T c2_d_a;
  real_T c2_d_b;
  real_T c2_d_y;
  int32_T c2_e_a;
  int32_T c2_f_a;
  c2_b_ix0 = c2_ix0;
  c2_b_iy0 = c2_iy0;
  c2_b_c = c2_c;
  c2_b_s = c2_s;
  c2_c_ix0 = c2_b_ix0;
  c2_c_iy0 = c2_b_iy0;
  c2_c_c = c2_b_c;
  c2_c_s = c2_b_s;
  c2_ix = c2_c_ix0;
  c2_iy = c2_c_iy0;
  c2_b_check_forloop_overflow_error(chartInstance, FALSE);
  for (c2_k = 1; c2_k < 19; c2_k++) {
    c2_a = c2_c_c;
    c2_b = c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c2_ix), 1, 324, 1, 0) - 1];
    c2_y = c2_a * c2_b;
    c2_b_a = c2_c_s;
    c2_b_b = c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c2_iy), 1, 324, 1, 0) - 1];
    c2_b_y = c2_b_a * c2_b_b;
    c2_temp = c2_y + c2_b_y;
    c2_c_a = c2_c_c;
    c2_c_b = c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c2_iy), 1, 324, 1, 0) - 1];
    c2_c_y = c2_c_a * c2_c_b;
    c2_d_a = c2_c_s;
    c2_d_b = c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c2_ix), 1, 324, 1, 0) - 1];
    c2_d_y = c2_d_a * c2_d_b;
    c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c2_iy), 1, 324, 1, 0) - 1] = c2_c_y - c2_d_y;
    c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c2_ix), 1, 324, 1, 0) - 1] = c2_temp;
    c2_e_a = c2_iy + 1;
    c2_iy = c2_e_a;
    c2_f_a = c2_ix + 1;
    c2_ix = c2_f_a;
  }
}

static void c2_h_eml_xrot(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_x[450], int32_T c2_ix0, int32_T c2_iy0, real_T c2_c, real_T c2_s)
{
  int32_T c2_b_ix0;
  int32_T c2_b_iy0;
  real_T c2_b_c;
  real_T c2_b_s;
  int32_T c2_c_ix0;
  int32_T c2_c_iy0;
  real_T c2_c_c;
  real_T c2_c_s;
  int32_T c2_ix;
  int32_T c2_iy;
  int32_T c2_k;
  real_T c2_a;
  real_T c2_b;
  real_T c2_y;
  real_T c2_b_a;
  real_T c2_b_b;
  real_T c2_b_y;
  real_T c2_temp;
  real_T c2_c_a;
  real_T c2_c_b;
  real_T c2_c_y;
  real_T c2_d_a;
  real_T c2_d_b;
  real_T c2_d_y;
  int32_T c2_e_a;
  int32_T c2_f_a;
  c2_b_ix0 = c2_ix0;
  c2_b_iy0 = c2_iy0;
  c2_b_c = c2_c;
  c2_b_s = c2_s;
  c2_c_ix0 = c2_b_ix0;
  c2_c_iy0 = c2_b_iy0;
  c2_c_c = c2_b_c;
  c2_c_s = c2_b_s;
  c2_ix = c2_c_ix0;
  c2_iy = c2_c_iy0;
  c2_check_forloop_overflow_error(chartInstance);
  for (c2_k = 1; c2_k < 26; c2_k++) {
    c2_a = c2_c_c;
    c2_b = c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c2_ix), 1, 450, 1, 0) - 1];
    c2_y = c2_a * c2_b;
    c2_b_a = c2_c_s;
    c2_b_b = c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c2_iy), 1, 450, 1, 0) - 1];
    c2_b_y = c2_b_a * c2_b_b;
    c2_temp = c2_y + c2_b_y;
    c2_c_a = c2_c_c;
    c2_c_b = c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c2_iy), 1, 450, 1, 0) - 1];
    c2_c_y = c2_c_a * c2_c_b;
    c2_d_a = c2_c_s;
    c2_d_b = c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c2_ix), 1, 450, 1, 0) - 1];
    c2_d_y = c2_d_a * c2_d_b;
    c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c2_iy), 1, 450, 1, 0) - 1] = c2_c_y - c2_d_y;
    c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c2_ix), 1, 450, 1, 0) - 1] = c2_temp;
    c2_e_a = c2_iy + 1;
    c2_iy = c2_e_a;
    c2_f_a = c2_ix + 1;
    c2_ix = c2_f_a;
  }
}

static void c2_i_eml_xswap(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_x[324], int32_T c2_ix0, int32_T c2_iy0)
{
  int32_T c2_b_ix0;
  int32_T c2_b_iy0;
  int32_T c2_c_ix0;
  int32_T c2_c_iy0;
  int32_T c2_ix;
  int32_T c2_iy;
  int32_T c2_k;
  real_T c2_temp;
  int32_T c2_a;
  int32_T c2_b_a;
  c2_b_ix0 = c2_ix0;
  c2_b_iy0 = c2_iy0;
  c2_c_ix0 = c2_b_ix0;
  c2_c_iy0 = c2_b_iy0;
  c2_ix = c2_c_ix0;
  c2_iy = c2_c_iy0;
  c2_b_check_forloop_overflow_error(chartInstance, FALSE);
  for (c2_k = 1; c2_k < 19; c2_k++) {
    c2_temp = c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
      "", (real_T)c2_ix), 1, 324, 1, 0) - 1];
    c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c2_ix), 1, 324, 1, 0) - 1] = c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
      _SFD_INTEGER_CHECK("", (real_T)c2_iy), 1, 324, 1, 0) - 1];
    c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c2_iy), 1, 324, 1, 0) - 1] = c2_temp;
    c2_a = c2_ix + 1;
    c2_ix = c2_a;
    c2_b_a = c2_iy + 1;
    c2_iy = c2_b_a;
  }
}

static void c2_j_eml_xswap(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_x[450], int32_T c2_ix0, int32_T c2_iy0)
{
  int32_T c2_b_ix0;
  int32_T c2_b_iy0;
  int32_T c2_c_ix0;
  int32_T c2_c_iy0;
  int32_T c2_ix;
  int32_T c2_iy;
  int32_T c2_k;
  real_T c2_temp;
  int32_T c2_a;
  int32_T c2_b_a;
  c2_b_ix0 = c2_ix0;
  c2_b_iy0 = c2_iy0;
  c2_c_ix0 = c2_b_ix0;
  c2_c_iy0 = c2_b_iy0;
  c2_ix = c2_c_ix0;
  c2_iy = c2_c_iy0;
  c2_check_forloop_overflow_error(chartInstance);
  for (c2_k = 1; c2_k < 26; c2_k++) {
    c2_temp = c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
      "", (real_T)c2_ix), 1, 450, 1, 0) - 1];
    c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c2_ix), 1, 450, 1, 0) - 1] = c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
      _SFD_INTEGER_CHECK("", (real_T)c2_iy), 1, 450, 1, 0) - 1];
    c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c2_iy), 1, 450, 1, 0) - 1] = c2_temp;
    c2_a = c2_ix + 1;
    c2_ix = c2_a;
    c2_b_a = c2_iy + 1;
    c2_iy = c2_b_a;
  }
}

static void c2_m_eml_xgemm(SFc2_controllerInstanceStruct *chartInstance, int32_T
  c2_k, real_T c2_A[324], real_T c2_B[450], real_T c2_C[450])
{
  int32_T c2_b_k;
  int32_T c2_m;
  int32_T c2_n;
  int32_T c2_c_k;
  real_T c2_alpha1;
  int32_T c2_lda;
  int32_T c2_ldb;
  real_T c2_beta1;
  int32_T c2_ldc;
  char_T c2_TRANSB;
  char_T c2_TRANSA;
  c2_b_k = c2_k;
  c2_b_below_threshold(chartInstance);
  if (c2_b_k < 1) {
  } else {
    c2_m = 18;
    c2_n = 25;
    c2_c_k = c2_b_k;
    c2_alpha1 = 1.0;
    c2_lda = 18;
    c2_ldb = 25;
    c2_beta1 = 0.0;
    c2_ldc = 18;
    c2_TRANSB = 'C';
    c2_TRANSA = 'N';
    dgemm32(&c2_TRANSA, &c2_TRANSB, &c2_m, &c2_n, &c2_c_k, &c2_alpha1, &c2_A[0],
            &c2_lda, &c2_B[0], &c2_ldb, &c2_beta1, &c2_C[0], &c2_ldc);
  }
}

static void c2_n_eml_xgemm(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_A[450], real_T c2_B[558], real_T c2_C[775])
{
  int32_T c2_m;
  int32_T c2_n;
  int32_T c2_k;
  real_T c2_alpha1;
  int32_T c2_lda;
  int32_T c2_ldb;
  real_T c2_beta1;
  int32_T c2_ldc;
  char_T c2_TRANSB;
  char_T c2_TRANSA;
  c2_m = 25;
  c2_n = 31;
  c2_k = 18;
  c2_alpha1 = 1.0;
  c2_lda = 25;
  c2_ldb = 18;
  c2_beta1 = 0.0;
  c2_ldc = 25;
  c2_TRANSB = 'N';
  c2_TRANSA = 'N';
  dgemm32(&c2_TRANSA, &c2_TRANSB, &c2_m, &c2_n, &c2_k, &c2_alpha1, &c2_A[0],
          &c2_lda, &c2_B[0], &c2_ldb, &c2_beta1, &c2_C[0], &c2_ldc);
}

static void c2_o_eml_xgemm(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_A[775], real_T c2_B[775], real_T c2_C[625])
{
  int32_T c2_m;
  int32_T c2_n;
  int32_T c2_k;
  real_T c2_alpha1;
  int32_T c2_lda;
  int32_T c2_ldb;
  real_T c2_beta1;
  int32_T c2_ldc;
  char_T c2_TRANSB;
  char_T c2_TRANSA;
  c2_m = 25;
  c2_n = 25;
  c2_k = 31;
  c2_alpha1 = 1.0;
  c2_lda = 25;
  c2_ldb = 31;
  c2_beta1 = 0.0;
  c2_ldc = 25;
  c2_TRANSB = 'N';
  c2_TRANSA = 'N';
  dgemm32(&c2_TRANSA, &c2_TRANSB, &c2_m, &c2_n, &c2_k, &c2_alpha1, &c2_A[0],
          &c2_lda, &c2_B[0], &c2_ldb, &c2_beta1, &c2_C[0], &c2_ldc);
}

static void c2_p_eml_xgemm(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_A[558], real_T c2_B[31], real_T c2_C[18])
{
  int32_T c2_m;
  int32_T c2_n;
  int32_T c2_k;
  real_T c2_alpha1;
  int32_T c2_lda;
  int32_T c2_ldb;
  real_T c2_beta1;
  int32_T c2_ldc;
  char_T c2_TRANSB;
  char_T c2_TRANSA;
  c2_m = 18;
  c2_n = 1;
  c2_k = 31;
  c2_alpha1 = 1.0;
  c2_lda = 18;
  c2_ldb = 31;
  c2_beta1 = 0.0;
  c2_ldc = 18;
  c2_TRANSB = 'N';
  c2_TRANSA = 'N';
  dgemm32(&c2_TRANSA, &c2_TRANSB, &c2_m, &c2_n, &c2_k, &c2_alpha1, &c2_A[0],
          &c2_lda, &c2_B[0], &c2_ldb, &c2_beta1, &c2_C[0], &c2_ldc);
}

static void c2_q_eml_xgemm(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_A[558], real_T c2_B[558], real_T c2_C[324])
{
  int32_T c2_m;
  int32_T c2_n;
  int32_T c2_k;
  real_T c2_alpha1;
  int32_T c2_lda;
  int32_T c2_ldb;
  real_T c2_beta1;
  int32_T c2_ldc;
  char_T c2_TRANSB;
  char_T c2_TRANSA;
  c2_m = 18;
  c2_n = 18;
  c2_k = 31;
  c2_alpha1 = 1.0;
  c2_lda = 18;
  c2_ldb = 31;
  c2_beta1 = 0.0;
  c2_ldc = 18;
  c2_TRANSB = 'N';
  c2_TRANSA = 'N';
  dgemm32(&c2_TRANSA, &c2_TRANSB, &c2_m, &c2_n, &c2_k, &c2_alpha1, &c2_A[0],
          &c2_lda, &c2_B[0], &c2_ldb, &c2_beta1, &c2_C[0], &c2_ldc);
}

static void c2_r_eml_xgemm(SFc2_controllerInstanceStruct *chartInstance, real_T
  c2_A[625], real_T c2_B[25], real_T c2_C[25])
{
  int32_T c2_m;
  int32_T c2_n;
  int32_T c2_k;
  real_T c2_alpha1;
  int32_T c2_lda;
  int32_T c2_ldb;
  real_T c2_beta1;
  int32_T c2_ldc;
  char_T c2_TRANSB;
  char_T c2_TRANSA;
  c2_m = 25;
  c2_n = 1;
  c2_k = 25;
  c2_alpha1 = 1.0;
  c2_lda = 25;
  c2_ldb = 25;
  c2_beta1 = 0.0;
  c2_ldc = 25;
  c2_TRANSB = 'N';
  c2_TRANSA = 'N';
  dgemm32(&c2_TRANSA, &c2_TRANSB, &c2_m, &c2_n, &c2_k, &c2_alpha1, &c2_A[0],
          &c2_lda, &c2_B[0], &c2_ldb, &c2_beta1, &c2_C[0], &c2_ldc);
}

static void init_dsm_address_info(SFc2_controllerInstanceStruct *chartInstance)
{
}

/* SFunction Glue Code */
void sf_c2_controller_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(4207009739U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(1460165875U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(2090145415U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1364959912U);
}

mxArray *sf_c2_controller_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("d2WlP80jrFyW0QPAteBoe");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,20,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(25);
      pr[1] = (double)(1);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(25);
      pr[1] = (double)(1);
      mxSetField(mxData,2,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,2,"type",mxType);
    }

    mxSetField(mxData,2,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(31);
      pr[1] = (double)(1);
      mxSetField(mxData,3,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,3,"type",mxType);
    }

    mxSetField(mxData,3,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(18);
      pr[1] = (double)(1);
      mxSetField(mxData,4,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,4,"type",mxType);
    }

    mxSetField(mxData,4,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(31);
      pr[1] = (double)(31);
      mxSetField(mxData,5,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,5,"type",mxType);
    }

    mxSetField(mxData,5,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(31);
      pr[1] = (double)(1);
      mxSetField(mxData,6,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,6,"type",mxType);
    }

    mxSetField(mxData,6,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(31);
      pr[1] = (double)(1);
      mxSetField(mxData,7,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,7,"type",mxType);
    }

    mxSetField(mxData,7,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(6);
      pr[1] = (double)(1);
      mxSetField(mxData,8,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,8,"type",mxType);
    }

    mxSetField(mxData,8,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(7);
      pr[1] = (double)(1);
      mxSetField(mxData,9,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,9,"type",mxType);
    }

    mxSetField(mxData,9,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(12);
      pr[1] = (double)(31);
      mxSetField(mxData,10,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,10,"type",mxType);
    }

    mxSetField(mxData,10,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(12);
      pr[1] = (double)(31);
      mxSetField(mxData,11,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,11,"type",mxType);
    }

    mxSetField(mxData,11,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(12);
      pr[1] = (double)(1);
      mxSetField(mxData,12,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,12,"type",mxType);
    }

    mxSetField(mxData,12,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(12);
      pr[1] = (double)(1);
      mxSetField(mxData,13,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,13,"type",mxType);
    }

    mxSetField(mxData,13,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(3);
      pr[1] = (double)(1);
      mxSetField(mxData,14,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,14,"type",mxType);
    }

    mxSetField(mxData,14,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(6);
      pr[1] = (double)(31);
      mxSetField(mxData,15,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,15,"type",mxType);
    }

    mxSetField(mxData,15,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(3);
      pr[1] = (double)(3);
      mxSetField(mxData,16,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,16,"type",mxType);
    }

    mxSetField(mxData,16,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(4);
      pr[1] = (double)(1);
      mxSetField(mxData,17,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,17,"type",mxType);
    }

    mxSetField(mxData,17,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(25);
      pr[1] = (double)(1);
      mxSetField(mxData,18,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,18,"type",mxType);
    }

    mxSetField(mxData,18,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(3);
      pr[1] = (double)(1);
      mxSetField(mxData,19,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,19,"type",mxType);
    }

    mxSetField(mxData,19,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,
                mxREAL));
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,3,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(25);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(3);
      pr[1] = (double)(1);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,2,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,2,"type",mxType);
    }

    mxSetField(mxData,2,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  return(mxAutoinheritanceInfo);
}

static const mxArray *sf_get_sim_state_info_c2_controller(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x4'type','srcId','name','auxInfo'{{M[1],M[30],T\"CoMError\",},{M[1],M[54],T\"normCoMError\",},{M[1],M[5],T\"tau\",},{M[8],M[0],T\"is_active_c2_controller\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 4, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c2_controller_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc2_controllerInstanceStruct *chartInstance;
    chartInstance = (SFc2_controllerInstanceStruct *) ((ChartInfoStruct *)
      (ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (_controllerMachineNumber_,
           2,
           1,
           1,
           23,
           0,
           0,
           0,
           0,
           1,
           &(chartInstance->chartNumber),
           &(chartInstance->instanceNumber),
           ssGetPath(S),
           (void *)S);
        if (chartAlreadyPresent==0) {
          /* this is the first instance */
          init_script_number_translation(_controllerMachineNumber_,
            chartInstance->chartNumber);
          sf_debug_set_chart_disable_implicit_casting(_controllerMachineNumber_,
            chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(_controllerMachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);
          _SFD_SET_DATA_PROPS(0,1,1,0,"demoState");
          _SFD_SET_DATA_PROPS(1,1,1,0,"q");
          _SFD_SET_DATA_PROPS(2,1,1,0,"qInit");
          _SFD_SET_DATA_PROPS(3,1,1,0,"qD");
          _SFD_SET_DATA_PROPS(4,1,1,0,"measuredExternalForces");
          _SFD_SET_DATA_PROPS(5,1,1,0,"M");
          _SFD_SET_DATA_PROPS(6,1,1,0,"h");
          _SFD_SET_DATA_PROPS(7,1,1,0,"g");
          _SFD_SET_DATA_PROPS(8,1,1,0,"H");
          _SFD_SET_DATA_PROPS(9,1,1,0,"PosRightFoot");
          _SFD_SET_DATA_PROPS(10,1,1,0,"feetJacobians");
          _SFD_SET_DATA_PROPS(11,1,1,0,"handsJacobians");
          _SFD_SET_DATA_PROPS(12,2,0,1,"tau");
          _SFD_SET_DATA_PROPS(13,1,1,0,"feetJdqd");
          _SFD_SET_DATA_PROPS(14,1,1,0,"handsJddq");
          _SFD_SET_DATA_PROPS(15,1,1,0,"xcom");
          _SFD_SET_DATA_PROPS(16,1,1,0,"Jcom");
          _SFD_SET_DATA_PROPS(17,1,1,0,"Desired_x_dx_ddx_CoM");
          _SFD_SET_DATA_PROPS(18,1,1,0,"kH");
          _SFD_SET_DATA_PROPS(19,1,1,0,"kImp");
          _SFD_SET_DATA_PROPS(20,2,0,1,"CoMError");
          _SFD_SET_DATA_PROPS(21,1,1,0,"IntErrorCoM");
          _SFD_SET_DATA_PROPS(22,2,0,1,"normCoMError");
          _SFD_STATE_INFO(0,0,2);
          _SFD_CH_SUBSTATE_COUNT(0);
          _SFD_CH_SUBSTATE_DECOMP(0);
        }

        _SFD_CV_INIT_CHART(0,0,0,0);

        {
          _SFD_CV_INIT_STATE(0,0,0,0,0,0,NULL,NULL);
        }

        _SFD_CV_INIT_TRANS(0,0,NULL,NULL,0,NULL);

        /* Initialization of MATLAB Function Model Coverage */
        _SFD_CV_INIT_EML(0,1,1,4,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,2208);
        _SFD_CV_INIT_EML_IF(0,1,0,1162,1179,-1,1231);
        _SFD_CV_INIT_EML_IF(0,1,1,1233,1250,-1,1324);
        _SFD_CV_INIT_EML_IF(0,1,2,1326,1343,-1,1417);
        _SFD_CV_INIT_EML_IF(0,1,3,1419,1436,-1,1475);
        _SFD_CV_INIT_SCRIPT(0,1,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_SCRIPT_FCN(0,0,"Sf",11,-1,135);
        _SFD_TRANS_COV_WTS(0,0,0,1,0);
        if (chartAlreadyPresent==0) {
          _SFD_TRANS_COV_MAPS(0,
                              0,NULL,NULL,
                              0,NULL,NULL,
                              1,NULL,NULL,
                              0,NULL,NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)NULL);

        {
          unsigned int dimVector[1];
          dimVector[0]= 25;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_c_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 25;
          _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_c_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 31;
          _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_k_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 18;
          dimVector[1]= 1;
          _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_m_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 31;
          dimVector[1]= 31;
          _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_l_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 31;
          _SFD_SET_DATA_COMPILED_PROPS(6,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_k_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 31;
          _SFD_SET_DATA_COMPILED_PROPS(7,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_k_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 6;
          _SFD_SET_DATA_COMPILED_PROPS(8,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_j_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 7;
          _SFD_SET_DATA_COMPILED_PROPS(9,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_i_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 12;
          dimVector[1]= 31;
          _SFD_SET_DATA_COMPILED_PROPS(10,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_h_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 12;
          dimVector[1]= 31;
          _SFD_SET_DATA_COMPILED_PROPS(11,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_h_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 25;
          _SFD_SET_DATA_COMPILED_PROPS(12,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_c_sf_marshallOut,(MexInFcnForType)
            c2_c_sf_marshallIn);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 12;
          dimVector[1]= 1;
          _SFD_SET_DATA_COMPILED_PROPS(13,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_g_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 12;
          dimVector[1]= 1;
          _SFD_SET_DATA_COMPILED_PROPS(14,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_g_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(15,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_b_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 6;
          dimVector[1]= 31;
          _SFD_SET_DATA_COMPILED_PROPS(16,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_f_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 3;
          dimVector[1]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(17,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_e_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 4;
          _SFD_SET_DATA_COMPILED_PROPS(18,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_d_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 25;
          _SFD_SET_DATA_COMPILED_PROPS(19,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_c_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(20,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_b_sf_marshallOut,(MexInFcnForType)
            c2_b_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(21,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_b_sf_marshallOut,(MexInFcnForType)NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(22,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)c2_sf_marshallIn);

        {
          real_T *c2_demoState;
          real_T *c2_normCoMError;
          real_T (*c2_q)[25];
          real_T (*c2_qInit)[25];
          real_T (*c2_qD)[31];
          real_T (*c2_measuredExternalForces)[18];
          real_T (*c2_M)[961];
          real_T (*c2_h)[31];
          real_T (*c2_g)[31];
          real_T (*c2_H)[6];
          real_T (*c2_PosRightFoot)[7];
          real_T (*c2_feetJacobians)[372];
          real_T (*c2_handsJacobians)[372];
          real_T (*c2_tau)[25];
          real_T (*c2_feetJdqd)[12];
          real_T (*c2_handsJddq)[12];
          real_T (*c2_xcom)[3];
          real_T (*c2_Jcom)[186];
          real_T (*c2_Desired_x_dx_ddx_CoM)[9];
          real_T (*c2_kH)[4];
          real_T (*c2_kImp)[25];
          real_T (*c2_CoMError)[3];
          real_T (*c2_IntErrorCoM)[3];
          c2_normCoMError = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
          c2_IntErrorCoM = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S,
            19);
          c2_CoMError = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 2);
          c2_kImp = (real_T (*)[25])ssGetInputPortSignal(chartInstance->S, 18);
          c2_kH = (real_T (*)[4])ssGetInputPortSignal(chartInstance->S, 17);
          c2_Desired_x_dx_ddx_CoM = (real_T (*)[9])ssGetInputPortSignal
            (chartInstance->S, 16);
          c2_Jcom = (real_T (*)[186])ssGetInputPortSignal(chartInstance->S, 15);
          c2_xcom = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 14);
          c2_handsJddq = (real_T (*)[12])ssGetInputPortSignal(chartInstance->S,
            13);
          c2_feetJdqd = (real_T (*)[12])ssGetInputPortSignal(chartInstance->S,
            12);
          c2_tau = (real_T (*)[25])ssGetOutputPortSignal(chartInstance->S, 1);
          c2_handsJacobians = (real_T (*)[372])ssGetInputPortSignal
            (chartInstance->S, 11);
          c2_feetJacobians = (real_T (*)[372])ssGetInputPortSignal
            (chartInstance->S, 10);
          c2_PosRightFoot = (real_T (*)[7])ssGetInputPortSignal(chartInstance->S,
            9);
          c2_H = (real_T (*)[6])ssGetInputPortSignal(chartInstance->S, 8);
          c2_g = (real_T (*)[31])ssGetInputPortSignal(chartInstance->S, 7);
          c2_h = (real_T (*)[31])ssGetInputPortSignal(chartInstance->S, 6);
          c2_M = (real_T (*)[961])ssGetInputPortSignal(chartInstance->S, 5);
          c2_measuredExternalForces = (real_T (*)[18])ssGetInputPortSignal
            (chartInstance->S, 4);
          c2_qD = (real_T (*)[31])ssGetInputPortSignal(chartInstance->S, 3);
          c2_qInit = (real_T (*)[25])ssGetInputPortSignal(chartInstance->S, 2);
          c2_q = (real_T (*)[25])ssGetInputPortSignal(chartInstance->S, 1);
          c2_demoState = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, c2_demoState);
          _SFD_SET_DATA_VALUE_PTR(1U, *c2_q);
          _SFD_SET_DATA_VALUE_PTR(2U, *c2_qInit);
          _SFD_SET_DATA_VALUE_PTR(3U, *c2_qD);
          _SFD_SET_DATA_VALUE_PTR(4U, *c2_measuredExternalForces);
          _SFD_SET_DATA_VALUE_PTR(5U, *c2_M);
          _SFD_SET_DATA_VALUE_PTR(6U, *c2_h);
          _SFD_SET_DATA_VALUE_PTR(7U, *c2_g);
          _SFD_SET_DATA_VALUE_PTR(8U, *c2_H);
          _SFD_SET_DATA_VALUE_PTR(9U, *c2_PosRightFoot);
          _SFD_SET_DATA_VALUE_PTR(10U, *c2_feetJacobians);
          _SFD_SET_DATA_VALUE_PTR(11U, *c2_handsJacobians);
          _SFD_SET_DATA_VALUE_PTR(12U, *c2_tau);
          _SFD_SET_DATA_VALUE_PTR(13U, *c2_feetJdqd);
          _SFD_SET_DATA_VALUE_PTR(14U, *c2_handsJddq);
          _SFD_SET_DATA_VALUE_PTR(15U, *c2_xcom);
          _SFD_SET_DATA_VALUE_PTR(16U, *c2_Jcom);
          _SFD_SET_DATA_VALUE_PTR(17U, *c2_Desired_x_dx_ddx_CoM);
          _SFD_SET_DATA_VALUE_PTR(18U, *c2_kH);
          _SFD_SET_DATA_VALUE_PTR(19U, *c2_kImp);
          _SFD_SET_DATA_VALUE_PTR(20U, *c2_CoMError);
          _SFD_SET_DATA_VALUE_PTR(21U, *c2_IntErrorCoM);
          _SFD_SET_DATA_VALUE_PTR(22U, c2_normCoMError);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration(_controllerMachineNumber_,
        chartInstance->chartNumber,chartInstance->instanceNumber);
    }
  }
}

static const char* sf_get_instance_specialization()
{
  return "KqRpz7O27ajkGn27KWKyOE";
}

static void sf_opaque_initialize_c2_controller(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc2_controllerInstanceStruct*) chartInstanceVar
    )->S,0);
  initialize_params_c2_controller((SFc2_controllerInstanceStruct*)
    chartInstanceVar);
  initialize_c2_controller((SFc2_controllerInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_enable_c2_controller(void *chartInstanceVar)
{
  enable_c2_controller((SFc2_controllerInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c2_controller(void *chartInstanceVar)
{
  disable_c2_controller((SFc2_controllerInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_gateway_c2_controller(void *chartInstanceVar)
{
  sf_c2_controller((SFc2_controllerInstanceStruct*) chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c2_controller(SimStruct* S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c2_controller
    ((SFc2_controllerInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c2_controller();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_raw2high'.\n");
  }

  return plhs[0];
}

extern void sf_internal_set_sim_state_c2_controller(SimStruct* S, const mxArray *
  st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c2_controller();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c2_controller((SFc2_controllerInstanceStruct*)
    chartInfo->chartInstance, mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c2_controller(SimStruct* S)
{
  return sf_internal_get_sim_state_c2_controller(S);
}

static void sf_opaque_set_sim_state_c2_controller(SimStruct* S, const mxArray
  *st)
{
  sf_internal_set_sim_state_c2_controller(S, st);
}

static void sf_opaque_terminate_c2_controller(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc2_controllerInstanceStruct*) chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
    }

    finalize_c2_controller((SFc2_controllerInstanceStruct*) chartInstanceVar);
    free((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }

  unload_controller_optimization_info();
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc2_controller((SFc2_controllerInstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c2_controller(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c2_controller((SFc2_controllerInstanceStruct*)
      (((ChartInfoStruct *)ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c2_controller(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_controller_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(S,sf_get_instance_specialization(),infoStruct,
      2);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(S,sf_get_instance_specialization(),
                infoStruct,2,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop(S,
      sf_get_instance_specialization(),infoStruct,2,
      "gatewayCannotBeInlinedMultipleTimes"));
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 4, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 5, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 6, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 7, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 8, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 9, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 10, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 11, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 12, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 13, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 14, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 15, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 16, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 17, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 18, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 19, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,2,20);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,2,3);
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,2);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(1731472093U));
  ssSetChecksum1(S,(541206972U));
  ssSetChecksum2(S,(1937740940U));
  ssSetChecksum3(S,(2947810416U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c2_controller(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c2_controller(SimStruct *S)
{
  SFc2_controllerInstanceStruct *chartInstance;
  chartInstance = (SFc2_controllerInstanceStruct *)malloc(sizeof
    (SFc2_controllerInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc2_controllerInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway = sf_opaque_gateway_c2_controller;
  chartInstance->chartInfo.initializeChart = sf_opaque_initialize_c2_controller;
  chartInstance->chartInfo.terminateChart = sf_opaque_terminate_c2_controller;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c2_controller;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c2_controller;
  chartInstance->chartInfo.getSimState = sf_opaque_get_sim_state_c2_controller;
  chartInstance->chartInfo.setSimState = sf_opaque_set_sim_state_c2_controller;
  chartInstance->chartInfo.getSimStateInfo = sf_get_sim_state_info_c2_controller;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c2_controller;
  chartInstance->chartInfo.mdlStart = mdlStart_c2_controller;
  chartInstance->chartInfo.mdlSetWorkWidths = mdlSetWorkWidths_c2_controller;
  chartInstance->chartInfo.extModeExec = NULL;
  chartInstance->chartInfo.restoreLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.restoreBeforeLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.storeCurrentConfiguration = NULL;
  chartInstance->S = S;
  ssSetUserData(S,(void *)(&(chartInstance->chartInfo)));/* register the chart instance with simstruct */
  init_dsm_address_info(chartInstance);
  if (!sim_mode_is_rtw_gen(S)) {
  }

  sf_opaque_init_subchart_simstructs(chartInstance->chartInfo.chartInstance);
  chart_debug_initialization(S,1);
}

void c2_controller_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c2_controller(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c2_controller(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c2_controller(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c2_controller_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
