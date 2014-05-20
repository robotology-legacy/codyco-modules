/* Include files */

#include "blascompat32.h"
#include "WBCLibrary_sfun.h"
#include "c2_WBCLibrary.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "WBCLibrary_sfun_debug_macros.h"

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static const char * c2_debug_family_names[5] = { "nargin", "nargout", "JcIn",
  "qD", "y" };

/* Function Declarations */
static void initialize_c2_WBCLibrary(SFc2_WBCLibraryInstanceStruct
  *chartInstance);
static void initialize_params_c2_WBCLibrary(SFc2_WBCLibraryInstanceStruct
  *chartInstance);
static void enable_c2_WBCLibrary(SFc2_WBCLibraryInstanceStruct *chartInstance);
static void disable_c2_WBCLibrary(SFc2_WBCLibraryInstanceStruct *chartInstance);
static void c2_update_debugger_state_c2_WBCLibrary(SFc2_WBCLibraryInstanceStruct
  *chartInstance);
static const mxArray *get_sim_state_c2_WBCLibrary(SFc2_WBCLibraryInstanceStruct *
  chartInstance);
static void set_sim_state_c2_WBCLibrary(SFc2_WBCLibraryInstanceStruct
  *chartInstance, const mxArray *c2_st);
static void finalize_c2_WBCLibrary(SFc2_WBCLibraryInstanceStruct *chartInstance);
static void sf_c2_WBCLibrary(SFc2_WBCLibraryInstanceStruct *chartInstance);
static void c2_chartstep_c2_WBCLibrary(SFc2_WBCLibraryInstanceStruct
  *chartInstance);
static void initSimStructsc2_WBCLibrary(SFc2_WBCLibraryInstanceStruct
  *chartInstance);
static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber);
static const mxArray *c2_sf_marshallOut(void *chartInstanceVoid, void *c2_inData);
static void c2_emlrt_marshallIn(SFc2_WBCLibraryInstanceStruct *chartInstance,
  const mxArray *c2_y, const char_T *c2_identifier, real_T c2_b_y[6]);
static void c2_b_emlrt_marshallIn(SFc2_WBCLibraryInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[6]);
static void c2_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_b_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static const mxArray *c2_c_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static const mxArray *c2_d_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static real_T c2_c_emlrt_marshallIn(SFc2_WBCLibraryInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static void c2_info_helper(c2_ResolvedFunctionInfo c2_info[166]);
static void c2_b_info_helper(c2_ResolvedFunctionInfo c2_info[166]);
static void c2_c_info_helper(c2_ResolvedFunctionInfo c2_info[166]);
static void c2_pinv(SFc2_WBCLibraryInstanceStruct *chartInstance, real_T c2_A[72],
                    real_T c2_X[72]);
static void c2_eml_scalar_eg(SFc2_WBCLibraryInstanceStruct *chartInstance);
static void c2_check_forloop_overflow_error(SFc2_WBCLibraryInstanceStruct
  *chartInstance);
static void c2_eml_error(SFc2_WBCLibraryInstanceStruct *chartInstance);
static void c2_eml_xgesvd(SFc2_WBCLibraryInstanceStruct *chartInstance, real_T
  c2_A[72], real_T c2_U[72], real_T c2_S[6], real_T c2_V[36]);
static real_T c2_abs(SFc2_WBCLibraryInstanceStruct *chartInstance, real_T c2_x);
static void c2_realmin(SFc2_WBCLibraryInstanceStruct *chartInstance);
static real_T c2_eml_xnrm2(SFc2_WBCLibraryInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_x[72], int32_T c2_ix0);
static void c2_b_check_forloop_overflow_error(SFc2_WBCLibraryInstanceStruct
  *chartInstance, boolean_T c2_overflow);
static real_T c2_eml_div(SFc2_WBCLibraryInstanceStruct *chartInstance, real_T
  c2_x, real_T c2_y);
static void c2_eml_xscal(SFc2_WBCLibraryInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_a, real_T c2_x[72], int32_T c2_ix0, real_T c2_b_x[72]);
static real_T c2_eml_xdotc(SFc2_WBCLibraryInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_x[72], int32_T c2_ix0, real_T c2_y[72], int32_T c2_iy0);
static void c2_eml_xaxpy(SFc2_WBCLibraryInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_a, int32_T c2_ix0, real_T c2_y[72], int32_T c2_iy0, real_T
  c2_b_y[72]);
static real_T c2_b_eml_xnrm2(SFc2_WBCLibraryInstanceStruct *chartInstance,
  int32_T c2_n, real_T c2_x[6], int32_T c2_ix0);
static void c2_b_eml_xscal(SFc2_WBCLibraryInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_a, real_T c2_x[6], int32_T c2_ix0, real_T c2_b_x[6]);
static void c2_b_eml_xaxpy(SFc2_WBCLibraryInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_a, real_T c2_x[72], int32_T c2_ix0, real_T c2_y[12], int32_T
  c2_iy0, real_T c2_b_y[12]);
static void c2_c_eml_xaxpy(SFc2_WBCLibraryInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_a, real_T c2_x[12], int32_T c2_ix0, real_T c2_y[72], int32_T
  c2_iy0, real_T c2_b_y[72]);
static real_T c2_b_eml_xdotc(SFc2_WBCLibraryInstanceStruct *chartInstance,
  int32_T c2_n, real_T c2_x[36], int32_T c2_ix0, real_T c2_y[36], int32_T c2_iy0);
static void c2_d_eml_xaxpy(SFc2_WBCLibraryInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_a, int32_T c2_ix0, real_T c2_y[36], int32_T c2_iy0, real_T
  c2_b_y[36]);
static void c2_b_eml_scalar_eg(SFc2_WBCLibraryInstanceStruct *chartInstance);
static void c2_c_eml_xscal(SFc2_WBCLibraryInstanceStruct *chartInstance, real_T
  c2_a, real_T c2_x[36], int32_T c2_ix0, real_T c2_b_x[36]);
static void c2_eps(SFc2_WBCLibraryInstanceStruct *chartInstance);
static void c2_c_eml_scalar_eg(SFc2_WBCLibraryInstanceStruct *chartInstance);
static void c2_b_eml_error(SFc2_WBCLibraryInstanceStruct *chartInstance);
static real_T c2_sqrt(SFc2_WBCLibraryInstanceStruct *chartInstance, real_T c2_x);
static void c2_c_eml_error(SFc2_WBCLibraryInstanceStruct *chartInstance);
static void c2_eml_xrotg(SFc2_WBCLibraryInstanceStruct *chartInstance, real_T
  c2_a, real_T c2_b, real_T *c2_b_a, real_T *c2_b_b, real_T *c2_c, real_T *c2_s);
static void c2_eml_xrot(SFc2_WBCLibraryInstanceStruct *chartInstance, real_T
  c2_x[36], int32_T c2_ix0, int32_T c2_iy0, real_T c2_c, real_T c2_s, real_T
  c2_b_x[36]);
static void c2_b_eml_xrot(SFc2_WBCLibraryInstanceStruct *chartInstance, real_T
  c2_x[72], int32_T c2_ix0, int32_T c2_iy0, real_T c2_c, real_T c2_s, real_T
  c2_b_x[72]);
static void c2_eml_xswap(SFc2_WBCLibraryInstanceStruct *chartInstance, real_T
  c2_x[36], int32_T c2_ix0, int32_T c2_iy0, real_T c2_b_x[36]);
static void c2_b_eml_xswap(SFc2_WBCLibraryInstanceStruct *chartInstance, real_T
  c2_x[72], int32_T c2_ix0, int32_T c2_iy0, real_T c2_b_x[72]);
static void c2_eml_xgemm(SFc2_WBCLibraryInstanceStruct *chartInstance, int32_T
  c2_k, real_T c2_A[36], real_T c2_B[72], real_T c2_C[72], real_T c2_b_C[72]);
static void c2_d_eml_scalar_eg(SFc2_WBCLibraryInstanceStruct *chartInstance);
static void c2_b_eml_xgemm(SFc2_WBCLibraryInstanceStruct *chartInstance, real_T
  c2_A[72], real_T c2_B[300], real_T c2_C[150], real_T c2_b_C[150]);
static void c2_e_eml_scalar_eg(SFc2_WBCLibraryInstanceStruct *chartInstance);
static const mxArray *c2_e_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static int32_T c2_d_emlrt_marshallIn(SFc2_WBCLibraryInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static uint8_T c2_e_emlrt_marshallIn(SFc2_WBCLibraryInstanceStruct
  *chartInstance, const mxArray *c2_b_is_active_c2_WBCLibrary, const char_T
  *c2_identifier);
static uint8_T c2_f_emlrt_marshallIn(SFc2_WBCLibraryInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_d_eml_xscal(SFc2_WBCLibraryInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_a, real_T c2_x[72], int32_T c2_ix0);
static void c2_e_eml_xaxpy(SFc2_WBCLibraryInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_a, int32_T c2_ix0, real_T c2_y[72], int32_T c2_iy0);
static void c2_e_eml_xscal(SFc2_WBCLibraryInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_a, real_T c2_x[6], int32_T c2_ix0);
static void c2_f_eml_xaxpy(SFc2_WBCLibraryInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_a, real_T c2_x[72], int32_T c2_ix0, real_T c2_y[12], int32_T
  c2_iy0);
static void c2_g_eml_xaxpy(SFc2_WBCLibraryInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_a, real_T c2_x[12], int32_T c2_ix0, real_T c2_y[72], int32_T
  c2_iy0);
static void c2_h_eml_xaxpy(SFc2_WBCLibraryInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_a, int32_T c2_ix0, real_T c2_y[36], int32_T c2_iy0);
static void c2_f_eml_xscal(SFc2_WBCLibraryInstanceStruct *chartInstance, real_T
  c2_a, real_T c2_x[36], int32_T c2_ix0);
static void c2_b_sqrt(SFc2_WBCLibraryInstanceStruct *chartInstance, real_T *c2_x);
static void c2_b_eml_xrotg(SFc2_WBCLibraryInstanceStruct *chartInstance, real_T *
  c2_a, real_T *c2_b, real_T *c2_c, real_T *c2_s);
static void c2_c_eml_xrot(SFc2_WBCLibraryInstanceStruct *chartInstance, real_T
  c2_x[36], int32_T c2_ix0, int32_T c2_iy0, real_T c2_c, real_T c2_s);
static void c2_d_eml_xrot(SFc2_WBCLibraryInstanceStruct *chartInstance, real_T
  c2_x[72], int32_T c2_ix0, int32_T c2_iy0, real_T c2_c, real_T c2_s);
static void c2_c_eml_xswap(SFc2_WBCLibraryInstanceStruct *chartInstance, real_T
  c2_x[36], int32_T c2_ix0, int32_T c2_iy0);
static void c2_d_eml_xswap(SFc2_WBCLibraryInstanceStruct *chartInstance, real_T
  c2_x[72], int32_T c2_ix0, int32_T c2_iy0);
static void c2_c_eml_xgemm(SFc2_WBCLibraryInstanceStruct *chartInstance, int32_T
  c2_k, real_T c2_A[36], real_T c2_B[72], real_T c2_C[72]);
static void c2_d_eml_xgemm(SFc2_WBCLibraryInstanceStruct *chartInstance, real_T
  c2_A[72], real_T c2_B[300], real_T c2_C[150]);
static void init_dsm_address_info(SFc2_WBCLibraryInstanceStruct *chartInstance);

/* Function Definitions */
static void initialize_c2_WBCLibrary(SFc2_WBCLibraryInstanceStruct
  *chartInstance)
{
  chartInstance->c2_sfEvent = CALL_EVENT;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c2_is_active_c2_WBCLibrary = 0U;
}

static void initialize_params_c2_WBCLibrary(SFc2_WBCLibraryInstanceStruct
  *chartInstance)
{
}

static void enable_c2_WBCLibrary(SFc2_WBCLibraryInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c2_WBCLibrary(SFc2_WBCLibraryInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c2_update_debugger_state_c2_WBCLibrary(SFc2_WBCLibraryInstanceStruct
  *chartInstance)
{
}

static const mxArray *get_sim_state_c2_WBCLibrary(SFc2_WBCLibraryInstanceStruct *
  chartInstance)
{
  const mxArray *c2_st;
  const mxArray *c2_y = NULL;
  int32_T c2_i0;
  real_T c2_u[6];
  const mxArray *c2_b_y = NULL;
  uint8_T c2_hoistedGlobal;
  uint8_T c2_b_u;
  const mxArray *c2_c_y = NULL;
  real_T (*c2_d_y)[6];
  c2_d_y = (real_T (*)[6])ssGetOutputPortSignal(chartInstance->S, 1);
  c2_st = NULL;
  c2_st = NULL;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_createcellarray(2), FALSE);
  for (c2_i0 = 0; c2_i0 < 6; c2_i0++) {
    c2_u[c2_i0] = (*c2_d_y)[c2_i0];
  }

  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 6), FALSE);
  sf_mex_setcell(c2_y, 0, c2_b_y);
  c2_hoistedGlobal = chartInstance->c2_is_active_c2_WBCLibrary;
  c2_b_u = c2_hoistedGlobal;
  c2_c_y = NULL;
  sf_mex_assign(&c2_c_y, sf_mex_create("y", &c2_b_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c2_y, 1, c2_c_y);
  sf_mex_assign(&c2_st, c2_y, FALSE);
  return c2_st;
}

static void set_sim_state_c2_WBCLibrary(SFc2_WBCLibraryInstanceStruct
  *chartInstance, const mxArray *c2_st)
{
  const mxArray *c2_u;
  real_T c2_dv0[6];
  int32_T c2_i1;
  real_T (*c2_y)[6];
  c2_y = (real_T (*)[6])ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c2_doneDoubleBufferReInit = TRUE;
  c2_u = sf_mex_dup(c2_st);
  c2_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 0)), "y",
                      c2_dv0);
  for (c2_i1 = 0; c2_i1 < 6; c2_i1++) {
    (*c2_y)[c2_i1] = c2_dv0[c2_i1];
  }

  chartInstance->c2_is_active_c2_WBCLibrary = c2_e_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 1)),
     "is_active_c2_WBCLibrary");
  sf_mex_destroy(&c2_u);
  c2_update_debugger_state_c2_WBCLibrary(chartInstance);
  sf_mex_destroy(&c2_st);
}

static void finalize_c2_WBCLibrary(SFc2_WBCLibraryInstanceStruct *chartInstance)
{
}

static void sf_c2_WBCLibrary(SFc2_WBCLibraryInstanceStruct *chartInstance)
{
  int32_T c2_i2;
  int32_T c2_i3;
  int32_T c2_i4;
  real_T (*c2_qD)[25];
  real_T (*c2_y)[6];
  real_T (*c2_JcIn)[372];
  c2_qD = (real_T (*)[25])ssGetInputPortSignal(chartInstance->S, 1);
  c2_y = (real_T (*)[6])ssGetOutputPortSignal(chartInstance->S, 1);
  c2_JcIn = (real_T (*)[372])ssGetInputPortSignal(chartInstance->S, 0);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 0U, chartInstance->c2_sfEvent);
  for (c2_i2 = 0; c2_i2 < 372; c2_i2++) {
    _SFD_DATA_RANGE_CHECK((*c2_JcIn)[c2_i2], 0U);
  }

  for (c2_i3 = 0; c2_i3 < 6; c2_i3++) {
    _SFD_DATA_RANGE_CHECK((*c2_y)[c2_i3], 1U);
  }

  for (c2_i4 = 0; c2_i4 < 25; c2_i4++) {
    _SFD_DATA_RANGE_CHECK((*c2_qD)[c2_i4], 2U);
  }

  chartInstance->c2_sfEvent = CALL_EVENT;
  c2_chartstep_c2_WBCLibrary(chartInstance);
  sf_debug_check_for_state_inconsistency(_WBCLibraryMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void c2_chartstep_c2_WBCLibrary(SFc2_WBCLibraryInstanceStruct
  *chartInstance)
{
  int32_T c2_i5;
  real_T c2_JcIn[372];
  int32_T c2_i6;
  real_T c2_qD[25];
  uint32_T c2_debug_family_var_map[5];
  real_T c2_nargin = 2.0;
  real_T c2_nargout = 1.0;
  real_T c2_y[6];
  int32_T c2_i7;
  int32_T c2_i8;
  int32_T c2_i9;
  real_T c2_b_JcIn[72];
  real_T c2_a[72];
  int32_T c2_i10;
  int32_T c2_i11;
  int32_T c2_i12;
  int32_T c2_i13;
  real_T c2_b[300];
  int32_T c2_i14;
  real_T c2_b_y[150];
  int32_T c2_i15;
  real_T c2_b_a[72];
  int32_T c2_i16;
  real_T c2_b_b[300];
  int32_T c2_i17;
  real_T c2_c_b[25];
  int32_T c2_i18;
  int32_T c2_i19;
  int32_T c2_i20;
  real_T c2_C[6];
  int32_T c2_i21;
  int32_T c2_i22;
  int32_T c2_i23;
  int32_T c2_i24;
  int32_T c2_i25;
  int32_T c2_i26;
  int32_T c2_i27;
  real_T (*c2_c_y)[6];
  real_T (*c2_b_qD)[25];
  real_T (*c2_c_JcIn)[372];
  c2_b_qD = (real_T (*)[25])ssGetInputPortSignal(chartInstance->S, 1);
  c2_c_y = (real_T (*)[6])ssGetOutputPortSignal(chartInstance->S, 1);
  c2_c_JcIn = (real_T (*)[372])ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 0U, chartInstance->c2_sfEvent);
  for (c2_i5 = 0; c2_i5 < 372; c2_i5++) {
    c2_JcIn[c2_i5] = (*c2_c_JcIn)[c2_i5];
  }

  for (c2_i6 = 0; c2_i6 < 25; c2_i6++) {
    c2_qD[c2_i6] = (*c2_b_qD)[c2_i6];
  }

  sf_debug_symbol_scope_push_eml(0U, 5U, 5U, c2_debug_family_names,
    c2_debug_family_var_map);
  sf_debug_symbol_scope_add_eml_importable(&c2_nargin, 0U, c2_d_sf_marshallOut,
    c2_b_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c2_nargout, 1U, c2_d_sf_marshallOut,
    c2_b_sf_marshallIn);
  sf_debug_symbol_scope_add_eml(c2_JcIn, 2U, c2_c_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(c2_qD, 3U, c2_b_sf_marshallOut);
  sf_debug_symbol_scope_add_eml_importable(c2_y, 4U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 3);
  c2_i7 = 0;
  for (c2_i8 = 0; c2_i8 < 6; c2_i8++) {
    for (c2_i9 = 0; c2_i9 < 12; c2_i9++) {
      c2_b_JcIn[c2_i9 + c2_i7] = c2_JcIn[c2_i9 + c2_i7];
    }

    c2_i7 += 12;
  }

  c2_pinv(chartInstance, c2_b_JcIn, c2_a);
  for (c2_i10 = 0; c2_i10 < 72; c2_i10++) {
    c2_a[c2_i10] = -c2_a[c2_i10];
  }

  c2_i11 = 0;
  for (c2_i12 = 0; c2_i12 < 25; c2_i12++) {
    for (c2_i13 = 0; c2_i13 < 12; c2_i13++) {
      c2_b[c2_i13 + c2_i11] = c2_JcIn[(c2_i13 + c2_i11) + 72];
    }

    c2_i11 += 12;
  }

  c2_d_eml_scalar_eg(chartInstance);
  c2_d_eml_scalar_eg(chartInstance);
  for (c2_i14 = 0; c2_i14 < 150; c2_i14++) {
    c2_b_y[c2_i14] = 0.0;
  }

  for (c2_i15 = 0; c2_i15 < 72; c2_i15++) {
    c2_b_a[c2_i15] = c2_a[c2_i15];
  }

  for (c2_i16 = 0; c2_i16 < 300; c2_i16++) {
    c2_b_b[c2_i16] = c2_b[c2_i16];
  }

  c2_d_eml_xgemm(chartInstance, c2_b_a, c2_b_b, c2_b_y);
  for (c2_i17 = 0; c2_i17 < 25; c2_i17++) {
    c2_c_b[c2_i17] = c2_qD[c2_i17];
  }

  c2_e_eml_scalar_eg(chartInstance);
  c2_e_eml_scalar_eg(chartInstance);
  for (c2_i18 = 0; c2_i18 < 6; c2_i18++) {
    c2_y[c2_i18] = 0.0;
  }

  for (c2_i19 = 0; c2_i19 < 6; c2_i19++) {
    c2_y[c2_i19] = 0.0;
  }

  for (c2_i20 = 0; c2_i20 < 6; c2_i20++) {
    c2_C[c2_i20] = c2_y[c2_i20];
  }

  for (c2_i21 = 0; c2_i21 < 6; c2_i21++) {
    c2_y[c2_i21] = c2_C[c2_i21];
  }

  for (c2_i22 = 0; c2_i22 < 6; c2_i22++) {
    c2_C[c2_i22] = c2_y[c2_i22];
  }

  for (c2_i23 = 0; c2_i23 < 6; c2_i23++) {
    c2_y[c2_i23] = c2_C[c2_i23];
  }

  for (c2_i24 = 0; c2_i24 < 6; c2_i24++) {
    c2_y[c2_i24] = 0.0;
    c2_i25 = 0;
    for (c2_i26 = 0; c2_i26 < 25; c2_i26++) {
      c2_y[c2_i24] += c2_b_y[c2_i25 + c2_i24] * c2_c_b[c2_i26];
      c2_i25 += 6;
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, -3);
  sf_debug_symbol_scope_pop();
  for (c2_i27 = 0; c2_i27 < 6; c2_i27++) {
    (*c2_c_y)[c2_i27] = c2_y[c2_i27];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 0U, chartInstance->c2_sfEvent);
}

static void initSimStructsc2_WBCLibrary(SFc2_WBCLibraryInstanceStruct
  *chartInstance)
{
}

static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber)
{
}

static const mxArray *c2_sf_marshallOut(void *chartInstanceVoid, void *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i28;
  real_T c2_b_inData[6];
  int32_T c2_i29;
  real_T c2_u[6];
  const mxArray *c2_y = NULL;
  SFc2_WBCLibraryInstanceStruct *chartInstance;
  chartInstance = (SFc2_WBCLibraryInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i28 = 0; c2_i28 < 6; c2_i28++) {
    c2_b_inData[c2_i28] = (*(real_T (*)[6])c2_inData)[c2_i28];
  }

  for (c2_i29 = 0; c2_i29 < 6; c2_i29++) {
    c2_u[c2_i29] = c2_b_inData[c2_i29];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 6), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_emlrt_marshallIn(SFc2_WBCLibraryInstanceStruct *chartInstance,
  const mxArray *c2_y, const char_T *c2_identifier, real_T c2_b_y[6])
{
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_y), &c2_thisId, c2_b_y);
  sf_mex_destroy(&c2_y);
}

static void c2_b_emlrt_marshallIn(SFc2_WBCLibraryInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[6])
{
  real_T c2_dv1[6];
  int32_T c2_i30;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv1, 1, 0, 0U, 1, 0U, 1, 6);
  for (c2_i30 = 0; c2_i30 < 6; c2_i30++) {
    c2_y[c2_i30] = c2_dv1[c2_i30];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_y;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_b_y[6];
  int32_T c2_i31;
  SFc2_WBCLibraryInstanceStruct *chartInstance;
  chartInstance = (SFc2_WBCLibraryInstanceStruct *)chartInstanceVoid;
  c2_y = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_y), &c2_thisId, c2_b_y);
  sf_mex_destroy(&c2_y);
  for (c2_i31 = 0; c2_i31 < 6; c2_i31++) {
    (*(real_T (*)[6])c2_outData)[c2_i31] = c2_b_y[c2_i31];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_b_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i32;
  real_T c2_b_inData[25];
  int32_T c2_i33;
  real_T c2_u[25];
  const mxArray *c2_y = NULL;
  SFc2_WBCLibraryInstanceStruct *chartInstance;
  chartInstance = (SFc2_WBCLibraryInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i32 = 0; c2_i32 < 25; c2_i32++) {
    c2_b_inData[c2_i32] = (*(real_T (*)[25])c2_inData)[c2_i32];
  }

  for (c2_i33 = 0; c2_i33 < 25; c2_i33++) {
    c2_u[c2_i33] = c2_b_inData[c2_i33];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 25), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static const mxArray *c2_c_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i34;
  int32_T c2_i35;
  int32_T c2_i36;
  real_T c2_b_inData[372];
  int32_T c2_i37;
  int32_T c2_i38;
  int32_T c2_i39;
  real_T c2_u[372];
  const mxArray *c2_y = NULL;
  SFc2_WBCLibraryInstanceStruct *chartInstance;
  chartInstance = (SFc2_WBCLibraryInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i34 = 0;
  for (c2_i35 = 0; c2_i35 < 31; c2_i35++) {
    for (c2_i36 = 0; c2_i36 < 12; c2_i36++) {
      c2_b_inData[c2_i36 + c2_i34] = (*(real_T (*)[372])c2_inData)[c2_i36 +
        c2_i34];
    }

    c2_i34 += 12;
  }

  c2_i37 = 0;
  for (c2_i38 = 0; c2_i38 < 31; c2_i38++) {
    for (c2_i39 = 0; c2_i39 < 12; c2_i39++) {
      c2_u[c2_i39 + c2_i37] = c2_b_inData[c2_i39 + c2_i37];
    }

    c2_i37 += 12;
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 12, 31), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static const mxArray *c2_d_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  real_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_WBCLibraryInstanceStruct *chartInstance;
  chartInstance = (SFc2_WBCLibraryInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(real_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static real_T c2_c_emlrt_marshallIn(SFc2_WBCLibraryInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  real_T c2_y;
  real_T c2_d0;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_d0, 1, 0, 0U, 0, 0U, 0);
  c2_y = c2_d0;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_nargout;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y;
  SFc2_WBCLibraryInstanceStruct *chartInstance;
  chartInstance = (SFc2_WBCLibraryInstanceStruct *)chartInstanceVoid;
  c2_nargout = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_nargout), &c2_thisId);
  sf_mex_destroy(&c2_nargout);
  *(real_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

const mxArray *sf_c2_WBCLibrary_get_eml_resolved_functions_info(void)
{
  const mxArray *c2_nameCaptureInfo;
  c2_ResolvedFunctionInfo c2_info[166];
  const mxArray *c2_m0 = NULL;
  int32_T c2_i40;
  c2_ResolvedFunctionInfo *c2_r0;
  c2_nameCaptureInfo = NULL;
  c2_nameCaptureInfo = NULL;
  c2_info_helper(c2_info);
  c2_b_info_helper(c2_info);
  c2_c_info_helper(c2_info);
  sf_mex_assign(&c2_m0, sf_mex_createstruct("nameCaptureInfo", 1, 166), FALSE);
  for (c2_i40 = 0; c2_i40 < 166; c2_i40++) {
    c2_r0 = &c2_info[c2_i40];
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c2_r0->context)), "context", "nameCaptureInfo",
                    c2_i40);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->name, 15, 0U,
      0U, 0U, 2, 1, strlen(c2_r0->name)), "name", "nameCaptureInfo", c2_i40);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->dominantType,
      15, 0U, 0U, 0U, 2, 1, strlen(c2_r0->dominantType)), "dominantType",
                    "nameCaptureInfo", c2_i40);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->resolved, 15,
      0U, 0U, 0U, 2, 1, strlen(c2_r0->resolved)), "resolved", "nameCaptureInfo",
                    c2_i40);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", &c2_r0->fileTimeLo,
      7, 0U, 0U, 0U, 0), "fileTimeLo", "nameCaptureInfo", c2_i40);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", &c2_r0->fileTimeHi,
      7, 0U, 0U, 0U, 0), "fileTimeHi", "nameCaptureInfo", c2_i40);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", &c2_r0->mFileTimeLo,
      7, 0U, 0U, 0U, 0), "mFileTimeLo", "nameCaptureInfo", c2_i40);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", &c2_r0->mFileTimeHi,
      7, 0U, 0U, 0U, 0), "mFileTimeHi", "nameCaptureInfo", c2_i40);
  }

  sf_mex_assign(&c2_nameCaptureInfo, c2_m0, FALSE);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c2_nameCaptureInfo);
  return c2_nameCaptureInfo;
}

static void c2_info_helper(c2_ResolvedFunctionInfo c2_info[166])
{
  c2_info[0].context = "";
  c2_info[0].name = "pinv";
  c2_info[0].dominantType = "double";
  c2_info[0].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m";
  c2_info[0].fileTimeLo = 1286818828U;
  c2_info[0].fileTimeHi = 0U;
  c2_info[0].mFileTimeLo = 0U;
  c2_info[0].mFileTimeHi = 0U;
  c2_info[1].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv";
  c2_info[1].name = "eml_index_class";
  c2_info[1].dominantType = "";
  c2_info[1].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[1].fileTimeLo = 1323170578U;
  c2_info[1].fileTimeHi = 0U;
  c2_info[1].mFileTimeLo = 0U;
  c2_info[1].mFileTimeHi = 0U;
  c2_info[2].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv";
  c2_info[2].name = "eml_scalar_eg";
  c2_info[2].dominantType = "double";
  c2_info[2].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[2].fileTimeLo = 1286818796U;
  c2_info[2].fileTimeHi = 0U;
  c2_info[2].mFileTimeLo = 0U;
  c2_info[2].mFileTimeHi = 0U;
  c2_info[3].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv";
  c2_info[3].name = "svd";
  c2_info[3].dominantType = "double";
  c2_info[3].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/svd.m";
  c2_info[3].fileTimeLo = 1286818832U;
  c2_info[3].fileTimeHi = 0U;
  c2_info[3].mFileTimeLo = 0U;
  c2_info[3].mFileTimeHi = 0U;
  c2_info[4].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/svd.m";
  c2_info[4].name = "eml_index_class";
  c2_info[4].dominantType = "";
  c2_info[4].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[4].fileTimeLo = 1323170578U;
  c2_info[4].fileTimeHi = 0U;
  c2_info[4].mFileTimeLo = 0U;
  c2_info[4].mFileTimeHi = 0U;
  c2_info[5].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/svd.m";
  c2_info[5].name = "eml_int_forloop_overflow_check";
  c2_info[5].dominantType = "";
  c2_info[5].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[5].fileTimeLo = 1332168672U;
  c2_info[5].fileTimeHi = 0U;
  c2_info[5].mFileTimeLo = 0U;
  c2_info[5].mFileTimeHi = 0U;
  c2_info[6].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper";
  c2_info[6].name = "intmax";
  c2_info[6].dominantType = "char";
  c2_info[6].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c2_info[6].fileTimeLo = 1311255316U;
  c2_info[6].fileTimeHi = 0U;
  c2_info[6].mFileTimeLo = 0U;
  c2_info[6].mFileTimeHi = 0U;
  c2_info[7].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/svd.m";
  c2_info[7].name = "isfinite";
  c2_info[7].dominantType = "double";
  c2_info[7].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isfinite.m";
  c2_info[7].fileTimeLo = 1286818758U;
  c2_info[7].fileTimeHi = 0U;
  c2_info[7].mFileTimeLo = 0U;
  c2_info[7].mFileTimeHi = 0U;
  c2_info[8].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isfinite.m";
  c2_info[8].name = "isinf";
  c2_info[8].dominantType = "double";
  c2_info[8].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isinf.m";
  c2_info[8].fileTimeLo = 1286818760U;
  c2_info[8].fileTimeHi = 0U;
  c2_info[8].mFileTimeLo = 0U;
  c2_info[8].mFileTimeHi = 0U;
  c2_info[9].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isfinite.m";
  c2_info[9].name = "isnan";
  c2_info[9].dominantType = "double";
  c2_info[9].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m";
  c2_info[9].fileTimeLo = 1286818760U;
  c2_info[9].fileTimeHi = 0U;
  c2_info[9].mFileTimeLo = 0U;
  c2_info[9].mFileTimeHi = 0U;
  c2_info[10].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/svd.m";
  c2_info[10].name = "eml_error";
  c2_info[10].dominantType = "char";
  c2_info[10].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m";
  c2_info[10].fileTimeLo = 1305318000U;
  c2_info[10].fileTimeHi = 0U;
  c2_info[10].mFileTimeLo = 0U;
  c2_info[10].mFileTimeHi = 0U;
  c2_info[11].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/svd.m";
  c2_info[11].name = "eml_xgesvd";
  c2_info[11].dominantType = "double";
  c2_info[11].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/eml_xgesvd.m";
  c2_info[11].fileTimeLo = 1286818806U;
  c2_info[11].fileTimeHi = 0U;
  c2_info[11].mFileTimeLo = 0U;
  c2_info[11].mFileTimeHi = 0U;
  c2_info[12].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/eml_xgesvd.m";
  c2_info[12].name = "eml_lapack_xgesvd";
  c2_info[12].dominantType = "double";
  c2_info[12].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/internal/eml_lapack_xgesvd.m";
  c2_info[12].fileTimeLo = 1286818810U;
  c2_info[12].fileTimeHi = 0U;
  c2_info[12].mFileTimeLo = 0U;
  c2_info[12].mFileTimeHi = 0U;
  c2_info[13].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/internal/eml_lapack_xgesvd.m";
  c2_info[13].name = "eml_matlab_zsvdc";
  c2_info[13].dominantType = "double";
  c2_info[13].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c2_info[13].fileTimeLo = 1295284866U;
  c2_info[13].fileTimeHi = 0U;
  c2_info[13].mFileTimeLo = 0U;
  c2_info[13].mFileTimeHi = 0U;
  c2_info[14].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c2_info[14].name = "eml_index_class";
  c2_info[14].dominantType = "";
  c2_info[14].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[14].fileTimeLo = 1323170578U;
  c2_info[14].fileTimeHi = 0U;
  c2_info[14].mFileTimeLo = 0U;
  c2_info[14].mFileTimeHi = 0U;
  c2_info[15].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c2_info[15].name = "eml_scalar_eg";
  c2_info[15].dominantType = "double";
  c2_info[15].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[15].fileTimeLo = 1286818796U;
  c2_info[15].fileTimeHi = 0U;
  c2_info[15].mFileTimeLo = 0U;
  c2_info[15].mFileTimeHi = 0U;
  c2_info[16].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c2_info[16].name = "eml_index_plus";
  c2_info[16].dominantType = "double";
  c2_info[16].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[16].fileTimeLo = 1286818778U;
  c2_info[16].fileTimeHi = 0U;
  c2_info[16].mFileTimeLo = 0U;
  c2_info[16].mFileTimeHi = 0U;
  c2_info[17].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[17].name = "eml_index_class";
  c2_info[17].dominantType = "";
  c2_info[17].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[17].fileTimeLo = 1323170578U;
  c2_info[17].fileTimeHi = 0U;
  c2_info[17].mFileTimeLo = 0U;
  c2_info[17].mFileTimeHi = 0U;
  c2_info[18].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c2_info[18].name = "min";
  c2_info[18].dominantType = "coder.internal.indexInt";
  c2_info[18].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c2_info[18].fileTimeLo = 1311255318U;
  c2_info[18].fileTimeHi = 0U;
  c2_info[18].mFileTimeLo = 0U;
  c2_info[18].mFileTimeHi = 0U;
  c2_info[19].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c2_info[19].name = "eml_min_or_max";
  c2_info[19].dominantType = "char";
  c2_info[19].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m";
  c2_info[19].fileTimeLo = 1334071490U;
  c2_info[19].fileTimeHi = 0U;
  c2_info[19].mFileTimeLo = 0U;
  c2_info[19].mFileTimeHi = 0U;
  c2_info[20].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c2_info[20].name = "eml_scalar_eg";
  c2_info[20].dominantType = "coder.internal.indexInt";
  c2_info[20].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[20].fileTimeLo = 1286818796U;
  c2_info[20].fileTimeHi = 0U;
  c2_info[20].mFileTimeLo = 0U;
  c2_info[20].mFileTimeHi = 0U;
  c2_info[21].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c2_info[21].name = "eml_scalexp_alloc";
  c2_info[21].dominantType = "coder.internal.indexInt";
  c2_info[21].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c2_info[21].fileTimeLo = 1330608434U;
  c2_info[21].fileTimeHi = 0U;
  c2_info[21].mFileTimeLo = 0U;
  c2_info[21].mFileTimeHi = 0U;
  c2_info[22].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c2_info[22].name = "eml_index_class";
  c2_info[22].dominantType = "";
  c2_info[22].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[22].fileTimeLo = 1323170578U;
  c2_info[22].fileTimeHi = 0U;
  c2_info[22].mFileTimeLo = 0U;
  c2_info[22].mFileTimeHi = 0U;
  c2_info[23].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum";
  c2_info[23].name = "eml_scalar_eg";
  c2_info[23].dominantType = "coder.internal.indexInt";
  c2_info[23].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[23].fileTimeLo = 1286818796U;
  c2_info[23].fileTimeHi = 0U;
  c2_info[23].mFileTimeLo = 0U;
  c2_info[23].mFileTimeHi = 0U;
  c2_info[24].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c2_info[24].name = "max";
  c2_info[24].dominantType = "double";
  c2_info[24].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/max.m";
  c2_info[24].fileTimeLo = 1311255316U;
  c2_info[24].fileTimeHi = 0U;
  c2_info[24].mFileTimeLo = 0U;
  c2_info[24].mFileTimeHi = 0U;
  c2_info[25].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/max.m";
  c2_info[25].name = "eml_min_or_max";
  c2_info[25].dominantType = "char";
  c2_info[25].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m";
  c2_info[25].fileTimeLo = 1334071490U;
  c2_info[25].fileTimeHi = 0U;
  c2_info[25].mFileTimeLo = 0U;
  c2_info[25].mFileTimeHi = 0U;
  c2_info[26].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c2_info[26].name = "eml_scalar_eg";
  c2_info[26].dominantType = "double";
  c2_info[26].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[26].fileTimeLo = 1286818796U;
  c2_info[26].fileTimeHi = 0U;
  c2_info[26].mFileTimeLo = 0U;
  c2_info[26].mFileTimeHi = 0U;
  c2_info[27].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c2_info[27].name = "eml_scalexp_alloc";
  c2_info[27].dominantType = "double";
  c2_info[27].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c2_info[27].fileTimeLo = 1330608434U;
  c2_info[27].fileTimeHi = 0U;
  c2_info[27].mFileTimeLo = 0U;
  c2_info[27].mFileTimeHi = 0U;
  c2_info[28].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum";
  c2_info[28].name = "eml_scalar_eg";
  c2_info[28].dominantType = "double";
  c2_info[28].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[28].fileTimeLo = 1286818796U;
  c2_info[28].fileTimeHi = 0U;
  c2_info[28].mFileTimeLo = 0U;
  c2_info[28].mFileTimeHi = 0U;
  c2_info[29].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum";
  c2_info[29].name = "eml_relop";
  c2_info[29].dominantType = "function_handle";
  c2_info[29].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_relop.m";
  c2_info[29].fileTimeLo = 1326727998U;
  c2_info[29].fileTimeHi = 0U;
  c2_info[29].mFileTimeLo = 0U;
  c2_info[29].mFileTimeHi = 0U;
  c2_info[30].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_relop.m";
  c2_info[30].name = "coder.internal.indexIntRelop";
  c2_info[30].dominantType = "";
  c2_info[30].resolved =
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexIntRelop.m";
  c2_info[30].fileTimeLo = 1326728322U;
  c2_info[30].fileTimeHi = 0U;
  c2_info[30].mFileTimeLo = 0U;
  c2_info[30].mFileTimeHi = 0U;
  c2_info[31].context =
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexIntRelop.m!float_class_contains_indexIntClass";
  c2_info[31].name = "eml_float_model";
  c2_info[31].dominantType = "char";
  c2_info[31].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m";
  c2_info[31].fileTimeLo = 1326727996U;
  c2_info[31].fileTimeHi = 0U;
  c2_info[31].mFileTimeLo = 0U;
  c2_info[31].mFileTimeHi = 0U;
  c2_info[32].context =
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexIntRelop.m!is_signed_indexIntClass";
  c2_info[32].name = "intmin";
  c2_info[32].dominantType = "char";
  c2_info[32].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m";
  c2_info[32].fileTimeLo = 1311255318U;
  c2_info[32].fileTimeHi = 0U;
  c2_info[32].mFileTimeLo = 0U;
  c2_info[32].mFileTimeHi = 0U;
  c2_info[33].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum";
  c2_info[33].name = "isnan";
  c2_info[33].dominantType = "coder.internal.indexInt";
  c2_info[33].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m";
  c2_info[33].fileTimeLo = 1286818760U;
  c2_info[33].fileTimeHi = 0U;
  c2_info[33].mFileTimeLo = 0U;
  c2_info[33].mFileTimeHi = 0U;
  c2_info[34].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c2_info[34].name = "eml_index_minus";
  c2_info[34].dominantType = "double";
  c2_info[34].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c2_info[34].fileTimeLo = 1286818778U;
  c2_info[34].fileTimeHi = 0U;
  c2_info[34].mFileTimeLo = 0U;
  c2_info[34].mFileTimeHi = 0U;
  c2_info[35].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c2_info[35].name = "eml_index_class";
  c2_info[35].dominantType = "";
  c2_info[35].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[35].fileTimeLo = 1323170578U;
  c2_info[35].fileTimeHi = 0U;
  c2_info[35].mFileTimeLo = 0U;
  c2_info[35].mFileTimeHi = 0U;
  c2_info[36].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c2_info[36].name = "max";
  c2_info[36].dominantType = "coder.internal.indexInt";
  c2_info[36].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/max.m";
  c2_info[36].fileTimeLo = 1311255316U;
  c2_info[36].fileTimeHi = 0U;
  c2_info[36].mFileTimeLo = 0U;
  c2_info[36].mFileTimeHi = 0U;
  c2_info[37].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c2_info[37].name = "eml_int_forloop_overflow_check";
  c2_info[37].dominantType = "";
  c2_info[37].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[37].fileTimeLo = 1332168672U;
  c2_info[37].fileTimeHi = 0U;
  c2_info[37].mFileTimeLo = 0U;
  c2_info[37].mFileTimeHi = 0U;
  c2_info[38].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c2_info[38].name = "eml_index_times";
  c2_info[38].dominantType = "coder.internal.indexInt";
  c2_info[38].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c2_info[38].fileTimeLo = 1286818780U;
  c2_info[38].fileTimeHi = 0U;
  c2_info[38].mFileTimeLo = 0U;
  c2_info[38].mFileTimeHi = 0U;
  c2_info[39].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
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
  c2_info[40].name = "eml_index_plus";
  c2_info[40].dominantType = "coder.internal.indexInt";
  c2_info[40].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[40].fileTimeLo = 1286818778U;
  c2_info[40].fileTimeHi = 0U;
  c2_info[40].mFileTimeLo = 0U;
  c2_info[40].mFileTimeHi = 0U;
  c2_info[41].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c2_info[41].name = "eml_index_minus";
  c2_info[41].dominantType = "coder.internal.indexInt";
  c2_info[41].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c2_info[41].fileTimeLo = 1286818778U;
  c2_info[41].fileTimeHi = 0U;
  c2_info[41].mFileTimeLo = 0U;
  c2_info[41].mFileTimeHi = 0U;
  c2_info[42].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c2_info[42].name = "eml_xnrm2";
  c2_info[42].dominantType = "double";
  c2_info[42].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xnrm2.m";
  c2_info[42].fileTimeLo = 1299076776U;
  c2_info[42].fileTimeHi = 0U;
  c2_info[42].mFileTimeLo = 0U;
  c2_info[42].mFileTimeHi = 0U;
  c2_info[43].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xnrm2.m";
  c2_info[43].name = "eml_blas_inline";
  c2_info[43].dominantType = "";
  c2_info[43].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c2_info[43].fileTimeLo = 1299076768U;
  c2_info[43].fileTimeHi = 0U;
  c2_info[43].mFileTimeLo = 0U;
  c2_info[43].mFileTimeHi = 0U;
  c2_info[44].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xnrm2.m!below_threshold";
  c2_info[44].name = "length";
  c2_info[44].dominantType = "double";
  c2_info[44].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m";
  c2_info[44].fileTimeLo = 1303146206U;
  c2_info[44].fileTimeHi = 0U;
  c2_info[44].mFileTimeLo = 0U;
  c2_info[44].mFileTimeHi = 0U;
  c2_info[45].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m!intlength";
  c2_info[45].name = "eml_index_class";
  c2_info[45].dominantType = "";
  c2_info[45].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[45].fileTimeLo = 1323170578U;
  c2_info[45].fileTimeHi = 0U;
  c2_info[45].mFileTimeLo = 0U;
  c2_info[45].mFileTimeHi = 0U;
  c2_info[46].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xnrm2.m";
  c2_info[46].name = "eml_index_class";
  c2_info[46].dominantType = "";
  c2_info[46].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[46].fileTimeLo = 1323170578U;
  c2_info[46].fileTimeHi = 0U;
  c2_info[46].mFileTimeLo = 0U;
  c2_info[46].mFileTimeHi = 0U;
  c2_info[47].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xnrm2.m";
  c2_info[47].name = "eml_refblas_xnrm2";
  c2_info[47].dominantType = "double";
  c2_info[47].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c2_info[47].fileTimeLo = 1299076784U;
  c2_info[47].fileTimeHi = 0U;
  c2_info[47].mFileTimeLo = 0U;
  c2_info[47].mFileTimeHi = 0U;
  c2_info[48].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c2_info[48].name = "abs";
  c2_info[48].dominantType = "double";
  c2_info[48].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c2_info[48].fileTimeLo = 1286818694U;
  c2_info[48].fileTimeHi = 0U;
  c2_info[48].mFileTimeLo = 0U;
  c2_info[48].mFileTimeHi = 0U;
  c2_info[49].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c2_info[49].name = "eml_scalar_abs";
  c2_info[49].dominantType = "double";
  c2_info[49].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  c2_info[49].fileTimeLo = 1286818712U;
  c2_info[49].fileTimeHi = 0U;
  c2_info[49].mFileTimeLo = 0U;
  c2_info[49].mFileTimeHi = 0U;
  c2_info[50].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c2_info[50].name = "realmin";
  c2_info[50].dominantType = "char";
  c2_info[50].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m";
  c2_info[50].fileTimeLo = 1307651242U;
  c2_info[50].fileTimeHi = 0U;
  c2_info[50].mFileTimeLo = 0U;
  c2_info[50].mFileTimeHi = 0U;
  c2_info[51].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m";
  c2_info[51].name = "eml_realmin";
  c2_info[51].dominantType = "char";
  c2_info[51].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_realmin.m";
  c2_info[51].fileTimeLo = 1307651244U;
  c2_info[51].fileTimeHi = 0U;
  c2_info[51].mFileTimeLo = 0U;
  c2_info[51].mFileTimeHi = 0U;
  c2_info[52].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_realmin.m";
  c2_info[52].name = "eml_float_model";
  c2_info[52].dominantType = "char";
  c2_info[52].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m";
  c2_info[52].fileTimeLo = 1326727996U;
  c2_info[52].fileTimeHi = 0U;
  c2_info[52].mFileTimeLo = 0U;
  c2_info[52].mFileTimeHi = 0U;
  c2_info[53].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c2_info[53].name = "eml_index_class";
  c2_info[53].dominantType = "";
  c2_info[53].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[53].fileTimeLo = 1323170578U;
  c2_info[53].fileTimeHi = 0U;
  c2_info[53].mFileTimeLo = 0U;
  c2_info[53].mFileTimeHi = 0U;
  c2_info[54].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c2_info[54].name = "eml_index_minus";
  c2_info[54].dominantType = "double";
  c2_info[54].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c2_info[54].fileTimeLo = 1286818778U;
  c2_info[54].fileTimeHi = 0U;
  c2_info[54].mFileTimeLo = 0U;
  c2_info[54].mFileTimeHi = 0U;
  c2_info[55].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c2_info[55].name = "eml_index_times";
  c2_info[55].dominantType = "coder.internal.indexInt";
  c2_info[55].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c2_info[55].fileTimeLo = 1286818780U;
  c2_info[55].fileTimeHi = 0U;
  c2_info[55].mFileTimeLo = 0U;
  c2_info[55].mFileTimeHi = 0U;
  c2_info[56].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c2_info[56].name = "eml_index_plus";
  c2_info[56].dominantType = "coder.internal.indexInt";
  c2_info[56].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[56].fileTimeLo = 1286818778U;
  c2_info[56].fileTimeHi = 0U;
  c2_info[56].mFileTimeLo = 0U;
  c2_info[56].mFileTimeHi = 0U;
  c2_info[57].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
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
  c2_info[58].name = "eml_div";
  c2_info[58].dominantType = "double";
  c2_info[58].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c2_info[58].fileTimeLo = 1313347810U;
  c2_info[58].fileTimeHi = 0U;
  c2_info[58].mFileTimeLo = 0U;
  c2_info[58].mFileTimeHi = 0U;
  c2_info[59].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c2_info[59].name = "eml_xscal";
  c2_info[59].dominantType = "double";
  c2_info[59].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xscal.m";
  c2_info[59].fileTimeLo = 1299076776U;
  c2_info[59].fileTimeHi = 0U;
  c2_info[59].mFileTimeLo = 0U;
  c2_info[59].mFileTimeHi = 0U;
  c2_info[60].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xscal.m";
  c2_info[60].name = "eml_blas_inline";
  c2_info[60].dominantType = "";
  c2_info[60].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c2_info[60].fileTimeLo = 1299076768U;
  c2_info[60].fileTimeHi = 0U;
  c2_info[60].mFileTimeLo = 0U;
  c2_info[60].mFileTimeHi = 0U;
  c2_info[61].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xscal.m!below_threshold";
  c2_info[61].name = "length";
  c2_info[61].dominantType = "double";
  c2_info[61].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m";
  c2_info[61].fileTimeLo = 1303146206U;
  c2_info[61].fileTimeHi = 0U;
  c2_info[61].mFileTimeLo = 0U;
  c2_info[61].mFileTimeHi = 0U;
  c2_info[62].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xscal.m";
  c2_info[62].name = "eml_index_class";
  c2_info[62].dominantType = "";
  c2_info[62].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[62].fileTimeLo = 1323170578U;
  c2_info[62].fileTimeHi = 0U;
  c2_info[62].mFileTimeLo = 0U;
  c2_info[62].mFileTimeHi = 0U;
  c2_info[63].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xscal.m";
  c2_info[63].name = "eml_scalar_eg";
  c2_info[63].dominantType = "double";
  c2_info[63].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[63].fileTimeLo = 1286818796U;
  c2_info[63].fileTimeHi = 0U;
  c2_info[63].mFileTimeLo = 0U;
  c2_info[63].mFileTimeHi = 0U;
}

static void c2_b_info_helper(c2_ResolvedFunctionInfo c2_info[166])
{
  c2_info[64].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xscal.m";
  c2_info[64].name = "eml_refblas_xscal";
  c2_info[64].dominantType = "double";
  c2_info[64].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xscal.m";
  c2_info[64].fileTimeLo = 1299076784U;
  c2_info[64].fileTimeHi = 0U;
  c2_info[64].mFileTimeLo = 0U;
  c2_info[64].mFileTimeHi = 0U;
  c2_info[65].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xscal.m";
  c2_info[65].name = "eml_index_class";
  c2_info[65].dominantType = "";
  c2_info[65].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[65].fileTimeLo = 1323170578U;
  c2_info[65].fileTimeHi = 0U;
  c2_info[65].mFileTimeLo = 0U;
  c2_info[65].mFileTimeHi = 0U;
  c2_info[66].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xscal.m";
  c2_info[66].name = "eml_index_minus";
  c2_info[66].dominantType = "double";
  c2_info[66].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c2_info[66].fileTimeLo = 1286818778U;
  c2_info[66].fileTimeHi = 0U;
  c2_info[66].mFileTimeLo = 0U;
  c2_info[66].mFileTimeHi = 0U;
  c2_info[67].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xscal.m";
  c2_info[67].name = "eml_index_times";
  c2_info[67].dominantType = "coder.internal.indexInt";
  c2_info[67].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c2_info[67].fileTimeLo = 1286818780U;
  c2_info[67].fileTimeHi = 0U;
  c2_info[67].mFileTimeLo = 0U;
  c2_info[67].mFileTimeHi = 0U;
  c2_info[68].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xscal.m";
  c2_info[68].name = "eml_index_plus";
  c2_info[68].dominantType = "coder.internal.indexInt";
  c2_info[68].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[68].fileTimeLo = 1286818778U;
  c2_info[68].fileTimeHi = 0U;
  c2_info[68].mFileTimeLo = 0U;
  c2_info[68].mFileTimeHi = 0U;
  c2_info[69].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xscal.m";
  c2_info[69].name = "eml_int_forloop_overflow_check";
  c2_info[69].dominantType = "";
  c2_info[69].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[69].fileTimeLo = 1332168672U;
  c2_info[69].fileTimeHi = 0U;
  c2_info[69].mFileTimeLo = 0U;
  c2_info[69].mFileTimeHi = 0U;
  c2_info[70].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c2_info[70].name = "eml_xdotc";
  c2_info[70].dominantType = "double";
  c2_info[70].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdotc.m";
  c2_info[70].fileTimeLo = 1299076772U;
  c2_info[70].fileTimeHi = 0U;
  c2_info[70].mFileTimeLo = 0U;
  c2_info[70].mFileTimeHi = 0U;
  c2_info[71].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdotc.m";
  c2_info[71].name = "eml_blas_inline";
  c2_info[71].dominantType = "";
  c2_info[71].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c2_info[71].fileTimeLo = 1299076768U;
  c2_info[71].fileTimeHi = 0U;
  c2_info[71].mFileTimeLo = 0U;
  c2_info[71].mFileTimeHi = 0U;
  c2_info[72].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdotc.m";
  c2_info[72].name = "eml_xdot";
  c2_info[72].dominantType = "double";
  c2_info[72].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdot.m";
  c2_info[72].fileTimeLo = 1299076772U;
  c2_info[72].fileTimeHi = 0U;
  c2_info[72].mFileTimeLo = 0U;
  c2_info[72].mFileTimeHi = 0U;
  c2_info[73].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdot.m";
  c2_info[73].name = "eml_blas_inline";
  c2_info[73].dominantType = "";
  c2_info[73].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c2_info[73].fileTimeLo = 1299076768U;
  c2_info[73].fileTimeHi = 0U;
  c2_info[73].mFileTimeLo = 0U;
  c2_info[73].mFileTimeHi = 0U;
  c2_info[74].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xdot.m!below_threshold";
  c2_info[74].name = "length";
  c2_info[74].dominantType = "double";
  c2_info[74].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m";
  c2_info[74].fileTimeLo = 1303146206U;
  c2_info[74].fileTimeHi = 0U;
  c2_info[74].mFileTimeLo = 0U;
  c2_info[74].mFileTimeHi = 0U;
  c2_info[75].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xdot.m";
  c2_info[75].name = "eml_index_class";
  c2_info[75].dominantType = "";
  c2_info[75].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[75].fileTimeLo = 1323170578U;
  c2_info[75].fileTimeHi = 0U;
  c2_info[75].mFileTimeLo = 0U;
  c2_info[75].mFileTimeHi = 0U;
  c2_info[76].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xdot.m";
  c2_info[76].name = "eml_refblas_xdot";
  c2_info[76].dominantType = "double";
  c2_info[76].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdot.m";
  c2_info[76].fileTimeLo = 1299076772U;
  c2_info[76].fileTimeHi = 0U;
  c2_info[76].mFileTimeLo = 0U;
  c2_info[76].mFileTimeHi = 0U;
  c2_info[77].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdot.m";
  c2_info[77].name = "eml_refblas_xdotx";
  c2_info[77].dominantType = "char";
  c2_info[77].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m";
  c2_info[77].fileTimeLo = 1299076774U;
  c2_info[77].fileTimeHi = 0U;
  c2_info[77].mFileTimeLo = 0U;
  c2_info[77].mFileTimeHi = 0U;
  c2_info[78].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m";
  c2_info[78].name = "eml_scalar_eg";
  c2_info[78].dominantType = "double";
  c2_info[78].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[78].fileTimeLo = 1286818796U;
  c2_info[78].fileTimeHi = 0U;
  c2_info[78].mFileTimeLo = 0U;
  c2_info[78].mFileTimeHi = 0U;
  c2_info[79].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m";
  c2_info[79].name = "eml_index_class";
  c2_info[79].dominantType = "";
  c2_info[79].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[79].fileTimeLo = 1323170578U;
  c2_info[79].fileTimeHi = 0U;
  c2_info[79].mFileTimeLo = 0U;
  c2_info[79].mFileTimeHi = 0U;
  c2_info[80].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m";
  c2_info[80].name = "eml_int_forloop_overflow_check";
  c2_info[80].dominantType = "";
  c2_info[80].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[80].fileTimeLo = 1332168672U;
  c2_info[80].fileTimeHi = 0U;
  c2_info[80].mFileTimeLo = 0U;
  c2_info[80].mFileTimeHi = 0U;
  c2_info[81].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m";
  c2_info[81].name = "eml_index_plus";
  c2_info[81].dominantType = "coder.internal.indexInt";
  c2_info[81].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[81].fileTimeLo = 1286818778U;
  c2_info[81].fileTimeHi = 0U;
  c2_info[81].mFileTimeLo = 0U;
  c2_info[81].mFileTimeHi = 0U;
  c2_info[82].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c2_info[82].name = "eml_xaxpy";
  c2_info[82].dominantType = "double";
  c2_info[82].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xaxpy.m";
  c2_info[82].fileTimeLo = 1299076770U;
  c2_info[82].fileTimeHi = 0U;
  c2_info[82].mFileTimeLo = 0U;
  c2_info[82].mFileTimeHi = 0U;
  c2_info[83].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xaxpy.m";
  c2_info[83].name = "eml_blas_inline";
  c2_info[83].dominantType = "";
  c2_info[83].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c2_info[83].fileTimeLo = 1299076768U;
  c2_info[83].fileTimeHi = 0U;
  c2_info[83].mFileTimeLo = 0U;
  c2_info[83].mFileTimeHi = 0U;
  c2_info[84].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xaxpy.m!below_threshold";
  c2_info[84].name = "length";
  c2_info[84].dominantType = "double";
  c2_info[84].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m";
  c2_info[84].fileTimeLo = 1303146206U;
  c2_info[84].fileTimeHi = 0U;
  c2_info[84].mFileTimeLo = 0U;
  c2_info[84].mFileTimeHi = 0U;
  c2_info[85].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xaxpy.m";
  c2_info[85].name = "eml_index_class";
  c2_info[85].dominantType = "";
  c2_info[85].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[85].fileTimeLo = 1323170578U;
  c2_info[85].fileTimeHi = 0U;
  c2_info[85].mFileTimeLo = 0U;
  c2_info[85].mFileTimeHi = 0U;
  c2_info[86].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xaxpy.m";
  c2_info[86].name = "eml_scalar_eg";
  c2_info[86].dominantType = "double";
  c2_info[86].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[86].fileTimeLo = 1286818796U;
  c2_info[86].fileTimeHi = 0U;
  c2_info[86].mFileTimeLo = 0U;
  c2_info[86].mFileTimeHi = 0U;
  c2_info[87].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xaxpy.m";
  c2_info[87].name = "eml_refblas_xaxpy";
  c2_info[87].dominantType = "double";
  c2_info[87].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xaxpy.m";
  c2_info[87].fileTimeLo = 1299076772U;
  c2_info[87].fileTimeHi = 0U;
  c2_info[87].mFileTimeLo = 0U;
  c2_info[87].mFileTimeHi = 0U;
  c2_info[88].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xaxpy.m";
  c2_info[88].name = "eml_index_class";
  c2_info[88].dominantType = "";
  c2_info[88].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[88].fileTimeLo = 1323170578U;
  c2_info[88].fileTimeHi = 0U;
  c2_info[88].mFileTimeLo = 0U;
  c2_info[88].mFileTimeHi = 0U;
  c2_info[89].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xaxpy.m";
  c2_info[89].name = "eml_isa_uint";
  c2_info[89].dominantType = "coder.internal.indexInt";
  c2_info[89].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isa_uint.m";
  c2_info[89].fileTimeLo = 1286818784U;
  c2_info[89].fileTimeHi = 0U;
  c2_info[89].mFileTimeLo = 0U;
  c2_info[89].mFileTimeHi = 0U;
  c2_info[90].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xaxpy.m";
  c2_info[90].name = "eml_index_minus";
  c2_info[90].dominantType = "double";
  c2_info[90].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c2_info[90].fileTimeLo = 1286818778U;
  c2_info[90].fileTimeHi = 0U;
  c2_info[90].mFileTimeLo = 0U;
  c2_info[90].mFileTimeHi = 0U;
  c2_info[91].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xaxpy.m";
  c2_info[91].name = "eml_int_forloop_overflow_check";
  c2_info[91].dominantType = "";
  c2_info[91].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[91].fileTimeLo = 1332168672U;
  c2_info[91].fileTimeHi = 0U;
  c2_info[91].mFileTimeLo = 0U;
  c2_info[91].mFileTimeHi = 0U;
  c2_info[92].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xaxpy.m";
  c2_info[92].name = "eml_index_plus";
  c2_info[92].dominantType = "double";
  c2_info[92].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[92].fileTimeLo = 1286818778U;
  c2_info[92].fileTimeHi = 0U;
  c2_info[92].mFileTimeLo = 0U;
  c2_info[92].mFileTimeHi = 0U;
  c2_info[93].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xaxpy.m";
  c2_info[93].name = "eml_index_plus";
  c2_info[93].dominantType = "coder.internal.indexInt";
  c2_info[93].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[93].fileTimeLo = 1286818778U;
  c2_info[93].fileTimeHi = 0U;
  c2_info[93].mFileTimeLo = 0U;
  c2_info[93].mFileTimeHi = 0U;
  c2_info[94].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper";
  c2_info[94].name = "intmin";
  c2_info[94].dominantType = "char";
  c2_info[94].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m";
  c2_info[94].fileTimeLo = 1311255318U;
  c2_info[94].fileTimeHi = 0U;
  c2_info[94].mFileTimeLo = 0U;
  c2_info[94].mFileTimeHi = 0U;
  c2_info[95].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c2_info[95].name = "abs";
  c2_info[95].dominantType = "double";
  c2_info[95].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c2_info[95].fileTimeLo = 1286818694U;
  c2_info[95].fileTimeHi = 0U;
  c2_info[95].mFileTimeLo = 0U;
  c2_info[95].mFileTimeHi = 0U;
  c2_info[96].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c2_info[96].name = "mtimes";
  c2_info[96].dominantType = "double";
  c2_info[96].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[96].fileTimeLo = 1289519692U;
  c2_info[96].fileTimeHi = 0U;
  c2_info[96].mFileTimeLo = 0U;
  c2_info[96].mFileTimeHi = 0U;
  c2_info[97].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c2_info[97].name = "realmin";
  c2_info[97].dominantType = "char";
  c2_info[97].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m";
  c2_info[97].fileTimeLo = 1307651242U;
  c2_info[97].fileTimeHi = 0U;
  c2_info[97].mFileTimeLo = 0U;
  c2_info[97].mFileTimeHi = 0U;
  c2_info[98].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c2_info[98].name = "eps";
  c2_info[98].dominantType = "char";
  c2_info[98].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c2_info[98].fileTimeLo = 1326727996U;
  c2_info[98].fileTimeHi = 0U;
  c2_info[98].mFileTimeLo = 0U;
  c2_info[98].mFileTimeHi = 0U;
  c2_info[99].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c2_info[99].name = "eml_is_float_class";
  c2_info[99].dominantType = "char";
  c2_info[99].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_is_float_class.m";
  c2_info[99].fileTimeLo = 1286818782U;
  c2_info[99].fileTimeHi = 0U;
  c2_info[99].mFileTimeLo = 0U;
  c2_info[99].mFileTimeHi = 0U;
  c2_info[100].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c2_info[100].name = "eml_eps";
  c2_info[100].dominantType = "char";
  c2_info[100].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_eps.m";
  c2_info[100].fileTimeLo = 1326727996U;
  c2_info[100].fileTimeHi = 0U;
  c2_info[100].mFileTimeLo = 0U;
  c2_info[100].mFileTimeHi = 0U;
  c2_info[101].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_eps.m";
  c2_info[101].name = "eml_float_model";
  c2_info[101].dominantType = "char";
  c2_info[101].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m";
  c2_info[101].fileTimeLo = 1326727996U;
  c2_info[101].fileTimeHi = 0U;
  c2_info[101].mFileTimeLo = 0U;
  c2_info[101].mFileTimeHi = 0U;
  c2_info[102].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c2_info[102].name = "eml_error";
  c2_info[102].dominantType = "char";
  c2_info[102].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m";
  c2_info[102].fileTimeLo = 1305318000U;
  c2_info[102].fileTimeHi = 0U;
  c2_info[102].mFileTimeLo = 0U;
  c2_info[102].mFileTimeHi = 0U;
  c2_info[103].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum";
  c2_info[103].name = "eml_const_nonsingleton_dim";
  c2_info[103].dominantType = "double";
  c2_info[103].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_const_nonsingleton_dim.m";
  c2_info[103].fileTimeLo = 1286818696U;
  c2_info[103].fileTimeHi = 0U;
  c2_info[103].mFileTimeLo = 0U;
  c2_info[103].mFileTimeHi = 0U;
  c2_info[104].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum";
  c2_info[104].name = "eml_scalar_eg";
  c2_info[104].dominantType = "double";
  c2_info[104].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[104].fileTimeLo = 1286818796U;
  c2_info[104].fileTimeHi = 0U;
  c2_info[104].mFileTimeLo = 0U;
  c2_info[104].mFileTimeHi = 0U;
  c2_info[105].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum";
  c2_info[105].name = "eml_index_class";
  c2_info[105].dominantType = "";
  c2_info[105].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[105].fileTimeLo = 1323170578U;
  c2_info[105].fileTimeHi = 0U;
  c2_info[105].mFileTimeLo = 0U;
  c2_info[105].mFileTimeHi = 0U;
  c2_info[106].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum_sub";
  c2_info[106].name = "eml_index_class";
  c2_info[106].dominantType = "";
  c2_info[106].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[106].fileTimeLo = 1323170578U;
  c2_info[106].fileTimeHi = 0U;
  c2_info[106].mFileTimeLo = 0U;
  c2_info[106].mFileTimeHi = 0U;
  c2_info[107].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum_sub";
  c2_info[107].name = "isnan";
  c2_info[107].dominantType = "double";
  c2_info[107].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m";
  c2_info[107].fileTimeLo = 1286818760U;
  c2_info[107].fileTimeHi = 0U;
  c2_info[107].mFileTimeLo = 0U;
  c2_info[107].mFileTimeHi = 0U;
  c2_info[108].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum_sub";
  c2_info[108].name = "eml_index_plus";
  c2_info[108].dominantType = "coder.internal.indexInt";
  c2_info[108].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[108].fileTimeLo = 1286818778U;
  c2_info[108].fileTimeHi = 0U;
  c2_info[108].mFileTimeLo = 0U;
  c2_info[108].mFileTimeHi = 0U;
  c2_info[109].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum_sub";
  c2_info[109].name = "eml_int_forloop_overflow_check";
  c2_info[109].dominantType = "";
  c2_info[109].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[109].fileTimeLo = 1332168672U;
  c2_info[109].fileTimeHi = 0U;
  c2_info[109].mFileTimeLo = 0U;
  c2_info[109].mFileTimeHi = 0U;
  c2_info[110].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum_sub";
  c2_info[110].name = "eml_relop";
  c2_info[110].dominantType = "function_handle";
  c2_info[110].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_relop.m";
  c2_info[110].fileTimeLo = 1326727998U;
  c2_info[110].fileTimeHi = 0U;
  c2_info[110].mFileTimeLo = 0U;
  c2_info[110].mFileTimeHi = 0U;
  c2_info[111].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c2_info[111].name = "sqrt";
  c2_info[111].dominantType = "double";
  c2_info[111].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m";
  c2_info[111].fileTimeLo = 1286818752U;
  c2_info[111].fileTimeHi = 0U;
  c2_info[111].mFileTimeLo = 0U;
  c2_info[111].mFileTimeHi = 0U;
  c2_info[112].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m";
  c2_info[112].name = "eml_error";
  c2_info[112].dominantType = "char";
  c2_info[112].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m";
  c2_info[112].fileTimeLo = 1305318000U;
  c2_info[112].fileTimeHi = 0U;
  c2_info[112].mFileTimeLo = 0U;
  c2_info[112].mFileTimeHi = 0U;
  c2_info[113].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m";
  c2_info[113].name = "eml_scalar_sqrt";
  c2_info[113].dominantType = "double";
  c2_info[113].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sqrt.m";
  c2_info[113].fileTimeLo = 1286818738U;
  c2_info[113].fileTimeHi = 0U;
  c2_info[113].mFileTimeLo = 0U;
  c2_info[113].mFileTimeHi = 0U;
  c2_info[114].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c2_info[114].name = "eml_xrotg";
  c2_info[114].dominantType = "double";
  c2_info[114].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xrotg.m";
  c2_info[114].fileTimeLo = 1299076776U;
  c2_info[114].fileTimeHi = 0U;
  c2_info[114].mFileTimeLo = 0U;
  c2_info[114].mFileTimeHi = 0U;
  c2_info[115].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xrotg.m";
  c2_info[115].name = "eml_blas_inline";
  c2_info[115].dominantType = "";
  c2_info[115].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c2_info[115].fileTimeLo = 1299076768U;
  c2_info[115].fileTimeHi = 0U;
  c2_info[115].mFileTimeLo = 0U;
  c2_info[115].mFileTimeHi = 0U;
  c2_info[116].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xrotg.m";
  c2_info[116].name = "eml_refblas_xrotg";
  c2_info[116].dominantType = "double";
  c2_info[116].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xrotg.m";
  c2_info[116].fileTimeLo = 1299076784U;
  c2_info[116].fileTimeHi = 0U;
  c2_info[116].mFileTimeLo = 0U;
  c2_info[116].mFileTimeHi = 0U;
  c2_info[117].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xrotg.m";
  c2_info[117].name = "abs";
  c2_info[117].dominantType = "double";
  c2_info[117].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c2_info[117].fileTimeLo = 1286818694U;
  c2_info[117].fileTimeHi = 0U;
  c2_info[117].mFileTimeLo = 0U;
  c2_info[117].mFileTimeHi = 0U;
  c2_info[118].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xrotg.m";
  c2_info[118].name = "mrdivide";
  c2_info[118].dominantType = "double";
  c2_info[118].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c2_info[118].fileTimeLo = 1342810944U;
  c2_info[118].fileTimeHi = 0U;
  c2_info[118].mFileTimeLo = 1319729966U;
  c2_info[118].mFileTimeHi = 0U;
  c2_info[119].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c2_info[119].name = "rdivide";
  c2_info[119].dominantType = "double";
  c2_info[119].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c2_info[119].fileTimeLo = 1286818844U;
  c2_info[119].fileTimeHi = 0U;
  c2_info[119].mFileTimeLo = 0U;
  c2_info[119].mFileTimeHi = 0U;
  c2_info[120].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c2_info[120].name = "eml_div";
  c2_info[120].dominantType = "double";
  c2_info[120].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c2_info[120].fileTimeLo = 1313347810U;
  c2_info[120].fileTimeHi = 0U;
  c2_info[120].mFileTimeLo = 0U;
  c2_info[120].mFileTimeHi = 0U;
  c2_info[121].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xrotg.m";
  c2_info[121].name = "sqrt";
  c2_info[121].dominantType = "double";
  c2_info[121].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m";
  c2_info[121].fileTimeLo = 1286818752U;
  c2_info[121].fileTimeHi = 0U;
  c2_info[121].mFileTimeLo = 0U;
  c2_info[121].mFileTimeHi = 0U;
  c2_info[122].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xrotg.m!eml_ceval_xrotg";
  c2_info[122].name = "eml_scalar_eg";
  c2_info[122].dominantType = "double";
  c2_info[122].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[122].fileTimeLo = 1286818796U;
  c2_info[122].fileTimeHi = 0U;
  c2_info[122].mFileTimeLo = 0U;
  c2_info[122].mFileTimeHi = 0U;
  c2_info[123].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c2_info[123].name = "eml_xrot";
  c2_info[123].dominantType = "double";
  c2_info[123].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xrot.m";
  c2_info[123].fileTimeLo = 1299076776U;
  c2_info[123].fileTimeHi = 0U;
  c2_info[123].mFileTimeLo = 0U;
  c2_info[123].mFileTimeHi = 0U;
  c2_info[124].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xrot.m";
  c2_info[124].name = "eml_blas_inline";
  c2_info[124].dominantType = "";
  c2_info[124].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c2_info[124].fileTimeLo = 1299076768U;
  c2_info[124].fileTimeHi = 0U;
  c2_info[124].mFileTimeLo = 0U;
  c2_info[124].mFileTimeHi = 0U;
  c2_info[125].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xrot.m";
  c2_info[125].name = "eml_index_class";
  c2_info[125].dominantType = "";
  c2_info[125].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[125].fileTimeLo = 1323170578U;
  c2_info[125].fileTimeHi = 0U;
  c2_info[125].mFileTimeLo = 0U;
  c2_info[125].mFileTimeHi = 0U;
  c2_info[126].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xrot.m";
  c2_info[126].name = "eml_scalar_eg";
  c2_info[126].dominantType = "double";
  c2_info[126].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[126].fileTimeLo = 1286818796U;
  c2_info[126].fileTimeHi = 0U;
  c2_info[126].mFileTimeLo = 0U;
  c2_info[126].mFileTimeHi = 0U;
  c2_info[127].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xrot.m";
  c2_info[127].name = "eml_refblas_xrot";
  c2_info[127].dominantType = "double";
  c2_info[127].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xrot.m";
  c2_info[127].fileTimeLo = 1299076784U;
  c2_info[127].fileTimeHi = 0U;
  c2_info[127].mFileTimeLo = 0U;
  c2_info[127].mFileTimeHi = 0U;
}

static void c2_c_info_helper(c2_ResolvedFunctionInfo c2_info[166])
{
  c2_info[128].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xrot.m";
  c2_info[128].name = "eml_index_class";
  c2_info[128].dominantType = "";
  c2_info[128].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[128].fileTimeLo = 1323170578U;
  c2_info[128].fileTimeHi = 0U;
  c2_info[128].mFileTimeLo = 0U;
  c2_info[128].mFileTimeHi = 0U;
  c2_info[129].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xrot.m";
  c2_info[129].name = "eml_int_forloop_overflow_check";
  c2_info[129].dominantType = "";
  c2_info[129].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[129].fileTimeLo = 1332168672U;
  c2_info[129].fileTimeHi = 0U;
  c2_info[129].mFileTimeLo = 0U;
  c2_info[129].mFileTimeHi = 0U;
  c2_info[130].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xrot.m";
  c2_info[130].name = "mtimes";
  c2_info[130].dominantType = "double";
  c2_info[130].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[130].fileTimeLo = 1289519692U;
  c2_info[130].fileTimeHi = 0U;
  c2_info[130].mFileTimeLo = 0U;
  c2_info[130].mFileTimeHi = 0U;
  c2_info[131].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xrot.m";
  c2_info[131].name = "eml_index_plus";
  c2_info[131].dominantType = "coder.internal.indexInt";
  c2_info[131].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[131].fileTimeLo = 1286818778U;
  c2_info[131].fileTimeHi = 0U;
  c2_info[131].mFileTimeLo = 0U;
  c2_info[131].mFileTimeHi = 0U;
  c2_info[132].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c2_info[132].name = "eml_xswap";
  c2_info[132].dominantType = "double";
  c2_info[132].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xswap.m";
  c2_info[132].fileTimeLo = 1299076778U;
  c2_info[132].fileTimeHi = 0U;
  c2_info[132].mFileTimeLo = 0U;
  c2_info[132].mFileTimeHi = 0U;
  c2_info[133].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xswap.m";
  c2_info[133].name = "eml_blas_inline";
  c2_info[133].dominantType = "";
  c2_info[133].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c2_info[133].fileTimeLo = 1299076768U;
  c2_info[133].fileTimeHi = 0U;
  c2_info[133].mFileTimeLo = 0U;
  c2_info[133].mFileTimeHi = 0U;
  c2_info[134].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xswap.m";
  c2_info[134].name = "eml_index_class";
  c2_info[134].dominantType = "";
  c2_info[134].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[134].fileTimeLo = 1323170578U;
  c2_info[134].fileTimeHi = 0U;
  c2_info[134].mFileTimeLo = 0U;
  c2_info[134].mFileTimeHi = 0U;
  c2_info[135].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xswap.m";
  c2_info[135].name = "eml_refblas_xswap";
  c2_info[135].dominantType = "double";
  c2_info[135].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c2_info[135].fileTimeLo = 1299076786U;
  c2_info[135].fileTimeHi = 0U;
  c2_info[135].mFileTimeLo = 0U;
  c2_info[135].mFileTimeHi = 0U;
  c2_info[136].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c2_info[136].name = "eml_index_class";
  c2_info[136].dominantType = "";
  c2_info[136].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[136].fileTimeLo = 1323170578U;
  c2_info[136].fileTimeHi = 0U;
  c2_info[136].mFileTimeLo = 0U;
  c2_info[136].mFileTimeHi = 0U;
  c2_info[137].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c2_info[137].name = "abs";
  c2_info[137].dominantType = "coder.internal.indexInt";
  c2_info[137].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c2_info[137].fileTimeLo = 1286818694U;
  c2_info[137].fileTimeHi = 0U;
  c2_info[137].mFileTimeLo = 0U;
  c2_info[137].mFileTimeHi = 0U;
  c2_info[138].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c2_info[138].name = "eml_scalar_abs";
  c2_info[138].dominantType = "coder.internal.indexInt";
  c2_info[138].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  c2_info[138].fileTimeLo = 1286818712U;
  c2_info[138].fileTimeHi = 0U;
  c2_info[138].mFileTimeLo = 0U;
  c2_info[138].mFileTimeHi = 0U;
  c2_info[139].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c2_info[139].name = "eml_int_forloop_overflow_check";
  c2_info[139].dominantType = "";
  c2_info[139].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[139].fileTimeLo = 1332168672U;
  c2_info[139].fileTimeHi = 0U;
  c2_info[139].mFileTimeLo = 0U;
  c2_info[139].mFileTimeHi = 0U;
  c2_info[140].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c2_info[140].name = "eml_index_plus";
  c2_info[140].dominantType = "coder.internal.indexInt";
  c2_info[140].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[140].fileTimeLo = 1286818778U;
  c2_info[140].fileTimeHi = 0U;
  c2_info[140].mFileTimeLo = 0U;
  c2_info[140].mFileTimeHi = 0U;
  c2_info[141].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv";
  c2_info[141].name = "mtimes";
  c2_info[141].dominantType = "double";
  c2_info[141].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[141].fileTimeLo = 1289519692U;
  c2_info[141].fileTimeHi = 0U;
  c2_info[141].mFileTimeLo = 0U;
  c2_info[141].mFileTimeHi = 0U;
  c2_info[142].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv";
  c2_info[142].name = "eps";
  c2_info[142].dominantType = "char";
  c2_info[142].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c2_info[142].fileTimeLo = 1326727996U;
  c2_info[142].fileTimeHi = 0U;
  c2_info[142].mFileTimeLo = 0U;
  c2_info[142].mFileTimeHi = 0U;
  c2_info[143].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv";
  c2_info[143].name = "eml_int_forloop_overflow_check";
  c2_info[143].dominantType = "";
  c2_info[143].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[143].fileTimeLo = 1332168672U;
  c2_info[143].fileTimeHi = 0U;
  c2_info[143].mFileTimeLo = 0U;
  c2_info[143].mFileTimeHi = 0U;
  c2_info[144].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv";
  c2_info[144].name = "eml_index_plus";
  c2_info[144].dominantType = "double";
  c2_info[144].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[144].fileTimeLo = 1286818778U;
  c2_info[144].fileTimeHi = 0U;
  c2_info[144].mFileTimeLo = 0U;
  c2_info[144].mFileTimeHi = 0U;
  c2_info[145].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv";
  c2_info[145].name = "eml_div";
  c2_info[145].dominantType = "double";
  c2_info[145].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c2_info[145].fileTimeLo = 1313347810U;
  c2_info[145].fileTimeHi = 0U;
  c2_info[145].mFileTimeLo = 0U;
  c2_info[145].mFileTimeHi = 0U;
  c2_info[146].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv";
  c2_info[146].name = "eml_xscal";
  c2_info[146].dominantType = "double";
  c2_info[146].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xscal.m";
  c2_info[146].fileTimeLo = 1299076776U;
  c2_info[146].fileTimeHi = 0U;
  c2_info[146].mFileTimeLo = 0U;
  c2_info[146].mFileTimeHi = 0U;
  c2_info[147].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv";
  c2_info[147].name = "eml_index_plus";
  c2_info[147].dominantType = "coder.internal.indexInt";
  c2_info[147].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[147].fileTimeLo = 1286818778U;
  c2_info[147].fileTimeHi = 0U;
  c2_info[147].mFileTimeLo = 0U;
  c2_info[147].mFileTimeHi = 0U;
  c2_info[148].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv";
  c2_info[148].name = "eml_xgemm";
  c2_info[148].dominantType = "char";
  c2_info[148].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m";
  c2_info[148].fileTimeLo = 1299076772U;
  c2_info[148].fileTimeHi = 0U;
  c2_info[148].mFileTimeLo = 0U;
  c2_info[148].mFileTimeHi = 0U;
  c2_info[149].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m";
  c2_info[149].name = "eml_blas_inline";
  c2_info[149].dominantType = "";
  c2_info[149].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c2_info[149].fileTimeLo = 1299076768U;
  c2_info[149].fileTimeHi = 0U;
  c2_info[149].mFileTimeLo = 0U;
  c2_info[149].mFileTimeHi = 0U;
  c2_info[150].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m!below_threshold";
  c2_info[150].name = "min";
  c2_info[150].dominantType = "double";
  c2_info[150].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c2_info[150].fileTimeLo = 1311255318U;
  c2_info[150].fileTimeHi = 0U;
  c2_info[150].mFileTimeLo = 0U;
  c2_info[150].mFileTimeHi = 0U;
  c2_info[151].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m!below_threshold";
  c2_info[151].name = "mtimes";
  c2_info[151].dominantType = "double";
  c2_info[151].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[151].fileTimeLo = 1289519692U;
  c2_info[151].fileTimeHi = 0U;
  c2_info[151].mFileTimeLo = 0U;
  c2_info[151].mFileTimeHi = 0U;
  c2_info[152].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c2_info[152].name = "eml_index_class";
  c2_info[152].dominantType = "";
  c2_info[152].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[152].fileTimeLo = 1323170578U;
  c2_info[152].fileTimeHi = 0U;
  c2_info[152].mFileTimeLo = 0U;
  c2_info[152].mFileTimeHi = 0U;
  c2_info[153].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c2_info[153].name = "eml_scalar_eg";
  c2_info[153].dominantType = "double";
  c2_info[153].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[153].fileTimeLo = 1286818796U;
  c2_info[153].fileTimeHi = 0U;
  c2_info[153].mFileTimeLo = 0U;
  c2_info[153].mFileTimeHi = 0U;
  c2_info[154].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c2_info[154].name = "eml_refblas_xgemm";
  c2_info[154].dominantType = "char";
  c2_info[154].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemm.m";
  c2_info[154].fileTimeLo = 1299076774U;
  c2_info[154].fileTimeHi = 0U;
  c2_info[154].mFileTimeLo = 0U;
  c2_info[154].mFileTimeHi = 0U;
  c2_info[155].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemm.m";
  c2_info[155].name = "eml_index_minus";
  c2_info[155].dominantType = "double";
  c2_info[155].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c2_info[155].fileTimeLo = 1286818778U;
  c2_info[155].fileTimeHi = 0U;
  c2_info[155].mFileTimeLo = 0U;
  c2_info[155].mFileTimeHi = 0U;
  c2_info[156].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemm.m";
  c2_info[156].name = "eml_index_class";
  c2_info[156].dominantType = "";
  c2_info[156].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[156].fileTimeLo = 1323170578U;
  c2_info[156].fileTimeHi = 0U;
  c2_info[156].mFileTimeLo = 0U;
  c2_info[156].mFileTimeHi = 0U;
  c2_info[157].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemm.m";
  c2_info[157].name = "eml_scalar_eg";
  c2_info[157].dominantType = "double";
  c2_info[157].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[157].fileTimeLo = 1286818796U;
  c2_info[157].fileTimeHi = 0U;
  c2_info[157].mFileTimeLo = 0U;
  c2_info[157].mFileTimeHi = 0U;
  c2_info[158].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemm.m";
  c2_info[158].name = "eml_index_times";
  c2_info[158].dominantType = "coder.internal.indexInt";
  c2_info[158].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c2_info[158].fileTimeLo = 1286818780U;
  c2_info[158].fileTimeHi = 0U;
  c2_info[158].mFileTimeLo = 0U;
  c2_info[158].mFileTimeHi = 0U;
  c2_info[159].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemm.m";
  c2_info[159].name = "eml_index_plus";
  c2_info[159].dominantType = "coder.internal.indexInt";
  c2_info[159].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[159].fileTimeLo = 1286818778U;
  c2_info[159].fileTimeHi = 0U;
  c2_info[159].mFileTimeLo = 0U;
  c2_info[159].mFileTimeHi = 0U;
  c2_info[160].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemm.m";
  c2_info[160].name = "eml_int_forloop_overflow_check";
  c2_info[160].dominantType = "";
  c2_info[160].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[160].fileTimeLo = 1332168672U;
  c2_info[160].fileTimeHi = 0U;
  c2_info[160].mFileTimeLo = 0U;
  c2_info[160].mFileTimeHi = 0U;
  c2_info[161].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemm.m";
  c2_info[161].name = "eml_index_plus";
  c2_info[161].dominantType = "double";
  c2_info[161].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[161].fileTimeLo = 1286818778U;
  c2_info[161].fileTimeHi = 0U;
  c2_info[161].mFileTimeLo = 0U;
  c2_info[161].mFileTimeHi = 0U;
  c2_info[162].context = "";
  c2_info[162].name = "mtimes";
  c2_info[162].dominantType = "double";
  c2_info[162].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[162].fileTimeLo = 1289519692U;
  c2_info[162].fileTimeHi = 0U;
  c2_info[162].mFileTimeLo = 0U;
  c2_info[162].mFileTimeHi = 0U;
  c2_info[163].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[163].name = "eml_index_class";
  c2_info[163].dominantType = "";
  c2_info[163].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[163].fileTimeLo = 1323170578U;
  c2_info[163].fileTimeHi = 0U;
  c2_info[163].mFileTimeLo = 0U;
  c2_info[163].mFileTimeHi = 0U;
  c2_info[164].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[164].name = "eml_scalar_eg";
  c2_info[164].dominantType = "double";
  c2_info[164].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[164].fileTimeLo = 1286818796U;
  c2_info[164].fileTimeHi = 0U;
  c2_info[164].mFileTimeLo = 0U;
  c2_info[164].mFileTimeHi = 0U;
  c2_info[165].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[165].name = "eml_xgemm";
  c2_info[165].dominantType = "char";
  c2_info[165].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m";
  c2_info[165].fileTimeLo = 1299076772U;
  c2_info[165].fileTimeHi = 0U;
  c2_info[165].mFileTimeLo = 0U;
  c2_info[165].mFileTimeHi = 0U;
}

static void c2_pinv(SFc2_WBCLibraryInstanceStruct *chartInstance, real_T c2_A[72],
                    real_T c2_X[72])
{
  int32_T c2_i41;
  int32_T c2_k;
  int32_T c2_b_k;
  real_T c2_x;
  real_T c2_b_x;
  boolean_T c2_b;
  boolean_T c2_b0;
  real_T c2_c_x;
  boolean_T c2_b_b;
  boolean_T c2_b1;
  boolean_T c2_c_b;
  int32_T c2_i42;
  real_T c2_b_A[72];
  real_T c2_V[36];
  real_T c2_s[6];
  real_T c2_U[72];
  int32_T c2_i43;
  real_T c2_S[36];
  int32_T c2_c_k;
  real_T c2_d_k;
  real_T c2_d_b;
  real_T c2_y;
  real_T c2_a;
  real_T c2_tol;
  int32_T c2_r;
  int32_T c2_e_k;
  int32_T c2_f_k;
  int32_T c2_b_a;
  int32_T c2_vcol;
  int32_T c2_b_r;
  int32_T c2_e_b;
  int32_T c2_f_b;
  boolean_T c2_overflow;
  int32_T c2_j;
  int32_T c2_b_j;
  real_T c2_b_y;
  real_T c2_z;
  int32_T c2_c_a;
  int32_T c2_i44;
  real_T c2_b_V[36];
  int32_T c2_i45;
  real_T c2_b_U[72];
  boolean_T exitg1;
  for (c2_i41 = 0; c2_i41 < 72; c2_i41++) {
    c2_X[c2_i41] = 0.0;
  }

  c2_check_forloop_overflow_error(chartInstance);
  for (c2_k = 1; c2_k < 73; c2_k++) {
    c2_b_k = c2_k;
    c2_x = c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c2_b_k), 1, 72, 1, 0) - 1];
    c2_b_x = c2_x;
    c2_b = muDoubleScalarIsInf(c2_b_x);
    c2_b0 = !c2_b;
    c2_c_x = c2_x;
    c2_b_b = muDoubleScalarIsNaN(c2_c_x);
    c2_b1 = !c2_b_b;
    c2_c_b = (c2_b0 && c2_b1);
    if (!c2_c_b) {
      c2_eml_error(chartInstance);
    }
  }

  for (c2_i42 = 0; c2_i42 < 72; c2_i42++) {
    c2_b_A[c2_i42] = c2_A[c2_i42];
  }

  c2_eml_xgesvd(chartInstance, c2_b_A, c2_U, c2_s, c2_V);
  for (c2_i43 = 0; c2_i43 < 36; c2_i43++) {
    c2_S[c2_i43] = 0.0;
  }

  for (c2_c_k = 0; c2_c_k < 6; c2_c_k++) {
    c2_d_k = 1.0 + (real_T)c2_c_k;
    c2_S[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", c2_d_k),
           1, 6, 1, 0) + 6 * (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", c2_d_k), 1, 6, 2, 0) - 1)) - 1] =
      c2_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      c2_d_k), 1, 6, 1, 0) - 1];
  }

  c2_eps(chartInstance);
  c2_d_b = c2_S[0];
  c2_y = 12.0 * c2_d_b;
  c2_a = c2_y;
  c2_tol = c2_a * 2.2204460492503131E-16;
  c2_r = 0;
  c2_check_forloop_overflow_error(chartInstance);
  c2_e_k = 1;
  exitg1 = FALSE;
  while ((exitg1 == FALSE) && (c2_e_k < 7)) {
    c2_f_k = c2_e_k;
    if (!(c2_S[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_f_k), 1, 6, 1, 0) + 6 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
            (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_f_k), 1, 6, 2, 0) - 1)) -
          1] > c2_tol)) {
      exitg1 = TRUE;
    } else {
      c2_b_a = c2_r + 1;
      c2_r = c2_b_a;
      c2_e_k++;
    }
  }

  if (c2_r > 0) {
    c2_vcol = 1;
    c2_b_r = c2_r;
    c2_e_b = c2_b_r;
    c2_f_b = c2_e_b;
    if (1 > c2_f_b) {
      c2_overflow = FALSE;
    } else {
      c2_overflow = (c2_f_b > 2147483646);
    }

    c2_b_check_forloop_overflow_error(chartInstance, c2_overflow);
    for (c2_j = 1; c2_j <= c2_b_r; c2_j++) {
      c2_b_j = c2_j;
      c2_b_y = c2_S[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK
                      ("", (real_T)c2_b_j), 1, 6, 1, 0) + 6 *
                     (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c2_b_j), 1, 6, 2, 0) - 1)) - 1];
      c2_z = 1.0 / c2_b_y;
      c2_f_eml_xscal(chartInstance, c2_z, c2_V, c2_vcol);
      c2_c_a = c2_vcol + 6;
      c2_vcol = c2_c_a;
    }

    for (c2_i44 = 0; c2_i44 < 36; c2_i44++) {
      c2_b_V[c2_i44] = c2_V[c2_i44];
    }

    for (c2_i45 = 0; c2_i45 < 72; c2_i45++) {
      c2_b_U[c2_i45] = c2_U[c2_i45];
    }

    c2_c_eml_xgemm(chartInstance, c2_r, c2_b_V, c2_b_U, c2_X);
  }
}

static void c2_eml_scalar_eg(SFc2_WBCLibraryInstanceStruct *chartInstance)
{
}

static void c2_check_forloop_overflow_error(SFc2_WBCLibraryInstanceStruct
  *chartInstance)
{
}

static void c2_eml_error(SFc2_WBCLibraryInstanceStruct *chartInstance)
{
  int32_T c2_i46;
  static char_T c2_varargin_1[33] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A',
    'T', 'L', 'A', 'B', ':', 's', 'v', 'd', '_', 'm', 'a', 't', 'r', 'i', 'x',
    'W', 'i', 't', 'h', 'N', 'a', 'N', 'I', 'n', 'f' };

  char_T c2_u[33];
  const mxArray *c2_y = NULL;
  for (c2_i46 = 0; c2_i46 < 33; c2_i46++) {
    c2_u[c2_i46] = c2_varargin_1[c2_i46];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 10, 0U, 1U, 0U, 2, 1, 33), FALSE);
  sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 1U, 14,
    c2_y));
}

static void c2_eml_xgesvd(SFc2_WBCLibraryInstanceStruct *chartInstance, real_T
  c2_A[72], real_T c2_U[72], real_T c2_S[6], real_T c2_V[36])
{
  int32_T c2_i47;
  real_T c2_b_A[72];
  int32_T c2_i48;
  real_T c2_s[6];
  int32_T c2_i49;
  real_T c2_e[6];
  int32_T c2_i50;
  real_T c2_work[12];
  int32_T c2_i51;
  int32_T c2_i52;
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
  int32_T c2_i53;
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
  int32_T c2_i54;
  real_T c2_d_A[72];
  int32_T c2_i55;
  real_T c2_e_A[72];
  real_T c2_t;
  int32_T c2_c_q;
  boolean_T c2_b_overflow;
  int32_T c2_ii;
  int32_T c2_b_ii;
  int32_T c2_f_b;
  int32_T c2_pmq;
  int32_T c2_i56;
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
  int32_T c2_i57;
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
  int32_T c2_i58;
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
  int32_T c2_i59;
  real_T c2_b_U[72];
  int32_T c2_i60;
  real_T c2_c_U[72];
  int32_T c2_e_q;
  boolean_T c2_h_overflow;
  int32_T c2_e_ii;
  int32_T c2_q_a;
  int32_T c2_i61;
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
  int32_T c2_i62;
  real_T c2_b_Vf[36];
  int32_T c2_i63;
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
  int32_T c2_i64;
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
  int32_T c2_i65;
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
  int32_T c2_i66;
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
  int32_T c2_i67;
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
  for (c2_i47 = 0; c2_i47 < 72; c2_i47++) {
    c2_b_A[c2_i47] = c2_A[c2_i47];
  }

  c2_eml_scalar_eg(chartInstance);
  for (c2_i48 = 0; c2_i48 < 6; c2_i48++) {
    c2_s[c2_i48] = 0.0;
  }

  for (c2_i49 = 0; c2_i49 < 6; c2_i49++) {
    c2_e[c2_i49] = 0.0;
  }

  for (c2_i50 = 0; c2_i50 < 12; c2_i50++) {
    c2_work[c2_i50] = 0.0;
  }

  for (c2_i51 = 0; c2_i51 < 72; c2_i51++) {
    c2_U[c2_i51] = 0.0;
  }

  for (c2_i52 = 0; c2_i52 < 36; c2_i52++) {
    c2_Vf[c2_i52] = 0.0;
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
      for (c2_i53 = 0; c2_i53 < 72; c2_i53++) {
        c2_c_A[c2_i53] = c2_b_A[c2_i53];
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
        c2_d_eml_xscal(chartInstance, c2_nmqp1, c2_d1, c2_b_A, c2_qq);
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
          for (c2_i54 = 0; c2_i54 < 72; c2_i54++) {
            c2_d_A[c2_i54] = c2_b_A[c2_i54];
          }

          for (c2_i55 = 0; c2_i55 < 72; c2_i55++) {
            c2_e_A[c2_i55] = c2_b_A[c2_i55];
          }

          c2_t = c2_eml_xdotc(chartInstance, c2_nmqp1, c2_d_A, c2_qq, c2_e_A,
                              c2_qjj);
          c2_t = -c2_eml_div(chartInstance, c2_t, c2_b_A
                             [(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c2_b_q), 1, 12, 1, 0) + 12 *
                               (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c2_b_q), 1, 6, 2, 0) - 1)) - 1]);
          c2_e_eml_xaxpy(chartInstance, c2_nmqp1, c2_t, c2_qq, c2_b_A, c2_qjj);
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
      for (c2_i56 = 0; c2_i56 < 6; c2_i56++) {
        c2_b_e[c2_i56] = c2_e[c2_i56];
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
        c2_e_eml_xscal(chartInstance, c2_pmq, c2_d2, c2_e, c2_qp1);
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
            for (c2_i57 = 0; c2_i57 < 72; c2_i57++) {
              c2_f_A[c2_i57] = c2_b_A[c2_i57];
            }

            c2_f_eml_xaxpy(chartInstance, c2_nmq,
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
            for (c2_i58 = 0; c2_i58 < 12; c2_i58++) {
              c2_b_work[c2_i58] = c2_work[c2_i58];
            }

            c2_g_eml_xaxpy(chartInstance, c2_nmq, c2_t, c2_b_work, c2_qp1,
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
        for (c2_i59 = 0; c2_i59 < 72; c2_i59++) {
          c2_b_U[c2_i59] = c2_U[c2_i59];
        }

        for (c2_i60 = 0; c2_i60 < 72; c2_i60++) {
          c2_c_U[c2_i60] = c2_U[c2_i60];
        }

        c2_t = c2_eml_xdotc(chartInstance, c2_nmqp1, c2_b_U, c2_qq, c2_c_U,
                            c2_qjj);
        c2_t = -c2_eml_div(chartInstance, c2_t, c2_U[_SFD_EML_ARRAY_BOUNDS_CHECK
                           ("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_qq),
                            1, 72, 1, 0) - 1]);
        c2_e_eml_xaxpy(chartInstance, c2_nmqp1, c2_t, c2_qq, c2_U, c2_qjj);
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
      c2_i61 = c2_q_a;
      c2_p_b = c2_i61;
      c2_q_b = c2_p_b;
      if (1 > c2_q_b) {
        c2_i_overflow = FALSE;
      } else {
        c2_i_overflow = (c2_q_b > 2147483646);
      }

      c2_b_check_forloop_overflow_error(chartInstance, c2_i_overflow);
      for (c2_f_ii = 1; c2_f_ii <= c2_i61; c2_f_ii++) {
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
          for (c2_i62 = 0; c2_i62 < 36; c2_i62++) {
            c2_b_Vf[c2_i62] = c2_Vf[c2_i62];
          }

          for (c2_i63 = 0; c2_i63 < 36; c2_i63++) {
            c2_c_Vf[c2_i63] = c2_Vf[c2_i63];
          }

          c2_t = c2_b_eml_xdotc(chartInstance, c2_pmq, c2_b_Vf, c2_qp1q, c2_c_Vf,
                                c2_qp1jj);
          c2_t = -c2_eml_div(chartInstance, c2_t,
                             c2_Vf[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c2_qp1q), 1, 36, 1, 0) - 1]);
          c2_h_eml_xaxpy(chartInstance, c2_pmq, c2_t, c2_qp1q, c2_Vf, c2_qp1jj);
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
        c2_i64 = 12;
        c2_d_eml_xscal(chartInstance, c2_i64, c2_r, c2_U, c2_colq);
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
        c2_f_eml_xscal(chartInstance, c2_r, c2_Vf, c2_colqp1);
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
    c2_c_eml_scalar_eg(chartInstance);
    c2_xk = c2_b_x;
    c2_yk = c2_e_y;
    c2_c_x = c2_xk;
    c2_f_y = c2_yk;
    c2_c_eml_scalar_eg(chartInstance);
    c2_maxval = muDoubleScalarMax(c2_c_x, c2_f_y);
    c2_b_varargin_1 = c2_snorm;
    c2_c_varargin_2 = c2_maxval;
    c2_d_varargin_2 = c2_b_varargin_1;
    c2_b_varargin_3 = c2_c_varargin_2;
    c2_d_x = c2_d_varargin_2;
    c2_g_y = c2_b_varargin_3;
    c2_e_x = c2_d_x;
    c2_h_y = c2_g_y;
    c2_c_eml_scalar_eg(chartInstance);
    c2_b_xk = c2_e_x;
    c2_b_yk = c2_h_y;
    c2_f_x = c2_b_xk;
    c2_i_y = c2_b_yk;
    c2_c_eml_scalar_eg(chartInstance);
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
      c2_i65 = c2_cb_a;
      c2_k_overflow = FALSE;
      c2_b_check_forloop_overflow_error(chartInstance, c2_k_overflow);
      c2_j_ii = c2_i65;
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
        c2_i66 = c2_mb_a;
        c2_i_q = c2_b_q;
        c2_nb_a = c2_i66;
        c2_hb_b = c2_i_q;
        c2_ob_a = c2_nb_a;
        c2_ib_b = c2_hb_b;
        if (c2_ob_a < c2_ib_b) {
          c2_m_overflow = FALSE;
        } else {
          c2_m_overflow = (c2_ib_b < -2147483647);
        }

        c2_b_check_forloop_overflow_error(chartInstance, c2_m_overflow);
        for (c2_k = c2_i66; c2_k >= c2_i_q; c2_k--) {
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
          c2_c_eml_xrot(chartInstance, c2_Vf, c2_colk, c2_colm, c2_b_cs, c2_b_sn);
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
          c2_d_eml_xrot(chartInstance, c2_U, c2_colk, c2_colqm1, c2_b_cs,
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
          c2_i67 = c2_dc_a;
          c2_o_overflow = FALSE;
          c2_b_check_forloop_overflow_error(chartInstance, c2_o_overflow);
          for (c2_c_ix = c2_i67; c2_c_ix < 6; c2_c_ix++) {
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
          c2_c_eml_xrot(chartInstance, c2_Vf, c2_colk, c2_colkp1, c2_b_cs,
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
            c2_d_eml_xrot(chartInstance, c2_U, c2_colk, c2_colkp1, c2_b_cs,
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
          c2_b_eml_scalar_eg(chartInstance);
          c2_d8 = -1.0;
          c2_f_eml_xscal(chartInstance, c2_d8, c2_Vf, c2_colq);
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
              c2_c_eml_xswap(chartInstance, c2_Vf, c2_colq, c2_colqp1);
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
              c2_d_eml_xswap(chartInstance, c2_U, c2_colq, c2_colqp1);
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

static real_T c2_abs(SFc2_WBCLibraryInstanceStruct *chartInstance, real_T c2_x)
{
  real_T c2_b_x;
  c2_b_x = c2_x;
  return muDoubleScalarAbs(c2_b_x);
}

static void c2_realmin(SFc2_WBCLibraryInstanceStruct *chartInstance)
{
}

static real_T c2_eml_xnrm2(SFc2_WBCLibraryInstanceStruct *chartInstance, int32_T
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

static void c2_b_check_forloop_overflow_error(SFc2_WBCLibraryInstanceStruct
  *chartInstance, boolean_T c2_overflow)
{
  int32_T c2_i68;
  static char_T c2_cv0[34] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'i', 'n', 't', '_', 'f', 'o', 'r', 'l', 'o', 'o', 'p',
    '_', 'o', 'v', 'e', 'r', 'f', 'l', 'o', 'w' };

  char_T c2_u[34];
  const mxArray *c2_y = NULL;
  int32_T c2_i69;
  static char_T c2_cv1[23] = { 'c', 'o', 'd', 'e', 'r', '.', 'i', 'n', 't', 'e',
    'r', 'n', 'a', 'l', '.', 'i', 'n', 'd', 'e', 'x', 'I', 'n', 't' };

  char_T c2_b_u[23];
  const mxArray *c2_b_y = NULL;
  if (!c2_overflow) {
  } else {
    for (c2_i68 = 0; c2_i68 < 34; c2_i68++) {
      c2_u[c2_i68] = c2_cv0[c2_i68];
    }

    c2_y = NULL;
    sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 10, 0U, 1U, 0U, 2, 1, 34),
                  FALSE);
    for (c2_i69 = 0; c2_i69 < 23; c2_i69++) {
      c2_b_u[c2_i69] = c2_cv1[c2_i69];
    }

    c2_b_y = NULL;
    sf_mex_assign(&c2_b_y, sf_mex_create("y", c2_b_u, 10, 0U, 1U, 0U, 2, 1, 23),
                  FALSE);
    sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 2U,
      14, c2_y, 14, c2_b_y));
  }
}

static real_T c2_eml_div(SFc2_WBCLibraryInstanceStruct *chartInstance, real_T
  c2_x, real_T c2_y)
{
  return c2_x / c2_y;
}

static void c2_eml_xscal(SFc2_WBCLibraryInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_a, real_T c2_x[72], int32_T c2_ix0, real_T c2_b_x[72])
{
  int32_T c2_i70;
  for (c2_i70 = 0; c2_i70 < 72; c2_i70++) {
    c2_b_x[c2_i70] = c2_x[c2_i70];
  }

  c2_d_eml_xscal(chartInstance, c2_n, c2_a, c2_b_x, c2_ix0);
}

static real_T c2_eml_xdotc(SFc2_WBCLibraryInstanceStruct *chartInstance, int32_T
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

static void c2_eml_xaxpy(SFc2_WBCLibraryInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_a, int32_T c2_ix0, real_T c2_y[72], int32_T c2_iy0, real_T
  c2_b_y[72])
{
  int32_T c2_i71;
  for (c2_i71 = 0; c2_i71 < 72; c2_i71++) {
    c2_b_y[c2_i71] = c2_y[c2_i71];
  }

  c2_e_eml_xaxpy(chartInstance, c2_n, c2_a, c2_ix0, c2_b_y, c2_iy0);
}

static real_T c2_b_eml_xnrm2(SFc2_WBCLibraryInstanceStruct *chartInstance,
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

static void c2_b_eml_xscal(SFc2_WBCLibraryInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_a, real_T c2_x[6], int32_T c2_ix0, real_T c2_b_x[6])
{
  int32_T c2_i72;
  for (c2_i72 = 0; c2_i72 < 6; c2_i72++) {
    c2_b_x[c2_i72] = c2_x[c2_i72];
  }

  c2_e_eml_xscal(chartInstance, c2_n, c2_a, c2_b_x, c2_ix0);
}

static void c2_b_eml_xaxpy(SFc2_WBCLibraryInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_a, real_T c2_x[72], int32_T c2_ix0, real_T c2_y[12], int32_T
  c2_iy0, real_T c2_b_y[12])
{
  int32_T c2_i73;
  int32_T c2_i74;
  real_T c2_b_x[72];
  for (c2_i73 = 0; c2_i73 < 12; c2_i73++) {
    c2_b_y[c2_i73] = c2_y[c2_i73];
  }

  for (c2_i74 = 0; c2_i74 < 72; c2_i74++) {
    c2_b_x[c2_i74] = c2_x[c2_i74];
  }

  c2_f_eml_xaxpy(chartInstance, c2_n, c2_a, c2_b_x, c2_ix0, c2_b_y, c2_iy0);
}

static void c2_c_eml_xaxpy(SFc2_WBCLibraryInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_a, real_T c2_x[12], int32_T c2_ix0, real_T c2_y[72], int32_T
  c2_iy0, real_T c2_b_y[72])
{
  int32_T c2_i75;
  int32_T c2_i76;
  real_T c2_b_x[12];
  for (c2_i75 = 0; c2_i75 < 72; c2_i75++) {
    c2_b_y[c2_i75] = c2_y[c2_i75];
  }

  for (c2_i76 = 0; c2_i76 < 12; c2_i76++) {
    c2_b_x[c2_i76] = c2_x[c2_i76];
  }

  c2_g_eml_xaxpy(chartInstance, c2_n, c2_a, c2_b_x, c2_ix0, c2_b_y, c2_iy0);
}

static real_T c2_b_eml_xdotc(SFc2_WBCLibraryInstanceStruct *chartInstance,
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

static void c2_d_eml_xaxpy(SFc2_WBCLibraryInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_a, int32_T c2_ix0, real_T c2_y[36], int32_T c2_iy0, real_T
  c2_b_y[36])
{
  int32_T c2_i77;
  for (c2_i77 = 0; c2_i77 < 36; c2_i77++) {
    c2_b_y[c2_i77] = c2_y[c2_i77];
  }

  c2_h_eml_xaxpy(chartInstance, c2_n, c2_a, c2_ix0, c2_b_y, c2_iy0);
}

static void c2_b_eml_scalar_eg(SFc2_WBCLibraryInstanceStruct *chartInstance)
{
}

static void c2_c_eml_xscal(SFc2_WBCLibraryInstanceStruct *chartInstance, real_T
  c2_a, real_T c2_x[36], int32_T c2_ix0, real_T c2_b_x[36])
{
  int32_T c2_i78;
  for (c2_i78 = 0; c2_i78 < 36; c2_i78++) {
    c2_b_x[c2_i78] = c2_x[c2_i78];
  }

  c2_f_eml_xscal(chartInstance, c2_a, c2_b_x, c2_ix0);
}

static void c2_eps(SFc2_WBCLibraryInstanceStruct *chartInstance)
{
}

static void c2_c_eml_scalar_eg(SFc2_WBCLibraryInstanceStruct *chartInstance)
{
}

static void c2_b_eml_error(SFc2_WBCLibraryInstanceStruct *chartInstance)
{
  int32_T c2_i79;
  static char_T c2_varargin_1[30] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A',
    'T', 'L', 'A', 'B', ':', 's', 'v', 'd', '_', 'N', 'o', 'C', 'o', 'n', 'v',
    'e', 'r', 'g', 'e', 'n', 'c', 'e' };

  char_T c2_u[30];
  const mxArray *c2_y = NULL;
  for (c2_i79 = 0; c2_i79 < 30; c2_i79++) {
    c2_u[c2_i79] = c2_varargin_1[c2_i79];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 10, 0U, 1U, 0U, 2, 1, 30), FALSE);
  sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 1U, 14,
    c2_y));
}

static real_T c2_sqrt(SFc2_WBCLibraryInstanceStruct *chartInstance, real_T c2_x)
{
  real_T c2_b_x;
  c2_b_x = c2_x;
  c2_b_sqrt(chartInstance, &c2_b_x);
  return c2_b_x;
}

static void c2_c_eml_error(SFc2_WBCLibraryInstanceStruct *chartInstance)
{
  int32_T c2_i80;
  static char_T c2_varargin_1[30] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o',
    'o', 'l', 'b', 'o', 'x', ':', 's', 'q', 'r', 't', '_', 'd', 'o', 'm', 'a',
    'i', 'n', 'E', 'r', 'r', 'o', 'r' };

  char_T c2_u[30];
  const mxArray *c2_y = NULL;
  for (c2_i80 = 0; c2_i80 < 30; c2_i80++) {
    c2_u[c2_i80] = c2_varargin_1[c2_i80];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 10, 0U, 1U, 0U, 2, 1, 30), FALSE);
  sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 1U, 14,
    c2_y));
}

static void c2_eml_xrotg(SFc2_WBCLibraryInstanceStruct *chartInstance, real_T
  c2_a, real_T c2_b, real_T *c2_b_a, real_T *c2_b_b, real_T *c2_c, real_T *c2_s)
{
  *c2_b_a = c2_a;
  *c2_b_b = c2_b;
  c2_b_eml_xrotg(chartInstance, c2_b_a, c2_b_b, c2_c, c2_s);
}

static void c2_eml_xrot(SFc2_WBCLibraryInstanceStruct *chartInstance, real_T
  c2_x[36], int32_T c2_ix0, int32_T c2_iy0, real_T c2_c, real_T c2_s, real_T
  c2_b_x[36])
{
  int32_T c2_i81;
  for (c2_i81 = 0; c2_i81 < 36; c2_i81++) {
    c2_b_x[c2_i81] = c2_x[c2_i81];
  }

  c2_c_eml_xrot(chartInstance, c2_b_x, c2_ix0, c2_iy0, c2_c, c2_s);
}

static void c2_b_eml_xrot(SFc2_WBCLibraryInstanceStruct *chartInstance, real_T
  c2_x[72], int32_T c2_ix0, int32_T c2_iy0, real_T c2_c, real_T c2_s, real_T
  c2_b_x[72])
{
  int32_T c2_i82;
  for (c2_i82 = 0; c2_i82 < 72; c2_i82++) {
    c2_b_x[c2_i82] = c2_x[c2_i82];
  }

  c2_d_eml_xrot(chartInstance, c2_b_x, c2_ix0, c2_iy0, c2_c, c2_s);
}

static void c2_eml_xswap(SFc2_WBCLibraryInstanceStruct *chartInstance, real_T
  c2_x[36], int32_T c2_ix0, int32_T c2_iy0, real_T c2_b_x[36])
{
  int32_T c2_i83;
  for (c2_i83 = 0; c2_i83 < 36; c2_i83++) {
    c2_b_x[c2_i83] = c2_x[c2_i83];
  }

  c2_c_eml_xswap(chartInstance, c2_b_x, c2_ix0, c2_iy0);
}

static void c2_b_eml_xswap(SFc2_WBCLibraryInstanceStruct *chartInstance, real_T
  c2_x[72], int32_T c2_ix0, int32_T c2_iy0, real_T c2_b_x[72])
{
  int32_T c2_i84;
  for (c2_i84 = 0; c2_i84 < 72; c2_i84++) {
    c2_b_x[c2_i84] = c2_x[c2_i84];
  }

  c2_d_eml_xswap(chartInstance, c2_b_x, c2_ix0, c2_iy0);
}

static void c2_eml_xgemm(SFc2_WBCLibraryInstanceStruct *chartInstance, int32_T
  c2_k, real_T c2_A[36], real_T c2_B[72], real_T c2_C[72], real_T c2_b_C[72])
{
  int32_T c2_i85;
  int32_T c2_i86;
  real_T c2_b_A[36];
  int32_T c2_i87;
  real_T c2_b_B[72];
  for (c2_i85 = 0; c2_i85 < 72; c2_i85++) {
    c2_b_C[c2_i85] = c2_C[c2_i85];
  }

  for (c2_i86 = 0; c2_i86 < 36; c2_i86++) {
    c2_b_A[c2_i86] = c2_A[c2_i86];
  }

  for (c2_i87 = 0; c2_i87 < 72; c2_i87++) {
    c2_b_B[c2_i87] = c2_B[c2_i87];
  }

  c2_c_eml_xgemm(chartInstance, c2_k, c2_b_A, c2_b_B, c2_b_C);
}

static void c2_d_eml_scalar_eg(SFc2_WBCLibraryInstanceStruct *chartInstance)
{
}

static void c2_b_eml_xgemm(SFc2_WBCLibraryInstanceStruct *chartInstance, real_T
  c2_A[72], real_T c2_B[300], real_T c2_C[150], real_T c2_b_C[150])
{
  int32_T c2_i88;
  int32_T c2_i89;
  real_T c2_b_A[72];
  int32_T c2_i90;
  real_T c2_b_B[300];
  for (c2_i88 = 0; c2_i88 < 150; c2_i88++) {
    c2_b_C[c2_i88] = c2_C[c2_i88];
  }

  for (c2_i89 = 0; c2_i89 < 72; c2_i89++) {
    c2_b_A[c2_i89] = c2_A[c2_i89];
  }

  for (c2_i90 = 0; c2_i90 < 300; c2_i90++) {
    c2_b_B[c2_i90] = c2_B[c2_i90];
  }

  c2_d_eml_xgemm(chartInstance, c2_b_A, c2_b_B, c2_b_C);
}

static void c2_e_eml_scalar_eg(SFc2_WBCLibraryInstanceStruct *chartInstance)
{
}

static const mxArray *c2_e_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_WBCLibraryInstanceStruct *chartInstance;
  chartInstance = (SFc2_WBCLibraryInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(int32_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static int32_T c2_d_emlrt_marshallIn(SFc2_WBCLibraryInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  int32_T c2_y;
  int32_T c2_i91;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_i91, 1, 6, 0U, 0, 0U, 0);
  c2_y = c2_i91;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_sfEvent;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  int32_T c2_y;
  SFc2_WBCLibraryInstanceStruct *chartInstance;
  chartInstance = (SFc2_WBCLibraryInstanceStruct *)chartInstanceVoid;
  c2_b_sfEvent = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_sfEvent),
    &c2_thisId);
  sf_mex_destroy(&c2_b_sfEvent);
  *(int32_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static uint8_T c2_e_emlrt_marshallIn(SFc2_WBCLibraryInstanceStruct
  *chartInstance, const mxArray *c2_b_is_active_c2_WBCLibrary, const char_T
  *c2_identifier)
{
  uint8_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c2_b_is_active_c2_WBCLibrary), &c2_thisId);
  sf_mex_destroy(&c2_b_is_active_c2_WBCLibrary);
  return c2_y;
}

static uint8_T c2_f_emlrt_marshallIn(SFc2_WBCLibraryInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  uint8_T c2_y;
  uint8_T c2_u0;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_u0, 1, 3, 0U, 0, 0U, 0);
  c2_y = c2_u0;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_d_eml_xscal(SFc2_WBCLibraryInstanceStruct *chartInstance, int32_T
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
  int32_T c2_i92;
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
  c2_i92 = c2_e_a + c2_b_b;
  c2_f_a = c2_d_ix0;
  c2_c_b = c2_i92;
  c2_g_a = c2_f_a;
  c2_d_b = c2_c_b;
  if (c2_g_a > c2_d_b) {
    c2_overflow = FALSE;
  } else {
    c2_overflow = (c2_d_b > 2147483646);
  }

  c2_b_check_forloop_overflow_error(chartInstance, c2_overflow);
  for (c2_k = c2_d_ix0; c2_k <= c2_i92; c2_k++) {
    c2_b_k = c2_k;
    c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c2_b_k), 1, 72, 1, 0) - 1] = c2_c_a * c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("",
      (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_k), 1, 72, 1, 0) - 1];
  }
}

static void c2_e_eml_xaxpy(SFc2_WBCLibraryInstanceStruct *chartInstance, int32_T
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
  int32_T c2_i93;
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
    c2_i93 = c2_f_a;
    c2_b = c2_i93;
    c2_b_b = c2_b;
    if (0 > c2_b_b) {
      c2_overflow = FALSE;
    } else {
      c2_overflow = (c2_b_b > 2147483646);
    }

    c2_b_check_forloop_overflow_error(chartInstance, c2_overflow);
    for (c2_k = 0; c2_k <= c2_i93; c2_k++) {
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

static void c2_e_eml_xscal(SFc2_WBCLibraryInstanceStruct *chartInstance, int32_T
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
  int32_T c2_i94;
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
  c2_i94 = c2_e_a + c2_b_b;
  c2_f_a = c2_d_ix0;
  c2_c_b = c2_i94;
  c2_g_a = c2_f_a;
  c2_d_b = c2_c_b;
  if (c2_g_a > c2_d_b) {
    c2_overflow = FALSE;
  } else {
    c2_overflow = (c2_d_b > 2147483646);
  }

  c2_b_check_forloop_overflow_error(chartInstance, c2_overflow);
  for (c2_k = c2_d_ix0; c2_k <= c2_i94; c2_k++) {
    c2_b_k = c2_k;
    c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c2_b_k), 1, 6, 1, 0) - 1] = c2_c_a * c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("",
      (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_k), 1, 6, 1, 0) - 1];
  }
}

static void c2_f_eml_xaxpy(SFc2_WBCLibraryInstanceStruct *chartInstance, int32_T
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
  int32_T c2_i95;
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
    c2_i95 = c2_f_a;
    c2_b = c2_i95;
    c2_b_b = c2_b;
    if (0 > c2_b_b) {
      c2_overflow = FALSE;
    } else {
      c2_overflow = (c2_b_b > 2147483646);
    }

    c2_b_check_forloop_overflow_error(chartInstance, c2_overflow);
    for (c2_k = 0; c2_k <= c2_i95; c2_k++) {
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

static void c2_g_eml_xaxpy(SFc2_WBCLibraryInstanceStruct *chartInstance, int32_T
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
  int32_T c2_i96;
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
    c2_i96 = c2_f_a;
    c2_b = c2_i96;
    c2_b_b = c2_b;
    if (0 > c2_b_b) {
      c2_overflow = FALSE;
    } else {
      c2_overflow = (c2_b_b > 2147483646);
    }

    c2_b_check_forloop_overflow_error(chartInstance, c2_overflow);
    for (c2_k = 0; c2_k <= c2_i96; c2_k++) {
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

static void c2_h_eml_xaxpy(SFc2_WBCLibraryInstanceStruct *chartInstance, int32_T
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
  int32_T c2_i97;
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
    c2_i97 = c2_f_a;
    c2_b = c2_i97;
    c2_b_b = c2_b;
    if (0 > c2_b_b) {
      c2_overflow = FALSE;
    } else {
      c2_overflow = (c2_b_b > 2147483646);
    }

    c2_b_check_forloop_overflow_error(chartInstance, c2_overflow);
    for (c2_k = 0; c2_k <= c2_i97; c2_k++) {
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

static void c2_f_eml_xscal(SFc2_WBCLibraryInstanceStruct *chartInstance, real_T
  c2_a, real_T c2_x[36], int32_T c2_ix0)
{
  real_T c2_b_a;
  int32_T c2_b_ix0;
  real_T c2_c_a;
  int32_T c2_c_ix0;
  int32_T c2_d_ix0;
  int32_T c2_d_a;
  int32_T c2_i98;
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
  c2_i98 = c2_d_a;
  c2_e_a = c2_d_ix0;
  c2_b = c2_i98;
  c2_f_a = c2_e_a;
  c2_b_b = c2_b;
  if (c2_f_a > c2_b_b) {
    c2_overflow = FALSE;
  } else {
    c2_overflow = (c2_b_b > 2147483646);
  }

  c2_b_check_forloop_overflow_error(chartInstance, c2_overflow);
  for (c2_k = c2_d_ix0; c2_k <= c2_i98; c2_k++) {
    c2_b_k = c2_k;
    c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c2_b_k), 1, 36, 1, 0) - 1] = c2_c_a * c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("",
      (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_k), 1, 36, 1, 0) - 1];
  }
}

static void c2_b_sqrt(SFc2_WBCLibraryInstanceStruct *chartInstance, real_T *c2_x)
{
  if (*c2_x < 0.0) {
    c2_c_eml_error(chartInstance);
  }

  *c2_x = muDoubleScalarSqrt(*c2_x);
}

static void c2_b_eml_xrotg(SFc2_WBCLibraryInstanceStruct *chartInstance, real_T *
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

static void c2_c_eml_xrot(SFc2_WBCLibraryInstanceStruct *chartInstance, real_T
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

static void c2_d_eml_xrot(SFc2_WBCLibraryInstanceStruct *chartInstance, real_T
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

static void c2_c_eml_xswap(SFc2_WBCLibraryInstanceStruct *chartInstance, real_T
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

static void c2_d_eml_xswap(SFc2_WBCLibraryInstanceStruct *chartInstance, real_T
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

static void c2_c_eml_xgemm(SFc2_WBCLibraryInstanceStruct *chartInstance, int32_T
  c2_k, real_T c2_A[36], real_T c2_B[72], real_T c2_C[72])
{
  int32_T c2_b_k;
  int32_T c2_c_k;
  int32_T c2_a;
  int32_T c2_km1;
  int32_T c2_cr;
  int32_T c2_b_cr;
  int32_T c2_b_a;
  int32_T c2_i99;
  int32_T c2_c_a;
  int32_T c2_i100;
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
  int32_T c2_i101;
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
  int32_T c2_i102;
  int32_T c2_k_a;
  int32_T c2_i103;
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
    c2_i99 = c2_b_a;
    c2_c_a = c2_b_cr + 6;
    c2_i100 = c2_c_a;
    c2_d_a = c2_i99;
    c2_b = c2_i100;
    c2_e_a = c2_d_a;
    c2_b_b = c2_b;
    if (c2_e_a > c2_b_b) {
      c2_overflow = FALSE;
    } else {
      c2_overflow = (c2_b_b > 2147483646);
    }

    c2_b_check_forloop_overflow_error(chartInstance, c2_overflow);
    for (c2_ic = c2_i99; c2_ic <= c2_i100; c2_ic++) {
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
    c2_i101 = c2_g_a + c2_d_b;
    c2_h_a = c2_b_br;
    c2_e_b = c2_i101;
    c2_i_a = c2_h_a;
    c2_f_b = c2_e_b;
    if (c2_i_a > c2_f_b) {
      c2_b_overflow = FALSE;
    } else {
      c2_b_overflow = (c2_f_b > 2147483635);
    }

    c2_b_check_forloop_overflow_error(chartInstance, c2_b_overflow);
    for (c2_ib = c2_b_br; c2_ib <= c2_i101; c2_ib += 12) {
      c2_b_ib = c2_ib;
      if (c2_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_b_ib), 1, 72, 1, 0) - 1] != 0.0) {
        c2_temp = c2_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c2_b_ib), 1, 72, 1, 0) - 1];
        c2_ia = c2_ar;
        c2_j_a = c2_b_cr + 1;
        c2_i102 = c2_j_a;
        c2_k_a = c2_b_cr + 6;
        c2_i103 = c2_k_a;
        c2_l_a = c2_i102;
        c2_g_b = c2_i103;
        c2_m_a = c2_l_a;
        c2_h_b = c2_g_b;
        if (c2_m_a > c2_h_b) {
          c2_c_overflow = FALSE;
        } else {
          c2_c_overflow = (c2_h_b > 2147483646);
        }

        c2_b_check_forloop_overflow_error(chartInstance, c2_c_overflow);
        for (c2_c_ic = c2_i102; c2_c_ic <= c2_i103; c2_c_ic++) {
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

static void c2_d_eml_xgemm(SFc2_WBCLibraryInstanceStruct *chartInstance, real_T
  c2_A[72], real_T c2_B[300], real_T c2_C[150])
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
  c2_m = 6;
  c2_n = 25;
  c2_k = 12;
  c2_alpha1 = 1.0;
  c2_lda = 6;
  c2_ldb = 12;
  c2_beta1 = 0.0;
  c2_ldc = 6;
  c2_TRANSB = 'N';
  c2_TRANSA = 'N';
  dgemm32(&c2_TRANSA, &c2_TRANSB, &c2_m, &c2_n, &c2_k, &c2_alpha1, &c2_A[0],
          &c2_lda, &c2_B[0], &c2_ldb, &c2_beta1, &c2_C[0], &c2_ldc);
}

static void init_dsm_address_info(SFc2_WBCLibraryInstanceStruct *chartInstance)
{
}

/* SFunction Glue Code */
void sf_c2_WBCLibrary_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(23503727U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(808174710U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(208187468U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1702090337U);
}

mxArray *sf_c2_WBCLibrary_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("YPfLMmHrLttJfs1b6XCp6C");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,2,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(12);
      pr[1] = (double)(31);
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
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,
                mxREAL));
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,1,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(6);
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
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  return(mxAutoinheritanceInfo);
}

static const mxArray *sf_get_sim_state_info_c2_WBCLibrary(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x2'type','srcId','name','auxInfo'{{M[1],M[5],T\"y\",},{M[8],M[0],T\"is_active_c2_WBCLibrary\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 2, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c2_WBCLibrary_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc2_WBCLibraryInstanceStruct *chartInstance;
    chartInstance = (SFc2_WBCLibraryInstanceStruct *) ((ChartInfoStruct *)
      (ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (_WBCLibraryMachineNumber_,
           2,
           1,
           1,
           3,
           0,
           0,
           0,
           0,
           0,
           &(chartInstance->chartNumber),
           &(chartInstance->instanceNumber),
           ssGetPath(S),
           (void *)S);
        if (chartAlreadyPresent==0) {
          /* this is the first instance */
          init_script_number_translation(_WBCLibraryMachineNumber_,
            chartInstance->chartNumber);
          sf_debug_set_chart_disable_implicit_casting(_WBCLibraryMachineNumber_,
            chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(_WBCLibraryMachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);
          _SFD_SET_DATA_PROPS(0,1,1,0,"JcIn");
          _SFD_SET_DATA_PROPS(1,2,0,1,"y");
          _SFD_SET_DATA_PROPS(2,1,1,0,"qD");
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
        _SFD_CV_INIT_EML(0,1,1,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,78);
        _SFD_TRANS_COV_WTS(0,0,0,1,0);
        if (chartAlreadyPresent==0) {
          _SFD_TRANS_COV_MAPS(0,
                              0,NULL,NULL,
                              0,NULL,NULL,
                              1,NULL,NULL,
                              0,NULL,NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 12;
          dimVector[1]= 31;
          _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_c_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 6;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)
            c2_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 25;
          _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_b_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          real_T (*c2_JcIn)[372];
          real_T (*c2_y)[6];
          real_T (*c2_qD)[25];
          c2_qD = (real_T (*)[25])ssGetInputPortSignal(chartInstance->S, 1);
          c2_y = (real_T (*)[6])ssGetOutputPortSignal(chartInstance->S, 1);
          c2_JcIn = (real_T (*)[372])ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, *c2_JcIn);
          _SFD_SET_DATA_VALUE_PTR(1U, *c2_y);
          _SFD_SET_DATA_VALUE_PTR(2U, *c2_qD);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration(_WBCLibraryMachineNumber_,
        chartInstance->chartNumber,chartInstance->instanceNumber);
    }
  }
}

static const char* sf_get_instance_specialization()
{
  return "oA4Ltog088r3mb29PrbNT";
}

static void sf_opaque_initialize_c2_WBCLibrary(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc2_WBCLibraryInstanceStruct*) chartInstanceVar
    )->S,0);
  initialize_params_c2_WBCLibrary((SFc2_WBCLibraryInstanceStruct*)
    chartInstanceVar);
  initialize_c2_WBCLibrary((SFc2_WBCLibraryInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_enable_c2_WBCLibrary(void *chartInstanceVar)
{
  enable_c2_WBCLibrary((SFc2_WBCLibraryInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c2_WBCLibrary(void *chartInstanceVar)
{
  disable_c2_WBCLibrary((SFc2_WBCLibraryInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_gateway_c2_WBCLibrary(void *chartInstanceVar)
{
  sf_c2_WBCLibrary((SFc2_WBCLibraryInstanceStruct*) chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c2_WBCLibrary(SimStruct* S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c2_WBCLibrary
    ((SFc2_WBCLibraryInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c2_WBCLibrary();/* state var info */
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

extern void sf_internal_set_sim_state_c2_WBCLibrary(SimStruct* S, const mxArray *
  st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c2_WBCLibrary();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c2_WBCLibrary((SFc2_WBCLibraryInstanceStruct*)
    chartInfo->chartInstance, mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c2_WBCLibrary(SimStruct* S)
{
  return sf_internal_get_sim_state_c2_WBCLibrary(S);
}

static void sf_opaque_set_sim_state_c2_WBCLibrary(SimStruct* S, const mxArray
  *st)
{
  sf_internal_set_sim_state_c2_WBCLibrary(S, st);
}

static void sf_opaque_terminate_c2_WBCLibrary(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc2_WBCLibraryInstanceStruct*) chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
    }

    finalize_c2_WBCLibrary((SFc2_WBCLibraryInstanceStruct*) chartInstanceVar);
    free((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }

  unload_WBCLibrary_optimization_info();
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc2_WBCLibrary((SFc2_WBCLibraryInstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c2_WBCLibrary(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c2_WBCLibrary((SFc2_WBCLibraryInstanceStruct*)
      (((ChartInfoStruct *)ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c2_WBCLibrary(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_WBCLibrary_optimization_info();
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
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,2,2);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,2,1);
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,2);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(256529023U));
  ssSetChecksum1(S,(3649851420U));
  ssSetChecksum2(S,(875177650U));
  ssSetChecksum3(S,(2261628365U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c2_WBCLibrary(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c2_WBCLibrary(SimStruct *S)
{
  SFc2_WBCLibraryInstanceStruct *chartInstance;
  chartInstance = (SFc2_WBCLibraryInstanceStruct *)malloc(sizeof
    (SFc2_WBCLibraryInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc2_WBCLibraryInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway = sf_opaque_gateway_c2_WBCLibrary;
  chartInstance->chartInfo.initializeChart = sf_opaque_initialize_c2_WBCLibrary;
  chartInstance->chartInfo.terminateChart = sf_opaque_terminate_c2_WBCLibrary;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c2_WBCLibrary;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c2_WBCLibrary;
  chartInstance->chartInfo.getSimState = sf_opaque_get_sim_state_c2_WBCLibrary;
  chartInstance->chartInfo.setSimState = sf_opaque_set_sim_state_c2_WBCLibrary;
  chartInstance->chartInfo.getSimStateInfo = sf_get_sim_state_info_c2_WBCLibrary;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c2_WBCLibrary;
  chartInstance->chartInfo.mdlStart = mdlStart_c2_WBCLibrary;
  chartInstance->chartInfo.mdlSetWorkWidths = mdlSetWorkWidths_c2_WBCLibrary;
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

void c2_WBCLibrary_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c2_WBCLibrary(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c2_WBCLibrary(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c2_WBCLibrary(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c2_WBCLibrary_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
