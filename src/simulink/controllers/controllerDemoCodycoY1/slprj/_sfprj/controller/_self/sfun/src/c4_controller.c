/* Include files */

#include "blascompat32.h"
#include "controller_sfun.h"
#include "c4_controller.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "controller_sfun_debug_macros.h"

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static const char * c4_debug_family_names[8] = { "res", "nargin", "nargout",
  "umin", "umax", "u", "tol", "inRange" };

/* Function Declarations */
static void initialize_c4_controller(SFc4_controllerInstanceStruct
  *chartInstance);
static void initialize_params_c4_controller(SFc4_controllerInstanceStruct
  *chartInstance);
static void enable_c4_controller(SFc4_controllerInstanceStruct *chartInstance);
static void disable_c4_controller(SFc4_controllerInstanceStruct *chartInstance);
static void c4_update_debugger_state_c4_controller(SFc4_controllerInstanceStruct
  *chartInstance);
static const mxArray *get_sim_state_c4_controller(SFc4_controllerInstanceStruct *
  chartInstance);
static void set_sim_state_c4_controller(SFc4_controllerInstanceStruct
  *chartInstance, const mxArray *c4_st);
static void finalize_c4_controller(SFc4_controllerInstanceStruct *chartInstance);
static void sf_c4_controller(SFc4_controllerInstanceStruct *chartInstance);
static void initSimStructsc4_controller(SFc4_controllerInstanceStruct
  *chartInstance);
static void init_script_number_translation(uint32_T c4_machineNumber, uint32_T
  c4_chartNumber);
static const mxArray *c4_sf_marshallOut(void *chartInstanceVoid, void *c4_inData);
static real_T c4_emlrt_marshallIn(SFc4_controllerInstanceStruct *chartInstance,
  const mxArray *c4_inRange, const char_T *c4_identifier);
static real_T c4_b_emlrt_marshallIn(SFc4_controllerInstanceStruct *chartInstance,
  const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId);
static void c4_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData);
static const mxArray *c4_b_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData);
static const mxArray *c4_c_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData);
static void c4_c_emlrt_marshallIn(SFc4_controllerInstanceStruct *chartInstance,
  const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId, boolean_T c4_y[25]);
static void c4_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData);
static void c4_check_forloop_overflow_error(SFc4_controllerInstanceStruct
  *chartInstance);
static const mxArray *c4_d_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData);
static int32_T c4_d_emlrt_marshallIn(SFc4_controllerInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId);
static void c4_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData);
static uint8_T c4_e_emlrt_marshallIn(SFc4_controllerInstanceStruct
  *chartInstance, const mxArray *c4_b_is_active_c4_controller, const char_T
  *c4_identifier);
static uint8_T c4_f_emlrt_marshallIn(SFc4_controllerInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId);
static void init_dsm_address_info(SFc4_controllerInstanceStruct *chartInstance);

/* Function Definitions */
static void initialize_c4_controller(SFc4_controllerInstanceStruct
  *chartInstance)
{
  chartInstance->c4_sfEvent = CALL_EVENT;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c4_is_active_c4_controller = 0U;
}

static void initialize_params_c4_controller(SFc4_controllerInstanceStruct
  *chartInstance)
{
}

static void enable_c4_controller(SFc4_controllerInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c4_controller(SFc4_controllerInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c4_update_debugger_state_c4_controller(SFc4_controllerInstanceStruct
  *chartInstance)
{
}

static const mxArray *get_sim_state_c4_controller(SFc4_controllerInstanceStruct *
  chartInstance)
{
  const mxArray *c4_st;
  const mxArray *c4_y = NULL;
  real_T c4_hoistedGlobal;
  real_T c4_u;
  const mxArray *c4_b_y = NULL;
  uint8_T c4_b_hoistedGlobal;
  uint8_T c4_b_u;
  const mxArray *c4_c_y = NULL;
  real_T *c4_inRange;
  c4_inRange = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c4_st = NULL;
  c4_st = NULL;
  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_createcellarray(2), FALSE);
  c4_hoistedGlobal = *c4_inRange;
  c4_u = c4_hoistedGlobal;
  c4_b_y = NULL;
  sf_mex_assign(&c4_b_y, sf_mex_create("y", &c4_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c4_y, 0, c4_b_y);
  c4_b_hoistedGlobal = chartInstance->c4_is_active_c4_controller;
  c4_b_u = c4_b_hoistedGlobal;
  c4_c_y = NULL;
  sf_mex_assign(&c4_c_y, sf_mex_create("y", &c4_b_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c4_y, 1, c4_c_y);
  sf_mex_assign(&c4_st, c4_y, FALSE);
  return c4_st;
}

static void set_sim_state_c4_controller(SFc4_controllerInstanceStruct
  *chartInstance, const mxArray *c4_st)
{
  const mxArray *c4_u;
  real_T *c4_inRange;
  c4_inRange = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c4_doneDoubleBufferReInit = TRUE;
  c4_u = sf_mex_dup(c4_st);
  *c4_inRange = c4_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell
    (c4_u, 0)), "inRange");
  chartInstance->c4_is_active_c4_controller = c4_e_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c4_u, 1)),
     "is_active_c4_controller");
  sf_mex_destroy(&c4_u);
  c4_update_debugger_state_c4_controller(chartInstance);
  sf_mex_destroy(&c4_st);
}

static void finalize_c4_controller(SFc4_controllerInstanceStruct *chartInstance)
{
}

static void sf_c4_controller(SFc4_controllerInstanceStruct *chartInstance)
{
  int32_T c4_i0;
  int32_T c4_i1;
  int32_T c4_i2;
  real_T c4_hoistedGlobal;
  int32_T c4_i3;
  real_T c4_umin[25];
  int32_T c4_i4;
  real_T c4_umax[25];
  int32_T c4_i5;
  real_T c4_u[25];
  real_T c4_tol;
  uint32_T c4_debug_family_var_map[8];
  boolean_T c4_res[25];
  real_T c4_b_res;
  real_T c4_nargin = 4.0;
  real_T c4_nargout = 1.0;
  real_T c4_inRange;
  int32_T c4_i6;
  boolean_T c4_x[25];
  int32_T c4_i7;
  boolean_T c4_bv0[25];
  int32_T c4_i8;
  int32_T c4_i9;
  int32_T c4_k;
  int32_T c4_b_k;
  int32_T c4_i10;
  int32_T c4_i11;
  int32_T c4_i12;
  boolean_T c4_b_u[25];
  const mxArray *c4_y = NULL;
  real_T *c4_b_inRange;
  real_T *c4_b_tol;
  real_T (*c4_c_u)[25];
  real_T (*c4_b_umax)[25];
  real_T (*c4_b_umin)[25];
  c4_b_tol = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
  c4_b_inRange = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c4_c_u = (real_T (*)[25])ssGetInputPortSignal(chartInstance->S, 2);
  c4_b_umax = (real_T (*)[25])ssGetInputPortSignal(chartInstance->S, 1);
  c4_b_umin = (real_T (*)[25])ssGetInputPortSignal(chartInstance->S, 0);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 3U, chartInstance->c4_sfEvent);
  for (c4_i0 = 0; c4_i0 < 25; c4_i0++) {
    _SFD_DATA_RANGE_CHECK((*c4_b_umin)[c4_i0], 0U);
  }

  for (c4_i1 = 0; c4_i1 < 25; c4_i1++) {
    _SFD_DATA_RANGE_CHECK((*c4_b_umax)[c4_i1], 1U);
  }

  for (c4_i2 = 0; c4_i2 < 25; c4_i2++) {
    _SFD_DATA_RANGE_CHECK((*c4_c_u)[c4_i2], 2U);
  }

  _SFD_DATA_RANGE_CHECK(*c4_b_inRange, 3U);
  _SFD_DATA_RANGE_CHECK(*c4_b_tol, 4U);
  chartInstance->c4_sfEvent = CALL_EVENT;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 3U, chartInstance->c4_sfEvent);
  c4_hoistedGlobal = *c4_b_tol;
  for (c4_i3 = 0; c4_i3 < 25; c4_i3++) {
    c4_umin[c4_i3] = (*c4_b_umin)[c4_i3];
  }

  for (c4_i4 = 0; c4_i4 < 25; c4_i4++) {
    c4_umax[c4_i4] = (*c4_b_umax)[c4_i4];
  }

  for (c4_i5 = 0; c4_i5 < 25; c4_i5++) {
    c4_u[c4_i5] = (*c4_c_u)[c4_i5];
  }

  c4_tol = c4_hoistedGlobal;
  sf_debug_symbol_scope_push_eml(0U, 8U, 9U, c4_debug_family_names,
    c4_debug_family_var_map);
  sf_debug_symbol_scope_add_eml_importable(c4_res, MAX_uint32_T,
    c4_c_sf_marshallOut, c4_b_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c4_b_res, MAX_uint32_T,
    c4_sf_marshallOut, c4_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c4_nargin, 1U, c4_sf_marshallOut,
    c4_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c4_nargout, 2U, c4_sf_marshallOut,
    c4_sf_marshallIn);
  sf_debug_symbol_scope_add_eml(c4_umin, 3U, c4_b_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(c4_umax, 4U, c4_b_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(c4_u, 5U, c4_b_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(&c4_tol, 6U, c4_sf_marshallOut);
  sf_debug_symbol_scope_add_eml_importable(&c4_inRange, 7U, c4_sf_marshallOut,
    c4_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 4);
  for (c4_i6 = 0; c4_i6 < 25; c4_i6++) {
    c4_x[c4_i6] = (c4_u[c4_i6] < c4_umin[c4_i6] + c4_tol);
  }

  for (c4_i7 = 0; c4_i7 < 25; c4_i7++) {
    c4_bv0[c4_i7] = (c4_u[c4_i7] > c4_umax[c4_i7] - c4_tol);
  }

  for (c4_i8 = 0; c4_i8 < 25; c4_i8++) {
    c4_res[c4_i8] = (c4_x[c4_i8] || c4_bv0[c4_i8]);
  }

  sf_debug_symbol_switch(0U, 0U);
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 5);
  for (c4_i9 = 0; c4_i9 < 25; c4_i9++) {
    c4_x[c4_i9] = c4_res[c4_i9];
  }

  c4_b_res = (real_T)c4_x[0];
  c4_check_forloop_overflow_error(chartInstance);
  for (c4_k = 2; c4_k < 26; c4_k++) {
    c4_b_k = c4_k;
    c4_b_res += (real_T)c4_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
      _SFD_INTEGER_CHECK("", (real_T)c4_b_k), 1, 25, 1, 0) - 1];
  }

  sf_debug_symbol_switch(0U, 1U);
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 6);
  if (CV_EML_IF(0, 1, 0, c4_b_res == 0.0)) {
    _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 7);
    c4_inRange = 1.0;
  } else {
    _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 9);
    c4_inRange = 0.0;
    _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 10);
    for (c4_i10 = 0; c4_i10 < 25; c4_i10++) {
      c4_x[c4_i10] = (c4_u[c4_i10] < c4_umin[c4_i10] + c4_tol);
    }

    for (c4_i11 = 0; c4_i11 < 25; c4_i11++) {
      c4_bv0[c4_i11] = (c4_u[c4_i11] > c4_umax[c4_i11] - c4_tol);
    }

    sf_mex_printf("%s =\\n", "ans");
    for (c4_i12 = 0; c4_i12 < 25; c4_i12++) {
      c4_b_u[c4_i12] = (c4_x[c4_i12] || c4_bv0[c4_i12]);
    }

    c4_y = NULL;
    sf_mex_assign(&c4_y, sf_mex_create("y", c4_b_u, 11, 0U, 1U, 0U, 1, 25),
                  FALSE);
    sf_mex_call_debug("disp", 0U, 1U, 14, c4_y);
  }

  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, -10);
  sf_debug_symbol_scope_pop();
  *c4_b_inRange = c4_inRange;
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 3U, chartInstance->c4_sfEvent);
  sf_debug_check_for_state_inconsistency(_controllerMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void initSimStructsc4_controller(SFc4_controllerInstanceStruct
  *chartInstance)
{
}

static void init_script_number_translation(uint32_T c4_machineNumber, uint32_T
  c4_chartNumber)
{
}

static const mxArray *c4_sf_marshallOut(void *chartInstanceVoid, void *c4_inData)
{
  const mxArray *c4_mxArrayOutData = NULL;
  real_T c4_u;
  const mxArray *c4_y = NULL;
  SFc4_controllerInstanceStruct *chartInstance;
  chartInstance = (SFc4_controllerInstanceStruct *)chartInstanceVoid;
  c4_mxArrayOutData = NULL;
  c4_u = *(real_T *)c4_inData;
  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_create("y", &c4_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c4_mxArrayOutData, c4_y, FALSE);
  return c4_mxArrayOutData;
}

static real_T c4_emlrt_marshallIn(SFc4_controllerInstanceStruct *chartInstance,
  const mxArray *c4_inRange, const char_T *c4_identifier)
{
  real_T c4_y;
  emlrtMsgIdentifier c4_thisId;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_y = c4_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_inRange), &c4_thisId);
  sf_mex_destroy(&c4_inRange);
  return c4_y;
}

static real_T c4_b_emlrt_marshallIn(SFc4_controllerInstanceStruct *chartInstance,
  const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId)
{
  real_T c4_y;
  real_T c4_d0;
  sf_mex_import(c4_parentId, sf_mex_dup(c4_u), &c4_d0, 1, 0, 0U, 0, 0U, 0);
  c4_y = c4_d0;
  sf_mex_destroy(&c4_u);
  return c4_y;
}

static void c4_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData)
{
  const mxArray *c4_inRange;
  const char_T *c4_identifier;
  emlrtMsgIdentifier c4_thisId;
  real_T c4_y;
  SFc4_controllerInstanceStruct *chartInstance;
  chartInstance = (SFc4_controllerInstanceStruct *)chartInstanceVoid;
  c4_inRange = sf_mex_dup(c4_mxArrayInData);
  c4_identifier = c4_varName;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_y = c4_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_inRange), &c4_thisId);
  sf_mex_destroy(&c4_inRange);
  *(real_T *)c4_outData = c4_y;
  sf_mex_destroy(&c4_mxArrayInData);
}

static const mxArray *c4_b_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData)
{
  const mxArray *c4_mxArrayOutData = NULL;
  int32_T c4_i13;
  real_T c4_b_inData[25];
  int32_T c4_i14;
  real_T c4_u[25];
  const mxArray *c4_y = NULL;
  SFc4_controllerInstanceStruct *chartInstance;
  chartInstance = (SFc4_controllerInstanceStruct *)chartInstanceVoid;
  c4_mxArrayOutData = NULL;
  for (c4_i13 = 0; c4_i13 < 25; c4_i13++) {
    c4_b_inData[c4_i13] = (*(real_T (*)[25])c4_inData)[c4_i13];
  }

  for (c4_i14 = 0; c4_i14 < 25; c4_i14++) {
    c4_u[c4_i14] = c4_b_inData[c4_i14];
  }

  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_create("y", c4_u, 0, 0U, 1U, 0U, 1, 25), FALSE);
  sf_mex_assign(&c4_mxArrayOutData, c4_y, FALSE);
  return c4_mxArrayOutData;
}

static const mxArray *c4_c_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData)
{
  const mxArray *c4_mxArrayOutData = NULL;
  int32_T c4_i15;
  boolean_T c4_b_inData[25];
  int32_T c4_i16;
  boolean_T c4_u[25];
  const mxArray *c4_y = NULL;
  SFc4_controllerInstanceStruct *chartInstance;
  chartInstance = (SFc4_controllerInstanceStruct *)chartInstanceVoid;
  c4_mxArrayOutData = NULL;
  for (c4_i15 = 0; c4_i15 < 25; c4_i15++) {
    c4_b_inData[c4_i15] = (*(boolean_T (*)[25])c4_inData)[c4_i15];
  }

  for (c4_i16 = 0; c4_i16 < 25; c4_i16++) {
    c4_u[c4_i16] = c4_b_inData[c4_i16];
  }

  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_create("y", c4_u, 11, 0U, 1U, 0U, 1, 25), FALSE);
  sf_mex_assign(&c4_mxArrayOutData, c4_y, FALSE);
  return c4_mxArrayOutData;
}

static void c4_c_emlrt_marshallIn(SFc4_controllerInstanceStruct *chartInstance,
  const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId, boolean_T c4_y[25])
{
  boolean_T c4_bv1[25];
  int32_T c4_i17;
  sf_mex_import(c4_parentId, sf_mex_dup(c4_u), c4_bv1, 1, 11, 0U, 1, 0U, 1, 25);
  for (c4_i17 = 0; c4_i17 < 25; c4_i17++) {
    c4_y[c4_i17] = c4_bv1[c4_i17];
  }

  sf_mex_destroy(&c4_u);
}

static void c4_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData)
{
  const mxArray *c4_res;
  const char_T *c4_identifier;
  emlrtMsgIdentifier c4_thisId;
  boolean_T c4_y[25];
  int32_T c4_i18;
  SFc4_controllerInstanceStruct *chartInstance;
  chartInstance = (SFc4_controllerInstanceStruct *)chartInstanceVoid;
  c4_res = sf_mex_dup(c4_mxArrayInData);
  c4_identifier = c4_varName;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_res), &c4_thisId, c4_y);
  sf_mex_destroy(&c4_res);
  for (c4_i18 = 0; c4_i18 < 25; c4_i18++) {
    (*(boolean_T (*)[25])c4_outData)[c4_i18] = c4_y[c4_i18];
  }

  sf_mex_destroy(&c4_mxArrayInData);
}

const mxArray *sf_c4_controller_get_eml_resolved_functions_info(void)
{
  const mxArray *c4_nameCaptureInfo;
  c4_ResolvedFunctionInfo c4_info[8];
  c4_ResolvedFunctionInfo (*c4_b_info)[8];
  const mxArray *c4_m0 = NULL;
  int32_T c4_i19;
  c4_ResolvedFunctionInfo *c4_r0;
  c4_nameCaptureInfo = NULL;
  c4_nameCaptureInfo = NULL;
  c4_b_info = (c4_ResolvedFunctionInfo (*)[8])c4_info;
  (*c4_b_info)[0].context = "";
  (*c4_b_info)[0].name = "sum";
  (*c4_b_info)[0].dominantType = "logical";
  (*c4_b_info)[0].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m";
  (*c4_b_info)[0].fileTimeLo = 1314736612U;
  (*c4_b_info)[0].fileTimeHi = 0U;
  (*c4_b_info)[0].mFileTimeLo = 0U;
  (*c4_b_info)[0].mFileTimeHi = 0U;
  (*c4_b_info)[1].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m";
  (*c4_b_info)[1].name = "isequal";
  (*c4_b_info)[1].dominantType = "double";
  (*c4_b_info)[1].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isequal.m";
  (*c4_b_info)[1].fileTimeLo = 1286818758U;
  (*c4_b_info)[1].fileTimeHi = 0U;
  (*c4_b_info)[1].mFileTimeLo = 0U;
  (*c4_b_info)[1].mFileTimeHi = 0U;
  (*c4_b_info)[2].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isequal.m";
  (*c4_b_info)[2].name = "eml_isequal_core";
  (*c4_b_info)[2].dominantType = "double";
  (*c4_b_info)[2].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isequal_core.m";
  (*c4_b_info)[2].fileTimeLo = 1286818786U;
  (*c4_b_info)[2].fileTimeHi = 0U;
  (*c4_b_info)[2].mFileTimeLo = 0U;
  (*c4_b_info)[2].mFileTimeHi = 0U;
  (*c4_b_info)[3].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m";
  (*c4_b_info)[3].name = "eml_const_nonsingleton_dim";
  (*c4_b_info)[3].dominantType = "logical";
  (*c4_b_info)[3].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_const_nonsingleton_dim.m";
  (*c4_b_info)[3].fileTimeLo = 1286818696U;
  (*c4_b_info)[3].fileTimeHi = 0U;
  (*c4_b_info)[3].mFileTimeLo = 0U;
  (*c4_b_info)[3].mFileTimeHi = 0U;
  (*c4_b_info)[4].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m";
  (*c4_b_info)[4].name = "eml_scalar_eg";
  (*c4_b_info)[4].dominantType = "double";
  (*c4_b_info)[4].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  (*c4_b_info)[4].fileTimeLo = 1286818796U;
  (*c4_b_info)[4].fileTimeHi = 0U;
  (*c4_b_info)[4].mFileTimeLo = 0U;
  (*c4_b_info)[4].mFileTimeHi = 0U;
  (*c4_b_info)[5].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m";
  (*c4_b_info)[5].name = "eml_index_class";
  (*c4_b_info)[5].dominantType = "";
  (*c4_b_info)[5].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  (*c4_b_info)[5].fileTimeLo = 1323170578U;
  (*c4_b_info)[5].fileTimeHi = 0U;
  (*c4_b_info)[5].mFileTimeLo = 0U;
  (*c4_b_info)[5].mFileTimeHi = 0U;
  (*c4_b_info)[6].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m";
  (*c4_b_info)[6].name = "eml_int_forloop_overflow_check";
  (*c4_b_info)[6].dominantType = "";
  (*c4_b_info)[6].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  (*c4_b_info)[6].fileTimeLo = 1332168672U;
  (*c4_b_info)[6].fileTimeHi = 0U;
  (*c4_b_info)[6].mFileTimeLo = 0U;
  (*c4_b_info)[6].mFileTimeHi = 0U;
  (*c4_b_info)[7].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper";
  (*c4_b_info)[7].name = "intmax";
  (*c4_b_info)[7].dominantType = "char";
  (*c4_b_info)[7].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  (*c4_b_info)[7].fileTimeLo = 1311255316U;
  (*c4_b_info)[7].fileTimeHi = 0U;
  (*c4_b_info)[7].mFileTimeLo = 0U;
  (*c4_b_info)[7].mFileTimeHi = 0U;
  sf_mex_assign(&c4_m0, sf_mex_createstruct("nameCaptureInfo", 1, 8), FALSE);
  for (c4_i19 = 0; c4_i19 < 8; c4_i19++) {
    c4_r0 = &c4_info[c4_i19];
    sf_mex_addfield(c4_m0, sf_mex_create("nameCaptureInfo", c4_r0->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c4_r0->context)), "context", "nameCaptureInfo",
                    c4_i19);
    sf_mex_addfield(c4_m0, sf_mex_create("nameCaptureInfo", c4_r0->name, 15, 0U,
      0U, 0U, 2, 1, strlen(c4_r0->name)), "name", "nameCaptureInfo", c4_i19);
    sf_mex_addfield(c4_m0, sf_mex_create("nameCaptureInfo", c4_r0->dominantType,
      15, 0U, 0U, 0U, 2, 1, strlen(c4_r0->dominantType)), "dominantType",
                    "nameCaptureInfo", c4_i19);
    sf_mex_addfield(c4_m0, sf_mex_create("nameCaptureInfo", c4_r0->resolved, 15,
      0U, 0U, 0U, 2, 1, strlen(c4_r0->resolved)), "resolved", "nameCaptureInfo",
                    c4_i19);
    sf_mex_addfield(c4_m0, sf_mex_create("nameCaptureInfo", &c4_r0->fileTimeLo,
      7, 0U, 0U, 0U, 0), "fileTimeLo", "nameCaptureInfo", c4_i19);
    sf_mex_addfield(c4_m0, sf_mex_create("nameCaptureInfo", &c4_r0->fileTimeHi,
      7, 0U, 0U, 0U, 0), "fileTimeHi", "nameCaptureInfo", c4_i19);
    sf_mex_addfield(c4_m0, sf_mex_create("nameCaptureInfo", &c4_r0->mFileTimeLo,
      7, 0U, 0U, 0U, 0), "mFileTimeLo", "nameCaptureInfo", c4_i19);
    sf_mex_addfield(c4_m0, sf_mex_create("nameCaptureInfo", &c4_r0->mFileTimeHi,
      7, 0U, 0U, 0U, 0), "mFileTimeHi", "nameCaptureInfo", c4_i19);
  }

  sf_mex_assign(&c4_nameCaptureInfo, c4_m0, FALSE);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c4_nameCaptureInfo);
  return c4_nameCaptureInfo;
}

static void c4_check_forloop_overflow_error(SFc4_controllerInstanceStruct
  *chartInstance)
{
}

static const mxArray *c4_d_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData)
{
  const mxArray *c4_mxArrayOutData = NULL;
  int32_T c4_u;
  const mxArray *c4_y = NULL;
  SFc4_controllerInstanceStruct *chartInstance;
  chartInstance = (SFc4_controllerInstanceStruct *)chartInstanceVoid;
  c4_mxArrayOutData = NULL;
  c4_u = *(int32_T *)c4_inData;
  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_create("y", &c4_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c4_mxArrayOutData, c4_y, FALSE);
  return c4_mxArrayOutData;
}

static int32_T c4_d_emlrt_marshallIn(SFc4_controllerInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId)
{
  int32_T c4_y;
  int32_T c4_i20;
  sf_mex_import(c4_parentId, sf_mex_dup(c4_u), &c4_i20, 1, 6, 0U, 0, 0U, 0);
  c4_y = c4_i20;
  sf_mex_destroy(&c4_u);
  return c4_y;
}

static void c4_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData)
{
  const mxArray *c4_b_sfEvent;
  const char_T *c4_identifier;
  emlrtMsgIdentifier c4_thisId;
  int32_T c4_y;
  SFc4_controllerInstanceStruct *chartInstance;
  chartInstance = (SFc4_controllerInstanceStruct *)chartInstanceVoid;
  c4_b_sfEvent = sf_mex_dup(c4_mxArrayInData);
  c4_identifier = c4_varName;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_y = c4_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_b_sfEvent),
    &c4_thisId);
  sf_mex_destroy(&c4_b_sfEvent);
  *(int32_T *)c4_outData = c4_y;
  sf_mex_destroy(&c4_mxArrayInData);
}

static uint8_T c4_e_emlrt_marshallIn(SFc4_controllerInstanceStruct
  *chartInstance, const mxArray *c4_b_is_active_c4_controller, const char_T
  *c4_identifier)
{
  uint8_T c4_y;
  emlrtMsgIdentifier c4_thisId;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_y = c4_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c4_b_is_active_c4_controller), &c4_thisId);
  sf_mex_destroy(&c4_b_is_active_c4_controller);
  return c4_y;
}

static uint8_T c4_f_emlrt_marshallIn(SFc4_controllerInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId)
{
  uint8_T c4_y;
  uint8_T c4_u0;
  sf_mex_import(c4_parentId, sf_mex_dup(c4_u), &c4_u0, 1, 3, 0U, 0, 0U, 0);
  c4_y = c4_u0;
  sf_mex_destroy(&c4_u);
  return c4_y;
}

static void init_dsm_address_info(SFc4_controllerInstanceStruct *chartInstance)
{
}

/* SFunction Glue Code */
void sf_c4_controller_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1352948681U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3559787227U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(285377812U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1107450000U);
}

mxArray *sf_c4_controller_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("fCcfaito3rvqWHppkWoEcG");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,4,3,dataFields);

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
      pr[0] = (double)(1);
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
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  return(mxAutoinheritanceInfo);
}

static const mxArray *sf_get_sim_state_info_c4_controller(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x2'type','srcId','name','auxInfo'{{M[1],M[5],T\"inRange\",},{M[8],M[0],T\"is_active_c4_controller\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 2, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c4_controller_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc4_controllerInstanceStruct *chartInstance;
    chartInstance = (SFc4_controllerInstanceStruct *) ((ChartInfoStruct *)
      (ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (_controllerMachineNumber_,
           4,
           1,
           1,
           5,
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
          init_script_number_translation(_controllerMachineNumber_,
            chartInstance->chartNumber);
          sf_debug_set_chart_disable_implicit_casting(_controllerMachineNumber_,
            chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(_controllerMachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);
          _SFD_SET_DATA_PROPS(0,1,1,0,"umin");
          _SFD_SET_DATA_PROPS(1,1,1,0,"umax");
          _SFD_SET_DATA_PROPS(2,1,1,0,"u");
          _SFD_SET_DATA_PROPS(3,2,0,1,"inRange");
          _SFD_SET_DATA_PROPS(4,1,1,0,"tol");
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
        _SFD_CV_INIT_EML(0,1,1,1,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,204);
        _SFD_CV_INIT_EML_IF(0,1,0,116,125,143,204);
        _SFD_TRANS_COV_WTS(0,0,0,1,0);
        if (chartAlreadyPresent==0) {
          _SFD_TRANS_COV_MAPS(0,
                              0,NULL,NULL,
                              0,NULL,NULL,
                              1,NULL,NULL,
                              0,NULL,NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 25;
          _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c4_b_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 25;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c4_b_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 25;
          _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c4_b_sf_marshallOut,(MexInFcnForType)NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c4_sf_marshallOut,(MexInFcnForType)c4_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c4_sf_marshallOut,(MexInFcnForType)NULL);

        {
          real_T *c4_inRange;
          real_T *c4_tol;
          real_T (*c4_umin)[25];
          real_T (*c4_umax)[25];
          real_T (*c4_u)[25];
          c4_tol = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
          c4_inRange = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
          c4_u = (real_T (*)[25])ssGetInputPortSignal(chartInstance->S, 2);
          c4_umax = (real_T (*)[25])ssGetInputPortSignal(chartInstance->S, 1);
          c4_umin = (real_T (*)[25])ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, *c4_umin);
          _SFD_SET_DATA_VALUE_PTR(1U, *c4_umax);
          _SFD_SET_DATA_VALUE_PTR(2U, *c4_u);
          _SFD_SET_DATA_VALUE_PTR(3U, c4_inRange);
          _SFD_SET_DATA_VALUE_PTR(4U, c4_tol);
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
  return "9YtyKjPyEFzvjjYTTTeLyD";
}

static void sf_opaque_initialize_c4_controller(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc4_controllerInstanceStruct*) chartInstanceVar
    )->S,0);
  initialize_params_c4_controller((SFc4_controllerInstanceStruct*)
    chartInstanceVar);
  initialize_c4_controller((SFc4_controllerInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_enable_c4_controller(void *chartInstanceVar)
{
  enable_c4_controller((SFc4_controllerInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c4_controller(void *chartInstanceVar)
{
  disable_c4_controller((SFc4_controllerInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_gateway_c4_controller(void *chartInstanceVar)
{
  sf_c4_controller((SFc4_controllerInstanceStruct*) chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c4_controller(SimStruct* S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c4_controller
    ((SFc4_controllerInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c4_controller();/* state var info */
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

extern void sf_internal_set_sim_state_c4_controller(SimStruct* S, const mxArray *
  st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c4_controller();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c4_controller((SFc4_controllerInstanceStruct*)
    chartInfo->chartInstance, mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c4_controller(SimStruct* S)
{
  return sf_internal_get_sim_state_c4_controller(S);
}

static void sf_opaque_set_sim_state_c4_controller(SimStruct* S, const mxArray
  *st)
{
  sf_internal_set_sim_state_c4_controller(S, st);
}

static void sf_opaque_terminate_c4_controller(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc4_controllerInstanceStruct*) chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
    }

    finalize_c4_controller((SFc4_controllerInstanceStruct*) chartInstanceVar);
    free((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }

  unload_controller_optimization_info();
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc4_controller((SFc4_controllerInstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c4_controller(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c4_controller((SFc4_controllerInstanceStruct*)
      (((ChartInfoStruct *)ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c4_controller(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_controller_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(S,sf_get_instance_specialization(),infoStruct,
      4);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(S,sf_get_instance_specialization(),
                infoStruct,4,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop(S,
      sf_get_instance_specialization(),infoStruct,4,
      "gatewayCannotBeInlinedMultipleTimes"));
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,4,4);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,4,1);
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,4);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(4041150313U));
  ssSetChecksum1(S,(3707478353U));
  ssSetChecksum2(S,(3846941900U));
  ssSetChecksum3(S,(844137156U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c4_controller(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c4_controller(SimStruct *S)
{
  SFc4_controllerInstanceStruct *chartInstance;
  chartInstance = (SFc4_controllerInstanceStruct *)malloc(sizeof
    (SFc4_controllerInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc4_controllerInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway = sf_opaque_gateway_c4_controller;
  chartInstance->chartInfo.initializeChart = sf_opaque_initialize_c4_controller;
  chartInstance->chartInfo.terminateChart = sf_opaque_terminate_c4_controller;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c4_controller;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c4_controller;
  chartInstance->chartInfo.getSimState = sf_opaque_get_sim_state_c4_controller;
  chartInstance->chartInfo.setSimState = sf_opaque_set_sim_state_c4_controller;
  chartInstance->chartInfo.getSimStateInfo = sf_get_sim_state_info_c4_controller;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c4_controller;
  chartInstance->chartInfo.mdlStart = mdlStart_c4_controller;
  chartInstance->chartInfo.mdlSetWorkWidths = mdlSetWorkWidths_c4_controller;
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

void c4_controller_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c4_controller(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c4_controller(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c4_controller(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c4_controller_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
