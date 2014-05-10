/* Include files */

#include "blascompat32.h"
#include "controller_sfun.h"
#include "c3_controller.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "controller_sfun_debug_macros.h"

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static const char * c3_debug_family_names[15] = { "qFin", "a", "nargin",
  "nargout", "demoState", "q_AuxIn", "toIn", "q_o", "q", "qFinHands", "DT", "t",
  "to", "qDes", "q_AuxOut" };

/* Function Declarations */
static void initialize_c3_controller(SFc3_controllerInstanceStruct
  *chartInstance);
static void initialize_params_c3_controller(SFc3_controllerInstanceStruct
  *chartInstance);
static void enable_c3_controller(SFc3_controllerInstanceStruct *chartInstance);
static void disable_c3_controller(SFc3_controllerInstanceStruct *chartInstance);
static void c3_update_debugger_state_c3_controller(SFc3_controllerInstanceStruct
  *chartInstance);
static const mxArray *get_sim_state_c3_controller(SFc3_controllerInstanceStruct *
  chartInstance);
static void set_sim_state_c3_controller(SFc3_controllerInstanceStruct
  *chartInstance, const mxArray *c3_st);
static void finalize_c3_controller(SFc3_controllerInstanceStruct *chartInstance);
static void sf_c3_controller(SFc3_controllerInstanceStruct *chartInstance);
static void initSimStructsc3_controller(SFc3_controllerInstanceStruct
  *chartInstance);
static void init_script_number_translation(uint32_T c3_machineNumber, uint32_T
  c3_chartNumber);
static const mxArray *c3_sf_marshallOut(void *chartInstanceVoid, void *c3_inData);
static void c3_emlrt_marshallIn(SFc3_controllerInstanceStruct *chartInstance,
  const mxArray *c3_q_AuxOut, const char_T *c3_identifier, real_T c3_y[25]);
static void c3_b_emlrt_marshallIn(SFc3_controllerInstanceStruct *chartInstance,
  const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId, real_T c3_y[25]);
static void c3_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static const mxArray *c3_b_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static real_T c3_c_emlrt_marshallIn(SFc3_controllerInstanceStruct *chartInstance,
  const mxArray *c3_to, const char_T *c3_identifier);
static real_T c3_d_emlrt_marshallIn(SFc3_controllerInstanceStruct *chartInstance,
  const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId);
static void c3_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static const mxArray *c3_c_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static const mxArray *c3_d_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static int32_T c3_e_emlrt_marshallIn(SFc3_controllerInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId);
static void c3_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static uint8_T c3_f_emlrt_marshallIn(SFc3_controllerInstanceStruct
  *chartInstance, const mxArray *c3_b_is_active_c3_controller, const char_T
  *c3_identifier);
static uint8_T c3_g_emlrt_marshallIn(SFc3_controllerInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId);
static void init_dsm_address_info(SFc3_controllerInstanceStruct *chartInstance);

/* Function Definitions */
static void initialize_c3_controller(SFc3_controllerInstanceStruct
  *chartInstance)
{
  chartInstance->c3_sfEvent = CALL_EVENT;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c3_is_active_c3_controller = 0U;
}

static void initialize_params_c3_controller(SFc3_controllerInstanceStruct
  *chartInstance)
{
}

static void enable_c3_controller(SFc3_controllerInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c3_controller(SFc3_controllerInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c3_update_debugger_state_c3_controller(SFc3_controllerInstanceStruct
  *chartInstance)
{
}

static const mxArray *get_sim_state_c3_controller(SFc3_controllerInstanceStruct *
  chartInstance)
{
  const mxArray *c3_st;
  const mxArray *c3_y = NULL;
  int32_T c3_i0;
  real_T c3_u[25];
  const mxArray *c3_b_y = NULL;
  int32_T c3_i1;
  real_T c3_b_u[25];
  const mxArray *c3_c_y = NULL;
  real_T c3_hoistedGlobal;
  real_T c3_c_u;
  const mxArray *c3_d_y = NULL;
  uint8_T c3_b_hoistedGlobal;
  uint8_T c3_d_u;
  const mxArray *c3_e_y = NULL;
  real_T *c3_to;
  real_T (*c3_q_AuxOut)[25];
  real_T (*c3_qDes)[25];
  c3_q_AuxOut = (real_T (*)[25])ssGetOutputPortSignal(chartInstance->S, 3);
  c3_qDes = (real_T (*)[25])ssGetOutputPortSignal(chartInstance->S, 2);
  c3_to = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c3_st = NULL;
  c3_st = NULL;
  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_createcellarray(4), FALSE);
  for (c3_i0 = 0; c3_i0 < 25; c3_i0++) {
    c3_u[c3_i0] = (*c3_qDes)[c3_i0];
  }

  c3_b_y = NULL;
  sf_mex_assign(&c3_b_y, sf_mex_create("y", c3_u, 0, 0U, 1U, 0U, 1, 25), FALSE);
  sf_mex_setcell(c3_y, 0, c3_b_y);
  for (c3_i1 = 0; c3_i1 < 25; c3_i1++) {
    c3_b_u[c3_i1] = (*c3_q_AuxOut)[c3_i1];
  }

  c3_c_y = NULL;
  sf_mex_assign(&c3_c_y, sf_mex_create("y", c3_b_u, 0, 0U, 1U, 0U, 1, 25), FALSE);
  sf_mex_setcell(c3_y, 1, c3_c_y);
  c3_hoistedGlobal = *c3_to;
  c3_c_u = c3_hoistedGlobal;
  c3_d_y = NULL;
  sf_mex_assign(&c3_d_y, sf_mex_create("y", &c3_c_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c3_y, 2, c3_d_y);
  c3_b_hoistedGlobal = chartInstance->c3_is_active_c3_controller;
  c3_d_u = c3_b_hoistedGlobal;
  c3_e_y = NULL;
  sf_mex_assign(&c3_e_y, sf_mex_create("y", &c3_d_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c3_y, 3, c3_e_y);
  sf_mex_assign(&c3_st, c3_y, FALSE);
  return c3_st;
}

static void set_sim_state_c3_controller(SFc3_controllerInstanceStruct
  *chartInstance, const mxArray *c3_st)
{
  const mxArray *c3_u;
  real_T c3_dv0[25];
  int32_T c3_i2;
  real_T c3_dv1[25];
  int32_T c3_i3;
  real_T *c3_to;
  real_T (*c3_qDes)[25];
  real_T (*c3_q_AuxOut)[25];
  c3_q_AuxOut = (real_T (*)[25])ssGetOutputPortSignal(chartInstance->S, 3);
  c3_qDes = (real_T (*)[25])ssGetOutputPortSignal(chartInstance->S, 2);
  c3_to = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c3_doneDoubleBufferReInit = TRUE;
  c3_u = sf_mex_dup(c3_st);
  c3_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c3_u, 0)), "qDes",
                      c3_dv0);
  for (c3_i2 = 0; c3_i2 < 25; c3_i2++) {
    (*c3_qDes)[c3_i2] = c3_dv0[c3_i2];
  }

  c3_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c3_u, 1)),
                      "q_AuxOut", c3_dv1);
  for (c3_i3 = 0; c3_i3 < 25; c3_i3++) {
    (*c3_q_AuxOut)[c3_i3] = c3_dv1[c3_i3];
  }

  *c3_to = c3_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c3_u,
    2)), "to");
  chartInstance->c3_is_active_c3_controller = c3_f_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c3_u, 3)),
     "is_active_c3_controller");
  sf_mex_destroy(&c3_u);
  c3_update_debugger_state_c3_controller(chartInstance);
  sf_mex_destroy(&c3_st);
}

static void finalize_c3_controller(SFc3_controllerInstanceStruct *chartInstance)
{
}

static void sf_c3_controller(SFc3_controllerInstanceStruct *chartInstance)
{
  int32_T c3_i4;
  int32_T c3_i5;
  int32_T c3_i6;
  int32_T c3_i7;
  int32_T c3_i8;
  int32_T c3_i9;
  real_T c3_hoistedGlobal;
  real_T c3_b_hoistedGlobal;
  real_T c3_c_hoistedGlobal;
  real_T c3_d_hoistedGlobal;
  real_T c3_demoState;
  int32_T c3_i10;
  real_T c3_q_AuxIn[25];
  real_T c3_toIn;
  int32_T c3_i11;
  real_T c3_q_o[25];
  int32_T c3_i12;
  real_T c3_q[25];
  int32_T c3_i13;
  real_T c3_qFinHands[10];
  real_T c3_DT;
  real_T c3_t;
  uint32_T c3_debug_family_var_map[15];
  real_T c3_qFin[25];
  real_T c3_a;
  real_T c3_nargin = 8.0;
  real_T c3_nargout = 3.0;
  real_T c3_to;
  real_T c3_qDes[25];
  real_T c3_q_AuxOut[25];
  int32_T c3_i14;
  int32_T c3_i15;
  int32_T c3_i16;
  int32_T c3_i17;
  int32_T c3_i18;
  int32_T c3_i19;
  int32_T c3_i20;
  int32_T c3_i21;
  int32_T c3_i22;
  real_T c3_A;
  real_T c3_B;
  real_T c3_x;
  real_T c3_y;
  real_T c3_b_x;
  real_T c3_b_y;
  real_T c3_b_a;
  int32_T c3_i23;
  real_T c3_b[25];
  int32_T c3_i24;
  int32_T c3_i25;
  int32_T c3_i26;
  int32_T c3_i27;
  int32_T c3_i28;
  int32_T c3_i29;
  real_T c3_b_A;
  real_T c3_b_B;
  real_T c3_c_x;
  real_T c3_c_y;
  real_T c3_d_x;
  real_T c3_d_y;
  real_T c3_c_a;
  int32_T c3_i30;
  int32_T c3_i31;
  int32_T c3_i32;
  int32_T c3_i33;
  int32_T c3_i34;
  int32_T c3_i35;
  int32_T c3_i36;
  int32_T c3_i37;
  real_T c3_c_A;
  real_T c3_c_B;
  real_T c3_e_x;
  real_T c3_e_y;
  real_T c3_f_x;
  real_T c3_f_y;
  real_T c3_d_a;
  int32_T c3_i38;
  int32_T c3_i39;
  int32_T c3_i40;
  int32_T c3_i41;
  int32_T c3_i42;
  int32_T c3_i43;
  int32_T c3_i44;
  int32_T c3_i45;
  real_T *c3_b_to;
  real_T *c3_b_t;
  real_T *c3_b_DT;
  real_T *c3_b_toIn;
  real_T *c3_b_demoState;
  real_T (*c3_b_qDes)[25];
  real_T (*c3_b_q_AuxOut)[25];
  real_T (*c3_b_qFinHands)[10];
  real_T (*c3_b_q)[25];
  real_T (*c3_b_q_o)[25];
  real_T (*c3_b_q_AuxIn)[25];
  c3_b_q_AuxOut = (real_T (*)[25])ssGetOutputPortSignal(chartInstance->S, 3);
  c3_b_t = (real_T *)ssGetInputPortSignal(chartInstance->S, 7);
  c3_b_DT = (real_T *)ssGetInputPortSignal(chartInstance->S, 6);
  c3_b_qDes = (real_T (*)[25])ssGetOutputPortSignal(chartInstance->S, 2);
  c3_b_qFinHands = (real_T (*)[10])ssGetInputPortSignal(chartInstance->S, 5);
  c3_b_q = (real_T (*)[25])ssGetInputPortSignal(chartInstance->S, 4);
  c3_b_q_o = (real_T (*)[25])ssGetInputPortSignal(chartInstance->S, 3);
  c3_b_to = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c3_b_toIn = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
  c3_b_q_AuxIn = (real_T (*)[25])ssGetInputPortSignal(chartInstance->S, 1);
  c3_b_demoState = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 2U, chartInstance->c3_sfEvent);
  _SFD_DATA_RANGE_CHECK(*c3_b_demoState, 0U);
  for (c3_i4 = 0; c3_i4 < 25; c3_i4++) {
    _SFD_DATA_RANGE_CHECK((*c3_b_q_AuxIn)[c3_i4], 1U);
  }

  _SFD_DATA_RANGE_CHECK(*c3_b_toIn, 2U);
  _SFD_DATA_RANGE_CHECK(*c3_b_to, 3U);
  for (c3_i5 = 0; c3_i5 < 25; c3_i5++) {
    _SFD_DATA_RANGE_CHECK((*c3_b_q_o)[c3_i5], 4U);
  }

  for (c3_i6 = 0; c3_i6 < 25; c3_i6++) {
    _SFD_DATA_RANGE_CHECK((*c3_b_q)[c3_i6], 5U);
  }

  for (c3_i7 = 0; c3_i7 < 10; c3_i7++) {
    _SFD_DATA_RANGE_CHECK((*c3_b_qFinHands)[c3_i7], 6U);
  }

  for (c3_i8 = 0; c3_i8 < 25; c3_i8++) {
    _SFD_DATA_RANGE_CHECK((*c3_b_qDes)[c3_i8], 7U);
  }

  _SFD_DATA_RANGE_CHECK(*c3_b_DT, 8U);
  _SFD_DATA_RANGE_CHECK(*c3_b_t, 9U);
  for (c3_i9 = 0; c3_i9 < 25; c3_i9++) {
    _SFD_DATA_RANGE_CHECK((*c3_b_q_AuxOut)[c3_i9], 10U);
  }

  chartInstance->c3_sfEvent = CALL_EVENT;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 2U, chartInstance->c3_sfEvent);
  c3_hoistedGlobal = *c3_b_demoState;
  c3_b_hoistedGlobal = *c3_b_toIn;
  c3_c_hoistedGlobal = *c3_b_DT;
  c3_d_hoistedGlobal = *c3_b_t;
  c3_demoState = c3_hoistedGlobal;
  for (c3_i10 = 0; c3_i10 < 25; c3_i10++) {
    c3_q_AuxIn[c3_i10] = (*c3_b_q_AuxIn)[c3_i10];
  }

  c3_toIn = c3_b_hoistedGlobal;
  for (c3_i11 = 0; c3_i11 < 25; c3_i11++) {
    c3_q_o[c3_i11] = (*c3_b_q_o)[c3_i11];
  }

  for (c3_i12 = 0; c3_i12 < 25; c3_i12++) {
    c3_q[c3_i12] = (*c3_b_q)[c3_i12];
  }

  for (c3_i13 = 0; c3_i13 < 10; c3_i13++) {
    c3_qFinHands[c3_i13] = (*c3_b_qFinHands)[c3_i13];
  }

  c3_DT = c3_c_hoistedGlobal;
  c3_t = c3_d_hoistedGlobal;
  sf_debug_symbol_scope_push_eml(0U, 15U, 15U, c3_debug_family_names,
    c3_debug_family_var_map);
  sf_debug_symbol_scope_add_eml_importable(c3_qFin, 0U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c3_a, 1U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c3_nargin, 2U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c3_nargout, 3U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  sf_debug_symbol_scope_add_eml(&c3_demoState, 4U, c3_b_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(c3_q_AuxIn, 5U, c3_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(&c3_toIn, 6U, c3_b_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(c3_q_o, 7U, c3_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(c3_q, 8U, c3_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(c3_qFinHands, 9U, c3_c_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(&c3_DT, 10U, c3_b_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(&c3_t, 11U, c3_b_sf_marshallOut);
  sf_debug_symbol_scope_add_eml_importable(&c3_to, 12U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c3_qDes, 13U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c3_q_AuxOut, 14U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 4);
  for (c3_i14 = 0; c3_i14 < 25; c3_i14++) {
    c3_q_AuxOut[c3_i14] = c3_q[c3_i14];
  }

  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 5);
  for (c3_i15 = 0; c3_i15 < 25; c3_i15++) {
    c3_qDes[c3_i15] = c3_q_AuxIn[c3_i15];
  }

  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 6);
  c3_to = c3_toIn;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 7);
  if (CV_EML_IF(0, 1, 0, c3_demoState == 0.0)) {
    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 8);
    c3_to = c3_t;
    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 9);
    for (c3_i16 = 0; c3_i16 < 25; c3_i16++) {
      c3_qDes[c3_i16] = c3_q_AuxIn[c3_i16];
    }

    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 10);
    for (c3_i17 = 0; c3_i17 < 25; c3_i17++) {
      c3_q_AuxOut[c3_i17] = c3_q_AuxIn[c3_i17];
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 13);
  if (CV_EML_IF(0, 1, 1, c3_demoState == 1.0)) {
    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 14);
    for (c3_i18 = 0; c3_i18 < 25; c3_i18++) {
      c3_qFin[c3_i18] = c3_q_o[c3_i18];
    }

    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 15);
    c3_i19 = 0;
    for (c3_i20 = 0; c3_i20 < 5; c3_i20++) {
      c3_qFin[c3_i20 + 3] = c3_qFinHands[c3_i19];
      c3_i19 += 2;
    }

    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 16);
    c3_i21 = 0;
    for (c3_i22 = 0; c3_i22 < 5; c3_i22++) {
      c3_qFin[c3_i22 + 8] = c3_qFinHands[c3_i21 + 1];
      c3_i21 += 2;
    }

    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 17);
    c3_A = c3_t - c3_toIn;
    c3_B = c3_DT;
    c3_x = c3_A;
    c3_y = c3_B;
    c3_b_x = c3_x;
    c3_b_y = c3_y;
    c3_a = c3_b_x / c3_b_y;
    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 18);
    if (CV_EML_IF(0, 1, 2, c3_a > 1.0)) {
      _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 19);
      c3_a = 1.0;
    }

    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 21);
    c3_b_a = c3_a;
    for (c3_i23 = 0; c3_i23 < 25; c3_i23++) {
      c3_b[c3_i23] = c3_qFin[c3_i23] - c3_q_o[c3_i23];
    }

    for (c3_i24 = 0; c3_i24 < 25; c3_i24++) {
      c3_b[c3_i24] *= c3_b_a;
    }

    for (c3_i25 = 0; c3_i25 < 25; c3_i25++) {
      c3_qDes[c3_i25] = c3_q_o[c3_i25] + c3_b[c3_i25];
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 24);
  if (CV_EML_IF(0, 1, 3, c3_demoState == 2.0)) {
    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 25);
    for (c3_i26 = 0; c3_i26 < 25; c3_i26++) {
      c3_qFin[c3_i26] = c3_q_o[c3_i26];
    }

    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 26);
    c3_i27 = 0;
    for (c3_i28 = 0; c3_i28 < 5; c3_i28++) {
      c3_qFin[c3_i28 + 8] = c3_qFinHands[c3_i27 + 1];
      c3_i27 += 2;
    }

    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 27);
    for (c3_i29 = 0; c3_i29 < 5; c3_i29++) {
      c3_q_AuxOut[c3_i29 + 3] = c3_q_AuxIn[c3_i29 + 3];
    }

    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 29);
    c3_b_A = c3_t - c3_toIn;
    c3_b_B = c3_DT;
    c3_c_x = c3_b_A;
    c3_c_y = c3_b_B;
    c3_d_x = c3_c_x;
    c3_d_y = c3_c_y;
    c3_a = c3_d_x / c3_d_y;
    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 30);
    if (CV_EML_IF(0, 1, 4, c3_a > 1.0)) {
      _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 31);
      c3_a = 1.0;
    }

    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 33);
    c3_c_a = c3_a;
    for (c3_i30 = 0; c3_i30 < 25; c3_i30++) {
      c3_b[c3_i30] = c3_qFin[c3_i30] - c3_q_o[c3_i30];
    }

    for (c3_i31 = 0; c3_i31 < 25; c3_i31++) {
      c3_b[c3_i31] *= c3_c_a;
    }

    for (c3_i32 = 0; c3_i32 < 25; c3_i32++) {
      c3_qDes[c3_i32] = c3_q_o[c3_i32] + c3_b[c3_i32];
    }

    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 34);
    for (c3_i33 = 0; c3_i33 < 5; c3_i33++) {
      c3_qDes[c3_i33 + 3] = c3_q_AuxIn[c3_i33 + 3];
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 37);
  if (CV_EML_IF(0, 1, 5, c3_demoState == 3.0)) {
    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 38);
    for (c3_i34 = 0; c3_i34 < 25; c3_i34++) {
      c3_qFin[c3_i34] = c3_q_o[c3_i34];
    }

    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 39);
    c3_i35 = 0;
    for (c3_i36 = 0; c3_i36 < 5; c3_i36++) {
      c3_qFin[c3_i36 + 3] = c3_qFinHands[c3_i35];
      c3_i35 += 2;
    }

    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 40);
    for (c3_i37 = 0; c3_i37 < 5; c3_i37++) {
      c3_q_AuxOut[c3_i37 + 8] = c3_q_AuxIn[c3_i37 + 8];
    }

    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 42);
    c3_c_A = c3_t - c3_toIn;
    c3_c_B = c3_DT;
    c3_e_x = c3_c_A;
    c3_e_y = c3_c_B;
    c3_f_x = c3_e_x;
    c3_f_y = c3_e_y;
    c3_a = c3_f_x / c3_f_y;
    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 43);
    if (CV_EML_IF(0, 1, 6, c3_a > 1.0)) {
      _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 44);
      c3_a = 1.0;
    }

    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 46);
    c3_d_a = c3_a;
    for (c3_i38 = 0; c3_i38 < 25; c3_i38++) {
      c3_b[c3_i38] = c3_qFin[c3_i38] - c3_q_o[c3_i38];
    }

    for (c3_i39 = 0; c3_i39 < 25; c3_i39++) {
      c3_b[c3_i39] *= c3_d_a;
    }

    for (c3_i40 = 0; c3_i40 < 25; c3_i40++) {
      c3_qDes[c3_i40] = c3_q_o[c3_i40] + c3_b[c3_i40];
    }

    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 47);
    for (c3_i41 = 0; c3_i41 < 5; c3_i41++) {
      c3_qDes[c3_i41 + 8] = c3_q_AuxIn[c3_i41 + 8];
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 51);
  if (CV_EML_IF(0, 1, 7, c3_demoState == 4.0)) {
    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 52);
    for (c3_i42 = 0; c3_i42 < 10; c3_i42++) {
      c3_q_AuxOut[c3_i42 + 3] = c3_q_AuxIn[c3_i42 + 3];
    }

    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 53);
    for (c3_i43 = 0; c3_i43 < 10; c3_i43++) {
      c3_qDes[c3_i43 + 3] = c3_q_AuxIn[c3_i43 + 3];
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, -53);
  sf_debug_symbol_scope_pop();
  *c3_b_to = c3_to;
  for (c3_i44 = 0; c3_i44 < 25; c3_i44++) {
    (*c3_b_qDes)[c3_i44] = c3_qDes[c3_i44];
  }

  for (c3_i45 = 0; c3_i45 < 25; c3_i45++) {
    (*c3_b_q_AuxOut)[c3_i45] = c3_q_AuxOut[c3_i45];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 2U, chartInstance->c3_sfEvent);
  sf_debug_check_for_state_inconsistency(_controllerMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void initSimStructsc3_controller(SFc3_controllerInstanceStruct
  *chartInstance)
{
}

static void init_script_number_translation(uint32_T c3_machineNumber, uint32_T
  c3_chartNumber)
{
}

static const mxArray *c3_sf_marshallOut(void *chartInstanceVoid, void *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  int32_T c3_i46;
  real_T c3_b_inData[25];
  int32_T c3_i47;
  real_T c3_u[25];
  const mxArray *c3_y = NULL;
  SFc3_controllerInstanceStruct *chartInstance;
  chartInstance = (SFc3_controllerInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  for (c3_i46 = 0; c3_i46 < 25; c3_i46++) {
    c3_b_inData[c3_i46] = (*(real_T (*)[25])c3_inData)[c3_i46];
  }

  for (c3_i47 = 0; c3_i47 < 25; c3_i47++) {
    c3_u[c3_i47] = c3_b_inData[c3_i47];
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 0, 0U, 1U, 0U, 1, 25), FALSE);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, FALSE);
  return c3_mxArrayOutData;
}

static void c3_emlrt_marshallIn(SFc3_controllerInstanceStruct *chartInstance,
  const mxArray *c3_q_AuxOut, const char_T *c3_identifier, real_T c3_y[25])
{
  emlrtMsgIdentifier c3_thisId;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_q_AuxOut), &c3_thisId, c3_y);
  sf_mex_destroy(&c3_q_AuxOut);
}

static void c3_b_emlrt_marshallIn(SFc3_controllerInstanceStruct *chartInstance,
  const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId, real_T c3_y[25])
{
  real_T c3_dv2[25];
  int32_T c3_i48;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), c3_dv2, 1, 0, 0U, 1, 0U, 1, 25);
  for (c3_i48 = 0; c3_i48 < 25; c3_i48++) {
    c3_y[c3_i48] = c3_dv2[c3_i48];
  }

  sf_mex_destroy(&c3_u);
}

static void c3_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_q_AuxOut;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  real_T c3_y[25];
  int32_T c3_i49;
  SFc3_controllerInstanceStruct *chartInstance;
  chartInstance = (SFc3_controllerInstanceStruct *)chartInstanceVoid;
  c3_q_AuxOut = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_q_AuxOut), &c3_thisId, c3_y);
  sf_mex_destroy(&c3_q_AuxOut);
  for (c3_i49 = 0; c3_i49 < 25; c3_i49++) {
    (*(real_T (*)[25])c3_outData)[c3_i49] = c3_y[c3_i49];
  }

  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_b_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  real_T c3_u;
  const mxArray *c3_y = NULL;
  SFc3_controllerInstanceStruct *chartInstance;
  chartInstance = (SFc3_controllerInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_u = *(real_T *)c3_inData;
  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", &c3_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, FALSE);
  return c3_mxArrayOutData;
}

static real_T c3_c_emlrt_marshallIn(SFc3_controllerInstanceStruct *chartInstance,
  const mxArray *c3_to, const char_T *c3_identifier)
{
  real_T c3_y;
  emlrtMsgIdentifier c3_thisId;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_y = c3_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_to), &c3_thisId);
  sf_mex_destroy(&c3_to);
  return c3_y;
}

static real_T c3_d_emlrt_marshallIn(SFc3_controllerInstanceStruct *chartInstance,
  const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId)
{
  real_T c3_y;
  real_T c3_d0;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), &c3_d0, 1, 0, 0U, 0, 0U, 0);
  c3_y = c3_d0;
  sf_mex_destroy(&c3_u);
  return c3_y;
}

static void c3_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_to;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  real_T c3_y;
  SFc3_controllerInstanceStruct *chartInstance;
  chartInstance = (SFc3_controllerInstanceStruct *)chartInstanceVoid;
  c3_to = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_y = c3_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_to), &c3_thisId);
  sf_mex_destroy(&c3_to);
  *(real_T *)c3_outData = c3_y;
  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_c_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  int32_T c3_i50;
  int32_T c3_i51;
  int32_T c3_i52;
  real_T c3_b_inData[10];
  int32_T c3_i53;
  int32_T c3_i54;
  int32_T c3_i55;
  real_T c3_u[10];
  const mxArray *c3_y = NULL;
  SFc3_controllerInstanceStruct *chartInstance;
  chartInstance = (SFc3_controllerInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_i50 = 0;
  for (c3_i51 = 0; c3_i51 < 5; c3_i51++) {
    for (c3_i52 = 0; c3_i52 < 2; c3_i52++) {
      c3_b_inData[c3_i52 + c3_i50] = (*(real_T (*)[10])c3_inData)[c3_i52 +
        c3_i50];
    }

    c3_i50 += 2;
  }

  c3_i53 = 0;
  for (c3_i54 = 0; c3_i54 < 5; c3_i54++) {
    for (c3_i55 = 0; c3_i55 < 2; c3_i55++) {
      c3_u[c3_i55 + c3_i53] = c3_b_inData[c3_i55 + c3_i53];
    }

    c3_i53 += 2;
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 0, 0U, 1U, 0U, 2, 2, 5), FALSE);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, FALSE);
  return c3_mxArrayOutData;
}

const mxArray *sf_c3_controller_get_eml_resolved_functions_info(void)
{
  const mxArray *c3_nameCaptureInfo;
  c3_ResolvedFunctionInfo c3_info[4];
  c3_ResolvedFunctionInfo (*c3_b_info)[4];
  const mxArray *c3_m0 = NULL;
  int32_T c3_i56;
  c3_ResolvedFunctionInfo *c3_r0;
  c3_nameCaptureInfo = NULL;
  c3_nameCaptureInfo = NULL;
  c3_b_info = (c3_ResolvedFunctionInfo (*)[4])c3_info;
  (*c3_b_info)[0].context = "";
  (*c3_b_info)[0].name = "mrdivide";
  (*c3_b_info)[0].dominantType = "double";
  (*c3_b_info)[0].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  (*c3_b_info)[0].fileTimeLo = 1342810944U;
  (*c3_b_info)[0].fileTimeHi = 0U;
  (*c3_b_info)[0].mFileTimeLo = 1319729966U;
  (*c3_b_info)[0].mFileTimeHi = 0U;
  (*c3_b_info)[1].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  (*c3_b_info)[1].name = "rdivide";
  (*c3_b_info)[1].dominantType = "double";
  (*c3_b_info)[1].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  (*c3_b_info)[1].fileTimeLo = 1286818844U;
  (*c3_b_info)[1].fileTimeHi = 0U;
  (*c3_b_info)[1].mFileTimeLo = 0U;
  (*c3_b_info)[1].mFileTimeHi = 0U;
  (*c3_b_info)[2].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  (*c3_b_info)[2].name = "eml_div";
  (*c3_b_info)[2].dominantType = "double";
  (*c3_b_info)[2].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  (*c3_b_info)[2].fileTimeLo = 1313347810U;
  (*c3_b_info)[2].fileTimeHi = 0U;
  (*c3_b_info)[2].mFileTimeLo = 0U;
  (*c3_b_info)[2].mFileTimeHi = 0U;
  (*c3_b_info)[3].context = "";
  (*c3_b_info)[3].name = "mtimes";
  (*c3_b_info)[3].dominantType = "double";
  (*c3_b_info)[3].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  (*c3_b_info)[3].fileTimeLo = 1289519692U;
  (*c3_b_info)[3].fileTimeHi = 0U;
  (*c3_b_info)[3].mFileTimeLo = 0U;
  (*c3_b_info)[3].mFileTimeHi = 0U;
  sf_mex_assign(&c3_m0, sf_mex_createstruct("nameCaptureInfo", 1, 4), FALSE);
  for (c3_i56 = 0; c3_i56 < 4; c3_i56++) {
    c3_r0 = &c3_info[c3_i56];
    sf_mex_addfield(c3_m0, sf_mex_create("nameCaptureInfo", c3_r0->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c3_r0->context)), "context", "nameCaptureInfo",
                    c3_i56);
    sf_mex_addfield(c3_m0, sf_mex_create("nameCaptureInfo", c3_r0->name, 15, 0U,
      0U, 0U, 2, 1, strlen(c3_r0->name)), "name", "nameCaptureInfo", c3_i56);
    sf_mex_addfield(c3_m0, sf_mex_create("nameCaptureInfo", c3_r0->dominantType,
      15, 0U, 0U, 0U, 2, 1, strlen(c3_r0->dominantType)), "dominantType",
                    "nameCaptureInfo", c3_i56);
    sf_mex_addfield(c3_m0, sf_mex_create("nameCaptureInfo", c3_r0->resolved, 15,
      0U, 0U, 0U, 2, 1, strlen(c3_r0->resolved)), "resolved", "nameCaptureInfo",
                    c3_i56);
    sf_mex_addfield(c3_m0, sf_mex_create("nameCaptureInfo", &c3_r0->fileTimeLo,
      7, 0U, 0U, 0U, 0), "fileTimeLo", "nameCaptureInfo", c3_i56);
    sf_mex_addfield(c3_m0, sf_mex_create("nameCaptureInfo", &c3_r0->fileTimeHi,
      7, 0U, 0U, 0U, 0), "fileTimeHi", "nameCaptureInfo", c3_i56);
    sf_mex_addfield(c3_m0, sf_mex_create("nameCaptureInfo", &c3_r0->mFileTimeLo,
      7, 0U, 0U, 0U, 0), "mFileTimeLo", "nameCaptureInfo", c3_i56);
    sf_mex_addfield(c3_m0, sf_mex_create("nameCaptureInfo", &c3_r0->mFileTimeHi,
      7, 0U, 0U, 0U, 0), "mFileTimeHi", "nameCaptureInfo", c3_i56);
  }

  sf_mex_assign(&c3_nameCaptureInfo, c3_m0, FALSE);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c3_nameCaptureInfo);
  return c3_nameCaptureInfo;
}

static const mxArray *c3_d_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  int32_T c3_u;
  const mxArray *c3_y = NULL;
  SFc3_controllerInstanceStruct *chartInstance;
  chartInstance = (SFc3_controllerInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_u = *(int32_T *)c3_inData;
  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", &c3_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, FALSE);
  return c3_mxArrayOutData;
}

static int32_T c3_e_emlrt_marshallIn(SFc3_controllerInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId)
{
  int32_T c3_y;
  int32_T c3_i57;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), &c3_i57, 1, 6, 0U, 0, 0U, 0);
  c3_y = c3_i57;
  sf_mex_destroy(&c3_u);
  return c3_y;
}

static void c3_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_b_sfEvent;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  int32_T c3_y;
  SFc3_controllerInstanceStruct *chartInstance;
  chartInstance = (SFc3_controllerInstanceStruct *)chartInstanceVoid;
  c3_b_sfEvent = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_y = c3_e_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_b_sfEvent),
    &c3_thisId);
  sf_mex_destroy(&c3_b_sfEvent);
  *(int32_T *)c3_outData = c3_y;
  sf_mex_destroy(&c3_mxArrayInData);
}

static uint8_T c3_f_emlrt_marshallIn(SFc3_controllerInstanceStruct
  *chartInstance, const mxArray *c3_b_is_active_c3_controller, const char_T
  *c3_identifier)
{
  uint8_T c3_y;
  emlrtMsgIdentifier c3_thisId;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_y = c3_g_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c3_b_is_active_c3_controller), &c3_thisId);
  sf_mex_destroy(&c3_b_is_active_c3_controller);
  return c3_y;
}

static uint8_T c3_g_emlrt_marshallIn(SFc3_controllerInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId)
{
  uint8_T c3_y;
  uint8_T c3_u0;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), &c3_u0, 1, 3, 0U, 0, 0U, 0);
  c3_y = c3_u0;
  sf_mex_destroy(&c3_u);
  return c3_y;
}

static void init_dsm_address_info(SFc3_controllerInstanceStruct *chartInstance)
{
}

/* SFunction Glue Code */
void sf_c3_controller_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(939543390U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(1247530234U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(2556432072U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(395376349U);
}

mxArray *sf_c3_controller_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("bjdnLG5WLv12EDFucOAMtD");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,8,3,dataFields);

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

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(25);
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
      pr[0] = (double)(25);
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
      pr[0] = (double)(2);
      pr[1] = (double)(5);
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
      pr[0] = (double)(1);
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
      pr[0] = (double)(1);
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
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  return(mxAutoinheritanceInfo);
}

static const mxArray *sf_get_sim_state_info_c3_controller(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x4'type','srcId','name','auxInfo'{{M[1],M[5],T\"qDes\",},{M[1],M[21],T\"q_AuxOut\",},{M[1],M[13],T\"to\",},{M[8],M[0],T\"is_active_c3_controller\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 4, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c3_controller_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc3_controllerInstanceStruct *chartInstance;
    chartInstance = (SFc3_controllerInstanceStruct *) ((ChartInfoStruct *)
      (ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (_controllerMachineNumber_,
           3,
           1,
           1,
           11,
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
          _SFD_SET_DATA_PROPS(0,1,1,0,"demoState");
          _SFD_SET_DATA_PROPS(1,1,1,0,"q_AuxIn");
          _SFD_SET_DATA_PROPS(2,1,1,0,"toIn");
          _SFD_SET_DATA_PROPS(3,2,0,1,"to");
          _SFD_SET_DATA_PROPS(4,1,1,0,"q_o");
          _SFD_SET_DATA_PROPS(5,1,1,0,"q");
          _SFD_SET_DATA_PROPS(6,1,1,0,"qFinHands");
          _SFD_SET_DATA_PROPS(7,2,0,1,"qDes");
          _SFD_SET_DATA_PROPS(8,1,1,0,"DT");
          _SFD_SET_DATA_PROPS(9,1,1,0,"t");
          _SFD_SET_DATA_PROPS(10,2,0,1,"q_AuxOut");
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
        _SFD_CV_INIT_EML(0,1,1,8,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,1258);
        _SFD_CV_INIT_EML_IF(0,1,0,169,186,-1,282);
        _SFD_CV_INIT_EML_IF(0,1,1,284,301,-1,524);
        _SFD_CV_INIT_EML_IF(0,1,2,458,466,-1,489);
        _SFD_CV_INIT_EML_IF(0,1,3,526,543,-1,824);
        _SFD_CV_INIT_EML_IF(0,1,4,728,736,-1,759);
        _SFD_CV_INIT_EML_IF(0,1,5,826,843,-1,1128);
        _SFD_CV_INIT_EML_IF(0,1,6,1029,1037,-1,1060);
        _SFD_CV_INIT_EML_IF(0,1,7,1130,1147,-1,1254);
        _SFD_TRANS_COV_WTS(0,0,0,1,0);
        if (chartAlreadyPresent==0) {
          _SFD_TRANS_COV_MAPS(0,
                              0,NULL,NULL,
                              0,NULL,NULL,
                              1,NULL,NULL,
                              0,NULL,NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c3_b_sf_marshallOut,(MexInFcnForType)NULL);

        {
          unsigned int dimVector[1];
          dimVector[0]= 25;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c3_sf_marshallOut,(MexInFcnForType)NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c3_b_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c3_b_sf_marshallOut,(MexInFcnForType)c3_b_sf_marshallIn);

        {
          unsigned int dimVector[1];
          dimVector[0]= 25;
          _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c3_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 25;
          _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c3_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 2;
          dimVector[1]= 5;
          _SFD_SET_DATA_COMPILED_PROPS(6,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c3_c_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 25;
          _SFD_SET_DATA_COMPILED_PROPS(7,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c3_sf_marshallOut,(MexInFcnForType)
            c3_sf_marshallIn);
        }

        _SFD_SET_DATA_COMPILED_PROPS(8,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c3_b_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(9,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c3_b_sf_marshallOut,(MexInFcnForType)NULL);

        {
          unsigned int dimVector[1];
          dimVector[0]= 25;
          _SFD_SET_DATA_COMPILED_PROPS(10,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c3_sf_marshallOut,(MexInFcnForType)
            c3_sf_marshallIn);
        }

        {
          real_T *c3_demoState;
          real_T *c3_toIn;
          real_T *c3_to;
          real_T *c3_DT;
          real_T *c3_t;
          real_T (*c3_q_AuxIn)[25];
          real_T (*c3_q_o)[25];
          real_T (*c3_q)[25];
          real_T (*c3_qFinHands)[10];
          real_T (*c3_qDes)[25];
          real_T (*c3_q_AuxOut)[25];
          c3_q_AuxOut = (real_T (*)[25])ssGetOutputPortSignal(chartInstance->S,
            3);
          c3_t = (real_T *)ssGetInputPortSignal(chartInstance->S, 7);
          c3_DT = (real_T *)ssGetInputPortSignal(chartInstance->S, 6);
          c3_qDes = (real_T (*)[25])ssGetOutputPortSignal(chartInstance->S, 2);
          c3_qFinHands = (real_T (*)[10])ssGetInputPortSignal(chartInstance->S,
            5);
          c3_q = (real_T (*)[25])ssGetInputPortSignal(chartInstance->S, 4);
          c3_q_o = (real_T (*)[25])ssGetInputPortSignal(chartInstance->S, 3);
          c3_to = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
          c3_toIn = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
          c3_q_AuxIn = (real_T (*)[25])ssGetInputPortSignal(chartInstance->S, 1);
          c3_demoState = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, c3_demoState);
          _SFD_SET_DATA_VALUE_PTR(1U, *c3_q_AuxIn);
          _SFD_SET_DATA_VALUE_PTR(2U, c3_toIn);
          _SFD_SET_DATA_VALUE_PTR(3U, c3_to);
          _SFD_SET_DATA_VALUE_PTR(4U, *c3_q_o);
          _SFD_SET_DATA_VALUE_PTR(5U, *c3_q);
          _SFD_SET_DATA_VALUE_PTR(6U, *c3_qFinHands);
          _SFD_SET_DATA_VALUE_PTR(7U, *c3_qDes);
          _SFD_SET_DATA_VALUE_PTR(8U, c3_DT);
          _SFD_SET_DATA_VALUE_PTR(9U, c3_t);
          _SFD_SET_DATA_VALUE_PTR(10U, *c3_q_AuxOut);
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
  return "8L2MnxQKdpImT4MQBzbomC";
}

static void sf_opaque_initialize_c3_controller(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc3_controllerInstanceStruct*) chartInstanceVar
    )->S,0);
  initialize_params_c3_controller((SFc3_controllerInstanceStruct*)
    chartInstanceVar);
  initialize_c3_controller((SFc3_controllerInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_enable_c3_controller(void *chartInstanceVar)
{
  enable_c3_controller((SFc3_controllerInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c3_controller(void *chartInstanceVar)
{
  disable_c3_controller((SFc3_controllerInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_gateway_c3_controller(void *chartInstanceVar)
{
  sf_c3_controller((SFc3_controllerInstanceStruct*) chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c3_controller(SimStruct* S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c3_controller
    ((SFc3_controllerInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c3_controller();/* state var info */
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

extern void sf_internal_set_sim_state_c3_controller(SimStruct* S, const mxArray *
  st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c3_controller();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c3_controller((SFc3_controllerInstanceStruct*)
    chartInfo->chartInstance, mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c3_controller(SimStruct* S)
{
  return sf_internal_get_sim_state_c3_controller(S);
}

static void sf_opaque_set_sim_state_c3_controller(SimStruct* S, const mxArray
  *st)
{
  sf_internal_set_sim_state_c3_controller(S, st);
}

static void sf_opaque_terminate_c3_controller(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc3_controllerInstanceStruct*) chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
    }

    finalize_c3_controller((SFc3_controllerInstanceStruct*) chartInstanceVar);
    free((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }

  unload_controller_optimization_info();
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc3_controller((SFc3_controllerInstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c3_controller(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c3_controller((SFc3_controllerInstanceStruct*)
      (((ChartInfoStruct *)ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c3_controller(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_controller_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(S,sf_get_instance_specialization(),infoStruct,
      3);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(S,sf_get_instance_specialization(),
                infoStruct,3,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop(S,
      sf_get_instance_specialization(),infoStruct,3,
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
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,3,8);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,3,3);
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,3);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(999189254U));
  ssSetChecksum1(S,(878639794U));
  ssSetChecksum2(S,(1981577946U));
  ssSetChecksum3(S,(1678198605U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c3_controller(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c3_controller(SimStruct *S)
{
  SFc3_controllerInstanceStruct *chartInstance;
  chartInstance = (SFc3_controllerInstanceStruct *)malloc(sizeof
    (SFc3_controllerInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc3_controllerInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway = sf_opaque_gateway_c3_controller;
  chartInstance->chartInfo.initializeChart = sf_opaque_initialize_c3_controller;
  chartInstance->chartInfo.terminateChart = sf_opaque_terminate_c3_controller;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c3_controller;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c3_controller;
  chartInstance->chartInfo.getSimState = sf_opaque_get_sim_state_c3_controller;
  chartInstance->chartInfo.setSimState = sf_opaque_set_sim_state_c3_controller;
  chartInstance->chartInfo.getSimStateInfo = sf_get_sim_state_info_c3_controller;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c3_controller;
  chartInstance->chartInfo.mdlStart = mdlStart_c3_controller;
  chartInstance->chartInfo.mdlSetWorkWidths = mdlSetWorkWidths_c3_controller;
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

void c3_controller_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c3_controller(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c3_controller(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c3_controller(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c3_controller_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
