/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

/**
 * delta.cpp
 */

#include "../inc/MarlinConfig.h"

#if ENABLED(DELTA)

#include "delta.h"
#include "motion.h"

// For homing:
#include "planner.h"
#include "endstops.h"
#include "../lcd/marlinui.h"
#include "../MarlinCore.h"

#if HAS_BED_PROBE
  #include "probe.h"
#endif

#if ENABLED(SENSORLESS_HOMING)
  #include "../feature/tmc_util.h"
  #include "stepper/indirection.h"
#endif

#define DEBUG_OUT ENABLED(DEBUG_LEVELING_FEATURE)
#include "../core/debug_out.h"

// Initialized by settings.load()





float delta_height;
abc_float_t delta_endstop_adj{0};
float segments_per_second;
float delta_clip_start_height = Z_MAX_POS;
float delta_radius;
#if ENABLED(ROTARY_DELTA)
  float delta_joint_offset;//MOTOR_RADIUS - EFFECTOR_RADIUS
  float delta_arm;
  float delta_bicep;
  float delta_bicep_height;
  float delta_bicep2;
  float delta_bicep2_minus_arm2;
  float delta_bicep_drive_radius;
  float delta_tool_offset;
  float delta_bicep_height_adjusted; //bicep_height - tool_offset
  float delta_xy_compensation;
  float delta_z_compensation;
  float delta_z_decay;
  float cos30 = cosf(radians(30.0));
  float xy_comp;
  float z_comp;
  float z_decay;
#else
  
  float delta_diagonal_rod;
  abc_float_t delta_tower_angle_trim;
  xy_float_t delta_tower[ABC];
  abc_float_t delta_diagonal_rod_2_tower;
  abc_float_t delta_diagonal_rod_trim; 
#endif

float delta_safe_distance_from_top();

/**
 * Recalculate factors used for delta kinematics whenever
 * settings have been changed (e.g., by M665).
 */
void recalc_delta_settings() {
  #if ENABLED(ROTARY_DELTA)
    delta_bicep2 = delta_bicep * delta_bicep;
    delta_bicep2_minus_arm2 = delta_bicep2 - delta_arm * delta_arm;
    delta_bicep_height_adjusted = delta_bicep_height - delta_tool_offset;
    xy_comp = 1.0f+delta_xy_compensation/1000.0f;
    z_comp = delta_z_compensation/1000.0f;
    z_decay = delta_z_decay/1000.0f;
  #else
    constexpr abc_float_t trt = DELTA_RADIUS_TRIM_TOWER;
  delta_tower[A_AXIS].set(cos(RADIANS(210 + delta_tower_angle_trim.a)) * (delta_radius + trt.a), // front left tower
                          sin(RADIANS(210 + delta_tower_angle_trim.a)) * (delta_radius + trt.a));
  delta_tower[B_AXIS].set(cos(RADIANS(330 + delta_tower_angle_trim.b)) * (delta_radius + trt.b), // front right tower
                          sin(RADIANS(330 + delta_tower_angle_trim.b)) * (delta_radius + trt.b));
  delta_tower[C_AXIS].set(cos(RADIANS( 90 + delta_tower_angle_trim.c)) * (delta_radius + trt.c), // back middle tower
                          sin(RADIANS( 90 + delta_tower_angle_trim.c)) * (delta_radius + trt.c));
  delta_diagonal_rod_2_tower.set(sq(delta_diagonal_rod + delta_diagonal_rod_trim.a),
                                 sq(delta_diagonal_rod + delta_diagonal_rod_trim.b),
                                 sq(delta_diagonal_rod + delta_diagonal_rod_trim.c));
  #endif
  update_software_endstops(Z_AXIS);
  set_all_unhomed();
}

/**
 * Delta Inverse Kinematics
 *
 * Calculate the tower positions for a given machine
 * position, storing the result in the delta[] array.
 *
 * This is an expensive calculation, requiring 3 square
 * roots per segmented linear move, and strains the limits
 * of a Mega2560 with a Graphical Display.
 *
 * Suggested optimizations include:
 *
 * - Disable the home_offset (M206) and/or position_shift (G92)
 *   features to remove up to 12 float additions.
 */

#define DELTA_DEBUG(VAR) do { \
    SERIAL_ECHOLNPGM_P(PSTR("Cartesian X"), VAR.x, SP_Y_STR, VAR.y, SP_Z_STR, VAR.z); \
    SERIAL_ECHOLNPGM_P(PSTR("Delta A"), delta.a, SP_B_STR, delta.b, SP_C_STR, delta.c); \
  }while(0)

void inverse_kinematics(const xyz_pos_t &raw) {
  #if HAS_HOTEND_OFFSET
    // Delta hotend offsets must be applied in Cartesian space with no "spoofing"
    xyz_pos_t pos = { raw.x - hotend_offset[active_extruder].x,
                      raw.y - hotend_offset[active_extruder].y,
                      raw.z };
    #if ENABLED(ROTARY_DELTA)
      rotary_delta_ik(pos);
    #else
      DELTA_IK(pos);
    #endif
    //DELTA_DEBUG(pos);
  #else
    #if ENABLED(ROTARY_DELTA)
      //rotary_delta_ik(raw);
      adjusted_rotary_delta_ik(raw);
    #else
      DELTA_IK(raw);
    #endif
    //DELTA_DEBUG(raw);
  #endif
}
//compensate for machine flex as coordinates get farther from origin
void adjusted_rotary_delta_ik(const xyz_pos_t &pos){
  float d = hypotf(pos.x, pos.y);
  xyz_pos_t adjustedPos;
  adjustedPos.x = pos.x * xy_comp;
  adjustedPos.y = pos.y * xy_comp;
  adjustedPos.z = pos.z + d*z_comp*(1.0-z_decay*pos.z);// compensate z for sag. farther from origin needs more compensation
  rotary_delta_ik(adjustedPos);
}
void rotary_delta_ik(const xyz_pos_t &pos){
    float xSin30 = pos.x*0.5f;
    float xCos30 = pos.x*cos30;
    float ySin30 = pos.y*0.5f;
    float yCos30 = pos.y*cos30;
    float x, y, D, l_over_d, h_over_d;
    float z = pos.z - delta_bicep_height_adjusted;
    float z2 = z*z;
    
    float bicep_arc_length; //the arc length the bicep needs to travel to achieve the given position
    
    //calculate arm A (aligned with x axis so no transformation required)
    D = sq(pos.x-delta_joint_offset) + z2;
    l_over_d = (delta_bicep2_minus_arm2+sq(pos.y)+D)*0.5f/D;
    h_over_d = sqrt(delta_bicep2/D - sq(l_over_d));
    bicep_arc_length = delta_bicep * asin((l_over_d * z + h_over_d * (pos.x - delta_joint_offset))/delta_bicep);
    delta.x = bicep_arc_length;

    //calculate arm B (located 120 clockwise from Arm A)
    x = -xSin30 - yCos30; 
    y  = xCos30 - ySin30;
    D = sq(x-delta_joint_offset) + z2;
    l_over_d = (delta_bicep2_minus_arm2+sq(y)+D)*0.5f/D;
    h_over_d = sqrt(delta_bicep2/D - sq(l_over_d));
    bicep_arc_length = delta_bicep * asin((l_over_d * z + h_over_d * (x - delta_joint_offset))/delta_bicep);
    delta.y = bicep_arc_length;

    //calculate arm C (located 120 clockwise from Arm B)
    x = -xSin30 + yCos30; 
    y  = xCos30 + ySin30;
    D = sq(x-delta_joint_offset) + z2;
    l_over_d = (delta_bicep2_minus_arm2+sq(y)+D)*0.5f/D;
    h_over_d = sqrt(delta_bicep2/D - sq(l_over_d));
    bicep_arc_length = delta_bicep * asin((l_over_d * z + h_over_d * (x - delta_joint_offset))/delta_bicep);
    delta.z = bicep_arc_length;
}
/**
 * Calculate the highest Z position where the
 * effector has the full range of XY motion.
 */
float delta_safe_distance_from_top() {
  #if ENABLED(ROTARY_DELTA)
    return 0.0f;
  #else
    xyz_pos_t cartesian{0};
    inverse_kinematics(cartesian);
    const float centered_extent = delta.a;
    cartesian.y = DELTA_PRINTABLE_RADIUS;
    inverse_kinematics(cartesian);
    return ABS(centered_extent - delta.a);
  #endif
}

/**
 * Delta Forward Kinematics
 *
 * See the Wikipedia article "Trilateration"
 * https://en.wikipedia.org/wiki/Trilateration
 *
 * Establish a new coordinate system in the plane of the
 * three carriage points. This system has its origin at
 * tower1, with tower2 on the X axis. Tower3 is in the X-Y
 * plane with a Z component of zero.
 * We will define unit vectors in this coordinate system
 * in our original coordinate system. Then when we calculate
 * the Xnew, Ynew and Znew values, we can translate back into
 * the original system by moving along those unit vectors
 * by the corresponding values.
 *
 * Variable names matched to Marlin, c-version, and avoid the
 * use of any vector library.
 *
 * by Andreas Hardtung 2016-06-07
 * based on a Java function from "Delta Robot Kinematics V3"
 * by Steve Graves
 *
 * The result is stored in the cartes[] array.
 */
void forward_kinematics(const_float_t z1, const_float_t z2, const_float_t z3) {
  #if ENABLED(ROTARY_DELTA)
    //z1,z2 & z3 represent the arc lengths of the current bicep position. convert them to angles and then find the end points of the three biceps
    float bicep_angle = z1/delta_bicep;
    float bicep_reach;
    float pa[3] = {/*x*/ delta_bicep * cosf(bicep_angle)+ delta_joint_offset,
                   /*y*/ 0.0,
                   /*z*/ delta_bicep * sinf(bicep_angle) + delta_bicep_height_adjusted};

    bicep_angle = z2/delta_bicep;
    bicep_reach = delta_bicep * cosf(bicep_angle)+ delta_joint_offset;
    float pb[3] = {/*x*/ bicep_reach*-0.5f /*(sin30 = 0.5)*/,
                   /*y*/ bicep_reach*-cos30,
                   /*z*/ delta_bicep * sinf(bicep_angle) + delta_bicep_height_adjusted};

    bicep_angle = z3/delta_bicep;
    bicep_reach = delta_bicep * cosf(bicep_angle)+ delta_joint_offset;
    float pc[3] = {/*x*/ bicep_reach*-0.5f /*(sin30 = 0.5)*/,
                   /*y*/ bicep_reach*cos30,
                   /*z*/ delta_bicep * sinf(bicep_angle) + delta_bicep_height_adjusted};
    
    float ac[3] = {pa[0]-pc[0], pa[1]-pc[1], pa[2]-pc[2]};
    float bc[3] = {pb[0]-pc[0], pb[1]-pc[1], pb[2]-pc[2]};
    
    float ac_cross_bc[3] = {ac[1]*bc[2] - ac[2]*bc[1],
                            ac[2]*bc[0] - ac[0]*bc[2],
                            ac[0]*bc[1] - ac[1]*bc[0]};

    float mag_ac2 = ac[0]*ac[0] + ac[1]*ac[1] + ac[2]*ac[2];
    
    float mag_bc2 = bc[0]*bc[0] + bc[1]*bc[1] + bc[2]*bc[2];
    
    float mag_ac_cross_bc2 = ac_cross_bc[0]*ac_cross_bc[0] + ac_cross_bc[1]*ac_cross_bc[1] + ac_cross_bc[2]*ac_cross_bc[2];
    
    float v[3] = {mag_ac2*bc[0] - mag_bc2*ac[0],
                  mag_ac2*bc[1] - mag_bc2*ac[1],
                  mag_ac2*bc[2] - mag_bc2*ac[2]};
    float w[3] = {ac_cross_bc[1]*v[2] - ac_cross_bc[2]*v[1],
                  ac_cross_bc[2]*v[0] - ac_cross_bc[0]*v[2],
                  ac_cross_bc[0]*v[1] - ac_cross_bc[1]*v[0]};
    float center[3] = {pc[0] - w[0]*0.5f/mag_ac_cross_bc2,
                       pc[1] - w[1]*0.5f/mag_ac_cross_bc2,
                       pc[2] - w[2]*0.5f/mag_ac_cross_bc2};
    float center_pc[3] = {center[0]-pc[0], center[1]-pc[1], center[2]-pc[2]};
    float center_base_height = sqrtf((delta_arm*delta_arm - (center_pc[0]*center_pc[0] + center_pc[1]*center_pc[1] + center_pc[2]*center_pc[2]))/mag_ac_cross_bc2);
     
    cartes.set(/*x*/center[0] + ac_cross_bc[0]*center_base_height,
               /*y*/center[1] + ac_cross_bc[1]*center_base_height,
               /*z*/center[2] + ac_cross_bc[2]*center_base_height);
  #else
    // Create a vector in old coordinates along x axis of new coordinate
  const float p12[3] = { delta_tower[B_AXIS].x - delta_tower[A_AXIS].x, delta_tower[B_AXIS].y - delta_tower[A_AXIS].y, z2 - z1 },

  // Get the reciprocal of Magnitude of vector.
  d2 = sq(p12[0]) + sq(p12[1]) + sq(p12[2]), inv_d = RSQRT(d2),

  // Create unit vector by multiplying by the inverse of the magnitude.
  ex[3] = { p12[0] * inv_d, p12[1] * inv_d, p12[2] * inv_d },

  // Get the vector from the origin of the new system to the third point.
  p13[3] = { delta_tower[C_AXIS].x - delta_tower[A_AXIS].x, delta_tower[C_AXIS].y - delta_tower[A_AXIS].y, z3 - z1 },

  // Use the dot product to find the component of this vector on the X axis.
  i = ex[0] * p13[0] + ex[1] * p13[1] + ex[2] * p13[2],

  // Create a vector along the x axis that represents the x component of p13.
  iex[3] = { ex[0] * i, ex[1] * i, ex[2] * i };

  // Subtract the X component from the original vector leaving only Y. We use the
  // variable that will be the unit vector after we scale it.
  float ey[3] = { p13[0] - iex[0], p13[1] - iex[1], p13[2] - iex[2] };

  // The magnitude and the inverse of the magnitude of Y component
  const float j2 = sq(ey[0]) + sq(ey[1]) + sq(ey[2]), inv_j = RSQRT(j2);

  // Convert to a unit vector
  ey[0] *= inv_j; ey[1] *= inv_j; ey[2] *= inv_j;

  // The cross product of the unit x and y is the unit z
  // float[] ez = vectorCrossProd(ex, ey);
  const float ez[3] = {
    ex[1] * ey[2] - ex[2] * ey[1],
    ex[2] * ey[0] - ex[0] * ey[2],
    ex[0] * ey[1] - ex[1] * ey[0]
  },

  // We now have the d, i and j values defined in Wikipedia.
  // Plug them into the equations defined in Wikipedia for Xnew, Ynew and Znew
  Xnew = (delta_diagonal_rod_2_tower.a - delta_diagonal_rod_2_tower.b + d2) * inv_d * 0.5,
  Ynew = ((delta_diagonal_rod_2_tower.a - delta_diagonal_rod_2_tower.c + sq(i) + j2) * 0.5 - i * Xnew) * inv_j,
  Znew = SQRT(delta_diagonal_rod_2_tower.a - HYPOT2(Xnew, Ynew));

  // Start from the origin of the old coordinates and add vectors in the
  // old coords that represent the Xnew, Ynew and Znew to find the point
  // in the old system.
  cartes.set(delta_tower[A_AXIS].x + ex[0] * Xnew + ey[0] * Ynew - ez[0] * Znew,
             delta_tower[A_AXIS].y + ex[1] * Xnew + ey[1] * Ynew - ez[1] * Znew,
                                z1 + ex[2] * Xnew + ey[2] * Ynew - ez[2] * Znew);
  #endif
}

/**
 * A delta can only safely home all axes at the same time
 * This is like quick_home_xy() but for 3 towers.
 */
void home_delta() {
  DEBUG_SECTION(log_home_delta, "home_delta", DEBUGGING(LEVELING));

  // Init the current position of all carriages to 0,0,0
  current_position.reset();
  destination.reset();
  sync_plan_position();

  // Disable stealthChop if used. Enable diag1 pin on driver.
  #if ENABLED(SENSORLESS_HOMING)
    TERN_(X_SENSORLESS, sensorless_t stealth_states_x = start_sensorless_homing_per_axis(X_AXIS));
    TERN_(Y_SENSORLESS, sensorless_t stealth_states_y = start_sensorless_homing_per_axis(Y_AXIS));
    TERN_(Z_SENSORLESS, sensorless_t stealth_states_z = start_sensorless_homing_per_axis(Z_AXIS));
    TERN_(I_SENSORLESS, sensorless_t stealth_states_i = start_sensorless_homing_per_axis(I_AXIS));
    TERN_(J_SENSORLESS, sensorless_t stealth_states_j = start_sensorless_homing_per_axis(J_AXIS));
    TERN_(K_SENSORLESS, sensorless_t stealth_states_k = start_sensorless_homing_per_axis(K_AXIS));
  #endif

  // Move all carriages together linearly until an endstop is hit.
  current_position.z = DIFF_TERN(HAS_BED_PROBE, delta_height + 10, probe.offset.z);
  line_to_current_position(homing_feedrate(Z_AXIS));
  planner.synchronize();
  TERN_(HAS_DELTA_SENSORLESS_PROBING, endstops.report_states());

  // Re-enable stealthChop if used. Disable diag1 pin on driver.
  #if ENABLED(SENSORLESS_HOMING) && DISABLED(ENDSTOPS_ALWAYS_ON_DEFAULT)
    TERN_(X_SENSORLESS, end_sensorless_homing_per_axis(X_AXIS, stealth_states_x));
    TERN_(Y_SENSORLESS, end_sensorless_homing_per_axis(Y_AXIS, stealth_states_y));
    TERN_(Z_SENSORLESS, end_sensorless_homing_per_axis(Z_AXIS, stealth_states_z));
    TERN_(I_SENSORLESS, end_sensorless_homing_per_axis(I_AXIS, stealth_states_i));
    TERN_(J_SENSORLESS, end_sensorless_homing_per_axis(J_AXIS, stealth_states_j));
    TERN_(K_SENSORLESS, end_sensorless_homing_per_axis(K_AXIS, stealth_states_k));
  #endif

  endstops.validate_homing_move();

  // At least one carriage has reached the top.
  // Now re-home each carriage separately.
  homeaxis(A_AXIS);
  homeaxis(B_AXIS);
  homeaxis(C_AXIS);

  // Set all carriages to their home positions
  // Do this here all at once for Delta, because
  // XYZ isn't ABC. Applying this per-tower would
  // give the impression that they are the same.
  LOOP_ABC(i) set_axis_is_at_home((AxisEnum)i);

  sync_plan_position();

  #if DISABLED(DELTA_HOME_TO_SAFE_ZONE) && defined(HOMING_BACKOFF_POST_MM)
    constexpr xyz_float_t endstop_backoff = HOMING_BACKOFF_POST_MM;
    if (endstop_backoff.z) {
      current_position.z -= ABS(endstop_backoff.z) * Z_HOME_DIR;
      line_to_current_position(homing_feedrate(Z_AXIS));
    }
  #endif
}

#endif // DELTA
