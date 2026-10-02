#include "sdk.h"
#include "movement.h"
#include "prediction.h"
#include <numbers>

#ifndef FL_ONGROUND
#define FL_ONGROUND (1 << 0)
#endif

static void clamp_angles(vector& ang) {
  ang.m_x = std::clamp(ang.m_x, -89.f, 89.f);
  ang.m_y = std::remainderf(ang.m_y, 360.f);
  ang.m_z = 0.f;
}

// ---------------------------------------------------------------------------
// edgebug (ported from aletherium)
// ---------------------------------------------------------------------------

void c_movement::edgebug_reset() {
  m_eb_detected     = false;
  m_eb_lock_ticks   = 0;
  m_eb_current_tick = 0;
  m_eb_search_mode  = 0;
}

void c_movement::edgebug_pre() {
  const auto& ctrl = g_ui.m_controls.movement;
  if (!ctrl.edgebug->m_value || !ctrl.edgebug_key->value_->enabled) {
    edgebug_reset();
    return;
  }

  // save state before prediction.
  m_eb_velocity_backup = g_cl.m_local->m_velocity();
  m_eb_flags           = g_cl.m_local->flags();
}

bool c_movement::edgebug_check(usercmd_t* cmd) {
  const auto local = g_cl.m_local;
  if (!local || !local->is_alive())
    return false;

  const auto move_type = local->move_type();
  if (move_type == MOVETYPE_LADDER || move_type == MOVETYPE_NOCLIP)
    return false;

  static auto sv_gravity = g_interfaces.m_cvar->find_var("sv_gravity");
  if (!sv_gravity)
    return false;

  const float gravity     = sv_gravity->m_value.m_float_value;
  const float gravity_vel = gravity * 0.5f * g_interfaces.m_global_vars->m_interval_per_tick;

  const vector velocity = local->m_velocity();

  // a normal landing leaves us on the ground, an edgebug doesn't.
  if (local->flags() & FL_ONGROUND)
    return false;

  // check 1: was falling and velocity reset to the first-tick gravity value.
  if (m_eb_velocity_backup.m_z < -gravity_vel &&
      (roundf(velocity.m_z) == -roundf(gravity_vel) || fabsf(velocity.m_z + gravity_vel) < 2.8f))
    return true;

  // check 2: was falling and velocity increased but still negative (edge scrape),
  // verified by a steep surface being near the player.
  if (m_eb_velocity_backup.m_z < -6.0f && velocity.m_z > m_eb_velocity_backup.m_z &&
      velocity.m_z < -6.0f) {
    trace_t          trace;
    trace_world_only filter;
    vector           origin = local->m_vec_origin();
    origin.m_z += 200.f;

    const float step = static_cast<float>(pi_2) / 16.f;
    for (float a = 0.f; a < pi_2; a += step) {
      const vector start(32.f * cosf(a) + origin.m_x, 32.f * sinf(a) + origin.m_y, origin.m_z);
      const vector end = start - vector(0.f, 0.f, 300.f);

      ray_t ray;
      ray.initialize(start, end);
      g_interfaces.m_engine_trace->trace_ray(ray, MASK_PLAYERSOLID, &filter, &trace);

      if (trace.m_fraction != 1.f && trace.m_plane.normal.m_z < 0.6f)
        return true;
    }
  }

  return false;
}

// re-expresses the movement in cmd (which is relative to wish_angle) relative to old_angles.
void c_movement::edgebug_correct_movement(usercmd_t* cmd, vector wish_angle, vector old_angles) {
  if (old_angles == wish_angle)
    return;

  vector wish_forward, wish_right, wish_up, cmd_forward, cmd_right, cmd_up;
  wish_angle.angle_vectors(&wish_forward, &wish_right, &wish_up);
  old_angles.angle_vectors(&cmd_forward, &cmd_right, &cmd_up);

  const vector move(cmd->forwardmove_, cmd->sidemove_, cmd->upmove_);

  const float wf = sqrtf(wish_forward.m_x * wish_forward.m_x + wish_forward.m_y * wish_forward.m_y);
  const float wr = sqrtf(wish_right.m_x * wish_right.m_x + wish_right.m_y * wish_right.m_y);
  const float wu = fabsf(wish_up.m_z);
  const float cf = sqrtf(cmd_forward.m_x * cmd_forward.m_x + cmd_forward.m_y * cmd_forward.m_y);
  const float cr = sqrtf(cmd_right.m_x * cmd_right.m_x + cmd_right.m_y * cmd_right.m_y);
  const float cu = fabsf(cmd_up.m_z);
  if (wf == 0.f || wr == 0.f || wu == 0.f || cf == 0.f || cr == 0.f || cu == 0.f)
    return;

  const vector wfn(wish_forward.m_x / wf, wish_forward.m_y / wf, 0.f);
  const vector wrn(wish_right.m_x / wr, wish_right.m_y / wr, 0.f);
  const vector cfn(cmd_forward.m_x / cf, cmd_forward.m_y / cf, 0.f);
  const vector crn(cmd_right.m_x / cr, cmd_right.m_y / cr, 0.f);

  const float world_x = wfn.m_x * move.m_x + wrn.m_x * move.m_y;
  const float world_y = wfn.m_y * move.m_x + wrn.m_y * move.m_y;

  cmd->forwardmove_ = std::clamp(cfn.m_x * world_x + cfn.m_y * world_y, -450.f, 450.f);
  cmd->sidemove_    = std::clamp(crn.m_x * world_x + crn.m_y * world_y, -450.f, 450.f);
  cmd->upmove_      = std::clamp((cmd_up.m_z / cu) * (wish_up.m_z / wu) * move.m_z, -320.f, 320.f);
}

// auto strafe used while searching for an edgebug. writes movement relative to cmd->m_viewangles.
void c_movement::edgebug_auto_strafe(usercmd_t* cmd) {
  static float side = 1.f;
  side              = -side;

  const vector velocity   = g_cl.m_local->m_velocity();
  vector       wish_angle = cmd->m_viewangles;

  const float speed        = velocity.length_2d();
  const float ideal_strafe = speed > 0.f ? std::clamp(rad_to_deg(atanf(15.f / speed)), 0.f, 90.f) : 90.f;

  cmd->forwardmove_ = 0.f;

  static auto cl_sidespeed = g_interfaces.m_cvar->find_var("cl_sidespeed");
  const float side_speed   = cl_sidespeed ? cl_sidespeed->m_value.m_float_value : 450.f;

  static float old_yaw       = 0.f;
  const float  yaw_delta     = std::remainderf(wish_angle.m_y - old_yaw, 360.f);
  const float  abs_yaw_delta = fabsf(yaw_delta);
  old_yaw                    = wish_angle.m_y;

  if (abs_yaw_delta <= ideal_strafe || abs_yaw_delta >= 30.f) {
    const vector velocity_dir   = velocity.angle_to();
    const float  velocity_delta = std::remainderf(wish_angle.m_y - velocity_dir.m_y, 360.f);
    const float  retrack = (speed > 0.f ? std::clamp(rad_to_deg(atanf(30.f / speed)), 0.f, 90.f) : 90.f) * 2.f;

    if (velocity_delta <= retrack || speed <= 15.f) {
      if (-retrack <= velocity_delta || speed <= 15.f) {
        wish_angle.m_y += side * ideal_strafe;
        cmd->sidemove_ = side_speed * side;
      } else {
        wish_angle.m_y = velocity_dir.m_y - retrack;
        cmd->sidemove_ = side_speed;
      }
    } else {
      wish_angle.m_y = velocity_dir.m_y + retrack;
      cmd->sidemove_ = -side_speed;
    }

    edgebug_correct_movement(cmd, wish_angle, cmd->m_viewangles);
  } else if (yaw_delta > 0.f)
    cmd->sidemove_ = -side_speed;
  else
    cmd->sidemove_ = side_speed;
}

void c_movement::edgebug_post() {
  const auto& ctrl  = g_ui.m_controls.movement;
  const auto  local = g_cl.m_local;
  const auto  cmd   = g_cl.m_cmd;

  if (!ctrl.edgebug->m_value || !ctrl.edgebug_key->value_->enabled || !local ||
      !local->is_alive() || local->move_type() != MOVETYPE_WALK) {
    edgebug_reset();
    return;
  }

  // don't run if on ground or going up.
  if (m_eb_flags & FL_ONGROUND || m_eb_velocity_backup.m_z > 0.f) {
    edgebug_reset();
    return;
  }

  // haven's prediction leaves the local player one tick ahead, put them back on the last
  // engine-predicted frame before simulating anything.
  g_prediction.restore_to_predicted();

  // setup globals the same way the engine does when running commands.
  const int   old_tick_count = g_interfaces.m_global_vars->m_tick_count;
  const float old_cur_time   = g_interfaces.m_global_vars->m_cur_time;
  const float old_frame_time = g_interfaces.m_global_vars->m_frame_time;
  const float interval       = g_interfaces.m_global_vars->m_interval_per_tick;

  g_interfaces.m_global_vars->m_tick_count = local->m_tick_base();
  g_interfaces.m_global_vars->m_cur_time   = local->m_tick_base() * interval;
  g_interfaces.m_global_vars->m_frame_time = interval;

  const auto restore_globals = [&] {
    g_interfaces.m_global_vars->m_tick_count = old_tick_count;
    g_interfaces.m_global_vars->m_cur_time   = old_cur_time;
    g_interfaces.m_global_vars->m_frame_time = old_frame_time;
  };

  // backup original command.
  const int    backup_buttons = cmd->buttons_;
  const float  backup_forward = cmd->forwardmove_;
  const float  backup_side    = cmd->sidemove_;
  const vector backup_angles  = cmd->m_viewangles;

  // mouse delta from last frame, used to continue the player's turn in the strafe modes.
  static vector last_angles = backup_angles;
  vector        angle_delta = backup_angles - last_angles;
  angle_delta.m_y           = std::clamp(std::remainderf(angle_delta.m_y, 360.f), -(180.f / 128.f), 180.f / 128.f);
  angle_delta.m_x           = 0.f;
  angle_delta.m_z           = 0.f;
  angle_delta *= 0.5f;
  last_angles = backup_angles;

  const bool stack = ctrl.edgebug_stack->m_value;

  // replay finished last frame -> bookkeeping, then fall through into a fresh search so a
  // chained edgebug can be picked up on this very command.
  if (m_eb_detected) {
    m_eb_current_tick++;
    if (m_eb_current_tick > m_eb_lock_ticks) {
      if (stack && old_tick_count <= m_eb_stack_window)
        m_eb_stack_count++;
      else
        m_eb_stack_count = 1;
      m_eb_stack_window = old_tick_count + 96; // ~1.5s to land the next one

      if (m_eb_stack_count > 1)
        g_interfaces.m_cvar->console_printf("[haven] edgebug x%d\n", m_eb_stack_count);
      else
        g_interfaces.m_cvar->console_printf("[haven] edgebug\n");
      edgebug_reset();
    }
  }

  // search.
  if (!m_eb_detected) {
    struct eb_mode_t {
      int   type; // see eb_*
      bool  duck;
      float yaw_offset; // steer: yaw offset from current velocity direction
    };
    enum { eb_still, eb_user, eb_auto_strafe, eb_steer };

    const bool advanced     = ctrl.edgebug_advanced_search->m_value;
    const bool auto_strafe  = advanced && ctrl.edgebug_auto_strafe->m_value;
    const bool already_duck = backup_buttons & IN_DUCK;

    static eb_mode_t last_success      = {-1, false, 0.f};
    static bool      has_last_success  = false;

    eb_mode_t modes[48];
    int       mode_count = 0;

    // try the mode that worked last time first.
    const bool queued_last = has_last_success && !(already_duck && !last_success.duck);
    if (queued_last)
      modes[mode_count++] = last_success;

    const auto push = [&](int type, bool duck, float yaw_offset = 0.f) {
      if (already_duck && !duck)
        return;
      if (queued_last && last_success.type == type && last_success.duck == duck &&
          last_success.yaw_offset == yaw_offset)
        return; // already queued first
      if (mode_count < 48)
        modes[mode_count++] = {type, duck, yaw_offset};
    };

    push(eb_user, false);
    push(eb_user, true);
    if (advanced) {
      push(eb_still, false);
      push(eb_still, true);
    }
    if (auto_strafe) {
      push(eb_auto_strafe, false);
      push(eb_auto_strafe, true);
    }
    if (stack) {
      // steer left/right of the current flight path by increasing amounts.
      for (float offset : {15.f, 30.f, 50.f, 75.f, 100.f})
        for (float sign : {1.f, -1.f})
          for (bool duck : {true, false})
            push(eb_steer, duck, sign * offset);
    }
    has_last_success = false;

    // cap the amount of work per frame, stacking can queue a lot of modes.
    const int max_predictions   = stack ? 768 : 384;
    int       total_predictions = 0;

    for (int i = 0; i < mode_count && !m_eb_detected && total_predictions < max_predictions; i++) {
      const auto mode = modes[i];
      const bool duck = mode.duck;

      g_prediction.restore_to_predicted();

      cmd->m_viewangles = backup_angles;
      cmd->forwardmove_ = backup_forward;
      cmd->sidemove_    = backup_side;
      cmd->buttons_     = backup_buttons;

      vector current_angle = backup_angles;
      m_eb_velocity_backup = local->m_velocity();

      // steer target is fixed at search start so the mode describes "turn by X then hold".
      const vector start_velocity = local->m_velocity();
      const float  start_yaw = start_velocity.length_2d() > 1.f ? start_velocity.angle_to().m_y : backup_angles.m_y;
      const float  steer_target_yaw = std::remainderf(start_yaw + mode.yaw_offset, 360.f);

      const int horizon = mode.type == eb_steer ? 48 : 64;
      for (int tick = 0; tick < horizon && total_predictions < max_predictions; tick++) {
        if (local->flags() & FL_ONGROUND || local->m_velocity().m_z > 0.f)
          break;

        if (duck)
          cmd->buttons_ |= IN_DUCK;
        else
          cmd->buttons_ &= ~IN_DUCK;

        switch (mode.type) {
          case eb_still:
            cmd->forwardmove_ = 0.f;
            cmd->sidemove_    = 0.f;
            cmd->m_viewangles = backup_angles;
            break;
          case eb_user:
            cmd->forwardmove_ = backup_forward;
            cmd->sidemove_    = backup_side;
            if (fabsf(std::remainderf(current_angle.m_y - backup_angles.m_y, 360.f)) < 179.f) {
              current_angle = current_angle + angle_delta;
              clamp_angles(current_angle);
            }
            cmd->m_viewangles = current_angle;
            break;
          case eb_auto_strafe:
            cmd->m_viewangles = backup_angles;
            edgebug_auto_strafe(cmd);
            break;
          case eb_steer: {
            // air strafe (wishdir perpendicular to velocity) towards the target yaw, then coast.
            cmd->m_viewangles     = backup_angles;
            const vector velocity = local->m_velocity();
            const float  vel_yaw  = velocity.length_2d() > 1.f ? velocity.angle_to().m_y : backup_angles.m_y;
            const float  diff     = std::remainderf(steer_target_yaw - vel_yaw, 360.f);
            if (fabsf(diff) < 2.f) {
              cmd->forwardmove_ = 0.f;
              cmd->sidemove_    = 0.f;
            } else {
              const float wish_yaw = vel_yaw + std::copysign(90.f, diff);
              const float rot      = deg_to_rad(wish_yaw - backup_angles.m_y);
              cmd->forwardmove_    = cosf(rot) * 450.f;
              cmd->sidemove_       = -sinf(rot) * 450.f;
            }
            break;
          }
        }

        // store command for this tick.
        auto& stored       = m_eb_cmds[tick];
        stored.viewangles  = cmd->m_viewangles;
        stored.forwardmove = cmd->forwardmove_;
        stored.sidemove    = cmd->sidemove_;
        stored.buttons     = cmd->buttons_;
        stored.origin      = local->m_vec_origin();

        const vector pre_velocity = local->m_velocity();
        g_prediction.simulate(cmd);
        total_predictions++;
        const vector post_velocity = local->m_velocity();

        // check 1: velocity increased while falling (edge scrape) with a horizontal push.
        const float  velocity_diff = pre_velocity.m_z - post_velocity.m_z;
        const float  yaw_change    = std::remainderf(post_velocity.angle_to().m_y - pre_velocity.angle_to().m_y, 360.f);
        const bool   velocity_increased = floorf(post_velocity.m_z) > floorf(pre_velocity.m_z) &&
                                        pre_velocity.m_z < 0.f && post_velocity.m_z < 0.f &&
                                        pre_velocity.m_z * 0.25f > velocity_diff &&
                                        fabsf(yaw_change) < 45.f;
        const bool horizontal_increase = post_velocity.length_2d() > pre_velocity.length_2d();

        // check 2: gravity reset.
        const bool gravity_reset = edgebug_check(cmd);

        if ((velocity_increased && horizontal_increase) || gravity_reset) {
          m_eb_detected     = true;
          m_eb_lock_ticks   = tick + 1; // number of stored cmds to replay (0..tick)
          m_eb_current_tick = 1;        // replaying cmd 0 on this command
          m_eb_search_mode  = mode.type;
          m_eb_duck         = duck;
          last_success      = mode;
          has_last_success  = true;
          break;
        }

        m_eb_velocity_backup = post_velocity;
      }

      cmd->m_viewangles = backup_angles;
      cmd->forwardmove_ = backup_forward;
      cmd->sidemove_    = backup_side;
      cmd->buttons_     = backup_buttons;
    }
  }

  // execute.
  if (m_eb_detected) {
    g_prediction.restore_to_predicted();

    const int idx = m_eb_current_tick - 1;
    if (idx >= 0 && idx < 64) {
      const auto& stored = m_eb_cmds[idx];

      // if we drifted from the predicted path, give up.
      if ((local->m_vec_origin() - stored.origin).length() > 1.f) {
        edgebug_reset();
        restore_globals();
        return;
      }

      cmd->buttons_     = stored.buttons;
      cmd->forwardmove_ = stored.forwardmove;
      cmd->sidemove_    = stored.sidemove;

      // keep the player's real view angles, re-express the stored movement relative to them.
      edgebug_correct_movement(cmd, stored.viewangles, backup_angles);
      cmd->m_viewangles = backup_angles;
    }
  }

  restore_globals();
  g_prediction.restore_to_predicted();
}

void c_movement::edgebug_mouse_lock(float& x, float& y) {
  const auto& ctrl = g_ui.m_controls.movement;
  if (!ctrl.edgebug->m_value || !ctrl.edgebug_mouse_lock->m_value || !ctrl.edgebug_key->value_->enabled)
    return;

  if (!g_cl.m_local || !g_cl.m_local->is_alive())
    return;

  if (!m_eb_detected || m_eb_lock_ticks <= 0)
    return;

  // freeze the mouse while the predicted command sequence is being replayed.
  x = 0.f;
  y = 0.f;
}

void c_movement::bhop() {
  const auto MoveType = g_cl.m_local->move_type();
  if (MoveType == MOVETYPE_NOCLIP || MoveType == MOVETYPE_LADDER ||
      MoveType == MOVETYPE_OBSERVER)
    return;

  const bool  jump          = g_cl.m_cmd->buttons_ & IN_JUMP;
  static bool was_jump_held = false;
  const auto  ground        = g_cl.m_local->get_ground();

  if (ground && jump) {
    was_jump_held = true;
    return;
  }

  if (was_jump_held && !ground && jump) {
    g_cl.m_cmd->buttons_ &= ~IN_JUMP;
    return;
  }

  was_jump_held = jump;
}

double normalize_rad(double a) {
  a = std::fmod(a, std::numbers::pi_v<double> * 2);

  if (a >= std::numbers::pi_v<double>) {
    a -= 2 * std::numbers::pi_v<double>;
  } else if (a < -std::numbers::pi_v<double>) {
    a += 2 * std::numbers::pi_v<double>;
  }

  return a;
}

#define M_PI 3.141592653589793238463
double MaxAccelTheta(double wishspeed) {
  static auto air_accel  = g_interfaces.m_cvar->find_var("sv_airaccelerate");
  double      accel      = air_accel->m_value.m_float_value;
  double      accelspeed = accel * 450.f * g_interfaces.m_global_vars->m_interval_per_tick;
  if (accelspeed <= 0.0)
    return M_PI;
  auto vel = g_cl.m_local->m_velocity();
  if (vel.length_2d() < 1.f)
    return 0.0;

  double wishspeed_capped = 30;
  double tmp              = wishspeed_capped - accelspeed;
  if (tmp <= 0.0)
    return M_PI / 2;

  double speed = vel.length_2d();
  if (tmp < speed)
    return std::acos(tmp / speed);

  return 0.0;
}
double max_accel_into_yaw_theta(const double& vel_yaw, const double& yaw,
                                const double& wish_speed) {
  const double theta = MaxAccelTheta(wish_speed);

  if (theta == 0.0 || theta == std::numbers::pi_v<double>) {
    return normalize_rad(yaw - vel_yaw + theta);
  }

  return std::copysign(theta, normalize_rad(yaw - vel_yaw));
}

static inline double ButtonsPhi(float forward, float side) {
  return -std::atan2f(side, forward);
}

void c_movement::auto_strafe(float* view) {
  if (g_cl.m_local->get_ground() || !(g_cl.m_cmd->buttons_ & IN_JUMP)) {
    return;
  }

  const auto velocity = g_cl.m_local->m_velocity();

  const float speed = velocity.length_2d();

  // compute the ideal strafe angle for our velocity.
  const float ideal =
      (speed > 0.f) * rad_to_deg(std::asin(15.f / speed)) + (speed <= 0.f) * 90.f;
  const float ideal2 =
      (speed > 0.f) * rad_to_deg(std::asin(30.f / speed)) + (speed <= 0.f) * 90.f;

  m_switch *= -1;

  // get our viewangle change.

  vector direction;
  direction.m_x = 1 * (fabsf(g_cl.m_cmd->sidemove_) < 0 &&
                       fabsf(g_cl.m_cmd->forwardmove_) < 0); // branchless optimization
  direction.m_x += 1 * (g_cl.m_cmd->forwardmove_ > 0) + -1 * (g_cl.m_cmd->forwardmove_ < 0);
  direction.m_y = 1 * (g_cl.m_cmd->sidemove_ < 0) + -1 * (g_cl.m_cmd->sidemove_ > 0);

  *view += vector().look(direction).m_y;

  auto delta = *view - m_old_yaw;
  while (delta > 180)
    delta -= 360;
  while (delta < -180)
    delta += 360;
  m_old_yaw = *view;

  // convert to absolute change.
  const auto abs_delta = std::abs(delta);

  // set strafe direction based on mouse direction change.
  double vel_yaw_rad = std::atan2(velocity.m_y, velocity.m_x);
  double yaw_rad     = deg_to_rad(*view);
  double theta       = max_accel_into_yaw_theta(vel_yaw_rad, yaw_rad, 450.f);

  g_cl.m_cmd->sidemove_    = (theta > 0) * 450.f + (theta < 0) * -450.f;
  g_cl.m_cmd->forwardmove_ = 0;

  double phi = ButtonsPhi(g_cl.m_cmd->forwardmove_, g_cl.m_cmd->sidemove_);
  float  yaw = vel_yaw_rad - phi + theta;

  *view = rad_to_deg(normalize_rad(yaw));
}

void c_movement::correct_movement(vector old) {
  vector wish_forward, wish_right, wish_up, cmd_forward, cmd_right, cmd_up;

  const vector movedata{g_cl.m_cmd->forwardmove_, g_cl.m_cmd->sidemove_, g_cl.m_cmd->upmove_};

  old.angle_vectors(&wish_forward, &wish_right, &wish_up);
  g_cl.m_cmd->m_viewangles.angle_vectors(&cmd_forward, &cmd_right, &cmd_up);

  const auto v8  = sqrtf(wish_forward.m_x * wish_forward.m_x +
                         wish_forward.m_y * wish_forward.m_y),
             v10 = sqrt(wish_right.m_x * wish_right.m_x + wish_right.m_y * wish_right.m_y),
             v12 = sqrt(wish_up.m_z * wish_up.m_z);

  const vector wish_forward_norm(1.0f / v8 * wish_forward.m_x, 1.0f / v8 * wish_forward.m_y,
                                 0.f),
      wish_right_norm(1.0f / v10 * wish_right.m_x, 1.0f / v10 * wish_right.m_y, 0.f),
      wish_up_norm(0.f, 0.f, 1.0f / v12 * wish_up.m_z);

  const auto v14 = sqrtf(cmd_forward.m_x * cmd_forward.m_x + cmd_forward.m_y * cmd_forward.m_y),
             v16 = sqrt(cmd_right.m_x * cmd_right.m_x + cmd_right.m_y * cmd_right.m_y),
             v18 = sqrt(cmd_up.m_z * cmd_up.m_z);

  const vector cmd_forward_norm(1.0f / v14 * cmd_forward.m_x, 1.0f / v14 * cmd_forward.m_y,
                                1.0f / v14 * 0.0f),
      cmd_right_norm(1.0f / v16 * cmd_right.m_x, 1.0f / v16 * cmd_right.m_y, 1.0f / v16 * 0.0f),
      cmd_up_norm(0.f, 0.f, 1.0f / v18 * cmd_up.m_z);

  const auto v22 = wish_forward_norm.m_x * movedata.m_x,
             v26 = wish_forward_norm.m_y * movedata.m_x,
             v28 = wish_forward_norm.m_z * movedata.m_x,
             v24 = wish_right_norm.m_x * movedata.m_y, v23 = wish_right_norm.m_y * movedata.m_y,
             v25 = wish_right_norm.m_z * movedata.m_y, v30 = wish_up_norm.m_x * movedata.m_z,
             v27 = wish_up_norm.m_z * movedata.m_z, v29 = wish_up_norm.m_y * movedata.m_z;

  vector correct_movement{
      (cmd_forward_norm.m_x * v24 + cmd_forward_norm.m_y * v23 + cmd_forward_norm.m_z * v25) +
          (cmd_forward_norm.m_x * v22 + cmd_forward_norm.m_y * v26 +
           cmd_forward_norm.m_z * v28) +
          (cmd_forward_norm.m_y * v30 + cmd_forward_norm.m_x * v29 +
           cmd_forward_norm.m_z * v27),

      (cmd_right_norm.m_x * v24 + cmd_right_norm.m_y * v23 + cmd_right_norm.m_z * v25) +
          (cmd_right_norm.m_x * v22 + cmd_right_norm.m_y * v26 + cmd_right_norm.m_z * v28) +
          (cmd_right_norm.m_x * v29 + cmd_right_norm.m_y * v30 + cmd_right_norm.m_z * v27),

      (correct_movement.m_z =
           cmd_up_norm.m_x * v23 + cmd_up_norm.m_y * v24 + cmd_up_norm.m_z * v25) +
          (cmd_up_norm.m_x * v26 + cmd_up_norm.m_y * v22 + cmd_up_norm.m_z * v28) +
          (cmd_up_norm.m_x * v30 + cmd_up_norm.m_y * v29 + cmd_up_norm.m_z * v27)};

  g_cl.m_cmd->forwardmove_ = std::clamp<float>(correct_movement.m_x, -450.f, 450.f);
  g_cl.m_cmd->sidemove_    = std::clamp<float>(correct_movement.m_y, -450.f, 450.f);
  g_cl.m_cmd->upmove_      = std::clamp<float>(correct_movement.m_z, -320.f, 320.f);
}
