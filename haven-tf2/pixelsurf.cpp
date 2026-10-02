#include "sdk.h"
#include "pixelsurf.h"
#include "prediction.h"
#include "movement.h"

#ifndef FL_ONGROUND
#define FL_ONGROUND (1 << 0)
#endif

// sets the globals up like the engine does while running commands, restores on destruction.
struct sim_globals_t {
  int   tick_count;
  float cur_time, frame_time;
  sim_globals_t() {
    const auto gv = g_interfaces.m_global_vars;
    tick_count    = gv->m_tick_count;
    cur_time      = gv->m_cur_time;
    frame_time    = gv->m_frame_time;
    gv->m_tick_count = g_cl.m_local->m_tick_base();
    gv->m_cur_time   = gv->m_tick_count * gv->m_interval_per_tick;
    gv->m_frame_time = gv->m_interval_per_tick;
  }
  ~sim_globals_t() {
    const auto gv    = g_interfaces.m_global_vars;
    gv->m_tick_count = tick_count;
    gv->m_cur_time   = cur_time;
    gv->m_frame_time = frame_time;
  }
};

// velocity.z you sit at every tick while pixel surfing: ground zeroes z, FinishGravity adds half
// a tick of gravity after the hull slides off the ledge again.
float c_pixelsurf::surf_velocity() const {
  static auto sv_gravity = g_interfaces.m_cvar->find_var("sv_gravity");
  const float gravity    = sv_gravity ? sv_gravity->m_value.m_float_value : 800.f;
  return -(gravity * g_interfaces.m_global_vars->m_interval_per_tick * 0.5f);
}

bool c_pixelsurf::is_surf_velocity(float z) const { return fabsf(z - surf_velocity()) < 0.015f; }

static bool on_ground() { return g_cl.m_local->flags() & FL_ONGROUND; }

// move into the closest wall with the smallest push that keeps us on the pixel.
void c_pixelsurf::auto_align(usercmd_t* cmd) {
  const auto& ctrl = g_ui.m_controls.pixelsurf;
  m_wall_detected  = false;
  if (!ctrl.auto_align->m_value || on_ground())
    return;

  const auto   local  = g_cl.m_local;
  const vector origin = local->m_vec_origin();
  const vector mins = local->mins(), maxs = local->maxs();

  // find the nearest vertical surface around us.
  constexpr int    steps = 32;
  constexpr float  step  = static_cast<float>(pi_2) / steps;
  trace_world_only filter;
  trace_t          best{};
  float            best_fraction = 1.f, best_angle = 0.f;
  for (int i = 0; i < steps; i++) {
    const float  a = fmodf(m_align_start + i * step, static_cast<float>(pi_2));
    const vector end(cosf(a) + origin.m_x, sinf(a) + origin.m_y, origin.m_z);
    ray_t        ray;
    ray.initialize(origin, end, mins, maxs);
    trace_t trace;
    g_interfaces.m_engine_trace->trace_ray(ray, MASK_PLAYERSOLID, &filter, &trace);
    if (trace.m_fraction < 1.f && fabsf(trace.m_plane.normal.m_z) < 0.1f &&
        trace.m_fraction < best_fraction) {
      best_fraction = trace.m_fraction;
      best          = trace;
      best_angle    = a;
      m_wall_detected = true;
    }
  }
  if (!m_wall_detected) {
    m_align_start = 0.f;
    return;
  }
  m_align_start = best_angle;
  m_wall_normal = best.m_plane.normal;

  // don't fight the player if they're clearly moving away from the wall.
  const vector into_wall(-m_wall_normal.m_x, -m_wall_normal.m_y, 0.f);
  const vector velocity = local->m_velocity();
  if (velocity.length_2d() > 1.f &&
      fabsf(std::remainderf(velocity.angle_to().m_y - into_wall.angle_to().m_y, 360.f)) > 100.f)
    return;

  const float rot = deg_to_rad(into_wall.angle_to().m_y - cmd->m_viewangles.m_y);
  const float cos_rot = cosf(rot), sin_rot = -sinf(rot);

  bool found = false;
  for (float mult = 0.f; mult <= 100.f; mult += 10.f) {
    g_prediction.restore_to_predicted();
    cmd->forwardmove_ = cos_rot * mult;
    cmd->sidemove_    = sin_rot * mult;
    g_prediction.simulate(cmd);
    if (!on_ground() && is_surf_velocity(local->m_velocity().m_z)) {
      found = true;
      break;
    }
  }
  if (!found) {
    cmd->forwardmove_ = cos_rot * 10.f;
    cmd->sidemove_    = sin_rot * 10.f;
  }
  g_prediction.restore_to_predicted();
}

// duck at the right moment so the hull keeps catching the pixel.
void c_pixelsurf::pixel_surf(usercmd_t* cmd, const vector& velocity, int tick_count) {
  const auto& ctrl = g_ui.m_controls.pixelsurf;
  if (on_ground()) {
    m_should_surf = false;
    return;
  }
  if (!m_wall_detected)
    return;

  if (!m_should_surf) {
    const int backup_buttons = cmd->buttons_;
    bool      found          = false;
    for (int i = 0; i < 2 && !found; i++) {
      g_prediction.restore_to_predicted();
      if (i == 0)
        cmd->buttons_ &= ~IN_DUCK;
      else
        cmd->buttons_ |= IN_DUCK;

      for (int z = 0; z < 8; z++) {
        g_prediction.simulate(cmd);
        if (on_ground())
          break;
        if (!is_surf_velocity(g_cl.m_local->m_velocity().m_z))
          continue;
        if (i == 0) {
          // standing already surfs, nothing to do.
          cmd->buttons_ = backup_buttons;
          g_prediction.restore_to_predicted();
          return;
        }
        m_should_surf = true;
        m_surf_ticks  = tick_count + z + 16;
        found         = true;
        break;
      }
    }
    if (!found)
      cmd->buttons_ = backup_buttons;
    g_prediction.restore_to_predicted();
  } else {
    cmd->buttons_ |= IN_DUCK;
    if (tick_count > m_surf_ticks && !is_surf_velocity(velocity.m_z))
      m_should_surf = false;
  }
}

// aim at a wall and press the key: finds the heights on that wall you can pixel surf at.
void c_pixelsurf::calculator(usercmd_t* cmd) {
  const auto& ctrl = g_ui.m_controls.pixelsurf;
  if (!ctrl.calc->m_value) {
    m_calc_valid = false;
    m_calc_results.clear();
    return;
  }

  const bool down    = ctrl.calc_key->value_->enabled;
  const bool pressed = down && !m_calc_key_down;
  m_calc_key_down    = down;
  if (!pressed)
    return;

  const auto   local = g_cl.m_local;
  const vector eye   = local->m_vec_origin() + local->m_view_offset();
  vector       angles(cmd->m_viewangles.m_x, cmd->m_viewangles.m_y, 0.f);
  const vector end = eye + angles.angle_vector() * 4096.f;

  ray_t ray;
  ray.initialize(eye, end);
  trace_world_only filter;
  trace_t          trace;
  g_interfaces.m_engine_trace->trace_ray(ray, MASK_PLAYERSOLID, &filter, &trace);

  if (trace.m_fraction >= 1.f || fabsf(trace.m_plane.normal.m_z) > 0.3f) {
    m_calc_valid = false;
    m_calc_results.clear();
    g_cl.chat_print("\x07" "61BA3B" "haven \x07" "FFFFFF" "| pixel surf calc: aim at a wall");
    return;
  }

  m_calc_point  = trace.m_end;
  m_calc_normal = trace.m_plane.normal;
  m_calc_valid  = true;
  calculator_solve(cmd);

  char msg[96];
  snprintf(msg, sizeof(msg), "\x07" "61BA3B" "haven \x07" "FFFFFF" "| pixel surf calc: %d spot%s found",
           static_cast<int>(m_calc_results.size()), m_calc_results.size() == 1 ? "" : "s");
  g_cl.chat_print(msg);
}

void c_pixelsurf::calculator_solve(usercmd_t* cmd) {
  const auto& ctrl  = g_ui.m_controls.pixelsurf;
  const auto  local = g_cl.m_local;
  m_calc_results.clear();

  const int   range   = static_cast<int>(ctrl.calc_range->m_value);
  const float align   = local->maxs().m_x - 0.02197f; // hull pressed against the wall
  const float base_z  = floorf(m_calc_point.m_z);
  const vector& n     = m_calc_normal;
  const vector  into_wall(-n.m_x, -n.m_y, 0.f);
  const float   rot = deg_to_rad(into_wall.angle_to().m_y - cmd->m_viewangles.m_y);

  const int   backup_buttons = cmd->buttons_;
  const float backup_forward = cmd->forwardmove_, backup_side = cmd->sidemove_;

  const auto velocity_offset = g_netvars.m_offsets.dt_base_player.m_velocity;
  const auto ground_offset   = g_netvars.m_offsets.dt_base_player.m_ground_handle;

  for (int dz = -range; dz <= range; dz++) {
    // ground snaps the origin to brush height + DIST_EPSILON, so that's where feet end up.
    const float z = base_z + dz + 0.03125f;

    for (int duck = 0; duck < 2; duck++) {
      g_prediction.restore_to_predicted();

      // drop a copy of ourselves just above the candidate height, pressed into the wall.
      local->set_abs_origin({m_calc_point.m_x + n.m_x * align, m_calc_point.m_y + n.m_y * align, z + 1.f});
      local->get<vector>(velocity_offset) = {0.f, 0.f, -50.f};
      local->calculate_abs_velocity();
      local->flags() &= ~FL_ONGROUND;
      local->get<c_base_handle>(ground_offset).m_index = 0xFFFFFFFF;

      cmd->buttons_     = (backup_buttons & ~(IN_JUMP | IN_DUCK)) | (duck ? IN_DUCK : 0);
      cmd->forwardmove_ = cosf(rot) * 10.f;
      cmd->sidemove_    = -sinf(rot) * 10.f;

      int streak = 0;
      for (int t = 0; t < 12; t++) {
        g_prediction.simulate(cmd);
        if (on_ground())
          break; // real floor, not a pixel
        if (local->m_vec_origin().m_z < z - 2.f)
          break; // fell past it
        streak = is_surf_velocity(local->m_velocity().m_z) ? streak + 1 : 0;
        if (streak >= 3) {
          m_calc_results.push_back({{m_calc_point.m_x + n.m_x * 1.5f, m_calc_point.m_y + n.m_y * 1.5f, z}, duck != 0});
          break;
        }
      }
    }
  }

  cmd->buttons_     = backup_buttons;
  cmd->forwardmove_ = backup_forward;
  cmd->sidemove_    = backup_side;
  g_prediction.restore_to_predicted();
}

void c_pixelsurf::run(usercmd_t* cmd) {
  const auto& ctrl  = g_ui.m_controls.pixelsurf;
  const auto  local = g_cl.m_local;
  m_in_surf         = false;

  const bool surf_on = ctrl.enabled->m_value && ctrl.key->value_->enabled;
  if (!surf_on && !ctrl.calc->m_value) {
    m_should_surf = m_wall_detected = false;
    return;
  }
  if (!local || !local->is_alive() || local->move_type() != MOVETYPE_WALK ||
      g_local_move.edgebug_active()) {
    m_should_surf = m_wall_detected = false;
    return;
  }

  const int tick_count = g_interfaces.m_global_vars->m_tick_count;
  sim_globals_t globals;

  // haven's prediction leaves us a tick ahead, read the real current state.
  g_prediction.restore_to_predicted();
  const vector velocity = local->m_velocity();

  if (surf_on) {
    m_in_surf = !on_ground() && is_surf_velocity(velocity.m_z);
    auto_align(cmd);
    pixel_surf(cmd, velocity, tick_count);
  } else
    m_should_surf = m_wall_detected = false;

  calculator(cmd);
  g_prediction.restore_to_predicted();
}

void c_pixelsurf::draw() {
  const auto& ctrl = g_ui.m_controls.pixelsurf;
  if (!g_cl.m_local || !g_interfaces.m_engine->is_in_game())
    return;

  if (m_in_surf) {
    int w, h;
    g_interfaces.m_engine->get_screen_size(w, h);
    g_render.text(g_render.m_fonts.esp.name, {w * 0.5f, h * 0.5f + 24.f}, "pixel surf",
                  g_ui.m_theme, e_text_alignment::text_align_center);
  }

  if (!ctrl.calc->m_value || !m_calc_valid)
    return;

  vector screen;
  if (math::world_to_screen(m_calc_point, screen))
    g_render.outlined_rect({screen.m_x - 3.f, screen.m_y - 3.f}, {6.f, 6.f}, {255, 255, 255});

  for (const auto& point : m_calc_results) {
    if (!math::world_to_screen(point.pos, screen))
      continue;
    const color col = point.duck ? color(255, 200, 40) : g_ui.m_theme;
    g_render.filled_rect({screen.m_x - 2.f, screen.m_y - 2.f}, {5.f, 5.f}, col);
    g_render.text(g_render.m_fonts.esp.flags, {screen.m_x + 6.f, screen.m_y - 5.f},
                  point.duck ? "ps duck" : "ps", col);
  }
}
