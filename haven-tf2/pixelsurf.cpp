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

bool c_pixelsurf::is_surf_velocity(float z) const { return fabsf(z - surf_velocity()) < 0.1f; }

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

// ---------------------------------------------------------------------------
// calculator / finder
// ---------------------------------------------------------------------------

static constexpr const char* k_points_file = "haven_pixelsurf_points.json";
static constexpr float       k_air_duck_shift = 20.f; // origin rises by stand - duck hull height when ducking in air

bool c_pixelsurf::trace_view(usercmd_t* cmd, trace_t& trace) const {
  const auto   local = g_cl.m_local;
  const vector eye   = local->m_vec_origin() + local->m_view_offset();
  vector       angles(cmd->m_viewangles.m_x, cmd->m_viewangles.m_y, 0.f);
  ray_t        ray;
  ray.initialize(eye, eye + angles.angle_vector() * 4096.f);
  trace_world_only filter;
  g_interfaces.m_engine_trace->trace_ray(ray, MASK_PLAYERSOLID, &filter, &trace);
  return trace.m_fraction < 1.f;
}

std::string c_pixelsurf::current_map() const {
  std::string name = g_interfaces.m_engine->get_level_name() ? g_interfaces.m_engine->get_level_name() : "";
  if (const auto slash = name.find_last_of("/\\"); slash != std::string::npos)
    name = name.substr(slash + 1);
  if (const auto dot = name.rfind(".bsp"); dot != std::string::npos)
    name = name.substr(0, dot);
  return name;
}

// hold the key: first press picks the wall, dragging extends a vertical line along it.
// release: every integer height on that line is tested.
void c_pixelsurf::finder(usercmd_t* cmd) {
  const auto& ctrl = g_ui.m_controls.pixelsurf;
  const bool  held = ctrl.calc_key->value_->enabled;

  if (held) {
    trace_t trace;
    const bool hit = trace_view(cmd, trace);
    if (!m_finder_held) {
      // first press: pick the wall.
      m_finder_valid = hit && fabsf(trace.m_plane.normal.m_z) < 0.3f;
      if (m_finder_valid) {
        m_finder_start = m_finder_end = trace.m_end;
        m_finder_normal = trace.m_plane.normal;
        m_finder_disp   = trace.m_surface.name && strstr(trace.m_surface.name, "disp");
        m_found.clear();
      }
    } else if (m_finder_valid && hit) {
      // keep the line vertical on the wall we picked, only follow the aim height.
      m_finder_end = {m_finder_start.m_x, m_finder_start.m_y, trace.m_end.m_z};
    }
    m_finder_held = true;
    return;
  }

  if (!m_finder_held)
    return;
  m_finder_held = false;
  if (!m_finder_valid) {
    g_cl.chat_print("\x07" "61BA3B" "haven \x07" "FFFFFF" "| pixel surf calc: aim at a wall");
    return;
  }

  float min_z = fminf(m_finder_start.m_z, m_finder_end.m_z);
  float max_z = fmaxf(m_finder_start.m_z, m_finder_end.m_z);
  if (max_z - min_z < 1.f) {
    // a click instead of a drag: search around the point.
    min_z -= ctrl.calc_range->m_value;
    max_z += ctrl.calc_range->m_value;
  }
  finder_solve(cmd, min_z, max_z);

  char msg[96];
  snprintf(msg, sizeof(msg), "\x07" "61BA3B" "haven \x07" "FFFFFF" "| pixel surf calc: %d spot%s found",
           static_cast<int>(m_found.size()), m_found.size() == 1 ? "" : "s");
  g_cl.chat_print(msg);
}

// lobotomy's finder test: park the player airborne at exactly the scan height with the hull
// flush against the wall, run a single tick pushing into it, and look for the surf velocity.
//   mode 0: standing, still    mode 1: rising (head bounce)    mode 2: ducked
bool c_pixelsurf::test_height(usercmd_t* cmd, const vector& wall, const vector& n, float z, int mode) {
  const auto  local = g_cl.m_local;
  // hull edge a hair inside the wall plane, displacements need it a hair outside instead.
  const float align = m_finder_disp ? local->maxs().m_x + 0.001f : local->maxs().m_x - 0.02197f;
  const auto  ground_offset = g_netvars.m_offsets.dt_base_player.m_ground_handle;

  g_prediction.restore_to_predicted();

  // SetupMove starts from the network origin, so that is what has to move. abs origin is set
  // at the wall point the way the source does it.
  local->m_vec_origin() = {wall.m_x + n.m_x * align, wall.m_y + n.m_y * align, z};
  local->set_abs_origin({wall.m_x, wall.m_y, z});
  local->set_abs_velocity({0.f, 0.f, mode == 1 ? -surf_velocity() * 3.f : 0.f});
  local->flags() &= ~FL_ONGROUND;
  local->get<c_base_handle>(ground_offset).m_index = 0xFFFFFFFF;

  const vector into_wall(-n.m_x, -n.m_y, 0.f);
  const float  rot  = deg_to_rad(into_wall.angle_to().m_y - cmd->m_viewangles.m_y);
  cmd->buttons_     = mode == 2 ? IN_DUCK : 0;
  cmd->forwardmove_ = cosf(rot) * 6.f;
  cmd->sidemove_    = -sinf(rot) * 6.f;

  g_prediction.simulate(cmd);
  return is_surf_velocity(local->m_velocity().m_z);
}

void c_pixelsurf::finder_solve(usercmd_t* cmd, float min_z, float max_z) {
  m_found.clear();
  const int   backup_buttons = cmd->buttons_;
  const float backup_forward = cmd->forwardmove_, backup_side = cmd->sidemove_;
  const vector& n = m_finder_normal;
  const vector  wall(m_finder_start.m_x, m_finder_start.m_y, 0.f);

  // scan top to bottom in 1 unit steps, at the height ground snapping leaves feet on a pixel.
  const float top = max_z;
  for (float lerp = 0.f; lerp <= top - min_z; lerp += 1.f) {
    const float z = top < 0.f ? static_cast<float>(static_cast<int>(top)) - 0.97125f - lerp
                              : static_cast<float>(static_cast<int>(top)) + 0.03125f - lerp;

    bool found = false, duck = false;
    if (test_height(cmd, wall, n, z, 0) || test_height(cmd, wall, n, z, 1))
      found = true;
    else if (test_height(cmd, wall, n, z, 2))
      found = duck = true;
    if (!found)
      continue;

    // adjacent heights are the same pixel, keep the first (highest).
    if (!m_found.empty() && fabsf(m_found.back().pos.m_z - z) < 1.5f)
      continue;
    m_found.push_back({{wall.m_x + n.m_x * 1.5f, wall.m_y + n.m_y * 1.5f, z}, n, duck, ""});
  }

  cmd->buttons_     = backup_buttons;
  cmd->forwardmove_ = backup_forward;
  cmd->sidemove_    = backup_side;
  g_prediction.restore_to_predicted();
}

// aim at a marker and press: unsaved -> saved for this map, saved -> removed.
void c_pixelsurf::save_key(usercmd_t* cmd) {
  const auto& ctrl    = g_ui.m_controls.pixelsurf;
  const bool  down    = ctrl.save_key->value_->enabled;
  const bool  pressed = down && !m_save_key_down;
  m_save_key_down     = down;
  if (!pressed)
    return;

  int w, h;
  g_interfaces.m_engine->get_screen_size(w, h);
  const vector_2d centre(w * 0.5f, h * 0.5f);

  const std::string map = current_map();
  float best_dist = 30.f;
  int   best_found = -1, best_saved = -1;

  vector screen;
  for (int i = 0; i < static_cast<int>(m_found.size()); i++) {
    if (!math::world_to_screen(m_found[i].pos, screen))
      continue;
    const float d = hypotf(screen.m_x - centre.m_x, screen.m_y - centre.m_y);
    if (d < best_dist) best_dist = d, best_found = i, best_saved = -1;
  }
  for (int i = 0; i < static_cast<int>(m_saved.size()); i++) {
    if (m_saved[i].map != map || !math::world_to_screen(m_saved[i].pos, screen))
      continue;
    const float d = hypotf(screen.m_x - centre.m_x, screen.m_y - centre.m_y);
    if (d < best_dist) best_dist = d, best_saved = i, best_found = -1;
  }

  if (best_saved >= 0) {
    m_saved.erase(m_saved.begin() + best_saved);
    save_points();
    g_cl.chat_print("\x07" "61BA3B" "haven \x07" "FFFFFF" "| pixel surf point removed");
  } else if (best_found >= 0) {
    auto point = m_found[best_found];
    point.map  = map;
    m_saved.push_back(point);
    m_found.erase(m_found.begin() + best_found);
    save_points();
    char msg[96];
    snprintf(msg, sizeof(msg), "\x07" "61BA3B" "haven \x07" "FFFFFF" "| pixel surf point saved (z %.2f)", point.pos.m_z);
    g_cl.chat_print(msg);
  }
}

void c_pixelsurf::load_points() {
  m_points_loaded = true;
  m_saved.clear();
  try {
    std::ifstream stream(k_points_file);
    if (!stream.good())
      return;
    nlohmann::json json;
    stream >> json;
    for (auto& [map, points] : json.items())
      for (auto& p : points)
        m_saved.push_back({{p.value("x", 0.f), p.value("y", 0.f), p.value("z", 0.f)},
                           {p.value("nx", 0.f), p.value("ny", 0.f), p.value("nz", 0.f)},
                           p.value("duck", false),
                           map});
  } catch (std::exception& e) {
    printf_s(__FUNCTION__ " %s\n", e.what());
  }
}

void c_pixelsurf::save_points() const {
  try {
    nlohmann::json json = nlohmann::json::object();
    for (const auto& p : m_saved)
      json[p.map].push_back({{"x", p.pos.m_x}, {"y", p.pos.m_y}, {"z", p.pos.m_z},
                             {"nx", p.normal.m_x}, {"ny", p.normal.m_y}, {"nz", p.normal.m_z},
                             {"duck", p.duck}});
    std::ofstream stream(k_points_file);
    stream << std::setw(2) << json << std::endl;
  } catch (std::exception& e) {
    printf_s(__FUNCTION__ " %s\n", e.what());
  }
}

// simulate the jump types from the ground we're standing on and keep the feet height per tick,
// so points can be labelled with what gets you there. refreshed when the ground height changes.
void c_pixelsurf::update_arcs(usercmd_t* cmd, int tick_count) {
  const auto local = g_cl.m_local;
  if (!on_ground())
    return;
  const float ground_z = local->m_vec_origin().m_z;
  if (fabsf(ground_z - m_arc_ground_z) < 0.5f && tick_count < m_arc_next_tick && !m_arcs.empty())
    return;
  m_arc_ground_z  = ground_z;
  m_arc_next_tick = tick_count + 66;
  m_arcs.clear();

  usercmd_t sim        = *cmd;
  sim.forwardmove_     = sim.sidemove_ = sim.upmove_ = 0.f;

  // jump: release everything for a tick so the jump press registers, then jump once.
  {
    jump_arc_t arc{"jump", {}};
    g_prediction.restore_to_predicted();
    sim.buttons_ = 0;
    g_prediction.simulate(&sim);
    sim.buttons_ = IN_JUMP;
    g_prediction.simulate(&sim);
    sim.buttons_ = 0;
    for (int t = 0; t < 128 && !on_ground(); t++) {
      arc.rel_z.push_back(local->m_vec_origin().m_z - ground_z);
      g_prediction.simulate(&sim);
    }
    if (!arc.rel_z.empty())
      m_arcs.push_back(std::move(arc));
  }

  // drop: walking off the edge, plain gravity.
  {
    static auto sv_gravity = g_interfaces.m_cvar->find_var("sv_gravity");
    const float gravity    = sv_gravity ? sv_gravity->m_value.m_float_value : 800.f;
    const float dt         = g_interfaces.m_global_vars->m_interval_per_tick;
    jump_arc_t  arc{"drop", {}};
    float       v = 0.f, z = 0.f;
    for (int t = 0; t < 128; t++) {
      v -= gravity * dt * 0.5f;
      z += v * dt;
      v -= gravity * dt * 0.5f;
      arc.rel_z.push_back(z);
    }
    m_arcs.push_back(std::move(arc));
  }

  g_prediction.restore_to_predicted();
}

// same tolerance as the source calculators: feet within a couple hundredths below the pixel.
static bool height_matches(float feet, float target) {
  const float d = target - feet;
  return d > -0.01f && d < 0.03f;
}

std::string c_pixelsurf::reach_label(const calc_point_t& point) const {
  if (m_arcs.empty() || m_arc_ground_z == FLT_MAX)
    return {};
  std::string label;
  const auto add = [&](const char* name) {
    if (!label.empty())
      label += ", ";
    label += name;
  };
  for (const auto& arc : m_arcs) {
    bool plain = false, crouch = false;
    for (const float rel : arc.rel_z) {
      const float feet = m_arc_ground_z + rel;
      plain |= height_matches(feet, point.pos.m_z);
      crouch |= height_matches(feet + k_air_duck_shift, point.pos.m_z);
    }
    if (plain)
      add(arc.name);
    if (crouch)
      add((std::string(arc.name) + " + crouch").c_str());
  }
  return label;
}

void c_pixelsurf::run(usercmd_t* cmd) {
  const auto& ctrl  = g_ui.m_controls.pixelsurf;
  const auto  local = g_cl.m_local;
  m_in_surf         = false;

  const bool surf_on = ctrl.enabled->m_value && ctrl.key->value_->enabled;
  if (!surf_on && !ctrl.calc->m_value) {
    m_should_surf = m_wall_detected = m_finder_held = false;
    return;
  }
  if (!local || !local->is_alive() || local->move_type() != MOVETYPE_WALK ||
      g_local_move.edgebug_active()) {
    m_should_surf = m_wall_detected = m_finder_held = false;
    return;
  }
  if (!m_points_loaded)
    load_points();

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

  if (ctrl.calc->m_value) {
    finder(cmd);
    save_key(cmd);
    update_arcs(cmd, tick_count);
  } else
    m_finder_held = false;

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

  if (!ctrl.calc->m_value)
    return;

  vector screen, screen2;

  // the line being dragged along the wall.
  if (m_finder_held && m_finder_valid && math::world_to_screen(m_finder_start, screen) &&
      math::world_to_screen(m_finder_end, screen2)) {
    g_render.line({screen.m_x, screen.m_y}, {screen2.m_x, screen2.m_y}, {255, 255, 255});
    g_render.filled_rect({screen.m_x - 2.f, screen.m_y - 2.f}, {5.f, 5.f}, {255, 255, 255});
    g_render.filled_rect({screen2.m_x - 2.f, screen2.m_y - 2.f}, {5.f, 5.f}, {255, 255, 255});
  }

  const std::string map = current_map();
  const auto draw_point = [&](const calc_point_t& point, bool saved) {
    if (!math::world_to_screen(point.pos, screen))
      return;
    const color col = point.duck ? color(255, 200, 40) : g_ui.m_theme;
    if (saved) {
      g_render.filled_rect({screen.m_x - 3.f, screen.m_y - 3.f}, {7.f, 7.f}, col);
      g_render.outlined_rect({screen.m_x - 4.f, screen.m_y - 4.f}, {9.f, 9.f}, {0, 0, 0});
    } else
      g_render.outlined_rect({screen.m_x - 3.f, screen.m_y - 3.f}, {7.f, 7.f}, col);

    std::string label = point.duck ? "ps duck" : "ps";
    if (const auto reach = reach_label(point); !reach.empty())
      label += " [" + reach + "]";
    g_render.text(g_render.m_fonts.esp.flags, {screen.m_x + 8.f, screen.m_y - 5.f}, label.c_str(), col);
  };

  for (const auto& point : m_saved)
    if (point.map == map)
      draw_point(point, true);
  for (const auto& point : m_found)
    draw_point(point, false);
}
