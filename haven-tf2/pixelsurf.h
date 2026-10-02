#pragma once

class c_pixelsurf
{
    struct calc_point_t {
        vector      pos;    // on the wall surface, feet height
        vector      normal; // wall normal
        bool        duck;   // needs duck to hold the surf
        std::string map;    // empty for unsaved finder results
    };

    struct jump_arc_t {
        const char*        name;
        std::vector<float> rel_z; // feet height per tick relative to the ground we start on
    };

    // auto align
    bool   m_wall_detected = false;
    float  m_align_start   = 0.f;
    vector m_wall_normal   = {};

    // pixel surf
    bool m_should_surf = false;
    int  m_surf_ticks  = 0;
    bool m_in_surf     = false;

    // auto jump
    int m_auto_duck_until    = 0;
    int m_auto_jump_cooldown = 0;

    // finder: hold the key and drag a vertical line along a wall
    bool   m_finder_held   = false;
    bool   m_finder_valid  = false;
    bool   m_finder_disp   = false;
    vector m_finder_start  = {};
    vector m_finder_end    = {};
    vector m_finder_normal = {};
    std::vector<calc_point_t> m_found = {};

    // saved points (all maps), persisted to disk
    bool                      m_points_loaded = false;
    bool                      m_save_key_down = false;
    std::vector<calc_point_t> m_saved         = {};

    // jump arcs from the ground we're standing on, used to label points with how to reach them
    std::vector<jump_arc_t> m_arcs          = {};
    float                   m_arc_ground_z  = FLT_MAX;
    int                     m_arc_next_tick = 0;

    float surf_velocity() const;
    bool  is_surf_velocity(float z) const;
    bool  trace_view(usercmd_t* cmd, trace_t& trace) const;
    std::string current_map() const;

    void auto_align(usercmd_t* cmd);
    void pixel_surf(usercmd_t* cmd, const vector& velocity, int tick_count);
    void auto_jump(usercmd_t* cmd, int tick_count);

    void finder(usercmd_t* cmd);
    void finder_solve(usercmd_t* cmd, float min_z, float max_z);
    bool test_height(usercmd_t* cmd, const vector& wall, const vector& normal, float z, int mode);

    void save_key(usercmd_t* cmd);
    void load_points();
    void save_points() const;

    void update_arcs(usercmd_t* cmd, int tick_count);
    std::string reach_label(const calc_point_t& point) const;

public:
    // call after prediction, on the final command.
    void run(usercmd_t* cmd);
    void draw();

    bool in_pixel_surf() const { return m_in_surf; }
} inline g_pixelsurf;
