#pragma once

class c_pixelsurf
{
    struct calc_point_t {
        vector pos;
        bool   duck;
    };

    // auto align
    bool   m_wall_detected = false;
    float  m_align_start   = 0.f;
    vector m_wall_normal   = {};

    // pixel surf
    bool m_should_surf = false;
    int  m_surf_ticks  = 0;
    bool m_in_surf     = false;

    // calculator
    bool                      m_calc_valid      = false;
    bool                      m_calc_key_down   = false;
    vector                    m_calc_point      = {};
    vector                    m_calc_normal     = {};
    std::vector<calc_point_t> m_calc_results    = {};

    float surf_velocity() const;
    bool  is_surf_velocity(float z) const;

    void auto_align(usercmd_t* cmd);
    void pixel_surf(usercmd_t* cmd, const vector& velocity, int tick_count);
    void calculator(usercmd_t* cmd);
    void calculator_solve(usercmd_t* cmd);

public:
    // call after prediction, on the final command.
    void run(usercmd_t* cmd);
    void draw();

    bool in_pixel_surf() const { return m_in_surf; }
} inline g_pixelsurf;
