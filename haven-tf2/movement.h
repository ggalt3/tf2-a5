#pragma once
class c_movement
{
    enum eb_mode_type { eb_still, eb_user, eb_auto_strafe, eb_steer };

    struct edgebug_cmd_t {
        vector viewangles;
        float  forwardmove;
        float  sidemove;
        int    buttons;
        vector origin;
    };

    // edgebug state
    vector        m_eb_velocity_backup = {};
    int           m_eb_flags           = 0;
    bool          m_eb_detected        = false;
    bool          m_eb_duck            = false;
    int           m_eb_lock_ticks      = 0;
    int           m_eb_current_tick    = 0;
    int           m_eb_search_mode     = 0;
    int           m_eb_stack_count     = 0;
    int           m_eb_stack_window    = 0;
    vector        m_eb_target_origin   = {}; // predicted position of the edgebug
    edgebug_cmd_t m_eb_cmds[64]        = {};

    bool edgebug_check(usercmd_t* cmd);
    void edgebug_auto_strafe(usercmd_t* cmd);
    void edgebug_correct_movement(usercmd_t* cmd, vector wish_angle, vector old_angles);
    void edgebug_reset();

public:
    void bhop();
    void auto_strafe(float* view);
    void correct_movement(vector old);

    // call before engine prediction runs for this cmd.
    void edgebug_pre();
    // call after engine prediction has been finished/restored.
    void edgebug_post();
    // call from the ApplyMouse hook.
    void edgebug_mouse_lock(float& x, float& y);

    bool edgebug_active() const { return m_eb_detected; }
    // set when edgebug_post changed cmd->m_viewangles and wants the engine view to follow.
    bool m_eb_view_override = false;

    int m_switch = 1;
    float m_old_yaw = 0;
} inline g_local_move;
