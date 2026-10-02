#pragma once
class prediction
{
    float m_fOldCurrentTime, m_fOldFrameTime;
    int m_nOldTickCount;
    CMoveData m_MoveData = {};

public:
    int get_tickbase(usercmd_t* pCmd, c_base_player* pLocal);
    void start();
    void finish();

    // runs a single tick of movement for the local player using cmd. does not touch globals.
    void simulate(usercmd_t* cmd);
    // restores the local player to the last engine-predicted frame.
    void restore_to_predicted();
} inline g_prediction;
