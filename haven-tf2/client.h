#pragma once

class c_client {
public:
  const char*    m_user = "user";
  usercmd_t*     m_cmd;
  c_base_player* m_local;
  c_base_weapon* m_weapon;
  vector         m_shoot_pos;
  bool           m_unloading = false, m_debug_build = true;
  HWND           m_hwnd = nullptr;
  view_matrix    m_view_matrix;

  void init();
  void unload();
  void on_move(usercmd_t* cmd);
  // prints to the in-game chat (supports \x07RRGGBB colour codes), falls back to the console.
  void chat_print(const char* msg);
};
inline c_client g_cl;
