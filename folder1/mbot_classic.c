

int main() {


  //Wheel Speed PID Controller gains
  struct mbot_ctlr_cfg_t ctlr_cfg = {
    //Right Wheel PID
    (struct mbot_pid_cfg_t) {
      0.1,
      0.001,
      0.01,
      MAIN_LOOP_PERIOD * If_mult,
    },
    //Left Wheel PID
    (struct mbot_pid_cfg_t) {
      0.1,
      0.001,
      0.01,
      MAIN_LOOP_PERIOD * If_mult,
    },
    //Back
    (struct mbot_pid_cfg_t) {0, 0, 0, 0},
    //Vx
    (struct mbot_pid_cfg_t) {
      0.1,
      0.001,
      0.01,
      MAIN_LOOP_PERIOD * If_mult,
    },
    //Vy
    (struct mbot_pid_cfg_t) {0, 0, 0, 0},
    //Wz
    (struct mbot_pid_cfg_t) {
      0.5,
      0.01,
      0.02,
      MAIN_LOOP_PERIOD * If_mult,
    },
};
  






  
  
  }



  
}
