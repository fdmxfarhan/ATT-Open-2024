int buff[8];
int counter, GY;
int blocks;
int angle_ball, direction_ball, distance_ball, x_ball, y_ball;
int vm = 250;
int v;
// bool is_ball;
int LCD_Print_Mode = 0;
bool already_shooted = false;
int out_cnt = 0, comeback_cnt = 0;
int shr, shl, shb, dif , shf;
int shoot_sens, sensor[9];
bool LDR_F, LDR_B, LDR_R, LDR_L;
uint16_t LDR_SET_F = 0, LDR_SET_R = 0, LDR_SET_B = 0, LDR_SET_L = 0;
bool is_ball, is_goal, Ball_In_Kicker = false;
float K_P = 0, K_I = 0, K_D = 0, Heading, last_Heading , lastTime;
int angle_goal, direction_goal, distance_goal, x_goal, y_goal;
bool look_back = false;
bool turned_back = false;
int shoot_cnt = 0;
bool arrived_to_goal = false;
int turned_back_cnt = 0;
bool Ball_in_kick_init = false;
bool rl;
#define Walll_Distance 2540
#define Wallr_Distance 2700
#define back_Distance 1500
#define LDR_Sensitivity 500
#define out_cnt_timeout 40
#define out_move 5
/////pi 1
#define x_robot1 111
#define y_robot1 150
/////
/////pi 2
#define x_robot2 114
#define y_robot2 153
#define near_dis  120
/////
int x_robot, y_robot;
#define spinon digitalWrite(PC14, 1)
#define spinoff digitalWrite(PC14, 0)
#define shootsen 50
void setup() {
  inti();
}

void loop() {
  AI_2();
}
