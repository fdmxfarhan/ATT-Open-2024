#include <Wire.h>
#include <PixyI2C.h>
PixyI2C pixy;
#include <Adafruit_SH1106_STM32.h>
Adafruit_SH1106 display(-1);

#define spinon digitalWrite(PC14, 1)
#define spinoff digitalWrite(PC14, 0)

void inti() {
  // -------------------- Motor Settings
  pinMode(PB15, OUTPUT);
  pinMode(PB14, OUTPUT);
  pinMode(PB13, OUTPUT);
  pinMode(PB12, OUTPUT);
  pinMode(PC14, OUTPUT);  //////SPIN
  pinMode(PC15, OUTPUT);  //////SHOOT
  pinMode(PA8, PWM);
  pinMode(PB8, PWM);
  pinMode(PB7, PWM);
  pinMode(PB6, PWM);
  motor(0, 0, 0, 0);
  pinMode(PA12, INPUT_PULLUP);
  // -------------------- OLED Display Settings
  display.begin(0x2, 0x3C);
  display.setTextColor(WHITE);
  display.setTextSize(1);
  // -------------------- Loading
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 20);
  display.print("Loading");
  display.display();
  delay(500);
  display.print(".");
  display.display();
  delay(500);
  display.print(".");
  display.display();
  delay(500);
  display.print(".");
  display.display();
  delay(500);
  display.setTextSize(1);
  // -------------------- Pixy Settings
  pixy.init();
  // -------------------- Serial Settings for GY-25
  Serial1.begin(115200);
  Serial1.write(0xA5);
  Serial1.write(0x54);
  delay(500);
  Serial1.write(0xA5);
  Serial1.write(0x51);
  set_ldr();
  // Serial1.write(0XA5);
  // Serial1.write(0X55);

  if(digitalRead(PA12)){
    x_robot = x_robot1;
    y_robot = y_robot1;
  }
  else{
    x_robot = x_robot2;
    y_robot = y_robot2;
  }
  // display.clearDisplay();
  // display.print("go");
  // display.display();
}
void sensors() {
  // -------------------- Set Button
  if (digitalRead(PB5)) {
    digitalWrite(PC13, 0);
    if (LCD_Print_Mode == 0) {
      Serial1.write(0XA5);
      Serial1.write(0X55);
    } else if (LCD_Print_Mode == 1) set_ldr();
    while (digitalRead(PB5))
      ;
    digitalWrite(PC13, 1);
  }
  if (digitalRead(PB4)) {
    digitalWrite(PC13, 0);
    LCD_Print_Mode++;
    LCD_Print_Mode %= 2;
    while (digitalRead(PB4))
      ;
    digitalWrite(PC13, 1);
  }
  // if (digitalRead(PA15)) {
  //   digitalWrite(PC13, 0);
  //   shoot_key();
  //   while (digitalRead(PA15))
  //     ;
  //   digitalWrite(PC13, 1);
  // }
  // -------------------- GY-25 Read Data
  Serial1.write(0xA5);
  Serial1.write(0x51);
  while (true) {
    buff[counter] = Serial1.read();
    if (counter == 0 && buff[0] != 0xAA) break;
    counter++;
    if (counter == 8) {
      counter = 0;
      if (buff[0] == 0xAA && buff[7] == 0x55) {
        GY = (int16_t)(buff[1] << 8 | buff[2]) / 100;
        if(GY > 180) GY -= 360;
        if(GY <-180) GY += 360;
        if (look_back) {
          if (GY > 0) {
            GY = GY - 180;
          } else {
            GY = GY + 180;
          }
        }
      }
    }
  }
  // -------------------- Sharp Read Data
  sensor[0] = analogRead(PA0);
  sensor[1] = analogRead(PA1);
  sensor[2] = analogRead(PA2);
  sensor[3] = analogRead(PA3) - LDR_SET_F;
  sensor[4] = analogRead(PA4) - LDR_SET_B;
  sensor[5] = analogRead(PA5) - LDR_SET_R;
  sensor[6] = analogRead(PA6);
  sensor[7] = analogRead(PA7) - LDR_SET_L;
  sensor[8] = analogRead(PA8); 
  if (sensor[3] > LDR_Sensitivity) LDR_F = true;
  else LDR_F = false;
  if (sensor[4] > LDR_Sensitivity) LDR_B = true;
  else LDR_B = false;
  if (sensor[5] > LDR_Sensitivity) LDR_R = true;
  else LDR_R = false;
  if (sensor[7] > LDR_Sensitivity) LDR_L = true;
  else LDR_L = false;
  shb = sensor[1];
  shr = sensor[0];
  shl = sensor[2];
  shf = sensor[6];
  dif = (shl - shr) / 5;
  // -------------------- Shoot Sensor Read Data
  shoot_sens = analogRead(PB0);
  if (shoot_sens < shootsen) Ball_In_Kicker = true;  //>
  else {
    Ball_In_Kicker = false;
    already_shooted = false;
  }

  // -------------------- Pixy Read Data
  uint16_t blocks;
  blocks = pixy.getBlocks();
  is_ball = false;
  is_goal = false;
  if (blocks) {
    for (int i = 0; i < blocks; i++) {
      if (pixy.blocks[i].signature == 1) {
        x_ball = pixy.blocks[i].y;
        y_ball = pixy.blocks[i].x;
        angle_ball = get_angle(x_ball, y_ball);
        direction_ball = get_direction(angle_ball);
        distance_ball = sqrt(pow(x_ball - x_robot, 2) + pow(y_ball - y_robot, 2));
        is_ball = true;
      }
      if (pixy.blocks[i].signature == 2) {
        x_goal = pixy.blocks[i].y;
        y_goal = pixy.blocks[i].x;
        angle_goal = get_angle(x_goal, y_goal);
        direction_goal = get_direction(angle_goal);
        distance_goal = sqrt(pow(x_goal - x_robot, 2) + pow(y_goal - y_robot, 2));
        is_goal = true;
      }
    }
  }
}
int get_angle(int x, int y) {
  int angle = atan2(y - y_robot, x - x_robot) * 180 / PI;
  angle += 90;
  if (angle < 0) angle += 360;
  return angle;
}
int get_direction(int angle) {
  int direction;
  for (int i = 0; i < 16; i++) {
    if ((angle - 11.25 >= i * 22.5) && (angle - 11.25 < (i + 1) * 22.5)) direction = i + 1;
  }
  if (angle <= 11.25 || angle >= 348.5) direction = 0;
  return direction;
}
void print_all() {
  display.clearDisplay();

  if (LCD_Print_Mode == 0) {
    // --------------------- Pixy Print
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print("X:");
    display.println(x_ball);
    display.print("Y:");
    display.println(y_ball);
    display.print("GY:");
    display.println(GY);
    display.print("dis2:");
    display.println(distance_goal);
    display.print("ang:");
    display.println(angle_ball);
    display.print("sh:");
    display.println(shoot_sens);
    display.print("dis1:");
    display.println(distance_ball);
    if (is_goal) {
      display.println(angle_goal);
    } 
    else {
      display.println("No");
    }


  } else if (LCD_Print_Mode == 1) {
    display.setTextSize(1);
    display.setCursor(0, 0);
    for (int i = 0; i < 9; i++) {
        display.print(i);
        display.print(" : ");
        display.println(sensor[i]);     
      
    }
    // display.print("dir_ball:");
    // display.println(direction_ball);
  }
  // --------------------- GY-25 Print Circle
  display.drawCircle(100, 32, 12, WHITE);
  display.drawLine(
    100 + sin(GY * PI / 180) * 9, 
    32 - cos(GY * PI / 180) * 9, 
    100 - sin(GY * PI / 180) * 9, 
    32 + cos(GY * PI / 180) * 9, 
    WHITE);
  display.fillCircle(
    100 - sin(GY * PI / 180) * 9, 
    32 + cos(GY * PI / 180) * 9,
    2, WHITE);
  // --------------------- Ball Print Circle
  if (Ball_In_Kicker)
    display.fillCircle(100, 32 - 16, 3, WHITE);
  else if (is_ball)
    display.fillCircle(100 + sin(angle_ball * PI / 180) * 20, 32 - cos(angle_ball * PI / 180) * 20, 3, WHITE);
  // --------------------- Out LDR
  // if(shr > Wallr_Distance) display.fillRect(105, 27, 3, 10, WHITE);
  // if(shl > Walll_Distance) display.fillRect(53, 27, 3, 10, WHITE);
  // if(shb > back_Distance ) display.fillRect(75, 57, 10, 3, WHITE);
  if (LDR_F) display.fillRect(100 - 2, 32 - 16, 4, 6, WHITE);
  if (LDR_B) display.fillRect(100 - 2, 32 + 10, 4, 6, WHITE);
  if (LDR_R) display.fillRect(100 + 10, 32 - 2, 6, 4, WHITE);
  if (LDR_L) display.fillRect(100 - 16, 32 - 2, 6, 4, WHITE);

  display.setCursor(118, 0);
  if(digitalRead(PA12)) display.print("1");
  else                  display.print("2");
  display.display();
}
void set_ldr() {
  LDR_SET_F = analogRead(PA3);
  LDR_SET_B = analogRead(PA4);
  LDR_SET_R = analogRead(PA5);
  LDR_SET_L = analogRead(PA7);
}
void shoot_key() {
  digitalWrite(PC15, 1);
  delay(70);
  digitalWrite(PC15, 0);
  delay(200);
}
void shoot() {
  if (shoot_cnt > 20) {
    shoot_cnt = 0;
    already_shooted = false;
  }
  if (already_shooted) {
    shoot_cnt++;
    return;
  }
  if (Ball_In_Kicker) {
    digitalWrite(PC15, 1);
    delay(70);
    digitalWrite(PC15, 0);
    delay(200);
    shoot_cnt = 0;
    already_shooted = true;
  }
}
void motor(int ML1, int ML2, int MR2, int MR1) {
  // if (GY > 6 && GY <= 30) GY = 30;
  // else if (GY > 30 && GY <= 80) GY = 80;
  // else if (GY > 80 && GY <= 180) GY = 150;
  // else if (GY < -6 && GY >= -30) GY = -30;
  // else if (GY < -30 && GY >= -80) GY = -80;
  // else if (GY < -80 && GY >= -180) GY = -150;
  ML1 += GY;
  ML2 += GY;
  MR2 += GY;
  MR1 += GY;
  ML1 *= 255;
  ML2 *= 255;
  MR2 *= 255;
  MR1 *= 255;
  if (ML1 > 65535) ML1 = 65535;
  if (ML2 > 65535) ML2 = 65535;
  if (MR2 > 65535) MR2 = 65535;
  if (MR1 > 65535) MR1 = 65535;
  if (ML1 < -65535) ML1 = -65535;
  if (ML2 < -65535) ML2 = -65535;
  if (MR2 < -65535) MR2 = -65535;
  if (MR1 < -65535) MR1 = -65535;
  if (ML1 > 0) {
    digitalWrite(PB15, 0);
    pwmWrite(PA8, ML1);
  } else {
    digitalWrite(PB15, 1);
    pwmWrite(PA8, ML1 + 65535);
  }
  if (ML2 > 0) {
    digitalWrite(PB14, 0);
    pwmWrite(PB8, ML2);
  } else {
    digitalWrite(PB14, 1);
    pwmWrite(PB8, ML2 + 65535);
  }
  if (MR2 > 0) {
    digitalWrite(PB13, 0);
    pwmWrite(PB7, MR2);
  } else {
    digitalWrite(PB13, 1);
    pwmWrite(PB7, MR2 + 65535);
  }
  if (MR1 > 0) {
    digitalWrite(PB12, 0);
    pwmWrite(PB6, MR1);
  } else {
    digitalWrite(PB12, 1);
    pwmWrite(PB6, MR1 + 65535);
  }
}
void move(int direction) {
  if (direction == 0) motor(v, v, -v, -v);
  if (direction == 1) motor(v, v / 2, -v, -v / 2);
  if (direction == 2) motor(v, 0, -v, 0);
  if (direction == 3) motor(v, -v / 2, -v, v / 2);
  if (direction == 4) motor(v, -v, -v, v);
  if (direction == 5) motor(v / 2, -v, -v / 2, v);
  if (direction == 6) motor(0, -v, 0, v);
  if (direction == 7) motor(-v / 2, -v, v / 2, v);
  if (direction == 8) motor(-v, -v, v, v);
  if (direction == 9) motor(-v, -v / 2, v, v / 2);
  if (direction == 10) motor(-v, 0, v, 0);
  if (direction == 11) motor(-v, v / 2, v, -v / 2);
  if (direction == 12) motor(-v, v, v, -v);
  if (direction == 13) motor(-v / 2, v, v / 2, -v);
  if (direction == 14) motor(0, v, 0, -v);
  if (direction == 15) motor(v / 2, v, -v / 2, -v);
}
void stop() {
  motor(0, 0, 0, 0);
}
void out_sharp() {
  if (shr > Wallr_Distance) {
    while (direction_ball < 8 && direction_ball > 0 && is_ball) {
      sensors();
      print_all();
      v = 180;
      if (shr > Wallr_Distance + 300) move(12);
      else stop();
    }
  }
  if (shl > Walll_Distance) {
    while (direction_ball > 8 && is_ball) {
      sensors();
      print_all();
      v = 180;
      if (shl > Walll_Distance + 300) move(4);
      else stop();
    }
  }
  ////////////////////////////////////////////

  if (shb > back_Distance) {
    while ((direction_ball >= 1 && direction_ball <= 9) && is_ball) {
      sensors();
      print_all();
      v = 180;
      if (shb > back_Distance + 280) move(0);  /// by mrm
      else stop();
    }
  }
}
void moveForSec(int dir, int sec) {
  for (int i = 0; i < sec; i++) {
    sensors();
    print_all();
    move(dir);
  }
}
void moveInside() {
  if (LDR_R && LDR_F) move(10);
  if (LDR_L && LDR_F) move(6);
  if (LDR_R && LDR_B) move(14);
  if (LDR_L && LDR_B) move(2);
  if (LDR_F) move(8);
  if (LDR_R) move(12);
  if (LDR_B) move(0);
  if (LDR_L) move(4);
}
void catch_ball() {
  GY = 0;
  spinon;
  v = 0.3 * vm;
  if (direction_ball == 0) {
    move(0);
  } else if (direction_ball >= 1 && direction_ball <= 8) {
    motor(0, 70, 70, 0);
  } else {
    motor(0, -70, -70, 0);
  }
}
void out() {
  if(Ball_In_Kicker && is_goal){
    return;
  }
  out_cnt = 0;
  // back_catch_ball();
  if (LDR_F && LDR_R) {
    v = 230;
    moveForSec(10, 5);
    while ((angle_ball < 170 || angle_ball > 280) && is_ball && out_cnt < out_cnt_timeout ) {
      sensors();
      print_all();
      v = 230;
      if (LDR_R || LDR_F || LDR_B || LDR_L) moveInside();
      else stop();
      out_cnt++;
    }
  } else if (LDR_F && LDR_L) {
    v = 230;
    moveForSec(6, 5);
    while ((angle_ball < 80 || angle_ball > 190) && is_ball && out_cnt < out_cnt_timeout) {
      sensors();
      print_all();
      v = 230;
      if (LDR_R || LDR_F || LDR_B || LDR_L) moveInside();
      else stop();
      out_cnt++;
    }
  } else if (LDR_B && LDR_L) {
    v = 230;
    moveForSec(2, 5);
    while (angle_ball > 100 && angle_ball < 350 && is_ball && out_cnt < out_cnt_timeout) {
      sensors();
      print_all();
      v = 230;
      if (LDR_R || LDR_F || LDR_B || LDR_L) moveInside();
      else stop();
      out_cnt++;
    }
  } 
  else if (LDR_B && LDR_R) {
    v = 210;
    moveForSec(14, 5);
    while (angle_ball > 10 && angle_ball < 260 && is_ball && out_cnt < out_cnt_timeout) {
      sensors();
      print_all();
      v = 210;
      if (LDR_R || LDR_F || LDR_B || LDR_L) moveInside();
      else stop();
      out_cnt++;
    }
  }
  else if(LDR_L && LDR_R){
    v = 200;
    moveForSec(0, 10);
  } 
  else if (LDR_F) {
    v = 210;
    moveForSec(8, 9);
    while ((angle_ball < 90 || angle_ball > 270) && is_ball && out_cnt < out_cnt_timeout) {
      sensors();
      print_all();
      v = 210;
      if (LDR_R || LDR_F || LDR_B || LDR_L) moveInside();
      else stop();
      out_cnt++;
    }
  } else if (LDR_R) {
    v = 210;
    moveForSec(12, 7);
    while (out_cnt < out_cnt_timeout) {
      sensors();
      print_all();
      // catch_ball();
      v = 210;
      if(LDR_R || LDR_F || LDR_B || LDR_L) moveInside();
      else stop();
      out_cnt++;
    }
  } 
  else if (LDR_L) {
    v = 210;
    moveForSec(4, out_move);
    while (out_cnt < out_cnt_timeout) {
      sensors();
      print_all();
      v = 210;
      if(LDR_R || LDR_F || LDR_B || LDR_L) moveInside();
      else stop();
      // catch_ball();
      out_cnt++;
    }
  }
  else if (LDR_B) {
    v = 210;
    moveForSec(0, out_move);
    while (angle_ball > 90 && angle_ball < 270 && is_ball && out_cnt < out_cnt_timeout) {
      sensors();
      print_all();
      v = 210;
      if (LDR_R || LDR_F || LDR_B || LDR_L) moveInside();
      else stop();
      out_cnt++;
    }
  } 
 
}
void strat_1(){
  if(((LDR_B) || (shb > back_Distance)) && is_ball){
    if(direction_ball == 0){
      move(0);
    }
    else if(direction_ball > 0 && direction_ball <=5){
      moveForSec(direction_ball, 7);
    }
    else if (direction_ball >= 11 && direction_ball <= 15) {
      moveForSec(direction_ball, 7);
    }
    else{
      // if(out_cnt > out_cnt_timeout){
      //   stop();
      // }
      // else{
      //   moveForSec(0, 2);
      // }
      stop();
    }
  }
}
void out_with_catch() {
  if(Ball_In_Kicker) return;
  out_cnt = 0;
  if (LDR_F && LDR_R) {
    v = 230;
    moveForSec(10, 5);
    while ((angle_ball < 170 || angle_ball > 280) && is_ball && out_cnt < out_cnt_timeout && !Ball_In_Kicker) {
      sensors();
      print_all();
      v = 230;
      if (LDR_R || LDR_F || LDR_B || LDR_L) moveInside();
      else stop();
      out_cnt++;
    }
  } else if (LDR_F && LDR_L) {
    v = 230;
    moveForSec(6, 5);
    while ((angle_ball < 80 || angle_ball > 190) && is_ball && out_cnt < out_cnt_timeout) {
      sensors();
      print_all();
      v = 230;
      if (LDR_R || LDR_F || LDR_B || LDR_L) moveInside();
      else stop();
      out_cnt++;
    }
  } else if (LDR_B && LDR_L) {
    v = 230;
    moveForSec(2, 5);
    while (angle_ball > 100 && angle_ball < 350 && is_ball && out_cnt < out_cnt_timeout) {
      sensors();
      print_all();
      v = 230;
      if (LDR_R || LDR_F || LDR_B || LDR_L) moveInside();
      else stop();
      out_cnt++;
    }
  } else if (LDR_B && LDR_R) {
    v = 210;
    moveForSec(14, 5);
    while (angle_ball > 10 && angle_ball < 260 && is_ball && out_cnt < out_cnt_timeout) {
      sensors();
      print_all();
      v = 210;
      if (LDR_R || LDR_F || LDR_B || LDR_L) moveInside();
      else stop();
      out_cnt++;
    }
  } else if (LDR_F) {
    v = 210;
    moveForSec(8, 11);
    while ((angle_ball < 90 || angle_ball > 270) && is_ball && out_cnt < out_cnt_timeout) {
      sensors();
      print_all();
      v = 210;
      if (LDR_R || LDR_F || LDR_B || LDR_L) moveInside();
      else stop();
      out_cnt++;
    }
  } else if (LDR_R) {
    v = 210;
    moveForSec(12, 10);
    while (out_cnt < out_cnt_timeout && !Ball_In_Kicker) {
      sensors();
      print_all();
      catch_ball();
      // v = 210;
      // if(LDR_R || LDR_F || LDR_B || LDR_L) moveInside();
      // else stop();
      out_cnt++;
    }
  } else if (LDR_B) {
    v = 210;
    moveForSec(0, 11);
    while (angle_ball > 90 && angle_ball < 270 && is_ball && out_cnt < out_cnt_timeout) {
      sensors();
      print_all();
      // v = 210;
      // if (LDR_R || LDR_F || LDR_B || LDR_L) moveInside();
      // else stop();
      back_catch_ball();
      out_cnt++;
    }
  } else if (LDR_L) {
    v = 210;
    moveForSec(4, 10);
    while (out_cnt < out_cnt_timeout && !Ball_In_Kicker) {
      sensors();
      print_all();
      // v = 210;
      // if(LDR_R || LDR_F || LDR_B || LDR_L) moveInside();
      // else stop();
      catch_ball();
      out_cnt++;
    }
  }
}
void back_catch_ball(){
  if(is_ball && (direction_ball <= 7 || direction_ball >= 9)){
    v = 100;
    move(direction_ball);
  }
  else stop();
}
void AI_1(){
  sensors();
  print_all();
  // out_sharp();
  out();
  // strat_1();
  if(shr > shl){
    rl = true;
  }
  else{
    rl = false;
  }
  if(direction_ball == 0 && (distance_ball > near_dis || distance_ball < 32) && is_ball && !Ball_In_Kicker){
    moveForSec(0, 2);
  }
  else if (Ball_In_Kicker) {
    spinon;
    // look_back = true;
    if (is_goal) {
      v = 0.5 * vm;
      arrived_to_goal = true;
      GY = 0;if (angle_goal <= 180 && angle_goal >= 3)      motor(0, 80, 80, 0);
      else if (angle_goal > 180 && angle_goal < 357) motor(0, -80, -80, 0);
      else {
        stop();
        shoot();  
      }
    }
    else{
      // arrived_to_goal = false;
      move(0);
      // move(direction_goal);
      // shoot();
    }
  }
    // stop();
  else if (is_ball){
    spinon;
    arrived_to_goal = false;
    if (distance_ball >= 50){ //dor
      // spinoff;
      v = vm;
      if (direction_ball == 0) move(0);
      else if (direction_ball == 1) move(2);
      else if (direction_ball == 15) move(14);
      else if (direction_ball < 8 && direction_ball > 1) move(direction_ball + 3);
      else move(direction_ball - 3);
    } 
    // else if (distance_ball < 115) {
    //   v = vm;
    //   move(direction_ball);
    // } 
    else {        //////////nazdik
      v = 0.8 * vm;
      if (direction_ball == 0) move(0);
      else if (direction_ball <= 2 && direction_ball > 0) move(direction_ball + 3);
      else if (direction_ball <= 8 && direction_ball > 2) move(direction_ball + 5);
      else if (direction_ball > 8 && direction_ball <= 13) move(direction_ball - 5);
      else move(direction_ball - 3);
    }
  }
  // move(0);
  // delay(1000);
  // move(8);
  // delay(1000);
  // move(4);
  // delay(1000);
  // move(12);
  // delay(1000);
  else {
    // look_back = false;
    // stop();
    // if(shb >3000){
    //   move(0);
    // }
    if(shb < back_Distance){
      motor(-130 + dif , -130 - dif , 130 - dif , 130 + dif);
    }
    else{
      stop();
    }
    spinoff;
  }
}
void AI_2(){
  sensors();
  print_all();
  // out_sharp();
  out();
  // strat_1();
  if(shr > shl){
    rl = true;
  }
  else{
    rl = false;
  }
  if(direction_ball == 0 && (distance_ball > near_dis || distance_ball < 32) && is_ball && !Ball_In_Kicker){
    moveForSec(0, 2);
  }
  else if (Ball_In_Kicker) {
    spinon;
    look_back = true;
    sensors();
    int _GY = GY;
    if(!Ball_in_kick_init){
      moveForSec(0, 5);
      Ball_in_kick_init = true;
    }
    else if (!turned_back){
      GY = 0;
      v = 0.4 * vm;
      arrived_to_goal = false;
      if(_GY > 10) motor(0, 0, v, v);
      else if(_GY < -10) motor(-v, -v, 0, 0);
      else {
        if(turned_back_cnt > 10)  turned_back = true;
        else stop();
        turned_back_cnt++;
      }
    }
    else if (is_goal) {
      turned_back_cnt = 0;
      v = 0.5 * vm;
      if(!arrived_to_goal){
        move(direction_goal);
        if(LDR_B || LDR_F || LDR_L || LDR_R) arrived_to_goal = true;
      }
      else{
        GY = 0;
        if (angle_goal <= 180 && angle_goal >= 3)      motor(0, 80, 80, 0);
        else if (angle_goal > 180 && angle_goal < 357) motor(0, -80, -80, 0);
        else {
          spinoff;
          stop();
          shoot();  
        }
      }
    }
    else{
      v = 0.5 * vm;
      if(!arrived_to_goal){
        move(8);
      }
      else{
        look_back = false;
        if(GY > -10 && GY < 10){
          spinoff;
          stop();
          shoot(); 
        }
        else  stop();
      }
    }
  }
  else if (is_ball){
    spinon;
    look_back = false;
    arrived_to_goal = false;
    turned_back_cnt = 0;
    Ball_in_kick_init = false;
    if (distance_ball >= 50){ //dor
      v = vm;
      if (direction_ball == 0) move(0);
      else if (direction_ball == 1) move(2);
      else if (direction_ball == 15) move(14);
      else if (direction_ball < 8 && direction_ball > 1) move(direction_ball + 3);
      else move(direction_ball - 3);
    } 
    else {        //////////nazdik
      v = 0.8 * vm;
      if (direction_ball == 0) move(0);
      else if (direction_ball <= 2 && direction_ball > 0) move(direction_ball + 3);
      else if (direction_ball <= 8 && direction_ball > 2) move(direction_ball + 5);
      else if (direction_ball > 8 && direction_ball <= 13) move(direction_ball - 5);
      else move(direction_ball - 3);
    }
  }
  else {
    look_back = false;
    arrived_to_goal = false;
    Ball_in_kick_init = false;
    if(shb < back_Distance){
      motor(-130 + dif , -130 - dif , 130 - dif , 130 + dif);
    }
    else{
      stop();
    }
    spinoff;
  }
}