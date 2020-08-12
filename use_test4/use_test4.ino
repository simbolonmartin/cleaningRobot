unsigned char star[] = {0x00,0x00,0x01,0x01}; //啟動
unsigned char stop1[] = {0x00,0x00,0x00,0x00}; //停止
unsigned char stop2[] = {0x06,0x00,0x00,0x06}; //速度0
unsigned char mod[] = {0x02,0x00,0xc4,0xc6}; //速度模式
unsigned char _speed[] = {0x06,0x01,0x11,0x18}; //速度
unsigned char na_speed[] = {0x06,0xfe,0xef,0xf3}; //速度
unsigned char a_speed[] = {0x0a,0x14,0x14,0x32}; //加速度
unsigned char still[] = {0x1c,0x00,0x00,0x1c}; //讓速度持續跑

unsigned char respeed[] = {0x63,0x00,0x00,0x63}; //回傳速度指令
unsigned char redata[] = {0x80,0x00,0x80}; //回傳資料指令
unsigned char return_data[6]; //收取回傳值
unsigned char return_speed[4]; //收取回傳速度
unsigned char return_voltage[4]; //收取回傳電流
unsigned char return_hposition[4]; //收取回傳高位置
unsigned char return_lposition[4]; //收起回傳低位置

int count = 0; //計算速度封包
int count1 = 0; //計算電流封包
int count2 = 0; //計算高位置封包
int count3 = 0; //計算低位置封包
int last_rpm = 0; //存取上一筆速度
int last_vol = 0; //存取上一筆電流
//馬達回傳
unsigned char re_data[32];
unsigned char re_data1[32];
unsigned char return_lv[5]; //收取左輪回傳速度
int last_rpm_L = 0; //存取上一筆左輪速度
float re_lv = 0;//回傳rpm換算後左輪速度
unsigned char return_rv[5]; //收取右輪回傳速度
int last_rpm_R = 0; //存取上一筆右輪速度
float re_rv;//回傳rpm換算後右輪速度
IntervalTimer myTimer;
double L = 0.57;            //輪子距離m 
double smart_motor_vl;//給smart motor的左輪速度
double smart_motor_vr;//給smart motor的右輪速度
double vl;
double vr;
double encoding_v;
double encoding_w;
double last_encoding_v=0;
double last_encoding_w=0;
int return_count = 0;
char check_num;

void setup() {
  Serial.begin(57600);
  Serial1.begin(57600);
  Serial2.begin(57600);
  myTimer.begin(return_all_data, 100000);
}

void serialEvent()
{
  if(Serial.available())
  {
    char data = Serial.read();
    if(data == '1')
    {
//       two_wheel_kinematic(0.5, 0.35, vl, vr);
//       smart_motor_speed(vl, vr);
//       Serial1.write(mod,4);
//       Serial1.write(a_speed,4);
//       Serial1.write(still,4);
//       Serial1.write(_speed,4);
//       Serial1.write(star,4);
//       Serial1.write(redata,3);
       Serial2.write(mod,4);
       Serial2.write(a_speed,4);
       Serial2.write(still,4);
       Serial2.write(_speed,4);
       Serial2.write(star,4);
       Serial2.write(redata,3);
    }
    else if(data == '2')
    {
//       two_wheel_kinematic(0, 0, vl, vr);
//       smart_motor_speed(vl, vr);
//       Serial1.write(mod,4);
//       Serial1.write(a_speed,4);
//       Serial1.write(still,4);
//       Serial1.write(stop2,4);
//       Serial1.write(star,4);
       Serial2.write(mod,4);
       Serial2.write(a_speed,4);
       Serial2.write(still,4);
       Serial2.write(stop2,4);
       Serial2.write(star,4);
    }
    else if(data == '3')
    {
//       two_wheel_kinematic(-0.5, -0.35, vl, vr);
//       smart_motor_speed(vl, vr);
//       Serial1.write(mod,4);
//       Serial1.write(a_speed,4);
//       Serial1.write(still,4);
//       Serial1.write(na_speed,4);
//       Serial1.write(star,4);
       Serial2.write(mod,4);
       Serial2.write(a_speed,4);
       Serial2.write(still,4);
       Serial2.write(na_speed,4);
       Serial2.write(star,4);
    }
    else if(data == '4')
    {
       myTimer.begin(return_all_data, 100000);
    }
    else if(data == '5')
    {
       myTimer.end();
    }
  }
}

void loop() {
  
}

void two_wheel_kinematic(double V, double w, double &vl, double &vr)//二輪運動學
{
    vl = V - w*L/2;
    vr = V + w*L/2;
   
}

void return_all_data()
{
  Serial1.write(redata,3);
  Serial2.write(redata,3);
  //return_encode(re_lv, re_rv);
}

void return_encode(float re_lv, float re_rv)//回傳給上層
{
  float loc_lv = re_lv;
  float loc_rv = re_rv;
  double loc_encoding_v = (loc_lv + loc_rv) / 2;
  double loc_encoding_w = (loc_rv - loc_lv) / L;
  encoding_v = loc_encoding_v;
  encoding_w = loc_encoding_w;
  //Serial.println(encoding_v);
  //Serial.println(encoding_w);
}

void smart_motor_speed(double vl,double vr)
{
    smart_motor_vl = vl * 60 / (0.128*PI); //rpm
    smart_motor_vr = vr * 60 / (0.128*PI); //rpm
    delay(1);
    Vec_L_Control(smart_motor_vl); //送左輪封包
    delay(1);
    Vec_R_Control(smart_motor_vr); //送右輪封包
}

void Vec_L_Control(double smart_motor_vl)//控制左輪
{
   smart_motor_vl = smart_motor_vl;
   int motor_v = smart_motor_vl / 3000 * 8192;
   int v_h = motor_v/256;
   int v_l = motor_v%256;
      
   if(motor_v >=0 )
   {
     unsigned char v[] = {0x06, v_h, v_l, v_h+v_l+0x06};
     Serial1.write(mod,4);
     Serial1.write(a_speed,4);
     Serial1.write(still,4);
     Serial1.write(star,4);
     Serial1.write(v,4);
     //Serial1.write(redata,3);
   }
   else 
   {
     unsigned char v[] = {0x06, v_h-1, v_l, v_h+v_l+0x06-1};
     Serial1.write(mod,4);
     Serial1.write(a_speed,4);
     Serial1.write(still,4);
     Serial1.write(star,4);
     Serial1.write(v,4);
     //Serial1.write(redata,3);
   }
}

void Vec_R_Control(double smart_motor_vr)//控制右倫
{
   int motor_v = smart_motor_vr / 3000 * 8192;
   int v_h = motor_v/256;
   int v_l = motor_v%256;
      
   if(motor_v >=0 )
   {
     unsigned char v[] = {0x06, v_h, v_l, v_h+v_l+0x06};
     Serial2.write(mod,4);
     Serial2.write(a_speed,4);
     Serial2.write(still,4);
     Serial2.write(star,4);
     Serial2.write(v,4);
     //Serial2.write(redata,3);
   }
   else 
   {
     unsigned char v[] = {0x06, v_h-1, v_l, v_h+v_l+0x06-1};
     Serial2.write(mod,4);
     Serial2.write(a_speed,4);
     Serial2.write(still,4);
     Serial2.write(star,4);
     Serial2.write(v,4);
     //Serial2.write(redata,3);
   }
}
void serialEvent1()
{
//    float loc_lv = 0.0;
//    return_Lspeed(loc_lv);//取得左輪速度re_lv
//    re_lv = loc_lv;
    return_all_1();
}

void serialEvent2()
{
//    float loc_rv = 0.0;
//    return_Rspeed(loc_rv);//取得右輪速度re_lv
//    re_rv = loc_rv;
    return_all_2();
}

void return_all_1()
{
  if(Serial1.available()>0)
  {
    unsigned char data1 = Serial1.read();
    if(data1 == 0x80 && count==0)
    {
      re_data[count] = data1;
      count++;
    }
    else if(data1 == 0x00 && count==1)
    {
      re_data[count] = data1;
      count++;
    }
    else if(data1 == 0x01 && count==2)
    {
      re_data[count] = data1;
      count++;
    }
    else if(data1 == 0x81 && count==3)
    {
      re_data[count] = data1;
      count++;
    }
    else if(count > 3 && count<32)
    {
      re_data[count] = data1;
      count++;
    }
    else if(count==32)
    {
      check_num = re_data[12] + re_data[13] + re_data[14];
      if(re_data[12] == 0xe4 && re_data[16] == 0xe6 && check_num == re_data[15])
      {
        int re_v = (int)re_data[13]*256 + (int)re_data[14];
        if(re_v > 32768)re_v = re_v - 65536;
        int re_rpm = re_v * 3000 /8192;
        if(fabs(re_rpm) > 300)re_rpm = last_rpm_L;
//      if(fabs(re_rpm - last_rpm_L) > 50)
//      {
//         re_rpm = last_rpm_L;
//      }
        float loc_lv = re_rpm  * (0.128 *PI) / 60;
        //Serial.print("v:");
        Serial.println(re_rpm);
        last_rpm_L = re_rpm;
      }
      count = 0;
    }
    else
    {
      memset(re_data, 0, sizeof(re_data));
    }
  }
}

void return_all_2()
{
  if(Serial2.available()>0)
  {
    unsigned char data1 = Serial2.read();
    if(data1 == 0x80 && count1==0)
    {
      re_data1[count1] = data1;
      count1++;
    }
    else if(data1 == 0x00 && count1==1)
    {
      re_data1[count1] = data1;
      count1++;
    }
    else if(data1 == 0x01 && count1==2)
    {
      re_data1[count1] = data1;
      count1++;
    }
    else if(data1 == 0x81 && count1==3)
    {
      re_data1[count1] = data1;
      count1++;
    }
    else if(count1 > 3 && count1<32)
    {
      re_data1[count1] = data1;
      count1++;
    }
    else if(count1==32)
    {
      check_num = re_data1[12] + re_data1[13] + re_data1[14];
      if(re_data1[12] == 0xe4 && re_data1[16] == 0xe6 && check_num == re_data1[15])
      {
        int re_v = (int)re_data1[13]*256 + (int)re_data1[14];
        if(re_v > 32768)re_v = re_v - 65536;
        int re_rpm = re_v * 3000 /8192;
        if(fabs(re_rpm) > 300)re_rpm = last_rpm_R;
//      if(fabs(re_rpm - last_rpm_L) > 50)
//      {
//         re_rpm = last_rpm_L;
//      }
        float loc_rv = re_rpm  * (0.128 *PI) / 60;
        //Serial.print("v:");
        Serial.println(re_rpm);
        last_rpm_R = re_rpm;
      }
      count1 = 0;
    }
    else
    {
      memset(re_data1, 0, sizeof(re_data1));
    }
  }
}


void return_Lspeed(float &loc_lv)//接收回傳左輪速度
{
  if(Serial1.available()>0)
  {
    unsigned char data1 = Serial1.read();
    //速度回授
    if(data1 == 0xe4 && count==0)
    {
       return_lv[count] = data1;
       count++;
    }
    else if(count > 0 && count < 4)
    {
       return_lv[count] = data1;
       count++;
    }
    else if(data1 == 0xe6 && count == 4)
    {
       return_lv[count] = data1;
       check_num = return_lv[0]+return_lv[1]+return_lv[2];
       count = 0;
    }
    //計算真正速度rpm
    if(return_lv[0] == 0xe4 && return_lv[4] == 0xe6 && check_num == return_lv[3])
    {
      int re_v = (int)return_lv[1]*256 + (int)return_lv[2];
      if(re_v > 32768)re_v = re_v - 65536;
      int re_rpm = re_v * 3000 /8192;
//      if(fabs(re_rpm) > 1200)re_rpm = last_rpm_L;
//      if(fabs(re_rpm - last_rpm_L) > 50)
//      {
//         re_rpm = last_rpm_L;
//      }
      loc_lv = re_rpm  * (0.128 *PI) / 60;
      //Serial.print("v:");
      Serial.println(re_rpm);
      last_rpm_L = re_rpm;
    }
    else
    {
      //memset(return_lv, 0, sizeof(return_lv));
    }
  }
}

void return_Rspeed(float &loc_rv)//接收回傳右輪速度
{
  if(Serial2.available()>0)
  {
    unsigned char data1 = Serial2.read();
    //速度回授
    if(data1 == 0xe4 && count1==0)
    {
       return_rv[count1] = data1;
       count1++;
    }
    else if(count1 > 0 && count1 < 4)
    {
       return_rv[count1] = data1;
       count1++;
    }
    else if(data1 == 0xe6 && count1 == 4)
    {
       return_rv[count1] = data1;
       count1 = 0;
    }
    //計算真正速度rpm
    if(return_rv[0] == 0xe4 && return_rv[4] == 0xe6 && (return_rv[0]+return_rv[1]+return_rv[2]) == return_rv[3])
    {
      int re_v = (int)return_rv[1]*256 + (int)return_rv[2];
      if(re_v > 32768)re_v = re_v - 65536;
      int re_rpm = re_v * 3000 /8192;
//      if(fabs(re_rpm) > 1200)re_rpm = last_rpm_R;
//      if(fabs(re_rpm - last_rpm_R) > 50)
//      {
//         re_rpm = last_rpm_R;
//      }
      loc_rv = re_rpm  * (0.128 *PI) / 60;
      //Serial.print("v:");
      //Serial.print(loc_rv);
      last_rpm_R = re_rpm;
    }
    else
    {
      memset(return_rv, 0, sizeof(return_rv));
    }
  }
}
