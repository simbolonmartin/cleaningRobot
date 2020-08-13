unsigned char Databuf[100];     // 用來儲存收進來的 data byte  
unsigned char buf[10];     // 用來儲存收進來的 data byte 
double L = 0.57;            //輪子距離m  
int addr = 0;

double smart_motor_vl;//給smart motor的左輪速度
double smart_motor_vr;//給smart motor的右輪速度
double vl; //運動學算出來左輪速度
double vr; //運動學算出來右輪速度
double send_v;
double send_w;

int integer_v;
int float_v;
int integer_w;
int float_w;

int HighByte_integer_v;
int LowByte_integer_v;
int HighByte_float_v;
int LowByte_float_v;

int HighByte_integer_w;
int LowByte_integer_w;
int HighByte_float_w;
int LowByte_float_w;

//收資料用
unsigned char encoding_data[8]; //回傳encoding data
//回傳給車子encoding速度跟角速度
int re_integer_v;
int re_float_v;
int re_integer_w;
int re_float_w;

int re_HighByte_integer_v;
int re_LowByte_integer_v;
int re_HighByte_float_v;
int re_LowByte_float_v;

int re_HighByte_integer_w;
int re_LowByte_integer_w;
int re_HighByte_float_w;
int re_LowByte_float_w;
//馬達指令
unsigned char mod[] = {0x02,0x00,0xc4,0xc6}; //速度模式
unsigned char a_speed[] = {0x0a,0x14,0x14,0x32}; //加速度
unsigned char still[] = {0x1c,0x00,0x00,0x1c}; //讓速度持續跑
unsigned char redata[] = {0x80,0x00,0x80}; //回傳資料指令
unsigned char star[] = {0x00,0x00,0x01,0x01}; //啟動
//馬達回傳
unsigned char return_lv[4]; //收取左輪回傳速度
int count = 0; //計算左輪速度封包
int last_rpm_L = 0; //存取上一筆左輪速度
float re_lv = 0.0;//回傳rpm換算後左輪速度
unsigned char return_rv[4]; //收取右輪回傳速度
int count1 = 0; //計算右輪速度封包
int last_rpm_R = 0; //存取上一筆右輪速度
float re_rv = 0.0;//回傳rpm換算後右輪速度
float encoding_v;    //解碼車子整體速度
float encoding_w;    //解碼車子整體角速度
//用來回傳給上層v跟w
IntervalTimer myTimer;

void setup() 
{   
    Serial.begin(57600);//傳送接收電腦封包
    Serial1.begin(57600);//傳送接收左輪封包
    Serial2.begin(57600);//傳送接收右輪封包   
    myTimer.begin(return_encode,100000);//回傳封包給上層的timer
}  
void loop() 
{   
     
}  

void serialEvent()
{
    read_send_message();
    two_wheel_kinematic(send_v, send_w, vl, vr);
    smart_motor_speed(vl, vr);
}

void serialEvent1()
{
    return_Lspeed(re_lv);//取得左輪速度re_lv
}

void serialEvent2()
{
    return_Rspeed(re_rv);//取得右輪速度re_lv
}

void read_send_message()//收取封包
{
  // 檢查是否有資料可供讀取#@  
    if (Serial.available() > 0) 
    {  
        Serial.readBytes(Databuf, 13);
        for(int i=0;i<13;i++)
        {
            //Serial.print(Databuf[i]);
            //Serial.print(" ");
            if(i == 12)
            {
              //Serial.println("=======");
            }
            addr++;
        }
        if(addr == 13)
        {
              if(Databuf[0] == 83 && Databuf[1] == 84 && Databuf[10] == 69  && Databuf[11] == 78 && Databuf[12] == 68)
              {
                  //Serial.print(Databuf);
                  HighByte_integer_v = Databuf[2];
                  LowByte_integer_v = Databuf[3];
                  HighByte_float_v = Databuf[4];
                  LowByte_float_v = Databuf[5];
                  HighByte_integer_w = Databuf[6];
                  LowByte_integer_w = Databuf[7];
                  HighByte_float_w = Databuf[8];
                  LowByte_float_w = Databuf[9];

                  integer_v = HighByte_integer_v * 256 + LowByte_integer_v;
                  float_v = HighByte_float_v * 256 + LowByte_float_v;
                  integer_w = HighByte_integer_w * 256 + LowByte_integer_w;
                  float_w = HighByte_float_w * 256 + LowByte_float_w;

              
                  send_v = integer_v + float_v/1000.0;
                  send_w = integer_w + float_w/1000.0;


                  send_v = send_v - 20;
                  send_w = send_w - 20;
//                  Serial.print("send_v:");
//                  Serial.println(send_v);
//                  Serial.print("send_w:");
//                  Serial.println(send_w);
              }
              else 
              {
                  //Serial.print("send error message");
                  send_v = 0;
                  send_w = 0;
                  /*Serial.print(Databuf[0]);
                  Serial.print(" ");
                  Serial.print(Databuf[1]);
                  Serial.print(" ");
                  Serial.print(Databuf[10]);
                  Serial.print(" ");
                  Serial.print(Databuf[11]);
                  Serial.print(" ");
                  Serial.print(Databuf[12]);
                  Serial.print("=======");*/
              }
              addr = 0;
        }
        else 
        {
            send_v = 0;
            send_w = 0;
        }
    }  
}

void two_wheel_kinematic(double V, double w, double &vl, double &vr)//二輪運動學
{
    vl = V - w*L/2;
    vr = V + w*L/2;
   
//    Serial.print("send_vl:");
//    Serial.println(vl);
//    Serial.print("send_vr:");
//    Serial.println(vr);
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

void return_encode()//回傳給上層
{
  encoding_v = (re_lv + re_rv) / 2;
  encoding_w = re_rv - re_lv;
  send_encodingdata(encoding_v, encoding_w);
}

void Vec_L_Control(double smart_motor_vl)//控制左輪
{
   smart_motor_vl = -1 * smart_motor_vl;
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
     Serial1.write(redata,3);
   }
   else 
   {
     unsigned char v[] = {0x06, v_h-1, v_l, v_h+v_l+0x06-1};
     Serial1.write(mod,4);
     Serial1.write(a_speed,4);
     Serial1.write(still,4);
     Serial1.write(star,4);
     Serial1.write(v,4);
     Serial1.write(redata,3);
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
   }
   else 
   {
     unsigned char v[] = {0x06, v_h-1, v_l, v_h+v_l+0x06-1};
     Serial2.write(mod,4);
     Serial2.write(a_speed,4);
     Serial2.write(still,4);
     Serial2.write(star,4);
     Serial2.write(v,4);
   }
}

void return_Lspeed(float &re_lv)//接收回傳左輪速度
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
       count = 0;
    }
    //計算真正速度rpm
    if(return_lv[0] == 0xe4)
    {
      int re_v = (int)return_lv[1]*256 + (int)return_lv[2];
      if(re_v > 32768)re_v = re_v - 65536;
      int re_rpm = re_v * 3000 /8192;
      if(fabs(re_rpm) > 1200)re_rpm = last_rpm_L;
      if(fabs(re_rpm - last_rpm_L) > 50)
      {
         re_rpm = last_rpm_L;
      }
      re_lv = re_rpm  * (0.128 *PI) / 60;
      //Serial.print("v:");
      //Serial.println(re_lv);
      last_rpm_L = re_rpm;
    }
  }
}

void return_Rspeed(float &re_rv)//接收回傳右輪速度
{
  if(Serial2.available()>0)
  {
    unsigned char data1 = Serial1.read();
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
       count1 = 0;
    }
    //計算真正速度rpm
    if(return_rv[0] == 0xe4)
    {
      int re_v = (int)return_rv[1]*256 + (int)return_rv[2];
      if(re_v > 32768)re_v = re_v - 65536;
      int re_rpm = re_v * 3000 /8192;
      if(fabs(re_rpm) > 1200)re_rpm = last_rpm_R;
      if(fabs(re_rpm - last_rpm_R) > 50)
      {
         re_rpm = last_rpm_R;
      }
      re_rv = re_rpm  * (0.128 *PI) / 60;
      //Serial.print("v:");
      //Serial.println(re_rv);
      last_rpm_R = re_rpm;
    }
  }
}

void send_encodingdata(float encoding_v, float encoding_l)//回傳給車子解碼後的速度跟角速度
{
    encoding_v = encoding_v + 20;
    encoding_w = encoding_w + 20;
    re_integer_v = int(encoding_v);
    re_float_v = int(encoding_w);
    re_integer_w = int(encoding_v * 1000) % 1000;
    re_float_w = int(encoding_w * 1000) % 1000;

    re_HighByte_integer_v = re_integer_v / 256;
    re_LowByte_integer_v = re_integer_v % 256;
    re_HighByte_float_v = re_float_v / 256;
    re_LowByte_float_v = re_float_v % 256;
    re_HighByte_integer_w = re_integer_w / 256;
    re_LowByte_integer_w = re_integer_w % 256;
    re_HighByte_float_w = re_float_w / 256;
    re_LowByte_float_w = re_float_w % 256;

    encoding_data[0] = re_HighByte_integer_v;
    encoding_data[1] = re_LowByte_integer_v;
    encoding_data[2] = re_HighByte_float_v;
    encoding_data[3] = re_LowByte_float_v;
    encoding_data[4] = re_HighByte_integer_w;
    encoding_data[5] = re_LowByte_integer_w;
    encoding_data[6] =  re_HighByte_float_w;
    encoding_data[7] =  re_LowByte_float_w;

    Serial.print("E");
    Serial.print("C");
    Serial.print(";");
    Serial.print(encoding_data[0]);
    Serial.print(",");
    Serial.print(encoding_data[1]);
    Serial.print(",");
    Serial.print(encoding_data[2]);
    Serial.print(",");
    Serial.print(encoding_data[3]);
    Serial.print(",");
    Serial.print(encoding_data[4]);
    Serial.print(",");
    Serial.print(encoding_data[5]);
    Serial.print(",");
    Serial.print(encoding_data[6]);
    Serial.print(",");
    Serial.print(encoding_data[7]);
    Serial.print(";");
    Serial.println("E");
}
