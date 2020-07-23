unsigned char databuf[20];     // 用來儲存收進來的 data byte
unsigned char encoding_data[8]; //回傳encoding data
int count = 0;
int connection = -1;//判斷連線
int con_check = -1;//判斷連線
double vx;//上層速度x
double vy;//上層速度y
double w;//上層w
double v_magnitude; //squareroot of Vx^2 + Vy^2
double L = 0.57; // the distance between wheel in meter
double vl; //left_wheel speed in m/s
double vr; //right wheel speed in m/s
double r_wheel = 0.105; //in meter
double vl_rpm; //left_wheel speed in rpm
double vr_rpm; //right_wheel speed in rpm

unsigned char mod[] = {0x02,0x00,0xc4,0xc6}; //速度模式
unsigned char a_speed[] = {0x0a,0x14,0x14,0x32}; //加速度
unsigned char still[] = {0x1c,0x00,0x00,0x1c}; //讓速度持續跑
unsigned char stop2[] = {0x06,0x00,0x00,0x06}; //速度0
unsigned char star[] = {0x00,0x00,0x01,0x01}; //啟動
unsigned char redata[] = {0x80,0x00,0x80};

unsigned char return_lv[4]; //收取左輪回傳速度
unsigned char return_rv[4]; //收取右輪回傳速度

float encoding_v;    //解碼車子整體速度
float encoding_w;    //解碼車子整體角速度
float re_lv = 0.0;
float re_rv = 0.0;//回傳rpm換算後右輪速度

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

int last_rpm_L = 0; //存取上一筆左輪速度
int last_rpm_R = 0; //存取上一筆右輪速度


int count1 = 0; //計算右輪速度封包
int count2 = 0;

IntervalTimer myTimer;
void setup()
{
  Serial.begin(115200);//傳送接收電腦封包
  Serial3.begin(57600);//serial port for right wheel
  Serial4.begin(57600);//serial port for left wheel
  myTimer.begin(return_encode,100000);//回傳封包給上層的timer
}
void loop()
{
  if (connection == con_check) //判斷有無斷線
  {
    static int con_count = 0;
    con_count++;
    if (con_count > 10) //超過10次沒更新斷線
    {
      vx = 0;
      vy = 0;
      w = 0;
      con_count = 0;


      Serial3.write(mod, 4);
      Serial3.write(a_speed, 4);
      Serial3.write(still, 4);
      Serial3.write(stop2, 4);
      Serial3.write(star, 4);

      Serial4.write(mod, 4);
      Serial4.write(a_speed, 4);
      Serial4.write(still, 4);
      Serial4.write(stop2, 4);
      Serial4.write(star, 4);

      
    }
  }
  //    Serial.print("vx=");
  //    Serial.println(vx);
  //    Serial.print("vy=");
  //    Serial.println(vy);
  //    Serial.print("w=");
  //    Serial.println(w);
  con_check = connection;
  delay(100);
}

void serialEvent()
{
  receive_package();
  convertVw_to_sendingSerial();
}

void serialEvent4()
{
    return_Lspeed(re_lv);//取得左輪速度re_lv
}

void serialEvent3()
{
    return_Rspeed(re_rv);//取得右輪速度re_rv
}





void receive_package()
{
  if (Serial.available() > 0)
  {
    unsigned char data = Serial.read();
    //Serial.println(data);
    if (data == 'S' && count == 0)
    {
      databuf[count] = data;
      count++;
    }
    else if (data == 'T' && count == 1)
    {
      databuf[count] = data;
      count++;
    }
    else if (count > 1 && count < 17)
    {
      databuf[count] = data;
      count++;
    }
    else if (data == 'E' && count == 17)
    {
      databuf[count] = data;
      count++;
    }
    else if (data == 'N' && count == 18)
    {
      databuf[count] = data;
      count++;
    }
    else if (data == 'D' && count == 19)
    {
      databuf[count] = data;
      count = 0;

      char checksum = databuf[2] + databuf[3] + databuf[4] + databuf[5] + databuf[6] + databuf[7] + databuf[8] + databuf[9] + databuf[10] + databuf[11] + databuf[12] + databuf[13];
      if (databuf[0] == 'S' && databuf[1] == 'T' && databuf[17] == 'E' && databuf[18] == 'N' && databuf[19] == 'D' && databuf[14] == checksum)
      {
        connection = databuf[15];

        int HighByte_integer_vx = databuf[2];
        int LowByte_integer_vx = databuf[3];
        int HighByte_float_vx = databuf[4];
        int LowByte_float_vx = databuf[5];

        int HighByte_integer_vy = databuf[6];
        int LowByte_integer_vy = databuf[7];
        int HighByte_float_vy = databuf[8];
        int LowByte_float_vy = databuf[9];

        int HighByte_integer_w = databuf[10];
        int LowByte_integer_w = databuf[11];
        int HighByte_float_w = databuf[12];
        int LowByte_float_w = databuf[13];

        int interger_vx = HighByte_integer_vx * 256 + LowByte_integer_vx;
        int float_vx = HighByte_float_vx * 256 + LowByte_float_vx;

        int interger_vy = HighByte_integer_vy * 256 + LowByte_integer_vy;
        int float_vy = HighByte_float_vy * 256 + LowByte_float_vy;

        int interger_w = HighByte_integer_w * 256 + LowByte_integer_w;
        int float_w = HighByte_float_w * 256 + LowByte_float_w;

        double re_vx = interger_vx + float_vx / 1000.0;
        double re_vy = interger_vy + float_vy / 1000.0;
        double re_w = interger_w + float_w / 1000.0;

        vx = re_vx - 100;//拿去用
        vy = re_vy - 100;//拿去用
        w = re_w - 100;//拿去用

      }
    }
    else
    {
      memset(databuf, 0, sizeof(databuf));
      count = 0;
    }

  }
}


void convertVw_to_sendingSerial() {
  v_magnitude = vx; //for 2 wheeled robot, the only speed that affecting the robot is vx
  
  
  
  
  vl = v_magnitude - w * L / 2;
  vr = v_magnitude + w * L / 2;


  vl = vl/3;
  vr = vr/3;


  
  vl_rpm = vl * 60 / (2 * r_wheel * PI); //rpm
  vr_rpm = vr * 60 / (2 * r_wheel * PI); //rpm


  


  //for right wheel
  int vr_beforeHex = -1 * vr_rpm / 3000 * 8192; // For moving forward, we need vr_rpm need to be negative, that's the opposite of 2 wheeled robot general kinematic's rule, thats why we need to multiply it by -1
  int vr_h = vr_beforeHex / 256;
  int vr_l = vr_beforeHex % 256;

  if (vr_beforeHex >= 0 ) {
    unsigned char vr_hex[] = {0x06, vr_h, vr_l, vr_h + vr_l + 0x06};
    Serial3.write(mod, 4);
    Serial3.write(a_speed, 4);
    Serial3.write(still, 4);
    Serial3.write(star, 4);
    Serial3.write(vr_hex, 4);
    Serial3.write(redata,3);
  }
  else
  {
    unsigned char vr_hex[] = {0x06, vr_h - 1, vr_l, vr_h + vr_l + 0x06 - 1};
    Serial3.write(mod, 4);
    Serial3.write(a_speed, 4);
    Serial3.write(still, 4);
    Serial3.write(star, 4);
    Serial3.write(vr_hex, 4);
    Serial3.write(redata,3);
  }

  //for left wheel
  int vl_beforeHex = vl_rpm / 3000 * 8192; //For moving forward, we need vl_rpm positive, so it is okay to left it like this
  int vl_h = vl_beforeHex / 256;
  int vl_l = vl_beforeHex % 256;

  if (vl_beforeHex >= 0 ) {
    unsigned char vl_hex[] = {0x06, vl_h, vl_l, vl_h + vl_l + 0x06};
    Serial4.write(mod, 4);
    Serial4.write(a_speed, 4);
    Serial4.write(still, 4);
    Serial4.write(star, 4);
    Serial4.write(vl_hex, 4);
    Serial4.write(redata,3);
  }
  else
  {
    unsigned char vl_hex[] = {0x06, vl_h - 1, vl_l, vl_h + vl_l + 0x06 - 1};
    Serial4.write(mod, 4);
    Serial4.write(a_speed, 4);
    Serial4.write(still, 4);
    Serial4.write(star, 4);
    Serial4.write(vl_hex, 4);
    Serial4.write(redata,3);
  }
}


void return_encode()//回傳給上層
{
  encoding_v = (re_lv + re_rv) / 2;
  encoding_w = re_rv - re_lv;
  send_encodingdata(encoding_v, encoding_w);
}
void send_encodingdata(float encoding_v, float encoding_w)//回傳給車子解碼後的速度跟角速度
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

void return_Lspeed(float &re_lv)//接收回傳左輪速度
{
  if(Serial4.available()>0)
  {
    unsigned char data1 = Serial4.read();
    //速度回授
    if(data1 == 0xe4 && count2==0)
    {
       return_lv[count2] = data1;
       count2++;
    }
    else if(count2 > 0 && count2 < 4)
    {
       return_lv[count2] = data1;
       count2++;
    }
    else if(data1 == 0xe6 && count2 == 4)
    {
       count2 = 0;
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
      re_lv = re_rpm  * (0.210 *PI) / 60;
      //Serial.print("v:");
      //Serial.println(re_lv);
      last_rpm_L = re_rpm;
    }
  }
}

void return_Rspeed(float &re_rv)//接收回傳右輪速度
{
  if(Serial3.available()>0)
  {
    unsigned char data1 = Serial3.read();
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
      re_rv = re_rpm  * (0.210 *PI) / 60;
      //Serial.print("v:");
      //Serial.println(re_rv);
      last_rpm_R = re_rpm;
    }
  }
}
