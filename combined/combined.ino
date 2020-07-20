unsigned char databuf[20];     // 用來儲存收進來的 data byte
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



void setup()
{
  Serial.begin(115200);//傳送接收電腦封包
  Serial3.begin(57600);//serial port for right wheel
  Serial4.begin(57600);//serial port for left wheel

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

  vl_rpm = vl * 60 / (2 * r_wheel * PI); //rpm
  vr_rpm = vr * 60 / (2 * r_wheel * PI); //rpm


  //for right wheel
  int vr_beforeHex = -1 * vl_rpm / 3000 * 8192; // For moving forward, we need vr_rpm need to be negative, that's the opposite of 2 wheeled robot general kinematic's rule, thats why we need to multiply it by -1
  int vr_h = vr_beforeHex / 256;
  int vr_l = vr_beforeHex % 256;

  if (vr_beforeHex >= 0 ) {
    unsigned char vr_hex[] = {0x06, vr_h, vr_l, vr_h + vr_l + 0x06};
    Serial3.write(mod, 4);
    Serial3.write(a_speed, 4);
    Serial3.write(still, 4);
    Serial3.write(star, 4);
    Serial3.write(vr_hex, 4);
  }
  else
  {
    unsigned char vr_hex[] = {0x06, vr_h - 1, vr_l, vr_h + vr_l + 0x06 - 1};
    Serial3.write(mod, 4);
    Serial3.write(a_speed, 4);
    Serial3.write(still, 4);
    Serial3.write(star, 4);
    Serial3.write(vr_hex, 4);
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
  }
  else
  {
    unsigned char vl_hex[] = {0x06, vl_h - 1, vl_l, vl_h + vl_l + 0x06 - 1};
    Serial4.write(mod, 4);
    Serial4.write(a_speed, 4);
    Serial4.write(still, 4);
    Serial4.write(star, 4);
    Serial4.write(vl_hex, 4);
  }


}
