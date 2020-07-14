unsigned char star[] = {0x00, 0x00, 0x01, 0x01}; //啟動 starting
unsigned char stop1[] = {0x00, 0x00, 0x00, 0x00}; //停止
unsigned char stop2[] = {0x06, 0x00, 0x00, 0x06}; //速度0
unsigned char mod[] = {0x02, 0x00, 0xc4, 0xc6}; //速度模式 contorl mode



unsigned char _speed[] = {0x06, 0x01, 0x11, 0x18}; //速度 100 rpm
unsigned char _speed2[] = {0x06, 0x00, 0x88, 0x8e}; // 50 rpm
unsigned char _speed3[] = {0x06, 0x00, 0x36, 0x3c}; // 20 rpm
unsigned char _speed4[] = {0x06, 0xff, 0xca, 0xcf}; // -20 rpm
unsigned char _speed5[] = {0x06, 0x00, 0x52, 0x58}; // 30 rpm
unsigned char _speed6[] = {0x06, 0x00, 0x18, 0x1e}; // 9 rpm = 1 m/s
unsigned char _speed7[] = {0x06, 0xff, 0xe8, 0xed}; // -9 rpm = 1 m/s
unsigned char _speed2rpm[] = {0x06, 0x00, 0x05, 0x0b};
unsigned char _speed5rpm[] = {0x06, 0x00, 0x0e, 0x14};
unsigned char _speed5rpm2[] = {0x06,0xff,0xf2,0xf7}; // -negative
//unsigned char _speed4[] = {0x06,0x00,0x88,0x8e}; //

unsigned char na_speed[] = {0x06, 0xfe, 0xef, 0xf3}; //速度
unsigned char a_speed[] = {0x0a, 0x14, 0x14, 0x32}; //加速度
unsigned char still[] = {0x1c, 0x00, 0x00, 0x1c}; //讓速度持續跑

unsigned char respeed[] = {0x63, 0x00, 0x00, 0x63}; //回傳速度指令
unsigned char redata[] = {0x80, 0x00, 0x80}; //回傳資料指令
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
double last_encoding_v = 0;
double last_encoding_w = 0;
int return_count = 0;
char check_num;

void setup() {
  Serial.begin(57600);
  Serial4.begin(57600);
  Serial3.begin(57600);
  myTimer.begin(return_all_data, 100000);
}

void serialEvent()
{
  if (Serial.available())
  {
    char data = Serial.read();
    Serial.println(data);
    if (data == '1') //forward 20 rpm
    {

      Serial3.write(mod, 4);
      Serial3.write(a_speed, 4);
      Serial3.write(still, 4);
      Serial3.write(_speed4, 4);
      Serial3.write(star, 4);
      Serial3.write(redata, 3);


      Serial4.write(mod, 4);
      Serial4.write(a_speed, 4);
      Serial4.write(still, 4);
      Serial4.write(_speed3, 4);
      Serial4.write(star, 4);
      Serial4.write(redata, 3);
    }
    else if (data == '2') // stop
    {

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
    else if (data == '3')
    {

      Serial3.write(mod, 4);
      Serial3.write(a_speed, 4);
      Serial3.write(still, 4);
      Serial3.write(na_speed, 4);
      Serial3.write(star, 4);
    }
    else if (data == '4')
    {
      myTimer.begin(return_all_data, 100000);
    }
    else if (data == '5')
    {
      myTimer.end();
    }
    else if (data == '6') // sometimes error
    {

      Serial3.write(a_speed, 4);
      Serial3.write(still, 4);
      Serial3.write(_speed, 4);
      Serial3.write(star, 4);
      Serial3.write(redata, 3);


      Serial4.write(mod, 4);
      Serial4.write(a_speed, 4);
      Serial4.write(still, 4);
      Serial4.write(_speed2, 4);
      Serial4.write(star, 4);
      Serial4.write(redata, 3);
    }
    else if (data == '7') // backward
    {

      Serial3.write(mod, 4);
      Serial3.write(a_speed, 4);
      Serial3.write(still, 4);
      Serial3.write(_speed3, 4);
      Serial3.write(star, 4);
      Serial3.write(redata, 3);


      Serial4.write(mod, 4);
      Serial4.write(a_speed, 4);
      Serial4.write(still, 4);
      Serial4.write(_speed4, 4);
      Serial4.write(star, 4);
      Serial4.write(redata, 3);
    }
    else if (data == '8')
    {

      Serial3.write(mod, 4);
      Serial3.write(a_speed, 4);
      Serial3.write(still, 4);
      Serial3.write(_speed4, 4);
      Serial3.write(star, 4);
      Serial3.write(redata, 3);


      Serial4.write(mod, 4);
      Serial4.write(a_speed, 4);
      Serial4.write(still, 4);
      Serial4.write(_speed5, 4);
      Serial4.write(star, 4);
      Serial4.write(redata, 3);
    }
    else if (data == '9') // left
    {

      Serial3.write(mod, 4);
      Serial3.write(a_speed, 4);
      Serial3.write(still, 4);
      Serial3.write(_speed4, 4);
      Serial3.write(star, 4);
      Serial3.write(redata, 3);


      Serial4.write(mod, 4);
      Serial4.write(a_speed, 4);
      Serial4.write(still, 4);
      Serial4.write(_speed5rpm, 4);
      Serial4.write(star, 4);
      Serial4.write(redata, 3);

    }
    else if (data == '0') //right
    {

      Serial3.write(mod, 4);
      Serial3.write(a_speed, 4);
      Serial3.write(still, 4);
      Serial3.write(_speed5rpm2, 4);
      Serial3.write(star, 4);
      Serial3.write(redata, 3);

      Serial4.write(mod, 4);
      Serial4.write(a_speed, 4);
      Serial4.write(still, 4);
      Serial4.write(_speed3, 4);
      Serial4.write(star, 4);
      Serial4.write(redata, 3);
    }
    else if (data == 'a') // automated for strategy 1
    {
      //forward
      Serial3.write(mod, 4);
      Serial3.write(a_speed, 4);
      Serial3.write(still, 4);
      Serial3.write(_speed3, 4);
      Serial3.write(star, 4);
      Serial3.write(redata, 3);

      Serial4.write(mod, 4);
      Serial4.write(a_speed, 4);
      Serial4.write(still, 4);
      Serial4.write(_speed4, 4);
      Serial4.write(star, 4);
      Serial4.write(redata, 3);

      delay(3000);


      
      //stop
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
      
      delay(500);
      //left
      Serial3.write(mod, 4);
      Serial3.write(a_speed, 4);
      Serial3.write(still, 4);
      Serial3.write(_speed4, 4);
      Serial3.write(star, 4);
      Serial3.write(redata, 3);

      Serial4.write(mod, 4);
      Serial4.write(a_speed, 4);
      Serial4.write(still, 4);
      Serial4.write(_speed5rpm, 4);
      Serial4.write(star, 4);
      Serial4.write(redata, 3);
      delay(500);
      
      
    }
  }
}

void loop() {

  

}



void return_all_data()
{
  Serial4.write(redata, 3);
  Serial3.write(redata, 3);
  //return_encode(re_lv, re_rv);
}
