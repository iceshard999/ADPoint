#include <Arduino.h>
#include <Adafruit_ADS1X15.h>

#include <WiFi.h>
#include <Wire.h>
#include <ESP32Servo.h>
//#include <WebServer.h>

/* init Wifi and Mqtt */
// Replace the next variables with your SSID/Password combination
const char* ssid = "lab309";
const char* password = "ustc1234";
//WebServer server(80); //声明WebServer对象

long lastMsg = 0;
long lastADC = 0;
float value = 0;
float values[200];
long times[200];
int i_step = 0;
int flag = 0;
bool positive_flag = true;

int kalman_height=0;
int out_value = 0;
float out_last = 0; //上一次滤波值
char fisrt_flag = 1;

int16_t result;

/* Be sure to update this value based on the IC and the gain settings! */
//float   multiplier = 3.0F;    /* ADS1015 @ +/- 6.144V gain (12-bit results) */
//float multiplier = 0.1875F; /* ADS1115  @ +/- 6.144V gain (16-bit results) */
float multiplier = 0.125F; /* ADS1115  @ +/- 4.096V gain (16-bit results) */

char valueString[512];
//IP address to send UDP data to:
// either use the ip address of the server or 
// a network broadcast address
const char * udpAddress = "192.168.124.7";
const int udpPort = 47269;
//Are we currently connected?
boolean connected = false;

//The udp library class
WiFiUDP udp;

/* init ADS1115 */
Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */
//Adafruit_ADS1015 ads;     /* Use this for the 12-bit version */

// init Servos
Servo servo360;
Servo servo2;


//1. 结构体类型定义
typedef struct 
{
    float LastP;//上次估算协方差 初始化值为0.02
    float Now_P;//当前估算协方差 初始化值为0
    float out;//卡尔曼滤波器输出 初始化值为0
    float Kg;//卡尔曼增益 初始化值为0
    float Q;//过程噪声协方差 初始化值为0.001
    float R;//观测噪声协方差 初始化值为0.543
}KFP;//Kalman Filter parameter

//2. 以高度为例 定义卡尔曼结构体并初始化参数
KFP KFP_height={0.02,0,0,0,0.001,0.543};

/**
 *卡尔曼滤波器
 *@param KFP *kfp 卡尔曼结构体参数
 *   float input 需要滤波的参数的测量值（即传感器的采集值）
 *@return 滤波后的参数（最优值）
 */
 float kalmanFilter(KFP *kfp,float input)
 {
     //预测协方差方程：k时刻系统估算协方差 = k-1时刻的系统协方差 + 过程噪声协方差
     kfp->Now_P = kfp->LastP + kfp->Q;
     //卡尔曼增益方程：卡尔曼增益 = k时刻系统估算协方差 / （k时刻系统估算协方差 + 观测噪声协方差）
     kfp->Kg = kfp->Now_P / (kfp->Now_P + kfp->R);
     //更新最优值方程：k时刻状态变量的最优值 = 状态变量的预测值 + 卡尔曼增益 * （测量值 - 状态变量的预测值）
     kfp->out = kfp->out + kfp->Kg * (input -kfp->out);//因为这一次的预测值就是上一次的输出值
     //更新协方差方程: 本次的系统协方差付给 kfp->LastP 威下一次运算准备。
     kfp->LastP = (1-kfp->Kg) * kfp->Now_P;
     return kfp->out;
 }

// 低通滤波
float fc = 10.0f;     //截止频率
float Ts = 0.01f;    //采样周期
float pi = 3.14159f; //π
static float alpha = 0;     //滤波系数

/************************ 滤波器初始化 alpha *****************************/
void low_pass_filter_init(void)
{
  float b = 2.0 * pi * fc * Ts;
  alpha = b / (b + 1);
}

float low_pass_filter(float value)
{
  
  float out;

  /***************** 如果第一次进入，则给 out_last 赋值 ******************/
  
  if (fisrt_flag == 1)
  {
    fisrt_flag = 0;
    out_last = value;
  }

  /*************************** 一阶滤波 *********************************/
  out = out_last + alpha * (value - out_last);
  out_last = out;

  return out;
}


void didReceiveRequest(){
  //String arg0 = server.pathArg(0);
  //String arg1 = server.pathArg(1);
  //server.send(200, "text/plain", "This is the link/{}/{},The parameters are:" + arg0 + " & " + arg1);
}

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  int retry = 5;
  while (WiFi.status() != WL_CONNECTED && retry>0) {
    delay(500);
    Serial.print(".");
    retry--;
  }

  if (WiFi.status() == WL_CONNECTED){
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
  }else{
    Serial.println("WiFi NOT connected, offline mode");
  }
}



void setup(void)
{

  Serial.begin(115200);
  Serial.println("Hello!");

  Serial.println("Getting differential reading from AIN0 (P) and AIN1 (N)");
  Serial.println("ADC Range: +/- 6.144V (1 bit = 3mV/ADS1015, 0.1875mV/ADS1115)");

  setup_wifi();
  //server.on("/{}", didReceiveRequest);        //注册链接与回调函数
  //server.begin();
  // The ADC input range (or gain) can be changed via the following
  // functions, but be careful never to exceed VDD +0.3V max, or to
  // exceed the upper and lower limits if you adjust the input range!
  // Setting these values incorrectly may destroy your ADC!
  //                                                                ADS1015  ADS1115
  //                                                                -------  -------
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV

  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS.");
    while (1);
  }

  servo360.attach(33);
  servo2.attach(32);
  
  servo360.write(97);
  servo2.write(90);

  low_pass_filter_init();

}

void loop(void)
{
  long now = millis();

  if (now - lastADC >0) {
    lastADC = now;
    result = ads.readADC_Differential_0_1();

    kalman_height = kalmanFilter(&KFP_height,(float)result);
    out_value = int(low_pass_filter(result));

    value = out_value * multiplier;
  
    sprintf(valueString+strlen(valueString), "%ld:%.2f;",now, value);
    values[i_step] = value;

    i_step++;
  }

  if (now - lastMsg > 50) {
    if (WiFi.status() == WL_CONNECTED){
      udp.beginPacket(udpAddress,udpPort);
      udp.printf("Voltage(mV):%s|mV", valueString); 
      udp.endPacket();
    }

    // clear valueString
    memset(valueString, '\0', sizeof(valueString));
    float max = 0;
    float min = 0;
    for (int i=0;i<i_step;i++){
      if (values[i]>max){
        max = values[i];
      }
      if (values[i]<min){
        min = values[i];
      }
    }

    i_step = 0;
    lastMsg = now;

    //if(abs(max) > abs(min)){
    if(positive_flag == true){
      value = max;
    }else{
      value = min;
    }
    int v = floor(value*0.8)+90;
    if (v>135){
      v = 135;
    }else if(v<45){
      v = 45;
    }

    positive_flag = !positive_flag;
    servo2.write(v);
  }


  flag++;

}