//连接wifi后登陆MQTT，然后每1s上报一次数据(数据每次加1)
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <WiFiUdp.h>
#include <String.h>
#include <Ticker.h>
#include <stdio.h>

#define led 2  //发光二极管连接在8266的GPIO2上

#define WIFI_SSID "sunghoon"    //wifi名称
#define WIFI_PASSWORD "zzq200311"  //wifi密码
//MQTT三元组
#define ClientId "123400"
#define Username "bkrc"
#define Password "88888888"

#define MQTT_Address "115.28.209.116"
#define MQTT_Port 1883
#define Iot_link_MQTT_Topic_Report "device/008ab3a58b311e0f/up"
#define Iot_link_MQTT_Topic_Commands "device/008ab3a58b311e0f/down"

WiFiClient myesp8266Client;
PubSubClient client(myesp8266Client);
StaticJsonBuffer<300> jsonBuffer;
//StaticJsonDocument<200> jsonBuffer2; //声明一个sonDocument对象，长度200
char printf_buf[16] = { 0 };
unsigned char buffer[32];
unsigned char network_buffer[5];
char total_data[50] = { 0 };

unsigned char rx_buffer[32];
uint16_t rx_length = 0;

int data_temp = 1;
int mqtt_state = 0;
int mqtt_sub_state = 1;

uint16_t Smoke_value = 0, bh1750_value = 0, balance = 0, door_flag = 0;
void setup() {
  // put your setup code here, to run once:
  pinMode(led, HIGH);
  Serial.begin(9600);
  WIFI_Init();
  MQTT_Init();
}
void loop() {
  client.loop();  //保持MQTT服务器连接
  if (Serial.available()) {
    //读取串口数据
    rx_buffer[rx_length] = Serial.available();
    if (rx_length == 0) {
      if (rx_buffer[0] == 0x55)  //包头数据判断
      {
        rx_length++;
      } else {
        rx_length = 0;
      }
    } else {
      // 读取串口数据
      rx_buffer[rx_length] = Serial.available();
      if (rx_buffer[rx_length] == 0xBB) {
        rx_length = 0;
      } else {
        // 指针自增
        rx_length++;
        // 如果储存超出长度
        if (rx_length == 27) {
          // 重新从数组0开始储存字节
          rx_length = 0;
        }
      }
    }
    //如果储存超出长度
    if (rx_length >= 27) {
      //重新从数组0开始储存字节
      rx_length = 0;
    }
    Serial.readBytes(rx_buffer, Serial.available());

    //将串口发来的数据进行转换JSON数据发布到云平台
    protocolJSON(rx_buffer);
    //清空串口数据
    while (Serial.read() >= 0) {};
  }
  // put your main code here, to run repeatedly:
  if (WiFi.status() == WL_CONNECTED) {
    //MQTT_POST();
  } else {
    WIFI_Init();
  }
  if (!client.connected() && mqtt_state == 1) {
    client.connect(ClientId, Username, Password);
    if (client.connect(ClientId)) {
      // Serial.println("connected");
      // 连接成功时订阅主题
      client.subscribe(Iot_link_MQTT_Topic_Commands);

    } else {

      //      Serial.print("failed, rc=");
      //      Serial.print(client.state());
      //      Serial.println(" try again in 5 seconds");
      network_buffer[0] = 0xAA;
      network_buffer[1] = 0x02;
      network_buffer[2] = 0x00;
      network_buffer[3] = 0x00;
      network_buffer[4] = 0xBB;
      Serial.write(network_buffer, 5);
      MQTT_Init();
      delay(500);
    }
  }
  delay(500);
  //data_temp++;
}
//连接wifi
void WIFI_Init() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    digitalWrite(led, LOW);
    // Serial.println("WiFi Not Connect");
    network_buffer[0] = 0x55;
    network_buffer[1] = 0xAA;
    network_buffer[2] = 0x00;
    network_buffer[3] = 0x00;
    network_buffer[4] = 0x00;
    network_buffer[5] = 0xBB;
    Serial.write(network_buffer, 6);
  }
  digitalWrite(led, LOW);
  delay(500);
  digitalWrite(led, HIGH);
  // Serial.println("WiFi Connected OK!");
  network_buffer[0] = 0x55;
  network_buffer[1] = 0xAA;
  network_buffer[2] = 0x01;
  network_buffer[3] = 0x00;
  network_buffer[4] = 0x00;
  network_buffer[5] = 0xBB;
  Serial.write(network_buffer, 6);
  MQTT_Init();
}
//连接MQTT
void MQTT_Init() {
  client.setServer(MQTT_Address, MQTT_Port);
  client.setClient(myesp8266Client);
  client.subscribe(Iot_link_MQTT_Topic_Commands);
  client.setCallback(callback);

  while (!client.connected()) {
    client.connect(ClientId, Username, Password);
    if (client.connect(ClientId)) {
      // Serial.println("connected");
      // 连接成功时订阅主题
      client.subscribe(Iot_link_MQTT_Topic_Commands);

    } else {

      //      Serial.print("failed, rc=");
      //      Serial.print(client.state());
      //      Serial.println(" try again in 5 seconds");
      network_buffer[0] = 0x55;
      network_buffer[1] = 0xAA;
      network_buffer[2] = 0x02;
      network_buffer[3] = 0x00;
      network_buffer[4] = 0x00;
      network_buffer[5] = 0xBB;
      Serial.write(network_buffer, 6);
      MQTT_Init();
      delay(500);
    }
  }
  //先网关发送链接成功数据
  network_buffer[0] = 'T';
  Serial.write(network_buffer, 6);
  mqtt_state = 1;
  // Serial.println("MQTT Connected OK!");
}
//DynamicJsonDocument doc(1024);
/* 云端下发 */
void callback(char* topic, byte* payload, unsigned int length) {
  //  for (int i = 0; i < length; i++) {
  //    Serial.print((char)payload[i]);
  //  }
  mqtt_sub_state = 0;
  JsonObject& root = jsonBuffer.parseObject(payload);
  // Test if parsing succeeds.
  if (!root.success()) {
    Serial.println("parseObject() failed");
    return;
  }
  String sign = root["message"];

  JsonObject& root1 = jsonBuffer.parseObject(sign);
  String sign1 = root1["sign"];

  //接收到，led灯闪烁一次
  digitalWrite(2, LOW);
  delay(100);
  digitalWrite(2, HIGH);

  jsonBuffer.clear();
}

void protocolJSON(unsigned char JsonData[]) {
  //以下将生成好的JSON格式消息格式化输出到字符数组中，便于下面通过PubSubClient库发送到服务器
  char JSONmessageBuffer[1024];
  StaticJsonBuffer<1024> json_buffer;  //定义静态的JSON缓冲区用于存储JSON消息
  JsonObject& root = json_buffer.createObject();
  JsonObject& data = root.createNestedObject("data");
  root["sign"] = "008ab3a58b311e0f";
  root["type"] = 3;

  JsonObject& temphumi = data.createNestedObject("Temp_Humi_Smoke");            //温湿度传感器模块标识

  sprintf(printf_buf, "%d", 0);

  temphumi["temp"] = printf_buf;  //温度模型标识
  temphumi["humi"] = printf_buf;  //湿度模型标识
  temphumi["smoke"] = printf_buf;

  if (JsonData[0] == 0x55) {
    if (JsonData[1] == 0xAA) {
      sprintf(total_data, "%d", JsonData[2]);
      temphumi["temp"] = total_data;
      memset(total_data, 0, sizeof(total_data));
      delay(50);

      sprintf(total_data, "%d", JsonData[3]);
      temphumi["humi"] = total_data;
      memset(total_data, 0, sizeof(total_data));
      delay(50);

       sprintf(total_data, "%d", JsonData[4]);
      temphumi["smoke"] = total_data;
      memset(total_data, 0, sizeof(total_data));
      delay(50);
    }
  }

  root.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));  //JSON转换成字符串

  //  Serial.println("Sending message to MQTT topic..");
  //  Serial.println(JSONmessageBuffer);
  //json数据
  client.publish(Iot_link_MQTT_Topic_Report, JSONmessageBuffer);
  network_buffer[0] = 0x55;
  network_buffer[1] = 0xAA;
  network_buffer[2] = 0x03;
  network_buffer[3] = 0x00;
  network_buffer[4] = 0x00;
  network_buffer[5] = 0xBB;
  Serial.write(network_buffer, 6);
  //接收到，led灯闪烁一次
  digitalWrite(2, LOW);
  delay(100);
  digitalWrite(2, HIGH);
}
