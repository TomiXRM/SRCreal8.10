#include <Arduino.h>
#include "stdio.h"
#include "Wire.h"                  //I2C
#include "SparkFunMLX90614.h"      //放射熱センサー
#include "Adafruit_Sensor.h"       //Sensor
#include "Adafruit_BNO055.h"       //BNO055
#include "utility/imumaths.h"      //BNO055
#include "DualVNH5019MotorShield.h"//モータードライバー
#include "Adafruit_NeoPixel.h"     //NeoPixel
#include "avr/power.h"             //NeoPixel
#include "LiquidCrystal_I2C.h"

#define buzzer 51
#define neopixel 52
#define TREGGER 38
#define ECHO 40
#define Ball 46
#define pc Serial
#define BALL Serial1
#define OB 1
#define AM 0
#define wait_ms delay
//movement_flag
#define FW 1
#define LSPIN 2
#define RSPIN 3
#define STOP 4
#define BK 5
/*********************ルール**********************
   Moon base 1から出発し、mining pitで温度測定を行いMoon base 2を目指す
   • Mining pitまでは、パルス光赤外線を利用する。
   • Mining pit で停止した後、Moon base 2で赤外線ボールの電源を入れ、Mining pitの赤外線ボールの電源を切る
   • Moon base 2までの間に障害物を設置する(オプション)
   • Mining pitでの温度を試合終了時間から5分以内に本部に提出 (オプション)
   • 雨天、曇天時を考慮し、今回はGPSによる制御を用いなくとも 達成できる様にした。
 */
/*********************変数*********************/
float volt, vvolt = 11.1; //電圧
int movement_flag = RSPIN;//初期値
int duration = 0;
char text[16];
char MODEFLAG;
struct {
        uint8_t highbyte;
        uint8_t lowbyte;
        short intdat;
} data;
struct {
        int IRangle;//ボールの方角
        int before_angle;
        int IRlong;//ボールの距離
        int anglefilter = 1;
        float temp;//温度
        double US;//超音波（CM）
        int flont_dir;
        int now_dir;
        int dir;//方角
        uint8_t sp;
} sensors;
/*********************設定*********************/
DualVNH5019MotorShield MD;
Adafruit_BNO055 bno = Adafruit_BNO055();
Adafruit_NeoPixel leds = Adafruit_NeoPixel(8, neopixel, NEO_GRB + NEO_KHZ800);
IRTherm therm;
LiquidCrystal_I2C lcd(0x27,16,2);
/**********************設定********************/
void setup() {
        lcd.init();
        lcd.backlight();
        lcd.print("Rikakai");
        Serial.begin(115200);
        BALL.begin(115200);
        leds.setBrightness(255);
        leds.begin();
        MD.init();
        pinMode(buzzer,OUTPUT);
        pinMode(TREGGER, OUTPUT);
        pinMode(ECHO, INPUT);
        pinMode(Ball, OUTPUT);
        therm.begin(); // Initialize thermal IR sensor
        therm.setUnit(TEMP_C); // Set the library's units to Farenheit
        while(!bno.begin())
        {
                Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        }
        int8_t temp = bno.getTemp();   //なしでうごくかな？？
        bno.setExtCrystalUse(true);   //なしでうごくかな？？
        Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");   //*/
        Serial.println("Rikakai SRC real proglam"); Serial.println("");
        wait_ms(100);
        lcd.setCursor(0, 1);
        lcd.print("SRCReal         ");
        wait_ms(500);
        lcd.print("start           ");
        Serial.println("Start!!");
        MODEFLAG = 'A';
        if(pc.available() > 0) {
                MODEFLAG = (char)pc.read();
                pc.print("Start Mode ");
                pc.print(MODEFLAG);
        }
}
/*********************方位センサー*********************/
int getdig(){
        int dig = 0;
        imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);//0x1Aから2byte
        dig = euler.x() - sensors.flont_dir;
        if(dig > 180) {
                dig -= 360;
        }else if(dig < -179) {
                dig +=360;
        }   //*/
        return dig;
}   //*/
void setflont(int set){
        imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);//0x1Aから2byte
        sensors.flont_dir = euler.x() + set;
}   //*/
/*********************動作*********************/
void move(int mode,int power){
        switch (mode) {
        case FW:
                MD.setSpeeds(power, power);
                break;
        case BK:
                MD.setSpeeds(-power, -power);
                break;
        case RSPIN:
                MD.setSpeeds(power, -power);
                break;
        case LSPIN:
                MD.setSpeeds(-power, power);
                break;
        case STOP:
                MD.setBrakes(power, power);
                break;
        }
}
/*********************超音波センサー*********************/
double USread() {
        double distance;
        // 10マイクロ秒出力 -> パルスが送信
        digitalWrite(TREGGER, HIGH);
        delayMicroseconds(10);
        digitalWrite(TREGGER, LOW);
        // パルス送信から受信までの時間を取得
        duration = pulseIn(ECHO, HIGH,17000);
        if (duration > 0) {
                // 往復にかかった時間 / 2 * 音速(340m/s)
                distance = (double)duration / 2 * 340  / 10000;
        }else{
                distance = 289;
        }
        return distance;
}
/*********************battery*********************/
void battely_(bool mode = 0){
        const float j = 0.7;
        volt = ((analogRead(A8))*9.9/1024);
        vvolt = vvolt*j + volt*(1-j);//ローパス
        if(vvolt < 9.1) {
                for(int i=0; i<9; i++) {
                        leds.setPixelColor(i, leds.Color(255,0,0));
                        leds.show();
                }
                MD.setBrakes(0,0);
                while(1) {
                        Serial.println("low voltage!!");
                }
        }else{
                //MD.setSpeeds(200,200);
                for(int i=0; i<9; i++) {
                        leds.setPixelColor(i, leds.Color(0,255,0));
                        leds.show();
                        if(mode == 1) {
                                Serial.print(vvolt);
                                Serial.println("v");
                        }
                }
        }
}
/*********************放射熱センサー*********************/
float radiation_thermal(uint8_t mode) {
        if (therm.read())
        {
                if(mode == OB) {
                        //Serial.print("Objeect:"+String(therm.object(),2));//センサーを向けた温度
                        return therm.object();
                }else if(mode == AM) {
                        //Serial.println("Ambient:"+String(therm.ambient(),2));//周囲の気温
                        return therm.ambient();
                }
        }
}
/*********************ボールセンサー*********************/
void IRread(int error_angle = 0){
        float j = 0.5;
        digitalWrite(Ball, HIGH); //センサー用マイコンの割り込みピンにHIGHを贈る
        delayMicroseconds(10);
        digitalWrite(Ball, LOW); //センサー用マイコンの割り込みピンをLOWに戻す
        delayMicroseconds(300); //割り込み発生させてから300us待つ

        while(BALL.available()) { //センサー用マイコンからの送信データをチェック
                if ( BALL.read() == 'H' ) {
                        data.lowbyte = BALL.read(); // 下位バイトの読み取り
                        data.highbyte = BALL.read(); // 上位バイトの読み取り
                        data.intdat = data.highbyte*256+data.lowbyte;//復元
                }
        }
        /*
           sensor.IRlong
           5:とても近い
           4:近い
           3:遠い
           2:ボールなし
         */
        if(data.intdat > 5000) {
                sensors.IRangle = data.intdat - 5000;
                sensors.IRlong = 5;
        }else if(data.intdat > 4000) {
                sensors.IRangle = data.intdat - 4000;
                sensors.IRlong = 4;
        }else if(data.intdat > 3000) {
                sensors.IRangle = data.intdat - 3000;
                sensors.IRlong = 3;
        }else if(data.intdat == 2000 || data.intdat == 0) {
                sensors.IRangle = 5000;
                sensors.IRlong = 2;
        }
        if(sensors.IRangle % 1000 == 0&&sensors.IRangle != 5000) {
                sensors.IRangle = 0;
        }
        if(sensors.IRangle > 180 && sensors.IRangle != 5000) {
                sensors.IRangle = sensors.IRangle - 360;
                //sensors.IRangle -= 23;
        }
        //if(sensors.IRangle > -20 && sensors.IRangle < -5) sensors.IRangle = -90;

        if(sensors.IRangle > (-1*error_angle) &&  sensors.IRangle < error_angle) {
                sensors.IRangle = 0;
        }      //*/
        //sensors.IRangle = sensors.before_angle;

        /*sensors.anglefilter = sensors.anglefilter*j + sensors.IRangle*(1-j);   //ローパス
           sensors.IRangle = sensors.anglefilter;*/
}
/*********************センサー情報アップデート*********************/
void UPDATE(uint8_t lec_mode){
        IRread(20);//14が安定
        sensors.temp = radiation_thermal(OB);
        sensors.US = USread();
        sensors.dir = getdig();
        //digitalWrite(buzzer,0);
        lcd.setCursor(0,0); //lcd.print("                ");
        sprintf(text, "IR:%4d",sensors.IRangle);
        lcd.print(text);
        if(lec_mode == 1) {

                lcd.setCursor(0,1);
                lcd.print("temp:");
                lcd.setCursor(5,1);
                lcd.print(sensors.temp);
        }
}
/*********************シリアルモニタ*********************/
void PCprint(){
        Serial.print("ball angle\t"); Serial.print(sensors.IRangle);
        Serial.print("\tball long\t"); Serial.print(sensors.IRlong);
        Serial.print("\tthemal\t"); Serial.print(sensors.temp);
        Serial.print("\tdir\t"); Serial.print(sensors.dir);
        Serial.print("\tUltrasonic sensor\t"); Serial.print(sensors.US);
        Serial.print("\tduration\t"); Serial.println(duration);
}
/*********************リトライ用*********************/
void MODEread(){
        if(pc.available() > 0) {
                MODEFLAG = (char)pc.read();
                pc.read();//開業バッファ読み込み
                pc.print("Start Mode ");
                pc.print(MODEFLAG);
        }
}
/*********************Miningpit方向修正*********************/
void Miningpit_Direction_correction(int mode){
        while(1) {
                UPDATE(0);
                if(sensors.dir <= 10 && sensors.dir >= -10) {
                        MD.setSpeeds(0,0);
                        break;
                }else if(sensors.dir > 15) {
                        //右側
                        MD.setSpeeds(-400, 400);
                }else if(sensors.dir < -15 ) {
                        //左側
                        MD.setSpeeds(400, -400);
                }
        }
}
/*********************Moon base 1 to mining pit*********************/
void Moonbase1ToMiningpit(){
        while(1) {
                UPDATE(1);
                PCprint();
                if(sensors.IRangle == 0) {//正面
                        if(sensors.US < 35 && sensors.US > 25 && sensors.US != 0) {
                                MD.setSpeeds(0,0);
                                wait_ms(300);
                                lcd.on();
                                lcd.clear();
                                lcd.setCursor(0, 0);
                                lcd.print("Find the object!");
                                lcd.setCursor(0,1);
                                lcd.print(sensors.temp);//表示して逃げる
                                Miningpit_Direction_correction(0);
                                MODEFLAG = 'B';
                                break;
                        }else if(sensors.US > 30) {
                                MD.setSpeeds(300,300);
                        }else if(sensors.US < 30) {
                                MD.setSpeeds(-300,-300);
                        }
                }else if(sensors.IRangle < 180 && sensors.IRangle >=20) {//右側
                        MD.setSpeeds(400,-400);
                        movement_flag = RSPIN;
                }else if(sensors.IRangle > -180&& sensors.IRangle < -20) {//左側
                        MD.setSpeeds(-400,400);
                        movement_flag = LSPIN;
                }else if(sensors.IRangle == 5000) {//ない
                        if(movement_flag == LSPIN || movement_flag == RSPIN) {
                                move(movement_flag,400);
                        }else{
                                move(RSPIN,400);
                        }
                }
        }
}
/*********************mining pit to Moon base 2*********************/
void MiningpitToMoonbase2(){
        setflont(-90);
        UPDATE(0);//温度はアップデートしない
        Miningpit_Direction_correction(0);//前向く

        while(1) {
                UPDATE(0);//温度はアップデートしない
                if(sensors.IRangle == 0) {//正面
                        if(sensors.US < 35 && sensors.US > 25 && sensors.US != 0) {
                                MD.setSpeeds(0,0);
                                lcd.setCursor(0, 0);
                                lcd.print("Find object!!   ");
                                wait_ms(100);
                                Miningpit_Direction_correction(0);//前向く
                                setflont(-45);//現在向いている方向を基準に基準を-45ずらす
                                Miningpit_Direction_correction(0);//前向く
                                MD.setSpeeds(400, 400);
                                lcd.setCursor(0, 0);
                                lcd.print("aboid object!!  ");
                                wait_ms(300);
                                MD.setSpeeds(0,0);
                                setflont(+45);//現在向いている方向を基準に基準を+45ずらす
                                Miningpit_Direction_correction(0);   //前向く
                                MD.setSpeeds(400, 400);
                                wait_ms(200);
                                MD.setSpeeds(0,0);
                                //while(1) ;
                                break;   //*/
                        }else if(sensors.US > 30) {
                                MD.setSpeeds(300,300);
                        }else if(sensors.US < 30) {
                                MD.setSpeeds(-300,-300);
                        }
                }else if(sensors.IRangle < 180 && sensors.IRangle >=20) {//右側
                        MD.setSpeeds(400,-400);
                        movement_flag = RSPIN;
                }else if(sensors.IRangle > -180&& sensors.IRangle < -20) {//左側
                        MD.setSpeeds(-400,400);
                        movement_flag = LSPIN;
                }else if(sensors.IRangle == 5000) {//ない
                        if(movement_flag == LSPIN || movement_flag == RSPIN) {
                                move(movement_flag,400);
                        }else{
                                move(RSPIN,400);
                        }
                }
        }   //*/
/*********************障害物を避けてから*********************/
        while (1) {
                UPDATE(0);
                if(sensors.IRangle == 0) {//正面
                        if(sensors.US < 35 && sensors.US > 25 && sensors.US != 0) {
                                MD.setSpeeds(0,0);
                                wait_ms(300);
                                lcd.setCursor(0, 0);
                                lcd.print("SRCReal Finish!!");
                                break;
                        }else if(sensors.US > 30) {
                                MD.setSpeeds(300,300);
                        }else if(sensors.US < 30) {
                                MD.setSpeeds(-300,-300);
                        }
                }else if(sensors.IRangle < 180 && sensors.IRangle >=20) {//右側
                        MD.setSpeeds(400,-400);
                        movement_flag = RSPIN;
                }else if(sensors.IRangle > -180&& sensors.IRangle < -20) {//左側
                        MD.setSpeeds(-400,400);
                        movement_flag = LSPIN;
                }else if(sensors.IRangle == 5000) {//ない
                        if(movement_flag == LSPIN || movement_flag == RSPIN) {
                                move(movement_flag,400);
                        }else{
                                move(RSPIN,400);
                        }
                }
        }
}
/*********************main*********************/
void loop() {
        setflont(-90);
        MD.Lock(0);//1:lock,0:free
        lcd.clear();
        while(1) {
                if(MODEFLAG == 'A') {
                        Moonbase1ToMiningpit();//moon base1からmining pitへ
                }else if(MODEFLAG == 'B') {
                        //while(1){}
                        MiningpitToMoonbase2();//mining pitからmoon base2へ
                        MD.Lock(1);
                        while(1) ;
                }   //*/
        }
}
