/*
   program untuk transporter robot
   board: ESP8266 DC MOTOR BOARD
   output: motor 1 (kiri) 8-7, motor 2 (kanan) 6-5
   Format data swerve (X, Y, S, X2, bstate)
   angka swerve dipakai untuk memutar arah hadap robot sementara
   urutan pengiriman data:
   - magic byte
   - x value (kiri kanan) 0-255, center=128
   - y value (maju mundur) 0-255, center=128
   - s value (naik turun arm) 0-180
   - x2 value (swerve)
   - button (grip buka tutup) 0=stop, 1=tutup, 2=buka, 4=putar kanan 90, 8=putar kiri 90
   (tidak pakai) output gyro: -PI sampai +PI, putar kanan --> makin besar
   Kontroller di HP dengan app inventor:
   https://drive.google.com/file/d/1tef8k8jJbKRhRBvKcuJgtwJ6TbcYfy70/view?usp=drive_link
*/
#include "Wire.h"
#include<ESP8266WiFi.h>
#include<WiFiClient.h>
#include<WiFiUdp.h>
#include<ESP8266WebServer.h>
ESP8266WebServer server(80);
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//board ini tidak ada power untuk servo
//#include <Servo.h>
#include <EEPROM.h>
#include "classes.cpp"

//output motor:

//kiri=8-7, kanan=6-5

  #define M1A D8
  #define M1B D7
  #define M2A D6
  #define M2B D5

//kiri=6-5, kanan=8-7
/*
#define M2A 6
#define M2B 5
#define M1A 8
#define M1B 7
*/

//servo:

//eeprom:
#define MAGIC 0xbd
#define ADR_DATA 0x02

//koneksi:
#define MYSSID "ESProver"
#define MYPASS "aloysius"

WiFiUDP udp;
#define UDPPORT 1234
char udpinbuf[255];
int udplen=0;

//untuk mengaktifkan gerakan test motor di awal, uncomment baris berikut ini
//#define TESTMOTORS

//===========================gyro====================================
//offset dicoba dengan example IMUzero
/*
  ....................  XAccel      YAccel        ZAccel      XGyro     YGyro     ZGyro
  [-3021,-3019] --> [-9,11]  [261,262] --> [-5,9]  [1141,1142] --> [16383,16400] [102,103] --> [-1,4]  [58,59] --> [0,4] [-3,-2] --> [-2,1]
  .................... [-3021,-3020] --> [-9,6] [261,262] --> [-4,9]  [1141,1142] --> [16384,16400] [102,103] --> [-2,4]  [58,59] --> [0,4] [-3,-2] --> [-2,1]
  .................... [-3021,-3020] --> [-14,6]  [261,262] --> [-4,9]  [1141,1141] --> [16384,16387] [102,103] --> [-3,4]  [58,59] --> [0,4] [-3,-2] --> [-3,1]
  -------------- done --------------
*/
//urutan: xgyro, ygyro, zgyro, xaccel, yaccel, zaccel
int gyrooffsets[6] = {102, 58, -2, -3021, 261, 1141}; //robot 1
MPU6050 mpu;
Quaternion q;
VectorFloat grav;
uint8_t packetsize, fifocount;
float ypr[3];  //yaw pitch roll
uint8_t fifobuffer[64];

struct savedvars {
  float kp, ki, kd, bo;
  int powermax, powerslow, powerputar;
  int gx, gy, gz, grx, gry, grz;
  byte magic;
  float m1f,m2f;
}vars1;


int commandsource = 0;  //0 = serial, 1 = btserial

float arahhadap = 0, deltaarah = 0, targetarah = 0, targetarah1 = 0, deltatargetarah = 0;
long t = 0, tlastcommand = 0, tlastproses = 0, tlastcalc = 0;
int motorenable = 0, adagyro = 0;
int xdata = 128, ydata = 128, sdata = 90, x2data = 128, bdata = 0;
float m1faktor=1; //pengali terakhir untuk power motor (kalau kiri kanan beda)
float m2faktor=1;
float powermaju = 0, powerputar = 0, deltamotor = 0;
float powerkiri = 0, powerkanan = 0;
float lastpower = 0;
float boostfactor = 0;
enum states {
  AWAL,
  SETKP,
  SETKI,
  SETKD,
  SETF,
  SAVEEE,
  READEE,
  XDATA,
  YDATA,
  X2DATA,
  SDATA,
  BDATA,
  PMAX,
  PLOW,
  PPUTAR
} state;

PIDController depan(70, 1.5, 15000, 50, 50);  //set motor value dalam persen, maks 100

const char ssid[] = MYSSID;
const char password[] = MYPASS;

inline void ledon() {
  digitalWrite(2, LOW);
}
inline void ledoff() {
  digitalWrite(2, HIGH);
}
int charhextobyte(char c) {
  if (c >= '0' && c <= '9') {
    return c - '0';
  }
  else if (c >= 'A' && c <= 'F') {
    return c - 'A' + 10;
  }
  else if (c >= 'a' && c <= 'f') {
    return c - 'a' + 10;
  }
  return 0;
}
int get2byte(const char *s) {
  int c = 0;
  //Serial.print("get2byte>");
  //Serial.print(s[0]);
  //Serial.print(s[1]);
  c = (charhextobyte(s[0]) << 4) + charhextobyte(s[1]);
  //Serial.print("->");
  //Serial.println(c);
  return c;
}
void updatemotor() {
  //kiri
  if (powerkiri >= 0) {
    analogWrite(M1A, powerkiri);
    analogWrite(M1B,0);
  } else {
    analogWrite(M1B, -powerkiri);
    analogWrite(M1A, 0);
  }
  //kanan
  if (powerkanan >= 0) {
    analogWrite(M2A, powerkanan);
    analogWrite(M2B, 0);
  } else {
    analogWrite(M2B, -powerkanan);
    analogWrite(M2A, 0);
  }
}
void proses1charcommand(char c) {
  switch (c) {
    case 'W': //maju
      maju(vars1.powermax);
      putar(0);
      break;
    case 'w': //maju pelan
      maju(vars1.powerslow);
      putar(0);
      break;
    case 'A': //kiri
      putardelta(-0.157);
      break;
    case 'S': //mundur pelan
      maju(-vars1.powerslow);
      putar(0);
      break;
    case 'D': //kanan
      putardelta(0.157); //9 derajat
      break;
    case 'X': //mundur cepat
      maju(-vars1.powermax);
      putar(0);
      break;
    case '.': //stop
      maju(0);
      putar(0);
      break;
    case 'G': //kipas angin
    /*
      if (fanstate == 0) {
        fanstate = 1;
        fanon();
      }
      else {
        fanstate = 0;
        fanoff();
      }
      */
      break;
    case '0': //controller off
      motorenable = 0;
      stopmovement();
      ledoff();
      break;
    case '1': //controller on
      motorenable = 1;
      ledon();
      break;
    case 'R': //putar kanan 90
      putardelta(M_PI_2);
      break;
    case 'L': //putar kiri 90
      putardelta(-M_PI_2);
      break;
  }
}

//#ifdef USE_UDP
//========================fungsi handle UDP packet=======================
void handleudppacket() {

  ledon();
  //Serial.print("UDP>Packet from:");
  //Serial.println(up.remoteIP());
  //Serial.print("length:");
  //Serial.println(ul);
  //Serial.print("data:");
  //Serial.write(pb, ul); //print per byte
  //for(int i=0;i<ul;i++){
  //  Serial.print(pb[i]);Serial.print(" ");
  //}
  //Serial.println();
  //proses data
  if (udpinbuf[0] == MAGIC) {
    if (udplen >= 6) { //harus ada 5 byte: magic, x, y, x2, arm_pos, bstate
      xdata = udpinbuf[1];
      ydata = udpinbuf[2];
      x2data=udpinbuf[3];
      sdata = udpinbuf[4];
      bdata = udpinbuf[5];
      prosesoutput();
      //prosesbstate();
      //s_angkat.write(sdata);
      Serial.print(xdata); Serial.print(',');
      Serial.print(ydata); Serial.print(',');
      Serial.print(sdata); Serial.print(',');
      Serial.println(bdata);
    }
  }
  else {
    proses1charcommand(udpinbuf[0]);
  }
  tlastcommand = t;
  ledoff();
}
//#else
//==============================html pages======================================
const char backlink[] PROGMEM = R"(<a href="/"><input type=button value="Back"></a>)";
const char pageheader[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
<head>
<title>ESP wifi server</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
button {user-select:none}
.b3c {height:80px; width:33%;}
.b1c {height:80px; width:80%;}
</style>
</head>
<body>
)rawliteral";

const char pageheaderminimal[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
<head>
<title>ESP wifi server</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
</head><body scroll="no" style="overflow:hidden; position:fixed">
)rawliteral";

const char indexpage[] PROGMEM = R"rawliteral(
<h1>TOPScience wifi car</h1><br>
<table style="width: 100%; table-layout:fixed;">
<tr><td colspan="3" align="center"><button class="b3c" ontouchstart="move('W')" onmousedown="move('W')" ontouchend="endmove()" onmouseup="endmove()">Maju</button></td></tr>
<tr><td colspan="3" align="center"><button class="b3c" ontouchstart="move('w')" onmousedown="move('w')" ontouchend="endmove()" onmouseup="endmove()">Maju pelan</button></td></tr>
<tr><td align="center"><button class="b1c" ontouchstart="move('A')" onmousedown="move('A')" ontouchend="endmove()" onmouseup="endmove()">Kiri</button></td>
<td align="center"><button class="b1c" ontouchstart="send('.')" onmousedown="send('.')">Stop</button></td>
<td align="center"><button class="b1c" ontouchstart="move('D')" onmousedown="move('D')" ontouchend="endmove()" onmouseup="endmove()">Kanan</button></td></tr>
<tr><td colspan="3" align="center"><button class="b3c" ontouchstart="move('S')" onmousedown="move('S')" ontouchend="endmove()" onmouseup="endmove()">Mundur pelan</button></td></tr>
<tr><td colspan="3" align="center"><button class="b3c" ontouchstart="move('X')" onmousedown="move('X')" ontouchend="endmove()" onmouseup="endmove()">Mundur cepat</button></td></tr>
<tr><td colspan="3" align="center">____________________</td></tr>
</table>
<br>
<h2>Motor enable: <input type="checkbox" id="mpcb" onclick="setmpwr()" style="width:20px;height:20px;"></h2>
<a href="/joy">Use joystick</a><br>
<script>
let intervID;
let isrunning=false;
function send(dir){
  fetch('/m?c=' + dir);
}
function move(dir){
  send(dir);
  if(isrunning==false){
    intervID=setInterval(function(){send(dir);},100);
    isrunning=true;
  }
}
function endmove(){
  send('.');
  if(isrunning){
    clearInterval(intervID);
    isrunning=false;
  }
}
function setmpwr(){
  var mpcheck = document.getElementById("mpcb");
  if(mpcheck.checked==true){
    send('1');
  }else{
    send('0');
  }
}
</script>
)rawliteral";

const char joypage[] PROGMEM = R"rawliteral(
<h2>TOPScience wifi car</h2><br>
<button id='m_en' onclick="enbtn()">Enable motors</button><br>
interval:<input type='text' id='interval' size=3 value='100'> ms<br>
ymax:<input type='text' id='ymax' size=3 value='128'> xmax:<input type='text' id='xmax' size=3 value='128'><br>
<p id='disp'></p><br>
<canvas id="joy"></canvas><br>
<a href="/">Use buttons</a>
<script>
var joy, ctx;
var width, height, radius, x0, y0;
let cx=0, cy=0, xdata=128, ydata=128;
var intervID;
let m_en=false;
let mb='bd';
function motorenable(){
  fetch('/m?c=1');
  intervID=setInterval(function(){send(xdata,ydata);},document.getElementById('interval').value);
}
function motordisable(){
  fetch('/m?c=0');
  clearInterval(intervID);
}
function enbtn(){
  m_en = !m_en;
  document.getElementById('m_en').innerText=m_en?'Disable motors':'Enable motors';
  if (m_en){motorenable();}else{motordisable();}
  drawbackground();
}
function send(x,y){
  fetch('/m?x=' + mb + x.toString(16) + y.toString(16)+'5a00');
}
function dispevent(str){
  document.getElementById('event').innerText=str;
}
function dispxy(x,y){
  document.getElementById('disp').innerText='('+x+','+y+')';
}
function tobyte(x){
  return Math.max(Math.min(Math.round(x),255),0);
}
function marker(x,y){
  ctx.clearRect(0,0,width, height);
  drawbackground();
  ctx.beginPath();
  ctx.arc(x,y,20,0,Math.PI*2);
  ctx.stroke();
}
function getpos(event){
  let x=event.clientX || event.touches[0].clientX;
  let y=event.clientY || event.touches[0].clientY;
  cx=x-joy.offsetLeft;
  cy=y-joy.offsetTop;
  if(inrange(cx,cy)){
    marker(cx,cy);
    let ymax=document.getElementById('ymax').value;
    let xmax=document.getElementById('xmax').value;
    xdata=tobyte(128+(cx-x0)*(160/radius)*xmax/128);
    ydata=tobyte(128+(y0-cy)*(160/radius)*ymax/128);
    dispxy(xdata,ydata);
  }
}
function inrange(x,y){
  return Math.sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) <= radius
}
function drawbackground(){
  x0 = width/2;
  y0 = height/4;
  ctx.beginPath();
  ctx.arc(x0,y0,radius+20,0,Math.PI*2,true);
  ctx.fillStyle=m_en?'#E5ECE5':'#ECE5E5';
  ctx.fill();
}
function resize(){
  width = window.innerWidth;
  radius=100;
  height = radius * 6.5;
  ctx.canvas.width=width;
  ctx.canvas.height=height/2;
  drawbackground();
}

function center(event){
  xdata=128;
  ydata=128;
  marker(x0,y0);
  dispxy(xdata,ydata);
}

function init(){
  joy = document.getElementById('joy');
  ctx = joy.getContext('2d');
  resize();
  document.addEventListener('mousedown', getpos);
  document.addEventListener('mouseup', center);
  document.addEventListener('mousemove', getpos);

  document.addEventListener('touchstart', getpos);
  document.addEventListener('touchend', center);
  document.addEventListener('touchcancel', center);
  document.addEventListener('touchmove', getpos);
  document.addEventListener('resize', resize);
  document.getElementById('disp').innerText=0;
}
window.addEventListener('load', init);

</script>
)rawliteral";

const char pageend[] PROGMEM = "</body></html>";

//===============================akhir html pages===============================
//================================fungsi-fungsi server=========================
void handleroot() {
  //fungsi yang dipanggil kalau ada permintaan koneksi ke "/" dari client
  ledon();
  server.send(200, "text/html", String(pageheader) + String(indexpage)+String(pageend));
  Serial.println("Mainpage requested");
  ledoff();
}
void handlejoy(){
  ledon();
  server.send(200, "text/html", String(pageheaderminimal) + String(joypage)+String(pageend));
  Serial.println("Joystick requested");
  ledoff();
}  
void handlemove(){ // URL: /m
  ledon();
  String resp="";
  if(server.args() >0){
    String c1 = server.arg("c"); //1-char commands
    String c2 = server.arg("x"); //multibyte commands
    String c3 = server.arg("s"); //settings
    if(c1!=""){
      char c = c1.c_str()[0]; //ambil karakter pertama
      Serial.print("char:");Serial.println(c);
      proses1charcommand(c);
      resp += c;
    }
    if(c2!=""){ //multibyte commands
      //susunan byte: magic, x, y, arm_pos, bstate
      const char *s = c2.c_str();
      Serial.print("command:");
      Serial.println(c2);
      if(get2byte(s)==MAGIC){
        s+=2;
        xdata=get2byte(s);
        s+=2;
        ydata=get2byte(s);
        s+=2;
        sdata=get2byte(s);
        s+=2;
        bdata=get2byte(s);
        prosesoutput();
        //prosesbstate();
        //s_angkat.write(sdata);
        Serial.print(xdata); Serial.print(',');
        Serial.print(ydata); Serial.print(',');
        Serial.print(sdata); Serial.print(',');
        Serial.println(bdata);
        resp += c2;
      }
    }
  }
  //server.sendHeader("Connection","keep-alive",true);
  server.send(200,"text/plain",resp);
  tlastcommand=t;
  ledoff();
}
//#endif
//==========================fungsi-fungsi pengaturan gerak======================
inline void putar(int power) {  //skala -100 sampai +100
  deltamotor = power;
}

void maju(float power) {
  float bb = 0;
  if (fabs(power) > fabs(lastpower)) {
    bb = (power - lastpower) * boostfactor;
  }
  powermaju = bb + power;
  lastpower = power;
}

void putardelta(float delta) {
  targetarah = targetarah + delta;
  if (targetarah >= M_PI) {
    targetarah -= (2 * M_PI);
  } else if (targetarah <= -M_PI) {
    targetarah += (2 * M_PI);
  }
}
int sign(int a) {
  if (a >= 0) return 1;
  else return -1;
}
void prosesoutput() {
  long deltat = t - tlastproses;
  tlastproses = t;
  //dari ydata jadi maju mundur
  float y = (float)(ydata - 128)*2;//untuk ESP8166, analogwrite maksimumnya 255 * 0.78125; //dari skala 128 jadi skala 100
  maju(y);
  //kalau maju, body akan terputar ke kiri, jadi tambahkan sedikit power putar ke kanan
  //rotatecompensation = y * rotatecompensationfactor;
  //dari xdata jadi sudut putar
  deltaarah = (float)(xdata - 128) * 2.34375e-5 * deltat; //128 = 3 rad/s
  putardelta(deltaarah);
}
void stopmovement() {
  powerkiri = 0;
  powerkanan = 0;
  powermaju = 0;
  updatemotor();
  depan.reset();
  targetarah = arahhadap;
}
void noswerve() {
  deltatargetarah = 0;
}
//================================eeprom================================
int readsettingsfromeeprom() {
  if (EEPROM.read(0) == MAGIC) {
    EEPROM.get(ADR_DATA, vars1);
    if (vars1.magic == MAGIC) {
      Serial.print("saved kp:");
      Serial.print(vars1.kp);
      Serial.print(" ki:");
      Serial.print(vars1.ki);
      Serial.print(" kd:");
      Serial.print(vars1.kd);
      Serial.print(" b:");
      Serial.print(vars1.bo);
      Serial.println();
      Serial.print("powermax:");
      Serial.print(vars1.powermax);
      Serial.print(" slow:");
      Serial.print(vars1.powerslow);
      Serial.print(" putar:");
      Serial.print(vars1.powerputar);
      Serial.println();
      depan.setKP(vars1.kp);
      depan.setKI(vars1.ki);
      depan.setKD(vars1.kd);
      boostfactor = vars1.bo;
      return 1;
    } else {
      Serial.println("Tidak ada data tersimpan.");
    }
  }
  return 0;
}
void savesettingstoeeprom() {
  if (EEPROM.read(0) != MAGIC) {
    EEPROM.write(0, MAGIC);
  }
  vars1.kp = depan.getKP();
  vars1.ki = depan.getKI();
  vars1.kd = depan.getKD();
  vars1.bo = boostfactor;
  vars1.magic = MAGIC;
  EEPROM.put(ADR_DATA, vars1);
  EEPROM.commit();
}
//==============command processor====================================
void prosesdata(Stream &S) {
  float f;
  char c;
  int j;
  switch (state) {
    case AWAL:  //terima magic byte awal
      c = S.read();
      Serial.println(c);
      if (c == MAGIC) {
        state = XDATA;
      } else if (c == 'p') {  //set kp
        state = SETKP;
      } else if (c == 'i') {
        state = SETKI;
      } else if (c == 'd') {
        state = SETKD;
      } else if (c == 'f') {
        state = SETF;
      } else if (c == 'm') {
        f=S.parseFloat(); //langsung saja di sini tidak perlu pindah state
        vars1.m1f=f;
      } else if (c == 'n') {
        f=S.parseFloat();
        vars1.m2f=f;
      } else if (c == 's') {  //save kp ki kd ke eeprom
        savesettingstoeeprom();
        Serial.println("pid constants saved.");
      } else if (c == 'r') {  //baca kp ki kd dll
        S.print(depan.getKP());
        S.print(',');
        S.print(depan.getKI());
        S.print(',');
        S.print(depan.getKD());
        S.print(',');
        S.print(boostfactor);
        S.println();
        Serial.print("m1f=");
        Serial.print(vars1.m1f);
        Serial.print(" m2f=");
        Serial.print(vars1.m2f);
      } else if (c == 'P') {  //power max, power slow, power putar
        state = PMAX;
      } else if (c == 'R') {  //putar kanan 90
        putardelta(M_PI_2);
        noswerve();
      } else if (c == 'L') {  //putar kiri 90
        putardelta(-M_PI_2);
        noswerve();
      } else if (c == '0') {
        motorenable = 0;
        stopmovement();
        ledoff();
      } else if (c == '1') {
        motorenable = 1;
        targetarah = arahhadap;
        depan.reset();
        ledon();
      } else if (c == 'W') {  //perintah WSAD
        maju(vars1.powermax);
        putar(0);
        noswerve();
      } else if (c == 'w') {  //maju pelan
        maju(vars1.powerslow);
        putar(0);
        noswerve();
      } else if (c == 'S') {  //mundur pelan
        maju(-vars1.powerslow);
        putar(0);
      } else if (c == 'X') {  //mundur full power
        maju(-vars1.powermax);
        putar(0);
      } else if (c == 'A') {  //putar kiri
        maju(0);
        putar(-vars1.powerputar);
        noswerve();
      } else if (c == 'D') {  //putar kanan
        maju(0);
        putar(vars1.powerputar);
        noswerve();
      }
      break;
    case XDATA:
      xdata = S.read();
      state = YDATA;
      break;
    case YDATA:
      ydata = S.read();
      state = X2DATA;
      break;
    case X2DATA:
      x2data = S.read();
      state = SDATA;
      break;
    case SDATA:
      sdata = S.read();
      state = BDATA;
      break;
    case BDATA:
      bdata = S.read();
      state = AWAL;
      //proses hitungan di sini
      prosesoutput();

      S.print("t:");
      S.print((int)(targetarah * 180 / M_PI));
      S.print(" a:");
      S.println((int)(arahhadap * 180 / M_PI));
      /*
        Serial.print("target:");
        Serial.print(targetarah);
        Serial.print("\tarah:");
        Serial.println(arahhadap);
      */
      

      break;
    case SETKP:  //set KP
      f = S.parseFloat();
      depan.setKP(f);
      state = AWAL;
      break;
    case SETKI:  //set KI
      f = S.parseFloat();
      depan.setKI(f);
      state = AWAL;
      break;
    case SETKD:  //set KD
      f = S.parseFloat();
      depan.setKD(f);
      state = AWAL;
      break;
    case SETF:  //set rotate compensation factor
      f = S.parseFloat();
      vars1.bo = f;
      state = AWAL;
      break;
    case PMAX:  //set power max, power  slow, power putar
      j = S.read();
      if (j > 0 && j < 100) vars1.powermax = j;
      state = PLOW;
      break;
    case PLOW:
      j = S.read();
      if (j > 0 && j < 100) vars1.powerslow = j;
      state = PPUTAR;
      break;
    case PPUTAR:
      j = S.read();
      if (j > 0 && j < 100) vars1.powerputar = j;
      state = AWAL;
  }  //tutup switch(state)
}
//=============================================================
//===============awal program=================================

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  EEPROM.begin(512);
  Wire.begin();

  pinMode(2, OUTPUT);
  ledon();
  readsettingsfromeeprom();

  if (mpu.testConnection()) {
    Serial.println("MPU6050 koneksi OK");
    //tone(12, 500); delay(100); noTone(12); delay(400);
  } else {
    Serial.println("MPU connection failed");
  }
  if (mpu.dmpInitialize() == 0) {
    Serial.println("DMP OK");
    mpu.setXGyroOffset(gyrooffsets[0]);
    mpu.setYGyroOffset(gyrooffsets[1]);
    mpu.setZGyroOffset(gyrooffsets[2]);
    mpu.setXAccelOffset(gyrooffsets[3]);
    mpu.setYAccelOffset(gyrooffsets[4]);
    mpu.setZAccelOffset(gyrooffsets[5]);
    mpu.setDMPEnabled(true);
    packetsize = mpu.dmpGetFIFOPacketSize();
    adagyro = 1;
  }

  //experimental features untuk PID
  depan.setEminInt(10 * M_PI / 180);

  Serial.print("starting hotspot...");
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  IPAddress ip = WiFi.softAPIP();
  Serial.print("server address: ");
  Serial.println(ip);
  
//#ifdef USE_UDP
  //pakai UDP
  udp.begin(UDPPORT);
  Serial.println("Start UDP listening.");
  //udp.onPacket(handleudppacket);
//#else
  //pakai http client
  server.on("/", handleroot);
  server.on("/m", handlemove);
  server.on("/joy", handlejoy);
  server.begin();
  Serial.println("server started");
//#endif
  
  pinMode(M1A,OUTPUT);
  pinMode(M1B,OUTPUT);
  pinMode(M2A,OUTPUT);
  pinMode(M2B,OUTPUT);

#ifdef TESTMOTORS
  Serial.println("Testing motors...");
  powerkiri = 80;
  powerkanan = 80;
  updatemotor();
  delay(100);
  powerkiri = -80;
  powerkanan = -80;
  updatemotor();
  delay(100);
  powerkiri = 0;
  powerkanan = 0;
  updatemotor();
  delay(100);
#endif
#ifdef TESTSERVO

  delay(200);

#endif
  vars1.powermax = 100;
  vars1.powerslow = 24;
  vars1.powerputar = 30;
  
  Serial.println("Setup done");
  ledoff();
}
//===================================main loop================================
void loop() {
  t = millis();
  server.handleClient();
  
  //Serial.print("loop");
  //Serial.println(t);
  if (adagyro) {
    //Serial.println("adagyro");
    if (mpu.dmpGetCurrentFIFOPacket(fifobuffer)) {
      mpu.dmpGetQuaternion(&q, fifobuffer);
      mpu.dmpGetGravity(&grav, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &grav);
      arahhadap = ypr[0];  //dalam radian
      //Serial.println(arahhadap);
      //kalau dekat batas +- pi:
      targetarah1 = targetarah + deltatargetarah;  //delta ini untuk mensimulasikan gerak swerve
      if (targetarah1 >= M_PI) {
        targetarah1 -= (2 * M_PI);
      } else if (targetarah1 <= -M_PI) {
        targetarah1 += (2 * M_PI);
      }

      if (targetarah1 > M_PI_2) {
        if (arahhadap < 0) arahhadap += 2 * M_PI;
      } else if (targetarah1 < -M_PI_2) {
        if (arahhadap > 0) arahhadap -= 2 * M_PI;
      }
      float deltat = t - tlastcalc;
      powerputar = depan.calc(arahhadap, targetarah1, deltat);  //positif = putar kanan
      //kalau arah hadap < target (kurang ke kanan), selisih = negatif, output harus positif
      putar(-powerputar);

      tlastcalc = t;
    }
  } else {  //tidak ada gyro! apa yang harus diperbuat?
    //Serial.println("nogyro");
    float deltat = t - tlastcalc;
    if (deltat > 50) {               //update 20 kali perdetik
      if (targetarah > arahhadap) {  //putarkanan
        powerputar = vars1.powerputar;
      } else if (targetarah < arahhadap) {
        powerputar = -vars1.powerputar;
      } else {
        powerputar = 0;
      }
      targetarah = 0;
      arahhadap = 0;
      //kalau arah hadap < target (kurang ke kanan), selisih = negatif, output harus positif
      //putar(-powerputar);
      int xx = xdata - 128;
      int xx2 = x2data - 128;
      if (abs(xx) >= abs(xx2)) {
        putar(xx);
      }
      else {
        putar(xx2);
      }

      tlastcalc = t;
    }
  }
  //Serial.println("Serial?");
  while (Serial.available()) {
    tlastcommand = t;
    prosesdata(Serial);
  }
  
  //Serial.println("UDP?");
  int inpacket = udp.parsePacket();
  if(inpacket){
    Serial.print("packet:");
    Serial.println(inpacket);
    udplen=udp.read(udpinbuf,255);
    handleudppacket();
  }
  
  //Serial.println("timeout?");

  if (t - tlastcommand > 120) {
    //Serial.println("stopmovement");
    stopmovement();
  }
  //update motor di setiap loop
  //Serial.println("motorenable?");
  if (motorenable) {
    //Serial.println("motorenable");
    //putar positif ke kanan, power kiri tambahkan, power kanan kurangi
    powerkiri = (powermaju + deltamotor)*vars1.m1f;
    powerkanan = (powermaju - deltamotor)*vars1.m2f;
    if (powerkiri > 255) powerkiri = 255;
    else if (powerkiri < -255) powerkiri = -255;
    if (powerkanan > 255) powerkanan = 255;
    else if (powerkanan < -255) powerkanan = -255;
    updatemotor();
  }
  //Serial.println("end loop");
  
}
