#include<Arduino.h>
class PIDController {
    float KP = 0, KI = 0, KD = 0, Omax = 0, Imax = 0;
    float Iacc = 0,  selisihsebelum = 0;
    float t_, tireset;
    float emin;
    int resetimode;
  public:
    PIDController(float kp, float ki, float kd, float outmax, float imax) {
      KP = kp; KI = ki; KD = kd; Imax = imax; Omax = outmax;
      t_ = 0;
      tireset = 0;
      resetimode = 0;
      emin = 0;
    }
    float calc(float nilai, float target, float dt) {
      float selisih = nilai - target;
      //float Ppart = KP * selisih;
      //if (Ppart > Imax)Ppart = Imax; else if (Ppart < -Imax)Ppart = -Imax;
      //float newIacc = Iacc + selisih * KI * dt;
      t_ += dt;
      if (resetimode) {
        if (selisih * selisihsebelum < 0) {
          Iacc = 0;
        }
        if (tireset > 0) {
          if (t_ > tireset) {
            t_ -= tireset;
            Iacc = 0;
          }
        }
      }
      else {
        Iacc += selisih * KI * dt;
      }
      if (emin > 0) {
        if (fabs(selisih) > emin) {
          Iacc = 0;
        }
      }
      //float Iaccmax = Imax - fabs(Ppart);
      //if (Iaccmax < 0)Iaccmax = 0;
      if (Iacc > Imax)Iacc = Imax; else if (Iacc < -Imax)Iacc = -Imax;

      float diff = (selisih - selisihsebelum) / dt;
      selisihsebelum = selisih;
      //float diff = (nilai - nilaisebelum) / dt;
      //nilaisebelum = nilai;
      //Serial.println(diff*10000);
      float output = KP * selisih + Iacc + KD * diff;
      if (output > Omax)output = Omax; else if (output < -Omax)output = -Omax;
      return output;
    }
    void setKP(float x) {
      KP = x;
    }
    void setKI(float x) {
      KI = x;
    }
    void setKD(float x) {
      KD = x;
    }
    void setIreset(int mode) {
      resetimode = mode; //1 atau 0
    }
    void setTIreset(float t) {
      t_ = t;
    }
    void setEminInt(float e) {
      emin = e;
    }
    void setOutmax(float x) {
      Omax = x;
    }
    void setImax(float x) {
      Imax = x;
    }
    float getKP() {
      return KP;
    }
    float getKI() {
      return KI;
    }
    float getKD() {
      return KD;
    }
    void reset() {
      Iacc = 0;
      selisihsebelum = 0;
    }
};
class Smoothfilter {
  private:
    float outval;
    int navg;
  public:
    Smoothfilter(int n) {
      navg = n;
      outval = 0;
    }
    float calc(float x) {
      outval = ((outval * navg) + x) / (navg + 1.);
      return outval;
    }
    void setvalue(float x) {
      outval = x;
    }
    void setnavg(int n) {
      navg = n;
    }
};
class RateLimiter {
  private:
    int target;
    int posisisekarang;
    float r;
    long *timervar;
    long lastt, updateinterval;
  public:
    RateLimiter(float maxrate, long *Timervar) {
      r = maxrate; //satuannya dalam point per milisekon
      timervar = Timervar;
      posisisekarang = 0;
      updateinterval = 2;
    }
    void jumpto(int posisi) {
      posisisekarang = posisi;
      target = posisi;
    }
    int update(int newtarget) {
      target = newtarget;
      //long tt = millis();
      if ((*timervar - lastt) > updateinterval) {
        float selisih = (*timervar - lastt) * r;
        lastt = *timervar;
        if (target > posisisekarang) {
          if (target - posisisekarang > selisih) {
            posisisekarang += selisih;
          }
          else {
            posisisekarang = target;
          }
        }
        else {
          if (posisisekarang - target > selisih) {
            posisisekarang -= selisih;
          }
          else {
            posisisekarang = target;
          }
        }
        //Serial.println(selisih);
      }
      return posisisekarang;
    }
};
class c_aksi {
  private:
    void (*gerakan)(long);
  public:
    long durasi;
    c_aksi() {} //default constructor
    void set_aksi(void (*Gerakan)(long), long Durasi) {
      gerakan = Gerakan;
      durasi = Durasi;
    }
    void exec(long t) {
      gerakan(t);
    }
};

#define MAX_AKSI 10
class Sequencer {
  private:
    c_aksi aksi[MAX_AKSI];
    long t1[MAX_AKSI]; //akhir dari setiap aksi
    long t0;
    int n, current;
    bool isrunning;
  public:
    Sequencer(){ //contructor
      t0=0;
      n=0;
      current=0;
      isrunning=false;
    }
    void add(void (*g)(long), long durasi) { //fungsi g yang dilakukan memiliki satu parameter long (waktu)
      aksi[n].set_aksi(g, durasi);
      if (n == 0) {
        t1[n] = durasi;
      }
      else {
        t1[n] = t1[n - 1] + durasi;
      }
      n++;
    }
    void go(long t) {
      //kalau aksi terakhir memiliki durasi -1 maka akan terus di-eksekusi tanpa akhir.
      //kalau aksi terakhir memiliki durasi positif maka sesudah durasi itu habis, tidak ada eksekusi lagi
      if (isrunning) {
        if (current < n) {
          if ((t < t1[current]) || (aksi[current].durasi < 0)) {
            long t01 = t0;
            if (current > 0) t01 = t1[current - 1];
            aksi[current].exec(t - t01); //jalankan aksi ke-[current]
          }
          else { //durasi sudah habis, aksi berikutnya
            current++; //aksi berikutnya
          }
        }
        else { //sudah mencapai akhir daftar aksi
          isrunning = false;
        }
      }
    }
    void jumpto(int i) {
      current = i;
    }
    void stop(){
      isrunning=false;
    }
    void restart() {
      current = 0;
      isrunning = true;
      t0 = millis(); //waktu start
    }
};

#define MAX_SEQUENCERS 8
class ActionManager {
  private:
    int n;
    long *t; //variabel timer yang diupdate di tempat lain, biasanya di program utama
  public:
    Sequencer* S[MAX_SEQUENCERS]; //array of pointers
    ActionManager() {
      n = 0;
      t=NULL;
    }
    ActionManager(long *timervar) {
      t = timervar;
      n = 0;
    }
    void add(Sequencer *s) {
      S[n] = s;
      n++;
    }
    void settimer(long *timervar) {
      t = timervar;
    }
    void clear() {
      n = 0;
    }
    void start() { //menjalankan semua sequencer bersamaan
      for (int i = 0; i < n; i++) {
        S[i]->restart();
      }
    }
    void start(int i){
      S[i]->restart();
    }
    void stop(){ //menghentikan semua sequencer yang sedang berjalan
      for (int i = 0; i < n; i++) {
        S[i]->stop();
      }
    }      
    void go() { //panggil fungsi ini di mainloop
      long t1;
      if (t != NULL) {
        t1 = millis();
      }
      else {
        t1 = *t;
      }
      for (int i = 0; i < n; i++) {
        S[i]->go(t1);
      }
    }
};
