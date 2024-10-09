#ifndef Controller_h
#define Controller_h

class Controller {
  public:
    Controller() {
      Kp = 10;
      Kd = 1;
    }

    float step(float pitch, float pitch_rate) {
      return Kp*pitch+Kd*pitch_rate;
    }

  private:
    float Kp, Kd;
}

#endif