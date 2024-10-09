#ifndef Controller_h
#define Controller_h

#include "Arduino.h"

class Controller {
  public:
    Controller(){}

    float step(float r_v, float theta, float theta_dot) {
      for(int i = 0; i < 4; i++) {
        state[i][2] = state[i][1];
        state[i][1] = state[i][0];
      }
      state[0][0] = r_v;
      state[1][0] = theta;
      state[2][0] = theta_dot;

      float u_d = 0;
      for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
          u_d = u_d+K_coeff[i][j]*state[i][j];
       }
      }
      u_d = u_d-K_coeff[3][1]*state[3][1]-K_coeff[3][2]*state[3][2];
      //u_d = constrain(u_d, -10, 10);
      state[3][0] = u_d;

      return u_d;
    }

    private:
      const float K_coeff[4][3] = {{-13.187, -7.434, 20.609},
                                   {-3, 7, -3},
                                   {-0.435, -0.245, 0.680},
                                   {1, -3.116, 2.115}};
      float state[4][3] = {0};  
};

#endif