#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <cstddef>

class Parameters {
public:

    static constexpr  int rows = 16;
    static constexpr  int cols = 1800;

    static constexpr  float T = 160; 
    static constexpr  float Y = 170; 

    static constexpr  float T_h_max = 177;  
    static constexpr  float T_h_min = 165; 
    static constexpr  float D_h = 50;
    static constexpr  float w1_h = 0.50, w2_h = 0.50, w3_h = 0.00, w4_h = 0.00, w5_h = 0.00;


    static constexpr  float T_v_max = 175;  
    static constexpr  float T_v_min = 170;  
    static constexpr  float D_v = 50;  
    static constexpr  float w1_v = 0.33, w2_v = 0.34, w3_v = 0.33, w4_v = 0.00, w5_v = 0.00;


    static constexpr  float fusion_angle = 25;
    static constexpr  float fusion_dis = 0.1;

    static constexpr  int lineIDX[16] = {0,1,2,3,4,5,6,7,15,14,13,12,11,10,9,8};
    
};

#endif // PARAMETERS_H

#pragma once

