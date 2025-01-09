#ifndef N100WP
#define N100WP

class IMUDataprocess
{
public:
    float v_n=0;
    float v_n_1=0;
    float a_n=0;
    float a_n_1=0;
    float Deltat=0.01;
    float S_n=0;
    float S_n_1=0;

    float computeS(float at)
    {
        a_n=at*9.8;
        v_n+=at*Deltat;
        S_n+=v_n*Deltat;

        return v_n;
    }

   


};


#endif