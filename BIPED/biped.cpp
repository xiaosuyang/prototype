#include "biped.h"
#include "math/orientation_tools.h"

void Biped::RJ0RJ1convert(float DEG_0,float DEG_1,float &L0,float &L1)
{
    DEG_0=deg2rad(DEG_0);
    DEG_1=deg2rad(DEG_1);

    Vec3<float> B_O2P1{-65,190,17};
    Vec3<float> F1_O2t1{-65,54,-30};


    Vec3<float> y0=B_O2P1-F1_O2t1;
    Vec3<float> y=B_O2P1-ori::coordinateRotation(ori::CoordinateAxis::Z,DEG_0)*ori::coordinateRotation(ori::CoordinateAxis::X,DEG_1)*F1_O2t1;
    
    L0=(y.norm()-y0.norm())*0.001;

    Vec3<float> B_O2P1_1{65,190,17};
    Vec3<float> F1_O2t1_1{65,54,-30};

    y0=B_O2P1_1-F1_O2t1_1;
    y=B_O2P1_1-ori::coordinateRotation(ori::CoordinateAxis::Z,DEG_0)*ori::coordinateRotation(ori::CoordinateAxis::X,DEG_1)*F1_O2t1_1;

    L1=(y.norm()-y0.norm())*0.001;
    

}