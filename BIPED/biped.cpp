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
    
    L0=(y.norm()-y0.norm());

    Vec3<float> B_O2P1_1{65,190,17};
    Vec3<float> F1_O2t1_1{65,54,-30};

    y0=B_O2P1_1-F1_O2t1_1;
    y=B_O2P1_1-ori::coordinateRotation(ori::CoordinateAxis::Z,DEG_0)*ori::coordinateRotation(ori::CoordinateAxis::X,DEG_1)*F1_O2t1_1;

    L1=(y.norm()-y0.norm());
    

}

void Biped::LJ0LJ1convert(float DEG_0,float DEG_1,float &L0,float &L1)
{
    DEG_0=deg2rad(DEG_0);
    DEG_1=deg2rad(DEG_1);

    Vec3<float> B_O2P1{-85,-190,17};
    Vec3<float> F1_O2t1{-85,-54,-30};


    Vec3<float> y0=B_O2P1-F1_O2t1;
    Vec3<float> y=B_O2P1-ori::coordinateRotation(ori::CoordinateAxis::Z,DEG_0)*ori::coordinateRotation(ori::CoordinateAxis::X,DEG_1)*F1_O2t1;
    
    L0=(y.norm()-y0.norm());

    Vec3<float> B_O2P1_1{85,-190,17};
    Vec3<float> F1_O2t1_1{85,-54,-30};

    y0=B_O2P1_1-F1_O2t1_1;
    y=B_O2P1_1-ori::coordinateRotation(ori::CoordinateAxis::Z,DEG_0)*ori::coordinateRotation(ori::CoordinateAxis::X,DEG_1)*F1_O2t1_1;

    L1=(y.norm()-y0.norm());
}

void Biped::RJ4RJ5convert(float DEG_4,float DEG_5,float &L4, float &L5)
{
    float DEG_40=55.29;
    DEG_40=deg2rad( DEG_40);
    DEG_4=deg2rad(DEG_4);
    DEG_5=deg2rad(DEG_5);

    float OA4=25,OB4=188.64;

    float AB0=sqrt(OA4*OA4+OB4*OB4-2*OA4*OB4*cos(DEG_40));
    float AB=sqrt(OA4*OA4+OB4*OB4-2*OA4*OB4*cos(DEG_40-DEG_4));
    L4=AB-AB0;

    Vec3<float> W_OA(0,-45,150);
    Vec3<float> OB_5(0,-45,0);

    Vec3<float> W_OB=ori::coordinateRotation(ori::CoordinateAxis::Y,DEG_4)*ori::coordinateRotation(ori::CoordinateAxis::X,DEG_5)*OB_5;
   // std::cout<<"W_OB:\n"<<W_OB<<'\n';
    Vec3<float> AB_initial(0,0,150);
    Vec3<float> W_AB=W_OA-W_OB;
    L5=W_AB.norm()-AB_initial.norm();
    
}

void Biped::LJ4LJ5convert(float DEG_4,float DEG_5,float &L4, float &L5)
{

    float DEG_40=55.29;
    DEG_40=deg2rad( DEG_40);
    DEG_4=deg2rad(DEG_4);
    DEG_5=deg2rad(DEG_5);

    float OA4=25,OB4=188.64;

    float AB0=sqrt(OA4*OA4+OB4*OB4-2*OA4*OB4*cos(DEG_40));
    float AB=sqrt(OA4*OA4+OB4*OB4-2*OA4*OB4*cos(DEG_40-DEG_4));
    L4=AB-AB0;

    Vec3<float> W_OA(0,45,150);
    Vec3<float> OB_5(0,45,0);

    Vec3<float> W_OB=ori::coordinateRotation(ori::CoordinateAxis::Y,DEG_4)*ori::coordinateRotation(ori::CoordinateAxis::X,DEG_5)*OB_5;
   // std::cout<<"W_OB:\n"<<W_OB<<'\n';
    Vec3<float> AB_initial(0,0,150);
    Vec3<float> W_AB=W_OA-W_OB;
    L5=W_AB.norm()-AB_initial.norm();



}


bool Biped::AngleInit(float &rj2,float & rj3,float &rj4,float &lj2,float & lj3,float &lj4)
{
    if(timer>2)
    {
        rj2=lj2=rad2deg(Initialq[2]);
        rj3=lj3=rad2deg(Initialq[3]);
         rj4=lj4=rad2deg(Initialq[4]);
         return true;
    }
    rj2=lj2=timer/2*rad2deg(Initialq[2]);
    rj3=lj3=timer/2*rad2deg(Initialq[3]);
    rj4=lj4=timer/2*rad2deg(Initialq[4]);

    timer+=sampletime;
    return false;

    
}