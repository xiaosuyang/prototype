#ifndef LIP3D
#define LIP3D

#include "Eigen/Dense"
#include "utility/cppTypes.h"
#include "math/orientation_tools.h"
#include "math/MathUtilities.h"
#include <iostream>
using namespace Eigen;

class LIP3d
{

public:
    LIP3d(int Npreview)
    {
        Np=Npreview;
        An = 3;
        Un = 1;
        Cn = 1;
        A.resize(3, 3);
        B.resize(3, 1);
        C.resize(1, 3);

        A << 1, sampletime, pow(sampletime, 2)/2, 0, 1, sampletime, 0, 0, 1;
        B << pow(sampletime, 3) / 6, pow(sampletime, 2) / 2, sampletime;
        Zc=0.5;

        C << 1, 0, -1 * Zc / gravity;

        x0=MatrixXf::Zero(An,1);
    

        MatrixXf P;
	    P.resize(3, 3);

        //P<<60.6289,18.0762,0.0661,18.0762,5.5442,0.0659,0.0661,0.0659,0.0140;
        P<<60.6289,18.0762,0.0661,18.0762,5.5442,0.0659,0.0661,0.0659,0.0140;

        MatrixXf Q=MatrixXf::Ones(1,1), R=1e-6*MatrixXf::Ones(1,1);

        K.resize(1,3);
     
        K = (R + B.transpose() * P * B).inverse() * (B.transpose() * P * A);

        f.resize(1,Np);

        MatrixXf f1,f2;
        f1=(R+B.transpose()*P*B).inverse();
        f2=(A-B*K).transpose();

        for(int i=0;i<Np;i++)
        {
            f.block(0,i,1,1)=f1*B.transpose()*Matrix_Pow(f2,i)*C.transpose()*Q;
        }
        std::cout<<"Calculate Matrix\n"<<'\n';
        calculatepara();
    }

    void calculatepara()
    {
        MatrixXf C_dot_A=C*A;
        MatrixXf C_dot_B=C*B;

        MatrixXf A_tilde;
        A_tilde.resize(4,4);
        A_tilde<<1,C_dot_A(0,0),C_dot_A(0,1),C_dot_A(0,2)
        ,0,A(0,0),A(0,1),A(0,2)
        ,0,A(1,0),A(1,1),A(1,2)
        ,0,A(2,0),A(2,1),A(2,2);
        
        std::cout<<"A_tilde\n"<<A_tilde<<'\n';

        MatrixXf B_tilde;
        B_tilde.resize(4,1);
        B_tilde<<C_dot_B(0,0),B(0,0),B(1,0),B(2,0);

         std::cout<<"B_tilde\n"<<B_tilde<<'\n';

        MatrixXf C_tilde;
        C_tilde.resize(1,4);
        C_tilde<<1,0,0,0;

        MatrixXf P_tilde;
        P_tilde.resize(4,4);
        P_tilde<<47.2856740984495,1094.32465042238,252.933146084004,1.30363685153350
        ,1094.32465042238,26452.5641693346,6119.99502345253,32.8643060929554
       , 252.933146084004,6119.99502345253,1416.02481482394,7.63123788582405
       , 1.30363685153350,32.8643060929554,7.63123788582405,0.0475559641677950;

        MatrixXf Q=MatrixXf::Ones(1,1), R=1e-6*MatrixXf::Ones(1,1);

       MatrixXf K_tilde=(R+B_tilde.transpose()*P_tilde*B_tilde).inverse()*(B_tilde.transpose()*P_tilde*A_tilde);

        std::cout<<"K_tilde\n"<<K_tilde<<'\n';

       Ks.resize(1,1);
       Ks<<K_tilde(0,0);

       Kx.resize(1,K_tilde.cols()-1);

       Kx=K_tilde.block(0,1,1,K_tilde.cols()-1);

       MatrixXf Ac_tilde=A_tilde-B_tilde*K_tilde;

       G.resize(1,Np);

       G(0,0)=-Ks(0,0);

       MatrixXf I_tilde(4,1);
       I_tilde<<1,0,0,0;

       MatrixXf X_tilde=-Ac_tilde.transpose()*P_tilde*I_tilde;

       for(int i=0;i<Np;i++)
       {
            G.block(0,i,1,1)=(R+B_tilde.transpose()*P_tilde*B_tilde).inverse()*(B_tilde.transpose())*X_tilde;
            X_tilde=Ac_tilde.transpose()*X_tilde;

       }
       
        std::cout<<"G\n"<<G<<'\n';
    

    }



    const float gravity = 9.8;
    float Zc;

    MatrixXf A;
    MatrixXf B;
    MatrixXf C;
    MatrixXf x0;
    MatrixXf K;
    MatrixXf f;


    MatrixXf Ks;
    MatrixXf Kx;
    MatrixXf G;

    int An;
    int Un;
    int Cn;

    int Np;

    float sampletime = 0.01;
};

#endif