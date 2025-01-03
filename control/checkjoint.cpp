#include "../include/checkjoint.h"

extern float ifswing;
extern float Deg[3];
extern float KuanDeg[2];
extern float rj0angle, rj1angle, rj2angle, rj3angle, rj4angle, rj5angle;
extern float LDeg[3];
extern float LKuanDeg[2];
extern float lj0angle, lj1angle, lj2angle, lj3angle, lj4angle, lj5angle;
extern float zmpscope[4];
extern float cphasescope[2];
extern float walkphase;
extern float dyx;
extern float dyy;
extern float uxx;
extern float uyy;
extern float rlegswingphase,rlegcontactphase;

// void Checkjoint::checkgait()
// {
//     auto gait = walking;
//     auto &seResult = _stateEstimator->getResult();
//     // get then foot location in world frame
//     for (int i = 0; i < 2; i++)
//     {
//         pFoot[i] = seResult.position + seResult.rBody.transpose() * (statectrl->_biped.getHip2Location(i) +
//                                                                      statectrl->data[i].p);
//     }
//     cout << "右足世界位置:\n"
//          << pFoot[0] << '\n';
//     cout << "左足世界位置:\n"
//          << pFoot[1] << '\n';

//     if (firstRun)
//     {
//         std::cout << "First Run " << std::endl;
//         for (int i = 0; i < 2; i++)
//         {
//             footSwingTrajectories[i].setHeight(0.05);
//             footSwingTrajectories[i].setInitialPosition(pFoot[i]);
//             footSwingTrajectories[i].setFinalPosition(pFoot[i]);
//         }
//         firstRun = false;
//     }

//     swingTimes[0] = dtMPC * gait->_swing;
//     swingTimes[1] = dtMPC * gait->_swing;
//     Vec3<double> Pf;
//     double side_sign[2] = {1, -1};
//     for (int i = 0; i < 2; i++)
//     {
//         if (firstSwing[i])
//         {
//             swingTimeRemaining[i] = swingTimes[i];
//         }
//         else
//         {
//             swingTimeRemaining[i] -= dt;
//         }
//         footSwingTrajectories[i].setHeight(0.1);
//         Pf[0] = seResult.position[0];
//         Pf[1] = seResult.position[1];
//         Pf[2] = seResult.position[2] - 0.894;

//         footSwingTrajectories[i].setFinalPosition(Pf);
//         Footpos[i] = Pf.cast<float>();
//     }
//     Mat33<double> Kp;
//     Mat33<double> Kd;
//     Kp << 300, 0, 0,
//         0, 300, 0,
//         0, 0, 300;
//     Kd << 10, 0, 0,
//         0, 10, 0,
//         0, 0, 10;

//     gait->setIterations(iterationsBetweenMPC, iterationCounter);
//     Vec2<double> contactStates = gait->getContactSubPhase();
//     Vec2<double> swingStates = gait->getSwingSubPhase();

//     iterationCounter++;

//     Vec6<double> QDes[2];
//     Vec6<double> QdDes[2];
//     Vec3<double> Pdes[2];
//     Pdes[0] << 0, 0, -0.894;
//     Pdes[1] << 0, 0, -0.894;
//     statectrl->commands[0].kpCartesian = Kp;
//     statectrl->commands[0].kdCartesian = Kd;
//     statectrl->commands[1].kpCartesian = Kp;
//     statectrl->commands[1].kdCartesian = Kd;

//     for (int foot = 0; foot < 2; foot++)
//     {
//         double contactState = contactStates(foot);
//         double swingState = swingStates(foot);
//         cout << "腿" << foot << "contactstate:" << contactState << '\n';
//         cout << "腿" << foot << "swingstate:" << swingState << '\n';
//         if (swingState > 0)
//         {
//             if (firstSwing[foot])
//             {
//                 firstSwing[foot] = false;
//                 footSwingTrajectories[foot].setInitialPosition(Pf);
//             }
//             footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);
//             pDesFootWorld[foot] = footSwingTrajectories[foot].getPosition().cast<double>();
//             vDesFootWorld[foot] = footSwingTrajectories[foot].getVelocity().cast<double>();
//             pDesFoot[foot] = seResult.rBody * (pDesFootWorld[foot] - seResult.position);
//             vDesFoot[foot] = seResult.rBody * (vDesFootWorld[foot] - seResult.vWorld);

//             Pdes[foot] = pDesFoot[foot];
//             computeIK(statectrl->_biped, QDes[foot], &Pdes[foot], 0);
//             Vec6<double> vdess = Vec6<double>::Zero();
//             vdess.block<3, 1>(0, 0) = vDesFoot[foot];
//             QdDes[foot] = statectrl->data[foot].J_force_moment.inverse() * vdess;
//             statectrl->commands[foot].qDes = QDes[foot];
//             statectrl->commands[foot].qdDes = QdDes[foot];
//             statectrl->commands[foot].pDes = Pdes[foot];
//             statectrl->commands[foot].vDes = vDesFoot[foot];
//         }
//         else if (contactState > 0)
//         {
//             firstSwing[foot] = true;
//             Pdes[foot] << 0, 0, -0.894;
//             computeIK(statectrl->_biped, QDes[foot], &Pdes[foot], 0);
//             statectrl->commands[foot].qDes = QDes[foot];
//             statectrl->commands[foot].qdDes = Vec6<double>::Zero();
//             statectrl->commands[foot].pDes = Pdes[foot];
//         }
//     }

//     // cout << "\n右脚坐标:\n"
//     //      << Footpos[0] << '\n';
//     // cout << "设置的抬脚高度：" << footSwingTrajectories[0].getheight();
//     // plot_publish(0,0.2*sin(0.01*iterationCounter)+0.5);
//     // plot_publish(0, pDesFoot[0][0]);
//     // plot_publish(1, pDesFoot[1][2]);
//     // plot_publish(2, pDesFoot[0][2]);

// }

void Checkjoint::zmpgenerate(int deltapre) // deltapre:0~previewnum-1
{
    int iterpre = iterationCounter + deltapre;
    zmppre->setIterations(iterationsBetweenMPC, iterpre);
    float thisphase = zmppre->_phase;
    float lastphase = lastphaselist[deltapre];

    if (thisphase < lastphase)
        Iterlist[deltapre]++;

    int Iter = Iterlist[deltapre];

    if (Iter > 0 && thisphase > 5e-3 && thisphase < zmpChangep + 1e-3 && Iter < totalstep)
    {

        if (thisphase > zmpChangep)
            thisphase = zmpChangep;
        float cphase = thisphase / zmpChangep;
        xreflist[deltapre] = zmpxposref[Iter - 1] * (1 - cphase) + zmpxposref[Iter] * cphase;
        yreflist[deltapre] = zmpyposref[Iter - 1] * (1 - cphase) + zmpyposref[Iter] * cphase;
    }

    lastphaselist[deltapre] = thisphase;
    
}

void Checkjoint::zmpLegphasecompute()
{
    static int legphasecounter=1;
    if (Iter < 1)
    {
    }
    else
    {
        zmpfoot->setIterations(iterationsBetweenMPC, legphasecounter);
        Vec2<double> contactStates = zmpfoot->getContactSubPhase();
        Vec2<double> swingStates = zmpfoot->getSwingSubPhase();
        legphasecounter++;

        rlegswingphase=swingStates[0];
        rlegcontactphase=contactStates[0];

        Vec3<double> Pf[2], Pfstance[2];
        double side_sign[2] = {-1, 1};

        for(int i=0;i<2;i++)
        {
            Pf[i][1]=side_sign[i]*statectrl->_biped.leg_offset_y*0.9;
            Pf[i][2]=-1.12;

            int nextposindex=Iter+1;
            if(nextposindex<totalstep)
                Pf[i][1]=zmpxposref[nextposindex];
            




            //footSwingTrajectories[i].setHeight(0.055);
        }

        

        // for (int i = 0; i < 2; i++)
        // {

        //     Pf[i][1] = side_sign[i] * statectrl->_biped.leg_offset_y;
        //     Pf[i][2] = -1.1294;
  
        //     Pfstance[i] = Pf[i];

        //     if (swingStates(i) > 0)
        //     {
        //         Pfstance[i][0] = footxnow[i];
        //         Pf[i][0] = footxnextsquat[i];
        //     }
        //     else
        //     {
        //         Pfstance[i][0] = footxnextsquat[i];
        //         Pf[i][0] = footxnow[i];
        //     }

        //     footSwingTrajectories[i].setHeight(0.065);

        // }
    }
}

void Checkjoint::zmpwalk()
{
    Gait *gait = walkzmp;
    static int zmpstep = 0;
    static float lastphase = 0;
    gait->setIterations(iterationsBetweenMPC, iterationCounter);
    iterationCounter++;
    // static int Iter = 0;

    // // if ((1 - lastphase) < 1e-2 && gait->_phase < 1e-2)
    // //     Iter++;
    if (gait->_phase < lastphase)
        Iter++;

    for (int i = 0; i < PREVIEWNUM; i++)
    {
        zmpgenerate(i);
    }

    MatrixXf zmpxpreview;
    MatrixXf zmpypreview;
    zmpxpreview.resize(PREVIEWNUM, 1);
    zmpypreview.resize(PREVIEWNUM, 1);

    std::cout << "xref:\n"
              << '\n';

    for (int i = 0; i < PREVIEWNUM; i++)
    {
        zmpxpreview(i, 0) = xreflist[i];
        zmpypreview(i, 0) = yreflist[i];
        // std::cout<<zmpxpreview(i,0)<<'\n';
    }

    zmpscope[0] = xreflist[0];
    zmpscope[1] = yreflist[0];
    zmpscope[2] = xreflist[PREVIEWNUM - 1];
    zmpscope[3] = yreflist[PREVIEWNUM - 1];

    walkphase = gait->_phase;

    lastphase = gait->_phase;

    cout << "zmpwalk 迭代步: " << Iter << '\n';

    // std::cout<<"Ks:\n"<<dymodelx->Ks<<'\n';

    // std::cout<<"Kx:\n"<<dymodelx->Kx<<'\n';

    // std::cout<<"G:\n"<<dymodelx->G<<'\n';

    MatrixXf ZMP_x = dymodelx->C * dymodelx->x0;
    float ex = ZMP_x(0, 0) - zmpxpreview(0, 0);

    MatrixXf ZMP_y = dymodely->C * dymodely->x0;
    float ey = ZMP_y(0, 0) - zmpypreview(0, 0);

    sum_ex += ex;
    sum_ey += ey;

    MatrixXf sum_exM;
    sum_exM.resize(1, 1);
    sum_exM << sum_ex;

    MatrixXf sum_eyM;
    sum_eyM.resize(1, 1);
    sum_eyM << sum_ey;

    Eigen::MatrixXf ux, uy;
    // std::cout<<"计算控制量"<<'\n';

    ux = -dymodelx->Ks * sum_exM - dymodelx->Kx * dymodelx->x0 - dymodelx->G * zmpxpreview;
    uy = -dymodely->Ks * sum_eyM - dymodely->Kx * dymodely->x0 - dymodely->G * zmpypreview;

    // std::cout<<"计算状态量"<<'\n';
    dymodelx->x0 = dymodelx->A * dymodelx->x0 + dymodelx->B * ux;
    dymodely->x0 = dymodely->A * dymodely->x0 + dymodely->B * uy;

    dyx = dymodelx->x0(0, 0);
    dyy = dymodely->x0(0, 0);

    zmpLegphasecompute();

    // uxx = ZMP_x(0, 0);
    // uyy = ZMP_y(0, 0);
}

void Checkjoint::squat()
{
    Gait *gait = gaitsquat;

    const StateEstimate &seResult = _stateEstimator->getResult();
    // get then foot location in world frame

    Vec3<double> walkspeed(0.5, 0, 0);

    swingTimes[0] = dtMPC * gait->_swing;
    swingTimes[1] = dtMPC * gait->_swing;
    Vec3<double> Pf[2], Pfstance[2];
    double side_sign[2] = {-1, 1};

    gait->setIterations(iterationsBetweenMPC, iterationCounter);

    Vec2<double> contactStates = gait->getContactSubPhase();
    Vec2<double> swingStates = gait->getSwingSubPhase();

    for (int i = 0; i < 2; i++)
    {

        double pfx_real = walkspeed[0] * 0.5 * gait->_stance * dtMPC;

        Pf[i][1] = side_sign[i] * statectrl->_biped.leg_offset_y;

        Pf[i][2] = -1.1294;
        Pf[i][0] = footxnow[i];

        Pfstance[i] = Pf[i];

        if (swingStates(i) > 0)
        {
            Pfstance[i][0] = footxnow[i];
            Pf[i][0] = footxnextsquat[i];
        }
        else
        {
            Pfstance[i][0] = footxnextsquat[i];
            Pf[i][0] = footxnow[i];
        }

        footSwingTrajectories[i].setHeight(0.065);
        //   footSwingTrajectories[i].setFinalPosition(Pf[i]);
    }

    // Mat33<double> Kp;
    // Mat33<double> Kd;
    // Kp << 50, 0, 0,
    //     0, 50, 0,
    //     0, 0, 50;
    // Kd << 2, 0, 0,
    //     0, 2, 0,
    //     0, 0, 2;

    iterationCounter++;

    Vec6<double> QDes[2];
    Vec6<double> QdDes[2];
    Vec3<double> Pdes[2];
    // Pdes[0] << 0, 0, -0.894-statectrl->_biped.leg_offset_z;
    // Pdes[1] << 0, 0, -0.894-statectrl->_biped.leg_offset_z;
    // statectrl->commands[0].kpCartesian = Kp;
    // statectrl->commands[0].kdCartesian = Kd;
    // statectrl->commands[1].kpCartesian = Kp;
    // statectrl->commands[1].kdCartesian = Kd;
    ifswing = swingStates(0);

    for (int foot = 0; foot < 2; foot++)
    {
        double contactState = contactStates(0);
        double swingState = swingStates(0);
        // cout << "腿" << foot << "contactstate:" << contactState << '\n';
        // cout << "腿" << foot << "swingstate:" << swingState << '\n';
        if (swingState > 0)
        {
            if (foot == 0 && firstSwing[foot])
            {
                firstSwing[foot] = false;
                if (!firstRun)
                {
                    footxnow[foot] = footxnextsquat[foot];
                    Pfstance[foot][0] = footxnow[foot];
                    Pf[foot][0] = footxnextsquat[foot];
                }

                footSwingTrajectories[foot].setInitialPosition(Pfstance[foot]);
            }
            else if (foot == 1 && firstSwing[foot] == false)
            {
                firstSwing[foot] = true;
                footSwingTrajectories[foot].setInitialPosition(Pfstance[foot]);
            }
            footSwingTrajectories[foot].setFinalPosition(Pf[foot]);
            footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);

            pDesFootWorld[foot] = footSwingTrajectories[foot].getPosition().cast<double>();
            vDesFootWorld[foot] = footSwingTrajectories[foot].getVelocity().cast<double>();
            pDesFoot[foot] = seResult.rBody * (pDesFootWorld[foot] - seResult.position);
            vDesFoot[foot] = seResult.rBody * (vDesFootWorld[foot] - seResult.vWorld);

            Pdes[foot] = pDesFoot[foot];
            IKinbodyframe(statectrl->_biped, QDes[foot], &Pdes[foot], foot);
        }
        else if (contactState > 0)
        {
            if (foot == 0 && firstSwing[foot] == false)
            {
                firstSwing[foot] = true;
                footSwingTrajectories[foot].setInitialPosition(Pf[foot]);
            }
            else if (foot == 1 && firstSwing[foot])
            {
                firstSwing[foot] = false;
                footSwingTrajectories[foot].setInitialPosition(Pf[foot]);
            }
            footSwingTrajectories[foot].setFinalPosition(Pfstance[foot]);

            footSwingTrajectories[foot].computeSwingTrajectoryBezier(contactState, swingTimes[foot]);

            pDesFootWorld[foot] = footSwingTrajectories[foot].getPosition().cast<double>();
            vDesFootWorld[foot] = footSwingTrajectories[foot].getVelocity().cast<double>();
            pDesFootWorld[foot][2] = Pfstance[foot][2];
            pDesFoot[foot] = seResult.rBody * (pDesFootWorld[foot] - seResult.position);
            vDesFoot[foot] = seResult.rBody * (vDesFootWorld[foot] - seResult.vWorld);

            Pdes[foot] = pDesFoot[foot];

            IKinbodyframe(statectrl->_biped, QDes[foot], &Pdes[foot], foot);
        }
    }

    if (firstRun)
    {
        firstRun = false;
    }

    rj0angle = rad2deg((float)QDes[0][0]);
    rj1angle = rad2deg((float)QDes[0][1]);
    rj2angle = rad2deg((float)QDes[0][2]);
    rj3angle = rad2deg((float)QDes[0][3]);
    rj4angle = rad2deg((float)QDes[0][4]);
    rj5angle = rad2deg((float)QDes[0][5]);

    lj0angle = rad2deg((float)QDes[1][0]);
    lj1angle = rad2deg((float)QDes[1][1]);
    lj2angle = rad2deg((float)QDes[1][2]);
    lj3angle = rad2deg((float)QDes[1][3]);
    lj4angle = rad2deg((float)QDes[1][4]);
    lj5angle = rad2deg((float)QDes[1][5]);

    Deg[2] = Pdes[0][2]; // 足部位置Z
    Deg[1] = Pdes[0][1];
    Deg[0] = Pdes[0][0];
    LDeg[2] = Pdes[1][2];
    LDeg[1] = Pdes[1][1];
    LDeg[0] = Pdes[1][0];
}

void Checkjoint::checkgait1()
{
    Gait *gait = walking;
    //  Gait* gaitpre=walkpre;

    const StateEstimate &seResult = _stateEstimator->getResult();
    // get then foot location in world frame
    for (int i = 0; i < 2; i++)
    {
        pFoot[i] = seResult.position + seResult.rBody.transpose() * (statectrl->_biped.getHip2Location(i) +
                                                                     statectrl->data[i].p);

        // pFoot[i] = seResult.position + Mat33<double>::Identity() * (statectrl->_biped.getHip2Location(i) +
        //                                                              statectrl->data[i].p);
    }

    Vec3<double> walkspeed(0.5, 0, 0);

    swingTimes[0] = dtMPC * gait->_swing;
    swingTimes[1] = dtMPC * gait->_swing;
    Vec3<double> Pf[2], Pfstance[2];
    double side_sign[2] = {-1, 1};

    gait->setIterations(iterationsBetweenMPC, iterationCounter);
    // gaitpre->setIterations(iterationsBetweenMPC,iterationCounter+10);

    Vec2<double> contactStates = gait->getContactSubPhase();
    Vec2<double> swingStates = gait->getSwingSubPhase();

    //  Vec2<double> contactStatespre = gaitpre->getContactSubPhase();
    // Vec2<double> swingStatespre = gaitpre->getSwingSubPhase();

    for (int i = 0; i < 2; i++)
    {
        // if (firstSwing[i])
        // {
        //     swingTimeRemaining[i] = swingTimes[i];
        // }
        // else
        // {
        //     swingTimeRemaining[i] -= dt;
        // }

        //   Vec3<double> pRobotFrame = statectrl->_biped.getHip2Location(i);//trunk frame下髋关节位置
        // Pf = seResult.position +seResult.rBody.transpose() * pRobotFrame + walkspeed * swingTimeRemaining[i];
        double pfx_real = walkspeed[0] * 0.5 * gait->_stance * dtMPC;
        //  Pf[i][0]=0.1;
        Pf[i][1] = side_sign[i] * statectrl->_biped.leg_offset_y * 0.9;
        // std::cout<<"legoffsetY"<<statectrl->_biped.leg_offset_y<<'\n';
        // std::cout<<"Y方向"<<i<<":"<<Pf[i][1]<<'\n';

        // Pf[i][1]=0;
        // Pf[i][2] = seResult.position[2] -0.8697+statectrl->_biped.leg_offset_z;
        // Pf[i][2]=-1.1023;
        Pf[i][2] = -1.1294;

        Pf[i][0] = footxnow[i];

        // Pf[i][2]=0-0.894+statectrl->_biped.leg_offset_z;

        //     cout << "世界位置:\n"
        //  << seResult.position << '\n';

        Pfstance[i] = Pf[i];
        // Pfstance[i][0]*=-1;

        if (swingStates(i) > 0)
        {
            Pfstance[i][0] = footxnow[i];
            Pf[i][0] = footxnext[i];
        }
        else
        {
            Pfstance[i][0] = footxnext[i];
            Pf[i][0] = footxnow[i];
        }

        footSwingTrajectories[i].setHeight(0.065);
        //   footSwingTrajectories[i].setFinalPosition(Pf[i]);
    }

    // Mat33<double> Kp;
    // Mat33<double> Kd;
    // Kp << 50, 0, 0,
    //     0, 50, 0,
    //     0, 0, 50;
    // Kd << 2, 0, 0,
    //     0, 2, 0,
    //     0, 0, 2;

    iterationCounter++;

    Vec6<double> QDes[2];
    Vec6<double> QdDes[2];
    Vec3<double> Pdes[2];
    // Pdes[0] << 0, 0, -0.894-statectrl->_biped.leg_offset_z;
    // Pdes[1] << 0, 0, -0.894-statectrl->_biped.leg_offset_z;
    // statectrl->commands[0].kpCartesian = Kp;
    // statectrl->commands[0].kdCartesian = Kd;
    // statectrl->commands[1].kpCartesian = Kp;
    // statectrl->commands[1].kdCartesian = Kd;
    ifswing = swingStates(0);

    for (int foot = 0; foot < 2; foot++)
    {
        double contactState = contactStates(foot);
        double swingState = swingStates(foot);
        // cout << "腿" << foot << "contactstate:" << contactState << '\n';
        // cout << "腿" << foot << "swingstate:" << swingState << '\n';
        if (swingState > 0)
        {
            if (firstSwing[foot])
            {
                firstSwing[foot] = false;
                if (!firstRun)
                {
                    footxnow[foot] = footxnext[foot];
                    footxnext[foot] *= -1;
                    Pfstance[foot][0] = footxnow[foot];
                    Pf[foot][0] = footxnext[foot];
                }

                footSwingTrajectories[foot].setInitialPosition(Pfstance[foot]);
            }
            footSwingTrajectories[foot].setFinalPosition(Pf[foot]);
            footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);

            pDesFootWorld[foot] = footSwingTrajectories[foot].getPosition().cast<double>();
            vDesFootWorld[foot] = footSwingTrajectories[foot].getVelocity().cast<double>();
            pDesFoot[foot] = seResult.rBody * (pDesFootWorld[foot] - seResult.position);
            vDesFoot[foot] = seResult.rBody * (vDesFootWorld[foot] - seResult.vWorld);

            Pdes[foot] = pDesFoot[foot];
            IKinbodyframe(statectrl->_biped, QDes[foot], &Pdes[foot], foot);
        }
        else if (contactState > 0)
        {
            if (firstSwing[foot] == false)
            {
                firstSwing[foot] = true;
                if (!firstRun)
                {
                    footxnow[foot] = footxnext[foot];
                    footxnext[foot] *= -1;
                    Pfstance[foot][0] = footxnext[foot];
                    Pf[foot][0] = footxnow[foot];
                }

                footSwingTrajectories[foot].setInitialPosition(Pf[foot]);
                // footxnow[foot]=footxnext[foot];
                // footxnext[foot]*=-1;
            }
            footSwingTrajectories[foot].setFinalPosition(Pfstance[foot]);

            footSwingTrajectories[foot].computeSwingTrajectoryBezier(contactState, swingTimes[foot]);

            pDesFootWorld[foot] = footSwingTrajectories[foot].getPosition().cast<double>();
            vDesFootWorld[foot] = footSwingTrajectories[foot].getVelocity().cast<double>();
            pDesFootWorld[foot][2] = Pfstance[foot][2];
            pDesFoot[foot] = seResult.rBody * (pDesFootWorld[foot] - seResult.position);
            vDesFoot[foot] = seResult.rBody * (vDesFootWorld[foot] - seResult.vWorld);

            Pdes[foot] = pDesFoot[foot];

            IKinbodyframe(statectrl->_biped, QDes[foot], &Pdes[foot], foot);
        }
    }

    if (firstRun)
    {
        firstRun = false;
    }

    rj0angle = rad2deg((float)QDes[0][0]);
    rj1angle = rad2deg((float)QDes[0][1]);
    rj2angle = rad2deg((float)QDes[0][2]);
    rj3angle = rad2deg((float)QDes[0][3]);
    rj4angle = rad2deg((float)QDes[0][4]);
    rj5angle = rad2deg((float)QDes[0][5]);

    lj0angle = rad2deg((float)QDes[1][0]);
    lj1angle = rad2deg((float)QDes[1][1]);
    lj2angle = rad2deg((float)QDes[1][2]);
    lj3angle = rad2deg((float)QDes[1][3]);
    lj4angle = rad2deg((float)QDes[1][4]);
    lj5angle = rad2deg((float)QDes[1][5]);

    Deg[2] = Pdes[0][2]; // 足部位置Z
    Deg[1] = Pdes[0][1];
    Deg[0] = Pdes[0][0];
    LDeg[2] = Pdes[1][2];
    LDeg[1] = Pdes[1][1];
    LDeg[0] = Pdes[1][0];

    // plot_publish(0,footSwingTrajectories[0].getPosition()[0]);
    // plot_publish(1, footSwingTrajectories[0].getPosition()[1]);
    // plot_publish(2,footSwingTrajectories[0].getPosition()[2]);
}
