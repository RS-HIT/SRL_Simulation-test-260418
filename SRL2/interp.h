#ifndef INTERP_H
#define INTERP_H
/*
//spline trajectoryv
#define T_CUBIC								0.001f
#define DOFS_CUBIC							3
int interpSteps = 20;
float i_pos[DOFS_CUBIC], f_pos[DOFS_CUBIC], i_vel[DOFS_CUBIC], f_vel[DOFS_CUBIC], i_acc[DOFS_CUBIC], f_acc[DOFS_CUBIC], idxStep;
float a0[DOFS_CUBIC], a1[DOFS_CUBIC], a2[DOFS_CUBIC], a3[DOFS_CUBIC], a4[DOFS_CUBIC], a5[DOFS_CUBIC];

void linearInit(int nSteps, float p1[DOFS_CUBIC], float v1[DOFS_CUBIC])
{
    interpSteps = nSteps;
    for (int i = 0; i < DOFS_CUBIC; i++)
    {
        i_pos[i] = p1[i];
        i_vel[i] = v1[i];
        f_pos[i] = p1[i];
        f_vel[i] = v1[i];
    }
}

void cubicInit(int nSteps, float p1[DOFS_CUBIC], float v1[DOFS_CUBIC])
{
    interpSteps = nSteps;
    for (int i = 0; i < DOFS_CUBIC; i++)
    {
        i_pos[i] = p1[i];
        i_vel[i] = v1[i];
        f_pos[i] = p1[i];
        f_vel[i] = v1[i];
    }
}

void poly5Init(int nSteps, float p1[DOFS_CUBIC], float v1[DOFS_CUBIC])
{
    interpSteps = nSteps;
    for (int i = 0; i < DOFS_CUBIC; i++)
    {
        i_pos[i] = p1[i];
        i_vel[i] = v1[i];
        f_pos[i] = p1[i];
        f_vel[i] = v1[i];
    }
}

//匀加速
void linearBegin(float p1[DOFS_CUBIC], float v1[DOFS_CUBIC])
{
    for (int i = 0; i < DOFS_CUBIC; i++)
    {
        i_pos[i] = f_pos[i];
        i_vel[i] = f_vel[i];
        i_acc[i] = f_acc[i];
        f_pos[i] = p1[i];
        f_vel[i] = (2 * (f_pos[i] - i_pos[i])) / SIM_T - i_vel[i];
        f_acc[i] = (f_vel[i] - i_vel[i]) / SIM_T;
    }
    idxStep = 0;
}

//三次多项式
void cubicBegin(float p1[DOFS_CUBIC], float v1[DOFS_CUBIC])
{
    for (int i = 0; i < DOFS_CUBIC; i++)
    {
        i_pos[i] = f_pos[i];
        i_vel[i] = f_vel[i];
        f_pos[i] = p1[i];
        f_vel[i] = v1[i];
        float h = f_pos[i] - i_pos[i];
        float T = SIM_T;
        a0[i] = i_pos[i];
        a1[i] = i_vel[i];
        a2[i] = (3 * h - (2 * i_vel[i] + f_vel[i]) * T) / (T * T);
        a3[i] = (-2 * h + (i_vel[i] + f_vel[i]) * T) / (T * T * T);
    }
    idxStep = 0;
}
//五次多项式
void poly5Begin(float pos[DOFS_CUBIC], float vel[DOFS_CUBIC], float acc[DOFS_CUBIC])
{
    for (int i = 0; i < DOFS_CUBIC; i++)
    {
        i_pos[i] = f_pos[i];
        i_vel[i] = f_vel[i];
        i_acc[i] = f_acc[i];
        f_pos[i] = pos[i];
        f_vel[i] = vel[i];
        f_acc[i] = acc[i];
        float h = f_pos[i] - i_pos[i];
        float T = T_CUBIC * interpSteps;
        a0[i] = i_pos[i];
        a1[i] = i_vel[i];
        a2[i] = 0.5f * i_acc[i];
        a3[i] = (20 * h - (8 * f_vel[i] + 12 * i_vel[i]) * T - (3 * i_acc[i] - f_acc[i]) * T * T) / (2 * T * T * T);
        a4[i] = (-30 * h + (14 * f_vel[i] + 16 * i_vel[i]) * T + (3 * i_acc[i] - 2 * f_acc[i]) * T * T) / (2 * T * T * T * T);
        a5[i] = (12 * h - (6 * f_vel[i] + 6 * i_vel[i]) * T - (i_acc[i] - f_acc[i]) * T * T) / (2 * T * T * T * T * T);
    }
    idxStep = 0;
}

int linearStep(float p[DOFS_CUBIC], float v[DOFS_CUBIC], float a[DOFS_CUBIC])
{
    if (idxStep <= interpSteps)
    {
        for (int i = 0; i < DOFS_CUBIC; i++)
        {
            float t = T_CUBIC * idxStep;
            p[i] = i_pos[i] + i_vel[i] * t + 0.5f * f_acc[i] * t * t;
            v[i] = i_vel[i] + f_acc[i] * t;
            a[i] = f_acc[i];
        }
        idxStep++;
    }
    return idxStep;
}

int cubicStep(float p[DOFS_CUBIC], float v[DOFS_CUBIC], float a[DOFS_CUBIC])
{
    if (idxStep <= interpSteps)
    {
        float t = T_CUBIC * idxStep;
        for (int i = 0; i < DOFS_CUBIC; i++)
        {
            p[i] = a0[i] + a1[i] * t + a2[i] * t * t + a3[i] * t *t * t;
            v[i] = a1[i] + 2 * a2[i] * t + 3 * a3[i] * t * t;
            a[i] = 2 * a2[i] + 6 * a3[i] * t;
        }
        idxStep++;
    }
    return idxStep;
}
//五次多项式
int poly5Step(float p[DOFS_CUBIC], float v[DOFS_CUBIC], float a[DOFS_CUBIC])
{
    if (idxStep <= interpSteps)
    {
        float t = T_CUBIC * idxStep;
        for (int i = 0; i < DOFS_CUBIC; i++)
        {
            p[i] = a0[i] + a1[i] * t + a2[i] * t * t + a3[i] * t * t * t + a4[i] * t * t * t * t + a5[i] * t * t * t * t * t;
            v[i] = a1[i] + 2 * a2[i] * t + 3 * a3[i] * t * t + 4 * a4[i] * t * t * t + 5 * a5[i] * t * t * t * t;
            a[i] = 2 * a2[i] + 6 * a3[i] * t + 12 * a4[i] * t * t + 20 * a5[i] * t * t * t;
        }
        idxStep++;
    }
    return idxStep;
}
*/
#endif // INTERP_H
