//Ruixiaoliang
//20180629
#include <stdio.h>
#include <string.h>
#define SSA_NUM 6
#define SS_NUM (SSA_NUM*3)
#define WH_NUM 18
#define WHR_NUM (WH_NUM*2)
#define SX0 0

#define T_1US   0.000001f
#define T_10US  0.00001f

float v_cal(float cur_t);

int main(void)
{
    float sx[SS_NUM];                       //传感器x坐标
    float wx[WH_NUM];                       //车轴x坐标
    float ds[SS_NUM-1];                     //传感器间距
    float dw[WH_NUM-1];                     //轴距
    
    int lwn[SS_NUM];                        //每个传感器的丢轴个数
    int rwn[SS_NUM];                        //每个传感器的多轴个数
    int swn[SS_NUM];                        //每个传感器经过的实际轮轴数，加上多轴，减去丢轴
    
    int ws[WH_NUM];                         //每个轴经过的传感器数
    int i,j;
    float dt = T_10US;                      //计算周期
    float time = 0;                         //绝对时间，单位s
    float v = 2;                            //车速
    float t[WHR_NUM][SS_NUM];               //绝对时刻表。第i个轴经过第j个传感器时刻
    float dwst[WHR_NUM][SS_NUM];            //相对时刻表。第i个轴经过第j个传感器与第i-1个轴经过第j个传感器的时间差，dt[0][j]表示第0个轴经过第j个传感器的时刻
                                            //与第0个轴经过第j-1个传感器的时间差。dt[0][0] == 0;
    //float vds[WH_NUM][SSA_NUM];           //第i个轮轴经过第j组传感器时的速度
    unsigned char lost_ax[WH_NUM][SS_NUM];  //丢轴标志表，表示第i个轮轴经过第j个传感器时有丢轴
    float red_ax[WH_NUM][SS_NUM];           //多轴时间表，表示第i个轮轴经过第j个传感器时，经过多少时间有一个多轴
    int ws_order[WH_NUM][SS_NUM];
    int order = 0;
    int dis_row = 0;

    memset(lost_ax,0,sizeof(lost_ax));
    memset(red_ax,0,sizeof(red_ax));
    memset(dwst,0,sizeof(dwst));
    memset(t,0,sizeof(t));
    memset(sx,0,sizeof(sx));
    memset(wx,0,sizeof(wx));
    memset(ws_order,0,sizeof(ws_order));
    memset(ws,0,sizeof(ws));
    memset(lwn,0,sizeof(lwn));
    memset(rwn,0,sizeof(rwn));
    //memset(swn,0,sizeof(swn));
    //丢轴设置
    lost_ax[10][9] = 1;
    lost_ax[12][9] = 1;
    lost_ax[10][11] = 1;
    
    //多轴设置
    red_ax[7][6] = T_10US;
    red_ax[14][6] = T_10US;
    //red_ax[16][6] = T_10US;
    red_ax[2][9] = T_10US;
    
    
    t[0][0] = 0.0f;
    
    printf("This is train id model agorithm. in calculating...");
    
    sx[0] = 0.0f;
    wx[0] = -1.0f;
    
    ds[0] = 0.295f;
    ds[1] = 0.360f;
    ds[2] = 4.150f;
    ds[3] = 0.327f;
    ds[4] = 0.350f;
    ds[5] = 4.150f;
    ds[6] = 0.354f;
    ds[7] = 0.235f;
    ds[8] = 4.150f;
    ds[9] = 0.210f;
    ds[10] = 0.350f;
    ds[11] = 4.150f;
    ds[12] = 0.206f;
    ds[13] = 0.344f;
    ds[14] = 4.150f;
    ds[15] = 0.223f;
    ds[16] = 0.345f;

    dw[0] = 1.873f;
    dw[1] = 7.979f;
    dw[2] = 1.864f;
    dw[3] = 2.989f;
    dw[4] = 1.863f;
    dw[5] = 7.946f;
    dw[6] = 1.869f;
    dw[7] = 3.043f;
    dw[8] = 1.858f;
    dw[9] = 7.955f;
    dw[10] = 1.843f;
    dw[11] = 3.785f;
    dw[12] = 2.031f;
    dw[13] = 1.992f;
    dw[14] = 4.689f;
    dw[15] = 2.043f;
    dw[16] = 1.977f;
    //printf("\nsx[0] is %f",sx[0]);
    //初始化传感器坐标
    for(i=0;i<SS_NUM-1;i++)
    {
        sx[i+1] = sx[i] + ds[i];
    }
    //初始化轮轴坐标
    for(i=0;i<WH_NUM-1;i++)
    {
        wx[i+1] = wx[i] - dw[i];
        //ws[i] = 0;
    }
    for(i=0;i<SS_NUM;i++)
    {
        swn[i] = WH_NUM;
    }
    for(j=0;j<SS_NUM;j++)
    {
        printf("\nsx[%d] is %f",j,sx[j]);
    }
    for(j=0;j<WH_NUM;j++)
    {
        printf("\nwx[%d] is %f",j,wx[j]);
    }
    //火车前进，计算轮轴坐标，判断何时有车轮压过传感器，记录绝对时间
    while(wx[WH_NUM-1] < sx[SS_NUM-1] + 30*dt)
    {
        time += dt;
        v = v_cal(time);
        for(i=0;i<WH_NUM;i++)
        {
            wx[i] += dt * v;
        }
        for(i=0;i<WH_NUM;i++)
        {
            for(j=ws[i];j<SS_NUM;j++)
            {
                if(wx[i] >= sx[j])
                {
                    ws[i]++;
                    t[i][j] = time;
                    //printf("\nwheel %d on sensor %d",i,j);
                    ws_order[i][j] = order++;
                    //v = v_cal();
                }
                else
                {
                    break;
                }
            }
        }
    }
    /*for(i=0;i<WH_NUM;i++)
    {
        printf("\nwx[%d]=%f",i,wx[i]);
    }
    for(i=0;i<SS_NUM;i++)
    {
        printf("\nsx[%d]=%f",i,sx[i]);
    }*/
    //显示绝对时间
    printf("\n\noriginal absolute time table\n");
    for(i=0;i<SS_NUM;i++)
    {
        printf("     s%2d  ",i);
    }
    for(i=0;i<WH_NUM;i++)
    {
        printf("\nw%2d: ",i);
        for(j=0;j<SS_NUM;j++)
        {
            printf("%9f ",t[i][j]);
        }
    }
    
    //轮轴经过传感器次序表
    printf("\n\norder table\n");
    for(i=0;i<WH_NUM;i++)
    {
        printf("\nw%2d: ",i);
        for(j=0;j<SS_NUM;j++)
        {
            printf("%9d ",ws_order[i][j]);
        }
    }
    
    //计算相对时间
    /*int losti = 0;
    int k;
    float a1,a2;
    for(j=0;j<SS_NUM;j++)
    {
        losti = 0;
        for(i=0;(i+losti)<WH_NUM-1;i++)
        {
            for(k=i;(k+losti)<WH_NUM-1;k++)
            {
                if(0 == lost_ax[i+1][j])
                {
                    
                }
            }
            if(lost_ax[i+1][j])
            {
                dwst[i+1][j] = t[i+2][j] - t[i][j];
                losti++;
            }
            else
            {
                dwst[i+1][j] = t[i+1+losti][j] - t[i+losti][j];
            }
        }
    }*/
    //处理丢轴多轴
    int k;
    //int radi = 0;
    //int losti = 0;
    for(j=0;j<SS_NUM;j++)
    {
        for(i=0;i<WH_NUM;i++)
        {
            if(lost_ax[i][j])
            {
                t[i][j] = 0;
                swn[j]--;
                lwn[j]++;
                /*for(k=i;k<WH_NUM-1;k++)
                {
                    t[k][j] = t[k+1][j];
                }*/
            }
        }
        for(i=WH_NUM-1;i>0;i--)
        {
            if(red_ax[i][j] > T_1US)
            {
                //radi++;
                swn[j]++;
                rwn[j]++;
                for(k=WH_NUM-1+rwn[j];k>i;k--)
                {
                    t[k][j] = t[k-1][j];
                }
                t[i+1][j] = t[i][j] + red_ax[i][j];
            }
        }
    }
    //处理丢轴多轴后绝对时间表
    /*printf("\n\nabsolute time table include lost and redundant\n");
    for(i=0;i<SS_NUM;i++)
    {
        printf("     s%2d  ",i);
    }
    for(i=0;i<WH_NUM+rwn[j];i++)
    {
        printf("\nw%2d: ",i);
        for(j=0;j<SS_NUM;j++)
        {
            printf("%9f ",t[i][j]);
        }
    }*/
    //整理丢轴
    for(j=0;j<SS_NUM;j++)
    {
        for(i=0;i<WH_NUM-1+rwn[j];i++)
        {
            if(t[i][j] < T_1US)
            {
                for(k=i;k<WH_NUM-1+rwn[j];k++)
                {
                    t[k][j] = t[k+1][j];
                }
            }
        }
    }
    //计算相对时刻表
    //计算周期除了第0行后面的数据
    for(j=0;j<SS_NUM;j++)
    {
        for(i=0;i<swn[j];i++)
        {
            dwst[i+1][j] = t[i+1][j] - t[i][j];
        }
    }
    //计算第0行数据
    for(j=0;j<SS_NUM-1;j++)
    {
        dwst[0][j+1] = t[0][j+1] - t[0][j];
    }
    dwst[0][0] = 0;

    //最终发送有效数据
    printf("\n\nabsolute table");
    printf("\nt=[");
    for(j=0;j<SS_NUM;j++)
    {
        for(i=0;i<swn[j];i++)
        {
            printf("%8f,",t[i][j]);
        }
    }
    printf("];");
    for(j=0;j<SS_NUM;j++)
    {
        printf("\ndwst%d=[",j,swn[j]);
        for(i=0;i<swn[j];i++)
        {
            printf("%8f,",dwst[i][j]);
        }
        printf("];",j);
    }
    printf("\n\nrelative table");
    for(j=0;j<SS_NUM;j++)
    {
        printf("\nS%2d: %d  ",j,swn[j]);
        for(i=0;i<swn[j];i++)
        {
            printf("%8f ",dwst[i][j]);
        }
    }
}

//速度更新函数
float v_cal(float cur_t)
{
    float v0 = 23;       //初始速度
    float v1 = 23;      //中间速度
    float v2 = 23;      //最终速度
    float t_v0_v1 = 2;  //初始速度到中间速度时间，单位s
    float t_v1_v2 = 8;  //中间速度到最终速度时间，单位s
    float v;
    if(cur_t < t_v0_v1)
    {
        return v0 + cur_t*(v1 - v0)/t_v0_v1;
    }
    else if(cur_t < t_v1_v2)
    {
        return v0 + cur_t*(v2 - v1)/t_v1_v2;
    }
    else
    {
        return v1 + cur_t*(v2 - v1)/t_v1_v2;
    }
}