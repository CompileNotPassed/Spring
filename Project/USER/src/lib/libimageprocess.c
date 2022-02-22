#include "headfile.h"

short GetOTSU(unsigned char tmImage[128][160])
{
    signed short i, j;
    unsigned long Amount = 0;
    unsigned long PixelBack = 0;
    unsigned long PixelshortegralBack = 0;
    unsigned long Pixelshortegral = 0;
    signed long PixelshortegralFore = 0;
    signed long PixelFore = 0;
    float OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma; // 类间方差;
    signed short MinValue, MaxValue;
    signed short Threshold = 0;
    unsigned char HistoGram[256]; //

    for (j = 0; j < 256; j++)
        HistoGram[j] = 0; //初始化灰度直方图

    for (j = 0; j < 128; j++)
    {
        for (i = 0; i < 160; i++)
        {
            HistoGram[tmImage[j][i]]++; //统计灰度级中每个像素在整幅图像中的个数
        }
    }

    for (MinValue = 0; MinValue < 256 && HistoGram[MinValue] == 0; MinValue++)
        ; //获取最小灰度的值
    for (MaxValue = 255; MaxValue > MinValue && HistoGram[MinValue] == 0; MaxValue--)
        ; //获取最大灰度的值

    if (MaxValue == MinValue)
        return MaxValue; // 图像中只有一个颜色
    if (MinValue + 1 == MaxValue)
        return MinValue; // 图像中只有二个颜色

    for (j = MinValue; j <= MaxValue; j++)
        Amount += HistoGram[j]; //  像素总数

    Pixelshortegral = 0;
    for (j = MinValue; j <= MaxValue; j++)
    {
        Pixelshortegral += HistoGram[j] * j; //灰度值总数
    }
    SigmaB = -1;
    for (j = MinValue; j < MaxValue; j++)
    {
        PixelBack = PixelBack + HistoGram[j];                                              //前景像素点数
        PixelFore = Amount - PixelBack;                                                    //背景像素点数
        OmegaBack = (float)PixelBack / Amount;                                             //前景像素百分比
        OmegaFore = (float)PixelFore / Amount;                                             //背景像素百分比
        PixelshortegralBack += HistoGram[j] * j;                                           //前景灰度值
        PixelshortegralFore = Pixelshortegral - PixelshortegralBack;                       //背景灰度值
        MicroBack = (float)PixelshortegralBack / PixelBack;                                //前景灰度百分比
        MicroFore = (float)PixelshortegralFore / PixelFore;                                //背景灰度百分比
        Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore); //计算类间方差
        if (Sigma > SigmaB)                                                                //遍历最大的类间方差g //找出最大类间方差以及对应的阈值
        {
            SigmaB = Sigma;
            Threshold = j;
        }
    }
    return Threshold; //返回最佳阈值;
}

//Frame Process
void getBin(uint8 (*in)[160], uint8 out[][160], uint8 mode)
{
    switch (mode)
    {
    case 1:
    { //原样返回
        for (uint8 i = 0; i < 128; i++)
        {
            for (uint8 j = 0; j < 160; j++)
            {
                out[i][j] = in[i][j];
            }
        }
        break;
    }
    case 2:
    { //OTSU
        uint8 threshold = GetOTSU(in);
        for (uint8 i = 0; i < 128; i++)
        {
            for (uint8 j = 0; j < 160; j++)
            {
                if (in[i][j] < threshold)
                {
                    out[i][j] = 0;
                }
                else
                {
                    out[i][j] = 255;
                }
            }
        }
        break;
    }
    }
}

void imgFilter(uint8 img[128][160], uint8 mode)
{
    switch (mode)
    {
    case 1:
    { //均值滤波
        uint16 sum;
        for (uint8 i = 0 + 1; i < 128 - 1; i++)
        {
            for (uint8 j = 0 + 1; j < 160 - 1; j++)
            {
                sum = img[i - 1][j - 1] + img[i - 1][j] + img[i - 1][j + 1] +
                      img[i][j - 1] + img[i][j] + img[i][j + 1] +
                      img[i + 1][j - 1] + img[i + 1][j] + img[i + 1][j + 1];
                img[i][j] = (uint8)(sum / 9.0);
            }
        }
        break;
    }
    case 2:
    {             //四向滤波
        uint8 nr; //行
        uint8 nc; //列

        for (nr = 1; nr < 128 - 1; nr++)
        {
            for (nc = 1; nc < 160 - 1; nc = nc + 1)
            {
                if ((img[nr][nc] == 0) && (img[nr - 1][nc] + img[nr + 1][nc] + img[nr][nc + 1] + img[nr][nc - 1] > 2 * 255))
                {
                    img[nr][nc] = 255;
                }
                else if ((img[nr][nc] == 255) && (img[nr - 1][nc] + img[nr + 1][nc] + img[nr][nc + 1] + img[nr][nc - 1] < 2 * 255))
                {
                    img[nr][nc] = 0;
                }
            }
        }
    }
    }
}

void sharpenImage(uint8 img[128][160])
{
    int i, j, temp;
    int sharpen[9] = {-1, -1, -1, -1, 9, -1, -1, -1, -1}; //拉普拉斯锐化模板
    for (i = 1; i < 128 - 1; i++)
    { //1~height-2
        for (j = 1; j < 160 - 1; j++)
        { //1~width-2
            temp = img[i - 1][j - 1] * sharpen[0];
            temp += img[i - 1][j] * sharpen[1];
            temp += img[i - 1][j + 1] * sharpen[2];
            temp += img[i][j - 1] * sharpen[3];
            temp += img[i][j] * sharpen[4];
            temp += img[i][j + 1] * sharpen[5];
            temp += img[i + 1][j - 1] * sharpen[6];
            temp += img[i + 1][j] * sharpen[7];
            temp += img[i + 1][j + 1] * sharpen[8];
            if (temp > 255)
            {
                img[i][j] = 255;
            }
            else if (temp < 0)
            {
                img[i][j] = abs(temp);
            }
            else
            {
                img[i][j] = temp;
            }
        }
    }
}
