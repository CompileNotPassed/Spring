#include "headfile.h"

extern int16 horLine[2][128];
extern int16 verLine[160][2];

void horizontalDetect(uint8 img[128][160])
{
    //TODO LOST
#define LOST_FLAG 0xff;
    uint8 i, left, right, color = 255;
    for (i = 127; i > 120; i--)
    {
        for (left = 80; (img[i][left] == img[i][left - 1]) && (left > 1); left--)
        {
        }
        for (right = 80; (img[i][right] == img[i][right + 1]) && (right < 158); right++)
        {
        }
        img[i][left] = 103;
        img[i][left + 1] = 103;
        img[i][right] = 106;
        img[i][right - 1] = 106;
    }
    if (left == 0)
    {
        left = 1;
    }
    else if (left == 159)
    {
        left = 158;
    }
    if (right == 0)
    {
        right = 1;
    }
    else if (right == 159)
    {
        right = 158;
    }
    for (; i > 1; i--)
    {
        if (img[i - 1][left] != color)
        {
            for (; (img[i][left] == img[i][left + 1]) && (left < 127); left++)
            {
            }
        }
        else
        {
            for (; (img[i][left] == img[i][left - 1]) && (left > 1); left--)
            {
            }
        }
        if (img[i - 1][right] != color)
        {
            for (; (img[i][right] == img[i][right - 1]) && (right > 1); right--)
            {
            }
        }
        else
        {
            for (; (img[i][right] == img[i][left + 1]) && (right < 158); right++)
            {
            }
        }
        img[i][left] = 103;
        img[i][left + 1] = 103;
        img[i][right] = 106;
        img[i][right - 1] = 106;
    }
#undef LOST_FLAG
}

// void verticalDetect(uint8 img[128][160], int16 verLine[2][160], uint8 mode)
// {
//     //TODO LOST//不会写了 我菜死了
// #define LOST_FLAG 0xff;
//     uint8 i, up, down, color = img[45][80];
//     for (i = 80; i > 70; i--)
//     {
//         for (up = 45; (img[up][i] == img[up + 1][i]) && (up > 1); up--)
//         {
//         }
//         for (down = 45 + 1; (img[down][i] == img[down - 1][i]) && (down < 127); down++)
//         {
//         }
//         img[up][i] = 103;
//         img[up + 1][i] = 103;
//         img[down][i] = 106;
//         img[down - 1][i] = 106;
//     }
//     if (up == 0)
//     {
//         up = 1;
//     }
//     else if (up == 128)
//     {
//         up = 127;
//     }
//     if (down == 0)
//     {
//         down = 1;
//     }
//     else if (down == 128)
//     {
//         down = 127;
//     }
//     for (; i > 1; i--)
//     {
//         if (img[up][i-1] != color)
//         {
//             for (; (img[up][i] == img[up][left + 1]) && (left < 127); left++)
//             {
//             }
//         }
//         else
//         {
//             for (; (img[i][left] == img[i][left - 1]) && (left > 1); left--)
//             {
//             }
//         }
//         if (img[i - 1][right] != color)
//         {
//             for (; (img[i][right] == img[i][right - 1]) && (right > 1); right--)
//             {
//             }
//         }
//         else
//         {
//             for (; (img[i][right] == img[i][left + 1]) && (right < 158); right++)
//             {
//             }
//         }
//         img[up][i] = 103;
//         img[up + 1][i] = 103;
//         img[down][i] = 106;
//         img[down - 1][i] = 106;
//     }
// #undef LOST_FLAG
// }


// void borderDetect(uint8 img[][160], uint8 img[][160])
// {
//     uint8 i, left, right, color = img[128][80];
//     for (i = 128; i > 128; i--)
//     {
//         for (left = 80; (img[i][left] == img[i][left - 1]) && (left > 1); left--)
//         {
//         }
//         for (right = 80; (img[i][right] == img[i][right + 1]) && (right < 158); right++)
//         {
//         }
//         img[i][left] = 103;
//         img[i][left + 1] = 103;
//         img[i][left / 2 + right / 2] = 100;
//         img[i][right] = 106;
//         img[i][right - 1] = 106;
//     }
//     for (; i > 1; i--)
//     {
//         if (img[i - 1][left] != color)
//         {
//             for (; img[i][left] == img[i][left + 1]; left++)
//             {
//             }
//         }
//         else
//         {
//             for (; (img[i][left] == img[i][left - 1]) && (left > 1); left--)
//             {
//             }
//         }
//         if (img[i - 1][right] != color)
//         {
//             for (; img[i][right] == img[i][right - 1]; right--)
//             {
//             }
//         }
//         else
//         {
//             for (; (img[i][right] == img[i][left + 1]) && (right < 158); right++)
//             {
//             }
//         }
//         img[i][left] = 103;
//         img[i][left + 1] = 103;
//         img[i][left / 2 + right / 2] = 100;
//         img[i][right] = 106;
//         img[i][right - 1] = 106;
//     }
// }