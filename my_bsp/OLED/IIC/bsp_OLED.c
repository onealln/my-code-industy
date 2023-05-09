#include "bsp_OLED.h"
#include "zhiku.h"
#include "i2c.h"

uint8_t OlED_GRAM[8][128];

// static void I2c_WriteByte(uint8_t addr, uint8_t data)
//{
//     /*启动IIC*/
//     I2C_GenerateSTART(I2C1, ENABLE);
//     while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
//         ;
//     /*发送设备地址*/
//     I2C_Send7bitAddress(I2C1, OLED_ADDERSS, I2C_Direction_Transmitter);
//     while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
//         ;
//     /*发送数据地址*/
//     I2C_SendData(I2C1, addr);
//     while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING))
//         ;
//     /*发送数据*/
//     I2C_SendData(I2C1, data);
//     while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING))
//         ;
//     /*停止IIC*/
//     I2C_GenerateSTOP(I2C1, ENABLE);
// }

void I2C_EE_ByteWrite(uint8_t data, uint8_t addr)
{
    HAL_I2C_Mem_Write(&hi2c2, OLED_ADDERSS, addr, I2C_MEMADD_SIZE_8BIT, &data, 1,0x0FFF);
}

/*写指令*/
void WriteCmd(uint8_t IIC_Command)
{
    I2C_EE_ByteWrite(IIC_Command, 0x00);
}
/*写数据*/
void WriteData(uint8_t IIC_Data)
{
    I2C_EE_ByteWrite(IIC_Data, 0x40);
}

void WriteMoreData(uint8_t *dat, uint16_t length)
{
    //    I2C_EE_PageWrite(dat, 0x40, length);
    // HAL_I2C_Mem_Write_DMA(&hi2c2,OLED_ADDERSS,0x40,I2C_MEMADD_SIZE_8BIT,dat,length);
    HAL_I2C_Mem_Write(&hi2c2, OLED_ADDERSS, 0x40, I2C_MEMADD_SIZE_8BIT, dat, length, 0x0FFF);
}

// 更新显存到LCD
void OLED_Refresh_Gram(void)
{
    uint8_t i, n;

    for (i = 0; i < 8; i++)
    {
        WriteCmd(0xb0 + i);
        WriteCmd(0x00);
        WriteCmd(0x10);
        WriteMoreData(OlED_GRAM[i], 128);
        //    for (n = 0; n < 128; n++)
        //    {
        //        WriteData(OlED_GRAM[i][n]);
        //    }
    }
}

/*OLED初始化*/
void OLED_Init(void)
{
    uint32_t i = 100;

    WriteCmd(0xAE); // 关闭显示
    while (--i)
    {
        int i = 100000;
        for (; i > 0; i--)
            ;
    };
    WriteCmd(0xD5); // 设置时钟分频因子,震荡频率
    WriteCmd(80);   //[3:0],分频因子;[7:4],震荡频率
    WriteCmd(0xA8); // 设置驱动路数
    WriteCmd(0X3F); // 默认0X3F(1/64)
    WriteCmd(0xD3); // 设置显示偏移
    WriteCmd(0X00); // 默认为0

    WriteCmd(0x40); // 设置显示开始行 [5:0],行数.

    WriteCmd(0x8D); // 电荷泵设置
    WriteCmd(0x14); // bit2，开启/关闭
    WriteCmd(0x20); // 设置内存地址模式
    WriteCmd(0x02); //[1:0],00，列地址模式;01，行地址模式;10,页地址模式;默认10;
    WriteCmd(0xA1); // 段重定义设置,bit0:0,0->0;1,0->127;
    WriteCmd(0xC0); // 设置COM扫描方向;bit3:0,普通模式;1,重定义模式 COM[N-1]->COM0;N:驱动路数
    WriteCmd(0xDA); // 设置COM硬件引脚配置
    WriteCmd(0x12); //[5:4]配置

    WriteCmd(0x81); // 对比度设置
    WriteCmd(0xEF); // 1~255;默认0X7F (亮度设置,越大越亮)
    WriteCmd(0xD9); // 设置预充电周期
    WriteCmd(0xf1); //[3:0],PHASE 1;[7:4],PHASE 2;
    WriteCmd(0xDB); // 设置VCOMH 电压倍率
    WriteCmd(0x30); //[6:4] 000,0.65*vcc;001,0.77*vcc;011,0.83*vcc;

    WriteCmd(0xA4); // 全局显示开启;bit0:1,开启;0,关闭;(白屏/黑屏)
    WriteCmd(0xA6); // 设置显示方式;bit0:1,反相显示;0,正常显示
    WriteCmd(0xAF); // 开启显示
    OLED_ClearAll();
}

/*设置光标起始坐标*/
void OLED_SetPos(uint8_t x, uint8_t y)
{
    WriteCmd(0xB0 + y);
    WriteCmd((x & 0xF0) >> 4 | 0x10);
    WriteCmd((x & 0x0F) | 0x01);
}
/*填充整个屏幕*/
void OLED_Fill(uint8_t Fill_Data)
{
    uint8_t m, n;

    for (m = 0; m < 8; m++)
    {
        WriteCmd(0xB0 + m);
        WriteCmd(0x00);
        WriteCmd(0x10);

        for (n = 0; n < 128; n++)
        {
            OlED_GRAM[m][n] = Fill_Data;
        }
    }
}
/*清屏*/
void OLED_ClearAll(void)
{
    OLED_Fill(0x00);
    OLED_Refresh_Gram();
}
/*将OLED从睡眠中唤醒*/
void OLED_ON(void)
{
    WriteCmd(0xAF);
    WriteCmd(0x8D);
    WriteCmd(0x14);
}
/*将OLED休眠*/
void OLED_OFF(void)
{
    WriteCmd(0xAE);
    WriteCmd(0x8D);
    WriteCmd(0x10);
}

/*画点函数
x:0-127
y:0-63
size:字体16/12
t:1 填充 0,清空
*/
void OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t t)
{
    uint8_t pos, bx, temp = 0;

    if (x > 127 || y > 63)
        return;
    pos = 7 - y / 8;
    bx = y % 8;
    temp = 1 << (7 - bx);
    if (t)
        OlED_GRAM[pos][x] |= temp;
    else
        OlED_GRAM[pos][x] &= ~temp;
}
/*写一个字符
x:0-127
y:0-63
size:字体16/12
mode:0,反白显示；1，正常显示
*/
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t size, uint8_t mode)
{
    uint8_t temp, t, t1;
    uint8_t y0 = y;
    uint8_t csize = (size / 8 + ((size % 8) ? 1 : 0)) * (size / 2); // 得到字体一个字符对应点阵集所占的字节数
    chr = chr - ' ';                                                // 得到偏移后的值
    for (t = 0; t < csize; t++)
    {
        if (size == 12)
            temp = F6X8[chr][t]; // 调用1206字体
        else if (size == 16)
            temp = F8X16[chr][t]; // 调用1608字体
        // else if(size==24)temp=asc2_2412[chr][t];	//调用2412字体
        else
            return; // 没有的字库
        for (t1 = 0; t1 < 8; t1++)
        {
            if (temp & 0x80)
                OLED_DrawPoint(x, y, mode);
            else
                OLED_DrawPoint(x, y, !mode);
            temp <<= 1;
            y++;
            if ((y - y0) == size)
            {
                y = y0;
                x++;
                break;
            }
        }
    }
}

/*显示字符串
x:0-127
y:0-63
s:字符数组
size:字符大小16/12/24
*/
void OLED_ShowString(uint8_t x, uint8_t y, uint8_t *s, uint8_t size)
{
    while ((*s <= '~') && (*s >= ' '))
    {
        if (x > (128 - (size / 2)))
        {
            x = 0;
            y += size;
        }
        if (y > (64 - size))
        {
            y = x = 0;
            OLED_ClearAll();
        }
        OLED_ShowChar(x, y, *s, size, 1);
        x += size / 2;
        s++;
    }
}

/*m^n函数*/
uint16_t OLED_Pow(uint8_t m, uint8_t n)
{
    uint16_t result = 1;
    while (n--)
    {
        result *= m;
    }
    return result;
}

/******************************
 显示数字
 length：数字的位数
 size ： 数字的大小12/16/24
*******************************/
void OLED_ShowNum(uint8_t x, uint8_t y, uint16_t num, uint8_t length, uint8_t size)
{
    uint8_t t, temp;
    uint8_t enshow = 0;

    for (t = 0; t < length; t++)
    {
        temp = (num / OLED_Pow(10, length - t - 1)) % 10;
        if (enshow == 0 && t < (length - 1))
        {
            if (temp == 0)
            {
                OLED_ShowChar(x + (size / 2) * t, y, ' ', size, 1);
                continue;
            }
            else
                enshow = 1;
        }
        OLED_ShowChar(x + (size / 2) * t, y, temp + '0', size, 1);
    }
}

/*********************************************
 * @功能说明: 画直线
 * @形    参 {uint16_t} x1:起点横坐标
 * @形    参 {uint16_t} y1:起点纵坐标
 * @形    参 {uint16_t} x2:终点纵坐标
 * @形    参 {uint16_t} y2:终点纵坐标
 * @返 回 值 {*}
 *********************************************/
void OLED_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    uint16_t t;
    int xerr = 0, yerr = 0, delta_x, delta_y, distance;
    int incx, incy, uRow, uCol;
    delta_x = x2 - x1; // 计算坐标增量
    delta_y = y2 - y1;
    uRow = x1;
    uCol = y1;
    if (delta_x > 0)
        incx = 1; // 设置单步方向
    else if (delta_x == 0)
        incx = 0; // 垂直线
    else
    {
        incx = -1;
        delta_x = -delta_x;
    }
    if (delta_y > 0)
        incy = 1;
    else if (delta_y == 0)
        incy = 0; // 水平线
    else
    {
        incy = -1;
        delta_y = -delta_y;
    }
    if (delta_x > delta_y)
        distance = delta_x; // 选取基本增量坐标轴
    else
        distance = delta_y;
    for (t = 0; t <= distance + 1; t++) // 画线输出
    {
        OLED_DrawPoint(uRow, uCol, 1); // 画点
        xerr += delta_x;
        yerr += delta_y;
        if (xerr > distance)
        {
            xerr -= distance;
            uRow += incx;
        }
        if (yerr > distance)
        {
            yerr -= distance;
            uCol += incy;
        }
    }
}

#include <math.h>
/*********************************************
 * @功能说明: 任意角度画直线
 * @形    参 {uint32_t} x:起始点x坐标
 * @形    参 {uint32_t} y:起始点y坐标
 * @形    参 {float} du: 度数
 * @形    参 {uint32_t} len: 长度
 * @形    参 {uint8_t} c: 颜色 0、1
 * @返 回 值 {*}
 *********************************************/
void OLED_DrawAngleLine(uint32_t x, uint32_t y, float du, uint32_t len, uint8_t c)
{
    int i;
    int x0, y0;
    double k = du * (3.1415926535L / 180);

    for (i = 0; i < len; i++)
    {
        x0 = cos(k) * i;
        y0 = sin(k) * i;
        OLED_DrawPoint(x + x0, y + y0, c);
    }
}

/*********************************************
 * @功能说明: 任意角度画直线
 * @形    参 {uint32_t} x:起始点x坐标
 * @形    参 {uint32_t} y:起始点y坐标
 * @形    参 {float} du:  度数
 * @形    参 {uint32_t} len:线段半径
 * @形    参 {uint32_t} w: 线段长度
 * @形    参 {uint8_t} c: 颜色 0、1
 * @返 回 值 {*}
 *********************************************/
void OLED_DrawAngleLine2(uint32_t x, uint32_t y, float du, uint32_t len, uint32_t w, uint8_t c)
{
    int i;
    int x0, y0;
    double k = du * (3.1415926535 / 180);
    for (i = len - w; i < len; i++)
    {
        x0 = cos(k) * i;
        y0 = sin(k) * i;
        OLED_DrawPoint(x + x0, y + y0, c);
    }
}

/*********************************************
 * @功能说明:  画矩形
 * @形    参 {uint16_t} x1: 对角坐标
 * @形    参 {uint16_t} y1:
 * @形    参 {uint16_t} x2: 对角坐标
 * @形    参 {uint16_t} y2:
 * @返 回 值 {*}
 *********************************************/
void OLED_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    OLED_DrawLine(x1, y1, x2, y1);
    OLED_DrawLine(x1, y1, x1, y2);
    OLED_DrawLine(x1, y2, x2, y2);
    OLED_DrawLine(x2, y1, x2, y2);
}

/*********************************************
 * @功能说明:   在指定位置画圆
 * @形    参 {uint16_t} x0:  圆心坐标
 * @形    参 {uint16_t} y0:
 * @形    参 {uint8_t} r:    半径
 * @返 回 值 {*}
 *********************************************/
void OLED_Circle(uint16_t x0, uint16_t y0, uint8_t r)
{
    int a, b;
    int di;
    a = 0;
    b = r;
    di = 3 - (r << 1); // 判断下个点位置的标志
    while (a <= b)
    {
        OLED_DrawPoint(x0 + a, y0 - b, 1); // 5
        OLED_DrawPoint(x0 + b, y0 - a, 1); // 0
        OLED_DrawPoint(x0 + b, y0 + a, 1); // 4
        OLED_DrawPoint(x0 + a, y0 + b, 1); // 6
        OLED_DrawPoint(x0 - a, y0 + b, 1); // 1
        OLED_DrawPoint(x0 - b, y0 + a, 1);
        OLED_DrawPoint(x0 - a, y0 - b, 1); // 2
        OLED_DrawPoint(x0 - b, y0 - a, 1); // 7
        a++;
        if (di < 0)
            di += 4 * a + 6; // Bresenham画圆算法
        else
        {
            di += 10 + 4 * (a - b);
            b--;
        }
    }
}

/********************************************* 
 * @功能说明: 显示汉字
 * @形    参 {uint8_t} x:坐标
 * @形    参 {uint8_t} y:
 * @形    参 {uint8_t} size:大小 16\24
 * @形    参 {uint8_t} number: 在数组下标
 * @返 回 值 {*}
 *********************************************/
void OLED_ShowChineseFont(uint8_t x, uint8_t y, uint8_t size, uint8_t number)
{
    uint8_t i, j, x0 = x;
    uint8_t tmp;
    for (i = 0; i < size * size / 8; i++)
    {
        if (size == 16)
            tmp = ChineseFont_16_16[number][i];
        // else if(size==24)tmp=ChineseFont_24_24[number][i];
        for (j = 0; j < 8; j++)
        {
            if (tmp & 0x80)
                OLED_DrawPoint(x, y, 1);
            else
                OLED_DrawPoint(x, y, 0);
            x++;
            tmp <<= 1; // 继续判断下一位
        }
        if (x - x0 == size)
        {
            y++;    // 纵坐标自增
            x = x0; // 横坐标归位
        }
    }
}

/*待完善*/
void OLED_ShowCHinese(uint8_t x, uint8_t y, uint8_t no)
{
    uint8_t t, addr = 0;
    OLED_SetPos(x, y);
    for (t = 0; t < 16; t++)
    {
        // WriteData();
        addr += 1;
    }
    OLED_SetPos(x, y + 1);
    for (t = 0; t < 16; t++)
    {
        // WriteData();
        addr += 1;
    }
}
