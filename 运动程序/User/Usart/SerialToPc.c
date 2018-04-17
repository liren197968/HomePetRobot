#include "SerialToPc.h"

int DoubleToString(double DoubleData, unsigned char *pString)
{
    unsigned char       i = 0;
    double              DecimalData;
    int                 IntegerData;

    IntegerData = (int)DoubleData;
    DecimalData = DoubleData - IntegerData;                                     //得到小数部分

    if(IntegerData >= 100)                                                      //是否可被100整除
    {
        pString[i] = 48 + IntegerData / 100;
        IntegerData = IntegerData % 100;
        i++;
    }
    else if(IntegerData < 100 && i != 0)
    {
        pString[i] = 0 + 48;
        i++;
    }

    if(IntegerData >= 10)                                                       //是否可被10整除
    {
        pString[i] = 48 + IntegerData / 10;
        IntegerData = IntegerData % 10;
        i++;
    }
    else if(IntegerData < 10 && i != 0)
    {
        pString[i] = 48;
        i++;
    }

    pString[i] = 48 + IntegerData;

    if(DecimalData >= 0.000001)                                                 //判断是否为小数
    {
        i++;

        pString[i]='.';                                                         //加小数点

        i++;

        IntegerData = (int)(DecimalData * 1000000);
        pString[i] = 48 + IntegerData / 100000;
        IntegerData = IntegerData % 100000;

        if(IntegerData > 0)
        {
            i++;

            pString[i] = 48 + IntegerData / 10000;
            IntegerData = IntegerData % 10;
        }

        if(IntegerData > 0)
        {
            i++;

            pString[i] = 48 + IntegerData / 1000;
            IntegerData = IntegerData % 10;
        }

        if(IntegerData > 0)
        {
            i++;

            pString[i] = 48 + IntegerData / 100;
            IntegerData = IntegerData % 10;
        }

        if(IntegerData > 0)
        {
            i++;

            pString[i] = 48 + IntegerData / 10;
            IntegerData = IntegerData % 10;
        }

        if(IntegerData >= 0)
        {
            i++;

            pString[i] = 48 + IntegerData;
        }
    }

    i++;

    pString[i]='\0';

    return 1;                                                                   //数据转换成功
}

void UsartData_Send(unsigned char a)
{
    while(USART_GetFlagStatus(USART1, USART_FLAG_TC) != SET);
    USART_SendData(USART1, a);
}

void StringData_Send(unsigned char *pString)
{
    unsigned char       i = 0;

    while(pString[i] != '\0')
    {
        UsartData_Send(pString[i]);

        i++;
    }
}

int SerialDoubleData_Send(double DoubleData)
{
    int                 ReturnFlag = 0;
    unsigned char       TempString[MAX_NUM_LEN];

    ReturnFlag = DoubleToString(DoubleData, TempString);

    if(ReturnFlag)
    {
        StringData_Send(TempString);

        return 1;
    }
    else
    {
        return -1;
    }
}
