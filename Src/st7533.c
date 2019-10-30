#include "st7533.h"
#include "font.h"
#include "spi.h"


void spi_8b_init(void)
{
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	HAL_SPI_Init(&hspi1);
}
void spi_16b_init(void)
{
	hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
	HAL_SPI_Init(&hspi1);
}	

void lcd7735_senddata(unsigned char data) 
{
	spi_8b_init();
  HAL_SPI_Transmit(&hspi1, &data,1,0x1);
}

void lcd7735_send16bData(uint8_t msb,uint8_t lsb) 
{
	spi_16b_init();
	uint8_t masData[]={lsb,msb};
	HAL_SPI_Transmit(&hspi1,masData,1,0x1);
}


//void Back_Light(uint8_t t)
//{
//	if(t==1)
//	{
//		LCD_BL1;
//	}
//	else if(t==0)
//	{
//		LCD_BL0;
//	}
//}

/////////////////////////////////////////////////////////////////////////////////////////////////////
//----------------------------------------初始化 -----------------------------------//
/////////////////////////////////////////////////////////////////////////////////////////////////////


void lcd7735_ini(void)
{
//	Back_Light(1);
	
//	LCD_CS0;            // CS=0   
	LCD_RST0;           // RST=0 

	HAL_Delay(10);

	LCD_RST1;           // RST=1
	HAL_Delay(10);      // 


	lcd7735_sendCmd(0x11); // 

	HAL_Delay(120);      // 

	lcd7735_sendCmd(0x3A); //    режим цвета:
	lcd7735_sendData(0x05); //     16  бит
	lcd7735_sendCmd(0x36);// направление вывода изображения:
	lcd7735_sendData(0x14); //0x1C-снизу вверх, справа на лево, порядок цветов RGB;0x14-снизу вверх, справа на лево, порядок цветов BGR
	lcd7735_sendCmd(0x29);//Display on
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
//----------------------------------------发送命令----------------------------------------//
/////////////////////////////////////////////////////////////////////////////////////////////////////

void lcd7735_sendCmd(unsigned char cmd) 
{
	LCD_DC0;//dc至0
	lcd7735_senddata(cmd);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
//----------------------------------------发送数据------------------------------------------//
/////////////////////////////////////////////////////////////////////////////////////////////////////
void lcd7735_sendData(unsigned char data) 
{
	LCD_DC1;//我们把DC输出为单位T.K.传送数据
	lcd7735_senddata(data);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
//----------------------------填充指定屏幕区域----------------------------//
/////////////////////////////////////////////////////////////////////////////////////////////////////
void lcd7735_at(unsigned char startX, unsigned char startY, unsigned char stopX, unsigned char stopY) 
{
	lcd7735_sendCmd(0x2A);//列地址指令 
	LCD_DC1;
	lcd7735_senddata(0x00);
	lcd7735_senddata(startX);
	lcd7735_senddata(0x00);
	lcd7735_senddata(stopX);

	lcd7735_sendCmd(0x2B);//序列地址指令
	LCD_DC1;
	lcd7735_senddata(0x00);
	lcd7735_senddata(startY);
	lcd7735_senddata(0x00);
	lcd7735_senddata(stopY);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
//---------- 点亮所选直接空区的程序---------------------//
/////////////////////////////////////////////////////////////////////////////////////////////////////
void lcd7735_fillrect(unsigned char startX, unsigned char startY, unsigned char stopX, unsigned char stopY, unsigned int color) 
{
	unsigned char y;
	unsigned char x;
	unsigned char msb = ((unsigned char)((color & 0xFF00) >> 8));//高8位
	unsigned char lsb = ((unsigned char)(color & 0x00FF));	//低8位
//	LCD_CS0;//cs至0
	lcd7735_at(startX, startY, stopX, stopY);
	lcd7735_sendCmd(0x2C);//内存写入

	LCD_DC1;
	for (y = startY; y < stopY + 1; y++)
		for (x = startX; x < stopX + 1; x++) {
			lcd7735_send16bData(msb, lsb);//将16位分成高8位和低8位
		//  lcd7735_sendData(msb);//8位编码
		//  lcd7735_sendData(lsb);//8位编码
		}

//	LCD_CS1;

}

/////////////////////////////////////////////////////////////////////////////////////////////////////
//---------------------------------------  点亮像素点   -------------------------------------------//
/////////////////////////////////////////////////////////////////////////////////////////////////////
void lcd7735_putpix(unsigned char x, unsigned char y, unsigned int Color) 
{

//	LCD_CS0;
	lcd7735_at(x, y, x, y);
	lcd7735_sendCmd(0x2C);//存储记录
	LCD_DC1;
	lcd7735_senddata((unsigned char)((Color & 0xFF00) >> 8));//根据预先协议，改变字节大小
	lcd7735_senddata((unsigned char)(Color & 0x00FF));
//	LCD_CS1;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
//---------------------------------------     画线    -------------------------------------------//
/////////////////////////////////////////////////////////////////////////////////////////////////////
void lcd7735_line(unsigned char x1, unsigned char y1, unsigned char x2, unsigned char y2, unsigned int color) 
{
	signed char   dx, dy, sx, sy;
	unsigned char  x, y, mdx, mdy, l;

	if (x1 == x2) { // 
		lcd7735_fillrect(x1, y1, x1, y2, color);//线路是直线的话
		return;
	}

	if (y1 == y2) { // 
		lcd7735_fillrect(x1, y1, x2, y1, color);//线路是直线的话
		return;
	}
	//здесь расчет черчения косой линии 
	dx = x2 - x1; dy = y2 - y1;

	if (dx >= 0) { mdx = dx; sx = 1; }
	else { mdx = x1 - x2; sx = -1; }
	if (dy >= 0) { mdy = dy; sy = 1; }
	else { mdy = y1 - y2; sy = -1; }

	x = x1; y = y1;

	if (mdx >= mdy) {//如果x线的长度小于x线
		l = mdx;
		while (l > 0) {
			if (dy > 0) { y = y1 + mdy * (x - x1) / mdx; }
			else { y = y1 - mdy * (x - x1) / mdx; }
			lcd7735_putpix(x, y, color);//清除当前的颜色，并重新开始
			x = x + sx;
			l--;
		}
	}
	else {
		l = mdy;//如果线路长度小于x
		while (l > 0) {
			if (dy > 0) { x = x1 + ((mdx * (y - y1)) / mdy); }
			else { x = x1 + ((mdx * (y1 - y)) / mdy); }
			lcd7735_putpix(x, y, color);//清除当前的颜色，并重新开始
			y = y + sy;
			l--;
		}
	}
	lcd7735_putpix(x2, y2, color);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////
//----------------------------              绘制矩形（未填充）          ---------------------------//
/////////////////////////////////////////////////////////////////////////////////////////////////////
void lcd7735_rect(char x1, char y1, char x2, char y2, unsigned int color) 
{
	lcd7735_fillrect(x1, y1, x2, y1, color);
	lcd7735_fillrect(x1, y2, x2, y2, color);
	lcd7735_fillrect(x1, y1, x1, y2, color);
	lcd7735_fillrect(x2, y1, x2, y2, color);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
//------------------------------//字符//--------------------------//
/////////////////////////////////////////////////////////////////////////////////////////////////////
void lcd7735_putchar(unsigned char x, unsigned char y, unsigned char chr, unsigned int charColor, unsigned int bkgColor)
{
	unsigned char i;
	unsigned char j;

//	LCD_CS0;

	lcd7735_at(x, y, x + 12, y + 8);//规定显示区域大小
	lcd7735_sendCmd(0x2C);//存储记录

	LCD_DC1;//准备发送数据
	unsigned char k;
	for (i = 0; i < 7; i++)
		for (k = 2; k > 0; k--) {//K=1上半个字符，2下
			unsigned char chl = NewBFontLAT[((chr - 0x20) * 14 + i + 7 * (k - 1))];
			chl = chl << 2 * (k - 1); // 下半个字符移到1个位置左（从下面删除一条线）
			unsigned char h;
			if (k == 2) h = 6; else h = 7; // у нижней половины выведем только 6 точек вместо 7只有六点而不是七点。
			for (j = 0; j < h; j++) {
				unsigned int color;
				//HAL_Delay(100);
				if (chl & 0x80) color = charColor; else color = bkgColor;//если MSBit==0 тогда оставляем цвет бэграунда,если нет тогда цвет //символа如果MS比特==0，然后我们离开的颜色
				chl = chl << 1;//将处理过的位移到左
				unsigned char msb = ((unsigned char)((color & 0xFF00) >> 8));
				unsigned char lsb = ((unsigned char)(color & 0x00FF));	

				lcd7735_send16bData(msb, lsb);
				// lcd7735_sendData(msb);
				//lcd7735_sendData(lsb);


			}
		}
	// 从块宽度符号的垂直线向右绘制

	unsigned char msb = ((unsigned char)((bkgColor & 0xFF00) >> 8));//高8位
	unsigned char lsb = ((unsigned char)(bkgColor & 0x00FF));	//低8位
	for (j = 0; j < 13; j++) {

		lcd7735_send16bData(msb, lsb);//发16位数据
	  //	lcd7735_sendData(msb);
	  //	lcd7735_sendData(lsb);

	}
//	LCD_CS1;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
//-------------------------------------//行输出字符串	//-----------------------------------------//
/////////////////////////////////////////////////////////////////////////////////////////////////////
void lcd7735_putstr(unsigned char x, unsigned char y, const unsigned char str[], unsigned int charColor, unsigned int bkgColor) 
{

	while (*str != 0) {
		lcd7735_putchar(x, y, *str, charColor, bkgColor);
		y = y + 8;//Y轴间距
		str++;
	}
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
//-------------------------------------//输出32*32中文	//-----------------------------------------//
/////////////////////////////////////////////////////////////////////////////////////////////////////
//void lcd7735_putchar(unsigned char x, unsigned char y, unsigned char chr, unsigned int charColor, unsigned int bkgColor)
void showhanzi32(unsigned int x,unsigned int y,unsigned char index, unsigned int charColor,unsigned int bkgColor)	
{  
	
	unsigned char i,j,k;
	const unsigned char *temp=char8x16;    
	temp+=index*128;	// 128——》32*4*8 --》根据生成的十六进制数的多少来取
		
	for(j=0;j<32;j++)//32行
	{
		for(k=0;k<4;k++)// 4个4个取--------------一列被分为4个16进制 k大小与取模大小有关
		{
			for(i=0;i<8;i++)//取模软件影响，每个16进制为8个二进制------------一个16进制为8位
			{ 		     
			 	if((*temp&(1<<i))!=0)//取二进制  为1 是字体 为0 是背景  (1<<i)) i每回左移一位与temp相与 如果temp第i为为1 输出为1 否则为0
				{
					if(i==0)
						lcd7735_putpix(x+(7-i),y+j,charColor);//通用任意字体
					else
						lcd7735_putpix(x+7*k+8-i,y+j,charColor);//通用任意字体
				} 
				else
				{
					if(i==0)
						lcd7735_putpix(x+(7-i),y+j,bkgColor);
					else
						lcd7735_putpix(x+7*k+8-i,y+j,bkgColor);
				}   
			}
			temp++;
		}
	 }
}
void showimage(uint16_t x,uint16_t y) //显示40*40图片
{  
	uint16_t i,j,k,da;
	k=0;
	for(i=0;i<160;i++)//y
	{	
		for(j=0;j<128;j++)//x
		{
			da=qqimage[k*2+1];//取颜色
			da<<=8;//取颜色
			da|=qqimage[k*2]; //取颜色
			lcd7735_putpix(x+j,y+i,da);;					
			k++;  			
		}
	}
}

void lcd_showchar(int x, int y,char dat,unsigned short col,uint16_t color)
{
    unsigned char i,j;
    unsigned char temp;
    for(i=0; i<16; i++)
    {
        temp = tft_ascii[dat-32][i];//减32因为是取模是从空格开始取得 空格在ascii中序号是32
        for(j=0; j<8; j++)
        {
            if(temp&0x01)	lcd7735_sendData(col);
            else			
						{
							unsigned char msb = ((unsigned char)((color & 0xFF00) >> 8));
							unsigned char lsb = ((unsigned char)(color & 0x00FF));	

							lcd7735_send16bData(msb, lsb);
						}
            temp>>=1;
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      液晶坐标设置
//  @param      x     	        坐标x方向的起点
//  @param      y     	        坐标y方向的起点
//  @param      dat       	    需要显示的字符串
//  @return     void
//  @since      v1.0
//  Sample usage:               lcd_showchar(0,0,"seekfree");
//-------------------------------------------------------------------------------------------------------------------
void lcd_showchars(int x, int y,char dat[],unsigned short col,uint16_t color)
{
    unsigned int j;
    j = 0;
    while(dat[j] != '\0')
    {
        lcd_showchar(x+8*j,y*16,dat[j],col,color);
        j++;
    }
}




void show(unsigned int x,unsigned int y,unsigned char index, unsigned int high,unsigned int wide,unsigned int charColor,unsigned int bkgColor,unsigned int font)
{
	unsigned char i,j,k;
	const unsigned char *temp;
	
	switch(font)
	{
		case 0: {temp=char8x16; break;}
	}
	temp+=index*high*wide/8;	// 128——》32*4*8 --》根据生成的十六进制数的多少来取
		
	for(j=0;j<high;j++)//32行
	{
		for(k=0;k<wide/8;k++)// 4个4个取--------------一列被分为4个16进制 k大小与取模大小有关
		{
			for(i=0;i<8;i++)//取模软件影响，每个16进制为8个二进制------------一个16进制为8位
			{ 		     
			 	if((*temp&(1<<i))!=0)//取二进制  为1 是字体 为0 是背景  (1<<i)) i每回左移一位与temp相与 如果temp第i为为1 输出为1 否则为0
				{
					if(i==0)
						lcd7735_putpix(x+(7-i),y+j,charColor);//通用任意字体
					else
						lcd7735_putpix(x+7*k+7-i,y+j,charColor);//通用任意字体
				} 
				else
				{
					if(i==0)
						lcd7735_putpix(x+(7-i),y+j,bkgColor);
					else
						lcd7735_putpix(x+7*k+7-i,y+j,bkgColor);
				}   
			}
			temp++;
		}
	 }
	
}
		


