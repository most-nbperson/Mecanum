#ifndef __ST7533_H__
#define __ST7533_H__

#include "main.h"

//#define LCD_CS0 HAL_GPIO_WritePin(CS_GPIO_Port,CS_Pin,GPIO_PIN_RESET);
//#define LCD_CS1 HAL_GPIO_WritePin(CS_GPIO_Port,CS_Pin,GPIO_PIN_SET);

#define LCD_RST0 HAL_GPIO_WritePin(LCD_RST_GPIO_Port,LCD_RST_Pin,GPIO_PIN_RESET);
#define LCD_RST1 HAL_GPIO_WritePin(LCD_RST_GPIO_Port,LCD_RST_Pin,GPIO_PIN_SET);


#define LCD_DC0 HAL_GPIO_WritePin(LCD_A0_GPIO_Port,LCD_A0_Pin,GPIO_PIN_RESET);
#define LCD_DC1 HAL_GPIO_WritePin(LCD_A0_GPIO_Port,LCD_A0_Pin,GPIO_PIN_SET);

//#define LCD_BL0 HAL_GPIO_WritePin(BL_GPIO_Port,BL_Pin,GPIO_PIN_RESET);
//#define LCD_BL1 HAL_GPIO_WritePin(BL_GPIO_Port,BL_Pin,GPIO_PIN_SET);
//画笔颜色
#define WHITE         	 0xFFFF
#define BLACK         	 0x0000	  
#define BLUE         			 0x001F  
#define BRED             0XF81F
#define GRED 			 			0XFFE0
#define GBLUE			 			0X07FF
#define RED           	 0xF800
#define MAGENTA       	 0xF81F
#define GREEN         	 0x07E0
#define CYAN          	 0x7FFF
#define YELLOW        	 0xFFE0
#define BROWN 			 0XBC40 //棕色
#define BRRED 			 0XFC07 //棕红色
#define GRAY  			 0X8430 //灰色

void spi_8b_init(void);
void spi_16b_init(void);
void lcd7735_send16bData(uint8_t msb,uint8_t lsb);
void lcd7735_senddata(unsigned char data);
void Back_Light(uint8_t t);
//初始化
void lcd7735_ini(void);
//发送数据
void lcd7735_sendData(unsigned char data);
//发送命令
void lcd7735_sendCmd(unsigned char cmd); 
// 点亮指定区域
void lcd7735_fillrect(unsigned char startX, unsigned char startY, unsigned char stopX, unsigned char stopY, unsigned int color);
//点亮像素点
void lcd7735_putpix(unsigned char x, unsigned char y, unsigned int Color);
// 画线
void lcd7735_line(unsigned char x1, unsigned char y1, unsigned char x2, unsigned char y2, unsigned int color);
// 绘制矩形（未填充）  
void lcd7735_rect(char x1,char y1,char x2,char y2, unsigned int color);
//输出单个字符
void lcd7735_putchar(unsigned char x, unsigned char y, unsigned char chr, unsigned int charColor, unsigned int bkgColor);
//输出字符串	 
void lcd7735_putstr(unsigned char x, unsigned char y, const unsigned char str[], unsigned int charColor, unsigned int bkgColor);
// 计算指定屏幕区域
void lcd7735_at(unsigned char startX, unsigned char startY, unsigned char stopX, unsigned char stopY) ;
//输出32*32中文	//
void showhanzi32(unsigned int x,unsigned int y,unsigned char index, unsigned int charColor,unsigned int bkgColor);

void showimage(uint16_t x,uint16_t y);

void lcd_showchar(int x, int y,char dat,unsigned short col,uint16_t color);
void lcd_showchars(int x, int y,char dat[],unsigned short col,uint16_t color);
void show(unsigned int x,unsigned int y,unsigned char index, unsigned int high,unsigned int wide,unsigned int charColor,unsigned int bkgColor,unsigned int font);

#endif
