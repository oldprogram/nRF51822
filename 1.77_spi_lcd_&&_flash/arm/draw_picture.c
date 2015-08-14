#include "draw_picture.h"




////////////////////////////////////////////////////////////////////////////////////
//»æÖÆÍ¼Æ¬º¯Êý
////////////////////////////////////////////////////////////////////////////////////
//void Disp(unsigned short int width, unsigned short int height, unsigned short int *p)
//{
//	unsigned short int midX, midY;
//	unsigned int  i,j; //i-row,j-col
//	unsigned int  n,k; //n-row repeat count,k-col repeat count
//	
//	midX = Max_Column/2;
//	midY = Max_Row/2;
//
//	BlockWrite(midX-width/2,midX+width/2-1,midY-height/2,midY+height/2-1);  //display in the middle,TBD
//
//	nrf_gpio_pin_clear(LCD_CS);
//  	nrf_gpio_pin_set(LCD_DC);//DC(RS) low means send cmd
//
//		for(i=0;i<height;i++)//row,y
//		{
//				for(j=0;j<width;j++)//col,x
//		    {
//				    SendDataSPI((*(p+i*height+j))>>8); 
//					  SendDataSPI(*(p+i*height+j));  
//				}
//		}
//
//	nrf_gpio_pin_set(LCD_CS);
//}
#ifdef EVAL_PIC
void DispPic(unsigned int  *picture)
{
    unsigned int *p;
    int  i, j; //i-row,j-col
    int  n, k; //n-row repeat count,k-col repeat count

    BlockWrite(0, COL - 1, 0, ROW - 1);

    CS_CLEAR;
    RS_SET;

    for(n = 0; n < ROW / PIC_HEIGHT; n++) //n-row repeat count
    {
        for(i = 0; i < PIC_HEIGHT; i++)
        {
            p = picture;
            for(k = 0; k < COL / PIC_WIDTH; k++) //k-col repeat count
            {
                for(j = 0; j < PIC_WIDTH; j++)
                {
                    WriteDispData((*(p + i * PIC_HEIGHT + j)) >> 8,*(p + i * PIC_HEIGHT + j));
                }
            }

            p = picture;
            for(j = 0; j < COL % PIC_WIDTH; j++)
            {
				WriteDispData((*(p + i * PIC_HEIGHT + j)) >> 8,*(p + i * PIC_HEIGHT + j));
            }
        }
    }

    for(i = 0; i < ROW % PIC_HEIGHT; i++)
    {
        p = picture;
        for(k = 0; k < COL / PIC_WIDTH; k++) //k-col repeat count
        {
            for(j = 0; j < PIC_WIDTH; j++)
            {
                WriteDispData((*(p + i * PIC_HEIGHT + j)) >> 8,*(p + i * PIC_HEIGHT + j));
            }

        }

        p = picture;
        for(j = 0; j < COL % PIC_WIDTH; j++)
        {
            WriteDispData((*(p + i * PIC_HEIGHT + j)) >> 8,*(p + i * PIC_HEIGHT + j));
        }
    }

    CS_SET;
}
#endif

#ifdef SD_DEMO_PIC
void DispPicFromSD(unsigned char PicNum)
{
    unsigned long Address_S, Address_E;
    unsigned int  i;
    unsigned long k;

    SD_Start();

    CLKSEL = 0x03;
    BlockWrite(0, COL - 1, 0, ROW - 1);

    CS0 = 0;
    //RD0=1;
    RS = 1;

    k = (unsigned long)ROW * (unsigned long)COL * 2 / 512;
    Address_S = PicNum * k + 1;   	   ;
    Address_E = Address_S + k;
    for(; Address_S < Address_E; Address_S++)
    {
        MMC_SD_ReadSingleBlock(Address_S);



        for(i = 0; i < 512; i += 2)
        {
            WriteDispData(buffer[i], buffer[i + 1]);
        }

    }

    CS0 = 1;
    CLKSEL = 0x00;
}
#endif
