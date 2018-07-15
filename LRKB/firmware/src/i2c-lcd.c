/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    filename.c

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */
#include <stdint.h>
#include "system_config/default/framework/driver/i2c/drv_i2c_static_buffer_model.h"
/* This section lists the other files that are included in this file.
 */

/* TODO:  Include other files here if needed. */
#include "app.h"
extern APP_DATA appData;
uintptr_t LCD_OpStatus;
/* ************************************************************************** */
/* ************************************************************************** */
// Section: Interface Functions                                               */
/* ************************************************************************** */
/* ************************************************************************** */

/*  A brief description of a section can be given directly below the section
    banner.
 */

void DelayUs(uint32_t microseconds)
{
    uint32_t time;
    
    time = _CP0_GET_COUNT(); // Read Core Timer    
    time += (SYS_CLK_FREQ / 2 / 1000000) * microseconds; // calc the Stop Time    
    while ((int32_t)(time - _CP0_GET_COUNT() ) > 0){};    
}

void LCD_EventHandler(DRV_I2C_BUFFER_EVENT event, DRV_I2C_BUFFER_HANDLE bufferHandle, uintptr_t context)
{

    switch (event)
    {
        case DRV_I2C_BUFFER_EVENT_COMPLETE:
            break;
        case DRV_I2C_BUFFER_EVENT_ERROR:
            break;
        default:
            break;
    }
    return;
}
 
void LCD_Initialize(){
    uint8_t buf[2];
    appData.I2c0Handle = DRV_I2C_Open(DRV_I2C_INDEX_0, DRV_IO_INTENT_BLOCKING|DRV_IO_INTENT_WRITE);

    if (appData.I2c0Handle == DRV_HANDLE_INVALID) 
        return;
    
    buf[0] = LCD_I2C_TAIL | LCD_I2C_INST | LCD_I2C_WRITE;
    buf[1] = LCD_CMD_FUNC | LCD_IF_8BIT | LCD_2LINE | LCD_NORMAL_INST;
    appData.I2CWriteBufferHandle = DRV_I2C_Transmit(appData.I2c0Handle,LCD_I2C_ADDR,buf,sizeof(buf),NULL);
    DelayUs(LCD_CMD_WAIT);

    //Go to Extended Instruction sets.
    buf[1] = LCD_CMD_FUNC | LCD_IF_8BIT | LCD_2LINE | LCD_EXT_INST;
    appData.I2CWriteBufferHandle = DRV_I2C_Transmit(appData.I2c0Handle,LCD_I2C_ADDR,buf,sizeof(buf),NULL);
    DelayUs(LCD_CMD_WAIT);

    buf[1] = LCD_CMD_FREQ | LCD_CLK_347K;
    appData.I2CWriteBufferHandle = DRV_I2C_Transmit(appData.I2c0Handle,LCD_I2C_ADDR,buf,sizeof(buf),NULL);
    DelayUs(LCD_CMD_WAIT);

    buf[1] = LCD_CMD_CONTRAST | LCD_CONTRAST;
    appData.I2CWriteBufferHandle = DRV_I2C_Transmit(appData.I2c0Handle,LCD_I2C_ADDR,buf,sizeof(buf),NULL);
    DelayUs(LCD_CMD_WAIT);

    buf[1] = LCD_CMD_ICON | LCD_BOOST | LCD_CONT_H;
    appData.I2CWriteBufferHandle = DRV_I2C_Transmit(appData.I2c0Handle,LCD_I2C_ADDR,buf,sizeof(buf),NULL);
    DelayUs(LCD_CMD_WAIT);

    buf[1] = LCD_CMD_FOLLOWER;
    appData.I2CWriteBufferHandle = DRV_I2C_Transmit(appData.I2c0Handle,LCD_I2C_ADDR,buf,sizeof(buf),NULL);
    DelayUs(200 * 1000);
    
    //Revert to normal instruction sets
    buf[1] = LCD_CMD_FUNC | LCD_IF_8BIT | LCD_2LINE | LCD_NORMAL_INST;
    appData.I2CWriteBufferHandle = DRV_I2C_Transmit(appData.I2c0Handle,LCD_I2C_ADDR,buf,sizeof(buf),NULL);
    DelayUs(LCD_CMD_WAIT);

    buf[1] = LCD_CMD_ONOFF | LCD_DISPLAY_ON;
    appData.I2CWriteBufferHandle = DRV_I2C_Transmit(appData.I2c0Handle,LCD_I2C_ADDR,buf,sizeof(buf),NULL);
    DelayUs(LCD_CMD_WAIT);

    buf[1] = LCD_CMD_CLEAR;
    appData.I2CWriteBufferHandle = DRV_I2C_Transmit(appData.I2c0Handle,LCD_I2C_ADDR,buf,sizeof(buf),NULL);
    DelayUs(LCD_CLR_WAIT);

    DRV_I2C_BufferEventHandlerSet(appData.I2c0Handle, LCD_EventHandler, LCD_OpStatus);

}

// *****************************************************************************

/** 
  @Function
    int ExampleInterfaceFunctionName ( int param1, int param2 ) 

  @Summary
    Brief one-line description of the function.

  @Remarks
    Refer to the example_file.h interface header for function usage details.
 */
void LCD_Clear(){
    LCD_ClearNW();
    DelayUs(LCD_CLR_WAIT);
}
void LCD_ClearNW(){
    uint8_t buf[2];
    buf[0] = LCD_I2C_TAIL | LCD_I2C_INST | LCD_I2C_WRITE;
    buf[1] = LCD_CMD_CLEAR;
    appData.I2CWriteBufferHandle = DRV_I2C_Transmit(appData.I2c0Handle,LCD_I2C_ADDR,buf,sizeof(buf),NULL);
    DelayUs(LCD_CLR_WAIT);
}

//line 0:upper 1:lower
void LCD_Locate(uint8_t column,uint8_t line){
    uint8_t buf[2];
    uint8_t ddadr = (line * LCD_DD_PER_LINE) + column;
    buf[0] = LCD_I2C_TAIL | LCD_I2C_INST | LCD_I2C_WRITE;
    buf[1] = LCD_CMD_DDADR | ddadr;
    appData.I2CWriteBufferHandle = DRV_I2C_Transmit(appData.I2c0Handle,LCD_I2C_ADDR,buf,sizeof(buf),NULL);
    DelayUs(LCD_CMD_WAIT);
}

void LCD_SetDisplay(bool on, bool cursor, bool blink){
    uint8_t buf[2];
    buf[0] = LCD_I2C_TAIL | LCD_I2C_INST | LCD_I2C_WRITE;
    buf[1] = LCD_CMD_ONOFF | (on)? LCD_DISPLAY_ON:0 | (cursor)? LCD_CURSOR_ON:0 | (blink)? LCD_BLINK_ON:0 ;
    appData.I2CWriteBufferHandle = DRV_I2C_Transmit(appData.I2c0Handle,LCD_I2C_ADDR,buf,sizeof(buf),NULL);
    DelayUs(LCD_CMD_WAIT);    
}

void LCD_Putchar(char c){
    uint8_t buf[2];
    buf[0] = LCD_I2C_TAIL | LCD_I2C_DATA | LCD_I2C_WRITE;
    buf[1] = c;
    appData.I2CWriteBufferHandle = DRV_I2C_Transmit(appData.I2c0Handle,LCD_I2C_ADDR,buf,sizeof(buf),NULL);
    DelayUs(LCD_DAT_WAIT);
}

void LCD_Print(char *str){
    size_t len = strlen(str); 
    if(len > LCD_WIDTH) len = LCD_WIDTH;
#if 0
    uint8_t i;
    for (i = 0; i < len; i++){
        LCD_Putchar(*(str+i));
    }
#else
    uint8_t i2cbuf[LCD_WIDTH*2],*p = i2cbuf;
    uint8_t i;
    uint8_t cmd= LCD_I2C_CONTINUE | LCD_I2C_DATA | LCD_I2C_WRITE;    // =0xC0;
    for(i = 0;i < (len-1) ; i++){
        *p++ = cmd;
        *p++ = str[i];
    }
    *p++ = LCD_I2C_TAIL | LCD_I2C_DATA | LCD_I2C_WRITE;    // =0x40;
    *p++ = str[len-1];
    
    appData.I2CWriteBufferHandle = DRV_I2C_Transmit(appData.I2c0Handle,LCD_I2C_ADDR,i2cbuf,(len*2),NULL);
    DelayUs(LCD_DAT_WAIT * len);
#endif
}

void LCD_SetBL(bool light) {
    PLIB_PORTS_PinWrite(PORTS_ID_0,PORT_CHANNEL_A,PORTS_BIT_POS_4,( (light)? 1:0) );
}

/* *****************************************************************************
 End of File
 */
