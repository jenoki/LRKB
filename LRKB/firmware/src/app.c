/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It
    implements the logic of the application's state machine and it may call
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "app.h"


// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

//Key definition table
const KEY_DEFINE keytable[KEY_COUNT + 12] = {
//Switch definitions
    {HID_NONM,HID_0},
    {HID_NONM,HID_1},
    {HID_NONM,HID_2},
    {HID_NONM,HID_3},
    {HID_NONM,HID_x},
    {HID_NONM,HID_UP},
    {HID_NONM,HID_u},
    {HID_NONM,HID_a},
    {HID_NONM,HID_LEFT},
    {HID_NONM,HID_DOWN},
    {HID_NONM,HID_RIGHT},
    {HID_GUIM,HID_F11},
    {HID_GUIM,HID_RB},
    {HID_GUIM,HID_LB},
    {HID_ALTM,HID_1},
    {HID_NONM,HID_b},
//Rotators movements
    {HID_NONM,HID_RIGHT}, //lower CW
    {HID_NONM,HID_DOWN},  //lower push+CW
    {HID_NONM,HID_LEFT},  //lower CCW
    {HID_NONM,HID_UP},    //lower push+CCW
    {HID_GUIM|HID_SFTM,HID_EQ},   //upper CCW
    {HID_GUIM,HID_LB},           //upper push+CW
    {HID_GUIM|HID_SFTM,HID_MI},   //upper CCW
    {HID_GUIM,HID_RB},           //upper push+CCW
//etc.
    {HID_NONM,HID_NONE}, //padding for side switch
    {HID_NONM,HID_x},   // lower rotator switch
    {HID_NONM,HID_u},   // upper rotator switch
    {HID_NONM,HID_NONE}, //padding-not used
};

MOUSE_DEFINE mousetable[8]={
    {MOUSE_RPT_IDX_MOV_X,MOUSE_MOVE_RIGHT},  //lower CW
    {MOUSE_RPT_IDX_MOV_Y,MOUSE_MOVE_DOWN}, //lower push+CW
    {MOUSE_RPT_IDX_MOV_X,MOUSE_MOVE_LEFT},   //lower CCW
    {MOUSE_RPT_IDX_MOV_Y,MOUSE_MOVE_UP},   //lower push+CCW
    {MOUSE_RPT_IDX_ROTATE,WHEEL_DOWN},    //upper CW
    {MOUSE_RPT_IDX_TILT,WHEEL_RIGHT},   //upper push+CW
    {MOUSE_RPT_IDX_ROTATE,WHEEL_UP},  //upper CCW
    {MOUSE_RPT_IDX_TILT,WHEEL_LEFT},    //upper push+CCW
};

KEY_MODIFIER modifiers[KEY_COUNT];
#if 0
#define TEMP_TABLE_INDEX_TOP 490
#define TEMP_TABLE_INDEX_TAIL 745
#define TEMP_AVG_COUNT 32
uint16_t    temp_avg_buf[TEMP_AVG_COUNT];
uint8_t     temp_idx = 0;
uint16_t    temp_sum = 0;
const float temp_table[]={
//ADCval=490, 491, 492, 493, ......
        37.9,37.8,37.7,37.5,37.4,37.3,37.2,37.1,
        37.0,36.9,36.8,36.6,36.5,36.4,36.3,36.2,36.1,
        36.0,35.9,35.8,35.7,35.5,35.4,35.3,35.2,35.1,
        35.0,34.9,34.8,34.7,34.6,34.4,34.3,34.2,34.1,
        34.0,33.9,33.8,33.7,33.6,33.5,33.4,33.2,33.1,
        33.0,32.9,32.8,32.7,32.6,32.5,32.4,32.3,32.2,
        32.1,31.9,31.8,31.7,31.6,31.5,31.4,31.3,31.2,31.1,
        31.0,30.9,30.8,30.7,30.5,30.4,30.3,30.2,30.1,
        30.0,29.9,29.8,29.7,29.6,29.5,29.4,29.3,29.2,
        29.0,28.9,28.8,28.7,28.6,28.5,28.4,28.3,28.2,28.1,
        28.0,27.9,27.8,27.7,27.6,27.4,27.3,27.2,27.1,
        27.0,26.9,26.8,26.7,26.6,26.5,26.4,26.3,26.2,26.1,
        26.0,25.8,25.7,25.6,25.5,25.4,25.3,25.2,25.1,
        25.0,24.9,24.8,24.7,24.6,24.5,24.4,24.2,24.1,
        24.0,23.9,23.8,23.7,23.6,23.5,23.4,23.3,23.2,23.1,
        23.0,22.9,22.8,22.6,22.5,22.4,22.3,22.2,22.1,
        22.0,21.9,21.8,21.7,21.6,21.5,21.4,21.3,21.1,
        21.0,20.9,20.8,20.7,20.6,20.5,20.4,20.3,20.2,20.1,
        20.0,19.9,19.7,19.6,19.5,19.4,19.3,19.2,19.1,
        19.0,18.9,18.8,18.7,18.6,18.4,18.3,18.2,18.1,
        18.0,17.9,17.8,17.7,17.6,17.5,17.4,17.2,17.1,
        17.0,16.9,16.8,16.7,16.6,16.5,16.4,16.3,16.1,
        16.0,15.9,15.8,15.7,15.6,15.5,15.4,15.3,15.1,
        15.0,14.9,14.8,14.7,14.6,14.5,14.4,14.3,14.1,
        14.0,13.9,13.8,13.7,13.6,13.5,13.4,13.2,13.1,
        13.0,12.9,12.8,12.7,12.6,12.4,12.3,12.2,12.1,
        12.0,11.9,11.8,11.6,11.5,11.4,11.3,11.2,11.1,
        11.0,10.8,10.7,10.6,10.5,10.4,10.3,10.1,10.0
//ADCval= ......                               , 745
};
#endif

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.

    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;
#define APP_MAKE_BUFFER_DMA_READY
/*Keyboard Report to be transmitted*/
KEYBOARD_INPUT_REPORT APP_MAKE_BUFFER_DMA_READY keyboardInputReport;
/* Keyboard output report */
KEYBOARD_OUTPUT_REPORT APP_MAKE_BUFFER_DMA_READY keyboardOutputReport;

/* Mouse Report */
MOUSE_REPORT mouseReport APP_MAKE_BUFFER_DMA_READY;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

USB_DEVICE_HID_EVENT_RESPONSE APP_USBDeviceHIDEventHandler
(
    USB_DEVICE_HID_INDEX hidInstance,
    USB_DEVICE_HID_EVENT event,
    void * eventData,
    uintptr_t userData
)
{
	APP_DATA * appData = (APP_DATA *)userData;

    switch(event)
    {
        case USB_DEVICE_HID_EVENT_REPORT_SENT:

            /* This means the mouse report was sent.
             We are free to send another report */

			if(hidInstance == HID_HINSTANCE_KEYBOARD){
				appData->isReportSentComplete = true;
            }else{
				appData->isReportSentCompleteMouse = true;
			}
			break;

        case USB_DEVICE_HID_EVENT_REPORT_RECEIVED:

            /* This means we have received a report */
            appData->isReportReceived = true;
            break;

        case USB_DEVICE_HID_EVENT_SET_IDLE:

             /* Acknowledge the Control Write Transfer */
           USB_DEVICE_ControlStatus(appData->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);

            /* save Idle rate recieved from Host */
            appData->idleRate
                   = ((USB_DEVICE_HID_EVENT_DATA_SET_IDLE*)eventData)->duration;
            break;

        case USB_DEVICE_HID_EVENT_GET_IDLE:

            /* Host is requesting for Idle rate. Now send the Idle rate */
            USB_DEVICE_ControlSend(appData->deviceHandle, & (appData->idleRate),1);

            /* On successfully reciveing Idle rate, the Host would acknowledge back with a
               Zero Length packet. The HID function drvier returns an event
               USB_DEVICE_HID_EVENT_CONTROL_TRANSFER_DATA_SENT to the application upon
               receiving this Zero Length packet from Host.
               USB_DEVICE_HID_EVENT_CONTROL_TRANSFER_DATA_SENT event indicates this control transfer
               event is complete */

            break;

        case USB_DEVICE_HID_EVENT_SET_PROTOCOL:
            /* Host is trying set protocol. Now receive the protocol and save */
            appData->activeProtocol
                = ((USB_DEVICE_HID_EVENT_DATA_SET_PROTOCOL *)eventData)->protocolCode;

              /* Acknowledge the Control Write Transfer */
            USB_DEVICE_ControlStatus(appData->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);
            break;

        case  USB_DEVICE_HID_EVENT_GET_PROTOCOL:

            /* Host is requesting for Current Protocol. Now send the Idle rate */
             USB_DEVICE_ControlSend(appData->deviceHandle, &(appData->activeProtocol), 1);

             /* On successfully reciveing Idle rate, the Host would acknowledge
               back with a Zero Length packet. The HID function drvier returns
               an event USB_DEVICE_HID_EVENT_CONTROL_TRANSFER_DATA_SENT to the
               application upon receiving this Zero Length packet from Host.
               USB_DEVICE_HID_EVENT_CONTROL_TRANSFER_DATA_SENT event indicates
               this control transfer event is complete */
             break;

        case USB_DEVICE_HID_EVENT_CONTROL_TRANSFER_DATA_SENT:
            break;

        default:
            break;
    }

    return USB_DEVICE_HID_EVENT_RESPONSE_NONE;
}

/*******************************************************************************
  Function:
    void APP_USBDeviceEventHandler (USB_DEVICE_EVENT event,
        USB_DEVICE_EVENT_DATA * eventData)

  Summary:
    Event callback generated by USB device layer.

  Description:
    This event handler will handle all device layer events.

  Parameters:
    None.

  Returns:
    None.
 */

void APP_USBDeviceEventHandler(USB_DEVICE_EVENT event,
        void * eventData, uintptr_t context)
{
    USB_DEVICE_EVENT_DATA_CONFIGURED *configurationValue;

    switch(event)
    {
        case USB_DEVICE_EVENT_SOF:
            break;

        case USB_DEVICE_EVENT_RESET:
        case USB_DEVICE_EVENT_DECONFIGURED:
            /* Device got deconfigured */
            LCD_ClearNW();
            appData.isConfigured = false;
            appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
            break;

        case USB_DEVICE_EVENT_CONFIGURED:
            /* Device is configured */
            configurationValue = (USB_DEVICE_EVENT_DATA_CONFIGURED *)eventData;
            if(configurationValue->configurationValue == 1)
            {
                appData.isConfigured = true;
                /* Register the Application HID Event Handler. */
                USB_DEVICE_HID_EventHandlerSet(appData.hidInstance,
                        APP_USBDeviceHIDEventHandler, (uintptr_t)&appData);

                /* Register the Application HID Event Handler. (for mouse)*/
                USB_DEVICE_HID_EventHandlerSet(appData.hidInstanceMouse,
                        APP_USBDeviceHIDEventHandler, (uintptr_t)&appData);
            }
            LCD_SetBL(true);
            break;

        case USB_DEVICE_EVENT_SUSPENDED: 
			LED_Set(LED_COLOR_OFF, LED_BLINK_NONE);
            LCD_SetBL(false);
            break;

        case USB_DEVICE_EVENT_RESUMED:
        case USB_DEVICE_EVENT_POWER_DETECTED:
            /* Attach the device */
            USB_DEVICE_Attach (appData.deviceHandle);
            break;
        case USB_DEVICE_EVENT_POWER_REMOVED:
            /* There is no VBUS. We can detach the device */
            USB_DEVICE_Detach(appData.deviceHandle);
            break;
        case USB_DEVICE_EVENT_ERROR:
        default:
            break;

    }
}


// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

/*****************************************************
* Expand modifier key table
******************************************************/
void ExpandModifiers() {
	uint8_t mod_index;
	memset(modifiers, 0, sizeof(modifiers));
	for (mod_index = 0; mod_index < KEY_COUNT; mod_index++) {
		uint8_t modifier = keytable[mod_index].modifier;
		uint8_t bitcount = bitcount8(modifier);
		if (modifier != HID_NONM && bitcount > 1) {
			uint8_t dst_index = 0;
			uint8_t pattern = 0;
			if (modifier & HID_GUIM) {
				pattern += HID_GUIM;
				modifiers[mod_index].element[dst_index] = pattern;
				dst_index += 1;
			}
			if (modifier & HID_ALTM) {
				pattern += HID_ALTM;
				modifiers[mod_index].element[dst_index] = pattern;
				dst_index += 1;
			}
			if (modifier & HID_SFTM) {
				pattern += HID_SFTM;
				modifiers[mod_index].element[dst_index] = pattern;
				dst_index += 1;
			}
			if (modifier & HID_CTLM) {
				pattern += HID_CTLM;
				modifiers[mod_index].element[dst_index] = pattern;
			}
		}
	}
}

void SendModifierElement(uint8_t mod) {
	keyboardInputReport.data[KBD_RPT_IDX_MODIFIER] = mod;
	keyboardInputReport.data[1]
		= keyboardInputReport.data[2]
		= keyboardInputReport.data[3]
		= keyboardInputReport.data[4]
		= USB_HID_NOEVENT;
	if (appData.isReportSentComplete)
	{
		USB_DEVICE_HID_ReportSend(appData.hidInstance,
			&appData.sendTransferHandle,
			(uint8_t *)&keyboardInputReport,
			sizeof(KEYBOARD_INPUT_REPORT));
	}
	while (!appData.isReportSentComplete)
		;
}

void SendModifiers(uint8_t bitpos) {
	uint8_t i;
	for (i = 0; i < 4; i++) {
		uint8_t mod = modifiers[bitpos].element[i];
		if (mod == HID_NONM) break;
		SendModifierElement(mod);
	}
}


/********************************************************
 * Application Keyboard LED update routine.
 ********************************************************/
#if 0
void APP_KeyboardLEDStatus(void)
{
    /* This measn we have a valid output report from the host*/

    if(keyboardOutputReport.ledState.numLock
            == KEYBOARD_LED_STATE_ON)
	{
		LED_Set(LED_COLOR_GREEN,LED_BLINK_NONE);
    }else{
		LED_Set(LED_COLOR_OFF,LED_BLINK_NONE);
    }

    if(keyboardOutputReport.ledState.capsLock
            == KEYBOARD_LED_STATE_ON)
	{
        LED_Set(LED_COLOR_GREEN,LED_BLINK_NONE);
    }else{
        LED_Set(LED_COLOR_OFF,LED_BLINK_NONE);
    }
}
#endif
/********************************************************
 * Application Keyboard Emulation Routine
 ********************************************************/

bool APP_EmulateKeyboard(void)
{
    uint32_t rkey;
    uint8_t bitpos;
    bool isRotator,isMouseReport = false,isKeyReport = false;
    static bool prevKey,prevMouse;
    if(appData.isKeyPressed) {
        bitpos = ntz32(appData.keystat.wd);
        isRotator = (bitpos < (KEY_COUNT+8)) && (bitpos >= (KEY_COUNT));    //is bitpos value in 16-23 range?
        rkey = (appData.keystat.wd & MOD_SW_BIT_MASK);
        if(bitpos < KEY_COUNT || (appData.keymouse == SIDE_SW_KEYBOARD && isRotator) || bitpos == (KEY_COUNT+9) || bitpos == (KEY_COUNT+10) ){
			if(modifiers[bitpos].element[0] != HID_NONM) SendModifiers(bitpos);

            keyboardInputReport.data[KBD_RPT_IDX_MODIFIER] = keytable[bitpos].modifier;
            keyboardInputReport.data[KBD_RPT_IDX_KEYCODE] = keytable[bitpos].keycode;
            isKeyReport = prevKey = true;
        }else if(appData.keymouse == SIDE_SW_MOUSE && isRotator ){
            uint8_t i=(bitpos)-KEY_COUNT;
			mouseReport.data[mousetable[i].index] = mousetable[i].movement;
            isMouseReport = prevMouse = true;
		}else if(bitpos == (KEY_COUNT+8)){
            //Side switch
            appData.keymouse = (appData.keymouse == SIDE_SW_KEYBOARD)? SIDE_SW_MOUSE:SIDE_SW_KEYBOARD;
#if 0
		}else if(bitpos == (KEY_COUNT+9)){
            //Lower Rotator button
            LED_Set(LED_COLOR_GREEN,LED_BLINK_FAST);
		}else if(bitpos == (KEY_COUNT+10)){
            //Upper Rotator button
            LED_Set(LED_COLOR_MAGENTA,LED_BLINK_FAST);
#endif
		}else if(rkey == 0) {// Keys are released            
            LED_Set((appData.keymouse == SIDE_SW_KEYBOARD)? LED_COLOR_RED:LED_COLOR_BLUE, LED_BLINK_NONE);
            if(prevKey){
                keyboardInputReport.data[KBD_RPT_IDX_MODIFIER] =
                    keyboardInputReport.data[KBD_RPT_IDX_KEYCODE] = USB_HID_NOEVENT;
                isKeyReport = true;
                prevKey = false;
            }
            if(prevMouse){
                memset(&mouseReport.data, 0, sizeof(mouseReport.data));
                isMouseReport = true;
                prevMouse = false;
            }
        }else{
            ;//do nothing
        }

        if(isKeyReport){
            USB_DEVICE_HID_ReportSend(appData.hidInstance,
                &appData.sendTransferHandle,
                (uint8_t *)&keyboardInputReport,
                sizeof(KEYBOARD_INPUT_REPORT));
            isKeyReport = false;
        }
        if(isMouseReport){
            USB_DEVICE_HID_ReportSend(appData.hidInstanceMouse,
                &appData.sendTransferHandleMouse,
                (uint8_t*)&mouseReport,
                sizeof(MOUSE_REPORT));
            isMouseReport = false;
        }

        /* Clear the switch pressed flag */
        appData.isKeyPressed = false;
        return true;
    } else
        return false;
}


/**********************************************
 * This function is called by when the device
 * is de-configured. It resets the application
 * state in anticipation for the next device
 * configured event
 **********************************************/

void APP_StateReset(void)
{
    appData.isReportReceived = false;
    appData.isReportSentComplete = true;
    memset(&keyboardOutputReport.data, 0, 64);
    appData.isKeyPressed = false;
	appData.keymouse = SIDE_SW_KEYBOARD;
	appData.keyline = 0;
    appData.lcd_flag = false;
    appData.lcd_timer = LCD_TIMER_HALT;
    LCD_SetBL(false);
    appData.isReportSentCompleteMouse = true;

	LED_Set(LED_COLOR_OFF, LED_BLINK_NONE);
}


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;

    /* Initialize appData. */
    appData.deviceHandle = USB_DEVICE_HANDLE_INVALID;
    appData.isConfigured = false;

    /* Initialize keyboardInputReport */
    memset(&keyboardOutputReport.data, USB_HID_NOEVENT, 8);

    /* Initialize the led state */
    memset(&keyboardOutputReport.data, 0, 64);

    /* Initialize the switch state */
    appData.isKeyPressed = false;

    /* Initialize the HID instance index.  */
    appData.hidInstance = HID_HINSTANCE_KEYBOARD;

    /* Initialize tracking variables */
    appData.isReportReceived = false;
	appData.isReportSentComplete = true;

    appData.keymouse = SIDE_SW_KEYBOARD;
	appData.keyline = 0;

	//Initialize mouse
	appData.hidInstanceMouse = HID_HINSTANCE_MOUSE;
    appData.isReportSentCompleteMouse = true;

	ExpandModifiers();
    LED_Initialize();
    LED_Set(LED_COLOR_OFF, LED_BLINK_NONE);

    DRV_ADC_Open();
    DRV_ADC_Start();
    
    appData.lcd_timer = LCD_TIMER_INIT; // 20msec
    appData.lcd_flag = false;
    while(appData.lcd_flag==false){
        DelayUs( (LCD_TIMER_INIT / 5) * 100 );
    }
    LCD_Initialize();
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{
    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
		    /* Open the device layer */
            appData.deviceHandle = USB_DEVICE_Open( USB_DEVICE_INDEX_0,
                    DRV_IO_INTENT_READWRITE );

            if(appData.deviceHandle != USB_DEVICE_HANDLE_INVALID)
            {
                /* Register a callback with device layer to get event notification (for end point 0) */
                USB_DEVICE_EventHandlerSet(appData.deviceHandle,
                        APP_USBDeviceEventHandler, 0);

                appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
            }
            else
            {
                /* The Device Layer is not ready to be opened. We should try
                 * again later. */
            }
            break;
        }

        case APP_STATE_WAIT_FOR_CONFIGURATION:

            /* Check if the device is configured. The
             * isConfigured flag is updated in the
             * Device Event Handler */

            if(appData.isConfigured)
            {
                /* Initialize the flag and place a request for a
                 * output report */

                appData.isReportReceived = false;
                LED_Set(LED_COLOR_RED, LED_BLINK_NONE);

                // Opening message
                char msg[LCD_WIDTH + 1];
                LCD_Clear();
                LCD_Print("LRKB");

                LCD_Locate(4,1);
                sprintf(msg,"%x.%02x",(LRKB_REVISION >> 8), (LRKB_REVISION % 0x100) );
                LCD_Print(msg);
                LCD_SetBL(true);
                appData.lcd_timer = LCD_TIMER_DEFAULT; 
                appData.lcd_flag = false;

                USB_DEVICE_HID_ReportReceive(appData.hidInstance,
                    &appData.receiveTransferHandle,
                    (uint8_t *)&keyboardOutputReport,64);

				appData.state = APP_STATE_CHECK_IF_CONFIGURED;
            }else{
                if(DRV_ADC_SamplesAvailable()){
                    char msg[LCD_WIDTH];
                    uint16_t samp = DRV_ADC_SamplesRead(11);
                    int16_t offset = samp - 512;
                    float temp = 25.0 - (offset * 0.1);
                    //sprintf(msg,"%2.1fC\xdf", (double)temp);
                    sprintf(msg,"%d", samp);
                    LCD_Locate(2,1);
                    LCD_Print(msg);
                }
            }
            break;

        case APP_STATE_CHECK_IF_CONFIGURED:

            /* This state is needed because the device can get
             * unconfigured asynchronously. Any application state
             * machine reset should happen within the state machine
             * context only. */

            if(appData.isConfigured)
            {
                //for Increse responce, Ignore output reports.
                //appData.state = APP_STATE_CHECK_FOR_OUTPUT_REPORT;
                appData.state = APP_STATE_EMULATE_KEYBOARD;
            }
            else
            {
                /* This means the device got de-configured.
                 * We reset the state and the wait for configuration */

                APP_StateReset();
                appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
            }
            break;
#if 0
        case APP_STATE_CHECK_FOR_OUTPUT_REPORT:

            if(appData.isReportReceived == true)
            {
                /* Update the LED and schedule and
                 * request */

				APP_KeyboardLEDStatus();
                appData.isReportReceived = false;
                USB_DEVICE_HID_ReportReceive(appData.hidInstance,
                        &appData.receiveTransferHandle,
                        (uint8_t *)&keyboardOutputReport,64);
			}

            appData.state = APP_STATE_EMULATE_KEYBOARD;
            break;
#endif
        case APP_STATE_EMULATE_KEYBOARD:
            if( (appData.keymouse == SIDE_SW_KEYBOARD && appData.isReportSentComplete)
                    || (appData.keymouse == SIDE_SW_MOUSE && appData.isReportSentCompleteMouse) ) {
                APP_EmulateKeyboard();
            }
            if(appData.lcd_flag){
                LCD_Clear();
                LCD_SetBL(false);
                appData.lcd_flag = false;
            }
            appData.state = APP_STATE_CHECK_IF_CONFIGURED;
            break;

        case APP_STATE_ERROR:
            break;

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    } //switch

}

/*******************************************************************************
 End of File
 */

