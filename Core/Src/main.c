/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "lwip.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32746g_discovery.h"
#include "stm32746g_discovery_lcd.h"
#include "stm32746g_discovery_sdram.h"
#include "stm32746g_discovery_ts.h"
#include "stm32746g_discovery_qspi.h"
#include "echo_server.h"
#include "logo.h"
#include "bg_img.h"
#include "string.h"
#include "flag.h"
#include "next.h"
#include "humidity.h"
#include "pressure.h"
#include "stdbool.h"

#define SCREENSAVER_DELAY 300000
//#include "tcp.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern struct netif gnetif;  //  Required for netif_is_up()

#define TCP_WRITE_FLAG_COPY 0x01
#define TCP_WRITE_FLAG_MORE 0X02

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

DMA2D_HandleTypeDef hdma2d;

LTDC_HandleTypeDef hltdc;

QSPI_HandleTypeDef hqspi;

UART_HandleTypeDef huart1;

SDRAM_HandleTypeDef hsdram1;

/* USER CODE BEGIN PV */
uint8_t screen_state = 0;  // 0: Temp, 1: Humidity, 2: Pressure
char latest_temp[32];
char latest_hum[32];
char latest_press[32];
int temp_ready = 0;
int hum_ready = 0;
int press_ready = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LTDC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_DMA2D_Init(void);
static void MX_FMC_Init(void);
static void MX_QUADSPI_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//This function will send the characters from printf via UART1.
//Modified so you don't have to type \r\n anymore, just type \n.
#include <errno.h>
#include <sys/stat.h>
#include <sys/times.h>
#include <sys/unistd.h>

// ============================================================================
// Retarget printf to UART1 by implementing _write()
// Injects '\r' before '\n' so terminals expecting CRLF display lines correctly.
// ============================================================================
int _write(int file, char *ptr, int len) {
	for(int i = 0; i < len; i++){
		if(ptr[i]=='\n'){
			HAL_UART_Transmit(&huart1, (uint8_t*)"\r", 1, HAL_MAX_DELAY);  // Prepend CR so "\n" becomes "\r\n"
		}
		HAL_UART_Transmit(&huart1, (uint8_t*)&ptr[i], 1, HAL_MAX_DELAY);   // Send current character
	}
    return len;  // Inform newlib that all bytes were "written"
}


// ============================================================================
// Screensaver control
// Blanks the LCD after SCREENSAVER_DELAY of no touch; wakes on touch.
// ============================================================================
void checkScreensaver(void){
	static uint32_t ScreensaverStart = SCREENSAVER_DELAY+100;  // Next time (tick) at which screensaver should activate
	static uint8_t screensaver_status = 0;                     // 0 = off (display on), 1 = on (display off)

	// If current time has passed the target and screensaver is off, turn it on
	if(ScreensaverStart < HAL_GetTick() && screensaver_status == 0){
		// Screen saver on -> display Off (backlight + display)
		HAL_GPIO_WritePin(LCD_DISP_GPIO_PORT, LCD_DISP_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LCD_BL_CTRL_GPIO_PORT, LCD_BL_CTRL_PIN, GPIO_PIN_RESET);
		screensaver_status = 1;
		printf("Screensaver: on\n");
	}

	// Read current touch state
	TS_StateTypeDef TS_State;
	BSP_TS_GetState(&TS_State);

	// If any finger is detected, postpone screensaver and, if needed, wake display
	if(TS_State.touchDetected > 0){
		// New start value (postpone activation)
		ScreensaverStart = HAL_GetTick() + SCREENSAVER_DELAY;

		// If already off, turn display back on
		if(screensaver_status == 1){
			HAL_GPIO_WritePin(LCD_DISP_GPIO_PORT, LCD_DISP_PIN, GPIO_PIN_SET);
			HAL_Delay(100);  // Small delay to allow panel power-up
			HAL_GPIO_WritePin(LCD_BL_CTRL_GPIO_PORT, LCD_BL_CTRL_PIN, GPIO_PIN_SET);
			screensaver_status = 0;
			printf("Screensaver: off\n");
		}
	}
}


// ============================================================================
// RX buffer for incoming HTTP payload (fits small JSON and headers)
// ============================================================================
char rx_buffer[500];  // to store received quote (HTTP response body)

// ============================================================================
// LwIP TCP control block pointer for the client
// ============================================================================
static struct tcp_pcb *client_pcb;

// ============================================================================
// LwIP raw API forward declarations (provided by LwIP headers)
// - tcp_new(): allocate new PCB
// - tcp_connect(): initiate connection
// - tcp_arg(): set user argument for callbacks
// - tcp_write(): enqueue bytes to send
// - tcp_sent(): set sent callback
// - tcp_recv(): set receive callback
// - tcp_recved(): tell stack how many bytes we consumed
// - tcp_poll(): set periodic poll callback
// - tcp_close()/tcp_abort(): close or abort the connection
// - tcp_err(): set error callback
// ============================================================================
struct tcp_pcb *tcp_new(void);

err_t tcp_connect(struct tcp_pcb *pcb,struct ip_addr *ipaddr,u16_t port,err_t(*connected)(void *arg,struct tcp_pcb *tpcb,err_t err));
void tcp_arg(struct tcp_pcb *pcb,void *arg);

err_t tcp_write(struct tcp_pcb*pcb,void *dataptr,u16_t len,u8_t copy);
void tcp_sent(struct tcp_pcb *pcb, err_t(*sent)(void *arg,struct tcp_pcb *tpcb,u16_t len));

void tcp_recv(struct tcp_pcb *pcb,err_t(*recv)(void *arg, struct tcp_pcb *tpcb,struct pbuf *p,err_t err));
void tcp_recved(struct tcp_pcb *pcb,u16_t len);
void tcp_poll(struct tcp_pcb *pcb ,u8_t interval,
		err_t(*poll)(void *arg ,struct tcp_pcb *tpcb));
err_t tcp_close(struct tcp_pcb *pcb);
void tcp_abort(struct tcp_pcb *pcb);
void tcp_err(struct tcp_pcb *pcb, void(*err)(void *arg,err_t err));


// ============================================================================
// LwIP "sent" callback: called when previously queued data is ACKed by peer.
// Here we don't need to do anything, just return ERR_OK.
// ============================================================================
static err_t sent_callback(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
    return ERR_OK;
}


// ============================================================================
// LwIP "recv" callback: called when data arrives or when the connection closes.
// - If p == NULL, the remote side closed; close our PCB and return.
// - Otherwise copy payload, parse fields, notify LwIP how many bytes we used,
//   free the pbuf, and close connection (one-shot HTTP).
// ============================================================================
static err_t recv_callback(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    if (!p) {
        tcp_close(tpcb);            // Remote closed; close our side
        return ERR_OK;
    }

    // Copy payload into rx_buffer and null-terminate
    int len = (p->len < sizeof(rx_buffer) - 1) ? p->len : sizeof(rx_buffer) - 1;  // Clamp length to buffer size - 1
    memcpy(rx_buffer, p->payload, len);                                           // Copy from pbuf to local buffer
    rx_buffer[len] = '\0';                                                        // Ensure C-string termination

    // Find JSON-like fields inside response buffer
    char *start  = strstr(rx_buffer, "\"temp\":\"");   // Locate "temp" value start
    char *start1 = strstr(rx_buffer, "\"press\":\"");  // Locate "press" value start
    char *start2 = strstr(rx_buffer, "\"hum\":\"");    // Locate "hum" value start

    if (start) {
        start += strlen(":\Temp:" ":\:\:\:\" ");  // Intended: move pointer to start of value after the quote.


        char *end = strchr(start, '"');  // Find closing quote of the value
        if (end) {
            *end = '\0';  // Null-terminate at end of value (temporarily modifies rx_buffer)
        }

        printf("Temperature: %s\r\n", start);   // Print extracted value
        strcpy(latest_temp, start);             // Save globally for LCD
        temp_ready = 1;                         // Flag for screen update
    }

    if (start1) {
          start1 += strlen(":\Press:" ":\:\:\:\:\:\" ");

          char *end = strchr(start1, '"');  // Find closing quote
          if (end) {
              *end = '\0';  // Terminate string
          }

          printf("Pressure: %s\r\n", start1);
          strcpy(latest_press, start1);
          press_ready = 1;
      }

    if (start2) {
          start2 += strlen(":\HUM:" ":\:\:\:\" ");

          char *end = strchr(start2, '"');  // Find closing quote
          if (end) {
              *end = '\0';  // Terminate string
          }

          printf("Humidity: %s\r\n", start2);
          strcpy(latest_hum, start2);
          hum_ready = 1;
      }
    else {
        printf("Raw data: %s\r\n", rx_buffer);  // Fallback logging if keys not found
    }

    tcp_recved(tpcb, p->len);  // Tell LwIP we've consumed p->len bytes
    pbuf_free(p);              // Free the received pbuf chain
    tcp_close(tpcb);           // Close connection (one request per connection)
    return ERR_OK;
}


// ============================================================================
/* LwIP "connected" callback:
 * Called once TCP 3-way handshake completes.
 * Builds and sends a minimal HTTP/1.0 GET request in three tcp_write() calls,
 * then registers sent/recv callbacks.
 */
static err_t connected_callback(void *arg, struct tcp_pcb *tpcb, err_t err) {
    if (err != ERR_OK) return err;  // If connect failed, propagate error

    const char cAppId[] = "1234"; // change this to your own AppID
    const char cHttpHeaderTemplate1[] = "GET /xampp/readtemp.php?q=brussels,b&appid=";
    const char cHttpHeaderTemplate2[] = " HTTP/1.0\r\nHost: 192.168.69.11\r\n\r\n";  // local IP

    tcp_write(tpcb, cHttpHeaderTemplate1, strlen (cHttpHeaderTemplate1), TCP_WRITE_FLAG_COPY); // First part of the header
    tcp_write (tpcb, cAppId , strlen(cAppId), TCP_WRITE_FLAG_COPY);                            // Append AppID
    tcp_write(tpcb, cHttpHeaderTemplate2, strlen (cHttpHeaderTemplate2), 0);                   // Finish header (no COPY saves pbufs)

    tcp_sent(tpcb, sent_callback);  // Register ACK (sent) callback
    tcp_recv(tpcb, recv_callback);  // Register receive callback
    return ERR_OK;
}


// ============================================================================
// Initiate the TCP connection to the PHP server (port 80).
// Creates a new PCB, sets server IP, and calls tcp_connect() with our callback.
// ============================================================================
void tcp_client_connect(void) {
    ip_addr_t server_ip;
    IP4_ADDR(&server_ip, 192, 168, 69, 11);  // Your PC server

    client_pcb = tcp_new();   // Allocate new TCP PCB
    if (!client_pcb) return;  // Allocation failed; simply return

    tcp_connect(client_pcb, &server_ip, 80, connected_callback);  // Start async connect
}


// ============================================================================
// Draw static icons and a header line on the LCD.
// Uses BSP LCD APIs and a custom bitmap drawing helper.
// ============================================================================
void print_icons (void){

    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);

	  char buffer[50];
	  sprintf(buffer, "Live Weather in Belgium ");
	  BSP_LCD_DisplayStringAt(0, 10, (uint8_t*) buffer, CENTER_MODE);

	  // Draw small flag icon (top-right)
	   WDA_LCD_DrawBitmap(
	     FLAG_DATA,
	 	  450,                 // X pos (near right edge)
	 	  10,                  // Y pos
	     24,                   // width
	     24,                   // height
	     FLAG_DATA_FORMAT
	   );

	  // Draw main weather icon (center-ish)
	  WDA_LCD_DrawBitmap(
	    WEATHER_DATA,
		  208,                 // X pos
		  80,                  // Y pos
	    100,                  // width
	    100,                  // height
	    WEATHER_DATA_FORMAT
	  );
}


// ============================================================================
// Draw one of the three metric screens (Temperature / Humidity / Pressure).
// Layer 1 is used as the foreground; NEXT icon is drawn for cycling.
// ============================================================================
void draw_screen(uint8_t state) {
    BSP_LCD_SelectLayer(1);                      // Draw on foreground layer
    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
    BSP_LCD_SetFont(&Font24);

    switch (state) {
        case 0:
            BSP_LCD_DisplayStringAt(100, 200, (uint8_t*)"Temperature:", LEFT_MODE);
            BSP_LCD_DisplayStringAt(280, 200, (uint8_t*)latest_temp, LEFT_MODE);
            print_icons();  // Also draw header/icons for this screen
            break;

        case 1:
            BSP_LCD_DisplayStringAt(100, 200, (uint8_t*)"Humidity:", LEFT_MODE);
            BSP_LCD_DisplayStringAt(280, 200, (uint8_t*)latest_hum, LEFT_MODE);
            WDA_LCD_DrawBitmap(
                HUMIDITY_DATA, 208, 80, 100, 100, HUMIDITY_DATA_FORMAT
            );
            break;

        case 2:
            BSP_LCD_DisplayStringAt(100, 200, (uint8_t*)"Pressure:", LEFT_MODE);
            BSP_LCD_DisplayStringAt(280, 200, (uint8_t*)latest_press, LEFT_MODE);
            WDA_LCD_DrawBitmap(
                PRESSURE_DATA, 208, 80, 100, 100, PRESSURE_DATA_FORMAT
            );
            break;
    }

    // Draw the NEXT icon (used as a touch button to cycle screens)
    WDA_LCD_DrawBitmap(
        NEXT_DATA, NEXT_X, NEXT_Y, NEXT_DATA_X_PIXEL, NEXT_DATA_Y_PIXEL, NEXT_DATA_FORMAT
    );
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_LTDC_Init();
  MX_USART1_UART_Init();
  MX_DMA2D_Init();
  MX_FMC_Init();
  MX_LWIP_Init();
  MX_QUADSPI_Init();
  /* USER CODE BEGIN 2 */

    /*QSPI INIT*/
    BSP_QSPI_Init();                               // Initialize external QSPI flash (used for assets)
    BSP_QSPI_MemoryMappedMode();                   // Map QSPI into MCU address space for zero-copy reads
    WRITE_REG(QUADSPI->LPTR, 0xFFF);               // Extend QSPI Low-Power Timeout (LPTR) to max to avoid premature timeouts
    printf("Running LwIP & LCD start project...\n");
    echo_init();                                   // (Optional) Start LwIP echo server for network bring-up/testing

    BSP_LCD_Init();                                // Initialize LCD controller (LTDC via BSP)
    BSP_LCD_LayerDefaultInit(1, LCD_FB_START_ADDRESS);                     // Layer 1 framebuffer base (foreground)
    BSP_LCD_LayerDefaultInit(0, LCD_FB_START_ADDRESS+(480*272*4));         // Layer 0 framebuffer base (background), offset by one full ARGB8888 frame

    /* Enable the LCD */
    BSP_LCD_DisplayOn();                           // Power on LCD (panel + backlight as configured)

    /* Select the LCD Background Layer  */
    BSP_LCD_SelectLayer(0);                        // Work on background layer (layer 0)

    /* Clear the Background Layer */
    BSP_LCD_Clear(LCD_COLOR_WHITE);                // Fill background with white

    WDA_LCD_DrawBitmap(                            // Draw centered background image from QSPI/array
      bg_img_DATA,
  	  (480- bg_img_DATA_X_PIXEL)/2,              // Center X
  	  (272- bg_img_DATA_Y_PIXEL)/2,              // Center Y
  	  bg_img_DATA_X_PIXEL,                       // Width
  	  bg_img_DATA_Y_PIXEL,                       // Height
      bg_img_DATA_FORMAT
    );



    BSP_LCD_SelectLayer(1);                        // Switch to foreground layer (overlay)
    /* Clear the foreground Layer */
    BSP_LCD_Clear(LCD_COLOR_WHITE);                // Initial wipe (color doesn’t matter—will be made transparent next)

    /* Some sign */
    BSP_LCD_SetLayerVisible(1, ENABLE);            // Ensure foreground layer is enabled
    BSP_LCD_SetTransparency(1, 255);               // 0 = fully transparent, 255 = opaque (we want visible text/icons)
    BSP_LCD_Clear(LCD_COLOR_TRANSPARENT);          // Make the whole foreground layer transparent to start clean
    print_icons();                                 // Draw header/icons on the foreground layer

    /* Init touch screen */
    BSP_TS_Init(480,272);                          // Initialize touch controller with display resolution

    uint32_t last_request = 0;                     // Ticks of last HTTP GET (rate-limiting network requests)
    bool last_touch = false;                       // Edge detect for touch (press -> release)


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	      MX_LWIP_Process();                             // Run LwIP housekeeping (timers, input, timeouts)
	      checkScreensaver();                            // Update LCD on/off state based on touch inactivity

	      // If network interface is up AND at least 500 ms passed since last request, trigger a new HTTP GET
	      if (netif_is_up(&gnetif) && HAL_GetTick() - last_request > 500) {
	               tcp_client_connect();                 // Start async TCP connection to PHP server (will call callbacks)
	               last_request = HAL_GetTick();         // Mark time of this request (simple rate limiter)
	           }

	          // When a new temperature value has been parsed, refresh the screen
	          if (temp_ready) {
	              draw_screen(screen_state);             // Redraw current metric screen
	              temp_ready = 0;                        // Clear flag so we don't redraw continuously
	          }

	          TS_StateTypeDef ts;
	          BSP_TS_GetState(&ts);                      // Poll touch controller for current state

	          // Rising-edge detect: react only on transition from "no touch" to "touch"
	          if (ts.touchDetected && !last_touch) {
	              last_touch = true;                     // Touch started (remember state)

	              uint16_t x = ts.touchX[0];             // First touch point X
	              uint16_t y = ts.touchY[0];             // First touch point Y

	              // Hit-test: check if touch is inside the NEXT icon bounding box
	              if (x >= NEXT_X && x < NEXT_X + NEXT_DATA_X_PIXEL &&
	                  y >= NEXT_Y && y < NEXT_Y + NEXT_DATA_Y_PIXEL) {
	                  BSP_LCD_Clear(LCD_COLOR_TRANSPARENT);     // Clear foreground layer before drawing next screen

	                  screen_state = (screen_state + 1) % 3;    // Cycle screens: 0 → 1 → 2 → 0
	                  draw_screen(screen_state);                // Draw newly selected screen
	              }
	          } else if (!ts.touchDetected && last_touch) {
	              last_touch = false;                    // Falling-edge detect: touch released
	          }

	          HAL_Delay(30);                             // Small delay to reduce CPU usage and debounce/settle UI


	}

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DMA2D Initialization Function
  * @param None
  * @retval None
  */
static void MX_DMA2D_Init(void)
{

  /* USER CODE BEGIN DMA2D_Init 0 */

  /* USER CODE END DMA2D_Init 0 */

  /* USER CODE BEGIN DMA2D_Init 1 */

  /* USER CODE END DMA2D_Init 1 */
  hdma2d.Instance = DMA2D;
  hdma2d.Init.Mode = DMA2D_M2M;
  hdma2d.Init.ColorMode = DMA2D_OUTPUT_ARGB8888;
  hdma2d.Init.OutputOffset = 0;
  hdma2d.LayerCfg[1].InputOffset = 0;
  hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_ARGB8888;
  hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
  hdma2d.LayerCfg[1].InputAlpha = 0;
  if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DMA2D_Init 2 */

  /* USER CODE END DMA2D_Init 2 */

}

/**
  * @brief LTDC Initialization Function
  * @param None
  * @retval None
  */
static void MX_LTDC_Init(void)
{

  /* USER CODE BEGIN LTDC_Init 0 */

  /* USER CODE END LTDC_Init 0 */

  LTDC_LayerCfgTypeDef pLayerCfg = {0};
  LTDC_LayerCfgTypeDef pLayerCfg1 = {0};

  /* USER CODE BEGIN LTDC_Init 1 */

  /* USER CODE END LTDC_Init 1 */
  hltdc.Instance = LTDC;
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  hltdc.Init.HorizontalSync = 40;
  hltdc.Init.VerticalSync = 9;
  hltdc.Init.AccumulatedHBP = 53;
  hltdc.Init.AccumulatedVBP = 11;
  hltdc.Init.AccumulatedActiveW = 533;
  hltdc.Init.AccumulatedActiveH = 283;
  hltdc.Init.TotalWidth = 565;
  hltdc.Init.TotalHeigh = 285;
  hltdc.Init.Backcolor.Blue = 0;
  hltdc.Init.Backcolor.Green = 255;
  hltdc.Init.Backcolor.Red = 0;
  if (HAL_LTDC_Init(&hltdc) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg.WindowX0 = 0;
  pLayerCfg.WindowX1 = 480;
  pLayerCfg.WindowY0 = 0;
  pLayerCfg.WindowY1 = 272;
  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_ARGB1555;
  pLayerCfg.Alpha = 255;
  pLayerCfg.Alpha0 = 0;
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
  pLayerCfg.FBStartAdress = 0;
  pLayerCfg.ImageWidth = 480;
  pLayerCfg.ImageHeight = 272;
  pLayerCfg.Backcolor.Blue = 0;
  pLayerCfg.Backcolor.Green = 0;
  pLayerCfg.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg1.WindowX0 = 0;
  pLayerCfg1.WindowX1 = 480;
  pLayerCfg1.WindowY0 = 0;
  pLayerCfg1.WindowY1 = 272;
  pLayerCfg1.PixelFormat = LTDC_PIXEL_FORMAT_ARGB1555;
  pLayerCfg1.Alpha = 255;
  pLayerCfg1.Alpha0 = 0;
  pLayerCfg1.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
  pLayerCfg1.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
  pLayerCfg1.FBStartAdress = 0;
  pLayerCfg1.ImageWidth = 480;
  pLayerCfg1.ImageHeight = 272;
  pLayerCfg1.Backcolor.Blue = 0;
  pLayerCfg1.Backcolor.Green = 0;
  pLayerCfg1.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg1, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LTDC_Init 2 */

  /* USER CODE END LTDC_Init 2 */

}

/**
  * @brief QUADSPI Initialization Function
  * @param None
  * @retval None
  */
static void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 255;
  hqspi.Init.FifoThreshold = 1;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_NONE;
  hqspi.Init.FlashSize = 1;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  hqspi.Init.FlashID = QSPI_FLASH_ID_1;
  hqspi.Init.DualFlash = QSPI_DUALFLASH_DISABLE;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */

  /* USER CODE END QUADSPI_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/* FMC initialization function */
static void MX_FMC_Init(void)
{

  /* USER CODE BEGIN FMC_Init 0 */

  /* USER CODE END FMC_Init 0 */

  FMC_SDRAM_TimingTypeDef SdramTiming = {0};

  /* USER CODE BEGIN FMC_Init 1 */

  /* USER CODE END FMC_Init 1 */

  /** Perform the SDRAM1 memory initialization sequence
  */
  hsdram1.Instance = FMC_SDRAM_DEVICE;
  /* hsdram1.Init */
  hsdram1.Init.SDBank = FMC_SDRAM_BANK1;
  hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
  hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
  hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
  hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_1;
  hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_DISABLE;
  hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_DISABLE;
  hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_0;
  /* SdramTiming */
  SdramTiming.LoadToActiveDelay = 16;
  SdramTiming.ExitSelfRefreshDelay = 16;
  SdramTiming.SelfRefreshTime = 16;
  SdramTiming.RowCycleDelay = 16;
  SdramTiming.WriteRecoveryTime = 16;
  SdramTiming.RPDelay = 16;
  SdramTiming.RCDDelay = 16;

  if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FMC_Init 2 */

  /* USER CODE END FMC_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOJ_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOK_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_BL_CTRL_GPIO_Port, LCD_BL_CTRL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOI, LED_Pin|LCD_DISP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LCD_BL_CTRL_Pin */
  GPIO_InitStruct.Pin = LCD_BL_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_BL_CTRL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Pin LCD_DISP_Pin */
  GPIO_InitStruct.Pin = LED_Pin|LCD_DISP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
