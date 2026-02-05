/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <string.h>
#include <stdio.h>
#include <stdbool.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

extern UART_HandleTypeDef huart1; // Sensor-UART (zum Radar)
extern UART_HandleTypeDef huart2; // Debug-UART (zum PC)

#define BITFIELD_PDAT_DONE   0b00100100  // Bit2=PDAT, Bit5=DONE

#define RX_BUFFER_SIZE 512
uint8_t rx_buf[RX_BUFFER_SIZE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void Debug_Print(const char *s)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)s, (uint16_t)strlen(s), 1000);
}

static HAL_StatusTypeDef Sensor_Send(const uint8_t *data, uint16_t len)
{
    return HAL_UART_Transmit(&huart1, (uint8_t *)data, len, 1000);
}

static HAL_StatusTypeDef Sensor_Recv(uint8_t *data, uint16_t len)
{
    return HAL_UART_Receive(&huart1, data, len, 1000);
}

const char *get_error_description(uint8_t error_code)
{
    switch (error_code)
    {
    case 0: return "OK, no error";
    case 1: return "Unknown command";
    case 2: return "Invalid parameter value";
    case 3: return "Invalid RPST version";
    case 4: return "UART error (parity, framing, noise)";
    case 5: return "No calibration values";
    case 6: return "Timeout";
    case 7: return "Application corrupt or not programmed";
    default: return "Unknown Error";
    }
}

bool read_message(char header[5], uint8_t *payload, uint32_t *payload_len, uint32_t max_payload)
{

	/*uint8_t b;
	    char buf[8];

	    while (1) {
	        if (Sensor_Recv(&b, 1) != HAL_OK) {
	            Debug_Print("no byte\r\n");
	            return false;
	        }
	        snprintf(buf, sizeof(buf), "%02X ", (unsigned int)b);
	        Debug_Print(buf);
	    }*/

	 uint8_t hdr[4];
	    uint8_t len_bytes[4];

	    // 4-Byte Header lesen
	    if (Sensor_Recv(hdr, 4) != HAL_OK)
	    {
	        Debug_Print("read_message: header recv failed\r\n");
	        return false;
	    }

	    header[0] = (char)hdr[0];
	    header[1] = (char)hdr[1];
	    header[2] = (char)hdr[2];
	    header[3] = (char)hdr[3];
	    header[4] = '\0';

	    // 4-Byte Länge lesen (Little Endian)
	    if (Sensor_Recv(len_bytes, 4) != HAL_OK)
	    {
	        Debug_Print("read_message: length recv failed\r\n");
	        return false;
	    }

	    uint32_t len = (uint32_t)len_bytes[0]
	                 | ((uint32_t)len_bytes[1] << 8)
	                 | ((uint32_t)len_bytes[2] << 16)
	                 | ((uint32_t)len_bytes[3] << 24);

	    if (len > max_payload)
	    {
	        Debug_Print("read_message: payload too big\r\n");
	        return false;
	    }

	    if (len > 0)
	    {
	        if (Sensor_Recv(payload, (uint16_t)len) != HAL_OK)
	        {
	            Debug_Print("read_message: payload recv failed\r\n");
	            return false;
	        }
	    }

	    *payload_len = len;
	    return true;


   /* uint8_t hdr[4];
    uint8_t len_bytes[4];

    if (Sensor_Recv(hdr, 4) != HAL_OK)
        return false;

    header[0] = (char)hdr[0];
    header[1] = (char)hdr[1];
    header[2] = (char)hdr[2];
    header[3] = (char)hdr[3];
    header[4] = '\0';

    if (Sensor_Recv(len_bytes, 4) != HAL_OK)
        return false;

    uint32_t len = (uint32_t)len_bytes[0]
                 | ((uint32_t)len_bytes[1] << 8)
                 | ((uint32_t)len_bytes[2] << 16)
                 | ((uint32_t)len_bytes[3] << 24);

    if (len > max_payload)
        return false;

    if (len > 0)
        {
            if (Sensor_Recv(payload, (uint16_t)len) != HAL_OK)
                return false;
        }

        *payload_len = len;
        return true;*/
}

int send_command(const char *header, const uint8_t *payload, uint32_t payload_len)
{
    uint8_t hdr_len[8];
    uint8_t len_bytes[4];

    // 4-Byte Header
    hdr_len[0] = (uint8_t)header[0];
    hdr_len[1] = (uint8_t)header[1];
    hdr_len[2] = (uint8_t)header[2];
    hdr_len[3] = (uint8_t)header[3];

    // 4-Byte Länge (LE)
    len_bytes[0] = (uint8_t)(payload_len & 0xFF);
    len_bytes[1] = (uint8_t)((payload_len >> 8) & 0xFF);
    len_bytes[2] = (uint8_t)((payload_len >> 16) & 0xFF);
    len_bytes[3] = (uint8_t)((payload_len >> 24) & 0xFF);

    memcpy(&hdr_len[4], len_bytes, 4);

    Debug_Print("send INIT\r\n");

    if (Sensor_Send(hdr_len, 8) != HAL_OK)
            return -1;
    Debug_Print("INIT header sent\r\n");

        if (payload_len > 0)
        {
            if (Sensor_Send(payload, (uint16_t)payload_len) != HAL_OK)
                return -1;
        }
    Debug_Print("INIT payload sent\r\n");

        // RESP lesen
        char resp_header[5];
        uint8_t resp_payload[8];
        uint32_t resp_len = 0;

        if (!read_message(resp_header, resp_payload, &resp_len, sizeof(resp_payload))){
        	 Debug_Print("send_command: no RESP frame\r\n");
        	 return -1;
        }
        Debug_Print("got answer\r\n");

        if (strcmp(resp_header, "RESP") == 0 && resp_len >= 1)
            {
                uint8_t err = resp_payload[0];
                if (err != 0)
                {
                    char buf[80];
                    snprintf(buf, sizeof(buf), "ERROR: %u (%s)\r\n", err, get_error_description(err));
                    Debug_Print(buf);
                }
                return err;
            }

            return -1; // kein RESP
}

bool initialize_sensor(uint8_t baud_index)
{
	const uint32_t valid_baud[4] = {115200, 460800, 921600, 2000000};
	    if (baud_index > 3)
	        return false;



	    uint8_t payload = baud_index;
	    int err = send_command("INIT", &payload, 1);
	    if (err != 0)
	        return false;

	    Debug_Print("OK: INIT sent\r\n");


    // VERS lesen
	    char hdr[5];
	       uint8_t pld[64];
	       uint32_t len = 0;

	       if (read_message(hdr, pld, &len, sizeof(pld)))
	       {
	           if (strcmp(hdr, "VERS") == 0)
	           {
	               if (len < sizeof(pld))
	                   pld[len] = '\0';
	               char buf[80];
	               snprintf(buf, sizeof(buf), "FW: %s\r\n", (char *)pld);
	               Debug_Print(buf);
	           }
	           else
	           {
	               Debug_Print("Unexpected header instead of VERS\r\n");
	           }
	       }


            huart1.Init.BaudRate = valid_baud[baud_index];
            if (HAL_UART_Init(&huart1) != HAL_OK)
                return false;

            char buf2[80];
            snprintf(buf2, sizeof(buf2), "OK: Baud=%lu\r\n", (unsigned long)valid_baud[baud_index]);
            Debug_Print(buf2);

            return true;
}

bool get_next_frame_pdat_done(uint8_t bit_field)
{
    bit_field &= 0b00100111; // Maske wie im Python-Code

    uint8_t payload = bit_field;
    if (send_command("GNFD", &payload, 1) < 0)
        return false;

    uint8_t received_mask = 0;
    uint32_t start = HAL_GetTick();

    while (received_mask != bit_field)
    {
        if ((HAL_GetTick() - start) > 1000)
        {
            Debug_Print("ERROR: Timeout waiting for PDAT/DONE\r\n");
            return false;
        }

        char hdr[5];
        uint8_t pld[RX_BUFFER_SIZE];
        uint32_t len = 0;

        if (!read_message(hdr, pld, &len, sizeof(pld)))
            continue;

        // PDAT
        if ((bit_field & 0x04) && (strcmp(hdr, "PDAT") == 0))
        {
            uint32_t idx = 0;
            while ((len - idx) >= 6)
            {
                float distance;
                uint16_t mag_raw;

                memcpy(&distance, &pld[idx], 4);
                memcpy(&mag_raw, &pld[idx + 4], 2);

                float magnitude = (float)mag_raw / 100.0f;

                char buf[80];
                snprintf(buf, sizeof(buf),
                         "PDAT: Dist=%.3f m, Mag=%.2f dB\r\n",
                         distance, magnitude);
                Debug_Print(buf);

                idx += 6;
            }

            received_mask |= 0x04;
        }
        // DONE
        else if ((bit_field & 0x20) && (strcmp(hdr, "DONE") == 0))
        {
            if (len >= 4)
            {
                uint32_t frame_number =
                    (uint32_t)pld[0]
                    | ((uint32_t)pld[1] << 8)
                    | ((uint32_t)pld[2] << 16)
                    | ((uint32_t)pld[3] << 24);

                char buf[64];
                snprintf(buf, sizeof(buf), "DONE: Frame=%lu\r\n",
                         (unsigned long)frame_number);
                Debug_Print(buf);
            }
            received_mask |= 0x20;
        }
    }

    return true;
}

void disconnect_sensor(void)
	{
	    (void)send_command("GBYE", NULL, 0);
	    Debug_Print("Sensor disconnected\r\n");
	}

/*void send_init_raw(void)
{
    uint8_t msg[] = {
        0x49, 0x4E, 0x49, 0x54,   // 'I','N','I','T'
        0x01, 0x00, 0x00, 0x00,   // length = 1
        0x00                      // payload: baud index 0 = 115200
    };
    Sensor_Send(msg, sizeof(msg));
    Debug_Print("INIT raw sent\r\n");
}
void vld1_minimal_test(void)
{
    // huart1 muss vorher auf die gleiche UART-Konfiguration wie das FTDI eingestellt sein
    // (z.B. 115200 Baud, 8N1 oder 8E1 – passend zum PC-Terminal)

    uint8_t init_msg[] = {
        0x49, 0x4E, 0x49, 0x54,   // 'I','N','I','T'
        0x01, 0x00, 0x00, 0x00,   // length = 1
        0x00                      // payload: baud index 0 = 115200
    };

    // INIT-Frame senden
    Sensor_Send(init_msg, sizeof(init_msg));
    Debug_Print("INIT raw sent\r\n");

    // Alle empfangenen Bytes auf UART2 als Hex ausgeben
    uint8_t b;
    char buf[16];

    while (1)
    {
        HAL_StatusTypeDef st = Sensor_Recv(&b, 1);
        if (st == HAL_OK)
        {
            snprintf(buf, sizeof(buf), "%02X ", (unsigned int)b);
            Debug_Print(buf);
        }
        else if (st == HAL_TIMEOUT)
        {
            Debug_Print("\r\nRX timeout\r\n");
        }
        else
        {
            Debug_Print("\r\nRX error\r\n");
        }
    }
}*/

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
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  Debug_Print("Start...\r\n");

     if (!initialize_sensor(0)) // 3 -> 2 Mbaud
     {
         Debug_Print("INIT failed\r\n");
         //while (1) { HAL_Delay(500); }
     }

     Debug_Print("Press reset to stop\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


	  /*char hdr[5];
	      uint8_t pld[64];
	      uint32_t len = 0;

	      if (read_message(hdr, pld, &len, sizeof(pld)))
	      {
	          char buf[80];
	          snprintf(buf, sizeof(buf), "MSG: %s, len=%lu\r\n",
	                   hdr, (unsigned long)len);
	          Debug_Print(buf);
	      }
	      else
	      {
	          Debug_Print("read_message failed\r\n");
	      }

	      HAL_Delay(500);*/

	  get_next_frame_pdat_done(BITFIELD_PDAT_DONE);
	  HAL_Delay(00);

	  //HAL_UART_Transmit(&huart2, (uint8_t*)"TEST\r\n", 6, 1000);
	      //HAL_Delay(500);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
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
