/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include "pqueue.h"
#include "graph.h"


//#define SAMPLE_SIZE 32
#define PI 3.14159265358979323846
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

extern ADC_HandleTypeDef hadc1;



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_ADC1_Init(void);
void Poll_ADC_Samples(struct Complex coefs[], const size_t size);
struct Complex complex_multiply(struct Complex a, struct Complex b);
struct Complex complex_add(struct Complex a, struct Complex b);
struct Complex complex_subtract(struct Complex a, struct Complex b);
void FFT(struct Complex *x, const size_t N);
void Compute_Magnitudes(struct Complex coefs[], float magbuffer[], const size_t size);

/* USER CODE BEGIN PFP */
static struct Complex coefs[SAMPLE_SIZE];
static float magbuffer[SAMPLE_SIZE];
static struct priority_queue pq;
static struct Graph graph;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

	Poll_ADC_Samples(coefs, SAMPLE_SIZE);
  /* USER CODE END 2 */
	for(int i=0; i < SAMPLE_SIZE; i++) {
		coefs[i].imag = 0.0;
	}
	
	FFT(coefs, SAMPLE_SIZE);
	Compute_Magnitudes(coefs, magbuffer, SAMPLE_SIZE);
	
	priority_queue_from_array(&pq, magbuffer, SAMPLE_SIZE);
	priority_queue_build_max_heap(&pq);

	create_graph(&graph, SAMPLE_SIZE);
	for(int i=0; i < pq.heap_size; i++) {
		add_vertex(&graph, pq.heap[i]);
	}
	
	for (int i = 1; i < graph.graph_capacity - 1; i++) {
    float prev_val = graph.vertices[i - 1].magnitude;  // Assume `magnitude` is a field in each vertex
    float curr_val = graph.vertices[i].magnitude;
    float next_val = graph.vertices[i + 1].magnitude;

    // Check for peak
    if (curr_val > prev_val && curr_val > next_val) {
        // Add edge between current peak and its previous valley
        add_edge(&graph, i, i - 1);
        // Add edge between current peak and its next valley
        add_edge(&graph, i, i + 1);
    }
    // Check for valley
    else if (curr_val < prev_val && curr_val < next_val) {
        // Add edge between current valley and its previous peak
        add_edge(&graph, i, i - 1);
        // Add edge between current valley and its next peak
        add_edge(&graph, i, i + 1);
    }
	}
	
	bfs(&graph, 3);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}
/* USER CODE BEGIN 4 */
void Poll_ADC_Samples(struct Complex samples[], const size_t size)
{
    for (int i = 0; i < size; i++)
    {
        // Start the ADC conversion
        HAL_ADC_Start(&hadc1);

        // Poll for conversion to complete
        HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);

        // Get the ADC value and store it in the buffer (real part)
        uint32_t adcValue = HAL_ADC_GetValue(&hadc1);
        samples[i].real = (float)adcValue; // Real part of the complex number

        // Stop ADC (optional if using single conversion mode)
        HAL_ADC_Stop(&hadc1);
    }

    // Now you can pass `samples_real` and `samples_imag` to your FFT function.
}

// Function to multiply two complex numbers
struct Complex complex_multiply(struct Complex a, struct Complex b) {
    struct Complex result;
    result.real = (a.real * b.real) - (a.imag * b.imag);
    result.imag = (a.real * b.imag) + (a.imag * b.real);
    return result;
}

// Function to add two complex numbers
struct Complex complex_add(struct Complex a, struct Complex b) {
    struct Complex result;
    result.real = a.real + b.real;
    result.imag = a.imag + b.imag;
    return result;
}

// Function to subtract two complex numbers
struct Complex complex_subtract(struct Complex a, struct Complex b) {
    struct Complex result;
    result.real = a.real - b.real;
    result.imag = a.imag - b.imag;
    return result;
}

// Recursive FFT function
void FFT(struct Complex *x, const size_t size) {
    if (size <= 1) return;

    // Divide the array into two halves
	
	 struct Complex even[size/2];
   struct Complex odd[size/2];
    
    for (int i = 0; i < size / 2; i++) {
        even[i] = x[i * 2];
        odd[i] = x[i * 2 + 1];
    }

    // Recursive FFT on both halves
    FFT(even, size / 2);
    FFT(odd, size / 2);

    // Combine
    for (size_t k = 0; k < size / 2; k++) {
        struct Complex t;
        float angle = -2 * PI * k / size;
        struct Complex twiddle = {cos(angle), sin(angle)};
        t = complex_multiply(twiddle, odd[k]);
        x[k] = complex_add(even[k], t);
        x[k + size / 2] = complex_subtract(even[k], t);
    }
		
}
void Compute_Magnitudes(struct Complex coefs[], float magbuffer[], const size_t size) {
    for (int i = 0; i < size; i++) {
        magbuffer[i] = (sqrtf(coefs[i].real * coefs[i].real + coefs[i].imag * coefs[i].imag));
    }
}



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
