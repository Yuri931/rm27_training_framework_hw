#include "bsp_usart.hpp"

static uint8_t usart1RxBuffer[256];
static uint8_t usart6RxBuffer[256];


void USART_Init(void)
{
  USART1_Init();
  USART6_Init();
}

void USART6_Init()
{
  if(HAL_UARTEx_ReceiveToIdle_DMA(&huart6,usart6RxBuffer,sizeof(usart6RxBuffer))!=HAL_OK)
  {
    Error_Handler();
  }

  // TODO: 配置 USART6 的 DMA 接收、发送及空闲中断，并启动接收。
};

void USART1_Init()
{
  if(HAL_UARTEx_ReceiveToIdle_DMA(&huart1,usart1RxBuffer,sizeof(usart1RxBuffer))!=HAL_OK)
  {
    Error_Handler();
  }
  // TODO: 配置 USART1 的 DMA 接收、发送及空闲中断，并启动接收。
};

void USART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, enum USART_Mode mode)
{
  if(huart==nullptr||pData==nullptr||Size==0)
  {
    return;
  }
  //检查参数
  HAL_StatusTypeDef status;
  switch(mode)
  {
    case USART_MODE_BLOCK:
    {
      status=HAL_UART_Transmit(huart,pData,Size,100);
      break;
    }
    case USART_MODE_DMA:
    {
      status=HAL_UART_Transmit_DMA(huart,pData,Size);
      break;
    }
    case USART_MODE_IT:
    {
      status=HAL_UART_Transmit_IT(huart,pData,Size);
      break;
    }
    default: 
    {
      return;
    }
  }
  if(status!=HAL_OK)
  {
    Error_Handler();
  }

  // TODO: 检查参数，并按 mode 选择阻塞、DMA 或中断发送。
  (void)huart;
  (void)pData;
  (void)Size;
  (void)mode;
}

void USART_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
  //统一用DMA接收
    if(huart==nullptr||pData==nullptr||Size==0)
    {
      return;
    }
    if(HAL_UART_Receive_DMA(huart,pData,Size)!=HAL_OK)
    {
      Error_Handler();
    }

  // TODO: 选择接收方式并启动 HAL UART 接收。
  (void)huart;
  (void)pData;
  (void)Size;
}
