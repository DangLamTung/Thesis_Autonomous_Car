#include "stm32f4xx_hal.h"
#include "lwrb.h"
typedef uint8_t  u8;

typedef uint16_t u16;

typedef uint32_t u32;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
#define UART_TX_RINGBUFF_SZ 512
#define UART_TX_MAX_MESSAGE_LEN 256

u8 txBuf[UART_TX_RINGBUFF_SZ],txLen;
lwrb_t txRing;

void UARTTXInit(void) {
	lwrb_init(&txRing,txBuf,UART_TX_RINGBUFF_SZ);
}

void UARTTxData(void) {
  if(txLen) return; //If len > 0, DMA transfer is on-going. This function will be called again at transfer completion
  txLen=lwrb_get_linear_block_read_length(&txRing); //Get maximal length of buffer to read data as linear memory
  if(txLen){
   void* ringData=lwrb_get_linear_block_read_address(&txRing); // Get pointer to read memory
   HAL_UART_Transmit_DMA(&huart3,(uint8_t*) ringData,txLen); // Start DMA transfer
  }
}

void UARTAddToTxBuff(const void *data,u8 len) {
	lwrb_write(&txRing,data,len);
  UARTTxData();
}



void UARTTxComplete(void) {
  if (txLen) {
   lwrb_skip(&txRing,txLen); // Now skip the data (move read pointer) as they were successfully transferred over DMA
   txLen=0; // Reset length = DMA is not active
   UARTTxData(); // Try to send more
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3) {
   CLEAR_BIT(huart->Instance->CR1, (USART_CR1_TXEIE | USART_CR1_TCIE)); // Disable TXEIE and TCIE interrupts
   huart->gState = HAL_UART_STATE_READY; // Tx process is ended, restore huart->gState to Ready
   UARTTxComplete();
  }

}


//#define UART_RX_RINGBUFF_SZ 128
//#define UART_RX_MAX_MESSAGE_LEN 50
//#define UART_DMA_WRITE_PTR ((UART_RX_RINGBUFF_SZ - huart1.hdmarx->Instance->NDTR) & (UART_RX_RINGBUFF_SZ - 1))
//u8 processBuf[UART_RX_MAX_MESSAGE_LEN];
//
//char rxBuf[UART_RX_RINGBUFF_SZ],rxLen;
//lwrb_t rxRing;
//u16 rxLastPos,rxThisPos;
//
//void UARTRXInit(void) {
//  lwrb_init(&rxRing,rxBuf,UART_RX_RINGBUFF_SZ);
//  rxLastPos=0;
//  rxThisPos=0;
//  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);   // enable idle line interrupt
//  hdma_usart1_rx.Instance->CR &= ~DMA_SxCR_HTIE;  // disable uart half tx interrupt
//  HAL_UART_Receive_DMA(&huart1,rxBuf,UART_RX_RINGBUFF_SZ);
//}
//
//void UARTRxComplete(void) {
//  u8 addr;
//  u16 len;
//  rxThisPos=UART_DMA_WRITE_PTR; //get current write pointer
//  len=(rxThisPos-rxLastPos+UART_RX_RINGBUFF_SZ)%UART_RX_RINGBUFF_SZ; //calculate how far the DMA write pointer has moved
//  if(len<=UART_RX_MAX_MESSAGE_LEN) { //check message size
//   lwrb_advance(&rxRing,len); //move the ring buffer write pointer
//   rxLastPos=rxThisPos;
//   lwrb_read(&rxRing,processBuf,len); //read out the data to an array for processing
//  }
//  else {
////    while(1); //implement message to large exception
//  }
//}
