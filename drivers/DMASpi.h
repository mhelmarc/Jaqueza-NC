/*
 * dmaspi.h
 *
 *  This file is part of Jaqueza-NC a Non-PRU port of Remora
 *  (https://github.com/scottalford75/Remora) a free, opensource LinuxCNC
 *  component and Programmable Realtime Unit (PRU)
 *
 *  Copyright (C) 2021
 *
 */
#ifndef DMASPI_H_
#define DMASPI_H_
#include "config/spi.h"
#include "spidma.h"
#include "configuration.h"
#include "remora.h"

using namespace stm32plus;

using vPFunction = void (*)(volatile rxData_t*);

template<typename TSpi>
class SPIDma {

  private:

    enum {
      BUFFER_SIZE = SPI_BUFF_SIZE,
    };

    TSpi *_spi;
    typedef typename TSpi::Parameters SpiParam_t;

    SpiDmaTxChannel_t _spi_txdma;
    SpiDmaRxChannel_t _spi_rxdma;
    DMAStreamCopier_t _stream_copier;

    volatile txData_t *_txData;
    volatile rxData_t *_rxData;
    volatile rxData_t *_RxBuffer;    // pointer to current buffer used
    volatile rxData_t spiRxBuffer1;  // this buffer is used to check for valid data before moving it to rxData
    volatile rxData_t spiRxBuffer2;  // this buffer is used to check for valid data before moving it to rxData

    void setRxTxDmaEnableBits(void);
    void onDmaTxInterrupt(DmaEventType ev);
    void onDmaRxInterrupt(DmaEventType ev);
    void onDmaStreamCopierInterrupt(DmaEventType ev);

    void disableDmaInterrupts(void);
    void enableDmaInterrupts(void);

    vPFunction _spi_DmaRxInterruptCallback;

  public:
    SPIDma(volatile txData_t*, volatile rxData_t*, vPFunction, SpiParam_t);
    ~SPIDma();

    volatile int32_t *buff1_header;
    volatile int32_t *buff2_header;

    void init(void);
    void enableStreamCopier(volatile rxData_t*);
    void disableStreamCopier(void);
    void disableRxDmaStream(void);
    void enableRxDmaStream(void);
    void swapBuffer(volatile rxData_t*);

};


// Constructor
template<typename TSpi>
SPIDma<TSpi>::SPIDma(volatile txData_t *txdata, volatile rxData_t *rxdata, vPFunction fp, SpiParam_t params) :
    _txData(txdata), _rxData(rxdata), _spi_DmaRxInterruptCallback(fp) {

  buff1_header = &spiRxBuffer1.data.header;
  buff2_header = &spiRxBuffer2.data.header;

  _spi = new TSpi(params);
  _spi->enablePeripheral();

  setRxTxDmaEnableBits();

  _spi_rxdma.setNvicPriorities(1, 0);
  _spi_txdma.setNvicPriorities(1, 0);
  _stream_copier.setNvicPriorities(1,0);

  /* Setup DMA transmit/receive interrupts */
  _spi_rxdma.DmaInterruptEventSender.insertSubscriber(
      DmaInterruptEventSourceSlot::bind(this, &SPIDma::onDmaRxInterrupt));

  _spi_txdma.DmaInterruptEventSender.insertSubscriber(
      DmaInterruptEventSourceSlot::bind(this, &SPIDma::onDmaTxInterrupt));

  _stream_copier.DmaInterruptEventSender.insertSubscriber(
      DmaInterruptEventSourceSlot::bind(this,
          &SPIDma::onDmaStreamCopierInterrupt));

  init();
}


template<typename TSpi>
SPIDma<TSpi>::~SPIDma() {
  delete _spi;
}

/* There seems to be a sequence for the DMA to work
 * If DMA streams is initialize first, the Spi RX/TX DMAEN bits doesn't get enabled.
 * If the peripheral is initialize first, the DMA FIFO interrupt error kicks in.
 * Since we're not using FIFO mode I guess it's OK ??
 * The library does not have a provision to explicitly enable RX/TX DMAEN bits
 *
 * Learnt about conversion operator.
 * The Spi class has operator that return the address of the peripheral
 * So we don't have to hardcode the Spi number
 */
template<typename TSpi>
inline void SPIDma<TSpi>::setRxTxDmaEnableBits(void) {
  SPI_I2S_DMACmd(*_spi,(SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN), ENABLE);
}


template<typename TSpi>
void SPIDma<TSpi>::init(void) {

  disableDmaInterrupts();

  _stream_copier.disableStream();
  _spi_txdma.disableStream();
  _spi_rxdma.disableStream();

  _RxBuffer = &spiRxBuffer1;

  /* Streams are enabled with these calls */

  _spi_txdma.beginWrite(const_cast<uint8_t*>(_txData->tx.buffer), BUFFER_SIZE);
  _spi_rxdma.beginRead(const_cast<uint8_t*>(_RxBuffer->rx.buffer), BUFFER_SIZE);

  enableDmaInterrupts();

  /* Clear error flags */
  _spi_txdma.clearErrorFlag();
  _spi_rxdma.clearErrorFlag();
  _stream_copier.clearErrorFlag();

}

/*
 *  Start memory to memory copy
 */
template<typename TSpi>
void SPIDma<TSpi>::enableStreamCopier(volatile rxData_t *src) {
  _stream_copier.beginCopyMemory(const_cast<uint8_t*>(_rxData->rx.buffer),
      const_cast<uint8_t*>(src->rx.buffer), BUFFER_SIZE, DMA_Priority_Medium);
  enableRxDmaStream();
}


/* Disable the memory copy DMA stream */
template<typename TSpi>
void SPIDma<TSpi>::disableStreamCopier(void) {
  _stream_copier.disableStream();
}


/* Disable the receive DMA stream */
template<typename TSpi>
void SPIDma<TSpi>::disableRxDmaStream(void) {
  _spi_rxdma.disableStream();
}


/* Enable the receive DMA stream */
template<typename TSpi>
inline void SPIDma<TSpi>::enableRxDmaStream(void) {
  _spi_rxdma.enableStream();
}


/* Enable interrupts */
template<typename TSpi>
inline void SPIDma<TSpi>::enableDmaInterrupts(void) {
  _spi_txdma.enableInterrupts(SpiDmaRxTransferComplete);
  _spi_rxdma.enableInterrupts(SpiDmaTxTransferComplete);
  _stream_copier.enableInterrupts(StreamCopierTransferComplete);
}


/* Disable interrupt */
template<typename TSpi>
inline void SPIDma<TSpi>::disableDmaInterrupts(void) {
  _spi_txdma.disableInterrupts(SpiDmaRxTransferComplete);
  _spi_rxdma.disableInterrupts(SpiDmaTxTransferComplete);
  _stream_copier.disableInterrupts(StreamCopierTransferComplete);
}


/* This is called from the callback function in main */
template<typename TSpi>
void SPIDma<TSpi>::swapBuffer(volatile rxData_t* curbuf) {
  if (curbuf == &spiRxBuffer1){
    _RxBuffer = &spiRxBuffer2;
  }
  else if (curbuf == &spiRxBuffer2) {
    _RxBuffer = &spiRxBuffer1;
  }
}


/** DMA Receive interrupt
 */
template<typename TSpi>
void SPIDma<TSpi>::onDmaRxInterrupt(DmaEventType ev) {

  if (ev == DmaEventType::EVENT_COMPLETE) {

    /* call the function in main */
    _spi_DmaRxInterruptCallback(_RxBuffer);

    /* Interrupts automatically disables the streams
     * it is re-initialized when beginread()is called ,
     */
    _spi_rxdma.clearCompleteFlag();
    _spi_rxdma.beginRead(const_cast<uint8_t*>(_RxBuffer->rx.buffer), BUFFER_SIZE);
  }

}


/* DMA Transmit interrupt */
template<typename TSpi>
void SPIDma<TSpi>::onDmaTxInterrupt(DmaEventType ev) {

  if (ev == DmaEventType::EVENT_COMPLETE) {

    _spi_txdma.clearCompleteFlag();
    _spi_txdma.beginWrite(const_cast<uint8_t*>(_txData->tx.buffer),
        BUFFER_SIZE);

  }

}


/* DMA Stream copier one */
template<typename TSpi>
void SPIDma<TSpi>::onDmaStreamCopierInterrupt(DmaEventType ev) {
  if (ev == DmaEventType::EVENT_COMPLETE) {
    _stream_copier.clearCompleteFlag();
  }
}


#endif /* DMASPI_H_ */
