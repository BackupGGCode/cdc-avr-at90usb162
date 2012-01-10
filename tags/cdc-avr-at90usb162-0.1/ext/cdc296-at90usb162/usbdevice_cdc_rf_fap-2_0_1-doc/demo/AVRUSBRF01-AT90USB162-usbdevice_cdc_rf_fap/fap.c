/* Copyright (c) 2007 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is confidential property of Nordic Semiconductor. The use,
 * copying, transfer or disclosure of such information is prohibited except by express written
 * agreement with Nordic Semiconductor.
 *
 * $Rev: 1896 $
 *
 */
                                                 
/** @file
 * Implementation of frequency agility protocol for better robustness against
 * interference from co-existing radio frequency sources.  
 * 
 * @author Lasse Olsen
 *
 * @compiler This program has been tested with <compiler name>
 */

#include "fap.h"
#include <string.h>
#include "FAPtoNRF_API.h"

/*
#define FAP_INTERRUPTS_DISABLE() do{FAP_NRF_IRQ_DISABLE();\
                                    FAP_TIMER_IRQ_DISABLE();\
                                    FAP_NRF_IRQ_DISABLE(); }\
                                    while(0)

#define FAP_INTERRUPTS_ENABLE() do{FAP_NRF_IRQ_ENABLE();\
                                   FAP_TIMER_IRQ_ENABLE();\
                                  }\
                                  while(0)
*/
#define FAP_INTERRUPTS_DISABLE() (FAP_NRF_IRQ_DISABLE(), FAP_TIMER_IRQ_DISABLE(), FAP_NRF_IRQ_DISABLE())

#define FAP_INTERRUPTS_ENABLE()  (FAP_NRF_IRQ_ENABLE(), FAP_TIMER_IRQ_ENABLE())

static uint8_t fap_lfsr_get(uint8_t seed, uint8_t max_limit);
static void fap_set_system_idle(void);

static xdata uint8_t fap_ch_tab[FAP_CH_TAB_SIZE];
static xdata volatile uint8_t fap_rx_channel_guess; 
static xdata uint8_t fap_p0_adr[FAP_ADDRESS_WIDTH];
static xdata uint8_t fap_curr_tx_setup = 0xff;

// Internal function varaiables, efficient for compilers using compiled stack 
static xdata fap_tx_rx_struct_t fap_general_buffer; 
 
static xdata uint8_t fap_general_byte;

// Fap status register definitions
typedef enum
{
  FAP_START_NEW_TX,
  FAP_TX_RX_INFINITE_TIMEOUT_EN,
  FAP_LP_RX_EN,
  FAP_CH_SYNC_EN,
  FAP_PDOWN_IDLE_EN,
  FAP_TX_SUCCESS,
  FAP_RX_CH_HOLD,
  FAP_RADIO_PWR_UP         
} fap_status_reg_mask;

static xdata uint8_t fap_status;

static xdata volatile uint8_t fap_radio_startup_counter;
static xdata volatile int16_t fap_latency_counter, fap_latency_setup;
static xdata volatile uint8_t fap_retry_extend_counter;

static xdata volatile uint16_t fap_try_counter;
static xdata volatile uint16_t fap_timer_period;
static xdata volatile fap_modes_t fap_mode;

// Status variables
static xdata uint8_t fap_ch_tab_idx_tx;
static xdata uint8_t fap_ch_tab_idx_rx;
static xdata uint8_t fap_hold_rx_channel;

// For transmission statistics
static xdata uint16_t fap_ch_switch_counter;

//static xdata fap_tx_rx_struct_t fap_rx_buffer; 
//static bool fap_data_in_rx_buffer = false; 

//----------------------------------------------------------------------------- 
// Function bodies, application
//----------------------------------------------------------------------------- 

void fap_init(void)
{
  FAP_INTERRUPTS_DISABLE();

  fap_mode=FAP_IDLE;

  CE_LOW();
  
  // Activate hidden features
  if(hal_nrf_read_reg(FEATURE) != 0x06 || (hal_nrf_read_reg(DYNPD) != 0x3F))
  {
    hal_nrf_lock_unlock();
    hal_nrf_enable_ack_pl();
    hal_nrf_enable_dynamic_pl();
    hal_nrf_setup_dyn_pl(0xff);   // Use dynamic PL for all pipes
  }

  fap_ch_tab_idx_tx=0;
  fap_ch_tab_idx_rx=0;
  fap_hold_rx_channel=0;
  fap_timer_period=0; 
  fap_rx_channel_guess = 0; 
  
  // Default setup that later may be altered by application
  FAP_SET_BIT(fap_status, FAP_PDOWN_IDLE_EN);
  FAP_CLEAR_BIT(fap_status, FAP_LP_RX_EN);
  FAP_CLEAR_BIT(fap_status, FAP_CH_SYNC_EN);
  FAP_CLEAR_BIT(fap_status, FAP_RADIO_PWR_UP);
  FAP_SET_BIT(fap_status, FAP_TX_SUCCESS);

  // Default static setup
  hal_nrf_set_datarate(FAP_DATARATE);    
  hal_nrf_set_auto_retr(FAP_NRF_AUTO_RETRIES, FAP_AUTO_RETR_DELAY); 
  hal_nrf_set_output_power(FAP_OUTPUT_POWER); 
  hal_nrf_set_crc_mode(FAP_CRC);   
  hal_nrf_set_address_width(FAP_ADDRESS_WIDTH);
  
  hal_nrf_get_clear_irq_flags();
  
  hal_nrf_flush_rx();
  hal_nrf_flush_tx();
  
  FAP_INTERRUPTS_ENABLE();
}  

void fap_set_channels(uint8_t *channels)
{
  FAP_INTERRUPTS_DISABLE();

  memcpy(fap_ch_tab, channels, FAP_CH_TAB_SIZE);
  hal_nrf_set_rf_channel(fap_ch_tab[fap_ch_tab_idx_tx]); 

  FAP_INTERRUPTS_ENABLE();
}

bool fap_set_address(uint8_t dev, uint8_t* adr)
{ 
  if(fap_mode==FAP_IDLE)
  {
    FAP_INTERRUPTS_DISABLE();
  
    fap_curr_tx_setup = 0xff;
  
    if(dev == HAL_NRF_PIPE0)
    {
      memcpy(fap_p0_adr, adr, FAP_ADDRESS_WIDTH);
    } 
    hal_nrf_set_address(dev, adr);
    
    FAP_INTERRUPTS_ENABLE();
   
    return true;
  }
  else
  {
    return false;
  }
}

void fap_get_address(uint8_t dev, uint8_t* adr)
{
  FAP_INTERRUPTS_DISABLE();

  hal_nrf_get_address(dev, adr);
    
  FAP_INTERRUPTS_ENABLE();
}

void fap_rx_data(uint8_t rx_setup, uint16_t rx_timeout)
{
  fap_goto_idle_mode();

  fap_curr_tx_setup = 0xff; // Signals that TX setup must be restored at next transmission 

  if(rx_setup & (0x3f))     // Check if any pipe is enabeled for receiving  
  {
    FAP_INTERRUPTS_DISABLE();
  
    hal_nrf_set_address(HAL_NRF_PIPE0, fap_p0_adr);  
  
    hal_nrf_close_pipe(HAL_NRF_ALL); 
    for(fap_general_byte = 0; fap_general_byte < 6; fap_general_byte++)
    {
      if(rx_setup & (1 << fap_general_byte))
      {
        hal_nrf_open_pipe(fap_general_byte, EN_AA);  
      }
    }
  
    if(FAP_GET_BIT(rx_setup, FAP_LP_RX_BIT))
    {
      FAP_SET_BIT(fap_status, FAP_LP_RX_EN);  
    }
    else
    {
      FAP_CLEAR_BIT(fap_status, FAP_LP_RX_EN);    
    }

    hal_nrf_set_power_mode(HAL_NRF_PWR_UP);
    FAP_SET_BIT(fap_status, FAP_RADIO_PWR_UP);

    hal_nrf_set_operation_mode(HAL_NRF_PRX);
      
    if(rx_timeout==0)
    {
      FAP_SET_BIT(fap_status, FAP_TX_RX_INFINITE_TIMEOUT_EN);
    }
    else
    {
      FAP_CLEAR_BIT(fap_status, FAP_TX_RX_INFINITE_TIMEOUT_EN);
      fap_latency_counter=rx_timeout;   
    }
  
    fap_timer_period=0;
    fap_mode=FAP_RECEIVING;
  
    CE_HIGH();
    
    FAP_INTERRUPTS_ENABLE(); 
  }
}

void fap_set_output_power(hal_nrf_output_power_t power)
{
  FAP_INTERRUPTS_DISABLE();

  hal_nrf_set_output_power(power);

  FAP_INTERRUPTS_ENABLE();
}

bool fap_tx_data(fap_tx_rx_struct_t *datainput, uint16_t tx_timeout)
{

  if(datainput -> pl_length > FAP_MAX_FW_PL_LENGTH)
  {
    return false;
  }

  FAP_INTERRUPTS_DISABLE();

  if(fap_mode != FAP_TRANSMITTING)              // Thus IDLE or RECEIVING
  {
    if(fap_mode == FAP_RECEIVING)
    {
      fap_goto_idle_mode();  
    }
   
    // If TX setup modified
    if(fap_curr_tx_setup != (datainput -> pipe))
    {  
      hal_nrf_set_operation_mode(HAL_NRF_PTX);  
      hal_nrf_open_pipe(HAL_NRF_PIPE0, EN_AA);
  
      if(datainput -> pipe == FAP_DEVICE0)
      {
        hal_nrf_set_address(HAL_NRF_TX, fap_p0_adr);
        hal_nrf_set_address(HAL_NRF_PIPE0, fap_p0_adr);
      }
      else
      { 
        hal_nrf_get_address(HAL_NRF_PIPE1, &fap_general_buffer.pl[0]);

        if(datainput -> pipe != FAP_DEVICE1)
        {
          switch(datainput -> pipe)
          {
            default:
            case FAP_DEVICE2:
              hal_nrf_get_address(HAL_NRF_PIPE2, &fap_general_buffer.pl[0]);          
              break;
            case FAP_DEVICE3:
              hal_nrf_get_address(HAL_NRF_PIPE3, &fap_general_buffer.pl[0]);       
              break;
            case FAP_DEVICE4:
              hal_nrf_get_address(HAL_NRF_PIPE4, &fap_general_buffer.pl[0]);
              break;
            case FAP_DEVICE5:
              hal_nrf_get_address(HAL_NRF_PIPE5, &fap_general_buffer.pl[0]);
              break;
          }
        }
        hal_nrf_set_address(HAL_NRF_PIPE0, &fap_general_buffer.pl[0]);
        hal_nrf_set_address(HAL_NRF_TX, &fap_general_buffer.pl[0]);
      }    
      fap_curr_tx_setup = datainput -> pipe;    
    }
    fap_ch_switch_counter=0; 
    fap_retry_extend_counter=0;
    fap_try_counter=0;
 
    if(tx_timeout==0)
    {
      FAP_SET_BIT(fap_status, FAP_TX_RX_INFINITE_TIMEOUT_EN);
    }
    else
    {
      fap_latency_counter = fap_latency_setup = tx_timeout;
      FAP_CLEAR_BIT(fap_status, FAP_TX_RX_INFINITE_TIMEOUT_EN);
    }

    hal_nrf_flush_tx();
    hal_nrf_write_tx_pload(&(datainput -> pl[0]), datainput -> pl_length);

    FAP_SET_BIT(fap_status, FAP_TX_SUCCESS);      // Transmission by default "success"
  
    if(FAP_GET_BIT(fap_status, FAP_RADIO_PWR_UP) && !FAP_GET_BIT(fap_status, FAP_CH_SYNC_EN) )
    {  
      CE_HIGH();           
    }
    else
    {
      if(!FAP_GET_BIT(fap_status, FAP_RADIO_PWR_UP))
      {
        hal_nrf_set_power_mode(HAL_NRF_PWR_UP);
        FAP_SET_BIT(fap_status, FAP_RADIO_PWR_UP);
        fap_radio_startup_counter = FAP_RADIO_PWR_UP_DELAY;
      }

      FAP_SET_BIT(fap_status, FAP_START_NEW_TX);
    }

    fap_mode=FAP_TRANSMITTING;
    
     FAP_INTERRUPTS_ENABLE();
     return true;
  } 
  else // Check if criteria for starting new TX when in TX mode is fulfilled
  if(!FAP_GET_BIT(fap_status, FAP_CH_SYNC_EN) && \
          fap_curr_tx_setup == (datainput -> pipe) && \
          !hal_nrf_tx_fifo_full() && \
          tx_timeout == fap_latency_setup \
  )       
  {
     hal_nrf_write_tx_pload(&(datainput -> pl[0]), datainput -> pl_length);

    FAP_INTERRUPTS_ENABLE();
    return true;
  }
  else
  {
    FAP_INTERRUPTS_ENABLE();
    return false;
  }
}

void fap_flush_tx_fifo(void)
{
  FAP_INTERRUPTS_DISABLE();

  hal_nrf_flush_tx();

  FAP_INTERRUPTS_ENABLE();
}

bool fap_write_ack_pload(fap_tx_rx_struct_t* ackdata)
{
  FAP_INTERRUPTS_DISABLE();

  if(!hal_nrf_tx_fifo_full() && !(ackdata -> pl_length > FAP_MAX_ACK_PL_LENGTH))
  {
    hal_nrf_write_ack_pload(ackdata -> pipe, &(ackdata -> pl[0]), ackdata -> pl_length);
    FAP_INTERRUPTS_ENABLE();
    return true;
  }
  else
  {
    FAP_INTERRUPTS_ENABLE();
    return false;
  }
}

fap_modes_t fap_get_mode(void)
{
  return fap_mode;
}

bool fap_read_rx_fifo(fap_tx_rx_struct_t* return_struct)
{  
  volatile uint16_t tempvalue;

  FAP_INTERRUPTS_DISABLE(); 
  
  if(!hal_nrf_rx_fifo_empty())
  {
    tempvalue = hal_nrf_read_rx_pload(&(return_struct -> pl[0])); 
    FAP_INTERRUPTS_ENABLE();          
    
    return_struct -> pipe=MSB(tempvalue);   // MSByte, pipe    
    return_struct -> pl_length=LSB(tempvalue); // LSByte=length  
  /*  return_struct -> pipe=(uint8_t)(tempvalue>>8);   // MSByte, pipe    
    return_struct -> pl_length=(uint8_t)(tempvalue); // LSByte=length  */
   
    return true;
  }
  else
  {
    FAP_INTERRUPTS_ENABLE();  
    return false;
  }
}

bool fap_tx_success(void)
{
  return FAP_GET_BIT(fap_status, FAP_TX_SUCCESS);
}

bool fap_select_radio_idle_mode(fap_radio_idle_modes_t mode)
{
  if(fap_mode==FAP_IDLE)
  {
    FAP_INTERRUPTS_DISABLE();
 
    if(mode == FAP_PDOWN_IDLE)
    {
      FAP_SET_BIT(fap_status, FAP_PDOWN_IDLE_EN);
  
      if(fap_mode==FAP_IDLE)
      {
        hal_nrf_set_power_mode(HAL_NRF_PWR_DOWN);
        FAP_CLEAR_BIT(fap_status, FAP_RADIO_PWR_UP);
      }
    }
    else
    {
      FAP_CLEAR_BIT(fap_status, FAP_PDOWN_IDLE_EN);  
    
      // Update radio power mode if FAP is in idle mode
      hal_nrf_set_power_mode(HAL_NRF_PWR_UP);
      FAP_SET_BIT(fap_status, FAP_RADIO_PWR_UP);
    }
    
    FAP_INTERRUPTS_ENABLE();
    
    return true;
  }
  else
  {
    return false;
  }
}

uint16_t fap_get_tries(void)
{
  return fap_try_counter;
}

uint16_t fap_get_ch_switches(void)
{
  return  fap_ch_switch_counter;
}

void fap_ch_sync_enable()
{ 
  FAP_INTERRUPTS_DISABLE();

  FAP_SET_BIT(fap_status, FAP_CH_SYNC_EN);

  FAP_INTERRUPTS_ENABLE();
}

void fap_ch_sync_disable()
{
  FAP_INTERRUPTS_DISABLE();

  FAP_CLEAR_BIT(fap_status, FAP_CH_SYNC_EN);

  FAP_INTERRUPTS_ENABLE();
}
 
uint8_t fap_get_ch_offset()
{
  int8_t sync_var;
  
  FAP_INTERRUPTS_DISABLE();
  
  sync_var = (fap_ch_tab_idx_tx - fap_rx_channel_guess); 
  
  if(FAP_GET_BIT(fap_status, FAP_PDOWN_IDLE_EN))
  {
    sync_var -= FAP_RADIO_PWR_UP_DELAY; 
  }
   
  if(sync_var < 0)
  {
    sync_var += FAP_CH_TAB_SIZE;
  }
  
  FAP_INTERRUPTS_ENABLE();  
  
  return sync_var;   
}

void fap_nrf_isr_function(void)
{
  static uint8_t random_seed;

  FAP_TIMER_IRQ_DISABLE();                      

  fap_general_byte=hal_nrf_get_clear_irq_flags(); // Get/clear IRQ flags
  
  if(fap_mode != FAP_IDLE)
  {
    if(fap_general_byte & ((1<<NRF_MAX_RT)))
    {
      fap_try_counter += (FAP_NRF_AUTO_RETRIES+1);
        
      fap_retry_extend_counter= (fap_retry_extend_counter+1) % FAP_AUTO_RETRIES_EXTEND;
    
      if(fap_retry_extend_counter==0)
      {
        if(((fap_ch_switch_counter+1)%FAP_CH_TAB_SIZE) == 0)
        { 
          fap_ch_tab_idx_tx = fap_lfsr_get(random_seed, FAP_CH_TAB_SIZE);  
        }
        else
        {
          fap_ch_tab_idx_tx = fap_lfsr_get(0, FAP_CH_TAB_SIZE);    
        }   
        
        CE_LOW();                      
        hal_nrf_set_rf_channel(fap_ch_tab[fap_ch_tab_idx_tx]);                  
        fap_ch_switch_counter++;
      }

      fap_latency_counter -= (FAP_NRF_AUTO_RETRIES+1); 
      if(fap_latency_counter<1 && !FAP_GET_BIT(fap_status, FAP_TX_RX_INFINITE_TIMEOUT_EN))    
      {
        FAP_CLEAR_BIT(fap_status, FAP_TX_SUCCESS); 
        fap_set_system_idle();
      }
      else
      {
        CE_HIGH();                     
      }
    }
    else
    {
      if(fap_general_byte & ((1<<NRF_TX_DS)))
      {
         if(fap_mode==FAP_TRANSMITTING) 
        { 
       
           // Adjusts "guess counter" 
          fap_rx_channel_guess = (fap_ch_tab_idx_tx + 1) % FAP_CH_TAB_SIZE;                  
          fap_modify_timer_period();
        
          // Get number of tries. Add 1 for 1st attempt.                   
          fap_try_counter += (hal_nrf_get_transmit_attempts()+1);  
        
          // Get "random" seed fom number of sent packages
          // Used when altering channel search order
          random_seed = random_seed+fap_try_counter;  
             
          FAP_SET_BIT(fap_status, FAP_TX_SUCCESS);
       
          if(hal_nrf_tx_fifo_empty())
          {
            fap_set_system_idle();
          }
          else
          {
            fap_retry_extend_counter = 0;
            fap_try_counter = 0;
            fap_latency_counter = fap_latency_setup;
          }
                                             
          // Get "random" seed fom number of sent packages
          // Used when altering channel search order             
        } 
      }

      if(fap_general_byte & ((1<<NRF_RX_DR)))
      {
        fap_hold_rx_channel = FAP_RX_CH_HOLD_PERIODS;  // Applies in RX mode only
      }
    }     
  }
  
  FAP_TIMER_IRQ_ENABLE();  
}

void fap_timer_isr_function(void)
{
  FAP_NRF_IRQ_DISABLE();                    
  
  if(fap_mode==FAP_TRANSMITTING) 
  { 
    if(fap_radio_startup_counter>0)      
    {
      fap_radio_startup_counter--;
    }
    else
    if(FAP_GET_BIT(fap_status, FAP_START_NEW_TX))
    {
      CE_HIGH();                             
      FAP_CLEAR_BIT(fap_status, FAP_START_NEW_TX);
    }     
  }
  else  
  if(fap_mode==FAP_RECEIVING)
  {   
    if(FAP_GET_BIT(fap_status, FAP_TX_RX_INFINITE_TIMEOUT_EN) || (!FAP_GET_BIT(fap_status, FAP_TX_RX_INFINITE_TIMEOUT_EN) && fap_latency_counter>0))
    {     
      fap_latency_counter--;
      if(!FAP_GET_BIT(fap_status, FAP_LP_RX_EN))    
      {  
        fap_ch_tab_idx_rx = (fap_ch_tab_idx_rx+1) % FAP_CH_TAB_SIZE;             
          
        if(fap_hold_rx_channel == 0)
        {     
          CE_LOW();
          hal_nrf_set_rf_channel(fap_ch_tab[fap_ch_tab_idx_rx]);   
          CE_HIGH();
        }
        else
        {
          fap_hold_rx_channel--;
        } 
      } 
      else
      {
        if(fap_timer_period==0)
        {
          // Change channel
          if(fap_timer_period==0) 
          {
            fap_ch_tab_idx_rx = (fap_ch_tab_idx_rx+1) % FAP_CH_TAB_SIZE;          
            hal_nrf_set_rf_channel(fap_ch_tab[fap_ch_tab_idx_rx]);
          }
          hal_nrf_set_power_mode(HAL_NRF_PWR_UP);
          FAP_SET_BIT(fap_status, FAP_RADIO_PWR_UP);
        }

        if(fap_timer_period==FAP_RX_FULL_CH_REV)
        {
          CE_LOW();
          hal_nrf_set_power_mode(HAL_NRF_PWR_DOWN);
          FAP_CLEAR_BIT(fap_status, FAP_RADIO_PWR_UP);
        }

        if(fap_timer_period < FAP_RX_FULL_CH_REV)
        {
          // RX on
          if( ( (fap_timer_period-FAP_RADIO_PWR_UP_DELAY) % FAP_RX_SINGLE_CH_REV) == 0)          
          {
            CE_HIGH();    
          }
          // RX off  
          if( ( (fap_timer_period-FAP_RADIO_PWR_UP_DELAY-FAP_LP_RX_LISTEN_PERIODS) % FAP_RX_SINGLE_CH_REV) == 0) 
          {
            CE_LOW();
          }
        }
      }
    }
    else
    {
      fap_set_system_idle();  
    }
    
    if(FAP_GET_BIT(fap_status, FAP_LP_RX_EN))     
    {
      fap_timer_period = (fap_timer_period + 1) % FAP_LP_RX_POLL_PERIOD;
    }
    else
    {
      fap_timer_period = (fap_timer_period + 1) % FAP_CH_TAB_SIZE;
    }
  } 
  fap_rx_channel_guess = (fap_rx_channel_guess + 1) % FAP_CH_TAB_SIZE;
  
  FAP_NRF_IRQ_ENABLE();                      
}

void fap_flush_rx_fifo(void)
{
    FAP_INTERRUPTS_DISABLE();

    hal_nrf_flush_rx();

    FAP_INTERRUPTS_ENABLE();
}

void fap_goto_idle_mode()
{
  FAP_INTERRUPTS_DISABLE();

  FAP_CLEAR_BIT(fap_status, FAP_TX_RX_INFINITE_TIMEOUT_EN);
  
  FAP_INTERRUPTS_ENABLE();

  fap_latency_counter=0; 
  while(fap_mode!=FAP_IDLE);
}

//***************************************************************************** 
// Function bodies, internal
//*****************************************************************************

static uint8_t fap_lfsr_get(uint8_t seed, uint8_t max_limit)
{
  static uint8_t pseudoreg=0xff; // Can never be zero  
  uint8_t shiftbit;
  
  if(seed > 0)
  {
    pseudoreg = seed;      
  }
      
  shiftbit = (pseudoreg << 7) & 0x80;
  shiftbit ^= (pseudoreg << 6) & 0x80;
  shiftbit ^= (pseudoreg << 5) & 0x80;
  shiftbit ^= (pseudoreg & 0x80);
    
  pseudoreg = (shiftbit | (pseudoreg >> 1));      

  return pseudoreg % max_limit;
}

static void fap_set_system_idle(void)
{ 
  CE_LOW();
  fap_mode=FAP_IDLE;
  if(FAP_GET_BIT(fap_status, FAP_PDOWN_IDLE_EN))
  {
    hal_nrf_set_power_mode(HAL_NRF_PWR_DOWN);
    FAP_CLEAR_BIT(fap_status, FAP_RADIO_PWR_UP);
  }  
}

/**
@}
*/
