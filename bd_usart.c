/**
 * @file bd_usart.c
 * @author bdelta
 * @brief STM32F4 UART3 Library Implementation
 * @version 0.1
 * @date 2022-09-04
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "bd_usart.h"

char bd_usart3_tx_buf[BD_USART3_MAX_STR_LEN];

/* Initialization */
/* Must be called after GPIOD clk enabled */
void bd_usart3_setup(void)
{
    /* Enable the USART3 clock */
    *(volatile uint32_t *)(BD_RCC_REG) |= BD_RCC_USART3_EN_BIT;

    /* GPIOD PD8 Setup */
    /* Configure PD8 mode */
    *(volatile uint32_t *)(BD_GPIOD_MODER_REG) |= (BD_GPIO_MODER_AF_MODE_VAL << BD_GPIO_MODER_P8_SHIFT);

    /* Configure PD8 to be USART3_TX alternate func */
    *(volatile uint32_t *)(BD_GPIOD_AFRH_REG) |= BD_GPIO_AFRH8_AF7;

    /* Configure PD8 to be pullup */
    *(volatile uint32_t *)(BD_GPIOD_PUPDR_REG) |=
        (BD_GPIO_PUPDR_PULLUP_VAL << BD_GPIO_PUPDR_PUPDR8_BIT_SHIFT);

    /* Configure PD8 to be push-pull */
    *(volatile uint32_t *)(BD_GPIOD_OTYPER_REG) &= ~(1 << BD_GPIO_OTYPER_P8_SHIFT);

    /* PD8 very high freq */
    *(volatile uint32_t *)(BD_GPIOD_OSPEEDR_REG) |=
        (BD_GPIO_OSPEEDR_VERY_HIGH_SPEED_VAL << BD_GPIO_OSPEEDR_P8_SHIFT);

    /* Disable the USART3 */
    *(volatile uint32_t *)(BD_USART3CR1_REG) &= ~BD_USARTCR1_UEN_BIT;

    /* Clear the M bit to set 8 bit transfers */
    *(volatile uint32_t *)(BD_USART3CR1_REG) &= ~BD_USARTCR1_M_BIT;

    /* Set oversampling to 16 */
    *(volatile uint32_t *)(BD_USART3CR1_REG) &= ~BD_USARTCR1_OSAMP_BIT;

    /* Set stop bits to be 1 */
    *(volatile uint32_t *)(BD_USART3CR2_REG) &= ~BD_USARTCR2_STOP_BITS;

    /* Set tx mode */
    *(volatile uint32_t *)(BD_USART3CR1_REG) |= BD_USARTCR1_TE_BIT;

    // uint32_t usart_clk_freq = HAL_RCC_GetPCLK1Freq();
    /* 38 MHz, 9600 baud rate, oversample 8 = 494.791 */
    /* Baud rate setup */
    uint32_t bdd_val = *(volatile uint32_t *)(BD_USART3BRR_REG) & ~(0xFFFFUL);
    bdd_val |= BD_USART_DIV_VAL;
    *(volatile uint32_t *)(BD_USART3BRR_REG) = bdd_val;

    /* Enable the USART3 */
    *(volatile uint32_t *)(BD_USART3CR1_REG) |= BD_USARTCR1_UEN_BIT;
}

/* Transmission function helpers */
static uint8_t bd_get_txe_stat(void)
{
    return (*(bd_rdwr_reg *)(BD_USART3SR_REG) & BD_USARTSR_TXE_BIT) >> BD_USARTSR_TXE_BIT_SHIFT;
}

static void bd_wait_txe(void)
{
    while (bd_get_txe_stat() == 0)
        ;
}

static uint8_t bd_get_tc_stat(void)
{
    return (*(bd_rdwr_reg *)(BD_USART3SR_REG) & BD_USARTSR_TC_BIT) >> BD_USARTSR_TC_BIT_SHIFT;
}

static void bd_wait_tc(void)
{
    while (bd_get_tc_stat() == 0)
        ;
}

/* write to dr clears txe bit */
static void bd_write_dr(char c)
{
    *(bd_rdwr_reg *)(BD_USART3DR_REG) = c;
}

static void bd_send_idle_frame(void)
{
    *(bd_rdwr_reg *)(BD_USART3CR1_REG) |= BD_USARTCR1_TE_BIT;
}

/* For each character, write data to usart data reg */
void bd_print_uart3(char arr[BD_USART3_MAX_STR_LEN])
{
  uint32_t txe_stat = 0;
  uint32_t tc_stat = 0;

  bd_send_idle_frame();
  bd_wait_txe();

  for (uint32_t cur = 0; (arr[cur] != '\0') && (cur < BD_USART3_MAX_STR_LEN); ++cur) {
    bd_write_dr(arr[cur]);
    bd_wait_txe();
  }

  bd_wait_tc();
}

void bd_clear_uart3(void)
{
    const static char clear_seq[] = {0x1B, 0x5B, 0x32, 0x4A, 0};

    bd_print_uart3(clear_seq);
}
