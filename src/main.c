#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include "pwm_z42.h"

/* 
 * Mapeamento dos Canais PWM do LED RGB na FRDM-KL25Z
 * Vermelho: PTB18 (TPM2_CH0) | Verde: PTB19 (TPM2_CH1) | Azul: PTD1 (TPM0_CH1)
  */
#define PWM_MODULO      255

#define RED_TPM         TPM2
#define RED_CH          0
#define GREEN_TPM       TPM2
#define GREEN_CH        1
#define BLUE_TPM        TPM0
#define BLUE_CH         1

/*
 * Definições do Protocolo (6 bytes: ID | R | G | B | AÇÃO | CRC-8)*/
#define DEVICE_ID       0x01
#define PACKET_SIZE     6
#define RX_TIMEOUT_MS   50

#define ACT_LIGAR       0x01
#define ACT_DESLIGAR    0x02
#define ACT_TOGGLE      0x03
#define ACT_PISCA_1S    0x04
#define ACT_PISCA_2S    0x05

static const struct device *uart0_dev;

/* Buffer de recepção na ISR e fila para processar pacotes no main() */
static uint8_t rx_buf[PACKET_SIZE];
static uint8_t rx_idx = 0;
K_MSGQ_DEFINE(pkt_queue, PACKET_SIZE, 8, 1);

/* Estado atual de intensidade (0 a 255) e temporização do LED RGB */
static uint8_t val_r = 0;
static uint8_t val_g = 0;
static uint8_t val_b = 0;
static bool led_ligado = false;
static uint32_t intervalo_pisca_ms = 0;

/*
 * Cálculo de CRC-8 (Polinômio 0x07, Init 0x00, MSB-first)*/
static uint8_t calcular_crc8(const uint8_t *dados, size_t len)
{
    uint8_t crc = 0x00;

    for (size_t i = 0; i < len; i++) {
        crc ^= dados[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }

    return crc;
}

/*
 * Controle de Intensidade via pwm_z402 e Temporizador de Pisca */
static uint16_t ajustar_duty(uint8_t intensidade)
{
    return (intensidade == 255) ? (PWM_MODULO + 1) : intensidade;
}

static void atualizar_leds(bool ligar)
{
    led_ligado = ligar;

    if (ligar) {
        pwm_tpm_CnV(RED_TPM,   RED_CH,   ajustar_duty(val_r));
        pwm_tpm_CnV(GREEN_TPM, GREEN_CH, ajustar_duty(val_g));
        pwm_tpm_CnV(BLUE_TPM,  BLUE_CH,  ajustar_duty(val_b));
    } else {
        pwm_tpm_CnV(RED_TPM,   RED_CH,   0);
        pwm_tpm_CnV(GREEN_TPM, GREEN_CH, 0);
        pwm_tpm_CnV(BLUE_TPM,  BLUE_CH,  0);
    }
}

static void blink_work_cb(struct k_work *work)
{
    if (intervalo_pisca_ms > 0) {
        atualizar_leds(!led_ligado);
        k_work_reschedule(k_work_delayable_from_work(work), K_MSEC(intervalo_pisca_ms));
    }
}

K_WORK_DELAYABLE_DEFINE(blink_work, blink_work_cb);

/* =========================================================================
 * Timeout para Pacotes Truncados (menos de 6 bytes recebidos)
 * ========================================================================= */
static void rx_timeout_cb(struct k_timer *timer)
{
    ARG_UNUSED(timer);

    unsigned int key = irq_lock();
    if (rx_idx > 0) {
        rx_idx = 0;
    }
    irq_unlock(key);
}

K_TIMER_DEFINE(rx_timeout_timer, rx_timeout_cb, NULL);

/* =========================================================================
 * Callback de Interrupção da UART0
 * ========================================================================= */
static void uart0_cb(const struct device *dev, void *user_data)
{
    ARG_UNUSED(user_data);
    uint8_t c;

    if (!uart_irq_update(dev)) {
        return;
    }

    uart_err_check(dev);

    if (uart_irq_rx_ready(dev)) {
        while (uart_fifo_read(dev, &c, 1) == 1) {
            unsigned int key = irq_lock();

            rx_buf[rx_idx++] = c;

            if (rx_idx == PACKET_SIZE) {
                k_timer_stop(&rx_timeout_timer);
                rx_idx = 0;
                irq_unlock(key);

                k_msgq_put(&pkt_queue, rx_buf, K_NO_WAIT);
            } else {
                k_timer_start(&rx_timeout_timer, K_MSEC(RX_TIMEOUT_MS), K_NO_WAIT);
                irq_unlock(key);
            }
        }
    }
}

/*
 * Validação (ID e CRC-8) e Execução das Ações*/
static void processar_pacote(const uint8_t pkt[PACKET_SIZE])
{
    uint8_t id    = pkt[0];
    uint8_t r     = pkt[1];
    uint8_t g     = pkt[2];
    uint8_t b     = pkt[3];
    uint8_t acao  = pkt[4];
    uint8_t check = pkt[5];

    /* 1. Validação do ID */
    if (id != DEVICE_ID) {
        printk("Erro: ID invalido (0x%02X)\n", id);
        return;
    }

    /* 2. Validação do CRC-8 sobre os 5 primeiros bytes */
    uint8_t crc_esperado = calcular_crc8(pkt, PACKET_SIZE - 1);
    if (check != crc_esperado) {
        printk("Erro: CRC-8 incorreto (recebido 0x%02X, esperado 0x%02X)\n",
               check, crc_esperado);
        return;
    }

    k_work_cancel_delayable(&blink_work);
    intervalo_pisca_ms = 0;

    /* 3. Execução da Ação */
    switch (acao) {
    case ACT_LIGAR:
        val_r = r;
        val_g = g;
        val_b = b;
        atualizar_leds(true);
        printk("OK: LIGAR PWM (R=%u G=%u B=%u)\n", val_r, val_g, val_b);
        break;

    case ACT_DESLIGAR:
        atualizar_leds(false);
        printk("OK: DESLIGAR\n");
        break;

    case ACT_TOGGLE:
        if (r > 0 || g > 0 || b > 0) {
            val_r = r;
            val_g = g;
            val_b = b;
        }
        atualizar_leds(!led_ligado);
        printk("OK: TOGGLE -> %s (R=%u G=%u B=%u)\n",
               led_ligado ? "LIGADO" : "DESLIGADO", val_r, val_g, val_b);
        break;

    case ACT_PISCA_1S:
        val_r = r;
        val_g = g;
        val_b = b;
        intervalo_pisca_ms = 1000;
        atualizar_leds(true);
        k_work_reschedule(&blink_work, K_MSEC(intervalo_pisca_ms));
        printk("OK: PISCA 1s PWM (R=%u G=%u B=%u)\n", val_r, val_g, val_b);
        break;

    case ACT_PISCA_2S:
        val_r = r;
        val_g = g;
        val_b = b;
        intervalo_pisca_ms = 2000;
        atualizar_leds(true);
        k_work_reschedule(&blink_work, K_MSEC(intervalo_pisca_ms));
        printk("OK: PISCA 2s PWM (R=%u G=%u B=%u)\n", val_r, val_g, val_b);
        break;

    default:
        printk("Erro: Acao desconhecida (0x%02X)\n", acao);
        break;
    }
}

int main(void)
{
    if (!pwm_tpm_Init(TPM2, TPM_PLLFLL, PWM_MODULO, TPM_CLK, PS_128, EDGE_PWM) ||
        !pwm_tpm_Init(TPM0, TPM_PLLFLL, PWM_MODULO, TPM_CLK, PS_128, EDGE_PWM)) {
        printk("Erro ao inicializar timers TPM!\n");
        return -EIO;
    }

    if (!pwm_tpm_Ch_Init(RED_TPM,   RED_CH,   TPM_PWM_L, GPIOB, 18) ||
        !pwm_tpm_Ch_Init(GREEN_TPM, GREEN_CH, TPM_PWM_L, GPIOB, 19) ||
        !pwm_tpm_Ch_Init(BLUE_TPM,  BLUE_CH,  TPM_PWM_L, GPIOD, 1)) {
        printk("Erro ao inicializar canais PWM RGB!\n");
        return -EIO;
    }

    atualizar_leds(false);

    uart0_dev = DEVICE_DT_GET(DT_NODELABEL(uart0));
    if (!device_is_ready(uart0_dev)) {
        printk("uart0 nao esta pronta!\n");
        return -ENODEV;
    }

    const struct uart_config uart_cfg = {
        .baudrate  = 115200,
        .parity    = UART_CFG_PARITY_NONE,
        .stop_bits = UART_CFG_STOP_BITS_1,
        .data_bits = UART_CFG_DATA_BITS_8,
        .flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
    };
    uart_configure(uart0_dev, &uart_cfg);

    uart_irq_callback_user_data_set(uart0_dev, uart0_cb, NULL);
    uart_irq_rx_enable(uart0_dev);

    printk("Receptor UART RGB (PWM + CRC-8) iniciado (115200 8N1)\n");

    uint8_t pacote[PACKET_SIZE];
    while (1) {
        k_msgq_get(&pkt_queue, pacote, K_FOREVER);
        processar_pacote(pacote);
    }

    return 0;
}