#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>

/* LED configuration */
static const struct gpio_dt_spec led_r = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);
static const struct gpio_dt_spec led_g = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec led_b = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);

#define DEVICE_ID       0x01
#define PACKET_SIZE     6
#define RX_TIMEOUT_MS   50

#define ACT_LIGAR       0x01
#define ACT_DESLIGAR    0x02
#define ACT_TOGGLE      0x03
#define ACT_PISCA_1S    0x04
#define ACT_PISCA_2S    0x05

static const struct device *uart0_dev;

static uint8_t rx_buf[PACKET_SIZE];
static uint8_t rx_idx = 0;
K_MSGQ_DEFINE(pkt_queue, PACKET_SIZE, 8, 1);

/* Estado atual do LED RGB */
static bool cor_r = false;
static bool cor_g = false;
static bool cor_b = false;
static bool led_ligado = false;
static uint32_t intervalo_pisca_ms = 0;


static void atualizar_leds(bool ligar)
{
    led_ligado = ligar;
    gpio_pin_set_dt(&led_r, (ligar && cor_r) ? 1 : 0);
    gpio_pin_set_dt(&led_g, (ligar && cor_g) ? 1 : 0);
    gpio_pin_set_dt(&led_b, (ligar && cor_b) ? 1 : 0);
}

static void blink_work_cb(struct k_work *work)
{
    if (intervalo_pisca_ms > 0) {
        atualizar_leds(!led_ligado);
        k_work_reschedule(k_work_delayable_from_work(work), K_MSEC(intervalo_pisca_ms));
    }
}

K_WORK_DELAYABLE_DEFINE(blink_work, blink_work_cb);

static void rx_timeout_cb(struct k_timer *timer)
{
    ARG_UNUSED(timer);

    unsigned int key = irq_lock();
    if (rx_idx > 0) {
        rx_idx = 0; /* Descarta pacote incompleto */
    }
    irq_unlock(key);
}

K_TIMER_DEFINE(rx_timeout_timer, rx_timeout_cb, NULL);


static void uart0_cb(const struct device *dev, void *user_data)
{
    ARG_UNUSED(user_data);
    uint8_t c;

    if (!uart_irq_update(dev)) {
        return;
    }

    /* Limpa eventuais erros de overrun/framing da linha serial */
    uart_err_check(dev);

    if (uart_irq_rx_ready(dev)) {
        while (uart_fifo_read(dev, &c, 1) == 1) {
            unsigned int key = irq_lock();

            rx_buf[rx_idx++] = c;

            if (rx_idx == PACKET_SIZE) {
                /* Pacote de 6 bytes completo: para o timer e envia para o main */
                k_timer_stop(&rx_timeout_timer);
                rx_idx = 0;
                irq_unlock(key);

                k_msgq_put(&pkt_queue, rx_buf, K_NO_WAIT);
            } else {
                /* Reinicia janela de timeout aguardando o próximo byte */
                k_timer_start(&rx_timeout_timer, K_MSEC(RX_TIMEOUT_MS), K_NO_WAIT);
                irq_unlock(key);
            }
        }
    }
} 

static void processar_pacote(const uint8_t pkt[PACKET_SIZE])
{
    uint8_t id     = pkt[0];
    uint8_t r      = pkt[1];
    uint8_t g      = pkt[2];
    uint8_t b      = pkt[3];
    uint8_t acao   = pkt[4];
    uint8_t check  = pkt[5];

    /* 1. Validação do ID */
    if (id != DEVICE_ID) {
        printk("Erro: ID invalido (0x%02X)\n", id);
        return;
    }

    /* 2. Validação do Checksum (XOR dos 5 primeiros bytes) */
    uint8_t xor_esperado = id ^ r ^ g ^ b ^ acao;
    if (check != xor_esperado) {
        printk("Erro: CHECK incorreto (recebido 0x%02X, esperado 0x%02X)\n",
               check, xor_esperado);
        return;
    }

    /* Cancela qualquer pisca em andamento antes de aplicar o novo comando */
    k_work_cancel_delayable(&blink_work);
    intervalo_pisca_ms = 0;

    /* 3. Execução da Ação */
    switch (acao) {
    case ACT_LIGAR:
        cor_r = (r > 0);
        cor_g = (g > 0);
        cor_b = (b > 0);
        atualizar_leds(true);
        printk("OK: LIGAR (R=%d G=%d B=%d)\n", cor_r, cor_g, cor_b);
        break;

    case ACT_DESLIGAR:
        atualizar_leds(false);
        printk("OK: DESLIGAR\n");
        break;

    case ACT_TOGGLE:
        if (r > 0 || g > 0 || b > 0) {
            cor_r = (r > 0);
            cor_g = (g > 0);
            cor_b = (b > 0);
        }
        atualizar_leds(!led_ligado);
        printk("OK: TOGGLE -> %s\n", led_ligado ? "LIGADO" : "DESLIGADO");
        break;

    case ACT_PISCA_1S:
        cor_r = (r > 0);
        cor_g = (g > 0);
        cor_b = (b > 0);
        intervalo_pisca_ms = 1000;
        atualizar_leds(true);
        k_work_reschedule(&blink_work, K_MSEC(intervalo_pisca_ms));
        printk("OK: PISCA 1s\n");
        break;

    case ACT_PISCA_2S:
        cor_r = (r > 0);
        cor_g = (g > 0);
        cor_b = (b > 0);
        intervalo_pisca_ms = 2000;
        atualizar_leds(true);
        k_work_reschedule(&blink_work, K_MSEC(intervalo_pisca_ms));
        printk("OK: PISCA 2s\n");
        break;

    default:
        printk("Erro: Acao desconhecida (0x%02X)\n", acao);
        break;
    }
}

int main(void)
{
    /* Inicializa os 3 LEDs como desligados */
    if (!gpio_is_ready_dt(&led_r) ||
        !gpio_is_ready_dt(&led_g) ||
        !gpio_is_ready_dt(&led_b)) {
        return -ENODEV;
    }

    gpio_pin_configure_dt(&led_r, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led_g, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led_b, GPIO_OUTPUT_INACTIVE);

    /* Obtém e configura uart0 em 115200 8N1 */
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

    /* Configura callback da UART0 */
    uart_irq_callback_user_data_set(uart0_dev, uart0_cb, NULL);
    uart_irq_rx_enable(uart0_dev);

    printk("Receptor UART RGB iniciado (115200 8N1)\n");

    uint8_t pacote[PACKET_SIZE];
    while (1) {
        /* Aguarda pacotes completos sem consumir CPU */
        k_msgq_get(&pkt_queue, pacote, K_FOREVER);
        processar_pacote(pacote);
    }

    return 0;
}
