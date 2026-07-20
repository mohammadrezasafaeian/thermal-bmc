/* ============================================================
   Zone1 node — DEBUG BUILD v3 — bare-metal TWI slave
   + LED probe on PB0
   + UART status stream on TXD/PD1 (9600-8N1)
   ATmega32 @ 8 MHz internal RC, CodeVisionAVR

   WHAT THIS BUILD ANSWERS
   -----------------------
   The TWI ISR fires like crazy when both boards are up, but the
   node never shows ONLINE. Two surviving explanations:
     (a) bus-error storm  -> stream will read: 00 00 00 ...
     (b) transactions start but die partway -> stream shows which
         leg dies, by its exact status vocabulary
   Every TWI ISR entry logs TWSR into a ring; the main loop drains
   the ring out the UART as hex.

   LED PROTOCOL (PB0, active high) — unchanged from v2
   ---------------------------------------------------
   boot    : 3 quick flashes        = THIS image is running
   healthy : blip every ~2 s        = alive, regs OK, no TWI traffic
             double-blip            = TWI activity (merges to flicker
                                      under continuous 10 Hz polling —
                                      that's normal, read the UART)
   fail    : 5 rapid flashes, then TWCR as 8 slow bits (long=1,
             short=0, MSB first), pause, TWAR same way, repeat

   UART PROTOCOL
   -------------
   On boot:  "RST " then TWCR and TWAR as hex (sanity snapshot)
   Runtime:  one hex byte per TWI ISR entry = the TWSR status seen
   Ring overflow: '!' printed (statuses were dropped, not corrupted)
   ============================================================ */

#include <mega32.h>
#include <delay.h>

/* ============================================================
   SECTION 1 — I2C data contracts (must match STM32 exactly)
   ============================================================ */
typedef struct { unsigned int adc_raw;
                 unsigned int tach_pulses; } I2C_Telemetry;  /* 4B, master reads  */
typedef struct { unsigned char heater_pwm;
                 unsigned char fan_pwm;    } I2C_Command;    /* 2B, master writes */

I2C_Telemetry node_tel;
I2C_Command   node_cmd;

volatile unsigned int live_tach_count = 0;
float adc_ema = 0.0;

/* ============================================================
   SECTION 2 — TWI definitions (no twi.h anywhere)
   ============================================================ */
#define B_TWINT 0x80
#define B_TWEA  0x40
#define B_TWSTO 0x10
#define B_TWEN  0x04
#define B_TWIE  0x01
#define TWCR_ARMED (B_TWEA | B_TWEN | B_TWIE)            /* 0x45 */
#define TWCR_REARM (B_TWINT | B_TWEA | B_TWEN | B_TWIE)  /* 0xC5 */
#define NODE_TWAR  (0x20 << 1)                           /* 0x40 */

#define S_SLA_W_RX     0x60
#define S_DATA_RX_ACK  0x80
#define S_STOP_RSTART  0xA0
#define S_SLA_R_RX     0xA8
#define S_DATA_TX_ACK  0xB8
#define S_DATA_TX_NACK 0xC0
#define S_LAST_TX_ACK  0xC8
#define S_BUS_ERROR    0x00

unsigned char rx_buf[2];
unsigned char tx_buf[4];
unsigned char rx_idx = 0, tx_idx = 0;

/* ============================================================
   SECTION 3 — debug instrumentation
   ============================================================ */
/* --- TWI status ring: ISR produces, main loop consumes.
   Head/tail are single bytes -> reads/writes are inherently
   atomic on AVR. (Note: this producer/consumer shape is exactly
   the pattern your two OPEN torn-16-bit fixes will use.)       */
#define RING_SZ   32                 /* power of two */
#define RING_MASK (RING_SZ - 1)
unsigned char stat_ring[RING_SZ];
volatile unsigned char stat_head = 0;
unsigned char stat_tail = 0;
volatile unsigned char ring_ovf = 0;

/* --- LED on PB0 --- */
#define LED_ON()  (PORTB |=  0x01)
#define LED_OFF() (PORTB &= ~0x01)

void led_blip(unsigned char ms_on) { LED_ON(); delay_ms(ms_on); LED_OFF(); }

void led_dump_byte(unsigned char b)
{
    unsigned char i;
    for (i = 0; i < 8; i++) {
        LED_ON();
        if (b & 0x80) delay_ms(400); else delay_ms(80);
        LED_OFF();
        delay_ms(300);
        b <<= 1;
    }
}

/* --- UART TX-only, 9600-8N1 @ 8 MHz --- */
void uart_init(void)
{
    UBRRH = 0;
    UBRRL = 51;                       /* 8e6/(16*9600)-1, +0.2% error */
    UCSRB = (1<<TXEN);
    UCSRC = (1<<URSEL)|(1<<UCSZ1)|(1<<UCSZ0);
}

void uart_tx(unsigned char c)
{
    while (!(UCSRA & (1<<UDRE)));
    UDR = c;
}

void uart_hex(unsigned char b)
{
    unsigned char n = b >> 4;
    uart_tx(n < 10 ? '0'+n : 'A'+(n-10));
    n = b & 0x0F;
    uart_tx(n < 10 ? '0'+n : 'A'+(n-10));
    uart_tx(' ');
}

/* --- register probe + fail dump (LED path, unchanged) --- */
unsigned char twi_regs_ok(void)
{
    if ((TWCR & TWCR_ARMED) != TWCR_ARMED) return 0;
    if (TWAR != NODE_TWAR)                 return 0;
    return 1;
}

void probe_fail_dump(void)
{
    unsigned char i;
    while (1) {
        for (i = 0; i < 5; i++) { led_blip(40); delay_ms(80); }
        delay_ms(800);
        led_dump_byte(TWCR);  delay_ms(1200);
        led_dump_byte(TWAR);  delay_ms(2500);
        /* also push them out the UART each cycle, in case the
           terminal is connected: */
        uart_tx('F'); uart_tx(' ');
        uart_hex(TWCR); uart_hex(TWAR);
        uart_tx('\r'); uart_tx('\n');
    }
}

/* ============================================================
   SECTION 4 — ISRs
   ============================================================ */
interrupt [EXT_INT0] void ext_int0_isr(void)
{
    live_tach_count++;
}

interrupt [TIM1_COMPA] void timer1_compa_isr(void)
{
    node_tel.tach_pulses = live_tach_count;   /* ISR-to-ISR: no tearing */
    live_tach_count = 0;
}

interrupt [TWI] void twi_isr(void)
{
    unsigned char st = TWSR & 0xF8;

    /* log every entry; detect overflow without blocking */
    if ((unsigned char)(stat_head - stat_tail) < RING_SZ)
        stat_ring[stat_head & RING_MASK] = st;
    else
        ring_ovf = 1;
    stat_head++;

    switch (st)
    {
    case S_SLA_W_RX:                    /* master writes: open frame */
        rx_idx = 0;
        break;

    case S_DATA_RX_ACK:
        if (rx_idx < sizeof(rx_buf)) rx_buf[rx_idx++] = TWDR;
        /* PLACEHOLDER: extra bytes discarded but ACKed (your call) */
        break;

    case S_STOP_RSTART:
        /* PLACEHOLDER: apply-at-STOP, full frames only (your call) */
        if (rx_idx == sizeof(I2C_Command)) {
            node_cmd = *((I2C_Command*)rx_buf);
            OCR0 = node_cmd.heater_pwm;
            OCR2 = node_cmd.fan_pwm;
        }
        break;

    case S_SLA_R_RX:                    /* master reads: snapshot + 1st byte */
        *((I2C_Telemetry*)tx_buf) = node_tel;   /* gie off: TIM1 can't tear */
        tx_idx = 0;
        TWDR = tx_buf[tx_idx++];
        break;

    case S_DATA_TX_ACK:
        /* PLACEHOLDER: over-read pads 0xFF (your call) */
        TWDR = (tx_idx < sizeof(tx_buf)) ? tx_buf[tx_idx++] : 0xFF;
        break;

    case S_DATA_TX_NACK:
    case S_LAST_TX_ACK:
        break;

    case S_BUS_ERROR:
        TWCR = B_TWINT | B_TWSTO | TWCR_ARMED;  /* datasheet recovery */
        return;
    }

    TWCR = TWCR_REARM;
}

/* ============================================================
   SECTION 5 — ADC (polled, AVCC ref, 125 kHz)
   ============================================================ */
#define ADC_VREF_TYPE ((0<<REFS1) | (1<<REFS0) | (0<<ADLAR))

unsigned int read_adc(unsigned char ch)
{
    ADMUX = ch | ADC_VREF_TYPE;
    delay_us(10);
    ADCSRA |= (1<<ADSC);
    while ((ADCSRA & (1<<ADIF)) == 0);
    ADCSRA |= (1<<ADIF);
    return ADCW;
}

/* ============================================================
   SECTION 6 — main
   ============================================================ */
void main(void)
{
    unsigned char i;
    unsigned int  loop_ct = 0;
    unsigned char saw_traffic = 0;
    unsigned int  raw;

    /* 6.1 ports: PB0 LED, PB3 OC0, PD7 OC2, PD2 INT0 pull-up,
       PD1 = TXD (UART owns it once TXEN is set)               */
    DDRB  = (1<<DDB3) | (1<<DDB0);   PORTB = 0x00;
    DDRA  = 0x00;                    PORTA = 0x00;
    DDRC  = 0x00;                    PORTC = 0x00;
    DDRD  = (1<<DDD7);               PORTD = (1<<PORTD2);

    /* 6.2 timers */
    TCCR0 = (1<<WGM00)|(1<<COM01)|(1<<WGM01)|(1<<CS01)|(1<<CS00);
    TCNT0 = 0; OCR0 = 0;                       /* heater PWM ~488 Hz  */

    TCCR1A = 0;
    TCCR1B = (1<<WGM12)|(1<<CS12);
    TCNT1H = 0; TCNT1L = 0;
    OCR1AH = 0x7A; OCR1AL = 0x11;              /* 1 Hz CTC            */

    ASSR  = 0;                                 /* fan PWM 31.25 kHz — */
    TCCR2 = (1<<WGM20)|(1<<COM21)|(1<<WGM21)|(1<<CS20);  /* hand-set:  */
    TCNT2 = 0; OCR2 = 0;                       /* wizard regen reverts */

    TIMSK = (1<<OCIE1A);

    /* 6.3 INT0 rising */
    GICR |= (1<<INT0);
    MCUCR = (1<<ISC01)|(1<<ISC00);
    GIFR  = (1<<INTF0);

    /* 6.4 ADC */
    ADMUX  = ADC_VREF_TYPE;
    ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1);
    SFIOR  = 0;

    /* 6.5 UART up FIRST so everything after can speak */
    uart_init();

    /* 6.6 TWI slave arm */
    TWAR = NODE_TWAR;
    TWCR = TWCR_ARMED;

    #asm("sei")

    /* 6.7 boot signature: LED + UART snapshot of the armed regs */
    for (i = 0; i < 3; i++) { led_blip(60); delay_ms(140); }
    uart_tx('R'); uart_tx('S'); uart_tx('T'); uart_tx(' ');
    uart_hex(TWCR); uart_hex(TWAR);
    uart_tx('\r'); uart_tx('\n');
    delay_ms(400);

    /* 6.8 checkpoint B */
    if (!twi_regs_ok()) probe_fail_dump();

    adc_ema = (float)read_adc(0);

    /* 6.9 main loop */
    while (1)
    {
        raw = read_adc(0);
        adc_ema = adc_ema + 0.05 * ((float)raw - adc_ema);
        /* OWED (ledger): tearable 16-bit store — fix pending, and the
           ring above shows the exact pattern to use               */
        node_tel.adc_raw = (unsigned int)(adc_ema + 0.5);

        /* drain the status ring out the UART */
        while (stat_tail != stat_head) {
            uart_hex(stat_ring[stat_tail & RING_MASK]);
            stat_tail++;
            saw_traffic = 1;
        }
        if (ring_ovf) { ring_ovf = 0; uart_tx('!'); uart_tx(' '); }

        delay_ms(10);
        loop_ct++;

        /* checkpoint C: catch late register clobbering */
        if (!twi_regs_ok()) probe_fail_dump();

        /* LED: heartbeat when idle; newline groups the stream when
           traffic pauses (makes transactions readable in terminal) */
        if (saw_traffic) {
            saw_traffic = 0;
        } else if (loop_ct >= 200) {
            loop_ct = 0;
            led_blip(30);
            uart_tx('\r'); uart_tx('\n');   /* visual break in the log */
        }
    }
}