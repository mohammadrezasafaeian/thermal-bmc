/* ============================================================
   Zone1 node firmware — bare-metal TWI slave + differential NTC
   ATmega32 @ 8 MHz internal RC (VCC = 5 V), CodeVisionAVR

   ROLE
   ----
   Dumb physical layer for the thermal BMC. Owns: sensor sampling,
   PWM generation, tach counting. Makes NO control decisions.
   Supervisor (STM32F411) polls at ~10 Hz over I2C: 2-byte command
   write, then 4-byte telemetry read.

   SENSOR CHAIN (reworked after single-ended resolution proved
   inadequate: 80 C-to-short spanned only ~5 ADC counts)
   -------------------------------------------------------------
   Matched-R25 differential pair, ADC1(+) - ADC0(-), gain 10x:

     AVCC net -- 330 -- ADC1 -- NTC 10R@25C -- GND   (hot sensor)
     AVCC net -- 330 -- ADC0 -- 10R wirewound -- GND (reference,
                                     mounted AWAY from heater/fan)

   Differential = 0 at 25 C by construction. Fully ratiometric:
   both legs and the ADC reference share one rail -> rail voltage
   cancels exactly (USB sag irrelevant).
   Result: signed 10-bit, -512..+511. ~3 counts/C at 25 C,
   ~0.6 counts/C at 80 C (10x better than single-ended there).

   Bench reference points (rail 4.47 V, measured 2026-xx):
     T        R_ntc     counts
      0 C     29.3      +267
     25 C     10.0         0
     80 C     1.61      -126     <- hottest legitimate (Tier-3)
     short    0         -151     <- 25-count gap below 80 C
     open     inf     +511 SAT   <- unambiguous

   LED (PB0): 3 flashes at boot = image running; blip ~2 s = alive,
   no bus traffic; fail-dump = 5 flashes then TWCR/TWAR bit-blink.
   UART (PD1, 9600-8N1): "RST <TWCR> <TWAR>" banner at boot, then
   one hex TWSR status per TWI ISR entry (diagnostic stream,
   harmless to leave running; requires 5 V VCC for RC accuracy).
   ============================================================ */

#include <mega32.h>
#include <delay.h>

/* ============================================================
   SECTION 1 — I2C data contracts (must match STM32 exactly)
   ============================================================ */
/* TODO [DECISION #10 - CONTRACT SIGNEDNESS - unmade, blocking]:
   adc_raw now carries a signed differential reading (-512..+511)
   in a uint16_t wire field. Two coherent options:
     (A) re-bias +512 on this side (see the marked line in main);
         STM32 subtracts 512 after reception. Wire stays unsigned.
     (B) change this field to int16_t HERE AND on the STM32 side,
         ship two's complement as-is.
   Pick one. Whichever you pick, write the choice as a comment at
   BOTH struct definitions so the contract is self-documenting.
   The build below contains BOTH lines, (A) active, (B) commented -
   that is NOT a decision, it is a placeholder to make the file
   compile. Decide and delete the loser.                          */
typedef struct { unsigned int adc_raw;      /* see TODO above     */
                 unsigned int tach_pulses; } I2C_Telemetry; /* 4B */
typedef struct { unsigned char heater_pwm;
                 unsigned char fan_pwm;    } I2C_Command;   /* 2B */

I2C_Telemetry node_tel;
I2C_Command   node_cmd;

volatile unsigned int live_tach_count = 0;
float adc_ema = 0.0;

/* ============================================================
   SECTION 2 — TWI definitions (no twi.h)
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
   SECTION 3 — instrumentation (LED + UART status stream)
   ============================================================ */
#define RING_SZ   32
#define RING_MASK (RING_SZ - 1)
unsigned char stat_ring[RING_SZ];
volatile unsigned char stat_head = 0;   /* single-byte indices:   */
unsigned char stat_tail = 0;            /* atomic on AVR - this is
                                           the pattern for the two
                                           OWED torn-16-bit fixes */
volatile unsigned char ring_ovf = 0;

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

void uart_init(void)
{
    UBRRH = 0;
    UBRRL = 51;                       /* 8e6/(16*9600)-1           */
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
    /* 1 Hz snapshot. ISR-to-ISR: AVR doesn't nest -> no tearing. */
    node_tel.tach_pulses = live_tach_count;
    live_tach_count = 0;
}

interrupt [TWI] void twi_isr(void)
{
    unsigned char st = TWSR & 0xF8;

    if ((unsigned char)(stat_head - stat_tail) < RING_SZ)
        stat_ring[stat_head & RING_MASK] = st;
    else
        ring_ovf = 1;
    stat_head++;

    switch (st)
    {
    case S_SLA_W_RX:
        rx_idx = 0;
        break;

    case S_DATA_RX_ACK:
        if (rx_idx < sizeof(rx_buf)) rx_buf[rx_idx++] = TWDR;
        /* TODO [TWI POLICY 1/3 - unmade]: over-length writes are
           currently read-and-discarded but still ACKed. Decide:
           keep (forgiving), or NACK past byte 2 (strict - what
           does the master's HAL do with a mid-write NACK?).      */
        break;

    case S_STOP_RSTART:
        /* TODO [TWI POLICY 2/3 - unmade]: commands apply only on
           a complete 2-byte frame, at STOP. Alternative: apply
           eagerly per byte. Consider a torn 1-byte write dying
           mid-frame under each policy. Current behavior = safe
           default, but it was MY placeholder, not your decision. */
        if (rx_idx == sizeof(I2C_Command)) {
            node_cmd = *((I2C_Command*)rx_buf);
            OCR0 = node_cmd.heater_pwm;
            OCR2 = node_cmd.fan_pwm;
        }
        break;

    case S_SLA_R_RX:
        /* Snapshot: gie off inside ISR -> TIM1 can't tear this.
           Main-loop adc_raw store is the tearable one (see loop). */
        *((I2C_Telemetry*)tx_buf) = node_tel;
        tx_idx = 0;
        TWDR = tx_buf[tx_idx++];
        break;

    case S_DATA_TX_ACK:
        /* TODO [TWI POLICY 3/3 - unmade]: over-reads pad 0xFF.
           Fine? Or repeat last byte / wrap? 0xFF is my
           placeholder; make it yours or change it.               */
        TWDR = (tx_idx < sizeof(tx_buf)) ? tx_buf[tx_idx++] : 0xFF;
        break;

    case S_DATA_TX_NACK:
    case S_LAST_TX_ACK:
        break;

    case S_BUS_ERROR:
        TWCR = B_TWINT | B_TWSTO | TWCR_ARMED;
        return;
    }

    TWCR = TWCR_REARM;
}

/* ============================================================
   SECTION 5 — ADC
   ============================================================ */
#define ADC_VREF_TYPE ((0<<REFS1) | (1<<REFS0) | (0<<ADLAR))
#define ADC_DIFF_10X  0x09     /* MUX4..0 = 01001: ADC1-ADC0, 10x */

/* Single-ended read - kept for diagnostics (e.g. reading either
   leg absolutely during bring-up). Not used in the control path. */
unsigned int read_adc(unsigned char ch)
{
    ADMUX = ch | ADC_VREF_TYPE;
    delay_us(10);
    ADCSRA |= (1<<ADSC);
    while ((ADCSRA & (1<<ADIF)) == 0);
    ADCSRA |= (1<<ADIF);
    return ADCW;
}

/* Differential read, signed. Datasheet: first conversion after
   switching to a differential channel is invalid (gain stage
   settling) -> one throwaway per call. Cost at 100 Hz: ~2% duty.
   Deliberately re-selects ADMUX every call so correctness never
   depends on who touched ADMUX between calls (read_adc does!).   */
int read_adc_diff(void)
{
    int raw;
    ADMUX = ADC_VREF_TYPE | ADC_DIFF_10X;
    delay_us(200);                          /* gain stage settle  */
    ADCSRA |= (1<<ADSC);                    /* throwaway          */
    while (!(ADCSRA & (1<<ADIF)));  ADCSRA |= (1<<ADIF);
    ADCSRA |= (1<<ADSC);                    /* real conversion    */
    while (!(ADCSRA & (1<<ADIF)));  ADCSRA |= (1<<ADIF);
    raw = ADCW;
    if (raw & 0x0200) raw -= 1024;          /* sign-extend 10-bit */
    return raw;                             /* -512 .. +511       */
}

/* ============================================================
   SECTION 6 — main
   ============================================================ */
void main(void)
{
    unsigned char i;
    unsigned int  loop_ct = 0;
    unsigned char saw_traffic = 0;
    int raw;                                /* SIGNED now         */

    /* 6.1 ports: PB0 LED, PB3 OC0 heater, PD7 OC2 fan,
       PD2 INT0 tach (pull-up), PD1 TXD.
       PA0/PA1 = ADC0/ADC1 differential pair: inputs, no pull-ups. */
    DDRB  = (1<<DDB3) | (1<<DDB0);   PORTB = 0x00;
    DDRA  = 0x00;                    PORTA = 0x00;
    DDRC  = 0x00;                    PORTC = 0x00;
    DDRD  = (1<<DDD7);               PORTD = (1<<PORTD2);

    /* 6.2 timers */
    TCCR0 = (1<<WGM00)|(1<<COM01)|(1<<WGM01)|(1<<CS01)|(1<<CS00);
    TCNT0 = 0; OCR0 = 0;                    /* heater PWM ~488 Hz */

    TCCR1A = 0;
    TCCR1B = (1<<WGM12)|(1<<CS12);
    TCNT1H = 0; TCNT1L = 0;
    OCR1AH = 0x7A; OCR1AL = 0x11;           /* 1 Hz CTC           */

    /* Timer2: fan PWM 31.25 kHz. HAND-SET, wizard regen reverts
       this to Normal mode silently - do not re-run CodeWizard on
       this file without re-checking TCCR2.                       */
    ASSR  = 0;
    TCCR2 = (1<<WGM20)|(1<<COM21)|(1<<WGM21)|(1<<CS20);
    TCNT2 = 0; OCR2 = 0;

    TIMSK = (1<<OCIE1A);

    /* 6.3 INT0 rising edge (tach) */
    GICR |= (1<<INT0);
    MCUCR = (1<<ISC01)|(1<<ISC00);
    GIFR  = (1<<INTF0);

    /* 6.4 ADC: enable, /64 -> 125 kHz. Note: differential+10x
       wants ADC clock <= 200 kHz for full accuracy - 125 kHz OK. */
    ADMUX  = ADC_VREF_TYPE;
    ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1);
    SFIOR  = 0;

    /* 6.5 UART first, so everything after can speak */
    uart_init();

    /* 6.6 TWI slave arm */
    TWAR = NODE_TWAR;
    TWCR = TWCR_ARMED;

    #asm("sei")

    /* 6.7 boot signature */
    for (i = 0; i < 3; i++) { led_blip(60); delay_ms(140); }
    uart_tx('R'); uart_tx('S'); uart_tx('T'); uart_tx(' ');
    uart_hex(TWCR); uart_hex(TWAR);
    uart_tx('\r'); uart_tx('\n');
    delay_ms(400);

    if (!twi_regs_ok()) probe_fail_dump();

    /* 6.8 seed EMA from the differential channel (no cold ramp) */
    adc_ema = (float)read_adc_diff();

    /* 6.9 main loop: 100 Hz sample + stream drain */
    while (1)
    {
        raw = read_adc_diff();
        adc_ema = adc_ema + 0.05 * ((float)raw - adc_ema);

        /* TODO [DECISION #10]: (A) active as compile placeholder,
           (B) commented. DECIDE - see Section 1.
           OWED separately (torn-16-bit #2): this is a two-byte
           store; the TWI ISR can fire between the bytes and ship
           a chimera. The stat_ring head/tail pattern above is the
           fix shape. Staged behind the contract decision since
           the fix wraps whichever line survives.                 */
        node_tel.adc_raw = (unsigned int)(adc_ema + 512.5);        /* (A) */
        /* node_tel.adc_raw = (int)(adc_ema + (adc_ema>=0?0.5:-0.5)); (B) */

        while (stat_tail != stat_head) {
            uart_hex(stat_ring[stat_tail & RING_MASK]);
            stat_tail++;
            saw_traffic = 1;
        }
        if (ring_ovf) { ring_ovf = 0; uart_tx('!'); uart_tx(' '); }

        delay_ms(10);
        loop_ct++;

        if (!twi_regs_ok()) probe_fail_dump();

        if (saw_traffic) {
            saw_traffic = 0;
        } else if (loop_ct >= 200) {
            loop_ct = 0;
            led_blip(30);
            uart_tx('\r'); uart_tx('\n');
        }
    }
}