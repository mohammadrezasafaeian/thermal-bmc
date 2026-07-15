/* ============================================================
   Zone1 node — DEBUG BUILD v2 — bare-metal TWI slave
                + LED register-dump instrument on PB0
   ATmega32 @ 8 MHz internal RC, CodeVisionAVR

   PURPOSE OF THIS BUILD
   ---------------------
   The previous build proved:
     - the new image runs (boot flashes seen)
     - the chip executes continuously (no reset loop)
     - but the TWI register probe FAILS (rapid strobe)
   This build answers the only question left:
     WHAT VALUE do TWCR and TWAR actually contain
     at the moment the probe fails?
   It blinks the two bytes out on the LED, bit by bit.

   LED PROTOCOL (one LED on PB0, active high)
   ------------------------------------------
   power-up : 3 quick flashes        = this image is running
   healthy  : short blip every ~2 s  = alive, registers OK, no TWI traffic
              double-blip            = TWI interrupt fired (bus activity!)
   fail     : 5 rapid flashes        = "register dump follows"
              then 8 bits of TWCR    (LONG flash = 1, short blip = 0, MSB first)
              pause
              then 8 bits of TWAR    (same encoding)
              long pause, repeats forever

   Expected healthy values:
     TWCR = 0x45 = 0100 0101   (TWEA | TWEN | TWIE)
     TWAR = 0x40 = 0100 0000   (slave address 0x20 << 1)
   ============================================================ */

#include <mega32.h>
#include <delay.h>

/* ============================================================
   SECTION 1 — I2C data contracts (must match STM32 exactly)
   ============================================================ */
typedef struct { unsigned int adc_raw;      /* 10-bit NTC reading      */
                 unsigned int tach_pulses;  /* fan pulses per second   */
} I2C_Telemetry;                            /* 4 bytes, master READS   */

typedef struct { unsigned char heater_pwm;  /* 0..255 -> OCR0          */
                 unsigned char fan_pwm;     /* 0..255 -> OCR2          */
} I2C_Command;                              /* 2 bytes, master WRITES  */

I2C_Telemetry node_tel;
I2C_Command   node_cmd;

volatile unsigned int live_tach_count = 0;
float adc_ema = 0.0;

/* ============================================================
   SECTION 2 — TWI register bits and status codes
   (defined by hand: we are not using twi.h at all)
   ============================================================ */
#define B_TWINT 0x80   /* interrupt flag, write 1 to clear     */
#define B_TWEA  0x40   /* "ACK my own address" enable          */
#define B_TWSTO 0x10   /* stop (slave: recover from bus error) */
#define B_TWEN  0x04   /* TWI hardware enable                  */
#define B_TWIE  0x01   /* TWI interrupt enable                 */

#define TWCR_ARMED (B_TWEA | B_TWEN | B_TWIE)            /* 0x45 */
#define TWCR_REARM (B_TWINT | B_TWEA | B_TWEN | B_TWIE)  /* 0xC5 */
#define NODE_TWAR  (0x20 << 1)                           /* 0x40 */

/* TWSR status codes (upper 5 bits), slave mode */
#define S_SLA_W_RX     0x60   /* addressed for WRITE, ACKed         */
#define S_DATA_RX_ACK  0x80   /* data byte arrived, ACKed           */
#define S_STOP_RSTART  0xA0   /* STOP or repeated START seen        */
#define S_SLA_R_RX     0xA8   /* addressed for READ, ACKed          */
#define S_DATA_TX_ACK  0xB8   /* byte sent, master wants more       */
#define S_DATA_TX_NACK 0xC0   /* byte sent, master is done          */
#define S_LAST_TX_ACK  0xC8   /* last buffered byte sent, ACKed     */
#define S_BUS_ERROR    0x00   /* illegal START/STOP on the bus      */

unsigned char rx_buf[2];               /* incoming command  */
unsigned char tx_buf[4];               /* outgoing telemetry */
unsigned char rx_idx = 0, tx_idx = 0;

/* ============================================================
   SECTION 3 — debug instrumentation state
   ============================================================ */
volatile unsigned char twi_evt_count = 0;  /* ++ on every TWI ISR entry */
volatile unsigned char twi_last_stat = 0;  /* last TWSR status seen     */

#define LED_ON()  (PORTB |=  0x01)
#define LED_OFF() (PORTB &= ~0x01)

void led_blip(unsigned char ms_on)
{
    LED_ON(); delay_ms(ms_on); LED_OFF();
}

/* Blink one byte, MSB first.
   LONG flash (400 ms)  = bit is 1
   short blip (80 ms)   = bit is 0
   300 ms dark gap between bits.
   Total ~4-6 s per byte: slow on purpose - film it or count live. */
void led_dump_byte(unsigned char b)
{
    unsigned char i;
    for (i = 0; i < 8; i++)
    {
        LED_ON();
        if (b & 0x80) delay_ms(400);
        else          delay_ms(80);
        LED_OFF();
        delay_ms(300);
        b <<= 1;
    }
}

/* Terminal state when the register probe fails.
   IMPORTANT: reads the registers FRESH on every cycle -
   if something is rewriting them periodically, consecutive
   dump cycles will show DIFFERENT values. That itself is data. */
void probe_fail_dump(void)
{
    unsigned char i;
    while (1)
    {
        for (i = 0; i < 5; i++) { led_blip(40); delay_ms(80); }  /* preamble */
        delay_ms(800);

        led_dump_byte(TWCR);   /* byte 1 */
        delay_ms(1200);
        led_dump_byte(TWAR);   /* byte 2 */
        delay_ms(2500);        /* long gap, then repeat */
    }
}

/* The probe itself, one place, used at every checkpoint.
   Masks TWCR with the three armed bits (TWINT may legitimately
   be set at read time - that is not a failure).               */
unsigned char twi_regs_ok(void)
{
    if ((TWCR & TWCR_ARMED) != TWCR_ARMED) return 0;
    if (TWAR != NODE_TWAR)                 return 0;
    return 1;
}

/* ============================================================
   SECTION 4 — interrupt service routines
   ============================================================ */
interrupt [EXT_INT0] void ext_int0_isr(void)
{
    live_tach_count++;                 /* one pulse from the fan tach */
}

interrupt [TIM1_COMPA] void timer1_compa_isr(void)
{
    /* 1 Hz: snapshot tach count for the I2C payload.
       ISR-to-ISR is safe: AVR does not nest interrupts by default. */
    node_tel.tach_pulses = live_tach_count;
    live_tach_count = 0;
}

interrupt [TWI] void twi_isr(void)
{
    twi_evt_count++;                   /* debug: proves the ISR ran */
    twi_last_stat = TWSR & 0xF8;

    switch (twi_last_stat)
    {
    /* ---- master is WRITING a command to us ---- */
    case S_SLA_W_RX:                   /* transfer opens: reset index */
        rx_idx = 0;
        break;

    case S_DATA_RX_ACK:                /* one payload byte arrived */
        if (rx_idx < sizeof(rx_buf)) rx_buf[rx_idx++] = TWDR;
        /* PLACEHOLDER: bytes beyond 2 are discarded but still ACKed.
           Real policy = your open design decision.                 */
        break;

    case S_STOP_RSTART:                /* frame closed */
        /* PLACEHOLDER: apply only a complete 2-byte command.
           Eager-vs-at-STOP tradeoff = your open design decision.   */
        if (rx_idx == sizeof(I2C_Command)) {
            node_cmd = *((I2C_Command*)rx_buf);
            OCR0 = node_cmd.heater_pwm;
            OCR2 = node_cmd.fan_pwm;
        }
        break;

    /* ---- master is READING telemetry from us ---- */
    case S_SLA_R_RX:                   /* transfer opens: snapshot + first byte */
        /* Safe copy: global interrupts are OFF inside this ISR,
           so TIM1 cannot modify node_tel mid-copy. The main-loop
           adc_raw store is the one that can still tear (ledger).  */
        *((I2C_Telemetry*)tx_buf) = node_tel;
        tx_idx = 0;
        TWDR = tx_buf[tx_idx++];
        break;

    case S_DATA_TX_ACK:                /* master wants the next byte */
        /* PLACEHOLDER: over-read pads 0xFF = your open decision.   */
        TWDR = (tx_idx < sizeof(tx_buf)) ? tx_buf[tx_idx++] : 0xFF;
        break;

    case S_DATA_TX_NACK:               /* master done reading */
    case S_LAST_TX_ACK:
        break;

    /* ---- fault ---- */
    case S_BUS_ERROR:
        /* Datasheet recovery: set TWSTO+TWINT (no STOP actually
           driven in slave mode - it just resets the TWI logic).   */
        TWCR = B_TWINT | B_TWSTO | TWCR_ARMED;
        return;
    }

    TWCR = TWCR_REARM;                 /* clear TWINT, keep ACKing */
}

/* ============================================================
   SECTION 5 — ADC (polled, AVCC reference, 125 kHz clock)
   ============================================================ */
#define ADC_VREF_TYPE ((0<<REFS1) | (1<<REFS0) | (0<<ADLAR))

unsigned int read_adc(unsigned char ch)
{
    ADMUX = ch | ADC_VREF_TYPE;
    delay_us(10);                      /* mux settle */
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
    unsigned char last_evt = 0;
    unsigned int  loop_ct  = 0;
    unsigned int  raw;

    /* ---- 6.1 ports ----
       PB0 = debug LED   PB3 = OC0 heater PWM
       PD7 = OC2 fan PWM PD2 = INT0 tach input, pull-up on */
    DDRB  = (1<<DDB3) | (1<<DDB0);
    PORTB = 0x00;
    DDRA  = 0x00;  PORTA = 0x00;
    DDRC  = 0x00;  PORTC = 0x00;   /* PC0/PC1 = TWI, inputs, no pull-ups
                                      (bus has its own resistors)       */
    DDRD  = (1<<DDD7);
    PORTD = (1<<PORTD2);

    /* ---- 6.2 timers ---- */
    /* Timer0: heater PWM. Fast PWM, non-inverting, 8MHz/64 -> ~488 Hz */
    TCCR0 = (1<<WGM00)|(1<<COM01)|(1<<WGM01)|(1<<CS01)|(1<<CS00);
    TCNT0 = 0; OCR0 = 0;

    /* Timer1: 1 Hz CTC for the tach snapshot. 8MHz/256, OCR1A=31249 */
    TCCR1A = 0;
    TCCR1B = (1<<WGM12)|(1<<CS12);
    TCNT1H = 0; TCNT1L = 0;
    OCR1AH = 0x7A; OCR1AL = 0x11;      /* 31249 = 0x7A11 */

    /* Timer2: fan PWM. Fast PWM, non-inverting, no prescale -> 31.25 kHz.
       HAND-SET: a CodeWizard regeneration will silently revert this.   */
    ASSR  = 0;
    TCCR2 = (1<<WGM20)|(1<<COM21)|(1<<WGM21)|(1<<CS20);
    TCNT2 = 0; OCR2 = 0;

    TIMSK = (1<<OCIE1A);               /* only Timer1 compare A IRQ */

    /* ---- 6.3 external interrupt: tach on INT0, rising edge ---- */
    GICR |= (1<<INT0);
    MCUCR = (1<<ISC01)|(1<<ISC00);
    GIFR  = (1<<INTF0);

    /* ---- 6.4 ADC on ---- */
    ADMUX  = ADC_VREF_TYPE;
    ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1);   /* /64 -> 125 kHz */
    SFIOR  = 0;

    /* ---- 6.5 TWI slave arm (bare metal, the heart of the test) ---- */
    TWAR = NODE_TWAR;                  /* who I answer to: 0x40 on wire */
    TWCR = TWCR_ARMED;                 /* TWEA+TWEN+TWIE: ACK my address */

    #asm("sei")

    /* ---- 6.6 CHECKPOINT A: boot signature ----
       Proves THIS image is what's running. Old node firmware never
       touches PB0; no other build flashes 3 times at power-up.      */
    for (i = 0; i < 3; i++) { led_blip(60); delay_ms(140); }
    delay_ms(600);

    /* ---- 6.7 CHECKPOINT B: register probe, immediately after arming.
       If this fails, we go straight to the dump loop and blink out
       what the registers ACTUALLY hold. This is the measurement.    */
    if (!twi_regs_ok()) probe_fail_dump();

    adc_ema = (float)read_adc(0);      /* seed EMA, no slow ramp */

    /* ---- 6.8 main loop: sample NTC + CHECKPOINT C (continuous) ---- */
    while (1)
    {
        raw = read_adc(0);
        adc_ema = adc_ema + 0.05 * ((float)raw - adc_ema);

        /* OWED (ledger): 16-bit store, tearable by the TWI ISR */
        node_tel.adc_raw = (unsigned int)(adc_ema + 0.5);

        delay_ms(10);
        loop_ct++;

        /* CHECKPOINT C: re-probe every pass - catches anything that
           clobbers the registers LATER rather than at init.         */
        if (!twi_regs_ok()) probe_fail_dump();

        /* TWI activity indicator: double-blip per burst of traffic  */
        if (twi_evt_count != last_evt) {
            last_evt = twi_evt_count;
            led_blip(30); delay_ms(60); led_blip(30);
        }
        else if (loop_ct >= 200) {     /* ~2 s heartbeat when idle */
            loop_ct = 0;
            led_blip(30);
        }
    }
}