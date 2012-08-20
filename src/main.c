#include <stdint.h>
#include <avr/io.h>


#define CONFIG_UART 1


#if CONFIG_UART /* uart */

static inline void set_baud_rate(long baud)
{
  uint16_t UBRR0_value = ((F_CPU / 16 + baud / 2) / baud - 1);
  UBRR0H = UBRR0_value >> 8;
  UBRR0L = UBRR0_value;
}

static void uart_setup(void)
{
  /* #define CONFIG_FOSC (F_CPU * 2) */
  /* const uint16_t x = CONFIG_FOSC / (16 * BAUDS) - 1; */
#if 0 /* (bauds == 9600) */
  const uint16_t x = 206;
#elif 0 /* (bauds == 115200) */
  const uint16_t x = 16;
#elif 0 /* (bauds == 500000) */
  const uint16_t x = 3;
#elif 0 /* (bauds == 1000000) */
  const uint16_t x = 1;
#endif

  set_baud_rate(9600);

  /* baud doubler off  - Only needed on Uno XXX */
  UCSR0A &= ~(1 << U2X0);

  UCSR0B = 1 << TXEN0;

  /* default to 8n1 framing */
  UCSR0C = (3 << 1);
}

static void uart_write(const uint8_t* s, uint8_t n)
{
  for (; n; --n, ++s)
  {
    /* wait for transmit buffer to be empty */
    while (!(UCSR0A & (1 << UDRE0))) ;
    UDR0 = *s;
  }

  /* wait for last byte to be sent */
  while ((UCSR0A & (1 << 6)) == 0) ;
}

static inline uint8_t nibble(uint32_t x, uint8_t i)
{
  return (x >> (i * 4)) & 0xf;
}

static inline uint8_t hex(uint8_t x)
{
  return (x >= 0xa) ? 'a' + x - 0xa : '0' + x;
}

static void uart_write_hex(uint8_t* s, uint8_t n)
{
  uint8_t buf[2];

  for (; n; --n, ++s)
  {
    buf[1] = hex(nibble(*s, 0));
    buf[0] = hex(nibble(*s, 1));
    uart_write(buf, 2);
  }
}

static void uart_write_string(const char* s)
{
  for (; *s; ++s) uart_write((const uint8_t*)s, 1);
}

#define XSTR(__s) STR(__s)
#define STR(__s) #__s
#define PRINT_FAIL() uart_write_string(XSTR(__LINE__) ": fail\r\n")
#define PRINT_PASS() uart_write_string(XSTR(__LINE__) ": pass\r\n")

#endif /* CONFIG_UART */


/* spi module, used by sd */

static inline void spi_setup_master(void)
{
  /* doc8161.pdf, ch.18 */

  /* cs, pb2 */
  DDRB |= (1 << 2);
  PORTB |= 1 << 2;

  /* spi output pins: sck pb5, mosi pb3 */
  DDRB |= (1 << 5) | (1 << 3);

  /* spi input pins: miso pb4 */
  DDRB &= ~(1 << 4);
  /* disable pullup (already by default) */
  PORTB &= ~(1 << 4);

  /* enable spi, msb first, master, freq / 128 (125khz), sck low idle */
  SPCR = (1 << SPE) | (1 << MSTR) | (3 << SPR0);

  /* clear double speed */
  SPSR &= ~(1 << SPI2X);
}

static inline void spi_write_uint8(uint8_t x)
{
  /* write the byte and wait for transmission */

  /* while ((SPSR & (1 << SPIF)) == 1) ; */

  SPDR = x;

#if 1
  if (SPSR & (1 << WCOL))
  {
#if 1
    PRINT_FAIL();
#else
    /* access SPDR */
    volatile uint8_t fubar = SPDR;
    __asm__ __volatile__ ("" :"=m"(fubar));
    goto redo;
#endif
  }
#endif

  while ((SPSR & (1 << SPIF)) == 0) ;
}

static void spi_write(const uint8_t* s, uint8_t len)
{
  for (; len; --len, ++s) spi_write_uint8(*s);
}

static inline void spi_read_uint8(uint8_t* x)
{
  /* by writing to mosi, 8 clock pulses are generated
     allowing the slave to transmit its register on miso
   */
  spi_write_uint8(0xff);
  *x = SPDR;
}

static void spi_read(uint8_t* s, uint8_t len)
{
  for (; len; --len, ++s) spi_read_uint8(s);
}


/* sdcard */

/* some notes from specs */
/* host is the master, can communicate in ptop or broadcast. */
/* only supports sd card (not sdxc, sdhc ...) */
/* the data line must be high by the pullup */

#define SD_CMD_SIZE 6
static uint8_t sd_cmd_buf[SD_CMD_SIZE];

#define SD_INFO_V2 (1 << 0)
#define SD_INFO_SDHC (1 << 1)
static uint8_t sd_info = 0;

static inline void sd_make_cmd
(uint8_t op, uint8_t a, uint8_t b, uint8_t c, uint8_t d, uint8_t crc)
{
  /* start bit, command index */
  sd_cmd_buf[0] = (1 << 6) | op;

  /* argument */
  sd_cmd_buf[1] = a;
  sd_cmd_buf[2] = b;
  sd_cmd_buf[3] = c;
  sd_cmd_buf[4] = d;

  /* crc, stop bit */
  sd_cmd_buf[5] = (crc << 1) | 1;
}

static inline void sd_ss_high(void)
{
  PORTB |= 1 << 2;
}

static inline void sd_ss_low(void)
{
  PORTB &= ~(1 << 2);
}

static inline void sd_write_cmd(void)
{
  /* select the slave, write command */
  sd_ss_high();
  sd_ss_low();
  spi_write(sd_cmd_buf, SD_CMD_SIZE);
}

static int sd_read_r1(void)
{
  uint16_t i;

  /* read reply into sd_cmd_buf, r1 format */
  for (i = 0; i < 10000; ++i)
  {
    spi_read_uint8(&sd_cmd_buf[0]);
    /* 0xff means no data transfered (test msb 0) */
    if ((sd_cmd_buf[0] & 0x80) == 0) return 0;
  }

  return -1;
}

static int sd_read_r3(void)
{
  if (sd_read_r1() == -1) return -1;
  /* illegal command, dont read remaining bytes */
  if (sd_cmd_buf[0] & (1 << 2)) return 0;
  spi_read(sd_cmd_buf + 1, 4);
  return 0;
}

static inline int sd_read_r7(void)
{
  /* same length as r3 */
  return sd_read_r7();
}

static int sd_setup(void)
{
  /* sd initialization sequence */

  uint8_t i;

  spi_setup_master();

  /* send at least 74 warmup pulses. cs must be high. */
  sd_ss_high();
  for (i = 0; i < 10; ++i) spi_write_uint8(0xff);

  /* enter spi mode */
  /* cs low, send cmd0 (reset), check error and idle state */
  for (i = 0; i < 0xff; ++i)
  {
    sd_make_cmd(0x00, 0x00, 0x00, 0x00, 0x00, 0x4a);
    sd_write_cmd();

    if (sd_read_r1() != -1)
    {
      /* wait for in_idle_state == 1 */
      if (sd_cmd_buf[0] & (1 << 0)) break ;
    }
  }
  if (i == 0xff) { PRINT_FAIL(); return -1; }

  PRINT_PASS();
  uart_write_hex(sd_cmd_buf, 1);
  uart_write_string("\r\n");

  /* cmd8 (send_if_cond) */
  sd_make_cmd(0x08, 0x00, 0x00, 0x01, 0xaa, 0x43);
  sd_write_cmd();
  /* card echos back voltage and check pattern */
  if (sd_read_r7() == -1) { PRINT_FAIL(); return -1; }

  uart_write_hex(sd_cmd_buf, 5);
  uart_write_string("\r\n");

  /* illegal command */
  if (sd_cmd_buf[0] & (1 << 2))
  {
    /* sd version 1 */
    /* check for other errors */
    if (sd_cmd_buf[0] & ~(1 << 2)) { PRINT_FAIL(); return -1; }
  }
  else
  {
    /* sd version 2 */
    /* check errors, pattern, voltage (2.7v-3.6v) */
    if (sd_cmd_buf[0] & 0xfe) { PRINT_FAIL(); return -1; }
    if (sd_cmd_buf[4] != 0xaa) { PRINT_FAIL(); return -1; }
    if ((sd_cmd_buf[3] & 0xf) != 0x01) { PRINT_FAIL(); return -1; }
    sd_info |= SD_INFO_V2;
  }

  /* cmd58 (read ocr operation condition register) for voltages */
  sd_make_cmd(0x3a, 0x00, 0x00, 0x00, 0x00, 0xff);
  sd_write_cmd();
  if (sd_read_r3()) { PRINT_FAIL(); return -1; }
  /* accept 3.3v - 3.6v */
  if ((sd_cmd_buf[3] >> 4) == 0) { PRINT_FAIL(); return -1; }

  /* acmd41, wait for in_idle_state */
  while (1)
  {
    sd_make_cmd(0x29, 0x00, 0x00, 0x00, 0x00, 0xff);
    /* enable sdhc is v2 */
    if (sd_info & SD_INFO_V2) sd_cmd_buf[4] |= 1 << 7;
    sd_write_cmd();
    if (sd_read_r1() == -1) { PRINT_FAIL(); return -1; }
    /* wait for in_idle_state == 0 */
    if ((sd_cmd_buf[0] & (1 << 0)) == 0) break ;
  }

  /* get sdhc status */
  if (sd_info & SD_INFO_V2)
  {
    /* cmd58 (get ccs) */
    sd_make_cmd(0x3a, 0x00, 0x00, 0x00, 0x00, 0xff);
    sd_write_cmd();
    if (sd_read_r3()) { PRINT_FAIL(); return -1; }
    if (sd_cmd_buf[4] & (1 << 6)) sd_info |= SD_INFO_SDHC;
  }

  return 0;
}


/* main */

int main(void)
{
  /* delay loop */
  {
    volatile uint16_t i;
    for (i = 0; i < 10000; ++i) __asm__ __volatile__ ("nop\n\t");
    for (i = 0; i < 10000; ++i) __asm__ __volatile__ ("nop\n\t");
    for (i = 0; i < 10000; ++i) __asm__ __volatile__ ("nop\n\t");
    for (i = 0; i < 10000; ++i) __asm__ __volatile__ ("nop\n\t");
    for (i = 0; i < 10000; ++i) __asm__ __volatile__ ("nop\n\t");
    for (i = 0; i < 10000; ++i) __asm__ __volatile__ ("nop\n\t");
    for (i = 0; i < 10000; ++i) __asm__ __volatile__ ("nop\n\t");
    for (i = 0; i < 10000; ++i) __asm__ __volatile__ ("nop\n\t");
    for (i = 0; i < 10000; ++i) __asm__ __volatile__ ("nop\n\t");
    for (i = 0; i < 10000; ++i) __asm__ __volatile__ ("nop\n\t");
    for (i = 0; i < 10000; ++i) __asm__ __volatile__ ("nop\n\t");
    for (i = 0; i < 10000; ++i) __asm__ __volatile__ ("nop\n\t");
    for (i = 0; i < 10000; ++i) __asm__ __volatile__ ("nop\n\t");
    for (i = 0; i < 10000; ++i) __asm__ __volatile__ ("nop\n\t");
    for (i = 0; i < 10000; ++i) __asm__ __volatile__ ("nop\n\t");
    for (i = 0; i < 10000; ++i) __asm__ __volatile__ ("nop\n\t");
    for (i = 0; i < 10000; ++i) __asm__ __volatile__ ("nop\n\t");
    for (i = 0; i < 10000; ++i) __asm__ __volatile__ ("nop\n\t");
    for (i = 0; i < 10000; ++i) __asm__ __volatile__ ("nop\n\t");
    for (i = 0; i < 10000; ++i) __asm__ __volatile__ ("nop\n\t");
    for (i = 0; i < 10000; ++i) __asm__ __volatile__ ("nop\n\t");
    for (i = 0; i < 10000; ++i) __asm__ __volatile__ ("nop\n\t");
    for (i = 0; i < 10000; ++i) __asm__ __volatile__ ("nop\n\t");
    for (i = 0; i < 10000; ++i) __asm__ __volatile__ ("nop\n\t");
    for (i = 0; i < 10000; ++i) __asm__ __volatile__ ("nop\n\t");
    for (i = 0; i < 10000; ++i) __asm__ __volatile__ ("nop\n\t");
    for (i = 0; i < 10000; ++i) __asm__ __volatile__ ("nop\n\t");
    for (i = 0; i < 10000; ++i) __asm__ __volatile__ ("nop\n\t");
    for (i = 0; i < 10000; ++i) __asm__ __volatile__ ("nop\n\t");
    for (i = 0; i < 10000; ++i) __asm__ __volatile__ ("nop\n\t");
    for (i = 0; i < 10000; ++i) __asm__ __volatile__ ("nop\n\t");
    for (i = 0; i < 10000; ++i) __asm__ __volatile__ ("nop\n\t");
  }

  uart_setup();

  uart_write_string("hi\r\n");

  if (sd_setup() == -1)
  {
    uart_write_string("sd_setup() == -1\r\n");
  }
  else
  {
    uart_write_string("sd_setup() == 0\r\n");
  }

  return 0;
}
