#include <stdint.h>
#include <avr/io.h>


/* on 8 bit mcus, -mint8 not enabled by default and sizeof(int) == 2.
   we force the use the of register sized integer values via regtype_t
   to avoid useless code generation, and size reduction.
 */
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__)
typedef int8_t regtype_t;
typedef uint8_t uregtype_t;
#define CONFIG_SIZEOF_REGTYPE 1
#else
# error "unknown architecture"
#endif


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

#endif /* CONFIG_UART */


#define CONFIG_SD_DEBUG 1
#if CONFIG_SD_DEBUG
#define XSTR(__s) STR(__s)
#define STR(__s) #__s
#define PRINT_FAIL() uart_write_string(XSTR(__LINE__) ": fail\r\n")
#define PRINT_PASS() uart_write_string(XSTR(__LINE__) ": pass\r\n")
#define PRINT_BUF(__p, __n)				\
do {							\
  uart_write_string(XSTR(__LINE__) ": buf\r\n");	\
  uart_write_hex(__p, __n);				\
  uart_write_string("\r\n");				\
} while (0)
#else
#define PRINT_FAIL()
#define PRINT_PASS()
#define PRINT_BUF(__p, __n)
#endif /* CONFIG_SD_DEBUG */


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

static inline void spi_set_sck_freq(uint8_t x)
{
  /* x one of SPI_SCK_FREQ_FOSCX */
  /* where spi sck = fosc / X */
  /* see atmega328 specs, table 18.5 */
#define SPI_SCK_FREQ_FOSC2 ((1 << 2) | 0)
#define SPI_SCK_FREQ_FOSC4 ((0 << 2) | 0)
#define SPI_SCK_FREQ_FOSC8 ((1 << 2) | 1)
#define SPI_SCK_FREQ_FOSC16 ((0 << 2) | 1)
#define SPI_SCK_FREQ_FOSC32 ((1 << 2) | 2)
#define SPI_SCK_FREQ_FOSC64 ((0 << 2) | 2)
#define SPI_SCK_FREQ_FOSC128 ((0 << 2) | 3)

  SPCR &= ~(3 << SPR0);
  SPCR |= (x & 3) << SPR0;

  SPSR &= ~(1 << SPI2X);
  SPSR |= (((x >> 2) & 1) << SPI2X);
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

static inline uint8_t spi_read_uint8(void)
{
  /* by writing to mosi, 8 clock pulses are generated
     allowing the slave to transmit its register on miso
   */
  spi_write_uint8(0xff);
  return SPDR;
}

static void spi_read(uint8_t* s, regtype_t len)
{
  for (; len; --len, ++s) *s = spi_read_uint8();
}

#if (CONFIG_SIZEOF_REGTYPE == 1)

static inline void spi_read_512(uint8_t* s)
{
  /* spi_read len argument is uint8_t, too small for 512.
     we do not want to use a uint16_t on 8 bit mcus.
   */

  spi_read(s + 0x000, 0xff);
  spi_read(s + 0x0ff, 0xff);
  spi_read(s + 0x1fe, 0x02);
}

static inline void spi_write_512(const uint8_t* s)
{
  spi_write(s + 0x000, 0xff);
  spi_write(s + 0x0ff, 0xff);
  spi_write(s + 0x1fe, 0x02);
}

#else /* non 8 bits mcus */

static inline void spi_read_512(uint8_t* s)
{
  return spi_read(s, 512);
}

static inline void spi_write_512(const uint8_t* s)
{
  return spi_write(s, 512);
}

#endif /* CONFIG_REGTYPE_8 */


/* sdcard */

/* some notes from specs */
/* host is the master, can communicate in ptop or broadcast. */
/* only supports sd card (not sdxc, sdhc ...) */
/* the data line must be high by the pullup */

#define SD_RESP_SIZE 5
static uint8_t sd_resp_buf[SD_RESP_SIZE];

#define SD_BLOCK_SIZE 512
static uint8_t sd_block_buf[SD_BLOCK_SIZE];

#define SD_INFO_V2 (1 << 0)
#define SD_INFO_SDHC (1 << 1)
#define SD_INFO_WP (1 << 2)
static uint8_t sd_info = 0;

static inline void sd_ss_high(void)
{
  PORTB |= 1 << 2;
}

static inline void sd_ss_low(void)
{
  PORTB &= ~(1 << 2);
}

static inline void sd_wait_not_busy(void)
{
  /* TODO: timeout */
  while (spi_read_uint8() != 0xff) ;
}

static void sd_write_cmd
(uint8_t op, uint8_t a, uint8_t b, uint8_t c, uint8_t d, uint8_t crc)
{
  /* select the slave */
  sd_ss_high();
  sd_ss_low();

  /* wont work otherwise */
  sd_wait_not_busy();

  /* write the 6 bytes command */

  /* start bit, command index */
  spi_write_uint8((1 << 6) | op);

  /* argument */
  spi_write_uint8(a);
  spi_write_uint8(b);
  spi_write_uint8(c);
  spi_write_uint8(d);

  /* crc, stop bit */
  spi_write_uint8((crc << 1) | 1);
}

static inline void sd_write_cmd_32(uint8_t op, uint32_t x, uint8_t crc)
{
  const uint8_t a = (x >> 24) & 0xff;
  const uint8_t b = (x >> 16) & 0xff;
  const uint8_t c = (x >>  8) & 0xff;
  const uint8_t d = (x >>  0) & 0xff;

  sd_write_cmd(op, a, b, c, d, crc);
}

static regtype_t sd_read_r1(void)
{
  /* TODO: reduce to regtype_t if possible */
  uint16_t i;

  /* read reply into sd_resp_buf, r1 format */
  for (i = 0; i < 10000; ++i)
  {
    sd_resp_buf[0] = spi_read_uint8();
    /* 0xff means no data transfered (test msb 0) */
    if ((sd_resp_buf[0] & 0x80) == 0) return 0;
  }

  return -1;
}

static regtype_t sd_read_r1b(void)
{
  if (sd_read_r1()) return -1;
  /* busy signal, wait for non 0 value (card ready) */
  while (spi_read_uint8() == 0) ;
  return 0;
}

static regtype_t sd_read_r2(void)
{
  if (sd_read_r1()) return -1;
  if (sd_resp_buf[0] & (1 << 2)) return 0;
  sd_resp_buf[1] = spi_read_uint8();
  return 0;
}

static regtype_t sd_read_r3(void)
{
  if (sd_read_r1()) return -1;
  /* illegal command, dont read remaining bytes */
  if (sd_resp_buf[0] & (1 << 2)) return 0;
  spi_read(sd_resp_buf + 1, 4);
  return 0;
}

static inline regtype_t sd_read_r7(void)
{
  /* same length as r3 */
  return sd_read_r3();
}

__attribute__((unused)) static regtype_t sd_read_csd(void)
{
  /* in spi mode, the card responds with a response
     token followed by a data block of 16 bytes and
     a 2 bytes crc.
   */

  /* TODO: redundant with sd_read_block */

  /* read application card specific data */
  sd_write_cmd(0x09, 0x00, 0x00, 0x00, 0x00, 0xff);
  if (sd_read_r1() || sd_resp_buf[0]) return -1;

  /* read response token */
  while (spi_read_uint8() != 0xfe) ;

  /* read the data block */
  spi_read_512(sd_block_buf);

  /* skip 2 bytes crc */
  spi_read_uint8();
  spi_read_uint8();

  return 0;
}

#if CONFIG_UART

__attribute__((unused)) static void sd_print_csd(void)
{
  struct csd_v1
  {
    /* byte 0 */
    unsigned reserved1 : 6;
    unsigned csd_ver : 2;
    /* byte 1 */
    uint8_t taac;
    /* byte 2 */
    uint8_t nsac;
    /* byte 3 */
    uint8_t tran_speed;
    /* byte 4 */
    uint8_t ccc_high;
    /* byte 5 */
    unsigned read_bl_len : 4;
    unsigned ccc_low : 4;
    /* byte 6 */
    unsigned c_size_high : 2;
    unsigned reserved2 : 2;
    unsigned dsr_imp : 1;
    unsigned read_blk_misalign :1;
    unsigned write_blk_misalign : 1;
    unsigned read_bl_partial : 1;
    /* byte 7 */
    uint8_t c_size_mid;
    /* byte 8 */
    unsigned vdd_r_curr_max : 3;
    unsigned vdd_r_curr_min : 3;
    unsigned c_size_low :2;
    /* byte 9 */
    unsigned c_size_mult_high : 2;
    unsigned vdd_w_cur_max : 3;
    unsigned vdd_w_curr_min : 3;
    /* byte 10 */
    unsigned sector_size_high : 6;
    unsigned erase_blk_en : 1;
    unsigned c_size_mult_low : 1;
    /* byte 11 */
    unsigned wp_grp_size : 7;
    unsigned sector_size_low : 1;
    /* byte 12 */
    unsigned write_bl_len_high : 2;
    unsigned r2w_factor : 3;
    unsigned reserved3 : 2;
    unsigned wp_grp_enable : 1;
    /* byte 13 */
    unsigned reserved4 : 5;
    unsigned write_partial : 1;
    unsigned write_bl_len_low : 2;
    /* byte 14 */
    unsigned reserved5: 2;
    unsigned file_format : 2;
    unsigned tmp_write_protect : 1;
    unsigned perm_write_protect : 1;
    unsigned copy : 1;
    unsigned file_format_grp : 1;
    /* byte 15 */
    unsigned always1 : 1;
    unsigned crc : 7;
  } __attribute__((packed)) *csd = (struct csd_v1*)sd_block_buf;

  uint8_t x;

  uart_write_string("csd: ");
  uart_write_hex(sd_block_buf, 16);
  uart_write_string("\r\n");

  uart_write_string("vers: ");
  x = csd->csd_ver;
  uart_write_hex(&x, 1);
  uart_write_string("\r\n");

  x = csd->read_bl_len;
  uart_write_string("bl_len: ");
  uart_write_hex(&x, 1);
  uart_write_string("\r\n");

  uart_write_string("csize: ");
  x = csd->c_size_high;
  uart_write_hex(&x, 1);
  x = csd->c_size_mid;
  uart_write_hex(&x, 1);
  x = csd->c_size_low;
  uart_write_hex(&x, 1);
  uart_write_string("\r\n");
}

#endif /* CONFIG_UART */

__attribute__((unused))
static regtype_t sd_write_csd(void)
{
  sd_write_cmd(0x1c, 0x00, 0x00, 0x00, 0x00, 0xff);
  if (sd_read_r1() || sd_resp_buf[0]) return -1;
  return 0;
}

static inline uint32_t sd_bid_to_baddr(uint32_t bid)
{
  return ((sd_info & SD_INFO_SDHC) == 0) ? bid *= 512 : bid;
}

__attribute__((unused))
static regtype_t sd_read_block(uint32_t bid)
{
  /* block at addr is read in sd_block_buf */

  const uint32_t baddr = sd_bid_to_baddr(bid);

  /* cmd17, read single block */
  sd_write_cmd_32(0x11, baddr, 0xff);
  if (sd_read_r1() || sd_resp_buf[0]) { PRINT_FAIL(); return -1; }

  /* response token */
  while (spi_read_uint8() != 0xfe) ;

  /* data block */
  spi_read_512(sd_block_buf);

  /* skip 2 bytes crc */
  spi_read_uint8();
  spi_read_uint8();

  return 0;
}

__attribute__((unused))
static regtype_t sd_erase(uint32_t bid, uint32_t n)
{
  /* bid the index of the first block to erase */
  /* n the block count */

  uint32_t baddr;

  /* cmd32, erase_wr_blk_start_addr */
  baddr = sd_bid_to_baddr(bid);
  sd_write_cmd_32(0x20, baddr, 0xff);
  if (sd_read_r1() || sd_resp_buf[0]) { PRINT_FAIL(); return -1; }

  /* cmd33, erase_wr_blk_start_addr */
  baddr = sd_bid_to_baddr(bid + n - 1);
  sd_write_cmd_32(0x21, baddr, 0xff);
  if (sd_read_r1() || sd_resp_buf[0]) { PRINT_FAIL(); return -1; }

  /* cmd38, erase */
  sd_write_cmd(0x26, 0x00, 0x00, 0x00, 0x00, 0xff);
  if (sd_read_r1b() || sd_resp_buf[0]) { PRINT_FAIL(); return -1; }

  /* TODO: check status */

  return 0;
}

__attribute__((unused))
static regtype_t sd_write_block(uint32_t bid)
{
  /* sd_block_buf is written into block at addr */

  const uint32_t baddr = sd_bid_to_baddr(bid);

  /* cmd24, write_block */
  sd_write_cmd_32(0x18, baddr, 0xff);
  if (sd_read_r1() || sd_resp_buf[0]) { PRINT_FAIL(); return -1; }

  /* start block token */
  spi_write_uint8(0xfe);

  /* data block */
  spi_write_512(sd_block_buf);

  /* dummy crc */
  spi_write_uint8(0xff);
  spi_write_uint8(0xff);

  /* data response */
  if ((spi_read_uint8() & 0x1f) != 0x05) { PRINT_FAIL(); return -1; }

  /* busy signal */
  while (spi_read_uint8() == 0x00) ;

  /* cmd13, send_status */
  sd_write_cmd(0x0d, 0x00, 0x00, 0x00, 0x00, 0xff);
  if (sd_read_r2() || sd_resp_buf[0] || sd_resp_buf[1]) 
  {
    PRINT_FAIL();
    return -1;
  }

  return 0;
}

static regtype_t sd_setup(void)
{
  /* sd initialization sequence */

  uregtype_t i;
  uint8_t arg;

  spi_setup_master();

  /* send at least 74 warmup pulses. cs must be high. */
  sd_ss_high();
  for (i = 0; i < 10; ++i) spi_write_uint8(0xff);

  /* enter spi mode */
  /* cs low, send cmd0 (reset), check error and idle state */
  for (i = 0; i < 0xff; ++i)
  {
    sd_write_cmd(0x00, 0x00, 0x00, 0x00, 0x00, 0x4a);
    if (sd_read_r1() != -1)
    {
      /* wait for in_idle_state == 1 */
      if (sd_resp_buf[0] & (1 << 0)) break ;
    }
  }
  if (i == 0xff) { PRINT_FAIL(); return -1; }

  PRINT_BUF(sd_resp_buf, 1);

  /* cmd8 (send_if_cond) */
  sd_write_cmd(0x08, 0x00, 0x00, 0x01, 0xaa, 0x43);
  /* card echos back voltage and check pattern */
  if (sd_read_r7()) { PRINT_FAIL(); return -1; }

  PRINT_BUF(sd_resp_buf, 5);

  /* illegal command */
  if (sd_resp_buf[0] & (1 << 2))
  {
    /* sd version 1 */
    /* check for other errors */
    /* if (sd_resp_buf[0] & ~(1 << 2)) { PRINT_FAIL(); return -1; } */
  }
  else
  {
    /* sd version 2 */
    /* check errors, pattern, voltage (2.7v-3.6v) */
    if (sd_resp_buf[0] & 0xfe) { PRINT_FAIL(); return -1; }
    if (sd_resp_buf[4] != 0xaa) { PRINT_FAIL(); return -1; }
    if ((sd_resp_buf[3] & 0xf) != 0x01) { PRINT_FAIL(); return -1; }
    sd_info |= SD_INFO_V2;
  }

#if 0 /* fixme: optionnal, but should work */
  /* cmd58 (read ocr operation condition register) for voltages */
  sd_write_cmd(0x3a, 0x00, 0x00, 0x00, 0x00, 0xff);
  if (sd_read_r3()) { PRINT_FAIL(); return -1; }
  /* accept 3.3v - 3.6v */
  PRINT_BUF(sd_resp_buf, 5);
  if ((sd_resp_buf[3] >> 4) == 0) { PRINT_FAIL(); return -1; }
#endif

  PRINT_PASS();

  /* acmd41, wait for in_idle_state == 0 */

  /* enable sdhc is v2 */
  arg = (sd_info & SD_INFO_V2) ? (1 << 6) : 0;

  while (1)
  {
    /* acmd commands are preceded by cmd55 */
    sd_write_cmd(0x37, 0x00, 0x00, 0x00, 0x00, 0xff);
    if (sd_read_r1()) { PRINT_FAIL(); return -1; }

    sd_write_cmd(0x29, arg, 0x00, 0x00, 0x00, 0xff);
    if (sd_read_r1()) { PRINT_FAIL(); return -1; }

    /* wait for in_idle_state == 0 */
    if ((sd_resp_buf[0] & (1 << 0)) == 0) break ;
  }

  PRINT_PASS();

  /* get sdhc status */
  if (sd_info & SD_INFO_V2)
  {
    /* cmd58 (get ccs) */
    sd_write_cmd(0x3a, 0x00, 0x00, 0x00, 0x00, 0xff);
    if (sd_read_r3()) { PRINT_FAIL(); return -1; }
    if ((sd_resp_buf[1] & 0xc0) == 0xc0) sd_info |= SD_INFO_SDHC;
  }

  /* initialization sequence done, data transfer mode. */

  /* set block length to 512 if not sdhc */
  if ((sd_info & SD_INFO_SDHC) == 0)
  {
    sd_write_cmd(0x10, 0x00, 0x00, 0x02, 0x00, 0xff);
    if (sd_read_r1() || sd_resp_buf[0]) { PRINT_FAIL(); return -1; }
  }

#if 0 /* unused */
  /* TODO: cache card infos */
  if (sd_read_csd()) { PRINT_FAIL(); return -1; }
  sd_print_csd();

  /* TODO: disable wp, if supported and is_ronly == 0 */
  if ((sd_info & SD_INFO_WP) && (is_ronly == 0))
  {
    /* cmd29, clear_write_prot */
    sd_write_cmd(0x1d, 0x00, 0x00, 0x00, 0x00, 0xff);
    if (sd_read_r1b() || sd_resp_buf[0]) { PRINT_FAIL(); return -1; }
  }
#endif /* unused */

#if 0
  sd_set_freq();
#endif

  return 0;
}


/* main */

int main(void)
{
#if CONFIG_UART
  uart_setup();
#endif

  if (sd_setup() == -1)
  {
    uart_write_string("sd_setup() == -1\r\n");
    return -1;
  }

#if 0 /* write unit test */
  {
    regtype_t i;
    sd_read_block(0x10);
    for (i = 0; i < ('z' - 'a'); ++i) sd_block_buf[0x2a + i] = 'a' + i;
    sd_write_block(0x10);
  }
#endif /* write unit test */

#if 0 /* read unit test */
  {
    static const uint32_t bids[] = { 0x00, 0x10, 0x2a };

    regtype_t i;

    uart_write_string("sd_setup() == 0\r\n");

    for (i = 0; i < (sizeof(bids) / sizeof(bids[0])); ++i)
    {
      sd_read_block(bids[i]);

      /* print only the first 0x40 bytes */
      uart_write_hex(sd_block_buf + 0x00, 16);
      uart_write_string("\r\n");
      uart_write_hex(sd_block_buf + 0x10, 16);
      uart_write_string("\r\n");
      uart_write_hex(sd_block_buf + 0x20, 16);
      uart_write_string("\r\n");
      uart_write_hex(sd_block_buf + 0x30, 16);
      uart_write_string("\r\n");
      uart_write_string("\r\n");
    }
  }
#endif /* read unit test */

  return 0;
}
