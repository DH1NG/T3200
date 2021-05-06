/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "Studienarbeit.pio.h"
#include "hardware/i2c.h"
#include "Display.h"

#include <stdio.h>
#include <math.h>

#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "kiss_fftr.h"

//--------------------------------------------------------------------------------------------------------
// ADC FFT

// set this to determine sample rate
// 0     = 500,000 Hz
// 960   = 50,000 Hz
// 9600  = 5,000 Hz
#define CLOCK_DIV 320
#define FSAMP 150000

// Channel 0 is GPIO26
#define CAPTURE_CHANNEL 0
#define LED_PIN 25

// BE CAREFUL: anything over about 9000 here will cause things
// to silently break. The code will compile and upload, but due
// to memory issues nothing will work properly
#define NSAMP 5000

// globals
dma_channel_config cfg;
uint dma_chan;
float freqs[NSAMP];

void setup_adc_fft();
void sample(uint8_t *capture_buf);

//--------------------------------------------------------------------------------------------------------
// Für OLED Display

// Konstanten für das OLED Display
#define SET_CONTRAST 0x81
#define SET_ENTIRE_ON 0xA4
#define SET_NORM_INV 0xA6
#define SET_DISP 0xAE
#define SET_MEM_ADDR 0x20
#define SET_COL_ADDR 0x21
#define SET_PAGE_ADDR 0x22
#define SET_DISP_START_LINE 0x40
#define SET_SEG_REMAP 0xA0
#define SET_MUX_RATIO 0xA8
#define SET_COM_OUT_DIR 0xC0
#define SET_DISP_OFFSET 0xD3
#define SET_COM_PIN_CFG 0xDA // s10.1.18 page 40
#define SET_DISP_CLK_DIV 0xD5
#define SET_PRECHARGE 0xD9
#define SET_VCOM_DESEL 0xDB
#define SET_CHARGE_PUMP 0x8D

#define I2C_PORT i2c0 // I2C Port auf GPIO 4 /5

static uint8_t height = 32;
const uint8_t SID = 0x3C; // different height displays have different addr
const uint8_t width = 128;
//const uint8_t SID = (height == 64) ? 0x3C : 0x3D; // different height displays have different addr
//const int pages = height / 8;
//const bool external_vcc = false;

// Display Funktionen
//--------------------------------------------------------------------------------------------------------
uint8_t scr[1025];
static int cursorx = 0, cursory = 0;

int pages();
void write_cmd(uint8_t cmd);
void fill_scr(uint8_t v);
void send_data(uint8_t *data, int nbytes);
void send2(uint8_t v1, uint8_t v2);
void show_scr();
void write_cmd(uint8_t cmd);
void poweroff();
void poweron();
void contrast(uint8_t contrast);
void invert(uint8_t invert);
static void init_i2c();
void init_display();
void draw_pixel(int16_t x, int16_t y, int color);
void drawBitmap(int16_t x, int16_t y, const uint8_t bitmap[], int16_t w, int16_t h, uint16_t color);
void draw_letter_at(uint8_t x, uint8_t y, char c);
void ssd1306_print(const char *str);
void setCursorx(int x);
void setCursory(int y);
//--------------------------------------------------------------------------------------------------------

// PIO Funktionen
void Rechteck_Burst(PIO pio_obj, uint Statemachine, uint offset, uint pin, uint freq, uint Burstcount);
void blink_pin_forever(PIO pio, uint sm, uint offset, uint pin, uint freq);

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
// MAIN
int main()
{
  // I2C und Display Init
  init_i2c();
  init_display();

  ssd1306_print("Hallo Welt!"); // demonstrate some text
  show_scr();
  sleep_ms(1000);

  //ADC FFT Variablen INIT
  uint8_t cap_buf[NSAMP];
  kiss_fft_scalar fft_in[NSAMP]; // kiss_fft_scalar is a float
  kiss_fft_cpx fft_out[NSAMP];
  kiss_fftr_cfg cfg = kiss_fftr_alloc(NSAMP, false, 0, 0);

  setup_adc_fft(); // setup ports and outputs

  while (1) // Endlosschleife
  {
    //fill_scr(0);
    setCursorx(0);
    setCursory(0);
    ssd1306_print("                                             "); // ToDo OLED Textzeile eleganter leeren
    setCursorx(0);
    ssd1306_print("Sample"); // demonstrate some text
    show_scr();
    //sleep_ms(200);
    // get NSAMP samples at FSAMP
    sample(cap_buf);
    // fill fourier transform input while subtracting DC component
    uint64_t sum = 0;
    for (int i = 0; i < NSAMP; i++)
    {
      sum += cap_buf[i];
    }
    float avg = (float)sum / NSAMP;
    for (int i = 0; i < NSAMP; i++)
    {
      fft_in[i] = (float)cap_buf[i] - avg;
    }
    //fill_scr(0);
    setCursorx(0);
    ssd1306_print("FFT"); // demonstrate some text
    show_scr();
    //sleep_ms(2000);
    // compute fast fourier transform
    kiss_fftr(cfg, fft_in, fft_out);

    //fill_scr(0);
    //sleep_ms(2000);
    // compute power and calculate max freq component
    float min_power = 0;
    float max_power = 0;
    int max_idx = 0;
    // any frequency bin over NSAMP/2 is aliased (nyquist sampling theorum)
    for (int i = 0; i < NSAMP / 2; i++)
    {
      float power = fft_out[i].r * fft_out[i].r + fft_out[i].i * fft_out[i].i;
      if (power > max_power)
      {
        max_power = power;
        max_idx = i;
      }else if (power<min_power)
      {
        min_power = power;
      }
      
    }
    float max_freq = freqs[max_idx];
    //printf("Greatest Frequency Component: %0.1f Hz\n", max_freq);



    char Text_f[16] = "f ";
    char Text_A[16] = "A ";
    char max_freq_str[16];
    char max_power_str[16];
    sprintf(max_freq_str, "%0.1f", max_freq);

    //ssd1306_print(max_freq_str); // demonstrate some text

    sprintf(max_freq_str, "%0.1f", max_freq);
    sprintf(max_power_str, "%0.1f", max_power);
    //strcat(Text, "f: ");
    strcat(Text_f, max_freq_str);
    //strcat(Text, " A: ");
    strcat(Text_A, max_power_str);

    //fill_scr(0);
    setCursorx(0);
    setCursory(3);
    ssd1306_print(Text_f); 

    setCursorx(0);
    setCursory(4);
    ssd1306_print(Text_A); 
   /* sleep_ms(2000);
    

    int Werte_pro_Pixel_X = freqs[(NSAMP/2)-1]/128;
    int Werte_pro_Pixel_Y = (max_power - min_power) /64;
    int Pixel_Wert_Y[128]; // 128 Pixel haben jeweils diesen Wert
    int pixelCountX=0;

    int Werte_Array[Werte_pro_Pixel_X+1];

    int n = 0;
    while(n < NSAMP / 2){
        for (int i = n; i < n+Werte_pro_Pixel_X; i++)
        {
          if (i>NSAMP/2)
          {
            break;
          }
          Werte_Array[i-n] = fft_out[i].r;
        }
        
        int max_value_temp=0;
        for (int d = 0; d < Werte_pro_Pixel_X+1; d++)
        {
         if (Werte_Array[d]>max_value_temp)
         {
           max_value_temp = Werte_Array[d];
         }
         
        }
        
        Pixel_Wert_Y[pixelCountX] = max_value_temp / Werte_pro_Pixel_Y; 
        pixelCountX = pixelCountX+1;
        n = n+Werte_pro_Pixel_X;
    }
    fill_scr(0);
    for (int x = 0; x < 128; x++)
    {
      draw_pixel(x,Pixel_Wert_Y[x],1);
    }
    show_scr();
    sleep_ms(2000);
    */
  }

  // should never get here
  kiss_fft_free(cfg);
  //-------------------------------------------------------------------------------------------------

  ssd1306_print("Hallo Welt!"); // demonstrate some text
  show_scr();
  sleep_ms(2000);
  fill_scr(0); // clear screen

  drawBitmap(0, 0, splash1_data, 64, 64, 1);
  show_scr();
  sleep_ms(2000);
  fill_scr(0); // clear screen

  drawBitmap(0, 0, splash1_data2, 64, 64, 1);
  show_scr();
  sleep_ms(20000);

  for (;;)
    ;
  return 0;
}

//-------------------------------------------------------------------------------------------------
void blink_pin_forever(PIO pio, uint sm, uint offset, uint pin, uint freq)
{
  blink_program_init(pio, sm, offset, pin);
  pio_sm_set_enabled(pio, sm, true);

  printf("Blinking pin %d at %d Hz\n", pin, freq);
  pio->txf[sm] = clock_get_hz(clk_sys) / 2 * freq;
}

void Rechteck_Burst(PIO pio, uint sm, uint offset, uint pin, uint freq, uint Burstcount)
{
  blink_program_init(pio, sm, offset, pin);
  pio_sm_set_enabled(pio, sm, true);

  printf("Pin %d mit %d Hz und %d Bursts\n", pin, freq, Burstcount);
  pio->txf[sm] = clock_get_hz(clk_sys) / 2 * freq;

  /*
	Hier fehlt noch die Abfrage
	Für N Bursts und dann     pio_sm_set_enabled(pio, sm, false);
	*/
}

//--------------------------------------------------------------------------------------------------------
// Funktionen für ADC FFT

// run sample with predefined "sample rate" "n-samples" and put it into capture_buffer
void sample(uint8_t *capture_buf) 
{
  adc_fifo_drain();
  adc_run(false);

  dma_channel_configure(dma_chan, &cfg,
                        capture_buf,   // dst
                        &adc_hw->fifo, // src
                        NSAMP,         // transfer count
                        true           // start immediately
  );

  gpio_put(LED_PIN, 1);
  adc_run(true);
  dma_channel_wait_for_finish_blocking(dma_chan);
  gpio_put(LED_PIN, 0);
}

void setup_adc_fft()
{
  stdio_init_all();

  gpio_init(LED_PIN);              // On Board LED
  gpio_set_dir(LED_PIN, GPIO_OUT); // auf LED Pin auf OUTPUT schalten

  adc_gpio_init(26 + CAPTURE_CHANNEL);

  adc_init();
  adc_select_input(CAPTURE_CHANNEL);
  adc_fifo_setup(
      true,  // Write each completed conversion to the sample FIFO
      true,  // Enable DMA data request (DREQ)
      1,     // DREQ (and IRQ) asserted when at least 1 sample present
      false, // We won't see the ERR bit because of 8 bit reads; disable.
      true   // Shift each sample to 8 bits when pushing to FIFO
  );

  // set sample rate
  adc_set_clkdiv(CLOCK_DIV);

  sleep_ms(1000);
  // Set up the DMA to start transferring data as soon as it appears in FIFO
  uint dma_chan = dma_claim_unused_channel(true);
  cfg = dma_channel_get_default_config(dma_chan);

  // Reading from constant address, writing to incrementing byte addresses
  channel_config_set_transfer_data_size(&cfg, DMA_SIZE_8);
  channel_config_set_read_increment(&cfg, false);
  channel_config_set_write_increment(&cfg, true);

  // Pace transfers based on availability of ADC samples
  channel_config_set_dreq(&cfg, DREQ_ADC);

  // calculate frequencies of each bin
  float f_max = FSAMP;
  float f_res = f_max / NSAMP;
  for (int i = 0; i < NSAMP; i++)
  {
    freqs[i] = f_res * i;
  }
}

//---------------------------------------------------------------------------
// Display Funktionen

int pages() { return height / 8; }

//uint8_t scr[pages*width+1]; // extra byte holds data send instruction
uint8_t scr[1025]; // being: 8 pages (max) * 128 width + 1 I2C command byte

void write_cmd(uint8_t cmd);

void fill_scr(uint8_t v)
{
  memset(scr, v, sizeof(scr));
}

void send_data(uint8_t *data, int nbytes)
{
  i2c_write_blocking(I2C_PORT, SID, data, nbytes, false); // Funktion aus der i2c.h lib
}

void send2(uint8_t v1, uint8_t v2)
{
  uint8_t buf[2];
  buf[0] = v1;
  buf[1] = v2;
  send_data(buf, 2);
}

void show_scr()
{

  write_cmd(SET_MEM_ADDR); // 0x20
  write_cmd(0b01);         // vertical addressing mode

  write_cmd(SET_COL_ADDR); // 0x21
  write_cmd(0);
  write_cmd(127);

  write_cmd(SET_PAGE_ADDR); // 0x22
  write_cmd(0);
  write_cmd(pages() - 1);

  scr[0] = 0x40; // the data instruction
  int size = pages() * width + 1;
  send_data(scr, size);
}

void write_cmd(uint8_t cmd)
{
  send2(0x80, cmd);
}

void poweroff() { write_cmd(SET_DISP | 0x00); }

void poweron() { write_cmd(SET_DISP | 0x01); }

void contrast(uint8_t contrast)
{
  write_cmd(SET_CONTRAST);
  write_cmd(contrast);
}

void invert(uint8_t invert) { write_cmd(SET_NORM_INV | (invert & 1)); }

static void init_i2c()
{
  // This example will use I2C0 on GPIO4 (SDA) and GPIO5 (SCL)
  i2c_init(I2C_PORT, 100 * 1000);
  gpio_set_function(4, GPIO_FUNC_I2C);
  gpio_set_function(5, GPIO_FUNC_I2C);
  gpio_pull_up(4);
  gpio_pull_up(5);
}

void init_display()
{
  init_i2c();
  height = 64;

  static uint8_t cmds[] = {
      SET_DISP | 0x00, // display off 0x0E | 0x00

      SET_MEM_ADDR, // 0x20
      0x00,         // horizontal

      //# resolution and layout
      SET_DISP_START_LINE | 0x00, // 0x40
      SET_SEG_REMAP | 0x01,       //# column addr 127 mapped to SEG0

      SET_MUX_RATIO, // 0xA8
      63,            //(uint8_t)(height - 1),

      SET_COM_OUT_DIR | 0x08, //# scan from COM[N] to COM0  (0xC0 | val)
      SET_DISP_OFFSET,        // 0xD3
      0x00,

      //SET_COM_PIN_CFG, // 0xDA
      //0x02 if self.width > 2 * self.height else 0x12,
      //width > 2*height ? 0x02 : 0x12,
      SET_COM_PIN_CFG, 0x12, //(uint8_t)(height == 32 ? 0x02 : 0x12),

      //# timing and driving scheme
      SET_DISP_CLK_DIV, // 0xD5
      0x80,

      SET_PRECHARGE, // 0xD9
      //0x22 if self.external_vcc else 0xF1,
      //external_vcc ? 0x22 : 0xF1,
      0xF1,

      SET_VCOM_DESEL, // 0xDB
      //0x30,  //# 0.83*Vcc
      0x40, // changed by mcarter

      //# display
      SET_CONTRAST, // 0x81
      0xFF,         //# maximum

      SET_ENTIRE_ON, //# output follows RAM contents // 0xA4
      SET_NORM_INV,  //# not inverted 0xA6

      SET_CHARGE_PUMP, // 0x8D
      //0x10 if self.external_vcc else 0x14,
      //external_vcc ? 0x10 : 0x14,
      0x14,

      SET_DISP | 0x01};

  // write all the commands
  for (int i = 0; i < sizeof(cmds); i++)
    write_cmd(cmds[i]);
  fill_scr(0);
  show_scr();
}

void draw_pixel(int16_t x, int16_t y, int color)
{
  if (x < 0 || x >= width || y < 0 || y >= height)
    return;

  int page = y / 8;
  //page = y/pages;
  int bit = 1 << (y % 8);
  int xincr = 8;
  xincr = height / 8;
  uint8_t *ptr = scr + x * xincr + page + 1;

  switch (color)
  {
  case 1: // white
    *ptr |= bit;
    break;
  case 0: // black
    *ptr &= ~bit;
    break;
  case -1: //inverse
    *ptr ^= bit;
    break;
  }
}

void drawBitmap(int16_t x, int16_t y, const uint8_t bitmap[], int16_t w,
                int16_t h, uint16_t color)
{
  int16_t byteWidth = (w + 7) / 8; // Bitmap scanline pad = whole byte
  uint8_t byte = 0;

  for (int16_t j = 0; j < h; j++, y++)
  {
    for (int16_t i = 0; i < w; i++)
    {
      if (i & 7)
        byte <<= 1;
      else
        byte = bitmap[j * byteWidth + i / 8];
      if (byte & 0x80)
        draw_pixel(x + i, y, color);
    }
  }
}

// draw letter "c" at coords "x" "y"
void draw_letter_at(uint8_t x, uint8_t y, char c)
{
  if (c < ' ' || c > 0x7F)
    c = '?'; // 0x7F is the DEL key

  int offset = 4 + (c - ' ') * 6;
  for (int col = 0; col < 6; col++)
  {
    uint8_t line = ssd1306_font6x8[offset + col];
    for (int row = 0; row < 8; row++)
    {
      draw_pixel(x + col, y + row, line & 1);
      line >>= 1;
    }
  }

  for (int row = 0; row < 8; row++)
  {
    draw_pixel(x + 6, y + row, 0);
    draw_pixel(x + 7, y + row, 0);
  }
}

// void draw_letter(char c) { draw_letter_at(0, 0, c); }

void ssd1306_print(const char *str)
{
  char c;
  while (c = *str)
  {
    str++;
    if (c == '\n')
    {
      cursorx = 0; 
      cursory += 8; // weil 8 Pixel eine Zeile sind
      continue;
    }
    draw_letter_at(cursorx, cursory, c);
    cursorx += 8;
  }
}

void setCursorx(int x)
{
  const int pos = 8;
  cursorx = pos * x;
}

void setCursory(int y)
{
  const int pos = 8;
  cursory = pos * y;
}