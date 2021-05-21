/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <string.h>
#include <math.h>

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

// Ein sample benötigt 96 Zyklen
// 
// 0 - 96 = 500,000 Hz
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

static uint8_t height = 64;
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
void send_command(uint8_t cmd);
void fill_whole_screen(uint8_t v);
void send_data(uint8_t *data, int nbytes);
void send_2_byte(uint8_t v1, uint8_t v2);
void Refresh_Screen();
void send_command(uint8_t cmd);
void Display_init();
void draw_pixel(int16_t x, int16_t y, int color);
void draw_letter_at(uint8_t x, uint8_t y, char c);
void SSD1306_print(const char *str);
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
  Display_init();

  /*SSD1306_print("Hallo Welt!"); // demonstrate some text
  Refresh_Screen();
  sleep_ms(1000);
*/
  //ADC FFT Variablen INIT
  uint8_t cap_buf[NSAMP];
  kiss_fft_scalar fft_in[NSAMP]; // kiss_fft_scalar is a float
  kiss_fft_cpx fft_out[NSAMP];
  kiss_fftr_cfg cfg = kiss_fftr_alloc(NSAMP, false, 0, 0);

  setup_adc_fft(); // setup ports and outputs

  // Setup Rechteck burst
  PIO pio = pio0;                                                    // PIO0 verwenden
  uint sm = pio_claim_unused_sm(pio, true);                          // irgrendeine Freie State machine zum managen des Singals
  float freq = 56600;                                                // Gewünschte frequenz des Rechteck Signals
  uint32_t clckdiv = (uint32_t)(12500000 / freq * 2.5f * (1 << 16)); //Um wie viel muss ich den Systemclock 125Mhz teilen um auf 56.6kHz zu kommen?
                                                                     // Anmerkung 2.5f *(1<<16) setzt den Clock auf 12.5 Mhz das hat was mit dem Teiler zu tun siehe Pico-examples/pio/squarewave.c

  for (int i = 0; i < count_of(blink_program_instructions); ++i) // Assemblerprogramm in Instruction Memory meiner PIO Instanz laden
    pio->instr_mem[i] = blink_program_instructions[i];

  pio->sm[0].clkdiv = clckdiv; // Clockdiv für die Statemachine definieren
  pio->sm[0].pinctrl =
      (1 << PIO_SM0_PINCTRL_SET_COUNT_LSB) |
      (0 << PIO_SM0_PINCTRL_SET_BASE_LSB); // Pin Control register setzten sodass mein Signal auf GP0 raus kommt

  gpio_set_function(0, GPIO_FUNC_PIO0);
  hw_set_bits(&pio->ctrl, 1 << (PIO_CTRL_SM_ENABLE_LSB + 0)); // State Machine enable setzen

  while (1) // Endlosschleife
  {
    //printf("%d\n", clock_get_hz(clk_sys) / 2 * freq);
    fill_whole_screen(0);
    //setCursorx(0);
    //setCursory(0);
    //SSD1306_print("                                             "); // ToDo OLED Textzeile eleganter leeren
    //setCursorx(0);
    //SSD1306_print("Sam"); // demonstrate some text
    //Refresh_Screen();
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
    //fill_whole_screen(0);
    setCursorx(0);
    setCursory(1);
    SSD1306_print("FFT"); // demonstrate some text
    //Refresh_Screen();
    //sleep_ms(2000);
    // compute fast fourier transform
    kiss_fftr(cfg, fft_in, fft_out);

    //fill_whole_screen(0);
    //sleep_ms(2000);
    // compute power and calculate max freq component
    float min_power = 0;
    float max_power = 0;
    int max_idx = 0;
    // any frequency bin over NSAMP/2 is aliased (nyquist sampling theorum)
    for (int i = 0; i < NSAMP / 2; i++)
    {
      float power = sqrt(fft_out[i].r * fft_out[i].r + fft_out[i].i * fft_out[i].i);
      if (power > max_power)
      {
        max_power = power;
        max_idx = i;
      }
      else if (power < min_power)
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

    //SSD1306_print(max_freq_str); // demonstrate some text

    sprintf(max_freq_str, "%0.1f", max_freq);
    sprintf(max_power_str, "%0.1f", max_power);
    //strcat(Text, "f: ");
    strcat(Text_f, max_freq_str);
    //strcat(Text, " A: ");
    strcat(Text_A, max_power_str);

    //fill_whole_screen(0);
    setCursorx(0);
    setCursory(4);
    SSD1306_print(Text_f);

    setCursorx(0);
    setCursory(5);
    SSD1306_print(Text_A);
    //Refresh_Screen();

    //sleep_ms(750);

    //////////////////////////// DEBUG Area

    int Werte_pro_Pixel_X = 20; //freqs[(NSAMP/2)-1]/128;
    float Werte_pro_Pixel_Y = (max_power - min_power) * 2 / 64;
    // Werte_pro_Pixel_X ?
    int Pixel_Wert_Y[128]; // 128 Pixel haben jeweils diesen Wert
    int pixelCountX = 0;
    /*
    char Text[16] = "WPX:";
    char TextB[12];
    setCursorx(0);
    setCursory(1);
    sprintf(TextB, "%0.1f", );
    strcat(Text, TextB);
    SSD1306_print(Text);
*/
    double Werte_Array[Werte_pro_Pixel_X + 1];

    int n = 0;
    while (n < NSAMP / 2) // nur Frequenzen bis zur Nyquist Frequenz (f_abtast /2 ) anschauen 
    {
      for (int i = n; i < n + Werte_pro_Pixel_X; i++)
      {
        if (i > NSAMP / 2)
        {
          break;
        }
        Werte_Array[i - n] = sqrt(fft_out[i].r * fft_out[i].r + fft_out[i].i * fft_out[i].i);
        //Werte_Array[i - n] = i - n;
      }

      // Find max
      double max_value_temp = 0;
      for (int d = 0; d < Werte_pro_Pixel_X; d++)
      {
        if (Werte_Array[d] > max_value_temp)
        {
          max_value_temp = Werte_Array[d];
        }
      }
      /*
      setCursory(5);
      setCursorx(0);
      char TextB3[16];
      //fill_whole_screen(0);
      sprintf(TextB3, "%0.1f", max_value_temp);
      SSD1306_print(TextB3);

      setCursory(2);
      setCursorx(0);
      char TextB4[16];
      //fill_whole_screen(0);
      sprintf(TextB4, "%0.1f                          ", Werte_pro_Pixel_Y);
      SSD1306_print(TextB4);
      Refresh_Screen();*/
      //sleep_ms(100);

      Pixel_Wert_Y[pixelCountX] = max_value_temp / Werte_pro_Pixel_Y;
      for (int i = 0; i < Pixel_Wert_Y[pixelCountX]; i++)
      {
        draw_pixel(n / Werte_pro_Pixel_X, i, 1);
      }
      char Text[16] = "X:";
      char TextB[12];
      setCursorx(0);
      setCursory(2);
      sprintf(TextB, "%d  ", Pixel_Wert_Y[pixelCountX]);
      strcat(Text, TextB);
      SSD1306_print(Text);
      gpio_put(LED_PIN, 0);

      //sleep_ms(00);
      gpio_put(LED_PIN, 1);

      //draw_pixel(x, 64-Pixel_Wert_Y[pixelCountX], 1);
      pixelCountX = pixelCountX + 1;
      n = n + Werte_pro_Pixel_X;
    }
    //Refresh_Screen();

    // Find max
    /*
    double max_pixel_value = 0;
    double min_pixel_value = 0;
    for (int d = 0; d < 128 + 1; d++)
    {
      if (Pixel_Wert_Y[d] > max_pixel_value)
      {
        max_pixel_value = Pixel_Wert_Y[d];
      }
      else if (Pixel_Wert_Y[d] < min_pixel_value)
      {
        min_pixel_value = Pixel_Wert_Y[d];
      }
    }
  */
    /*  char Text[16] = "X:";
    char TextB[12];
    setCursorx(0);
    setCursory(1);
    sprintf(TextB, "%d", Werte_pro_Pixel_Y);
    strcat(Text, TextB);
    SSD1306_print(Text);
    Refresh_Screen();
*/
    //char Text2[16] = "PX:";
    //char TextB2[12];
    /*setCursorx(0);
    setCursory(5);
    sprintf(TextB2, "%e", Werte_pro_Pixel_Y);
    strcat(Text2, TextB2);
    SSD1306_print(Text2);
    Refresh_Screen();*/

    //fill_whole_screen(0);
    for (int x = 0; x < 128; x++)
    {
      //for (int i = 0; i < Pixel_Wert_Y[x]; i++)
      {

        //draw_pixel(x, Pixel_Wert_Y[x], 1);
        /*if (x < 64)
        {
           draw_pixel(x, 64 - x, 1);
        }
        else
        {
           draw_pixel(x, x - 64, 1);
        }*/
      }
      /*
      gpio_put(LED_PIN, 1);
      setCursorx(0);
      setCursory(0);
      sprintf(TextB2, "%d", x);
      SSD1306_print(TextB2);
      
      setCursorx(0);
      setCursory(5);
      char Text3[16] = "PX:";
      char TextB3[12];

      sprintf(TextB3, "x: %e", Pixel_Wert_Y[x]);
      strcat(Text3, TextB3);
      //SSD1306_print("                ");
      //Refresh_Screen();
      SSD1306_print(Text2);
      gpio_put(LED_PIN, 0);
*/

      //sleep_ms(40);
      //sleep_ms(1);
    }
    Refresh_Screen();

    //Refresh_Screen();
    gpio_put(LED_PIN, 0);

    //sleep_ms(20);
  }

  // should never get here
  kiss_fft_free(cfg);
  //-------------------------------------------------------------------------------------------------

  SSD1306_print("Hallo Welt!"); // demonstrate some text
  Refresh_Screen();
  sleep_ms(2000);
  fill_whole_screen(0); // clear screen
  Refresh_Screen();
  sleep_ms(20000);
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

void send_command(uint8_t cmd);

void fill_whole_screen(uint8_t v)
{
  memset(scr, v, sizeof(scr));
}

void send_data(uint8_t *Data, int Anzahl_bytes) // Daten über I2C Senden
{
  i2c_write_blocking(I2C_PORT, SID, Data, Anzahl_bytes, false); // Funktion aus der i2c.h lib
}

void send_2_byte(uint8_t byte1, uint8_t byte2)
{
  uint8_t byte_buffer[2];
  byte_buffer[0] = byte1;
  byte_buffer[1] = byte2;
  send_data(byte_buffer, 2);
}

void Refresh_Screen()
{

  send_command(SET_MEM_ADDR); // 0x20
  send_command(0x01);         // vertical addressing mode

  send_command(SET_COL_ADDR); // 0x21
  send_command(0);
  send_command(127);

  send_command(SET_PAGE_ADDR); // 0x22
  send_command(0);
  send_command(pages() - 1);

  scr[0] = 0x40; // the data instruction
  int size = pages() * width + 1;
  send_data(scr, size);
}

void send_command(uint8_t cmd)
{
  send_2_byte(0x80, cmd);
}


void Display_init()
{
  i2c_init(I2C_PORT, 100 * 1000);
  gpio_set_function(4, GPIO_FUNC_I2C);
  gpio_set_function(5, GPIO_FUNC_I2C);
  gpio_pull_up(4); // SDA Pullup
  gpio_pull_up(5); //SCL Pullup

  // Init Commands in Array -> Reihenfolge aus Application Note SSD1306.pdf S. 64 -> Software Initialization Chart 
static uint8_t init_cmds[] = {
      SET_MUX_RATIO, 63, // Ich möchte manuell eine Multiplex Ratio eingeben -> 0xA8 // Default 63        
      SET_DISP_OFFSET, 0x00, // 0xD3 > Offset auf 00h ( X-Wert der Matrix)
      SET_DISP_START_LINE, // Start Zeile 00 -> 0x40 (Y-Wert der Matrix) Line (0x40 - 0x7F) Zeile 0 bis 63
      SET_SEG_REMAP | 0x01, // 0xA0 -> 0xA1 > Mapping zwischen Display und Driver anpassen -> Mehr Gestaltungsmöglichkeiten pro Pixel (Kontrast etc)
      SET_COM_OUT_DIR | 0x08, // 0xC0 -> 0xC8
      SET_COM_PIN_CFG, 0x12, //0xDA auf 0x12 -> Sagen welcher COM vom SSD1306 welche Zeile im OLED ansteuern soll siehe SSD1306.pdf Table 10-3 COM Pins Hardware Config 7.
      SET_CONTRAST, 0x7F, // Kontrast 0x81 // Werte von 0x00 bis 0xFF -> "Helligkeit der Anzeige" 0xFF ist max
      SET_ENTIRE_ON, // 0xA4 > Outputs aus GDDRAM entnehmen 
      SET_NORM_INV, // Normal/inverse Modus > Ist eine 1 eine Pixel an oder aus -> Normal also 1=An
      SET_DISP_CLK_DIV, 0x80, // Taktrate einstellen
      SET_CHARGE_PUMP, 0x14, // Spannungsversorgung -> Soll die Eingangsspannung verstärkt werden? 0x14=ja
      SET_DISP | 0x01 //Display anzeigen
      };

  // Einmal die Init durcharbeiten
  for (int i = 0; i < sizeof(init_cmds); i++){
    send_command(init_cmds[i]);
  }
  fill_whole_screen(0);
  Refresh_Screen();
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

void SSD1306_print(const char *str)
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