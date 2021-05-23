/*
  Author: Niklas Gauder
  Usage: Für die verwendung auf einem Raspberry Pi Pico bzw einem RP2040 Prozessor
  -> 3 Teilig
      1. FFT an GP26
      2. OLED SSD1306 Display an I2C0
      3. PIO Output an GP0 (square wave)
*/

#define Squarewave_Frequenz 56600
#define Squarewave_BurstCount 8

#define FLAG_FFT false
#define FLAG_FFT_darstellen false
#define FLAG_FFT_Text_darstellen false
#define FLAG_FFT_Normal true // wenn false, dann wird die FFT von Oben nach unten angezeigt
#define FLAG_Zeitsignal true

#define ADC_Abtastrate 200000
// Es werden 5000 (NSMAP) Samples bei dieser Abtastrate getätigt -> bei 150kHz sind das 33,3 ms (5000/150000)

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
// 500000 Hz * 96 = Wunschfrequenz * Einzustellender_Teiler
// 96 * 500000/Wunschfrequenz = Teiler
// 0      = 500 kHz
// 96     = 500 kHz
// 128    = 250 kHz
// 240    = 200 kHz
// 320    = 150 kHz
// 960    = 50 kHz
#define CLOCK_DIV 500000 * 96 / ADC_Abtastrate //320
#define FSAMP 500000 * 96 / CLOCK_DIV          // Für Anti-Aliasing Filter -> Abtastfrequenz

// BE CAREFUL: anything over about 9000 here will cause things
// to silently break. The code will compile and upload, but due
// to memory issues nothing will work properly
#define NSAMP 5000//5000

// ADC FFT Channel 0 is GPIO26
#define CAPTURE_CHANNEL 0
#define LED_PIN 25

// globals
dma_channel_config cfg;
uint dma_chan;
float freqs[NSAMP];

void setup_adc_fft();
void sample(uint8_t *capture_buf);

//--------------------------------------------------------------------------------------------------------
// Für OLED Display

// Befehle für das OLED Display -> Hex Werte aus SSD1306.pdf
#define SET_MEMORY_ADDR 0x20
#define SET_COLUMN_ADDR 0x21
#define SET_PAGE_ADDR 0x22
#define SET_DISPLAY_START_LINE 0x40 // Range geht bix 0x7F
#define SET_CONTRAST_LEVEL 0x81
#define SET_CHARGE_PUMP 0x8D
#define SET_SEGMENT_RE_MAP 0xA0
#define SET_DISPLAY_ON 0xA4
#define SET_NORMAL_INVERS 0xA6
#define SET_MUX_RATIO 0xA8
#define SET_DISPLAY_ON_OFF 0xAE
#define SET_COM_OUT_DIR 0xC0
#define SET_DISPLAY_OFFSET 0xD3
#define SET_DISPLAY_CLK_DIV 0xD5
#define SET_VCOM_DESEL 0xDB
#define SET_PRECHARGE 0xD9
#define SET_COM_PIN_CFG 0xDA

#define I2C_PORT i2c0 // I2C Port auf GPIO 4 /5

static uint8_t height = 64;
const uint8_t width = 128;
const uint8_t SID = 0x3C;

// Display Funktionen
//--------------------------------------------------------------------------------------------------------
uint8_t scr[1025]; // 128 * 64 / 8 + 1
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
void Write_to_Display(const char *str);
void setCursorx(int x);
void setCursory(int y);
//--------------------------------------------------------------------------------------------------------

// PIO Funktionen
void Rechteck_Burst(PIO pio_obj, uint Statemachine, uint offset, uint pin, uint freq, uint Burstcount);

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
// MAIN
int main()
{
  // I2C und Display Init
  Display_init();

  //ADC FFT Variablen INIT
  uint8_t cap_buf[NSAMP];
  kiss_fft_scalar fft_in[NSAMP]; // kiss_fft_scalar is a float
  kiss_fft_cpx fft_out[NSAMP];
  kiss_fftr_cfg cfg = kiss_fftr_alloc(NSAMP, false, 0, 0);

  setup_adc_fft(); // setup ports and outputs

  // Setup Rechteck burst
  PIO pio = pio0;                                                    // PIO0 verwenden
  uint sm = pio_claim_unused_sm(pio, true);                          // irgrendeine Freie State machine zum managen des Singals
  float freq = Squarewave_Frequenz;                                  // Gewünschte frequenz des Rechteck Signals
  uint32_t clckdiv = (uint32_t)(12500000 / freq * 2.5f * (1 << 16)); // Um wie viel muss ich den Systemclock 125Mhz teilen um auf 56.6kHz zu kommen?
                                                                     // Anmerkung 2.5f *(1<<16) setzt den Clock auf 12.5 Mhz das hat was mit dem Teiler zu tun siehe Pico-examples/pio/squarewave.c
  uint32_t Anzahl_Rechtecke = Squarewave_BurstCount;                 //8;

  pio_sm_put_blocking(pio, sm, Anzahl_Rechtecke - 1);            // Anzahl - 1 weil PIO überprüft ob !=0 und nicht !=1 -> Of By One Error vermeiden
  for (int i = 0; i < count_of(burst_program_instructions); ++i) // maximal 32 Werte > Mehr speicher hat der PIO Controller nicht
  {                                                              // Assemblerprogramm in Instruction Memory meiner PIO Instanz laden
    pio->instr_mem[i] = burst_program_instructions[i];
  }

  pio->sm[0].clkdiv = clckdiv;                                                                     // Clockdiv für die Statemachine definieren
  pio->sm[0].pinctrl = (1 << PIO_SM0_PINCTRL_SET_COUNT_LSB) | (0 << PIO_SM0_PINCTRL_SET_BASE_LSB); // Pin Control register setzten sodass mein Signal auf GP0 raus kommt

  gpio_set_function(0, GPIO_FUNC_PIO0);                       // PIN 0 mit GPIO_FUNC_PIO0
  hw_set_bits(&pio->ctrl, 1 << (PIO_CTRL_SM_ENABLE_LSB)); // State Machine enable setzen

  while (1) // Endlosschleife
  {
    fill_whole_screen(0); // Bildschirm leeren

    // get NSAMP samples at FSAMP
    sample(cap_buf);
    // FFT In füllen und den DC Anteil herausrechnen
    uint64_t summe = 0;
    for (int i = 0; i < NSAMP; i++)
    {
      summe += cap_buf[i];
    }
    float avg = (float)summe / NSAMP; // Mittelwert über die gesamte Messung
    for (int i = 0; i < NSAMP; i++)
    {
      fft_in[i] = (float)cap_buf[i] - avg; // Jeder einzelne Wert ohne den Mittelwert (DC)
    }
/*
    setCursorx(0);           // 0tes Pixel
    setCursory(1);           // 1te Zeile
    Write_to_Display("FFT"); // Display Status
*/
    // FFT berechnen
    kiss_fftr(cfg, fft_in, fft_out);

    // Maximas ausrechnen
    float min_power = 0;
    float max_power = 0;
    int max_idx = 0;

    // Jede Frequenz überhalb NSAMP/2 ist ein Alias infolgedessen nur die untere Hälfte > Nyquist Abtast Theorem
    for (int i = 0; i < NSAMP / 2; i++)
    {
      float power = sqrt(fft_out[i].r * fft_out[i].r + fft_out[i].i * fft_out[i].i); // Betrag berechnen z = a + bi -> a^2 + b^2
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
    float max_freq = freqs[max_idx]; // Frequenz größter Amplitude
#if FLAG_FFT_Text_darstellen
    char Text_f[16] = "f ";
    char Text_A[16] = "A ";
    char max_freq_str[16];
    char max_power_str[16];
    sprintf(max_freq_str, "%0.1f", max_freq); // String formatieren > float to String konvertieren
    sprintf(max_power_str, "%0.1f", max_power);
    strcat(Text_f, max_freq_str); // Strings hintereinander reihen "f " + "12056" > "f 12056"
    strcat(Text_A, max_power_str);

    //fill_whole_screen(0);
    setCursorx(0);
    setCursory(4);            // 5te Zeile
    Write_to_Display(Text_f); // Text schreiben

    setCursorx(0);
    setCursory(5);            // 6te Zeile
    Write_to_Display(Text_A); // Text schreiben
                              //Refresh_Screen();
#endif
    //sleep_ms(750);

    //////////////////////////// FFT Darstellen
#if FLAG_FFT
    int Werte_pro_Pixel_X = 20; //freqs[(NSAMP/2)-1]/128;
    float Werte_pro_Pixel_Y = (max_power - min_power) * 2 / 64;
    // Werte_pro_Pixel_X ?
    int Pixel_Wert_Y[128]; // 128 Pixel haben jeweils diesen Wert
    int pixelCountX = 0;

    double Werte_Array[Werte_pro_Pixel_X + 1];

    int n = 0;
    while (n < NSAMP / 2) // nur Frequenzen bis zur Nyquist Frequenz (f_abtast / 2) anschauen
    {
      for (int i = n; i < n + Werte_pro_Pixel_X; i++)
      {
        if (i > NSAMP / 2)
        {
          break;
        }
        Werte_Array[i - n] = sqrt(fft_out[i].r * fft_out[i].r + fft_out[i].i * fft_out[i].i); // Betrag rechnen z = a +bi _> a^2 + b^2
      }

      // Find max Amplitude
      double max_value_temp = 0;
      for (int d = 0; d < Werte_pro_Pixel_X; d++)
      {
        if (Werte_Array[d] > max_value_temp)
        {
          max_value_temp = Werte_Array[d];
        }
      }

      Pixel_Wert_Y[pixelCountX] = max_value_temp / Werte_pro_Pixel_Y;
      if (FLAG_FFT_darstellen)
      {

#if FLAG_FFT_Normal
        for (int i = 63; i > 64 - Pixel_Wert_Y[pixelCountX]; i--) // Pixel 64 bis Pixel_Wert_Y[Momentane X Reihe] zeichnen
#elif
        for (int i = 0; i < Pixel_Wert_Y[pixelCountX]; i++) // Pixel 64 bis Pixel_Wert_Y[Momentane X Reihe] zeichnen
#endif
        {
          draw_pixel(n / Werte_pro_Pixel_X, i, 1); // evtl Zeitoptimierung wenn nicht jeder Pixel einzeln sondern das erst in eine 128*64 Matrix und dann zeichnen?
        }
      }
#if FLAG_FFT_Text_darstellen
      char Text[16] = "X:";
      char TextB[12];
      setCursorx(0);
      setCursory(2);
      sprintf(TextB, "%d  ", Pixel_Wert_Y[pixelCountX]);
      strcat(Text, TextB);
      Write_to_Display(Text);
      gpio_put(LED_PIN, 0);
#endif

      //sleep_ms(00);
      gpio_put(LED_PIN, 1);

      //draw_pixel(x, 64-Pixel_Wert_Y[pixelCountX], 1);
      pixelCountX = pixelCountX + 1;
      n = n + Werte_pro_Pixel_X;
    }
#endif

#if FLAG_Zeitsignal
    int X_Counter = 0, j = 0;
    int Zeitsignal_Werte_pro_Pixel_X = 1;
    int Zeitsignal_Werte_pro_Pixel_Y;
    float avg_zs_tmp = 0, sum_zs = 0; // zs heißt zeitsignal
    int zs_max_y = 0, zs_min_y = 0;
    int Pixel_y_Wert;

    for (int i = 0; i < NSAMP; i++)
    {
      if (fft_in[i] > zs_max_y)
      {
        zs_max_y = fft_in[i];
      }
      else if (fft_in[i] < zs_min_y)
      {
        zs_min_y = fft_in[i];
      }
    }
    Zeitsignal_Werte_pro_Pixel_Y = (zs_max_y - zs_min_y)/64;
    int zusatz=0;
    while (j < NSAMP && ((j + Zeitsignal_Werte_pro_Pixel_X) < NSAMP))
    {
      if (X_Counter >128)
      {
        X_Counter = 0;
        Refresh_Screen();
        sleep_ms(100);
        fill_whole_screen(0);
      }
      

      sum_zs = 0;
      float tmp_max = fft_in[j], tmp_min = fft_in[j];
      for (int i = j; i < j + Zeitsignal_Werte_pro_Pixel_X; i++)
      {
        //sum_zs += fft_in[i];
        tmp_max = fft_in[i] > tmp_max ? fft_in[i] : tmp_max;
        tmp_min = fft_in[i] < tmp_min ? fft_in[i] : tmp_min;
      }

      draw_pixel(X_Counter, 31 + tmp_max / (2 * Zeitsignal_Werte_pro_Pixel_Y), 1);
      draw_pixel(X_Counter, 31 + tmp_min / (2 * Zeitsignal_Werte_pro_Pixel_Y), 1);
      X_Counter = X_Counter + 1;
      j = j + Zeitsignal_Werte_pro_Pixel_X;
    }

#endif

    Refresh_Screen();
    gpio_put(LED_PIN, 0); // Status LED aus

    //sleep_ms(20);
  } // Ende Endlosschleife

  // should never get here
  kiss_fft_free(cfg);
  return 0;
}

//-------------------------------------------------------------------------------------------------
void Rechteck_Burst(PIO pio, uint sm, uint offset, uint pin, uint freq, uint Burstcount)
{
  burst_program_init(pio, sm, offset, pin);
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

  send_command(SET_MEMORY_ADDR); // 0x20
  send_command(0x01);            // vertical addressing mode

  send_command(SET_COLUMN_ADDR); // 0x21
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
      SET_MUX_RATIO, 63,         // Ich möchte manuell eine Multiplex Ratio eingeben -> 0xA8
      SET_DISPLAY_OFFSET, 0x00,  // 0xD3 > Offset auf 00h (X-Wert der Matrix)
      SET_DISPLAY_START_LINE,    // Start Zeile 0 -> 0x40 (Y-Wert der Matrix) Line (0x40 - 0x7F) Zeile 0 bis 63
      SET_SEGMENT_RE_MAP | 0x01, // 0xA0 -> 0xA1 > Mapping zwischen Display und Driver anpassen -> Mehr Gestaltungsmöglichkeiten pro Pixel (Kontrast etc)
      SET_COM_OUT_DIR | 0x08,    // 0xC0 -> 0xC8 > Definition welcher COM Port welche Zeile ansteuert
      SET_COM_PIN_CFG, 0x12,     //0xDA auf 0x12 -> Sagen welcher COM vom SSD1306 welche Zeile im OLED ansteuern soll siehe SSD1306.pdf Table 10-3 COM Pins Hardware Config 7.
      SET_CONTRAST_LEVEL, 0x7F,  // Kontrast 0x81 // Werte von 0x00 bis 0xFF -> "Helligkeit der Anzeige" 0xFF ist max
      SET_DISPLAY_ON,            // 0xA4 > Outputs aus GDDRAM entnehmen
      SET_NORMAL_INVERS,         // Normal/inverse Modus > Ist eine 1 als Pixel Wert an oder aus? -> Normal also 1=An
      SET_DISPLAY_CLK_DIV, 0x80, // Taktrate einstellen
      SET_CHARGE_PUMP, 0x14,     // Spannungsversorgung -> Soll die Eingangsspannung verstärkt werden? 0x14=ja
      SET_DISPLAY_ON_OFF | 0x01  //Display anzeigen
  };

  // Einmal die Init durcharbeiten
  for (int i = 0; i < sizeof(init_cmds); i++)
  {
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

void Write_to_Display(const char *str)
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