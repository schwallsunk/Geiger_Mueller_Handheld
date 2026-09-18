#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/i2c.h"
#include "hardware/timer.h"
#include "hardware/timer.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "pico/binary_info.h"
#include "fonts.h"
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

/* Example code to talk to a SSD1306 OLED display, 128 x 64 pixels

   NOTE: Ensure the device is capable of being driven at 3.3v NOT 5v. The Pico
   GPIO (and therefor I2C) cannot be used at 5v.

   Connections on Raspberry Pi Pico board, other boards may vary.

   GPIO 12 (pin 16)-> SDA on SSD1306 board
   GPIO 13 (pin 17)-> SCL on SSD1306 board
   3.3v (pin 36) -> VCC on SSD1306 board
   GND (pin 38)  -> GND on SSD1306 board
*/

// By default these devices are on bus address 0x3C or 0x3D. Check your documentation.
static int DEVICE_ADDRESS = 0x3C;

#define I2C_PORT i2c0
#define I2C_SDA_PIN 24
#define I2C_SCL_PIN 25

 // This can be overclocked, 2000 seems to work on the device being tested
 // Spec says 400 is the maximum. Try faster clocks until it stops working!
 // KHz.
#define I2C_CLOCK  400

#define SSD1306_LCDWIDTH            128
#define SSD1306_LCDHEIGHT           64
#define SSD1306_FRAMEBUFFER_SIZE    (SSD1306_LCDWIDTH * SSD1306_LCDHEIGHT / 8)

// Not currently used.
#define SSD1306_SETLOWCOLUMN        0x00
#define SSD1306_SETHIGHCOLUMN       0x10

#define SSD1306_MEMORYMODE          0x20
#define SSD1306_COLUMNADDR          0x21
#define SSD1306_PAGEADDR            0x22
#define SSD1306_DEACTIVATE_SCROLL   0x2E
#define SSD1306_ACTIVATE_SCROLL     0x2F

#define SSD1306_SETSTARTLINE        0x40

#define SSD1306_SETCONTRAST         0x81
#define SSD1306_CHARGEPUMP          0x8D

#define SSD1306_SEGREMAP0           0xA0
#define SSD1306_SEGREMAP127         0xA1
#define SSD1306_DISPLAYALLON_RESUME 0xA4
#define SSD1306_DISPLAYALLON        0xA5
#define SSD1306_NORMALDISPLAY       0xA6
#define SSD1306_INVERTDISPLAY       0xA7
#define SSD1306_SETMULTIPLEX        0xA8
#define SSD1306_DISPLAYOFF          0xAE
#define SSD1306_DISPLAYON           0xAF

#define SSD1306_COMSCANINC          0xC0
#define SSD1306_COMSCANDEC          0xC8

#define SSD1306_SETDISPLAYOFFSET    0xD3
#define SSD1306_SETDISPLAYCLOCKDIV  0xD5
#define SSD1306_SETPRECHARGE        0xD9
#define SSD1306_SETCOMPINS          0xDA
#define SSD1306_SETVCOMDETECT       0xDB

// We need a 0x40 in the byte before our framebuffer
uint8_t _Framebuffer[SSD1306_FRAMEBUFFER_SIZE + 1] = {0x40};
uint8_t *Framebuffer = _Framebuffer+1;

const uint16_t PERIOD_INTEGRATION_COUNTS = 1000U;  // Interrupt period in ms within which the counts are being averaged for
const uint8_t HV_TUBE_INT_PIN = 11U; // Geiger tube interrupt for ionization counting
const uint8_t HV_TUBE_PSU_PIN = 10U; // HV tube high voltage (HV) switching power supply stimulus pin
const uint8_t BUZZER_PIN = 9U; // Pin on which Piezo buzzer is hooked up to
const uint8_t LED_R_PIN = 20U; // Pin for the red color LED
const uint8_t LED_G_PIN = 19U; // Pin for the green color LED
const uint8_t LED_O_PIN = 15U; // Pin for the oroange color LED
const uint8_t SWITCH_1_PIN = 12U; // Center push joystick
const uint8_t SWITCH_2_PIN = 13U; // Pin for the switch 2 labelled SW2 on PCB 
const uint8_t SWITCH_3_PIN = 14U; // Pin for the switch 3 labelled SW3 on PCB
const uint8_t SWITCH_4_PIN = 15U; // Pin for the switch 4 labelled SW4 on PCB 
const uint8_t SWITCH_5_PIN = 16U; // Pin for the switch 4 labelled SW4 on PCB 

const uint8_t LENGHT_MOVING_AVG = 10U; // Pin for the switch 4 labelled SW4 on PCB 

uint16_t counts_array_moving_avg[10]; // Moving window array for averaged cps value
volatile uint16_t counter = 0;
volatile bool Buzzer_state = 0;
static repeating_timer_t timer; // struct used for the repeating measurement timer interrupt
volatile bool Flag_thread_lock = false;
volatile uint8_t counter_location = 0;

volatile bool thread_alive = true; // Flag multithreading



void counter_callback(){
    irq_set_enabled(IO_IRQ_BANK0,false);
    counter++;
    for(uint i =0;i<100;i++)
    {
        gpio_put(BUZZER_PIN,Buzzer_state);
        Buzzer_state = !Buzzer_state;
        gpio_put(BUZZER_PIN,Buzzer_state);
        Buzzer_state = !Buzzer_state;
    }

    irq_set_enabled(IO_IRQ_BANK0,true);
}

void print_measurements(uint16_t *buf,
                const uint size_buf){
  for(uint i=0; i<size_buf;i++){
    printf("%u,",*(buf+i));
    }
  printf("\n");
}

void print_bequerel(uint16_t *buf,
                const uint size_buf){
    uint32_t total_count = 0;
    double total_activity = 0.;
    for(uint i=0; i<size_buf;i++){
    total_count=*(buf+i)+total_count;
    }
    total_activity = (double)total_count/(size_buf*(PERIOD_INTEGRATION_COUNTS/1000));
    printf("The total acitvity in of the sample is %.4f Bq",total_activity);
    printf("\n");
    printf("The total amount of decays in of the sample over the last 10 seconds are %u decays",total_count);
    printf("\n");
    }

bool alarm_callback(repeating_timer_t *t) {
    if (counter_location < LENGHT_MOVING_AVG){
        counter_location++;
        counts_array_moving_avg[counter_location] = counter;
    }
    else{
        counts_array_moving_avg[0] = counter;
        counter_location = 0;
    }
    counter = 0;
    return true;
}

static void SendCommand(uint8_t cmd) {
    uint8_t buf[] = {0x00, cmd};
    i2c_write_blocking(I2C_PORT, DEVICE_ADDRESS, buf, 2, false);
}

static void SendCommandBuffer(uint8_t *inbuf, int len) {
    i2c_write_blocking(I2C_PORT, DEVICE_ADDRESS, inbuf, len, false);
}

static void SSD1306_initialise() {

uint8_t init_cmds[]=
    {0x00,
    SSD1306_DISPLAYOFF,
    SSD1306_SETMULTIPLEX, 0x3f,
    SSD1306_SETDISPLAYOFFSET, 0x00,
    SSD1306_SETSTARTLINE,
    SSD1306_SEGREMAP127,
    SSD1306_COMSCANDEC,
    SSD1306_SETCOMPINS, 0x12,
    SSD1306_SETCONTRAST, 0xff,
    SSD1306_DISPLAYALLON_RESUME,
    SSD1306_NORMALDISPLAY,
    SSD1306_SETDISPLAYCLOCKDIV, 0x80,
    SSD1306_CHARGEPUMP, 0x14,
    SSD1306_DISPLAYON,
    SSD1306_MEMORYMODE, 0x00,   // 0 = horizontal, 1 = vertical, 2 = page
    SSD1306_COLUMNADDR, 0, SSD1306_LCDWIDTH-1,  // Set the screen wrapping points
    SSD1306_PAGEADDR, 0, 7};

    SendCommandBuffer(init_cmds, sizeof(init_cmds));
}


static void InvertDisplay(bool yes) {
    if (yes)
        SendCommand(SSD1306_INVERTDISPLAY);
    else
        SendCommand(SSD1306_NORMALDISPLAY);
}

// This copies the entire framebuffer to the display.
static void UpdateDisplay() {
    i2c_write_blocking(I2C_PORT, DEVICE_ADDRESS, _Framebuffer, sizeof(_Framebuffer), false);
}

static void ClearDisplay() {
    memset(Framebuffer, 0, SSD1306_FRAMEBUFFER_SIZE);
    UpdateDisplay();
}


static void SetPixel(int x,int y, bool on) {
    assert(x >= 0 && x < SSD1306_LCDWIDTH && y >=0 && y < SSD1306_LCDHEIGHT);

    // The calculation to determine the correct bit to set depends on which address
    // mode we are in. This code assumes horizontal

    // The video ram on the SSD1306 is split up in to 8 rows, one bit per pixel.
    // Each row is 128 long by 8 pixels high, each byte vertically arranged, so byte 0 is x=0, y=0->7,
    // byte 1 is x = 1, y=0->7 etc

    // This code could be optimised, but is like this for clarity. The compiler
    // should do a half decent job optimising it anyway.

    const int BytesPerRow = 128; // 128 pixels, 1bpp, but each row is 8 pixel high, so (128 / 8) * 8

    int byte_idx = (y / 8) * BytesPerRow   +   x;
    uint8_t byte = Framebuffer[byte_idx];

    if (on)
        byte |=  1 << (y % 8);
    else
        byte &= ~(1 << (y % 8));

    Framebuffer[byte_idx] = byte;
}

// Basic Bresenhams.
static void DrawLine(int x0, int y0, int x1, int y1, bool on) {

    int dx =  abs(x1-x0);
    int sx = x0<x1 ? 1 : -1;
    int dy = -abs(y1-y0);
    int sy = y0<y1 ? 1 : -1;
    int err = dx+dy;
    int e2;

    while (true) {
        SetPixel(x0, y0, on);

        if (x0 == x1 && y0 == y1)
            break;
        e2 = 2*err;

        if (e2 >= dy)
        {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx)
        {
            err += dx;
            y0 += sy;
        }
    }
}

static uint8_t reverse(uint8_t b) {
   b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
   b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
   b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
   return b;
}


static inline int GetFontIndex(uint8_t ch) {
    if (ch >= 'A' && ch <='Z')
        return  ch - 'A' + 1;
    else if (ch >= '0' && ch <='9')
        return  ch - '0' + 27;
    else if (ch == '.')
        return 37;
    else
        return  0; // Not got that char so space.
}

static uint8_t reversed[sizeof(font)] = {0};

static void FillReversedCache() {
    // calculate and cache a reversed version of fhe font, because I defined it upside down...doh!
    for (int i=0;i<sizeof(font);i++)
        reversed[i] = reverse(font[i]);
}

static void WriteChar(uint x, uint y, uint8_t ch) {
    if (reversed[0] == 0)
        FillReversedCache();

    if (x > SSD1306_LCDWIDTH - 8 || y > SSD1306_LCDHEIGHT - 8)
        return;

    // For the moment, only write on Y row boundaries (every 8 vertical pixels)
    y = y/8;

    ch = toupper(ch);
    int idx = GetFontIndex(ch);
    int fb_idx = y * 128 + x;

    for (int i=0;i<8;i++) {
        Framebuffer[fb_idx++] = reversed[idx * 8 + i];
    }
}

static uint16_t ExpandByte(uint8_t b) {
    uint16_t w = 0;
    for (int i=7;i>=0;i--) {
        uint16_t t = (b & (1 << i));
        w |= (t << i);
        w |= (t << (i + 1));
    }
    return w;
}

static void WriteBigChar(uint x, uint y, uint8_t ch) {
    if (reversed[0] == 0)
        FillReversedCache();

    if (x > SSD1306_LCDWIDTH - 16 || y > SSD1306_LCDHEIGHT - 16)
        return;

    // For the moment, only write on Y row boundaries (every 8 vertical pixels)
    y = y/8;

    ch = toupper(ch);
    int idx = GetFontIndex(ch);
    int fb_idx = y * 128 + x;

    for (int i=0;i<8;i++) {
        uint16_t w = ExpandByte(reversed[idx * 8 + i]);
        Framebuffer[fb_idx] = w & 0x0ff;
        Framebuffer[fb_idx+1] = w & 0x0ff;
        Framebuffer[fb_idx+128] = w >> 8;
        Framebuffer[fb_idx+129] = w >> 8;
        fb_idx+=2;

    }
}

static void WriteString(int x, int y, uint8_t *str) {
    // Cull out any string off the screen
    if (x > SSD1306_LCDWIDTH - 8 || y > SSD1306_LCDHEIGHT - 8)
        return;

    while (*str) {
        WriteChar(x,y, *str++);
        x+=8;
    }
}

static void WriteBigString(int x, int y, uint8_t *str) {
    // Cull out any string off the screen
    if (x > SSD1306_LCDWIDTH - 16 || y > SSD1306_LCDHEIGHT - 16)
        return;

    while (*str) {
        WriteBigChar(x,y, *str++);
        x+=16;
    }
}

void update_oled(uint16_t *buf,const uint size_buf){
    uint32_t total_count = 0;
    double total_activity = 0.;
    char str[10];
    int max_val = 0;
    for(uint i=0; i<size_buf;i++){
    total_count=*(buf+i)+total_count;
    if (*(buf+i) > max_val) {
            max_val = *(buf+i);
        }
    }
    float height = (64-32)/(max_val+1);
    total_activity = (double)total_count/(size_buf*(PERIOD_INTEGRATION_COUNTS/1000));
    //snprintf(str, sizeof(str), "%d", total_count);
    if (((int)total_activity)<10){
        snprintf(str, sizeof(str), "%.1f", total_activity);
    }
    if (((int)total_activity)>10){
        snprintf(str, sizeof(str), "%e", (int)total_activity);
    }
    ClearDisplay();
    WriteBigString(90,8,"Bq");
    WriteBigString(8,8,str);
    for(uint i=0; i<size_buf;i++){
        for(uint j=0; j<2;j++){
        int local_height=(int)(*(buf+i)+1)*height;
        DrawLine(i*8+j,64-local_height,i*8+j, 64, true);
        //DrawLine(64-local_height,i*8+j,64,i*8+j,  true);
        }
    }

    
    

    UpdateDisplay();




}

// Multithreading stuff running on second core
void core1_event(){
  printf("Second core started\n");
  sleep_ms(50);
  i2c_init(I2C_PORT, 1000*I2C_CLOCK );
  sleep_ms(100);
  gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
  //gpio_pull_up(I2C_SDA_PIN);
  //gpio_pull_up(I2C_SCL_PIN);
  SSD1306_initialise();
  sleep_ms(100);
  ClearDisplay();
  sleep_ms(100);
  SendCommand(SSD1306_DISPLAYON);
  sleep_ms(100);
  //SendCommand(SSD1306_INVERTDISPLAY);
  //sleep_ms(100);
  
  gpio_init(LED_R_PIN); gpio_set_dir(LED_R_PIN, GPIO_OUT);gpio_put(LED_R_PIN,1); 
  gpio_init(SWITCH_1_PIN); gpio_set_dir(SWITCH_1_PIN, GPIO_IN);gpio_pull_up(SWITCH_1_PIN);
  gpio_init(SWITCH_2_PIN); gpio_set_dir(SWITCH_2_PIN, GPIO_IN);gpio_pull_up(SWITCH_2_PIN);
  gpio_init(SWITCH_3_PIN); gpio_set_dir(SWITCH_3_PIN, GPIO_IN);gpio_pull_up(SWITCH_3_PIN);
  gpio_init(SWITCH_4_PIN); gpio_set_dir(SWITCH_4_PIN, GPIO_IN);gpio_pull_up(SWITCH_4_PIN);
  sleep_ms(50);
  gpio_init(LED_O_PIN); gpio_set_dir(LED_O_PIN, GPIO_OUT);gpio_put(LED_O_PIN,1); 
  gpio_init(LED_G_PIN); gpio_set_dir(LED_G_PIN, GPIO_OUT);gpio_put(LED_G_PIN,1);
  DrawLine(0,5,SSD1306_LCDWIDTH-1, 5, true);
  DrawLine(0,10,SSD1306_LCDWIDTH-1, 10, true);
  DrawLine(0,15,SSD1306_LCDWIDTH-1, 15, true);
  DrawLine(0,51,SSD1306_LCDWIDTH-1, 51, true);
  DrawLine(0,56,SSD1306_LCDWIDTH-1, 56, true);
  DrawLine(0,61,SSD1306_LCDWIDTH-1, 61, true);
  ClearDisplay();
  sleep_ms(100);
  WriteBigString(0,31,"HELLO");
  UpdateDisplay();

  // Initialize I2C port

  printf("Starting the multicore\n");

  printf("Second core setup\n");

  while(thread_alive){
    if(!gpio_get(SWITCH_1_PIN)){
        print_measurements(counts_array_moving_avg,LENGHT_MOVING_AVG);
        sleep_ms(1000);
        }
    if(!gpio_get(SWITCH_2_PIN)){
        print_bequerel(counts_array_moving_avg,LENGHT_MOVING_AVG);
        sleep_ms(1000);
        }
    update_oled(counts_array_moving_avg,LENGHT_MOVING_AVG);
    sleep_ms(500);
    /*if(single_measurement_triggered){
      read_all_chn_speed(SPI_PORT_1,0x00,(MEASUREMENTS_BANK_1_BUFFER_0+timer_counter*2),SPI_Csn_1);
      read_all_chn_speed(SPI_PORT_1,0x01,(MEASUREMENTS_BANK_1_BUFFER_1+timer_counter*2),SPI_Csn_1);
      read_all_chn_speed(SPI_PORT_1,WEIRD_CHIP,(MEASUREMENTS_BANK_1_BUFFER_4+timer_counter*2),SPI_Csn_1);
      single_measurement_triggered = false;
    }*/
    //;
    //tight_loop_contents();
  }
}

int main()
{
    stdio_init_all();
    // Setup of all GPIOs
    gpio_set_irq_callback(&counter_callback);
    gpio_set_irq_enabled(HV_TUBE_INT_PIN, GPIO_IRQ_EDGE_FALL, true);
    gpio_init(BUZZER_PIN); gpio_set_dir(BUZZER_PIN, GPIO_OUT);gpio_put(BUZZER_PIN,0);
    gpio_set_function(HV_TUBE_PSU_PIN, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(HV_TUBE_PSU_PIN); // find out to which slice Pin16 is connected to => should be slice 0
    pwm_set_wrap(slice_num, 25000); // Setting period lenght in clock cycles here for cpu clocked at 150MHz
    pwm_set_chan_level(slice_num, 0, 15000); // When to break circuit 
    pwm_set_enabled(slice_num, true);
    printf("HV is on!\n");
    multicore_launch_core1(&core1_event);
    gpio_init(HV_TUBE_INT_PIN); gpio_set_dir(HV_TUBE_INT_PIN, GPIO_IN);
    irq_set_enabled(IO_IRQ_BANK0,true);
    add_repeating_timer_ms(PERIOD_INTEGRATION_COUNTS, alarm_callback, NULL, &timer);
    thread_alive=true;
    

    while (true) {
        if(!gpio_get(SWITCH_1_PIN)){
        print_measurements(counts_array_moving_avg,LENGHT_MOVING_AVG);
        sleep_ms(1000);
        }
        if(!gpio_get(SWITCH_3_PIN)){
        print_bequerel(counts_array_moving_avg,LENGHT_MOVING_AVG);
        sleep_ms(1000);
        }
    }
    cancel_repeating_timer (&timer);
}
