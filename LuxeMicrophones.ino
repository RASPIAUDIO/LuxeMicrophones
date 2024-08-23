
#include <Arduino.h>
#include <Wire.h>
#include "ESP_I2S.h"
#include "driver/gpio.h"
#include "esp_mac.h"
#include "SPI.h"
#include "FS.h"
#include "SD.h"
#include "SPIFFS.h"

// Initialize your objects and variables
I2SClass i2s;

// Define your pins and other constants
#define I2S_SCK 5
#define I2S_WS 25
#define I2S_SDOUT 26
#define I2S_MCLK 0
#define I2S_SDIN 35
#define PA (gpio_num_t)21
#define SDA 18
#define SCL 23
#define MU GPIO_NUM_12      // Pause/Play
#define VM GPIO_NUM_32      // Vol-
#define VP GPIO_NUM_19      // Vol+ 
#define SD_CS         13
#define SPI_MOSI      15
#define SPI_MISO      2
#define SPI_SCK       14





#define bytesToRead 2000
  uint8_t b[bytesToRead];

#define maxVol 50
  int n;
  size_t t;

bool BPause =false;
uint8_t mac [6];
char macStr[20];
char dev_name [30];
int vol, Pvol;

#define ES8388_ADDR 0x10

///////////////////////////////////////////////////////////////////////
// Write ES8388 register (using I2c)
///////////////////////////////////////////////////////////////////////
uint8_t ES8388_Write_Reg(uint8_t reg, uint8_t val)
{
  uint8_t buf[2], res;
  buf[0] = reg;
  buf[1] = val;
  Wire.beginTransmission(ES8388_ADDR);
  Wire.write(buf, 2);
  res = Wire.endTransmission();
  //    printf("%d\n", res);
  return res;
}

int ES8388_Init(void)
{
  // provides MCLK
  //  PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0_CLK_OUT1);
  //  WRITE_PERI_REG(PIN_CTRL, READ_PERI_REG(PIN_CTRL)& 0xFFFFFFF0);
  uint8_t st;
  // reset
  st = 0;
  st += ES8388_Write_Reg(0, 0x80);
  st += ES8388_Write_Reg(0, 0x00);
  // mute
  st += ES8388_Write_Reg(25, 0x04);
  st += ES8388_Write_Reg(1, 0x50);
  // powerup
  st += ES8388_Write_Reg(2, 0x00);
  // slave mode
  st += ES8388_Write_Reg(8, 0x00);
  // DAC powerdown
  st += ES8388_Write_Reg(4, 0xC0);
  // vmidsel/500k ADC/DAC idem
  st += ES8388_Write_Reg(0, 0x12);

  st += ES8388_Write_Reg(1, 0x00);
  // i2s 16 bits
  st += ES8388_Write_Reg(23, 0x18);
  // sample freq 256
  st += ES8388_Write_Reg(24, 0x02);

  // left DAC to left mixer
  st += ES8388_Write_Reg(39, 0x90);
  // right DAC to right mixer
  st += ES8388_Write_Reg(42, 0x90);
  // DACLRC ADCLRC idem
  st += ES8388_Write_Reg(43, 0x80);
  st += ES8388_Write_Reg(45, 0x00);
  // DAC volume max
  st += ES8388_Write_Reg(27, 0x00);
  st += ES8388_Write_Reg(26, 0x00);

  //ROUT1/LOUT1 volume max
  st += ES8388_Write_Reg(46, 0x21);
  st += ES8388_Write_Reg(47, 0x21);
  //ROUT2/LOUT2 volume max
  st += ES8388_Write_Reg(48, 0x21);
  st += ES8388_Write_Reg(49, 0x21);

  st += ES8388_Write_Reg(2 , 0xF0);
  st += ES8388_Write_Reg(2 , 0x00);
  st += ES8388_Write_Reg(29, 0x1C);
  // DAC power-up LOUT1/ROUT1 enabled
  st += ES8388_Write_Reg(4, 0x3C);
  // unmute
  st += ES8388_Write_Reg(25, 0x00);
  // amp validation
  gpio_set_level(PA, 1);
  return st;
}

///////////////////////////////////////////////////////////////////////
// ES8388 microphones init
///////////////////////////////////////////////////////////////////////
int ES8388_Microphones_Init(void)
{
  uint8_t st;
  st = 0;

 // ADC poweroff
   st += ES8388_Write_Reg(3,0xFF);
   // ADC amp 24dB
      st += ES8388_Write_Reg(9,0x88);   
   // LINPUT1/RINPUT1  
   st += ES8388_Write_Reg(10,0);
   // ADC stereo
   st += ES8388_Write_Reg(11,0x00);   
   //i2S 16b
//   st += ES8388_Write_Reg(12,0x8C); // ext micro only
//   st += ES8388_Write_Reg(12,0x4C); // int micro only
   st += ES8388_Write_Reg(12,0x0C); // both
     
   //MCLK 256
   st += ES8388_Write_Reg(13,0x02); 
   // ADC high pass filter
  // ES8388_Write_Reg(14,0x30);    
   // ADC Volume   
   st += ES8388_Write_Reg(16,0x00);
   st += ES8388_Write_Reg(17,0x00);
   // ALC OFF
  // ES8388_Write_Reg(18,0x3F);
   st += ES8388_Write_Reg(3,0x09);
   return st;
}
////////////////////////////////////////////////////////////////////////
//
// manages volume (via vol xOUT1, vol DAC)
//
////////////////////////////////////////////////////////////////////////
void ES8388vol_Set(uint8_t volx)
{
#define M maxVol-33
  printf("volume ==> %d\n", volx);
  ES8388_Write_Reg(25, 0x00);
  if (volx > maxVol) volx = maxVol;
  if (volx == 0)
  {
    ES8388_Write_Reg(25, 0x04);
  }
  if (volx >= M)
  {
    ES8388_Write_Reg(46, volx - M);
    ES8388_Write_Reg(47, volx - M);
    ES8388_Write_Reg(48, volx - M);
    ES8388_Write_Reg(49, volx - M);    
    ES8388_Write_Reg(26, 0x00);
    ES8388_Write_Reg(27, 0x00);

  }
  else
  {
    ES8388_Write_Reg(46, 0x00);
    ES8388_Write_Reg(47, 0x00);
    ES8388_Write_Reg(48, 0x00);
    ES8388_Write_Reg(49, 0x00);    
    ES8388_Write_Reg(26, (M - volx) * 3);
    ES8388_Write_Reg(27, (M - volx) * 3);
  }
}

void setup() {
  Serial.begin(115200);
////// GPIOs init  
  // power enable
  gpio_reset_pin(PA);
  gpio_set_direction(PA, GPIO_MODE_OUTPUT);

  //VP
  gpio_reset_pin(VP);
  gpio_set_direction(VP, GPIO_MODE_INPUT);
  gpio_set_pull_mode(VP, GPIO_PULLUP_ONLY);


  //VM
  gpio_reset_pin(VM);
  gpio_set_direction(VM, GPIO_MODE_INPUT);
  gpio_set_pull_mode(VM, GPIO_PULLUP_ONLY);

  // pause
  gpio_reset_pin(MU);
  gpio_set_direction(MU, GPIO_MODE_INPUT);
  gpio_set_pull_mode(MU, GPIO_PULLUP_ONLY);
  
//////I2S init
  i2s.setPins(I2S_SCK, I2S_WS, I2S_SDOUT, I2S_SDIN, I2S_MCLK);
  if (!i2s.begin(I2S_MODE_STD, 44100, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO, I2S_STD_SLOT_BOTH)) {
    Serial.println("Failed to initialize I2S!");
    while (1); // do nothing
  }
//////I2C init
  Wire.setPins(SDA, SCL);
  Wire.begin();

////// ES8388 codec init
  if(ES8388_Init() != 0)
  {
    printf("Codec init failed\n");
    return;
  }
  if(ES8388_Microphones_Init() != 0)
  {
    printf("Microphones init failed\n");
    return;          
  }
  printf("Codec init OK\n");

////// volume init
  vol = maxVol;
  ES8388vol_Set(vol);
  
///// SD init
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  delay(500);
  if(!SD.begin(SD_CS))printf("init. SD failed !\n");

  File f = SD.open("/record.wav", FILE_WRITE);
// Start recording
  while(gpio_get_level(MU) == 1) delay(10);   
  while(gpio_get_level(MU) == 0) delay(10);
  printf("RECORDING...\n");
  i2s.read();
  while(gpio_get_level(MU) == 1)
  {
    n = i2s.available();
    printf(" n = %d \n", n);    
    i2s.readBytes((char*)b, n);
    f.write(b, n);
  }
  f.close();      

  delay(2000);

  printf("PLAYING\n");      
  f = SD.open("/record.wav", FILE_READ);
//  f.seek(44);   
  do
  {
    n = f.read(b, bytesToRead); 
    i2s.write(b, n);     
  }while(n > 0);
 // i2s_zero_dma_buffer(I2SR);
  i2s.end();
  f.close();  
  printf("STOP\n");
}

void loop() {
    delay(100);
  }
