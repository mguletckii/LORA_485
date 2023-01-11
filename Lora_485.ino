
#include <SoftwareSerial.h>
#include "EBYTE22.h"


// Константы для плат на ESP8266

#define PIN_M0 16
#define PIN_M1 10
#define PIN_RX 2
#define PIN_TX 0
#define PIN_AX 14

#define PIN_485_TX 12
#define PIN_485_RX 13

#define PIN_USB_TX 1
#define PIN_USB_RX 3




SoftwareSerial Serial485(PIN_485_RX, PIN_485_TX);

SoftwareSerial E22Serial(PIN_TX, PIN_RX, false);    // Создаем объект SoftwareSerial для соединения с модулем через программный UART

SoftwareSerial Serial_USB(PIN_USB_RX, PIN_USB_TX);


EBYTE22 E22(&E22Serial, PIN_M0, PIN_M1, PIN_AX);   // Создаем экземпляр класса EBYTE22


byte rx_buffer_serial[512];
int rx_buffer_size = 0;

byte tx_buffer_serial[512];
int tx_buffer_size = 0;

byte buffer_comp[512];
int comp_size;
int decomp_size;

int i = 0;

int busy_counter = 0;

bool send_status;

int serial_speed = 19200;

int active_port = 0;

void E22_configurate() {
  E22Serial.end();
  E22Serial.begin(9600);
  delay(200);
   // При инициализации и некоторых других операциях (применение настроек, установка ключа), модуль переходит в режим конфигурации, в этом режиме UART модуля работает только на скорости 9600!!!
  E22.init(); // Инициализируем модуль(конфигурируются выводы контроллера, считываются настройки модуля).

  delay(200);
  E22.setMode(MODE_CONFIG);
  delay(200);
  E22.setUARTBaudRate(UBR_38400);
  
  E22.setAirDataRate(ADR_9600);

  E22.writeSettings(TEMPORARY);
  delay(200);
  E22.setMode(MODE_NORMAL);
  delay(200);
  E22Serial.end();
  E22Serial.begin(38400); 
}

void E22_busy() {

  while (E22.getBusy()){                        // Если модуль чем-то занят (на выводе AUX логический 0), 
    E22.completeTask(1000);                  // подождем, пока освободится с таймаутом в 5 секунд.

    busy_counter++;
    if (busy_counter > 6){
      E22_configurate();
      busy_counter = 0;
    }
  }
 
  busy_counter = 0;
}

void setup() {
  Serial_USB.begin(serial_speed, SWSERIAL_8E1);  // Устанавливаем соединение с компьютером (телефоном) для отладки.
  E22Serial.begin(9600);                    // Устанавливаем соединение с модулем (скорость 9600).
  Serial485.begin(serial_speed);
  
 
  E22_configurate();
 
}

void loop() {

  E22_busy();

  while((Serial_USB.available()) && ((active_port = 0)||(active_port = 1))){
    for(i = 0; i <= Serial_USB.available(); i++){
      rx_buffer_serial[rx_buffer_size] = Serial_USB.read();
      rx_buffer_size++;
    }
    delayMicroseconds(16000000/serial_speed);
    
    active_port = 1;
    
    if (rx_buffer_size > sizeof(rx_buffer_serial)){ 
      active_port = 0;
      rx_buffer_size = 0;
      break;  
    }
    
  }

  E22_busy();

  while((Serial485.available()) && ((active_port = 0)||(active_port = 2))){
    for(i = 0; i <= Serial485.available(); i++){
      rx_buffer_serial[rx_buffer_size] = Serial485.read();
      rx_buffer_size++;
    }
    delayMicroseconds(16000000/serial_speed);

    active_port = 2;
    
    if (rx_buffer_size > sizeof(rx_buffer_serial)){
      active_port = 0;
      rx_buffer_size = 0;
      break;  
    }
  }

  E22_busy();
  
  if(rx_buffer_size > 1){
    comp_size = dub_compress(rx_buffer_serial, rx_buffer_size, buffer_comp);

    if (comp_size > 0){   //Проверяем сжат ли пакет
      E22.sendByte(comp_size); //Отправляем размер пакета
      send_status = E22.sendStruct(&buffer_comp, comp_size);
    }else{
      E22.sendByte(rx_buffer_size); //Отправляем размер пакета
      send_status = E22.sendStruct(&rx_buffer_serial, rx_buffer_size);
    }
    E22.flush();
    
    rx_buffer_size = 0;  //Обнуляем счётчик
    comp_size = 0;
  }

  E22_busy();
  
  if (E22.available()){ 
    tx_buffer_size = E22.getByte();
    if (E22.getStruct(&tx_buffer_serial, tx_buffer_size)) {
      decomp_size = dub_decompress(tx_buffer_serial, tx_buffer_size, buffer_comp);
      if (active_port = 1){ Serial_USB.write(buffer_comp, decomp_size); }
      if (active_port = 2){ Serial485.write(buffer_comp, decomp_size); }
    }else{
     // Serial.println("Lora Received " + String(tx_buffer_size) + " bytes! Lost Packet!");  
    }
    
    active_port = 0;
    tx_buffer_size = 0;  //Обнуляем счётчик
    decomp_size = 0;
  }
}

int dub_compress(const byte *in, int len, byte *out){ // simple compression for Modbus like packages 
  int j = 0;
  int dub_count = 0;
  if (len > 10){
    for (int i=0; i<(len); i++){
      if ((in[i] == in[i+1])&(in[i] == in[i+2])&(in[i] == in[i+3])&(dub_count<63)){
        if (dub_count == 0){
          dub_count = 4;
        }else{
          dub_count++;
        }
      }else{
        if (dub_count > 0){
          out[j] = 0xFF;
          out[j+2] = 0b11000000 | dub_count;
          out[j+1] = in[i];
          j = j + 3;
          i = i + 2;
          dub_count = 0;
        }else{
          out[j] = in[i];
          j++;
        }  
      }
    }
    if (dub_count > 0){
         out[j] = 0xFF;
         out[j+2] = 0b11000000 | dub_count;
         out[j+1] = in[len-1];
         j = j + 3;
         dub_count = 0;
    }
    return j;
  }else{
    return 0;
  }
}

int dub_decompress(const byte *in, int len, byte *out){
   int j = 0;
   int g = 0;
   int count = 0;
   if (len > 3){
    for (int i=0; i<(len); i++){
     
      if ((in[i] == 0xFF)&((in[i+2]&0b11000000) == 0xC0)){
        count = (in[i+2] & 0b00111111);
        for(g=0; g < count; g++){
           out[j] = in[i+1];
           j++; 
        }
        i = i + 2;
      }else{
        out[j] = in[i];
        j++;
      }  
    }
    return j;
  }else{
    return 0;
  }
}
