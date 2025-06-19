#include <Wire.h>

#define BME280_ADDRESS 0X76


int32_t t_fine;
uint16_t dig_T1;
int16_t dig_T2, dig_T3;
uint16_t dig_P1;
int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
uint8_t dig_H1, dig_H3;
int16_t dig_H2, dig_H4, dig_H5;
int8_t dig_H6;

//BME280 write sends the slave address, followed by pairs of register addresses and register data
void I2C_Write(uint8_t register_address, uint8_t data){
  Wire.beginTransmission(BME280_ADDRESS); //slave address
  Wire.write(register_address); //register address
  Wire.write(data); //data to write
  Wire.endTransmission();
}


void I2C_Read(uint8_t register_address, uint8_t* buffer, uint8_t length){
  Wire.beginTransmission(BME280_ADDRESS); //slave address
  Wire.write(register_address); //register address
  Wire.endTransmission(false); //repeated stop condition per bme280 docs
  Wire.requestFrom(BME280_ADDRESS, length); //request 8 bytes, 3 registers for temp/press and 2 for humidity

  //loop to read registers if more than 1 
  for(uint8_t i=0; i<length; i++){
    if(Wire.available()){
      buffer[i] = Wire.read();
    }
  }
}

int32_t BME280_compensate_T_int32(int32_t adc_T){
    int32_t var1, var2, T;
    var1 = ((((adc_T>>3)-((int32_t)dig_T1<<1)))*((int32_t)dig_T2))>>11;
    var2 = (((((adc_T>>4)-((int32_t)dig_T1))*((adc_T>>4)-((int32_t)dig_T1)))>>12)*((int32_t)dig_T3))>>14;
    t_fine = var1 + var2;
    T=(t_fine * 5 + 128) >> 8;
    return T;
  }

  //compensate pressure values
  uint32_t BME280_compensate_P_uint64(int32_t adc_P){
    int64_t var1, var2, p;
    var1 = ((int64_t)t_fine)-128000;
    var2 = var1 * var1 * (int64_t)dig_P6;
    var2 = var2 + ((var1 * (int64_t)dig_P5)<<17);
    var2 = var2 + (((int64_t)dig_P4)<<35);
    var1 = ((var1 * var1 * (int64_t)dig_P3)>>8) + ((var1 * (int64_t)dig_P2)<<12);
    var1 = (((((int64_t)1)<<47)+var1)) * ((int64_t)dig_P1)>>33;
    if(var1 == 0) {
      return 0;
    }
    p = 1048576 - adc_P;
    p = (((p<<31)-var2)*3125)/var1;
    var1 = (((int64_t)dig_P9) * (p>>13)*(p>>13)) >> 25;
    var2 = (((int64_t)dig_P8)*p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7)<<4);
    return (uint32_t)p;
  }

  //compensate humdity values
  uint32_t bme280_compensate_H_int32(int32_t adc_H){
    int32_t v_x1_u32r;
    v_x1_u32r = (t_fine-((int32_t)76800));
    v_x1_u32r = (((((adc_H << 14) - (((int32_t)dig_H4) << 20) - (((int32_t)dig_H5) *
    v_x1_u32r)) + ((int32_t)16384)) >> 15) * (((((((v_x1_u32r *
    ((int32_t)dig_H6)) >> 10) * (((v_x1_u32r * ((int32_t)dig_H3)) >> 11) +
    ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)dig_H2) +
    8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
    ((int32_t)dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    return (uint32_t)(v_x1_u32r>>12);
  }

void setup(){
  Serial.begin(115200);
  Wire.begin();
  
  //verifying bme280 is connected
  uint8_t id;
  I2C_Read(0xD0, &id, 1);
  if(id != 0x60){
    Serial.print("Unexpected Chip ID: 0x");
    Serial.println(id, HEX);
    while(true);
  }else{
    Serial.println("BME280 connection succesful");
  }

  //configuring sensor
  I2C_Write(0xF2, 0x01); //ctrl_hum register, changes occur after 0xF4 
  I2C_Write(0xF5, 0x58); //config register, sets rate, filter and interface options. 
  I2C_Write(0xF4, 0xB7); //ctrl_meas register, sets pressure & temp data acquisition+sensor mode. 
  delay(1000);

  uint8_t calib_data1[24];
  uint8_t calib_data2[1];
  uint8_t calib_data3[8];

  //read temp+pressure calibration values
  I2C_Read(0x88, calib_data1, 24);
    dig_T1 = (uint16_t)(calib_data1[1] << 8) | calib_data1[0];
    dig_T2 = (int16_t)(calib_data1[3] << 8) | calib_data1[2];
    dig_T3 = (int16_t)(calib_data1[5] << 8) | calib_data1[4];

    dig_P1 =(uint16_t)(calib_data1[7] << 8) | calib_data1[6];
    dig_P2 =(int16_t)(calib_data1[9] << 8) | calib_data1[8];
    dig_P3 =(int16_t)(calib_data1[11] << 8) | calib_data1[10];
    dig_P4 =(int16_t)(calib_data1[13] << 8) | calib_data1[12];
    dig_P5 =(int16_t)(calib_data1[15] << 8) | calib_data1[14];
    dig_P6 =(int16_t)(calib_data1[17] << 8) | calib_data1[16];
    dig_P7 =(int16_t)(calib_data1[19] << 8) | calib_data1[18];
    dig_P8 =(int16_t)(calib_data1[21] << 8) | calib_data1[20];
    dig_P9 =(int16_t)(calib_data1[23] << 8) | calib_data1[22];



  I2C_Read(0xA1, calib_data2, 1);
    dig_H1 = calib_data2[0];

  I2C_Read(0xE1, calib_data3, 7);
    dig_H2 = (int16_t)(calib_data3[1] << 8) | calib_data3[0];
    dig_H3 = calib_data3[2];
    dig_H4 = (int16_t)((calib_data3[3] << 4) | (calib_data3[4] & 0x0F));
    dig_H5 = (int16_t)((calib_data3[5] << 4) | calib_data3[4] >> 4);
    dig_H6 = (int8_t)calib_data3[6];

  Serial.print("dig_T1: "); Serial.println(dig_T1);
  Serial.print("dig_P1: "); Serial.println(dig_P1);
  Serial.print("dig_H1: "); Serial.println(dig_H1);

}


void loop(){

  uint8_t data[8];
  I2C_Read(0xF7, data, 8);

  //takes raw values from registers and pieces them into a 20-bit number(except humidity, its 16-bits)
  int32_t adc_Pressure = ((uint32_t)data[0] << 12) | ((uint32_t)data[1] << 4) | ((uint32_t)data[2] >> 4);
  int32_t adc_Temp = ((uint32_t)data[3] << 12) | ((uint32_t)data[4] << 4) | ((uint32_t)data[5] >> 4 );
  int32_t adc_Humidity = ((uint32_t)data[6] <<8) | data[7];

  int32_t temp = BME280_compensate_T_int32(adc_Temp);
  uint32_t pressure = BME280_compensate_P_uint64(adc_Pressure);
  uint32_t humidity = bme280_compensate_H_int32(adc_Humidity);

  Serial.print("Temp celsius: ");
  Serial.println(temp/100.0);
  Serial.print("Pressure hPa: ");
  Serial.println(pressure/25600.0);
  Serial.print("Humidity (%): ");
  Serial.println(humidity/1024.0);
  delay(1000);

}
