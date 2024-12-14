#include <U8g2lib.h>
#include <Wire.h>

#define YERCEKIMI       9.81
#define ALPHA           0.1

// U8g2 ekran tanımı (SW SPI)
U8G2_ST7920_128X64_F_SW_SPI u8g2(U8G2_R0, /* clock=*/ 13, /* data=*/ 11, /* CS=*/ 10, /* reset=*/ 8);

//MPU6050 tanımlamaları
#define MPU_I2C_ADDR    0X68 //MPU6050 sensörünün I2C adresi -> datasheet veya I2C scanner
#define PWR_MGMT_1      0x6B //reset-sleep-temp-clk işlemleri için register adresi
#define SMPRT_DIV       0x19 //clk ayarı için register ad/Users/zeynepkaplan/Documentsresi
#define GYRO_CONFIG     0x1B //Gyroscope ölçüm ayarı register adresi
#define ACCEL_CONFIG    0x1C //Accelerometer ölçüm ayarı register adresi
#define ACCEL_XOUT_H    0x3B //Accelerometer verileri için okumaya başlanacak adres
#define GYRO_XOUT_H     0x43 //Gyroscope verileri için okumaya başlanacak adres
#define TEMP_OUT_H      0x41 //Sıcaklık verisi için başlangıç adresi

float accX, accY, accZ;
float gyroX, gyroY, gyroZ;
float hizX, hizY, hizZ;

// Grafiğin ayarları
#define ekran_genislik  128  
#define ekran_yukseklik 64 
#define genislik        25   // Grafiğin genişliği
#define yukseklik       54   // Grafiğin yüksekliği
#define x_baslangic     12    // Grafiğin sol başlangıç noktası
#define y_baslangic     42   // Grafiğin alt başlangıç noktası
#define x_baslangic_2   40  

uint16_t baslangic_zaman = 0; //zaman farkı için bir değişken
uint16_t anlik_zaman = 0;
uint16_t dongu_dizi[3] = {0}; //grafiğin sağa kayması için kontrol noktası

int16_t grafik_veri[genislik] = {0}; //grafik verilerinin tutulduğu dizi
int16_t grafik_veri_2[genislik] = {0};
int16_t grafik_veri_3[genislik] = {0};

//Sıcaklık ve Nem tanımlamaları
#define DHT_PIN 4 //DHT11 in bağlı olduğu pin numarası

int8_t presence, rh_byte1, rh_byte2, temp_byte1, temp_byte2, checksum;
float temp, hum;

void setup() {
  Wire.begin(); //I2C başlatma
  Mpu_ayarlar();
  u8g2.begin(); 
  baslangic_zaman = millis();
}

void loop() {
  anlik_zaman = millis();
  if(anlik_zaman - baslangic_zaman >= 500){ //her yarım saniyede grafiği çizdirme
    DHT_oku();
    Mpu_oku();
    delay(10);
    baslangic_zaman = anlik_zaman;
    grafik_ciz("Ivme Grafigi(m/s2)", "ax", "ay", "az", "m/s2", accX, accY, accZ, 20, 15, 10); //başlık, x-ekseni, y-ekseni ve altyazının çizilmesi
    //grafik_ciz("Gyro Grafigi(rad/s)", "gx", "gy", "gz", "rad/s", gyroX, gyroY, gyroZ, 10, 15, 10);
    //grafik_ciz("HIZ Grafigi(m/s)", "Vx", "Vy", "Vz", "m/s", hizX, hizY, hizZ, 10, 15, 10);
    //sicaklik_nem_goster();
  }
}

void Mpu_ayarlar() {
  Wire.beginTransmission(MPU_I2C_ADDR); //mpu ile iletişimi başlatma
  Wire.write(PWR_MGMT_1); //PWR_MGMT_1 adresinde işlem
  Wire.write(0x00); //tüm bitler 0 ayarlandı çünkü sensörden sürekli okuma yapılacak
  Wire.endTransmission(true);
  
  // Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
  //DLPF aktif değilse(default olarak aktif değil) Gyroscope Output Rate = 8kHz bu yüzden sample rate oranı 1kHz için SMPLRT_DIV 7 ayarlanmalı
  Wire.beginTransmission(MPU_I2C_ADDR);
  Wire.write(SMPRT_DIV); //SMPRT_DIV adresinde işlem
  Wire.write(0x07); 
  Wire.endTransmission(true);

  //GYRO_CONFIG ayarı -> ± 250 ̐/s için böylece daha hassas ölçüm
  Wire.beginTransmission(MPU_I2C_ADDR);
  Wire.write(GYRO_CONFIG); //GYRO_CONFIG adresinde işlem
  Wire.write(0x00); // 0 ayarlandı çünkü ± 250 ̐/s
  Wire.endTransmission(true);
  //ACCEL_CONFIG ayarı -> ± 2g
  Wire.beginTransmission(MPU_I2C_ADDR); 
  Wire.write(ACCEL_CONFIG); //ACCEL_CONFIG adresinde işlem
  Wire.write(0x00); // 0 ayarlandı çünkü ± 2g
  Wire.endTransmission(true);
  delay(200);
}

void Mpu_oku() {
  //ACC ölçüm
  Wire.beginTransmission(MPU_I2C_ADDR); 
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_I2C_ADDR, 6, true); //6 register adresi okuma çünkü her acc verisi için HIGH VE LOW adlı iki ayrı register

  //± 2g için hassaslık 16384 LSB/g --> okunan verilere bölünerek g formatı elde edilir
  // g formatı elde edildikten sonra m/s² birimine çevirmek için '1 g = 9.80665 m/s²' değerine çarpılır
  accX = ( ( ( Wire.read() << 8 ) | Wire.read() ) / 16384.0 ) * 9.80665 * ALPHA;
  accY = ( ( ( Wire.read() << 8 ) | Wire.read() ) / 16384.0 ) * 9.80665 * ALPHA;
  accZ = ( ( ( ( Wire.read() << 8 ) | Wire.read() ) / 16384.0 ) * 9.80665 - YERCEKIMI ) * ALPHA; //yer çekime çıkarma(9.81)

  //GYRO ölçüm
  Wire.beginTransmission(MPU_I2C_ADDR); 
  Wire.write(GYRO_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_I2C_ADDR, 6, true); //6 register adresi okuma çünkü her gyro verisi için HIGH VE LOW adlı iki ayrı register

  //± 250 ̐/s için hassaslık 131 LSB /°/s --> okunan verilere bölünerek dps(°/s) formatı elde edilir
  //rad/s birimini (π / 180) değeri ile çarparak elde edebiliriz
  gyroX = ( ( ( Wire.read() << 8 ) | Wire.read() ) / 131.0 ) * ( PI / 180.0 );
  gyroY = ( ( ( Wire.read() << 8 ) | Wire.read() ) / 131.0 ) * ( PI / 180.0 );
  gyroZ = ( ( ( Wire.read() << 8 ) | Wire.read() ) / 131.0 ) * ( PI / 180.0 );

  hizX = accX * (anlik_zaman - baslangic_zaman)/1000; //ms -> s
  hizY = accY * (anlik_zaman - baslangic_zaman)/1000;
  hizZ = accZ * (anlik_zaman - baslangic_zaman)/1000;
}

void eksenleri_ciz(char *sembol, float veri, uint8_t kayma_sayisi, uint16_t *grafik, uint16_t &dongu, int8_t max, int8_t max_sinir, int8_t k){
  //X ekseni
  u8g2.drawLine(x_baslangic + kayma_sayisi, yukseklik/2, x_baslangic + genislik + 6 + kayma_sayisi, yukseklik/2);
  u8g2.setFont(u8g2_font_tinyface_te);
  u8g2.setCursor(x_baslangic + genislik + 5 + kayma_sayisi, yukseklik/2 + 3);
  u8g2.print(">");
  u8g2.setCursor(x_baslangic + genislik + 7 + kayma_sayisi, yukseklik/2 + 6);
  u8g2.print("t");
  //Y ekseni
  u8g2.drawLine(x_baslangic + kayma_sayisi, y_baslangic, x_baslangic + kayma_sayisi, yukseklik - y_baslangic);
  u8g2.setCursor(x_baslangic - 1 + kayma_sayisi, yukseklik - y_baslangic + 3);
  u8g2.print("^");
  u8g2.setCursor(x_baslangic - 8 + kayma_sayisi, yukseklik - y_baslangic + 4);
  u8g2.print(sembol);
  u8g2.setCursor(x_baslangic - 11 + kayma_sayisi, y_baslangic);
  u8g2.print("-");
  u8g2.print(sembol);
  veri = map(veri*k , -max, max, -max_sinir, max_sinir);
  if(dongu > genislik - 1) { //grafik x ekseni genişliğine ulaşmışsa kaydırma işlemi
    for (uint16_t i = 0; i < genislik - 1; i++) {
      grafik[i] = grafik[i + 1];
    }
    grafik[genislik - 1] = veri;
  }
  else{
    grafik[dongu] = veri;
  }

  //grafiğin dongu sayısına kadar çizdirilmesi
  for (uint16_t i = 0; i < dongu && i < genislik - 1; i++) {
    u8g2.drawLine(x_baslangic + i + kayma_sayisi, yukseklik / 2 - grafik[i], x_baslangic + i + 1 + kayma_sayisi, yukseklik / 2 - grafik[i + 1]);
  }
  delay(100);
  dongu++;
}

void grafik_ciz(char *baslik, char *sembol_x, char *sembol_y, char *sembol_z, char *birim, float veri_x, float veri_y, float veri_z, int8_t max, int8_t max_sinir, int8_t k) {
  u8g2.clearBuffer(); //ekranı temizle
  //Başlık
  u8g2.setFont(u8g2_font_micropixel_tf);
  u8g2.setCursor(0, 6);
  u8g2.print(baslik);

  //eksenler
  eksenleri_ciz(sembol_x, veri_x, 0, grafik_veri, dongu_dizi[0], max, max_sinir, k); //1.grafik
  eksenleri_ciz(sembol_y, veri_y, genislik + 15, grafik_veri_2, dongu_dizi[1], max, max_sinir, k); //2.grafik
  eksenleri_ciz(sembol_z, veri_z, 2*(genislik + 15), grafik_veri_3, dongu_dizi[2], max, max_sinir, k); //3.grafik

  //altyazı bölümü
  u8g2.drawLine(0, y_baslangic + 4, ekran_genislik, y_baslangic + 4);
  u8g2.setCursor(2, y_baslangic + 12);
  u8g2.print(sembol_x);
  u8g2.print(" = ");
  u8g2.print(veri_x);
  u8g2.print(birim);
  u8g2.setCursor(ekran_genislik/3 - 2, y_baslangic + 20);
  u8g2.print(sembol_y);
  u8g2.print(" = ");
  u8g2.print(veri_y);
  u8g2.print(birim);
  u8g2.setCursor(ekran_genislik/2 + 3, y_baslangic + 12);
  u8g2.print(sembol_z);
  u8g2.print(" = ");
  u8g2.print(veri_z);
  u8g2.print(birim);
  u8g2.setCursor(ekran_genislik/1.6 + 3, 6);
  u8g2.print("t = ");
  u8g2.print(anlik_zaman/1000);
  u8g2.print("s");
  u8g2.sendBuffer(); //ekrana çizdirme
}

int8_t DHT_veri_al(){
  int8_t i = 0;
  for( int j=0; j<8; j++){
    while( !( digitalRead(DHT_PIN) ) );
    delayMicroseconds(50);
    if( !( digitalRead(DHT_PIN) ) ){
      i &= ~( 1<<(7-j) ); //ilgili bite 0 yazılır
    }
    else{
      i |= ( 1<<(7-j) ); //ilgili bite 1 yazılır
    }
    while( digitalRead(DHT_PIN) ); //pin LOW olana kadar beklenir
  }
  return i;
}

void DHT_oku(){
  //sensörü başlatmak için önce data LOW ile başla sinyali gönderilir
  pinMode(DHT_PIN, OUTPUT);
  digitalWrite(DHT_PIN, LOW);
  delay(18);
  //DHT11 başla sinyalinden sonra 80us LOW ardından 80us HIGH olur
  pinMode(DHT_PIN, INPUT);
  int8_t res = 0;
  delayMicroseconds(40);
    if( !( digitalRead(DHT_PIN) ) ){ //Önce LOW olduğu kontrol edilir
      delayMicroseconds(80);
      if( digitalRead(DHT_PIN) ){ //80us sonra da HIGH
        res = 1;
      } else {
        res = -1;
      }
  }
  while(digitalRead(DHT_PIN));
  if (res == 1) { // Sensör yanıt verdiyse
    rh_byte1 = DHT_veri_al();
    rh_byte2 = DHT_veri_al();
    temp_byte1 = DHT_veri_al();
    temp_byte2 = DHT_veri_al();
    checksum = DHT_veri_al();

    // Checksum doğrulaması
    if (checksum == (rh_byte1 + rh_byte2 + temp_byte1 + temp_byte2)) {
      // Sıcaklık ve nem hesaplama
      hum = (float)rh_byte1; // DHT11 de nem tam sayı olarak döner
      temp = (float)temp_byte1; // DHT11 de sıcaklık tam sayı olarak döner
    }
  } 
}

void sicaklik_nem_goster(){
  u8g2.clearBuffer();
  u8g2.drawFrame(0,0,128,32);
  u8g2.setFont( u8g2_font_squeezed_b7_tr);
  u8g2.setCursor(5,20);
  u8g2.print("SICAKLIK = ");
  u8g2.print(temp);
  u8g2.print(" °C");
  u8g2.setCursor(5,52);
  u8g2.print("NEM MIKTARI = ");
  u8g2.drawFrame(0,32,128,32);
  u8g2.print(hum);
  u8g2.print(" %");
  u8g2.sendBuffer(); 
}
