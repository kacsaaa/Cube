#include <Wire.h>

//Globális változók deklaráslása
int gyro_x, gyro_y, gyro_z;
float acc_x, acc_y, acc_z, acc_total_vector;
int temperature;
float gyro_x_offset, gyro_y_offset, gyro_z_offset;
long loop_timer;
float angle_x, angle_y;
float angle_x_acc, angle_z_acc;
int subtimer;
#define offset_timer 1000
#define frequency 200.0f


void setup() {
	Wire.begin();															//I2C kommunikáció megkezdése
	Serial.begin(57600);

	setup_mpu_6050_registers();												//MPU-6050 regisztereinek beállítása: 65.5 <-- 1°/s; 8 <-- 1/g
	gyro_x_offset = 0;
	gyro_z_offset = 0;
	subtimer = 0;

	for (int i = 0; i < offset_timer; i++) {								//Offset mérése, majd egy erre használt változóba mentése
		read_data_mpu_6050();                                            
		gyro_x_offset += gyro_x;                                           
		gyro_z_offset += gyro_z;
	}
	gyro_x_offset /= offset_timer;											//Osztom az összeadott értéket, hogy a tényleges offset-et kapjam meg                                                  
	gyro_z_offset /= offset_timer;
	acc_total_vector = sqrt((acc_x*acc_x) + (acc_y*acc_y) + (acc_z*acc_z));
	angle_x_acc = asin((float)acc_y / acc_total_vector)* RAD_TO_DEG;
	angle_x = angle_x_acc;													//Kiindulási értékek beállítása
	
	loop_timer = micros();													//Innentõl mérem egy ciklus idejét, hogy az integrálás megfelelõ legyen
}

void loop() {

	read_data_mpu_6050();													//Kiolvasom az aktuális adatokat a szenzorból -->1.7ms lemérve!

	gyro_x -= gyro_x_offset;												//Offset-tel kalibrálok                                         
	gyro_z -= gyro_z_offset;                                               
	
	angle_x -= gyro_x/frequency/65.5f;										//Hogy fokot kapjak 65.5-tel kell elosztanom a kiolvasott értéket, illetve még 250-nel,
																			//hogy a két kiolvasás közti szögelfordulást kapjam meg.
	acc_total_vector = sqrt((acc_x*acc_x) + (acc_y*acc_y) + (acc_z*acc_z));
	angle_x_acc = asin((float)acc_y / acc_total_vector)* RAD_TO_DEG;
	angle_x -= angle_y * sin((gyro_z/frequency/65.5f)*DEG_TO_RAD);			//A z irányú elfordulás következtében létrejövõ hibát kalibrálom
	angle_y += angle_x * sin((gyro_z/frequency/65.5f)*DEG_TO_RAD);
																
	angle_x = angle_x * 0.9991 + angle_x_acc * 0.0009;						//Giroszkóp drift-jének kompenzálása agyorsulásmérõ és giroszkóp súlyozott összegével
	
	Serial.print(angle_x);
	//Serial.print("|Angle_X_acc = "); Serial.print(angle_x_acc);
	//Serial.print("|Angle_X_gyr = "); Serial.print(gyro_x / 100.0f / 65.5f);
	//Serial.print(micros() - loop_timer);
	Serial.println();

	while (micros() - loop_timer < 1/frequency);								//Addig vár míg a számláló eléri az 5000-es (200Hz-hez tartozó) értéket
	loop_timer = micros();													//Resetelem a timer-t, hogy 0-ról induljon a különbség az új ciklusban
}

void read_data_mpu_6050() {													//Nyers adatok kiolvasására alkalmas függvény

	Wire.beginTransmission(0x68);											//Kommunikáció megkezdése a szenzorral (0x68 a címe)
	Wire.write(0x3B);														//A kezdõcímét az elsõ regiszternek kiküldöm
	Wire.endTransmission();													//Befejezem a kommunikációt

	Wire.requestFrom(0x68, 14);												//14 byte-nyi információt (7 regiszter) kérek
	while (Wire.available() < 14);											//Várok, míg mind a 14 byte megérkezik
	acc_x = Wire.read() << 8 | Wire.read();									//Az acc_x változóba töltöm az adott regiszter byte-jait (low és high)
	acc_y = Wire.read() << 8 | Wire.read();
	acc_z = Wire.read() << 8 | Wire.read();
	temperature = Wire.read() << 8 | Wire.read();                           
	gyro_x = Wire.read() << 8 | Wire.read();
	gyro_y = Wire.read() << 8 | Wire.read();
	gyro_z = Wire.read() << 8 | Wire.read();

}

void setup_mpu_6050_registers() {											//A szenzor regisztereinek beállítására alkalmas függvény
																			//MPU-6050 felkeltése, ezt minden tápraadáskor meg kell tenni
	Wire.beginTransmission(0x68);											//Kommunikáció megkezdése
	Wire.write(0x6B);														//Elküldöm a regiszter címét, mellyel dolgozni kívánok
	Wire.write(0x00);														//Beállítítom
	Wire.endTransmission();													//Befejezem a kommunikációt
																			//Beállítom a gyorsulásmérõt, hogy a mérési tartomány +/-8g legyen
	Wire.beginTransmission(0x68);											//Kommunikáció megkezdése
	Wire.write(0x1C);														//Elküldöm a regiszter címét, mellyel dolgozni kívánok
	Wire.write(0x10);														//Regiszter beállítása
	Wire.endTransmission();													//Befejezem a kommunikációt
																			//Beállítom a giroszkóp 500dps full scale módját
	Wire.beginTransmission(0x68);											//Kommunikáció megkezdése
	Wire.write(0x1B);														//Elküldöm a regiszter címét, mellyel dolgozni kívánok
	Wire.write(0x08);														//Regiszter beállítása
	Wire.endTransmission();													//Befejezem a kommunikációt
}