#include <Wire.h>

//Glob�lis v�ltoz�k deklar�sl�sa
int gyro_x, gyro_y, gyro_z;
float acc_x, acc_y, acc_z, acc_total_vector;
int temperature;
float gyro_x_offset, gyro_y_offset, gyro_z_offset;
long loop_timer;
float angle_x, angle_z;
float angle_x_acc;
#define offset_timer 1000


void setup() {
	Wire.begin();															//I2C kommunik�ci� megkezd�se
	Serial.begin(57600);

	setup_mpu_6050_registers();												//MPU-6050 regisztereinek be�ll�t�sa: 65.5 <-- 1�/s; 8 <-- 1/g
	gyro_x_offset = 0;
	gyro_z_offset = 0;

	for (int i = 0; i < offset_timer; i++) {								//Offset m�r�se, majd egy erre haszn�lt v�ltoz�ba ment�se
		read_data_mpu_6050();                                            
		gyro_x_offset += gyro_x;                                           
		gyro_z_offset += gyro_z;                                            
	}
	gyro_x_offset /= offset_timer;											//Osztom az �sszeadott �rt�ket, hogy a t�nyleges offset-et kapjam meg                                                  
	gyro_z_offset /= offset_timer;
	acc_total_vector = sqrt((acc_x*acc_x) + (acc_y*acc_y) + (acc_z*acc_z));
	angle_x_acc = asin((float)acc_y / acc_total_vector)* RAD_TO_DEG;
	angle_x = angle_x_acc;													//Kiindul�si �rt�kek be�ll�t�sa
	
	loop_timer = micros();													//Innent�l m�rem egy ciklus idej�t, hogy az integr�l�s megfelel� legyen
}

void loop() {

	read_data_mpu_6050();													//Kiolvasom az aktu�lis adatokat a szenzorb�l

	gyro_x -= gyro_x_offset;												//Offset-tel kalibr�lok                                         
	gyro_z -= gyro_z_offset;                                               

	angle_x -= gyro_x/100.0f/65.5f;		//minuszra v�ltottam a pluszt!									 //Hogy fokot kapjak 65.5-tel kell elosztanom a kiolvasott �rt�ket, illetve m�g 250-nel,
																			//hogy a k�t kiolvas�s k�zti sz�gelfordul�st kapjam meg.
	acc_total_vector = sqrt((acc_x*acc_x) + (acc_y*acc_y) + (acc_z*acc_z));
	angle_x_acc = asin((float)acc_y / acc_total_vector)* RAD_TO_DEG;
																			
	//angle_x += angle_z * sin((gyro_z/100.0f/65.5f)*DEG_TO_RAD);				//A z ir�ny� elfordul�s k�vetkezt�ben l�trej�v� hib�t kalibr�lom
																		
	//angle_x = angle_x * 0.9991 + angle_x_acc * 0.0009;						//Giroszk�p drift-j�nek kompenz�l�sa agyorsul�sm�r� �s giroszk�p s�lyozott �sszeg�vel

	Serial.print("|Angle_X = "); Serial.print(angle_x);
	Serial.print("|Angle_X_acc = "); Serial.print(angle_x_acc);
	Serial.print("|Angle_X_gyr = "); Serial.print(gyro_x / 100.0f / 65.5f);
	Serial.println();

	while (micros() - loop_timer < 10000);									//Addig v�r m�g a sz�ml�l� el�ri az 1000-es (100Hz-hez tartoz�) �rt�ket
	loop_timer = micros();													//Resetelem a timer-t, hogy 0-r�l induljon a k�l�nbs�g az �j ciklusban
}

void read_data_mpu_6050() {													//Nyers adatok kiolvas�s�ra alkalmas f�ggv�ny

	Wire.beginTransmission(0x68);											//Kommunik�ci� megkezd�se a szenzorral (0x68 a c�me)
	Wire.write(0x3B);														//A kezd�c�m�t az els� regiszternek kik�ld�m
	Wire.endTransmission();													//Befejezem a kommunik�ci�t

	Wire.requestFrom(0x68, 14);												//14 byte-nyi inform�ci�t (7 regiszter) k�rek
	while (Wire.available() < 14);											//V�rok, m�g mind a 14 byte meg�rkezik
	acc_x = Wire.read() << 8 | Wire.read();									//Az acc_x v�ltoz�ba t�lt�m az adott regiszter byte-jait (low �s high)
	acc_y = Wire.read() << 8 | Wire.read();
	acc_z = Wire.read() << 8 | Wire.read();
	temperature = Wire.read() << 8 | Wire.read();                           
	gyro_x = Wire.read() << 8 | Wire.read();
	gyro_y = Wire.read() << 8 | Wire.read();
	gyro_z = Wire.read() << 8 | Wire.read();

}

void setup_mpu_6050_registers() {											//A szenzor regisztereinek be�ll�t�s�ra alkalmas f�ggv�ny
																			//MPU-6050 felkelt�se, ezt minden t�praad�skor meg kell tenni
	Wire.beginTransmission(0x68);											//Kommunik�ci� megkezd�se
	Wire.write(0x6B);														//Elk�ld�m a regiszter c�m�t, mellyel dolgozni k�v�nok
	Wire.write(0x00);														//Be�ll�t�tom
	Wire.endTransmission();													//Befejezem a kommunik�ci�t
																			//Be�ll�tom a gyorsul�sm�r�t, hogy a m�r�si tartom�ny +/-8g legyen
	Wire.beginTransmission(0x68);											//Kommunik�ci� megkezd�se
	Wire.write(0x1C);														//Elk�ld�m a regiszter c�m�t, mellyel dolgozni k�v�nok
	Wire.write(0x10);														//Regiszter be�ll�t�sa
	Wire.endTransmission();													//Befejezem a kommunik�ci�t
																			//Be�ll�tom a giroszk�p 500dps full scale m�dj�t
	Wire.beginTransmission(0x68);											//Kommunik�ci� megkezd�se
	Wire.write(0x1B);														//Elk�ld�m a regiszter c�m�t, mellyel dolgozni k�v�nok
	Wire.write(0x08);														//Regiszter be�ll�t�sa
	Wire.endTransmission();													//Befejezem a kommunik�ci�t
}