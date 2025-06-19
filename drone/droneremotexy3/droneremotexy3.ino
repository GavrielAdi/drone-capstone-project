//////////////////////////////////////////////
//        RemoteXY include library        //
//////////////////////////////////////////////

// you can enable debug logging to Serial at 115200
//#define REMOTEXY__DEBUGLOG    

// RemoteXY select connection mode and include library 
#define REMOTEXY_MODE__WIFI_POINT

#include <WiFi.h>

// RemoteXY connection settings 
#define REMOTEXY_WIFI_SSID "RemoteXY"
#define REMOTEXY_WIFI_PASSWORD "12345678"
#define REMOTEXY_SERVER_PORT 6377


#include <RemoteXY.h>

// RemoteXY GUI configuration  
#pragma pack(push, 1)  
uint8_t RemoteXY_CONF[] =   // 63 bytes
  { 255,6,0,0,0,56,0,19,0,0,0,0,31,1,200,84,1,1,4,0,
    5,39,41,30,30,32,2,26,31,5,153,42,30,30,32,2,26,31,2,28,
    14,10,5,0,2,26,31,31,79,78,0,79,70,70,0,4,12,36,14,36,
    0,2,26 };
    
// this structure defines all the variables and events of your control interface
struct {

  // input variables
  int8_t joystick_01_x; // from -100 to 100
  int8_t joystick_01_y; // from -100 to 100
  int8_t joystick_02_x; // from -100 to 100
  int8_t joystick_02_y; // from -100 to 100
  uint8_t switch_01; // =1 if switch ON and =0 if OFF
  int8_t slider_01; // from 0 to 100

    // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0

} RemoteXY;    
#pragma pack(pop)
/////////////////////////////////////////////
//         AKHIR BAGIAN RemoteXY         //
/////////////////////////////////////////////


#include <Wire.h>
#include <ESP32Servo.h> 

// ================================================================================= //
//         VARIABEL DAN KONSTANTA DRONE (TIDAK DIUBAH)                             //
// ================================================================================= //

volatile float RatePitch, RateRoll, RateYaw;
float RateCalibrationPitch, RateCalibrationRoll, RateCalibrationYaw, AccXCalibration, AccYCalibration, AccZCalibration;

int16_t GyroX;
int16_t GyroY;
int16_t GyroZ;

int ESCfreq = 500;
float PAngleRoll = 2; float PAnglePitch = PAngleRoll;
float IAngleRoll = 0.5; float IAnglePitch = IAngleRoll;
float DAngleRoll = 0.007; float DAnglePitch = DAngleRoll;

float PRateRoll = 0.625;
float IRateRoll = 2.1;
float DRateRoll = 0.0088;

float PRatePitch = PRateRoll;
float IRatePitch = IRateRoll;
float DRatePitch = DRateRoll;

float PRateYaw = 4;
float IRateYaw = 3;
float DRateYaw = 0;

uint32_t LoopTimer;
float t = 0.004;       // time cycle (4000 microseconds or 4ms)

Servo mot1;
Servo mot2;
Servo mot3;
Servo mot4;

const int mot1_pin = 4;
const int mot2_pin = 5;
const int mot3_pin = 18; 
const int mot4_pin = 19;

volatile float PtermRoll;
volatile float ItermRoll;
volatile float DtermRoll;
volatile float PIDOutputRoll;
volatile float PtermPitch;
volatile float ItermPitch;
volatile float DtermPitch;
volatile float PIDOutputPitch;
volatile float PtermYaw;
volatile float ItermYaw;
volatile float DtermYaw;
volatile float PIDOutputYaw;

int ThrottleIdle = 1170; // Minimum throttle value when armed (to keep motors spinning)
int ThrottleCutOff = 1000; // Minimum throttle value when disarmed (motors off)

volatile float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
volatile float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
volatile float InputRoll, InputThrottle, InputPitch, InputYaw;
volatile float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
volatile float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
// float PIDReturn[] = {0, 0, 0}; // This variable is declared but not used, can be removed.

volatile float AccX, AccY, AccZ;
volatile float AngleRoll, AnglePitch;
volatile float DesiredAngleRoll, DesiredAnglePitch;
volatile float ErrorAngleRoll, ErrorAnglePitch;
volatile float PrevErrorAngleRoll, PrevErrorAnglePitch;
volatile float PrevItermAngleRoll, PrevItermAnglePitch;

float complementaryAngleRoll = 0.0f;
float complementaryAnglePitch = 0.0f;

volatile float MotorInput1, MotorInput2, MotorInput3, MotorInput4;

// Variabel untuk timer Serial Monitor
uint32_t printTimer = 0;
const uint16_t PRINT_INTERVAL_MS = 250; // Cetak status setiap 250ms


// Function to read raw gyro and accelerometer signals from MPU6050
void gyro_signals(void)
{
  // Set accelerometer data rate and range
  Wire.beginTransmission(0x68); // MPU6050 address
  Wire.write(0x1A); // CONFIG register
  Wire.write(0x03); // Set DLPF to 44Hz (smoother data)
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C); // ACCEL_CONFIG register
  Wire.write(0x10); // Set accelerometer range to +/- 8G (LSB sensitivity = 4096 LSB/G)
  Wire.endTransmission();
  
  // Read accelerometer data
  Wire.beginTransmission(0x68);
  Wire.write(0x3B); // Start reading from ACCEL_XOUT_H
  Wire.endTransmission(); 
  Wire.requestFrom(0x68,6); // Request 6 bytes (AccX, AccY, AccZ)
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();

  // Set gyroscope data rate and range
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); // GYRO_CONFIG register 
  Wire.write(0x8); // Set gyroscope range to +/- 500 deg/s (LSB sensitivity = 65.5 LSB/(deg/s))
  Wire.endTransmission();                                       
  
  // Read gyroscope data
  Wire.beginTransmission(0x68);
  Wire.write(0x43); // Start reading from GYRO_XOUT_H
  Wire.endTransmission();
  Wire.requestFrom(0x68,6); // Request 6 bytes (GyroX, GyroY, GyroZ)
  GyroX = Wire.read()<<8 | Wire.read();
  GyroY = Wire.read()<<8 | Wire.read();
  GyroZ = Wire.read()<<8 | Wire.read();

  // Convert raw gyro readings to degrees/second
  RateRoll = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw = (float)GyroZ / 65.5;

  // Convert raw accelerometer readings to G's
  AccX = (float)AccXLSB / 4096;
  AccY = (float)AccYLSB / 4096;
  AccZ = (float)AccZLSB / 4096;
  
  // Calculate raw angles from accelerometer (less reliable for dynamic motion)
  // These are superseded by the complementary filter later
  AngleRoll = atan2(AccY, AccZ) * 57.2957795; // Using atan2 for better accuracy across all quadrants
  AnglePitch = atan2(-AccX, sqrt(AccY * AccY + AccZ * AccZ)) * 57.2957795; // Using atan2
}

// ======================================================================= //
//                       FUNGSI KALIBRASI SENSOR BARU                      //
// ======================================================================= //
// This function performs an automatic calibration of the MPU6050.
// It takes multiple samples of gyro and accelerometer data while the drone is assumed to be still,
// then calculates the average offsets to be subtracted from subsequent readings.
void calibrateSensors() {
  Serial.println("Kalibrasi sensor dimulai... Pastikan drone dalam posisi diam.");
  digitalWrite(15, HIGH); // Nyalakan LED (pin 15) selama kalibrasi

  float tempRateCalibrationRoll = 0, tempRateCalibrationPitch = 0, tempRateCalibrationYaw = 0;
  float tempAccXCalibration = 0, tempAccYCalibration = 0, tempAccZCalibration = 0;
  int numSamples = 2000; // Jumlah sampel untuk kalibrasi (2000 samples for ~2 seconds at 1kHz loop)

  for (int i = 0; i < numSamples; i++) {
    gyro_signals(); // Ambil data mentah dari MPU6050
    tempRateCalibrationRoll += RateRoll;
    tempRateCalibrationPitch += RatePitch;
    tempRateCalibrationYaw += RateYaw;
    tempAccXCalibration += AccX;
    tempAccYCalibration += AccY;
    tempAccZCalibration += AccZ;
    delay(1); // Tunggu sebentar antar sampel (minimal 1ms for stable I2C reads)
  }

  // Calculate average offsets
  RateCalibrationRoll = tempRateCalibrationRoll / numSamples;
  RateCalibrationPitch = tempRateCalibrationPitch / numSamples;
  RateCalibrationYaw = tempRateCalibrationYaw / numSamples;
  AccXCalibration = tempAccXCalibration / numSamples;
  AccYCalibration = tempAccYCalibration / numSamples;
  AccZCalibration = tempAccZCalibration / numSamples;

  digitalWrite(15, LOW); // Matikan LED setelah kalibrasi selesai
  Serial.println("Kalibrasi selesai.");
  Serial.print("Kalibrasi RateRoll: "); Serial.println(RateCalibrationRoll, 3);
  Serial.print("Kalibrasi RatePitch: "); Serial.println(RateCalibrationPitch, 3);
  Serial.print("Kalibrasi RateYaw: "); Serial.println(RateCalibrationYaw, 3);
  Serial.print("Kalibrasi AccX: "); Serial.println(AccXCalibration, 3);
  Serial.print("Kalibrasi AccY: "); Serial.println(AccYCalibration, 3);
  Serial.print("Kalibrasi AccZ: "); Serial.println(AccZCalibration, 3);
}

// ======================================================================= //
//                       FUNGSI KALIBRASI ESC BARU                         //
// ======================================================================= //
// This function helps in calibrating the Electronic Speed Controllers (ESCs).
// IMPORTANT: Read the instructions below carefully before running this!
void calibrateESCs() {
  Serial.println("Memulai kalibrasi ESC...");
  Serial.println("1. Pastikan baterai drone TIDAK terhubung.");
  Serial.println("2. Unggah kode ini ke ESP32.");
  Serial.println("3. Setelah \"Memulai kalibrasi ESC...\" muncul di Serial Monitor:");
  Serial.println("4. Hubungkan baterai ke drone. Anda akan mendengar serangkaian bunyi beep.");
  Serial.println("5. Setelah bunyi beep berhenti (menandakan nilai throttle maksimum telah dipelajari),");
  Serial.println("   motor akan berputar pada kecepatan maksimum (2000us)."); // Instructions are still 2000 for user guidance
  Serial.println("   Tunggu sekitar 2 detik untuk memastikan ESC merekam nilai ini.");
  Serial.println("6. Anda akan mendengar bunyi beep lagi (menandakan nilai throttle minimum telah dipelajari).");
  Serial.println("7. Kalibrasi ESC selesai.");
  Serial.println("8. Lepaskan koneksi baterai dari drone.");
  Serial.println("9. Matikan/hapus baris pemanggilan calibrateESCs() dari setup() dan unggah ulang kode.");
  Serial.println("Motor akan disetel ke maksimum (2000us) selama 5 detik, lalu minimum (1000us) selama 5 detik.");

  digitalWrite(15, HIGH); // Nyalakan LED selama kalibrasi ESC
  
  // Set all motors to maximum throttle for calibration
  mot1.writeMicroseconds(2000); // Send 2000us during calibration
  mot2.writeMicroseconds(2000);
  mot3.writeMicroseconds(2000);
  mot4.writeMicroseconds(2000);
  delay(5000); // Hold at max throttle for 5 seconds

  // Set all motors to minimum throttle for calibration
  mot1.writeMicroseconds(1000);
  mot2.writeMicroseconds(1000);
  mot3.writeMicroseconds(1000);
  mot4.writeMicroseconds(1000);
  delay(5000); // Hold at min throttle for 5 seconds

  digitalWrite(15, LOW); // Matikan LED setelah kalibrasi ESC selesai
  Serial.println("Kalibrasi ESC selesai. Harap ikuti langkah 8 dan 9 di atas.");
}


void setup(void) {
  
  Serial.begin(115200); // Initialize Serial communication
  RemoteXY_Init();      // Initialize RemoteXY library

  // Fast LED blinks to indicate system boot-up
  int led_time = 100;
  pinMode(15, OUTPUT);
  for (int i = 0; i < 5; i++) {
    digitalWrite(15, LOW); delay(led_time);
    digitalWrite(15, HIGH); delay(led_time);
  }
  digitalWrite(15, LOW); // Ensure LED is off after boot blinks

  // Initialize I2C (Wire) communication for MPU6050
  Wire.setClock(400000); // Set I2C clock speed to 400kHz
  Wire.begin();
  delay(250); // Give MPU some time to power up
  Wire.beginTransmission(0x68); // MPU6050 address
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0x00); // Set to zero (wakes up the MPU-6050)
  Wire.endTransmission();

  // Call the new calibration routine for sensors
  calibrateSensors(); 

  // Allocate PWM timers for ESP32 servos
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  delay(1000); // Wait for MPU and system to stabilize after power-up and calibration

  // Attach ESCs to PWM pins and set their operating frequency
  // *** PERUBAHAN DIAGNOSTIK: Mengatur rentang PWM maksimum ke 1950 microdetik ***
  mot1.attach(mot1_pin,1000,1950); mot1.setPeriodHertz(ESCfreq); delay(100);
  mot2.attach(mot2_pin,1000,1950); mot2.setPeriodHertz(ESCfreq); delay(100);
  mot3.attach(mot3_pin,1000,1950); mot3.setPeriodHertz(ESCfreq); delay(100);
  mot4.attach(mot4_pin,1000,1950); mot4.setPeriodHertz(ESCfreq); delay(100);

  // Set motors to ThrottleCutOff (1000 microseconds) at startup to ensure they are off
  mot1.writeMicroseconds(ThrottleCutOff);
  mot2.writeMicroseconds(ThrottleCutOff);
  mot3.writeMicroseconds(ThrottleCutOff);
  mot4.writeMicroseconds(ThrottleCutOff);
  delay(500); // Small delay to ensure commands are processed

  // UNCOMMENT THE LINE BELOW TO CALIBRATE YOUR ESCs.
  // READ THE INSTRUCTIONS IN THE calibrateESCs() FUNCTION CAREFULLY.
  // calibrateESCs(); // Call this function ONLY ONCE for ESC calibration.
                     // Remember to comment it out after successful calibration and re-upload the code.


  digitalWrite(15, HIGH);
  delay(500);
  digitalWrite(15, LOW);
  delay(500);

  LoopTimer = micros(); // Initialize the loop timer
}


void loop(void) {

  RemoteXY_Handler(); // Handle communication with RemoteXY app
  gyro_signals();     // Read raw sensor data

  // Subtract calibrated offsets from raw sensor readings
  RateRoll -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw -= RateCalibrationYaw;
  AccX -= AccXCalibration;
  AccY -= AccYCalibration;
  AccZ -= AccZCalibration;

  // Complementary filter to estimate Roll and Pitch angles.
  // Combines fast-reacting gyro data with stable (but noisy) accelerometer data.
  // 0.991 (gyro part) + 0.009 (accel part) = 1 (sum of weights)
  complementaryAngleRoll = 0.991 * (complementaryAngleRoll + RateRoll * t) + 0.009 * AngleRoll;
  complementaryAnglePitch = 0.991 * (complementaryAnglePitch + RatePitch * t) + 0.009 * AnglePitch;
  
  // Limiting complementary angles to prevent extreme values from distorting PID
  // These limits prevent 'flipping' issues if the drone is significantly tilted.
  complementaryAngleRoll = (complementaryAngleRoll > 30) ? 30 : ((complementaryAngleRoll < -30) ? -30 : complementaryAngleRoll); 
  complementaryAnglePitch = (complementaryAnglePitch > 30) ? 30 : ((complementaryAnglePitch < -30) ? -30 : complementaryAnglePitch); 

  // ======================================================================= //
  //         MENGAMBIL INPUT DARI RemoteXY KE SETPOINT                       //
  // ======================================================================= //
  
  // Map joystick_01_x (Roll) to desired angle (e.g., -15 to +15 degrees)
  // Input joystick (-100 to 100) multiplied by 0.15 -> Output -15 to 15 degrees.
  DesiredAngleRoll = 0.15 * RemoteXY.joystick_01_x;
  // Map joystick_01_y (Pitch) to desired angle (e.g., -15 to +15 degrees)
  // Input joystick (-100 to 100) multiplied by 0.15 -> Output -15 to 15 degrees.
  DesiredAnglePitch = 0.15 * RemoteXY.joystick_01_y;

  // Map slider_01 (Throttle) to motor pulse width (e.g., 1000 to 1950 microseconds)
  // *** PERUBAHAN DIAGNOSTIK: Mengatur rentang throttle maksimum ke 1950 microdetik ***
  InputThrottle = map(RemoteXY.slider_01, 0, 100, ThrottleCutOff, 1950); 
  
  // Map joystick_02_x (Yaw) to desired yaw rate (e.g., -90 to +90 degrees/second)
  // Input joystick (-100 to 100) multiplied by 0.9 -> Output -90 to 90 deg/s.
  DesiredRateYaw = 0.9 * RemoteXY.joystick_02_x;

  // ======================================================================= //
  //         LOGIKA STABILISASI & PID                                        //
  // ======================================================================= //
  
  // PID Level/Angle Mode (Outer loop - controls desired rate based on angle error)
  // Roll Axis Angle PID
  ErrorAngleRoll = DesiredAngleRoll - complementaryAngleRoll;
  PtermRoll = PAngleRoll * ErrorAngleRoll;
  ItermRoll = PrevItermAngleRoll + (IAngleRoll * (ErrorAngleRoll + PrevErrorAngleRoll) * (t / 2));
  ItermRoll = (ItermRoll > 400) ? 400 : ((ItermRoll < -400) ? -400 : ItermRoll); // Anti-windup for integrator
  DtermRoll = DAngleRoll * ((ErrorAngleRoll - PrevErrorAngleRoll) / t);
  PIDOutputRoll = PtermRoll + ItermRoll + DtermRoll;
  PIDOutputRoll = (PIDOutputRoll > 400) ? 400 : ((PIDOutputRoll < -400) ? -400 : PIDOutputRoll); // Clamp output
  DesiredRateRoll = PIDOutputRoll; // Output of angle PID is desired rate for inner loop
  PrevErrorAngleRoll = ErrorAngleRoll;
  PrevItermAngleRoll = ItermRoll;

  // Pitch Axis Angle PID
  ErrorAnglePitch = DesiredAnglePitch - complementaryAnglePitch;
  PtermPitch = PAnglePitch * ErrorAnglePitch;
  ItermPitch = PrevItermAnglePitch + (IAnglePitch * (ErrorAnglePitch + PrevErrorAnglePitch) * (t / 2));
  ItermPitch = (ItermPitch > 400) ? 400 : ((ItermPitch < -400) ? -400 : ItermPitch); // Anti-windup
  DtermPitch = DAngleRoll * ((ErrorAnglePitch - PrevErrorAnglePitch) / t);
  PIDOutputPitch = PtermPitch + ItermPitch + DtermPitch;
  PIDOutputPitch = (PIDOutputPitch > 400) ? 400 : ((PIDOutputPitch < -400) ? -400 : PIDOutputPitch); // Clamp output
  DesiredRatePitch = PIDOutputPitch; // Output of angle PID is desired rate for inner loop
  PrevErrorAnglePitch = ErrorAnglePitch;
  PrevItermAnglePitch = ItermPitch;

  // PID Rate Mode (Inner loop - controls motor output based on rate error)
  ErrorRateRoll = DesiredRateRoll - RateRoll;
  ErrorRatePitch = DesiredRatePitch - RatePitch;
  ErrorRateYaw = DesiredRateYaw - RateYaw;

  // Roll Axis Rate PID
  PtermRoll = PRateRoll * ErrorRateRoll;
  ItermRoll = PrevItermRateRoll + (IRateRoll * (ErrorRateRoll + PrevErrorRateRoll) * (t / 2));
  ItermRoll = (ItermRoll > 400) ? 400 : ((ItermRoll < -400) ? -400 : ItermRoll); // Anti-windup
  DtermRoll = DRateRoll * ((ErrorRateRoll - PrevErrorRateRoll) / t);
  PIDOutputRoll = PtermRoll + ItermRoll + DtermRoll;
  PIDOutputRoll = (PIDOutputRoll > 400) ? 400 : ((PIDOutputRoll < -400) ? -400 : PIDOutputRoll); // Clamp output
  InputRoll = PIDOutputRoll; // Final PID output for Roll
  PrevErrorRateRoll = ErrorRateRoll;
  PrevItermRateRoll = ItermRoll;

  // Pitch Axis Rate PID
  PtermPitch = PRatePitch * ErrorRatePitch;
  ItermPitch = PrevItermRatePitch + (IRatePitch * (ErrorRatePitch + PrevErrorRatePitch) * (t / 2));
  ItermPitch = (ItermPitch > 400) ? 400 : ((ItermPitch < -400) ? -400 : ItermPitch); // Anti-windup
  DtermPitch = DRatePitch * ((ErrorRatePitch - PrevErrorRatePitch) / t);
  PIDOutputPitch = PtermPitch + ItermPitch + DtermPitch;
  PIDOutputPitch = (PIDOutputPitch > 400) ? 400 : ((PIDOutputPitch < -400) ? -400 : PIDOutputPitch); // Clamp output
  InputPitch = PIDOutputPitch; // Final PID output for Pitch
  PrevErrorRatePitch = ErrorRatePitch;
  PrevItermRatePitch = ItermPitch;

  // Yaw Axis Rate PID
  PtermYaw = PRateYaw * ErrorRateYaw;
  ItermYaw = PrevItermRateYaw + (IRateYaw * (ErrorRateYaw + PrevErrorRateYaw) * (t / 2));
  ItermYaw = (ItermYaw > 400) ? 400 : ((ItermYaw < -400) ? -400 : ItermYaw); // Anti-windup
  DtermYaw = DRateYaw * ((ErrorRateYaw - PrevErrorRateYaw) / t);
  PIDOutputYaw = PtermYaw + ItermYaw + DtermYaw;
  PIDOutputYaw = (PIDOutputYaw > 400) ? 400 : ((PIDOutputYaw < -400) ? -400 : PIDOutputYaw); // Clamp output
  InputYaw = PIDOutputYaw; // Final PID output for Yaw
  PrevErrorRateYaw = ErrorRateYaw;
  PrevItermRateYaw = ItermYaw;

  // ======================================================================= //
  //         KONTROL MOTOR DAN LOGIKA ARMING                                 //
  // ======================================================================= //
  
  // Ensure InputThrottle doesn't exceed 1950 (adjusted max for ESCs) after mapping
  // *** PERUBAHAN DIAGNOSTIK: Mengatur InputThrottle maksimum ke 1950 microdetik ***
  if (InputThrottle > 1950) InputThrottle = 1950;
  
  // Motor Mixing: Calculate individual motor speeds based on throttle and PID corrections
  // This configuration is for a Quadcopter X-mode setup
  MotorInput1 = (InputThrottle - InputRoll - InputPitch - InputYaw); // Front Right
  MotorInput2 = (InputThrottle - InputRoll + InputPitch + InputYaw); // Rear Right
  MotorInput3 = (InputThrottle + InputRoll + InputPitch - InputYaw); // Rear Left
  MotorInput4 = (InputThrottle + InputRoll - InputPitch + InputYaw); // Front Left

  // Clamp motor outputs to the valid ESC range (1000-1950 microseconds)
  // These clamps are applied AFTER PID mixing
  // *** PERUBAHAN DIAGNOSTIK: Mengatur MotorInput maksimum ke 1950 microdetik ***
  if (MotorInput1 > 1950) MotorInput1 = 1950;
  if (MotorInput2 > 1950) MotorInput2 = 1950;
  if (MotorInput3 > 1950) MotorInput3 = 1950;
  if (MotorInput4 > 1950) MotorInput4 = 1950;

  // Set minimum motor speed when armed, to prevent motors from stopping completely.
  // This is crucial for preventing motors from "dying" in flight due to PID overcorrections.
  if (MotorInput1 < ThrottleIdle) MotorInput1 = ThrottleIdle;
  if (MotorInput2 < ThrottleIdle) MotorInput2 = ThrottleIdle;
  if (MotorInput3 < ThrottleIdle) MotorInput3 = ThrottleIdle;
  if (MotorInput4 < ThrottleIdle) MotorInput4 = ThrottleIdle;

  // ARMING LOGIC: Using switch_01 from RemoteXY application
  if (RemoteXY.switch_01 == 0) // If RemoteXY switch_01 is OFF (drone is disarmed)
  {
    // Set all motors to ThrottleCutOff (1000 microseconds) to ensure they are off
    MotorInput1 = ThrottleCutOff;
    MotorInput2 = ThrottleCutOff;
    MotorInput3 = ThrottleCutOff;
    MotorInput4 = ThrottleCutOff;
    
    // Reset all PID integrators and previous errors when disarmed.
    // This prevents "wind-up" (accumulated error) and sudden motor acceleration on re-arming.
    PrevErrorRateRoll=0; PrevErrorRatePitch=0; PrevErrorRateYaw=0;
    PrevItermRateRoll=0; PrevItermRatePitch=0; PrevItermRateYaw=0;
    PrevErrorAngleRoll=0; PrevErrorAnglePitch=0;    
    PrevItermAngleRoll=0; PrevItermAnglePitch=0;
    
    // Also reset complementary filter angle estimates to zero when disarmed
    // This ensures a clean start when re-armed.
    complementaryAngleRoll = 0.0f; 
    complementaryAnglePitch = 0.0f;
  }
  // When armed (RemoteXY.switch_01 == 1), motor inputs are applied as calculated above.

  // Write calculated motor inputs to the ESCs
  mot1.writeMicroseconds(MotorInput1);
  mot2.writeMicroseconds(MotorInput2);
  mot3.writeMicroseconds(MotorInput3);
  mot4.writeMicroseconds(MotorInput4);

  // ======================================================================= //
  //         MENCETAK STATUS KE SERIAL MONITOR (UNTUK DEBUGGING)             //
  // ======================================================================= //
  // Print status to Serial Monitor at a defined interval (e.g., every 250ms)
  if (millis() - printTimer >= PRINT_INTERVAL_MS) {
    printTimer = millis();
    
    Serial.print("ARM: ");
    Serial.print((RemoteXY.switch_01 == 1) ? "ON " : "OFF"); // Print arming status
    
    Serial.print(" | Thr: ");
    Serial.print(InputThrottle); // Print current throttle input
    
    Serial.print(" | Desired Rol: ");
    Serial.print(DesiredAngleRoll, 2); // Print desired Roll angle
    Serial.print(" | Actual Rol: ");
    Serial.print(complementaryAngleRoll, 2); // Print actual (estimated) Roll angle
    
    Serial.print(" | Desired Pit: ");
    Serial.print(DesiredAnglePitch, 2); // Print desired Pitch angle
    Serial.print(" | Actual Pit: ");
    Serial.print(complementaryAnglePitch, 2); // Print actual (estimated) Pitch angle
    
    Serial.print(" | Desired Yaw Rate: ");
    Serial.print(DesiredRateYaw, 2); // Print desired Yaw rate
    Serial.print(" | Actual Yaw Rate: ");
    Serial.print(RateYaw, 2); // Print actual Yaw rate

    Serial.print(" | M1:"); Serial.print(MotorInput1); // Print individual motor outputs
    Serial.print(" M2:"); Serial.print(MotorInput2);
    Serial.print(" M3:"); Serial.print(MotorInput3);
    Serial.print(" M4:"); Serial.println(MotorInput4);
  }

  // Menjaga frekuensi loop agar tetap konstan (misalnya 250Hz for 4ms loop time)
  // This 'while' loop waits until the target loop time 't' has passed.
  while (micros() - LoopTimer < (t * 1000000)) {
    // Do nothing, just wait
  }
  LoopTimer += (t * 1000000); // Update LoopTimer for the next cycle, ensuring cumulative accuracy
}
