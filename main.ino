/*
  ระบบตรวจวัดก๊าซชีวภาพด้วย Arduino
  Biogas Monitoring System with Arduino
  
  เซ็นเซอร์ที่ใช้:
  - MQ-4: ตรวจวัดก๊าซมีเทน (CH4) - Pin A0
  - MQ-135: ตรวจวัดแอมโมเนีย/CO2 - Pin A1  
  - MQ-136: ตรวจวัดไฮโดรเจนซัลไฟด์ (H2S) - Pin A2
  - pH Sensor: วัดความเป็นกรด-ด่าง - Pin A3
  
  LED แสดงสถานะ:
  - Pin D12: LED เขียว (สถานะปกติ)
  - Pin D13: LED แดง (สถานะเตือนภัย)
  
  ผู้พัฒนา: Arduino Biogas Monitor
  เวอร์ชัน: 1.0
  วันที่: 24 กรกฎาคม 2568
*/

#include <SoftwareSerial.h>
#include <EEPROM.h>

// ===========================================
// การกำหนดขา GPIO และค่าคงที่
// ===========================================

// ขา Analog สำหรับเซ็นเซอร์
#define MQ4_PIN A0      // เซ็นเซอร์ก๊าซมีเทน (CH4)
#define MQ135_PIN A1    // เซ็นเซอร์ก๊าซแอมโมเนีย/CO2  
#define MQ136_PIN A2    // เซ็นเซอร์ก๊าซ H2S
#define PH_PIN A3       // เซ็นเซอร์ pH

// ขา Digital สำหรับ LED
#define LED_GREEN_PIN 12  // LED เขียว (สถานะปกติ)
#define LED_RED_PIN 13    // LED แดง (สถานะเตือนภัย)

// ค่าคงที่สำหรับการคำนวณ
#define VOLTAGE_REF 5.0
#define ADC_RESOLUTION 1024.0
#define SERIAL_BAUD 9600

// ค่าคงที่สำหรับการเฉลี่ย
const int NUM_READINGS = 10;
const int READING_DELAY = 100; // มิลลิวินาที
const int MAIN_LOOP_DELAY = 3000; // 3 วินาที

// ระดับเตือนภัย (ppm)
const float METHANE_WARNING_LEVEL = 1000.0;    // 1000 ppm
const float METHANE_DANGER_LEVEL = 5000.0;     // 5000 ppm  
const float AMMONIA_WARNING_LEVEL = 25.0;      // 25 ppm
const float AMMONIA_DANGER_LEVEL = 50.0;       // 50 ppm
const float H2S_WARNING_LEVEL = 10.0;          // 10 ppm
const float H2S_DANGER_LEVEL = 20.0;           // 20 ppm

// ระดับ pH ที่เหมาะสม
const float PH_MIN_OPTIMAL = 6.5;
const float PH_MAX_OPTIMAL = 7.5;
const float PH_MIN_ACCEPTABLE = 6.0;
const float PH_MAX_ACCEPTABLE = 8.0;

// ===========================================
// ตัวแปรสำหรับเก็บค่าอ่าน
// ===========================================

// ค่าดิบจากเซ็นเซอร์
int mq4_raw = 0;
int mq135_raw = 0; 
int mq136_raw = 0;
int ph_raw = 0;

// ค่าแรงดัน
float mq4_voltage = 0.0;
float mq135_voltage = 0.0;
float mq136_voltage = 0.0;
float ph_voltage = 0.0;

// ค่าที่แปลงแล้ว
float methane_ppm = 0.0;
float ammonia_ppm = 0.0;
float h2s_ppm = 0.0;
float ph_value = 0.0;

// สถานะระบบ
bool system_warning = false;
bool system_danger = false;
unsigned long last_reading_time = 0;
unsigned long system_start_time = 0;

// การนับสถิติ
unsigned long total_readings = 0;
float max_methane = 0.0;
float max_ammonia = 0.0;
float max_h2s = 0.0;
float min_ph = 14.0;
float max_ph = 0.0;

// ===========================================
// ฟังก์ชัน Setup
// ===========================================

void setup() {
  // เริ่มต้น Serial Communication
  Serial.begin(SERIAL_BAUD);
  
  // ตั้งค่าขา Digital
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_RED_PIN, OUTPUT);
  
  // ตั้งค่าขา Analog เป็น Input (ไม่จำเป็นแต่ทำเพื่อความชัดเจน)
  pinMode(MQ4_PIN, INPUT);
  pinMode(MQ135_PIN, INPUT);
  pinMode(MQ136_PIN, INPUT);
  pinMode(PH_PIN, INPUT);
  
  // เริ่มต้นระบบ
  system_start_time = millis();
  
  // แสดงข้อความเริ่มต้น
  printWelcomeMessage();
  
  // โหลดค่า Calibration ของ pH (ถ้ามี)
  loadCalibrationData();
  
  // อุ่นเครื่องเซ็นเซอร์
  warmupSensors();
  
  // ทดสอบระบบ LED
  testLEDs();
  
  // แสดงข้อมูลการกำหนดค่า
  printConfiguration();
  
  Serial.println("💡 คำสั่งพิเศษ:");
  Serial.println("   พิมพ์ 'CAL' เพื่อ Calibrate เซ็นเซอร์ pH");
  Serial.println("   พิมพ์ 'RESET' เพื่อรีเซ็ตสถิติ");
  Serial.println("   พิมพ์ 'DIAG' เพื่อตรวจสอบระบบ");
  Serial.println();
  
  Serial.println("✅ ระบบพร้อมใช้งาน!");
  Serial.println("=====================================");
}

// ===========================================
// ฟังก์ชัน Loop หลัก
// ===========================================

void loop() {
  // ตรวจสอบคำสั่งจาก Serial
  checkSerialCommands();
  
  // อ่านค่าจากเซ็นเซอร์ทั้งหมด
  readAllSensors();
  
  // แปลงค่าเป็นหน่วยที่ต้องการ
  convertSensorValues();
  
  // แสดงผลลัพธ์
  displayResults();
  
  // ตรวจสอบระดับความปลอดภัย
  checkSafetyLevels();
  
  // ควบคุม LED
  controlLEDs();
  
  // แสดงข้อแนะนำ
  showRecommendations();
  
  // อัพเดทสถิติ
  updateStatistics();
  
  // แสดงสถิติ (ทุก 10 ครั้ง)
  if(total_readings % 10 == 0) {
    showStatistics();
  }
  
  Serial.println("-------------------------------------");
  
  // หน่วงเวลาก่อนอ่านครั้งถัดไป
  delay(MAIN_LOOP_DELAY);
}

// ===========================================
// ฟังก์ชันแสดงข้อความต้อนรับ
// ===========================================

void printWelcomeMessage() {
  Serial.println();
  Serial.println("=====================================");
  Serial.println("   ระบบตรวจวัดก๊าซชีวภาพ Arduino");
  Serial.println("   Biogas Monitoring System v1.0");
  Serial.println("=====================================");
  Serial.println();
  Serial.println("เซ็นเซอร์ที่ติดตั้ง:");
  Serial.println("🔬 MQ-4   : ก๊าซมีเทน (CH4) - Pin A0");
  Serial.println("🔬 MQ-135 : แอมโมเนีย/CO2 - Pin A1");  
  Serial.println("🔬 MQ-136 : ไฮโดรเจนซัลไฟด์ (H2S) - Pin A2");
  Serial.println("🔬 pH BNC : pH Detection Module with BNC Probe - Pin A3");
  Serial.println();
  Serial.println("LED แสดงสถานะ:");
  Serial.println("💚 LED เขียว : สถานะปกติ - Pin D12");
  Serial.println("❤️ LED แดง   : เตือนภัย - Pin D13");
  Serial.println();
  Serial.println("📋 เซ็นเซอร์ pH: Liquid pH Value Detection Sensor Module");
  Serial.println("   - BNC Electrode Probe");
  Serial.println("   - ช่วงการวัด: pH 0-14");
  Serial.println("   - อุณหภูมิทำงาน: 0-60°C");
  Serial.println("   - ความแม่นยำ: ±0.1 pH");
  Serial.println();
}

// ===========================================
// ฟังก์ชันอุ่นเครื่องเซ็นเซอร์
// ===========================================

void warmupSensors() {
  Serial.println("🔥 กำลังอุ่นเครื่องเซ็นเซอร์ MQ...");
  Serial.println("⏰ กรุณารอ 30 วินาที เพื่อความแม่นยำ");
  Serial.println();
  
  // แสดง Progress Bar
  for(int i = 30; i > 0; i--) {
    // LED เขียวกระพริบระหว่างอุ่นเครื่อง
    digitalWrite(LED_GREEN_PIN, HIGH);
    delay(250);
    digitalWrite(LED_GREEN_PIN, LOW);
    delay(250);
    
    // แสดงเวลาที่เหลือ
    Serial.print("⏱️  เหลือเวลา: ");
    if(i < 10) Serial.print("0");
    Serial.print(i);
    Serial.print(" วินาที ");
    
    // แสดง Progress Bar
    int progress = ((30 - i) * 20) / 30;
    Serial.print("[");
    for(int j = 0; j < 20; j++) {
      if(j < progress) Serial.print("█");
      else Serial.print("░");
    }
    Serial.print("] ");
    Serial.print(((30 - i) * 100) / 30);
    Serial.println("%");
    
    delay(500);
  }
  
  // เปิด LED เขียวค้างไว้
  digitalWrite(LED_GREEN_PIN, HIGH);
  Serial.println();
  Serial.println("✅ เซ็นเซอร์พร้อมใช้งาน!");
  Serial.println();
}

// ===========================================
// ฟังก์ชันทดสอบ LED
// ===========================================

void testLEDs() {
  Serial.println("🔍 ทดสอบระบบ LED...");
  
  // ทดสอบ LED เขียว
  Serial.print("💚 LED เขียว... ");
  digitalWrite(LED_GREEN_PIN, HIGH);
  digitalWrite(LED_RED_PIN, LOW);
  delay(1000);
  Serial.println("OK");
  
  // ทดสอบ LED แดง  
  Serial.print("❤️ LED แดง... ");
  digitalWrite(LED_GREEN_PIN, LOW);
  digitalWrite(LED_RED_PIN, HIGH);
  delay(1000);
  Serial.println("OK");
  
  // ทดสอบทั้งคู่พร้อมกัน
  Serial.print("🟡 LED ทั้งคู่... ");
  digitalWrite(LED_GREEN_PIN, HIGH);
  digitalWrite(LED_RED_PIN, HIGH);
  delay(1000);
  Serial.println("OK");
  
  // ปิดทั้งคู่
  digitalWrite(LED_GREEN_PIN, LOW);
  digitalWrite(LED_RED_PIN, LOW);
  delay(500);
  
  Serial.println("✅ ระบบ LED ทำงานปกติ");
  Serial.println();
}

// ===========================================
// ฟังก์ชันแสดงการกำหนดค่า
// ===========================================

void printConfiguration() {
  Serial.println("⚙️ การกำหนดค่าระบบ:");
  Serial.println("┌─────────────────────────────────────┐");
  Serial.print("│ ระดับเตือนมีเทน    : "); 
  Serial.print(METHANE_WARNING_LEVEL, 0); Serial.println(" ppm        │");
  Serial.print("│ ระดับอันตรายมีเทน  : "); 
  Serial.print(METHANE_DANGER_LEVEL, 0); Serial.println(" ppm       │");
  Serial.print("│ ระดับเตือนแอมโมเนีย : "); 
  Serial.print(AMMONIA_WARNING_LEVEL, 0); Serial.println(" ppm         │");
  Serial.print("│ ระดับอันตรายแอมโมเนีย: "); 
  Serial.print(AMMONIA_DANGER_LEVEL, 0); Serial.println(" ppm         │");
  Serial.print("│ ระดับเตือน H2S     : "); 
  Serial.print(H2S_WARNING_LEVEL, 0); Serial.println(" ppm         │");
  Serial.print("│ ระดับอันตราย H2S   : "); 
  Serial.print(H2S_DANGER_LEVEL, 0); Serial.println(" ppm         │");
  Serial.print("│ pH เหมาะสม         : "); 
  Serial.print(PH_MIN_OPTIMAL, 1); Serial.print(" - "); 
  Serial.print(PH_MAX_OPTIMAL, 1); Serial.println("       │");
  Serial.println("└─────────────────────────────────────┘");
  Serial.println();
}

// ===========================================
// ฟังก์ชันอ่านค่าจากเซ็นเซอร์ทั้งหมด
// ===========================================

void readAllSensors() {
  // อ่านค่า MQ-4 (มีเทน)
  mq4_raw = readSensorAverage(MQ4_PIN);
  mq4_voltage = (mq4_raw / ADC_RESOLUTION) * VOLTAGE_REF;
  
  // อ่านค่า MQ-135 (แอมโมเนีย/CO2)
  mq135_raw = readSensorAverage(MQ135_PIN);
  mq135_voltage = (mq135_raw / ADC_RESOLUTION) * VOLTAGE_REF;
  
  // อ่านค่า MQ-136 (H2S)
  mq136_raw = readSensorAverage(MQ136_PIN);
  mq136_voltage = (mq136_raw / ADC_RESOLUTION) * VOLTAGE_REF;
  
  // อ่านค่า pH
  ph_raw = readSensorAverage(PH_PIN);
  ph_voltage = (ph_raw / ADC_RESOLUTION) * VOLTAGE_REF;
  
  // อัพเดทเวลาการอ่านล่าสุด
  last_reading_time = millis();
}

// ===========================================
// ฟังก์ชันอ่านค่าเฉลี่ยจากเซ็นเซอร์
// ===========================================

int readSensorAverage(int pin) {
  long sum = 0;
  int validReadings = 0;
  
  for(int i = 0; i < NUM_READINGS; i++) {
    int reading = analogRead(pin);
    
    // ตรวจสอบค่าที่ถูกต้อง (0-1023)
    if(reading >= 0 && reading <= 1023) {
      sum += reading;
      validReadings++;
    }
    
    delay(READING_DELAY);
  }
  
  // ถ้าไม่มีค่าที่ถูกต้อง ให้คืนค่า 0
  if(validReadings == 0) return 0;
  
  return sum / validReadings;
}

// ===========================================
// ฟังก์ชันแปลงค่าเซ็นเซอร์
// ===========================================

void convertSensorValues() {
  // แปลงค่า MQ-4 เป็น ppm มีเทน
  methane_ppm = convertMQ4ToPPM(mq4_voltage);
  
  // แปลงค่า MQ-135 เป็น ppm แอมโมเนีย  
  ammonia_ppm = convertMQ135ToPPM(mq135_voltage);
  
  // แปลงค่า MQ-136 เป็น ppm H2S
  h2s_ppm = convertMQ136ToPPM(mq136_voltage);
  
  // แปลงค่า pH (ใช้ค่า Calibrated ถ้ามี)
  ph_value = convertToPH_Calibrated(ph_voltage);
}

// ===========================================
// ฟังก์ชันแปลงค่า MQ-4 เป็น ppm มีเทน
// ===========================================

float convertMQ4ToPPM(float voltage) {
  // สูตรการแปลงสำหรับ MQ-4 (ค่าประมาณจาก Datasheet)
  // ปรับแต่งตามการ Calibrate จริง
  
  if(voltage <= 0.4) return 0.0;
  
  // สูตร: ppm = 10^((voltage - offset) / slope)
  float log_ppm = (voltage - 0.45) / 0.3;
  float ppm = pow(10, log_ppm);
  
  // จำกัดค่าให้อยู่ในช่วงที่สมเหตุสมผล
  if(ppm < 0) ppm = 0;
  if(ppm > 100000) ppm = 100000;
  
  return ppm;
}

// ===========================================
// ฟังก์ชันแปลงค่า MQ-135 เป็น ppm แอมโมเนีย  
// ===========================================

float convertMQ135ToPPM(float voltage) {
  // สูตรการแปลงสำหรับ MQ-135
  // ค่านี้ใช้สำหรับแอมโมเนีย โดยประมาณ
  
  if(voltage <= 0.3) return 0.0;
  
  float log_ppm = (voltage - 0.4) / 0.35;
  float ppm = pow(10, log_ppm);
  
  if(ppm < 0) ppm = 0;
  if(ppm > 10000) ppm = 10000;
  
  return ppm;
}

// ===========================================
// ฟังก์ชันแปลงค่า MQ-136 เป็น ppm H2S
// ===========================================

float convertMQ136ToPPM(float voltage) {
  // สูตรการแปลงสำหรับ MQ-136 (H2S)
  
  if(voltage <= 0.3) return 0.0;
  
  float log_ppm = (voltage - 0.42) / 0.32;
  float ppm = pow(10, log_ppm);
  
  if(ppm < 0) ppm = 0;
  if(ppm > 1000) ppm = 1000;
  
  return ppm;
}

// ===========================================
// ฟังก์ชันแปลงค่าแรงดันเป็น pH
// สำหรับ Liquid pH Value Detection Sensor Module with BNC Electrode Probe
// ===========================================

float convertToPH(float voltage) {
  // สูตรการแปลงแรงดันเป็น pH สำหรับ BNC pH Sensor Module
  // เซ็นเซอร์นี้ทำงานที่ 25°C และมี resolution สูง
  
  // ค่า Default สำหรับ BNC pH Sensor Module (ต้อง Calibrate ด้วย pH Buffer)
  // pH 7.0 (Neutral) โดยปกติจะให้แรงดันประมาณ 2.5V
  // pH ลดลง 1 หน่วย (เป็นกรดมากขึ้น) = แรงดันเพิ่มขึ้นประมาณ 0.18V
  // pH เพิ่มขึ้น 1 หน่วย (เป็นด่างมากขึ้น) = แรงดันลดลงประมาณ 0.18V
  
  float neutralVoltage = 2.50;  // แรงดันที่ pH = 7.0 (ปรับตามการ Calibrate)
  float slope = -5.70;          // -1/0.18 ≈ -5.70 (ความชันเชิงลบ)
  
  // สูตร: pH = 7.0 + slope * (voltage - neutralVoltage)
  float ph = 7.0 + slope * (voltage - neutralVoltage);
  
  // จำกัดค่า pH ให้อยู่ในช่วง 0-14
  if(ph < 0) ph = 0;
  if(ph > 14) ph = 14;
  
  return ph;
}

// ===========================================
// ฟังก์ชันแสดงผลลัพธ์
// ===========================================

void displayResults() {
  // คำนวณเวลาการทำงาน
  unsigned long uptime = (millis() - system_start_time) / 1000;
  
  Serial.println("📊 ผลการตรวจวัดก๊าซชีวภาพ");
  Serial.print("⏰ เวลาทำงาน: ");
  printUptime(uptime);
  Serial.print(" | การอ่านครั้งที่: ");
  Serial.println(++total_readings);
  Serial.println();
  
  // แสดงค่าดิบ (ADC)
  Serial.println("🔢 ค่าดิบ (ADC 0-1023):");
  Serial.print("   MQ-4   : "); Serial.print(mq4_raw); Serial.print("/1023");
  Serial.print(" ("); Serial.print(mq4_voltage, 2); Serial.println("V)");
  Serial.print("   MQ-135 : "); Serial.print(mq135_raw); Serial.print("/1023");
  Serial.print(" ("); Serial.print(mq135_voltage, 2); Serial.println("V)");
  Serial.print("   MQ-136 : "); Serial.print(mq136_raw); Serial.print("/1023");
  Serial.print(" ("); Serial.print(mq136_voltage, 2); Serial.println("V)");
  Serial.print("   pH     : "); Serial.print(ph_raw); Serial.print("/1023");
  Serial.print(" ("); Serial.print(ph_voltage, 2); Serial.println("V)");
  Serial.println();
  
  // แสดงค่าที่แปลงแล้ว
  Serial.println("⚗️ ความเข้มข้นก๊าซ:");
  Serial.print("   🔥 มีเทน (CH4)        : ");
  Serial.print(methane_ppm, 1); Serial.println(" ppm");
  Serial.print("   💨 แอมโมเนีย (NH3)     : ");
  Serial.print(ammonia_ppm, 1); Serial.println(" ppm");
  Serial.print("   ☠️  ไฮโดรเจนซัลไฟด์ (H2S): ");
  Serial.print(h2s_ppm, 1); Serial.println(" ppm");
  Serial.println();
  
  Serial.println("🧪 คุณสมบัติของเหลว:");
  Serial.print("   📏 ค่า pH              : ");
  Serial.print(ph_value, 2);
  
  // แสดงสถานะ pH
  if(ph_value >= PH_MIN_OPTIMAL && ph_value <= PH_MAX_OPTIMAL) {
    Serial.println(" (เหมาะสม)");
  } else if(ph_value >= PH_MIN_ACCEPTABLE && ph_value <= PH_MAX_ACCEPTABLE) {
    Serial.println(" (ยอมรับได้)");
  } else {
    Serial.println(" (ไม่เหมาะสม)");
  }
  Serial.println();
  
  // คำนวณเปอร์เซ็นต์มีเทน
  float methane_percent = (methane_ppm / 10000.0) * 100.0;
  Serial.print("🔥 เปอร์เซ็นต์มีเทน      : ");
  Serial.print(methane_percent, 2); Serial.println("%");
  Serial.println();
}

// ===========================================
// ฟังก์ชันแสดงเวลาการทำงาน
// ===========================================

void printUptime(unsigned long seconds) {
  unsigned long hours = seconds / 3600;
  unsigned long minutes = (seconds % 3600) / 60;
  seconds = seconds % 60;
  
  if(hours < 10) Serial.print("0");
  Serial.print(hours);
  Serial.print(":");
  if(minutes < 10) Serial.print("0");
  Serial.print(minutes);
  Serial.print(":");
  if(seconds < 10) Serial.print("0");
  Serial.print(seconds);
}

// ===========================================
// ฟังก์ชันตรวจสอบระดับความปลอดภัย
// ===========================================

void checkSafetyLevels() {
  system_warning = false;
  system_danger = false;
  
  Serial.println("🛡️ การประเมินความปลอดภัย:");
  
  // ตรวจสอบระดับมีเทน
  if(methane_ppm >= METHANE_DANGER_LEVEL) {
    Serial.println("   🚨 มีเทน: อันตรายสูงมาก! อพยพทันที!");
    system_danger = true;
  } else if(methane_ppm >= METHANE_WARNING_LEVEL) {
    Serial.println("   ⚠️  มีเทน: ระดับเตือน - ระวังการลุกไหม้");
    system_warning = true;
  } else {
    Serial.println("   ✅ มีเทน: ระดับปลอดภัย");
  }
  
  // ตรวจสอบระดับแอมโมเนีย
  if(ammonia_ppm >= AMMONIA_DANGER_LEVEL) {
    Serial.println("   🚨 แอมโมเนีย: อันตรายต่อระบบหายใจ!");
    system_danger = true;
  } else if(ammonia_ppm >= AMMONIA_WARNING_LEVEL) {
    Serial.println("   ⚠️  แอมโมเนีย: ระดับเตือน - ใส่หน้ากาก");
    system_warning = true;
  } else {
    Serial.println("   ✅ แอมโมเนีย: ระดับปลอดภัย");
  }
  
  // ตรวจสอบระดับ H2S
  if(h2s_ppm >= H2S_DANGER_LEVEL) {
    Serial.println("   🚨 H2S: อันตรายมาก! ก๊าซพิษ!");
    system_danger = true;
  } else if(h2s_ppm >= H2S_WARNING_LEVEL) {
    Serial.println("   ⚠️  H2S: ระดับเตือน - อากาศถ่ายเทไม่ดี");
    system_warning = true;
  } else {
    Serial.println("   ✅ H2S: ระดับปลอดภัย");
  }
  
  // ตรวจสอบค่า pH
  if(ph_value < PH_MIN_ACCEPTABLE || ph_value > PH_MAX_ACCEPTABLE) {
    Serial.println("   ⚠️  pH: ไม่เหมาะสม - ส่งผลต่อการผลิตก๊าซ");
    system_warning = true;
  } else if(ph_value < PH_MIN_OPTIMAL || ph_value > PH_MAX_OPTIMAL) {
    Serial.println("   ⚡ pH: ยอมรับได้ - ควรปรับให้เหมาะสม");
  } else {
    Serial.println("   ✅ pH: เหมาะสมสำหรับการหมัก");
  }
  
  Serial.println();
  
  // สรุปสถานะรวม
  if(system_danger) {
    Serial.println("🚨 สถานะระบบ: อันตราย - ดำเนินการแก้ไขทันที!");
  } else if(system_warning) {
    Serial.println("⚠️  สถานะระบบ: เตือน - ควรติดตามและปรับปรุง");
  } else {
    Serial.println("✅ สถานะระบบ: ปลอดภัย - การดำเนินการปกติ");
  }
  Serial.println();
}

// ===========================================
// ฟังก์ชันควบคุม LED
// ===========================================

void controlLEDs() {
  if(system_danger) {
    // อันตราย: LED แดงกระพริบเร็ว
    digitalWrite(LED_RED_PIN, HIGH);
    digitalWrite(LED_GREEN_PIN, LOW);
    delay(100);
    digitalWrite(LED_RED_PIN, LOW);
    delay(100);
    digitalWrite(LED_RED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_RED_PIN, LOW);
    delay(100);
  } else if(system_warning) {
    // เตือน: LED แดงติดค้าง + LED เขียวดับ
    digitalWrite(LED_RED_PIN, HIGH);
    digitalWrite(LED_GREEN_PIN, LOW);
  } else {
    // ปกติ: LED เขียวติดค้าง + LED แดงดับ
    digitalWrite(LED_GREEN_PIN, HIGH);
    digitalWrite(LED_RED_PIN, LOW);
  }
}

// ===========================================
// ฟังก์ชันแสดงคำแนะนำ
// ===========================================

void showRecommendations() {
  Serial.println("💡 คำแนะนำการปรับปรุง:");
  
  bool hasRecommendation = false;
  
  // คำแนะนำสำหรับมีเทน
  if(methane_ppm < 200) {
    Serial.println("   🔥 มีเทนต่ำ: เพิ่มสารอินทรีย์หรือตรวจสอบอุณหภูมิ");
    hasRecommendation = true;
  } else if(methane_ppm > METHANE_WARNING_LEVEL) {
    Serial.println("   🔥 มีเทนสูง: เพิ่มการระบายอากาศและตรวจสอบการรั่ว");
    hasRecommendation = true;
  }
  
  // คำแนะนำสำหรับแอมโมเนีย
  if(ammonia_ppm > AMMONIA_WARNING_LEVEL) {
    Serial.println("   💨 แอมโมเนียสูง: ลดโปรตีนในวัตถุดิบหรือเพิ่มคาร์บอน");
    hasRecommendation = true;
  }
  
  // คำแนะนำสำหรับ H2S
  if(h2s_ppm > H2S_WARNING_LEVEL) {
    Serial.println("   ☠️  H2S สูง: ตรวจสอบการเน่าเสียหรือเพิ่มการคนผสม");
    hasRecommendation = true;
  }
  
  // คำแนะนำสำหรับ pH
  if(ph_value < PH_MIN_OPTIMAL) {
    Serial.println("   📏 pH ต่ำ (เป็นกรด): เพิ่มสารด่างหรือปูนขาว");
    hasRecommendation = true;
  } else if(ph_value > PH_MAX_OPTIMAL) {
    Serial.println("   📏 pH สูง (เป็นด่าง): เพิ่มสารอินทรีย์หรือลดสารด่าง");
    hasRecommendation = true;
  }
  
  if(!hasRecommendation) {
    Serial.println("   ✅ ไม่มีข้อแนะนำ - ระบบทำงานในสภาวะเหมาะสม");
  }
  
  Serial.println();
}

// ===========================================
// ฟังก์ชันอัพเดทสถิติ
// ===========================================

void updateStatistics() {
  // อัพเดทค่าสูงสุดของก๊าซ
  if(methane_ppm > max_methane) max_methane = methane_ppm;
  if(ammonia_ppm > max_ammonia) max_ammonia = ammonia_ppm;
  if(h2s_ppm > max_h2s) max_h2s = h2s_ppm;
  
  // อัพเดทค่าสูงสุดและต่ำสุดของ pH
  if(ph_value > max_ph) max_ph = ph_value;
  if(ph_value < min_ph) min_ph = ph_value;
}

// ===========================================
// ฟังก์ชันแสดงสถิติ
// ===========================================

void showStatistics() {
  Serial.println("📈 สถิติการทำงาน (ตั้งแต่เริ่มระบบ):");
  Serial.println("┌─────────────────────────────────────┐");
  Serial.print("│ การอ่านทั้งหมด      : "); 
  Serial.print(total_readings); Serial.println(" ครั้ง             │");
  Serial.print("│ มีเทนสูงสุด         : "); 
  Serial.print(max_methane, 1); Serial.println(" ppm             │");
  Serial.print("│ แอมโมเนียสูงสุด      : "); 
  Serial.print(max_ammonia, 1); Serial.println(" ppm             │");
  Serial.print("│ H2S สูงสุด          : "); 
  Serial.print(max_h2s, 1); Serial.println(" ppm             │");
  Serial.print("│ pH สูงสุด           : "); 
  Serial.print(max_ph, 2); Serial.println("                 │");
  Serial.print("│ pH ต่ำสุด           : "); 
  Serial.print(min_ph, 2); Serial.println("                 │");
  Serial.println("└─────────────────────────────────────┘");
  Serial.println();
}

// ===========================================
// ฟังก์ชันเพิ่มเติม - การ Calibrate pH BNC Sensor Module
// ===========================================

void calibratePH() {
  // ฟังก์ชันสำหรับ Calibrate เซ็นเซอร์ pH BNC Module
  // ใช้ pH Buffer Solutions: pH 4.0, pH 7.0, และ pH 10.0
  
  Serial.println("🔧 เริ่มการ Calibrate เซ็นเซอร์ pH BNC Module...");
  Serial.println("📋 ต้องใช้สารละลาย pH Buffer: pH 4.0, pH 7.0, pH 10.0");
  Serial.println();
  
  // ขั้นตอนที่ 1: Calibrate ที่ pH 7.0 (Neutral Point)
  Serial.println("ขั้นตอนที่ 1: การ Calibrate ที่ pH 7.0");
  Serial.println("1. ล้างหัว BNC Probe ด้วยน้ำกลั่น");
  Serial.println("2. เช็ดให้แห้งด้วยกระดาษทิชชู่");
  Serial.println("3. จุ่มหัว Probe ลงใน pH 7.0 Buffer");
  Serial.println("4. รอ 2-3 นาทีให้ค่าเสถียร");
  Serial.println("5. กด Enter เมื่อพร้อม...");
  
  // รอการกด Enter
  while(Serial.available() == 0) {
    delay(100);
  }
  while(Serial.available() > 0) {
    Serial.read(); // Clear buffer
  }
  
  // อ่านค่าที่ pH 7.0
  float ph7_voltage = 0;
  for(int i = 0; i < 20; i++) {
    ph7_voltage += (analogRead(PH_PIN) / 1023.0) * 5.0;
    delay(100);
  }
  ph7_voltage /= 20.0;
  
  Serial.print("📊 แรงดันที่ pH 7.0: ");
  Serial.print(ph7_voltage, 3);
  Serial.println(" V");
  
  // ขั้นตอนที่ 2: Calibrate ที่ pH 4.0 (Acid Point)
  Serial.println("\nขั้นตอนที่ 2: การ Calibrate ที่ pH 4.0");
  Serial.println("1. ล้างหัว Probe ด้วยน้ำกลั่น");
  Serial.println("2. เช็ดให้แห้ง");
  Serial.println("3. จุ่มหัว Probe ลงใน pH 4.0 Buffer");
  Serial.println("4. รอ 2-3 นาทีให้ค่าเสถียร");
  Serial.println("5. กด Enter เมื่อพร้อม...");
  
  while(Serial.available() == 0) {
    delay(100);
  }
  while(Serial.available() > 0) {
    Serial.read();
  }
  
  // อ่านค่าที่ pH 4.0
  float ph4_voltage = 0;
  for(int i = 0; i < 20; i++) {
    ph4_voltage += (analogRead(PH_PIN) / 1023.0) * 5.0;
    delay(100);
  }
  ph4_voltage /= 20.0;
  
  Serial.print("📊 แรงดันที่ pH 4.0: ");
  Serial.print(ph4_voltage, 3);
  Serial.println(" V");
  
  // คำนวณ Slope และ Offset
  float slope = (4.0 - 7.0) / (ph4_voltage - ph7_voltage);
  float offset = 7.0 - slope * ph7_voltage;
  
  Serial.println("\n🧮 ผลการคำนวณ:");
  Serial.print("   Slope: ");
  Serial.println(slope, 4);
  Serial.print("   Offset: ");
  Serial.println(offset, 4);
  
  // บันทึกค่าลง EEPROM (สำหรับใช้ต่อไป)
  EEPROM.put(0, slope);
  EEPROM.put(4, offset);
  EEPROM.put(8, ph7_voltage);
  
  Serial.println("💾 บันทึกค่า Calibration ลง EEPROM เรียบร้อย");
  Serial.println("🔄 รีสตาร์ทระบบเพื่อใช้ค่าใหม่...");
  Serial.println();
}

void loadCalibrationData() {
  // โหลดค่า Calibration จาก EEPROM
  float stored_slope, stored_offset, stored_neutral;
  
  EEPROM.get(0, stored_slope);
  EEPROM.get(4, stored_offset);
  EEPROM.get(8, stored_neutral);
  
  // ตรวจสอบว่าค่าที่อ่านมาสมเหตุสมผลหรือไม่
  if(!isnan(stored_slope) && !isnan(stored_offset) && 
     stored_slope > -10 && stored_slope < -3 &&
     stored_neutral > 1.0 && stored_neutral < 4.0) {
    
    Serial.println("📥 โหลดค่า Calibration จาก EEPROM:");
    Serial.print("   Slope: "); Serial.println(stored_slope, 4);
    Serial.print("   Offset: "); Serial.println(stored_offset, 4);
    Serial.print("   Neutral Voltage: "); Serial.print(stored_neutral, 3); Serial.println(" V");
    Serial.println("✅ ใช้ค่า Calibration ที่บันทึกไว้");
  } else {
    Serial.println("⚠️  ไม่พบค่า Calibration หรือค่าไม่ถูกต้อง");
    Serial.println("💡 แนะนำให้ทำการ Calibrate ก่อนใช้งาน");
  }
  Serial.println();
}

// ฟังก์ชันแปลงค่า pH ใหม่ (ใช้ค่าจาก EEPROM ถ้ามี)
float convertToPH_Calibrated(float voltage) {
  float stored_slope, stored_offset;
  
  EEPROM.get(0, stored_slope);
  EEPROM.get(4, stored_offset);
  
  // ถ้ามีค่า Calibration ใช้ค่านั้น
  if(!isnan(stored_slope) && !isnan(stored_offset) && 
     stored_slope > -10 && stored_slope < -3) {
    
    float ph = stored_slope * voltage + stored_offset;
    
    if(ph < 0) ph = 0;
    if(ph > 14) ph = 14;
    
    return ph;
  } else {
    // ถ้าไม่มีใช้ค่า Default
    return convertToPH(voltage);
  }
}

void resetStatistics() {
  // ฟังก์ชันรีเซ็ตสถิติ
  total_readings = 0;
  max_methane = 0.0;
  max_ammonia = 0.0;
  max_h2s = 0.0;
  min_ph = 14.0;
  max_ph = 0.0;
  system_start_time = millis();
  
  Serial.println("🔄 รีเซ็ตสถิติเรียบร้อย");
}

// ===========================================
// ฟังก์ชันตรวจสอบสถานะระบบ
// ===========================================

void systemDiagnostics() {
  Serial.println("🔍 การตรวจสอบระบบ:");
  Serial.println("┌─────────────────────────────────────┐");
  
  // ตรวจสอบแรงดันจ่ายไฟ
  float vcc = readVCC();
  Serial.print("│ แรงดันจ่ายไฟ       : "); 
  Serial.print(vcc, 2); Serial.println(" V               │");
  
  // ตรวจสอบเซ็นเซอร์ที่เสียหาย
  bool sensor_ok = true;
  if(mq4_raw == 0 || mq4_raw == 1023) {
    Serial.println("│ ⚠️  MQ-4 อาจเสียหาย                  │");
    sensor_ok = false;
  }
  if(mq135_raw == 0 || mq135_raw == 1023) {
    Serial.println("│ ⚠️  MQ-135 อาจเสียหาย                │");
    sensor_ok = false;
  }
  if(mq136_raw == 0 || mq136_raw == 1023) {
    Serial.println("│ ⚠️  MQ-136 อาจเสียหาย                │");
    sensor_ok = false;
  }
  if(ph_raw == 0 || ph_raw == 1023) {
    Serial.println("│ ⚠️  pH Sensor อาจเสียหาย             │");
    sensor_ok = false;
  }
  
  if(sensor_ok) {
    Serial.println("│ ✅ เซ็นเซอร์ทั้งหมดทำงานปกติ         │");
  }
  
  Serial.println("└─────────────────────────────────────┘");
}

// ===========================================
// ฟังก์ชันอ่านแรงดันจ่ายไฟ
// ===========================================

float readVCC() {
  // อ่านแรงดันจ่ายไฟผ่าน Internal Reference
  // ใช้สำหรับตรวจสอบความเสถียรของระบบ
  
  #if defined(__AVR_ATmega328P__)
    // สำหรับ Arduino Uno/Nano
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
    delay(2);
    ADCSRA |= _BV(ADSC);
    while (bit_is_set(ADCSRA,ADSC));
    
    uint8_t low  = ADCL;
    uint8_t high = ADCH;
    
    long result = (high<<8) | low;
    result = 1125300L / result;
  // ===========================================
// ฟังก์ชันตรวจสอบคำสั่งจาก Serial
// ===========================================

void checkSerialCommands() {
  if(Serial.available() > 0) {
    String command = Serial.readString();
    command.trim();
    command.toUpperCase();
    
    if(command == "CAL") {
      calibratePH();
    } else if(command == "RESET") {
      resetStatistics();
    } else if(command == "DIAG") {
      systemDiagnostics();
    } else if(command == "HELP") {
      printHelp();
    } else {
      Serial.print("❓ คำสั่งไม่รู้จัก: ");
      Serial.println(command);
      Serial.println("💡 พิมพ์ 'HELP' เพื่อดูคำสั่งที่ใช้ได้");
    }
    Serial.println();
  }
}

void printHelp() {
  Serial.println("📖 คำสั่งที่ใช้ได้:");
  Serial.println("┌─────────────────────────────────────┐");
  Serial.println("│ CAL   - Calibrate เซ็นเซอร์ pH       │");
  Serial.println("│ RESET - รีเซ็ตสถิติการทำงาน          │");
  Serial.println("│ DIAG  - ตรวจสอบสถานะระบบ            │");
  Serial.println("│ HELP  - แสดงความช่วยเหลือ            │");
  Serial.println("└─────────────────────────────────────┘");
}
