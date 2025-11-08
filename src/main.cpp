// ------------------------------------
// FUSION DES CAPTEURS MPU-6050 (I2C), DHT22 (Digital), MOTEUR (Digital), FSR 402 (Analogique) & LED RGB (PWM)
// ------------------------------------
#include <Arduino.h> 
#include "Wire.h"  // Necessaire pour le MPU-6050 (I2C)
#include "DHT.h"   // Necessaire pour le capteur DHT22 (Temperature/Humidite)

// ------------------------------------
// 1. DÉFINITIONS DES BROCHES ET INTERVALLES
// ------------------------------------
// DHT22
#define DHTTYPE DHT22 
const int DHTPin = 2;       // DHT22 Data connecte a D2
const long DHT_MPU_INTERVAL = 2000; // Intervalle de lecture des capteurs (2 secondes)

// Moteur et FSR
const int VIBRATION_MOTOR_PIN = 3; // Moteur Vibrateur DEPLACE vers D3
const int FSR_PIN = A0;            // FSR connecte a la broche analogique A0
const int PRESSURE_THRESHOLD = 400; // Seuil de pression (valeur brute 0-1023) pour activer le moteur

// Bouton Poussoir Unique (Nouveau)
const int SINGLE_BUTTON_PIN = 7;     // Bouton unique connecte a D4 (avec Pull-Down)
const int MOTION_THRESHOLD = 10000; // Seuil de mouvement brusque MPU (valeur brute +/- 10000)

// LED RGB 
const int LED_R_PIN = 9;
const int LED_G_PIN = 10;
const int LED_B_PIN = 11;
const long RGB_DEMO_INTERVAL = 30; // Vitesse de transition de la demo (ms) - Ajustee pour plus de douceur

// Instances des objets et variables de temps
DHT dht(DHTPin, DHTTYPE); 
unsigned long previousDhtMpuMillis = 0;
unsigned long previousRgbMillis = 0; 
int motorState = LOW; 

// Variables pour l'état du système
bool alertActive = false; // Etat d'alerte (déclenché par MPU)

// Variables pour le cycle de couleur RGB
int colorR = 255; 
int colorG = 0;
int colorB = 255; 
int step = 0; 

// ------------------------------------
// 2. DÉFINITIONS DU MPU-6050 (Gyroscope et Accéléromètre)
// ------------------------------------
const int I2C_adress_MPU = 0x68; 
int16_t Accel_x, Accel_y, Accel_z; 
int16_t Gyro_x, Gyro_y, Gyro_z;   
int16_t Temp_mpu_raw;             

// Variable utilitaire pour le formatage de chaine
char tmp_str[7]; 

// Fonction pour convertir int16_t en chaine formatee
char* convert_int16_to_str(int16_t i) {
  sprintf(tmp_str, "%6d", i); 
  return tmp_str;
}

// Fonction pour mettre a jour les couleurs de la LED RGB
void updateRgbLeds(int r, int g, int b) {
  // Utilisez analogWrite pour controler l'intensite
  analogWrite(LED_R_PIN, r);
  analogWrite(LED_G_PIN, g);
  analogWrite(LED_B_PIN, b);
}

// Fonction pour executer le cycle de couleur (DEMO)
void runRgbDemo() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousRgbMillis >= RGB_DEMO_INTERVAL) {
    previousRgbMillis = currentMillis;

    // Logique de transition ciblant Rose, Magenta et Cyan
    switch (step) {
      // 0: Magenta (R=255, B=255) -> Bleu (B=255, R diminue)
      case 0: 
        colorR -= 5;
        if (colorR <= 0) { colorR = 0; step = 1; }
        break;
      // 1: Bleu (B=255) -> Cyan (B=255, G augmente)
      case 1: 
        colorG += 5; 
        if (colorG >= 255) { colorG = 255; step = 2; }
        break;
      // 2: Cyan (G=255, B=255) -> Vert (G=255, B diminue)
      case 2: 
        colorB -= 5; 
        if (colorB <= 0) { colorB = 0; step = 3; }
        break;
      // 3: Vert (G=255) -> Jaune (G=255, R augmente)
      case 3: 
        colorR += 5; 
        if (colorR >= 255) { colorR = 255; step = 4; }
        break;
      // 4: Jaune (R=255, G=255) -> Rouge (R=255, G diminue)
      case 4: 
        colorG -= 5; 
        if (colorG <= 0) { colorG = 0; step = 5; }
        break;
      // 5: Rouge (R=255) -> Magenta (R=255, B augmente)
      case 5: 
        colorB += 5; 
        if (colorB >= 255) { colorB = 255; step = 0; }
        break;
    }
    updateRgbLeds(colorR, colorG, colorB);
  }
}


// ------------------------------------
// 3. FONCTION D'INITIALISATION (setup)
// ------------------------------------
void setup() {
  Serial.begin(9600); 
  Serial.println("--- Systeme Multi-Capteurs & Actionneurs Initialisation ---");
  
  // Initialisation I2C pour MPU-6050
  Wire.begin();
  
  // Initialisation du DHT
  dht.begin();
  Serial.println("DHT22 initialisé sur D2.");
  
  // Initialisation du Moteur Vibrateur, FSR, RGB et Bouton Unique
  pinMode(VIBRATION_MOTOR_PIN, OUTPUT); // Moteur sur D3
  pinMode(FSR_PIN, INPUT); 
  pinMode(SINGLE_BUTTON_PIN, INPUT); // Bouton sur D4
  pinMode(LED_R_PIN, OUTPUT);
  pinMode(LED_G_PIN, OUTPUT);
  pinMode(LED_B_PIN, OUTPUT);
  Serial.println("Actionneurs (Moteur D3, RGB D9/10/11) initialisés. FSR sur A0, Bouton sur D4.");
  Serial.print("Seuil de pression FSR pour moteur: "); Serial.println(PRESSURE_THRESHOLD);

  // Configuration et verification du MPU-6050
  Wire.beginTransmission(I2C_adress_MPU); 
  Wire.write(0x6B); 
  Wire.write(0);    
  byte status = Wire.endTransmission(true);

  if (status != 0) {
    Serial.println("==========================================");
    Serial.println("ERREUR: MPU-6050 non trouve (Verifiez cablage A4/A5).");
    Serial.print("Statut I2C: "); Serial.println(status);
    Serial.println("Le programme est bloque en attente du MPU.");
    Serial.println("==========================================");
    while (1) { delay(500); } 
  }
  
  Serial.println("MPU-6050 initialisé. Le rafraichissement est non-bloquant.");
  delay(1000);
}

// ------------------------------------
// 4. FONCTION PRINCIPALE (loop)
// ------------------------------------
void loop() {
  unsigned long currentMillis = millis();
  
  // Lecture des entrees
  int fsrReading = analogRead(FSR_PIN); // Lecture brute FSR 0-1023
  int buttonPressed = digitalRead(SINGLE_BUTTON_PIN); // HIGH si le bouton est presse, LOW sinon

  // --- LOGIQUE DE CONTRÔLE DU SYSTÈME ---

  // 1. Contrôle du moteur par le FSR (indépendant de l'alerte)
  if (fsrReading > PRESSURE_THRESHOLD) {
    digitalWrite(VIBRATION_MOTOR_PIN, HIGH);
    motorState = HIGH;
  } else {
    digitalWrite(VIBRATION_MOTOR_PIN, LOW);
    motorState = LOW;
  }
  
  // 2. LOGIQUE D'ALERTE MPU-6050 ET NEUTRALISATION PAR BOUTON
  if (alertActive) {
    // A. EN MODE ALERTE: Afficher ROUGE CLIGNOTANT et attendre le bouton
    if (buttonPressed == HIGH) { // Un seul bouton est necessaire
      alertActive = false; // Neutralisation de l'alerte
      updateRgbLeds(0, 255, 0); // Vert pour confirmation
      Serial.println("+++ ALERTE NEUTRALISEE! Appuyez pour continuer. +++");
      delay(2000);
    } else {
      // Rouge Clignotant
      if ((currentMillis / 200) % 2 == 0) {
        updateRgbLeds(255, 0, 0); // Rouge Vif
      } else {
        updateRgbLeds(0, 0, 0);   // Eteint
      }
    }
  } else {
    // B. EN MODE NORMAL: Gerer les autres actions (FSR, MPU, Demo RGB)
    if (fsrReading > PRESSURE_THRESHOLD) {
      updateRgbLeds(255, 0, 128); // ROSE VIF pour signaler la pression FSR
    } else {
      runRgbDemo(); // Executer la demo de cycle de couleur
    }
  }


  // --- LECTURE ET DÉTECTION D'ALERTE (Toutes les 2 secondes) ---
  if (currentMillis - previousDhtMpuMillis >= DHT_MPU_INTERVAL) {
    previousDhtMpuMillis = currentMillis;

    // 1. LECTURE MPU-6050 (Accéléromètre/Gyroscope)
    Wire.beginTransmission(I2C_adress_MPU);
    Wire.write(0x3B); 
    Wire.endTransmission(false); 
    Wire.requestFrom(I2C_adress_MPU, 7*2, true); 
    
    Accel_x = Wire.read()<<8 | Wire.read();
    Accel_y = Wire.read()<<8 | Wire.read();
    Accel_z = Wire.read()<<8 | Wire.read();
    Temp_mpu_raw = Wire.read()<<8 | Wire.read();
    Gyro_x = Wire.read()<<8 | Wire.read();
    Gyro_y = Wire.read()<<8 | Wire.read();
    Gyro_z = Wire.read()<<8 | Wire.read();

    // DÉTECTION DE CHOC/MOUVEMENT BRUSQUE
    if (!alertActive && (abs(Accel_x) > MOTION_THRESHOLD || abs(Accel_y) > MOTION_THRESHOLD)) {
        alertActive = true;
        Serial.println("!!! ALERTE MPU: Mouvement Brusque Detecte !!!");
    }

    // 2. LECTURE DHT22 (Humidité/Température)
    float h = dht.readHumidity();
    float t = dht.readTemperature(); 
    
    // 3. AFFICHAGE DES DONNÉES
    Serial.print(alertActive ? "[ALERTE ACTIVE]" : "[NORMAL]");
    Serial.print(" | Bouton="); Serial.print(buttonPressed == HIGH ? "PRESSED" : "LOW");
    Serial.print(" | Moteur="); Serial.print(motorState == HIGH ? "ON" : "OFF");
    Serial.print(" | FSR="); Serial.print(fsrReading);
    
    Serial.print(" | MPU6050 | aX="); Serial.print(convert_int16_to_str(Accel_x));
    Serial.print(" | aY="); Serial.print(convert_int16_to_str(Accel_y));
    Serial.print(" | aZ="); Serial.print(convert_int16_to_str(Accel_z));
    Serial.print(" | gX="); Serial.print(convert_int16_to_str(Gyro_x));
    Serial.print(" | gY="); Serial.print(convert_int16_to_str(Gyro_y));
    Serial.print(" | gZ="); Serial.print(convert_int16_to_str(Gyro_z));
    
    Serial.print(" | T_MPU_Puce="); Serial.print(Temp_mpu_raw/340.00+36.53); 

    if (!isnan(h) && !isnan(t)) {
      Serial.print(" | DHT22 | Humidite=");
      Serial.print(h, 1); 
      Serial.print(" % | Temperature=");
      Serial.print(t, 1); 
    } else {
      Serial.print(" | DHT22 | Lecture invalide (Verifiez D2) ");
    }
    Serial.println(" "); 
  }
}