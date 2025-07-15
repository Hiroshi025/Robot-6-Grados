#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>

// =============================================
// CONFIGURACIÓN AVANZADA DEL BRAZO ROBÓTICO
// =============================================

#define DEFAULT_SPEED 20  // Paso incremental (ajustar para mayor/menor velocidad)
#define DEBUG_MODE true
#define DEFAULT_FREQ 60
#define STEP_DELAY 50
#define SAFETY_DELAY 50
#define MAX_SEQUENCE_STEPS 9  // Reducido para ahorrar memoria

const uint16_t ABSOLUTE_MIN = 150;
const uint16_t ABSOLUTE_MAX = 600;

// Posición HOME personalizada
const uint16_t HOME_POSITIONS[6] = { 385, 150, 400, 480, 285, 600 };

// Posiciones de TEST
const uint16_t TEST_POSITIONS[6] = { 450, 400, 450, 300, 300, 400 };

// =============================================
// ESTRUCTURAS DE DATOS
// =============================================

struct ServoConfig {
  uint16_t minPulse;
  uint16_t maxPulse;
  uint16_t homePos;
  uint16_t currentPos;
  uint16_t testPos;
  const char* name;
};

struct SequenceStep {
  uint16_t positions[6];
  uint16_t duration;
};

struct RobotSequence {
  SequenceStep steps[MAX_SEQUENCE_STEPS];
  uint8_t stepCount;
  char name[30];
};

// =============================================
// VARIABLES GLOBALES
// =============================================

ServoConfig servos[6] = {
  { 150, 600, HOME_POSITIONS[0], HOME_POSITIONS[0], TEST_POSITIONS[0], "Base" },
  { 150, 600, HOME_POSITIONS[1], HOME_POSITIONS[1], TEST_POSITIONS[1], "Hombro" },
  { 150, 600, HOME_POSITIONS[2], HOME_POSITIONS[2], TEST_POSITIONS[2], "Codo" },
  { 150, 600, HOME_POSITIONS[3], HOME_POSITIONS[3], TEST_POSITIONS[3], "Muneca" },
  { 150, 450, HOME_POSITIONS[4], HOME_POSITIONS[4], TEST_POSITIONS[4], "Giro Pinza" },
  { 150, 450, HOME_POSITIONS[5], HOME_POSITIONS[5], TEST_POSITIONS[5], "Apertura Pinza" }
};

enum OperationMode {
  MODE_NORMAL,
  MODE_CALIBRATION,
  MODE_TEST,
  MODE_SEQUENCE
};

OperationMode currentMode = MODE_NORMAL;
uint8_t currentServo = 0;
bool systemArmed = false;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Secuencias programadas
RobotSequence sequences[2];
uint8_t currentSequence = 0;
uint8_t currentStep = 0;
unsigned long sequenceStartTime = 0;
bool isPlayingSequence = false;
bool repeatSequence = false;

// =============================================
// FUNCIONES PRINCIPALES
// =============================================

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000)
    ;  // Timeout de 2 segundos

  pwm.begin();
  pwm.setPWMFreq(DEFAULT_FREQ);

  initializeSystem();
  initializeSequences();
  printWelcomeMessage();
}

void loop() {
  if (isPlayingSequence) {
    playSequence();
  }

  if (Serial.available()) {
    handleSerialCommands();
  }
  delay(10);
}

// =============================================
// FUNCIONES DE CONTROL DE MOVIMIENTO
// =============================================

void moveServo(uint8_t servoNum, uint16_t targetPos) {
  if (!systemArmed) {
    Serial.println(F("Sistema no armado! Enviar 'A' para armar"));
    return;
  }

  targetPos = constrain(targetPos, servos[servoNum].minPulse, servos[servoNum].maxPulse);
  targetPos = constrain(targetPos, ABSOLUTE_MIN, ABSOLUTE_MAX);

  if (targetPos == servos[servoNum].currentPos) return;

  uint16_t current = servos[servoNum].currentPos;
  int step = (targetPos > current) ? DEFAULT_SPEED : -DEFAULT_SPEED;

  while (abs(current - targetPos) > DEFAULT_SPEED) {
    current += step;
    pwm.setPWM(servoNum, 0, current);
    delay(STEP_DELAY);
  }

  pwm.setPWM(servoNum, 0, targetPos);
  servos[servoNum].currentPos = targetPos;
  delay(SAFETY_DELAY);

  if (DEBUG_MODE) {
    Serial.print(servos[servoNum].name);
    Serial.print(F(" -> "));
    Serial.print(targetPos);
    Serial.println(F(" us"));
  }
}

void moveAllServos(uint16_t positions[6]) {
  bool allReached;

  if (DEBUG_MODE) {
    Serial.println(F("Moviendo todos los servos a:"));
    for (uint8_t i = 0; i < 6; i++) {
      Serial.print(servos[i].name);
      Serial.print(F(": "));
      Serial.println(positions[i]);
    }
  }

  do {
    allReached = true;
    for (uint8_t i = 0; i < 6; i++) {
      uint16_t current = servos[i].currentPos;
      uint16_t target = positions[i];

      if (abs(current - target) > DEFAULT_SPEED) {
        allReached = false;
        int step = (target > current) ? DEFAULT_SPEED : -DEFAULT_SPEED;
        current += step;
        pwm.setPWM(i, 0, current);
        servos[i].currentPos = current;
      }
    }
    delay(STEP_DELAY);
  } while (!allReached);

  for (uint8_t i = 0; i < 6; i++) {
    pwm.setPWM(i, 0, positions[i]);
    servos[i].currentPos = positions[i];
  }
  delay(SAFETY_DELAY);
}

void moveAllToHome() {
  Serial.println(F("\nMoviendo a posicion HOME..."));
  for (uint8_t i = 0; i < 6; i++) {
    moveServo(i, HOME_POSITIONS[i]);
  }
  Serial.println(F("Posicion HOME alcanzada"));
}

// =============================================
// FUNCIONES DE SECUENCIAS
// =============================================

void initializeSequences() {
  // Configurar Secuencia 1
  sequences[0].stepCount = 9;
  strncpy(sequences[0].name, "Proyecto Embasadora", sizeof(sequences[0].name));
  sequences[0].steps[0] = { { 382, 150, 495, 435, 285, 150 }, 2000 };
  sequences[0].steps[1] = { { 382, 187, 495, 390, 285, 150 }, 2000 };
  sequences[0].steps[2] = { { 382, 187, 495, 375, 285, 150 }, 2000 };
  sequences[0].steps[3] = { { 382, 187, 495, 375, 285, 700 }, 2000 }; //cierre de la garra para agarrar el frasco
  sequences[0].steps[4] = { { 382, 150, 440, 300, 285, 700 }, 2000 };
  sequences[0].steps[5] = { { 600, 150, 440, 300, 285, 700 }, 2000 };
  sequences[0].steps[6] = { { 600, 190, 440, 380, 285, 700 }, 2000 };
  sequences[0].steps[7] = { { 600, 150, 510, 300, 285, 190 }, 2000 };
  sequences[0].steps[8] = { { 380, 150, 400, 480, 285, 600 }, 2000 };

  // Configurar Secuencia 2
  sequences[1].stepCount = 1;
  strncpy(sequences[1].name, "Secuencia de Prueba", sizeof(sequences[1].name));
  sequences[1].steps[0] = { { 215, 225, 150, 150, 285, 450 }, 500 };
}

void playSequence() {
  if (millis() - sequenceStartTime >= sequences[currentSequence].steps[currentStep].duration) {
    currentStep++;

    if (currentStep >= sequences[currentSequence].stepCount) {
      if (repeatSequence) {
        currentStep = 0;
        sequenceStartTime = millis();
        Serial.println(F("\nReiniciando secuencia (modo repeticion)"));
        moveAllServos(sequences[currentSequence].steps[0].positions);
      } else {
        isPlayingSequence = false;
        Serial.println(F("\nSecuencia completada"));
      }
      return;
    }

    sequenceStartTime = millis();
    Serial.print(F("\nEjecutando paso "));
    Serial.print(currentStep + 1);
    Serial.print(F("/"));
    Serial.println(sequences[currentSequence].stepCount);

    moveAllServos(sequences[currentSequence].steps[currentStep].positions);
  }
}

void startSequence(uint8_t seqIndex) {
  if (seqIndex >= 2) {
    Serial.println(F("Indice de secuencia invalido"));
    return;
  }

  currentMode = MODE_SEQUENCE;
  currentSequence = seqIndex;
  currentStep = 0;
  isPlayingSequence = true;
  sequenceStartTime = millis();

  Serial.print(F("\nIniciando secuencia: "));
  Serial.println(sequences[seqIndex].name);

  moveAllServos(sequences[seqIndex].steps[0].positions);
}

void stopSequence() {
  isPlayingSequence = false;
  Serial.println("Secuencia detenida");
}

void printSequenceInfo(uint8_t seqIndex) {
  if (seqIndex >= 3) return;

  Serial.print("\nInformación de secuencia ");
  Serial.println(sequences[seqIndex].name);
  Serial.print("Pasos: ");
  Serial.println(sequences[seqIndex].stepCount);

  for (uint8_t i = 0; i < sequences[seqIndex].stepCount; i++) {
    Serial.print("Paso ");
    Serial.print(i + 1);
    Serial.print(" (");
    Serial.print(sequences[seqIndex].steps[i].duration);
    Serial.println(" ms):");

    for (uint8_t j = 0; j < 6; j++) {
      Serial.print(" ");
      Serial.print(servos[j].name);
      Serial.print(": ");
      Serial.println(sequences[seqIndex].steps[i].positions[j]);
    }
  }
}

// =============================================
// INTERFAZ DE USUARIO
// =============================================

void printWelcomeMessage() {
  Serial.println("\n=============================================");
  Serial.println("    SISTEMA DE CONTROL DE BRAZO ROBÓTICO");
  Serial.println("=============================================");
  Serial.println("Versión 2.2 - Con control de secuencias");
  Serial.println("---------------------------------------------");
  printSystemStatus();
  Serial.println("---------------------------------------------");
  printMainHelp();
}

void printSystemStatus() {
  Serial.println("Estado del sistema:");
  Serial.print("Modo: ");
  switch (currentMode) {
    case MODE_NORMAL: Serial.println("NORMAL"); break;
    case MODE_CALIBRATION: Serial.println("CALIBRACIÓN"); break;
    case MODE_TEST: Serial.println("TEST"); break;
    case MODE_SEQUENCE: Serial.println("SECUENCIA"); break;
  }

  Serial.print("Sistema: ");
  Serial.println(systemArmed ? "ARMADO" : "NO ARMADO");

  Serial.println("\nPosiciones actuales:");
  for (uint8_t i = 0; i < 6; i++) {
    Serial.print(" ");
    Serial.print(servos[i].name);
    Serial.print(": ");
    Serial.print(servos[i].currentPos);
    Serial.println(" us");
  }
}

void printMainHelp() {
  Serial.println(F("Comandos principales:"));
  Serial.println(F(" A - Armar sistema"));
  Serial.println(F(" D - Desarmar sistema"));
  Serial.println(F(" H - Mover a posición HOME"));
  Serial.println(F(" J - Mostrar estado actual"));
  Serial.println(F(" C - Modo Calibración"));
  Serial.println(F(" T - Modo Test"));
  Serial.println(F(" S - Modo Secuencia"));
  Serial.println(F(" 1-3 - Ejecutar secuencia 1-3"));
  Serial.println(F(" ? - Mostrar ayuda"));
}

void printSequenceHelp() {
  Serial.println(F("\nCOMANDOS DE SECUENCIA:"));
  Serial.println(F(" 1-3 : Ejecutar secuencia 1-3"));
  Serial.println(F(" I : Información de secuencia"));
  Serial.println(F(" X : Detener secuencia"));
  Serial.println(F(" R : Activar/desactivar repetición automática"));
  Serial.println(F(" ? : Mostrar esta ayuda"));
}

void printCalibrationHelp() {
  Serial.println(F("\nCOMANDOS DE CALIBRACIÓN:"));
  Serial.println(F(" +/- : Aumentar/disminuir posición"));
  Serial.println(F(" N/P : Siguiente/anterior servo"));
  Serial.println(F(" L : Guardar límite mínimo"));
  Serial.println(F(" U : Guardar límite máximo"));
  Serial.println(F(" H : Guardar posición HOME"));
  Serial.println(F(" X : Salir del modo calibración"));
  Serial.println(F(" ? : Mostrar esta ayuda"));
}

void printTestHelp() {
  Serial.println(F("\nCOMANDOS DE TEST:"));
  Serial.println(F(" T : Mover a posición TEST"));
  Serial.println(F(" H : Mover a posición HOME"));
  Serial.println(F(" N/P : Siguiente/anterior servo"));
  Serial.println(F(" X : Salir del modo test"));
  Serial.println(F(" ? : Mostrar esta ayuda"));
}

// =============================================
// MANEJO DE COMANDOS SERIALES
// =============================================

void handleSerialCommands() {
  while (Serial.available()) {  // Cambiado de if a while para limpiar el buffer
    char command = toupper(Serial.read());

    // Ignorar caracteres de control
    if (command == '\n' || command == '\r') {
      continue;
    }

    switch (currentMode) {
      case MODE_NORMAL: processNormalCommand(command); break;
      case MODE_CALIBRATION: processCalibrationCommand(command); break;
      case MODE_TEST: processTestCommand(command); break;
      case MODE_SEQUENCE: processSequenceCommand(command); break;
    }

    // Limpiar cualquier caracter adicional en el buffer
    while (Serial.available() && Serial.read() != '\n')
      ;
  }
}

void processNormalCommand(char command) {
  switch (command) {
    case 'A':
      systemArmed = true;
      Serial.println("Sistema ARMADO - Motores activados");
      break;

    case 'D':
      systemArmed = false;
      Serial.println("Sistema DESARMADO - Motores desactivados");
      break;

    case 'H':
      moveAllToHome();
      break;

    case 'J':
      printSystemStatus();
      break;

    case 'C':
      currentMode = MODE_CALIBRATION;
      printCalibrationHelp();
      break;

    case 'T':
      currentMode = MODE_TEST;
      printTestHelp();
      break;

    case 'S':
      currentMode = MODE_SEQUENCE;
      printSequenceHelp();
      break;

    case '1':
    case '2':
    case '3':
      startSequence(command - '1');
      break;

    case '?':
      printMainHelp();
      break;

    // Movimiento con teclas WASD
    case 'W':
      if (systemArmed) {
        uint16_t newPos = servos[currentServo].currentPos + 5;
        moveServo(currentServo, newPos);
      }
      break;

    case 'Z':
      if (systemArmed) {
        uint16_t newPos = servos[currentServo].currentPos - 5;
        moveServo(currentServo, newPos);
      }
      break;

    case 'Q':
      currentServo = (currentServo == 0) ? 5 : currentServo - 1;
      Serial.print("Servo seleccionado: ");
      Serial.println(servos[currentServo].name);
      break;

    case 'E':
      currentServo = (currentServo == 5) ? 0 : currentServo + 1;
      Serial.print("Servo seleccionado: ");
      Serial.println(servos[currentServo].name);
      break;

    default:
      Serial.println("Comando no reconocido. Enviar ? para ayuda");
  }
}

void processSequenceCommand(char command) {
  switch (command) {
    case '1':
    case '2':
    case '3':
      startSequence(command - '1');
      break;

    case 'I':
      if (!isPlayingSequence) {
        printSequenceInfo(currentSequence);
      } else {
        Serial.println("Espere a que termine la secuencia actual");
      }
      break;

    case 'X':
      stopSequence();
      break;

    case 'R':
      repeatSequence = !repeatSequence;
      Serial.print("Modo repetición: ");
      Serial.println(repeatSequence ? "ACTIVADO" : "DESACTIVADO");
      break;

    case '?':
      printSequenceHelp();
      break;

    case 'N':
      currentMode = MODE_NORMAL;
      Serial.println("\nMODO NORMAL ACTIVADO");
      printMainHelp();

    case 'C':
      currentMode = MODE_CALIBRATION;
      printCalibrationHelp();
      break;

      break;

    default:
      Serial.println("Comando no reconocido. Enviar ? para ayuda");
  }
}

void processCalibrationCommand(char command) {
  // Si el comando es un número, leer el valor completo desde el buffer serial
  if (isDigit(command)) {
    String numberStr = String(command);
    if (Serial.available()) {
      numberStr += Serial.readStringUntil('\n');
    }
    int value = numberStr.toInt();
    if (value >= 150 && value <= 600) {
      moveServo(currentServo, value);
      Serial.print("Servo ");
      Serial.print(servos[currentServo].name);
      Serial.print(" movido a ");
      Serial.println(value);
    } else {
      Serial.println("Valor fuera de rango (150-600)");
    }
    return;
  }

  switch (command) {
    case 'X':
      currentMode = MODE_NORMAL;
      Serial.println("\nMODO NORMAL ACTIVADO");
      printMainHelp();
      break;

    case 'N':
      currentServo = (currentServo + 1) % 6;
      Serial.print("Calibrando Servo ");
      Serial.print(servos[currentServo].name);
      Serial.print(" (posición actual: ");
      Serial.print(servos[currentServo].currentPos);
      Serial.println(")");
      break;

    case 'P':
      currentServo = (currentServo == 0) ? 5 : currentServo - 1;
      Serial.print("Calibrando Servo ");
      Serial.print(servos[currentServo].name);
      Serial.print(" (posición actual: ");
      Serial.print(servos[currentServo].currentPos);
      Serial.println(")");
      break;

    case '+':
      moveServo(currentServo, servos[currentServo].currentPos + 15);
      break;

    case '-':
      moveServo(currentServo, servos[currentServo].currentPos - 15);
      break;

    case 'L':
      servos[currentServo].minPulse = servos[currentServo].currentPos;
      //Serial.print("Límite mínimo guardado: ");
      Serial.println(servos[currentServo].minPulse);
      break;

    case 'U':
      servos[currentServo].maxPulse = servos[currentServo].currentPos;
      //Serial.print("Límite máximo guardado: ");
      Serial.println(servos[currentServo].maxPulse);
      break;

    case 'H':
      servos[currentServo].homePos = servos[currentServo].currentPos;
      //Serial.print("Posición HOME guardada: ");
      Serial.println(servos[currentServo].homePos);
      break;

    case '?':
      printCalibrationHelp();
      break;

    default:
      Serial.println("Comando no reconocido. Enviar ? para ayuda");
  }
}

void processTestCommand(char command) {
  switch (command) {
    case 'X':
      currentMode = MODE_NORMAL;
      Serial.println("\nMODO NORMAL ACTIVADO");
      printMainHelp();
      break;

    case 'T':
      moveServo(currentServo, servos[currentServo].testPos);
      Serial.print(servos[currentServo].name);
      Serial.println(" movido a posición TEST");
      break;

    case 'H':
      moveServo(currentServo, servos[currentServo].homePos);
      Serial.print(servos[currentServo].name);
      Serial.println(" movido a posición HOME");
      break;

    case 'N':
      currentServo = (currentServo + 1) % 6;
      Serial.print("Servo TEST seleccionado: ");
      Serial.println(servos[currentServo].name);
      break;

    case 'P':
      currentServo = (currentServo == 0) ? 5 : currentServo - 1;
      Serial.print("Servo TEST seleccionado: ");
      Serial.println(servos[currentServo].name);
      break;

    case '?':
      printTestHelp();
      break;

    default:
      Serial.print(F("Comando no reconocido: ["));
      Serial.print(command);
      Serial.println(F("]. Enviar ? para ayuda"));
      break;
  }
}

// =============================================
// FUNCIONES DE INICIALIZACIÓN
// =============================================

void initializeSystem() {
  // Mover todos los servos a posición HOME
  for (uint8_t i = 0; i < 6; i++) {
    pwm.setPWM(i, 0, HOME_POSITIONS[i]);
    servos[i].currentPos = HOME_POSITIONS[i];
    delay(100);
  }

  systemArmed = true;
  Serial.println(F("Sistema inicializado y armado"));
}