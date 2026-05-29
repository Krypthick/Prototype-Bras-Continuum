/*******************************************************************************
 * STM32 - CONTROLEUR MOTEURS 
 * Projet: Prototype bras continuum
 * Auteur: Jeff Truong
 *
 * Recoit les commandes UART depuis l'ESP32 mainboard et pilote 4 moteurs
 * pas a pas via les drivers A4988.
 ******************************************************************************/

#include "stm32f1xx_hal.h"
#include <Arduino.h>

HardwareSerial Serial1(PA10, PA9);

// Broches moteurs a ajuster selon schema. ------
const int STEP_PINS[] = {PB0, PB10, PB7, PB9};
const int DIR_PINS[]  = {PB1, PB2,  PB6, PB5};
const int ENABLE_PIN  = PB8;                    // Active les 4 drivers A4988 simultanement (LOW = actif)

// Vitesse min (demarrage) et max
const unsigned int STEP_DELAY_MIN = 3500;  // vitesse max (delai court)
const unsigned int STEP_DELAY_MAX = 6000;  // vitesse min (demarrage doux)
const unsigned int RAMPE_PAS = 50;         // accelere sur 50 pas

volatile int32_t  stepsRestants[4] = {0, 0, 0, 0};
volatile uint8_t  dirMoteurs[4]    = {0, 0, 0, 0};
unsigned long     dernierStep[4]   = {0, 0, 0, 0};
uint32_t          compteurPas[4]   = {0, 0, 0, 0}; // Compteur pour la rampe

typedef enum {
    STATE_WAIT_HEADER, STATE_MOTOR_ID, STATE_STEPS_L,
    STATE_STEPS_H, STATE_DIR, STATE_CHECKSUM
} UartState;

UartState rxState   = STATE_WAIT_HEADER;
uint8_t rxMotorId   = 0;
uint8_t rxSteps_L   = 0;
uint8_t rxSteps_H   = 0;
uint8_t rxDir       = 0;
uint8_t rxChecksum  = 0;

// Retourne un delai progressif: long au debut, court une fois la rampe terminee
unsigned int calculerDelai(int idx) {
    uint32_t pas = compteurPas[idx];
    if (pas >= RAMPE_PAS) {
        return STEP_DELAY_MIN;  // vitesse max atteinte
    }
    // Interpolation lineaire entre MAX (lent) et MIN (rapide)
    return STEP_DELAY_MAX - ((STEP_DELAY_MAX - STEP_DELAY_MIN) * pas / RAMPE_PAS);
}

// Met a jour l'etat du moteur et configure la direction
void handleMotorCommand(uint8_t motorId, uint16_t steps, uint8_t dir) {
    Serial.printf("[MOTEUR %d] %d steps, dir=%d\n", motorId, steps, dir);
    int idx = motorId - 1;
    
    // Si changement de direction ou nouveau mouvement, reset la rampe
    if (dirMoteurs[idx] != dir || stepsRestants[idx] == 0) {
        compteurPas[idx] = 0;
    }
    
    dirMoteurs[idx]    = dir;
    stepsRestants[idx] = steps;
    digitalWrite(DIR_PINS[idx], dir ? HIGH : LOW);
}

// Avance dans la machine a etats selon l'octet recu
void traiterOctet(uint8_t b) {
    switch (rxState) {
        case STATE_WAIT_HEADER:
            if (b == 0xAA) rxState = STATE_MOTOR_ID; //attente header
            break;
        case STATE_MOTOR_ID: // Attente de l'id du moteur
            if (b >= 1 && b <= 4) {
                rxMotorId = b;
                rxState = STATE_STEPS_L;
            } else {
                Serial.printf("[ERREUR] ID moteur invalide: %d\n", b);
                rxState = STATE_WAIT_HEADER;
            }
            break;
        case STATE_STEPS_L: //octets des nombres de pas
            rxSteps_L = b;
            rxState = STATE_STEPS_H;
            break;
        case STATE_STEPS_H:
            rxSteps_H = b;
            rxState = STATE_DIR;
            break;
        case STATE_DIR: //la direction
            rxDir = b;
            rxState = STATE_CHECKSUM; 
            break;
        case STATE_CHECKSUM: //recalcule du checksum 
            rxChecksum = b;
            // Verification d'integrite par XOR de tous les octets de donnees
            uint8_t checksumCalcule = rxMotorId ^ rxSteps_L ^ rxSteps_H ^ rxDir;
            if (rxChecksum == checksumCalcule) {
                // Trame valide: reconstitue le nombre de pas sur 16 bits
                uint16_t steps = rxSteps_L | ((uint16_t)rxSteps_H << 8);
                handleMotorCommand(rxMotorId, steps, rxDir);
            } else {
                Serial.printf("[ERREUR] Checksum invalide: recu=0x%02X calcule=0x%02X\n",
                    rxChecksum, checksumCalcule);
            }
            // Reset de la machine a etats pour la prochaine trame
            rxState = STATE_WAIT_HEADER;
            rxMotorId = rxSteps_L = rxSteps_H = rxDir = rxChecksum = 0;
            break;
    }
}

// --- INITIALISATION ---
void setup() {
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_AFIO_REMAP_SWJ_NOJTAG();
    delay(100);

    Serial.begin(115200);
    Serial1.begin(115200);
    delay(1000);

    Serial.println("\n=== STM32 - CONTROLEUR MOTEURS (rampe) ===");

    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, LOW);
    
    // Configuration des broches STEP et DIR de chaque moteur
    for (int i = 0; i < 4; i++) {
        pinMode(STEP_PINS[i], OUTPUT); digitalWrite(STEP_PINS[i], LOW);
        pinMode(DIR_PINS[i],  OUTPUT); digitalWrite(DIR_PINS[i],  LOW);
    }

    Serial.println("[OK] 4 moteurs configures");
    Serial.println("[ATTENTE] En attente de commandes...\n");
}

// --- BOUCLE PRINCIPALE ---
void loop() {
    while (Serial1.available()) {
        uint8_t b = Serial1.read();
        traiterOctet(b); //Traite tous les octets disponibles avant de generer des pas
    }
    //Generation des pulses STEP pour chaque moteur
    //Approche non-bloquante: chaque moteur tourne a son rythme
    //selon le delai calcule par la rampe d'acceleration
    unsigned long maintenant = micros();
    for (int i = 0; i < 4; i++) {
        if (stepsRestants[i] > 0) {
            unsigned int delai = calculerDelai(i);
            // Generation d'un pas si assez de temps s'est ecoule
            if ((maintenant - dernierStep[i]) >= delai) {
                dernierStep[i] = maintenant;
                digitalWrite(STEP_PINS[i], HIGH);
                delayMicroseconds(10);
                digitalWrite(STEP_PINS[i], LOW);
                stepsRestants[i]--;
                compteurPas[i]++;
            }
        } else {
            compteurPas[i] = 0;  // reset rampe quand arrete
        }
    }
}