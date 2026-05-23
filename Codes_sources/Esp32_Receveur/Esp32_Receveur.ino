/*******************************************************************************
 * RECEPTEUR ESP32 - Mainboard
 * Projet: Prototype bras continuum
 * Auteur: Jeff Truong
 *
 * Recoit ControlPacket depuis manette et transmet au STM32 via UART
 *
 * UART STM32: GPIO16 (RX) / GPIO17 (TX)
 *
 ******************************************************************************/

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

// --- UART vers STM32 ---
#define STM32_RX_GPIO  16
#define STM32_TX_GPIO  17
#define STM32_BAUD     115200

// --- STRUCTURE PAQUET (identique emetteur) ---
#pragma pack(push, 1)
typedef struct {
    uint8_t  seq;
    int8_t   joyX;
    int8_t   joyY;
    uint8_t  btnJoy;
    uint8_t  btnOpen;
    uint8_t  btnClose;
    uint16_t motorM1;
    uint8_t  dirM1;
    uint16_t motorM2;
    uint8_t  dirM2;
    uint16_t motorM3;
    uint8_t  dirM3;
    uint8_t  checksum;
} ControlPacket;
#pragma pack(pop)

// --- VALIDER CHECKSUM ---
bool validerChecksum(ControlPacket* pkt) {
    uint8_t calcule = (pkt->seq + pkt->joyX + pkt->joyY +
                       pkt->btnJoy + pkt->btnOpen + pkt->btnClose) & 0xFF;
    return calcule == pkt->checksum;
}

// --- ENVOYER COMMANDE MOTEUR AU STM32 ---
void envoyerMoteur(uint8_t motorId, uint16_t steps, uint8_t direction) {
    if (steps == 0) return;
    uint8_t checksum = motorId ^ (steps & 0xFF) ^ ((steps >> 8) & 0xFF) ^ direction;
    Serial2.write(0xAA);
    Serial2.write(motorId);
    Serial2.write((uint8_t)(steps & 0xFF));
    Serial2.write((uint8_t)((steps >> 8) & 0xFF));
    Serial2.write(direction);
    Serial2.write(checksum);
    Serial.printf("[UART] Moteur %d | Steps: %d | Dir: %d\n", motorId, steps, direction);
}

// --- CALLBACK RECEPTION ESP-NOW ---
void onReception(const esp_now_recv_info_t* info, const uint8_t* data, int longueur) {
    if (longueur != sizeof(ControlPacket)) {
        Serial.printf("[ERREUR] Taille paquet incorrecte: %d (attendu: %d)\n",
            longueur, sizeof(ControlPacket));
        return;
    }

    // Controle continuum arm par zones du joystick:
    // - 6 zones definies dans l'emetteur (UP, UP_LEFT, UP_RIGHT, DOWN, DOWN_LEFT, DOWN_RIGHT)
    // - Quand un cable est tire, les 2 autres relachent
    // - M1 = haut, M2 = bas-gauche, M3 = bas-droite
    // Bouton OPEN/CLOSE = pince (moteur 4)

    ControlPacket pkt;
    memcpy(&pkt, data, sizeof(ControlPacket));

    if (!validerChecksum(&pkt)) {
        Serial.println("[ERREUR] Checksum invalide");
        return;
    }

    Serial.printf("[OK] SEQ:%d | X:%d Y:%d | Open:%d Close:%d\n",
        pkt.seq, pkt.joyX, pkt.joyY, pkt.btnOpen, pkt.btnClose);

    if (pkt.motorM1 > 0) envoyerMoteur(1, pkt.motorM1, pkt.dirM1);
    else                 envoyerMoteur(1, 0, 0);

    if (pkt.motorM2 > 0) envoyerMoteur(2, pkt.motorM2, pkt.dirM2);
    else                 envoyerMoteur(2, 0, 0);

    if (pkt.motorM3 > 0) envoyerMoteur(3, pkt.motorM3, pkt.dirM3);
    else                 envoyerMoteur(3, 0, 0);


    if (pkt.btnOpen) {
        envoyerMoteur(4, 30, 1);
    } else if (pkt.btnClose) {
        envoyerMoteur(4, 30, 0);
    }
}

// --- SETUP ---
void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n=== DEMARRAGE RECEPTEUR ESP32 ===");

    Serial2.begin(STM32_BAUD, SERIAL_8N1, STM32_RX_GPIO, STM32_TX_GPIO);
    Serial.println("[OK] UART STM32 initialise (GPIO16/17)");

    WiFi.mode(WIFI_STA);
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    delay(500);
    Serial.print("[INFO] MAC mainboard: ");
    Serial.println(WiFi.macAddress());

    if (esp_now_init() != ESP_OK) {
        Serial.println("[ERREUR] Initialisation ESP-NOW echouee");
        return;
    }

    esp_now_register_recv_cb(onReception);
    Serial.println("[OK] ESP-NOW initialise");
    Serial.println("[ATTENTE] En attente de commandes...\n");
}

// --- LOOP ---
void loop() {
    if (Serial2.available()) {
        Serial.print("[STM32] Reponse: ");
        while (Serial2.available()) {
            Serial.printf("0x%02X ", Serial2.read());
        }
        Serial.println();
    }
}
