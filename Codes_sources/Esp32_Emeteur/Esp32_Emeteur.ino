/*******************************************************************************
 * ESP32 EMETTEUR - MANETTE DE CONTROLE
 * Projet: Prototype bras continuum
 * Auteur: Jeff Truong
 *
 * Modes:
 * - NORMAL: Joystick X+Y controle 3 moteurs en triangle (continuum arm)
 *           Boutons OPEN/CLOSE = pince
 * - TEST:   Joystick Y fait tourner 1 moteur a la fois
 *           Boutons OPEN/CLOSE = changer moteur
 *
 *           Bouton joystick (clic) = bascule entre les 2 modes
 ******************************************************************************/

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// --- BROCHES ---
#define JOY_X_GPIO      34
#define JOY_Y_GPIO      35
#define JOY_BTN_GPIO    32
#define BTN_OPEN_GPIO   25
#define BTN_CLOSE_GPIO  26

// --- CONFIGURATION JOYSTICK ---
#define JOY_CENTER    2048
#define JOY_DEADZONE  150
#define JOY_MIN       0
#define JOY_MAX       4095
#define MOTOR_SPEED_MAX 10

// --- MAC RECEPTEUR ---
uint8_t receiverMac[] = {0xF4, 0x65, 0x0B, 0xB5, 0x86, 0xD0};

// --- STRUCTURE PAQUET ---
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

ControlPacket packet;
unsigned long dernierEnvoi = 0;
const unsigned long INTERVALLE_ENVOI = 30;
bool connexionOK = false;

// --- ETAT MODES ---
uint8_t modeActuel = 0;        // 0 = NORMAL, 1 = TEST
uint8_t moteurTest = 1;        // Moteur selectionne en mode TEST (1-3)
bool dernierEtatJoyBtn = false;
bool dernierEtatOpen   = false;
bool dernierEtatClose  = false;

// --- UTILITAIRES ---
int appliquerDeadzone(int valeur, int centre, int deadzone) {
    int offset = valeur - centre;
    if (abs(offset) < deadzone) return 0;
    return offset;
}

int8_t adcVersPourcent(int valeurAdc, int centre, int deadzone) {
    int offset = appliquerDeadzone(valeurAdc, centre, deadzone);
    if (offset == 0) return 0;
    if (offset > 0) return map(offset, deadzone, JOY_MAX - centre, 0, 100);
    else            return map(offset, -deadzone, JOY_MIN - centre, 0, -100);
}

uint16_t pourcentVersVitesse(int8_t pourcent) {
    if (pourcent == 0) return 0;
    return map(abs(pourcent), 0, 100, 0, MOTOR_SPEED_MAX);
}

uint8_t calculerChecksum(ControlPacket* pkt) {
    return (pkt->seq + pkt->joyX + pkt->joyY +
            pkt->btnJoy + pkt->btnOpen + pkt->btnClose) & 0xFF;
}

void lireBoutons() {
    bool etatJoyBtn = !digitalRead(JOY_BTN_GPIO);  // pull-up: 1 = appuye
    bool etatOpen   = digitalRead(BTN_OPEN_GPIO);   // pull-down: 1 = appuye
    bool etatClose  = digitalRead(BTN_CLOSE_GPIO);  // pull-down: 1 = appuye

    // Bouton joystick = bascule mode (detection front montant)
    if (etatJoyBtn && !dernierEtatJoyBtn) {
        modeActuel = 1 - modeActuel;
        Serial.printf("\n[MODE] === %s ===\n",
            modeActuel == 0 ? "NORMAL (continuum arm)" : "TEST (1 moteur a la fois)");
        if (modeActuel == 1) {
            Serial.printf("[TEST] Moteur selectionne: %d\n", moteurTest);
        }
    }

    // En mode TEST, OPEN/CLOSE change le moteur
    if (modeActuel == 1) {
        if (etatOpen && !dernierEtatOpen) {
            moteurTest++;
            if (moteurTest > 3) moteurTest = 1;
            Serial.printf("[TEST] Moteur: %d\n", moteurTest);
        }
        if (etatClose && !dernierEtatClose) {
            moteurTest--;
            if (moteurTest < 1) moteurTest = 3;
            Serial.printf("[TEST] Moteur: %d\n", moteurTest);
        }
        // En mode TEST, pas de commande pince
        packet.btnOpen  = 0;
        packet.btnClose = 0;
    } else {
        // Mode NORMAL: boutons = pince
        packet.btnOpen  = etatOpen;
        packet.btnClose = etatClose;
    }

    packet.btnJoy        = etatJoyBtn;
    dernierEtatJoyBtn    = etatJoyBtn;
    dernierEtatOpen      = etatOpen;
    dernierEtatClose     = etatClose;
}

void lireJoystick() {
    int brutX = analogRead(JOY_X_GPIO);
    int brutY = analogRead(JOY_Y_GPIO);

    packet.joyX = adcVersPourcent(brutX, JOY_CENTER, JOY_DEADZONE);
    packet.joyY = adcVersPourcent(brutY, JOY_CENTER, JOY_DEADZONE);

    if (modeActuel == 0) {
        // ========== MODE NORMAL - CONTROLE PAR ZONES ==========
        // 6 zones du joystick selon le triangle:
        // M1 = haut, M2 = bas-gauche, M3 = bas-droite
        //
        // Quand on tire un cable, les 2 autres relachent

        int x = packet.joyX;
        int y = packet.joyY;

        // Vitesse selon distance du centre 
        int intensite = max(abs(x), abs(y));
        uint16_t steps = pourcentVersVitesse(intensite);

        // Par defaut tous les moteurs arretes
        packet.motorM1 = 0; packet.dirM1 = 0;
        packet.motorM2 = 0; packet.dirM2 = 0;
        packet.motorM3 = 0; packet.dirM3 = 0;

        if (x >= -100 && x <= -59 && y >= 1 && y <= 100) {
            // UP_LEFT: M2 tire, M1 tire un peu, M3 relache
            packet.motorM1 = steps; packet.dirM1 = 1;
            packet.motorM2 = steps; packet.dirM2 = 1;
            packet.motorM3 = steps; packet.dirM3 = 0;
        }
        else if (x >= -58 && x <= 58 && y >= 58 && y <= 100) {
            // UP: M1 tire, M2 et M3 relachent
            packet.motorM1 = steps; packet.dirM1 = 1;
            packet.motorM2 = steps; packet.dirM2 = 0;
            packet.motorM3 = steps; packet.dirM3 = 0;
        }
        else if (x >= 59 && x <= 100 && y >= 1 && y <= 100) {
            // UP_RIGHT: M3 tire, M1 tire un peu, M2 relache
            packet.motorM1 = steps; packet.dirM1 = 1;
            packet.motorM2 = steps; packet.dirM2 = 0;
            packet.motorM3 = steps; packet.dirM3 = 1;
        }
        else if (x >= 59 && x <= 100 && y >= -100 && y <= 0) {
            // DOWN_RIGHT: M3 tire, M2 et M1 relachent
            packet.motorM1 = steps; packet.dirM1 = 0;
            packet.motorM2 = steps; packet.dirM2 = 0;
            packet.motorM3 = steps; packet.dirM3 = 1;
        }
        else if (x >= -58 && x <= 58 && y >= -100 && y <= -58) {
            // DOWN: M2 et M3 tirent, M1 relache
            packet.motorM1 = steps; packet.dirM1 = 0;
            packet.motorM2 = steps; packet.dirM2 = 1;
            packet.motorM3 = steps; packet.dirM3 = 1;
        }
        else if (x >= -100 && x <= -59 && y >= -100 && y <= 0) {
            // DOWN_LEFT: M2 tire, M3 et M1 relachent
            packet.motorM1 = steps; packet.dirM1 = 0;
            packet.motorM2 = steps; packet.dirM2 = 1;
            packet.motorM3 = steps; packet.dirM3 = 0;
        }
        // Si dans aucune zone (centre/deadzone) = tous a 0 

    } else {
        // ========== MODE TEST - 1 SEUL MOTEUR ==========
        uint16_t steps = pourcentVersVitesse(packet.joyY);
        uint8_t dir    = (packet.joyY >= 0) ? 1 : 0;

        packet.motorM1 = (moteurTest == 1) ? steps : 0;
        packet.dirM1   = (moteurTest == 1) ? dir : 0;
        packet.motorM2 = (moteurTest == 2) ? steps : 0;
        packet.dirM2   = (moteurTest == 2) ? dir : 0;
        packet.motorM3 = (moteurTest == 3) ? steps : 0;
        packet.dirM3   = (moteurTest == 3) ? dir : 0;
    }
}

// --- CALLBACK ESP-NOW ---
void onEnvoi(const wifi_tx_info_t* info, esp_now_send_status_t statut) {
    connexionOK = (statut == ESP_NOW_SEND_SUCCESS);
    if (!connexionOK) Serial.println("[ERREUR] Echec envoi paquet");
}

// --- SETUP ---
void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n=== MANETTE ESP32 - BRAS ROBOTISE ===");
    Serial.println("Bouton joystick = bascule MODE NORMAL <-> TEST");
    Serial.println("MODE NORMAL: joystick = bras 3 moteurs, boutons = pince");
    Serial.println("MODE TEST:   joystick Y = 1 moteur, boutons = changer moteur\n");

    pinMode(JOY_BTN_GPIO,   INPUT_PULLUP);
    pinMode(BTN_OPEN_GPIO,  INPUT_PULLDOWN);
    pinMode(BTN_CLOSE_GPIO, INPUT_PULLDOWN);

    analogSetAttenuation(ADC_11db);
    analogSetWidth(12);

    delay(500);
    int testX = analogRead(JOY_X_GPIO);
    int testY = analogRead(JOY_Y_GPIO);
    Serial.printf("[INFO] Centre joystick - X: %d | Y: %d (attendu ~%d)\n",
        testX, testY, JOY_CENTER);

    WiFi.mode(WIFI_STA);
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    delay(500);
    Serial.print("[INFO] MAC manette: ");
    Serial.println(WiFi.macAddress());

    if (esp_now_init() != ESP_OK) {
        Serial.println("[ERREUR] Initialisation ESP-NOW echouee");
        while(1) delay(1000);
    }

    esp_now_register_send_cb(onEnvoi);

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, receiverMac, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("[ERREUR] Impossible d'ajouter le recepteur");
        while(1) delay(1000);
    }

    Serial.println("[OK] ESP-NOW initialise");
    memset(&packet, 0, sizeof(packet));

    Serial.println("[OK] Manette prete!");
    Serial.println("[MODE] === NORMAL (continuum arm) ===\n");
}

// --- LOOP ---
void loop() {
    unsigned long maintenant = millis();

    if (maintenant - dernierEnvoi >= INTERVALLE_ENVOI) {
        dernierEnvoi = maintenant;

        packet.seq++;
        lireBoutons();
        lireJoystick();
        packet.checksum = calculerChecksum(&packet);

        esp_now_send(receiverMac, (uint8_t*)&packet, sizeof(packet));

        // Debug si actif
        if (packet.motorM1 > 0 || packet.motorM2 > 0 || packet.motorM3 > 0 ||
            packet.btnOpen || packet.btnClose) {
            if (modeActuel == 0) {
                Serial.printf("[NORMAL] X:%d Y:%d | M1:%d D1:%d | M2:%d D2:%d | M3:%d D3:%d | Pince O:%d C:%d\n",
                    packet.joyX, packet.joyY,
                    packet.motorM1, packet.dirM1,
                    packet.motorM2, packet.dirM2,
                    packet.motorM3, packet.dirM3,
                    packet.btnOpen, packet.btnClose);
            } else {
                Serial.printf("[TEST] Moteur:%d | Y:%d | Steps:%d | Dir:%d\n",
                    moteurTest, packet.joyY,
                    moteurTest == 1 ? packet.motorM1 : moteurTest == 2 ? packet.motorM2 : packet.motorM3,
                    moteurTest == 1 ? packet.dirM1 : moteurTest == 2 ? packet.dirM2 : packet.dirM3);
            }
        }
    }
}