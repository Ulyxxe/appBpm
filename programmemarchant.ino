#include "fontData.h"  // Inclut la matrice de caractères

#define SIGNAL_PIN         A4      // Broche analogique d'entrée
#define THRESHOLD          2580    // Seuil de détection (filtré)
#define DEBOUNCE_TIME_MS   500     // Intervalle anti-rebond (à adapter)

// Paramètres pour la moyenne glissante
#define FILTER_WINDOW_SIZE 10      
int   samples[FILTER_WINDOW_SIZE];
long  sumSamples    = 0;
int   sampleIndex   = 0;

// Fenêtre de calcul de la fréquence moyenne
#define AVERAGING_WINDOW_MS 30000  // 60 secondes (modifiez selon vos besoins)
unsigned long startWindowTime = 0; // Temps de début de la fenêtre
int peakCount                = 0;  // Nombre de pics détectés sur la fenêtre

bool isPeakDetected          = false;
unsigned long lastPeakTime   = 0;  // Pour le debounce

// Initialisation pour l'écran OLED
void InitI2C(void);                // Déclaration de la fonction d'initialisation I2C
void InitScreen(void);             // Déclaration de la fonction d'initialisation écran OLED
void DisplayString(int x, int y, char *s);  // Affichage d'une chaîne sur l'écran

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Demarrage : Filtrage + Frequence moyenne sur 30 secondes");

  // Configuration ADC (dépend du support Energia / Tiva, 12 bits souvent possible)
  analogReadResolution(12);
  pinMode(SIGNAL_PIN, INPUT);

  // Initialiser le buffer pour la moyenne glissante
  for (int i = 0; i < FILTER_WINDOW_SIZE; i++) {
    samples[i] = 0;
    sumSamples += 0;
  }

  // Initialiser le temps de début de la fenêtre
  startWindowTime = millis();

  // Initialisation de l'I2C et de l'écran OLED
  InitI2C();
  InitScreen();
}

void loop() {
  // 1) Lecture brute du signal
  int newSample = analogRead(SIGNAL_PIN);

  // 2) Mise à jour de la somme et du tableau (moyenne glissante)
  sumSamples -= samples[sampleIndex];
  samples[sampleIndex] = newSample;
  sumSamples += newSample;

  // Calcul de la moyenne glissante
  int filteredValue = sumSamples / FILTER_WINDOW_SIZE;

  // Gestion de l'index circulaire
  sampleIndex = (sampleIndex + 1) % FILTER_WINDOW_SIZE;

  // 3) Détection de pics sur le signal filtré
  unsigned long currentTime = millis();

  // Front montant : on dépasse le THRESHOLD
  if (!isPeakDetected && (filteredValue > THRESHOLD)) {
    // Vérifier la durée depuis le dernier pic pour éviter la double détection
    if ((currentTime - lastPeakTime) > DEBOUNCE_TIME_MS) {
      isPeakDetected = true;
    }
  }

  // Front descendant : on repasse en-dessous du seuil => valider le pic
  if (isPeakDetected && (filteredValue < THRESHOLD)) {
    isPeakDetected = false;
    lastPeakTime   = currentTime;
    peakCount++;  // Incrémenter le nombre de pics dans la fenêtre
  }

  // 4) Vérifier si la fenêtre de X secondes est écoulée
  if ((currentTime - startWindowTime) >= AVERAGING_WINDOW_MS) {
    // Calcul du nombre de secondes écoulées dans la fenêtre
    float elapsedSec = (float)(currentTime - startWindowTime) / 1000.0;

    // Calcul de la fréquence moyenne en Hz = (nb de pics) / (temps en secondes)
    float freqHz = 0.0;
    if (elapsedSec > 0) {
      freqHz = (float)peakCount / elapsedSec;
    }

    // Conversion en BPM (Battements par minute) si nécessaire
    float bpm = freqHz * 60.0;

    // 5) Affichage des résultats sur le Moniteur Série
    Serial.print("Fenetre = ");
    Serial.print(elapsedSec);
    Serial.print(" s,  Pics = ");
    Serial.print(peakCount);
    Serial.print(", Frequence moyenne = ");
    Serial.print(freqHz);
    Serial.print(" Hz (");
    Serial.print(bpm);
    Serial.println(" BPM)");

    // Affichage sur l'écran OLED
    char bpmStr[16];
    sprintf(bpmStr, "BPM: %.1f", bpm);  // Conversion du BPM en chaîne de caractères
    DisplayString(10, 3, bpmStr);        // Affichage de la chaîne sur l'écran

    // 6) Réinitialiser la fenêtre
    peakCount       = 0;
    startWindowTime = currentTime;
  }

  // Petit délai pour réguler l'échantillonnage (ajustez selon vos besoins)
  delay(2);
}
