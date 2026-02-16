#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SoftwareSerial.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

const uint8_t BT_RX = 2;
const uint8_t BT_TX = 3;
SoftwareSerial btSerial(BT_RX, BT_TX);

String inputBuffer = "";
String lastLine = "";
unsigned long lastRxMs = 0;
unsigned long lastRefresh = 0;
const unsigned long REFRESH_INTERVAL = 100;
const unsigned long IDLE_TIMEOUT_MS = 250;

void setup() {
    btSerial.begin(9600);
    
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        while (1);
    }
    
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Affichage serie");
    display.println("En attente...");
    display.display();

    lastRxMs = millis();
}

void loop() {
    while (btSerial.available()) {
        char data = (char)btSerial.read();
        lastRxMs = millis();

        if (data == '\n' || data == '\r') {
            if (inputBuffer.length() > 0) {
                lastLine = inputBuffer;
                inputBuffer = "";
                updateDisplay();
            }
        } else {
            inputBuffer += data;
            updateDisplay();
        }
    }

    if (inputBuffer.length() > 0 && (millis() - lastRxMs) > IDLE_TIMEOUT_MS) {
        lastLine = inputBuffer;
        inputBuffer = "";
        updateDisplay();
    }
    
    if (millis() - lastRefresh > REFRESH_INTERVAL) {
        updateDisplay();
        lastRefresh = millis();
    }
}

void updateDisplay() {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Recu:");
    display.println("");

    if (inputBuffer.length() > 0) {
        display.println(inputBuffer);
    } else {
        display.println(lastLine);
    }
    display.display();
}