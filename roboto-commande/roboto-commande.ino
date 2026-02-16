
#include <SoftwareSerial.h>

// HC-06
// SoftwareSerial(rxPin, txPin) : rxPin = pin qui RECOIT (Arduino RX)
static const uint8_t BT_RX_PIN = 7; // relie au TXD du HC-06
static const uint8_t BT_TX_PIN = 8; // relie au RXD du HC-06
SoftwareSerial bt(BT_RX_PIN, BT_TX_PIN);

// L298N (IN1..IN4)
// Si ton module est branche en ordre D2,D3,D4,D5 sur IN1,IN2,IN3,IN4, garde ca:
//   IN1=D2, IN2=D3, IN3=D4, IN4=D5
// Ensuite, ajuste seulement LEFT_INVERT / RIGHT_INVERT si un moteur tourne a l'envers.
static const uint8_t LEFT_IN1  = 2;
static const uint8_t LEFT_IN2  = 3;
static const uint8_t RIGHT_IN3 = 4;
static const uint8_t RIGHT_IN4 = 5;

// 0 = sens normal, 1 = inverse le sens de ce moteur
#define LEFT_INVERT  1
#define RIGHT_INVERT 0

static inline void leftStop() {
	digitalWrite(LEFT_IN1, LOW);
	digitalWrite(LEFT_IN2, LOW);
}

static inline void leftForward() {
	if (LEFT_INVERT) {
		digitalWrite(LEFT_IN1, LOW);
		digitalWrite(LEFT_IN2, HIGH);
	} else {
		digitalWrite(LEFT_IN1, HIGH);
		digitalWrite(LEFT_IN2, LOW);
	}
}

static inline void leftBackward() {
	if (LEFT_INVERT) {
		digitalWrite(LEFT_IN1, HIGH);
		digitalWrite(LEFT_IN2, LOW);
	} else {
		digitalWrite(LEFT_IN1, LOW);
		digitalWrite(LEFT_IN2, HIGH);
	}
}

static inline void rightStop() {
	digitalWrite(RIGHT_IN3, LOW);
	digitalWrite(RIGHT_IN4, LOW);
}

static inline void rightForward() {
	if (RIGHT_INVERT) {
		digitalWrite(RIGHT_IN3, LOW);
		digitalWrite(RIGHT_IN4, HIGH);
	} else {
		digitalWrite(RIGHT_IN3, HIGH);
		digitalWrite(RIGHT_IN4, LOW);
	}
}

static inline void rightBackward() {
	if (RIGHT_INVERT) {
		digitalWrite(RIGHT_IN3, HIGH);
		digitalWrite(RIGHT_IN4, LOW);
	} else {
		digitalWrite(RIGHT_IN3, LOW);
		digitalWrite(RIGHT_IN4, HIGH);
	}
}

static inline void driveStop() {
	leftStop();
	rightStop();
}

static inline void driveForward() {
	leftForward();
	rightForward();
}

static inline void driveBackward() {
	leftBackward();
	rightBackward();
}

// Virage rapide (pivot sur place) :
// L = gauche recule, droite avance
// R = droite recule, gauche avance
static inline void turnLeft() {
	leftBackward();
	rightForward();
}

static inline void turnRight() {
	rightBackward();
	leftForward();
}

static inline void handleCommand(char c) {
	switch (c) {
		case 'F':
		case 'f':
			driveForward();
			break;
		case 'B':
		case 'b':
			driveBackward();
			break;
		case 'S':
		case 's':
			driveStop();
			break;
		case 'L':
		case 'l':
			turnLeft();
			break;
		case 'R':
		case 'r':
			turnRight();
			break;
		default:
			// ignore
			break;
	}
}

void setup() {
	pinMode(LEFT_IN1, OUTPUT);
	pinMode(LEFT_IN2, OUTPUT);
	pinMode(RIGHT_IN3, OUTPUT);
	pinMode(RIGHT_IN4, OUTPUT);

	driveStop();

	bt.begin(9600); // HC-06 est typiquement en 9600 par defaut
}

void loop() {
	static char lastCmd = '\0';

	while (bt.available() > 0) {
		char c = (char)bt.read();

		// On ignore les fins de ligne au cas ou l'appli envoie \r/\n
		if (c == '\r' || c == '\n' || c == ' ') {
			continue;
		}

		// Beaucoup d'apps envoient la meme lettre en continu pendant l'appui.
		// Re-appliquer la meme commande ne change rien, donc on ignore les doublons.
		if (c == lastCmd) {
			continue;
		}

		handleCommand(c);
		lastCmd = c;
	}
}

