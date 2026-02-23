#include <AltSoftSerial.h>
#include <Wire.h>
#include <math.h>

// HC-06
// IMPORTANT (Uno/Nano) : SoftwareSerial utilise les interruptions PCINT.
// Comme on utilise PCINT1_vect pour décoder les encodeurs, SoftwareSerial entre en conflit.
// Solution : AltSoftSerial (n'utilise pas PCINT) avec des pins FIXES :
//   - RX = D8 (reçoit depuis le TXD du HC-06)
//   - TX = D9 (envoie vers le RXD du HC-06)
// Câblage : HC-06 TXD -> D8, HC-06 RXD -> D9 (via diviseur si besoin), GND commun.
AltSoftSerial bt;

// HC-SR04
// D9 est utilisé par AltSoftSerial (TX), donc on déplace TRIG sur une pin libre.
static const uint8_t US_TRIG_PIN = 6;
static const uint8_t US_ECHO_PIN = 10;

// MPU-9250/6500 (I2C)
static const uint8_t IMU_ADDR = 0x68;
static const uint8_t REG_PWR_MGMT_1 = 0x6B;
static const uint8_t REG_GYRO_CONFIG = 0x1B;
static const uint8_t REG_ACCEL_CONFIG = 0x1C;
static const uint8_t REG_ACCEL_XOUT_H = 0x3B;
static const uint8_t REG_GYRO_ZOUT_H = 0x47;
static const float GYRO_SENS_250DPS = 131.0f; // LSB/(deg/s)
static const float ACC_SENS_2G = 16384.0f;    // LSB/g

// Filtrage gyro : deadband + adaptation lente de l'offset quand le robot est immobile.
static const float GYRO_DEADBAND_DPS = 0.8f;
static const float GYRO_OFFSET_ADAPT_ALPHA = 0.0020f;

// Robot pose (reference: depart = (0,0), en mm)
static int32_t poseXmm = 0;
static int32_t poseYmm = 0;
static float yawDeg = 0.0f;
static float gyroZOffset = 0.0f;

// Encodeurs (quadrature) -> odométrie réelle
// Sur Uno/Nano, on utilise les Pin Change Interrupts sur A0..A3 (PCINT8..PCINT11)
static const uint8_t ENC_L_A = A0;
static const uint8_t ENC_L_B = A1;
static const uint8_t ENC_R_A = A2;
static const uint8_t ENC_R_B = A3;

// Paramètres mécaniques (à calibrer sur ton robot chenillé)
static const float WHEEL_DIAMETER_MM = 45.0f; // diamètre roue d'entraînement de chenille
static const float WHEEL_BASE_MM = 110.0f;    // distance entre centres des 2 roues (entraxe)

// Encodeur: 48 "ticks" par tour (valeur constructeur). Selon la façon de compter, la résolution effective peut être x1/x2/x4.
static const int32_t ENC_PPR = 48;
static const int32_t ENC_DECODING = 4; // 4 = quadrature x4 (compte chaque transition). Mets 1 si ENC_PPR est déjà en x4.
static const float MM_PER_COUNT = (3.14159265359f * WHEEL_DIAMETER_MM) / (float)(ENC_PPR * ENC_DECODING);

// Si un côté part à l'envers, passe à 1.
#define ENC_LEFT_INVERT  0
#define ENC_RIGHT_INVERT 0

static volatile int32_t encLeftCount = 0;
static volatile int32_t encRightCount = 0;
static volatile uint8_t encLastLeftState = 0;
static volatile uint8_t encLastRightState = 0;

// NOTE : l'ancienne odométrie à vitesse fixe a été remplacée par les encodeurs.

// Autonomie
static const uint16_t OBSTACLE_MM = 200; // si obstacle < 20 cm
static const uint16_t CLEAR_MM = 260;    // repasse en avant > 26 cm
static const uint16_t AVOID_TURN_MS = 350;
static const uint16_t GOAL_TOL_MM = 120;
static const float GOAL_ANGLE_TOL_DEG = 12.0f;

enum RobotMode : uint8_t {
	MODE_MANUAL = 0,
	MODE_AUTO_EXPLORE = 1,
	MODE_AUTO_GOTO = 2
};

static RobotMode mode = MODE_MANUAL;
static bool streamEnabled = false;
static char currentMotion = 'S';

static int32_t goalXmm = 0;
static int32_t goalYmm = 0;
static bool hasGoal = false;

static unsigned long lastImuMs = 0;
static unsigned long lastPoseMs = 0;
static unsigned long lastTelemMs = 0;
static unsigned long lastDistMs = 0;
static uint16_t lastDistMm = 0;
static unsigned long lastEchoUs = 0;
static const uint16_t DIST_INTERVAL_MS = 90;

// Filtrage distance : ignore les zéros / glitches isolés.
static const uint16_t DIST_MIN_MM = 25;
static const uint16_t DIST_MAX_MM = 6000;
static const uint16_t DIST_MAX_JUMP_MM = 1200; // saute irréaliste en 90ms -> probablement bruit
static const uint8_t DIST_INVALID_STREAK_FOR_ZERO = 3;
static uint16_t lastDistGoodMm = 0;
static uint8_t distInvalidStreak = 0;
static unsigned long avoidUntilMs = 0;
static bool avoiding = false;

static inline float wrapAngleDeg(float deg) {
	while (deg <= -180.0f) deg += 360.0f;
	while (deg > 180.0f) deg -= 360.0f;
	return deg;
}

static inline float angleDiffDeg(float target, float current) {
	return wrapAngleDeg(target - current);
}

static inline uint8_t readEnc2Bits(uint8_t pinA, uint8_t pinB) {
	uint8_t a = (uint8_t)(digitalRead(pinA) ? 1 : 0);
	uint8_t b = (uint8_t)(digitalRead(pinB) ? 1 : 0);
	return (uint8_t)((a << 1) | b);
}

static inline int8_t quadDelta(uint8_t prev, uint8_t curr) {
	// Table quadrature (prev<<2 | curr) => -1,0,+1
	// 00->01 +1, 01->11 +1, 11->10 +1, 10->00 +1
	// inverse => -1
	static const int8_t table[16] = {
		0,  +1, -1,  0,
		-1,  0,  0, +1,
		+1,  0,  0, -1,
		0,  -1, +1,  0
	};
	return table[(prev << 2) | curr];
}

static inline int32_t iabs32(int32_t v) { return v < 0 ? -v : v; }

static inline int32_t iroundf(float x) {
	return (int32_t)(x >= 0.0f ? (x + 0.5f) : (x - 0.5f));
}

ISR(PCINT1_vect) {
	// Port C = A0..A5, on lit A0..A3
	uint8_t pinc = PINC;
	uint8_t la = (pinc & _BV(PC0)) ? 1 : 0;
	uint8_t lb = (pinc & _BV(PC1)) ? 1 : 0;
	uint8_t ra = (pinc & _BV(PC2)) ? 1 : 0;
	uint8_t rb = (pinc & _BV(PC3)) ? 1 : 0;
	uint8_t left = (uint8_t)((la << 1) | lb);
	uint8_t right = (uint8_t)((ra << 1) | rb);

	int8_t dl = quadDelta(encLastLeftState, left);
	int8_t dr = quadDelta(encLastRightState, right);
	encLastLeftState = left;
	encLastRightState = right;
	encLeftCount += (ENC_LEFT_INVERT ? -dl : dl);
	encRightCount += (ENC_RIGHT_INVERT ? -dr : dr);
}

static void i2cWrite8(uint8_t addr, uint8_t reg, uint8_t value) {
	Wire.beginTransmission(addr);
	Wire.write(reg);
	Wire.write(value);
	Wire.endTransmission(true);
}

static bool i2cReadBytes(uint8_t addr, uint8_t startReg, uint8_t *buf, uint8_t len) {
	Wire.beginTransmission(addr);
	Wire.write(startReg);
	if (Wire.endTransmission(false) != 0) {
		return false;
	}
	uint8_t read = Wire.requestFrom(addr, len, (uint8_t)true);
	if (read != len) {
		return false;
	}
	for (uint8_t i = 0; i < len; i++) {
		buf[i] = Wire.read();
	}
	return true;
}

static bool imuReadAccelRaw(int16_t *ax, int16_t *ay, int16_t *az) {
	uint8_t b[6];
	if (!i2cReadBytes(IMU_ADDR, REG_ACCEL_XOUT_H, b, 6)) {
		*ax = 0;
		*ay = 0;
		*az = 0;
		return false;
	}
	*ax = (int16_t)((b[0] << 8) | b[1]);
	*ay = (int16_t)((b[2] << 8) | b[3]);
	*az = (int16_t)((b[4] << 8) | b[5]);
	return true;
}

static inline int16_t accelRawToMg(int16_t raw) {
	// mg = raw / 16384 * 1000
	return (int16_t)((int32_t)raw * 1000L / (int32_t)ACC_SENS_2G);
}

static int16_t imuReadGyroZRaw() {
	uint8_t b[2];
	if (!i2cReadBytes(IMU_ADDR, REG_GYRO_ZOUT_H, b, 2)) {
		return 0;
	}
	return (int16_t)((b[0] << 8) | b[1]);
}

static void imuInitAndCalibrate() {
	Wire.begin();
	i2cWrite8(IMU_ADDR, REG_PWR_MGMT_1, 0x00); // wake
	delay(50);
	i2cWrite8(IMU_ADDR, REG_GYRO_CONFIG, 0x00); // +-250 dps
	i2cWrite8(IMU_ADDR, REG_ACCEL_CONFIG, 0x00); // +-2g
	delay(10);

	// Calibration offset gyro Z (robot immobile)
	const uint16_t samples = 600;
	int32_t sum = 0;
	for (uint16_t i = 0; i < samples; i++) {
		sum += imuReadGyroZRaw();
		delay(3);
	}
	gyroZOffset = (float)sum / (float)samples;

	lastImuMs = millis();
}

static void encodersInit() {
	pinMode(ENC_L_A, INPUT_PULLUP);
	pinMode(ENC_L_B, INPUT_PULLUP);
	pinMode(ENC_R_A, INPUT_PULLUP);
	pinMode(ENC_R_B, INPUT_PULLUP);

	noInterrupts();
	encLeftCount = 0;
	encRightCount = 0;
	encLastLeftState = readEnc2Bits(ENC_L_A, ENC_L_B);
	encLastRightState = readEnc2Bits(ENC_R_A, ENC_R_B);

	// Active PCINT pour le Port C (A0..A5)
	PCICR |= _BV(PCIE1);
	// A0..A3 => PCINT8..PCINT11
	PCMSK1 |= _BV(PCINT8) | _BV(PCINT9) | _BV(PCINT10) | _BV(PCINT11);
	interrupts();
}

static void imuUpdateYaw() {
	unsigned long now = millis();
	unsigned long dtMs = now - lastImuMs;
	if (dtMs == 0) return;
	lastImuMs = now;

	int16_t raw = imuReadGyroZRaw();
	float dps = ((float)raw - gyroZOffset) / GYRO_SENS_250DPS;
	if (fabs(dps) < GYRO_DEADBAND_DPS) dps = 0.0f;

	// Si on est à l'arrêt et que le gyro est proche de 0, on ré-ajuste très lentement l'offset.
	// Ça limite la dérive (température/offset résiduel) sans perturber les vrais mouvements.
	if (currentMotion == 'S' && dps == 0.0f) {
		gyroZOffset = gyroZOffset * (1.0f - GYRO_OFFSET_ADAPT_ALPHA) + (float)raw * GYRO_OFFSET_ADAPT_ALPHA;
	}
	yawDeg += dps * ((float)dtMs / 1000.0f);
	yawDeg = wrapAngleDeg(yawDeg);
}

static uint16_t readDistanceMm() {
	// Trigger 10us
	digitalWrite(US_TRIG_PIN, LOW);
	delayMicroseconds(2);
	digitalWrite(US_TRIG_PIN, HIGH);
	delayMicroseconds(10);
	digitalWrite(US_TRIG_PIN, LOW);

	// Timeout ~30ms -> ~5m
	unsigned long dur = pulseIn(US_ECHO_PIN, HIGH, 30000UL);
	lastEchoUs = dur;
	if (dur == 0) return 0;
	// us to mm: cm = us/58, mm = cm*10
	uint32_t mm = (uint32_t)(dur * 10UL) / 58UL;
	if (mm > DIST_MAX_MM) mm = DIST_MAX_MM;
	return (uint16_t)mm;
}

static uint16_t filterDistanceMm(uint16_t rawMm) {
	// Valeur manifestement invalide
	bool valid = (rawMm >= DIST_MIN_MM && rawMm <= DIST_MAX_MM);

	// Glitch : saut énorme par rapport à la dernière bonne mesure
	if (valid && lastDistGoodMm > 0) {
		uint16_t diff = (rawMm > lastDistGoodMm) ? (rawMm - lastDistGoodMm) : (lastDistGoodMm - rawMm);
		if (diff > DIST_MAX_JUMP_MM) {
			valid = false;
		}
	}

	if (!valid) {
		distInvalidStreak++;
		// Ignore les valeurs erronées isolées : on conserve la dernière bonne mesure.
		if (lastDistGoodMm > 0 && distInvalidStreak < DIST_INVALID_STREAK_FOR_ZERO) {
			return lastDistGoodMm;
		}
		return 0;
	}

	// Mesure valide
	distInvalidStreak = 0;
	lastDistGoodMm = rawMm;
	return rawMm;
}


static void updatePose() {
	unsigned long now = millis();
	unsigned long dtMs = now - lastPoseMs;
	if (dtMs == 0) return;
	lastPoseMs = now;

	// Lecture des compteurs encodeurs
	static int32_t lastL = 0;
	static int32_t lastR = 0;

	int32_t l, r;
	noInterrupts();
	l = encLeftCount;
	r = encRightCount;
	interrupts();

	int32_t dL = l - lastL;
	int32_t dR = r - lastR;
	lastL = l;
	lastR = r;

	// Si on est à l'arrêt, on ignore les micro variations (bruit) pour éviter une dérive X/Y.
	// IMPORTANT : ne pas ignorer les 1-tick quand on roule, sinon X/Y peut rester bloqué à 0 si l'encodeur est lent.
	if (currentMotion == 'S') {
		return;
	}
	if (dL == 0 && dR == 0)
		return;

	float leftMm = (float)dL * MM_PER_COUNT;
	float rightMm = (float)dR * MM_PER_COUNT;
	float ds = (leftMm + rightMm) * 0.5f;

	// On utilise le yaw gyro comme cap (meilleure stabilité qu'un yaw purement odométrique sur chenilles).
	float yawRad = yawDeg * 0.01745329252f;
	poseXmm += iroundf(ds * cos(yawRad));
	poseYmm += iroundf(ds * sin(yawRad));
}

static void sendTelemetry(uint16_t distMm) {
	// Format: T,<ms>,<x_mm>,<y_mm>,<yaw_cdeg>,<dist_mm>,<mode>,<motion>,<ax_mg>,<ay_mg>,<az_mg>,<encL>,<encR>\n
	int16_t axRaw, ayRaw, azRaw;
	imuReadAccelRaw(&axRaw, &ayRaw, &azRaw);
	int16_t axMg = accelRawToMg(axRaw);
	int16_t ayMg = accelRawToMg(ayRaw);
	int16_t azMg = accelRawToMg(azRaw);

	int32_t l, r;
	noInterrupts();
	l = encLeftCount;
	r = encRightCount;
	interrupts();

	bt.print('T');
	bt.print(',');
	bt.print(millis());
	bt.print(',');
	bt.print(poseXmm);
	bt.print(',');
	bt.print(poseYmm);
	bt.print(',');
	bt.print((int32_t)(yawDeg * 100.0f));
	bt.print(',');
	bt.print(distMm);
	bt.print(',');
	bt.print((int)mode);
	bt.print(',');
	bt.print(currentMotion);
	bt.print(',');
	bt.print(axMg);
	bt.print(',');
	bt.print(ayMg);
	bt.print(',');
	bt.print(azMg);
	bt.print(',');
	bt.print(l);
	bt.print(',');
	bt.print(r);
	bt.print('\n');
}

static void setMotion(char m);

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

static void setMotion(char m) {
	currentMotion = m;
	switch (m) {
		case 'F':
		case 'f':
			driveForward();
			break;
		case 'B':
		case 'b':
			driveBackward();
			break;
		case 'L':
		case 'l':
			turnLeft();
			break;
		case 'R':
		case 'r':
			turnRight();
			break;
		case 'S':
		case 's':
		default:
			driveStop();
			currentMotion = 'S';
			break;
	}
}

static inline void handleCommand(char c) {
	switch (c) {
		case 'F':
		case 'f':
			mode = MODE_MANUAL;
			hasGoal = false;
			avoiding = false;
			setMotion('F');
			break;
		case 'B':
		case 'b':
			mode = MODE_MANUAL;
			hasGoal = false;
			avoiding = false;
			setMotion('B');
			break;
		case 'S':
		case 's':
			mode = MODE_MANUAL;
			hasGoal = false;
			avoiding = false;
			setMotion('S');
			break;
		case 'L':
		case 'l':
			mode = MODE_MANUAL;
			hasGoal = false;
			avoiding = false;
			setMotion('L');
			break;
		case 'R':
		case 'r':
			mode = MODE_MANUAL;
			hasGoal = false;
			avoiding = false;
			setMotion('R');
			break;
		default:
			// ignore
			break;
	}
}

static void stopAll() {
	mode = MODE_MANUAL;
	hasGoal = false;
	avoiding = false;
	avoidUntilMs = 0;
	streamEnabled = false;
	setMotion('S');
}

static bool parseInt32(const char *s, int32_t *out) {
	char *endPtr = nullptr;
	long v = strtol(s, &endPtr, 10);
	if (endPtr == s) return false;
	*out = (int32_t)v;
	return true;
}

static void processLine(char *line) {
	// trim leading spaces
	while (*line == ' ' || *line == '\t') line++;
	if (*line == 0) return;

	// Commande = premier caractere (fonctionne aussi pour les commandes avec virgules)
	char cmd = line[0];

	// X = arrêt global : retour état d'attente (manuel + stop + stream off + reset auto)
	if (cmd == 'X' || cmd == 'x') {
		stopAll();
		bt.print(F("OK,X\n"));
		return;
	}

	// Legacy manual commands
	if (cmd == 'F' || cmd == 'f' || cmd == 'B' || cmd == 'b' || cmd == 'S' || cmd == 's' || cmd == 'L' || cmd == 'l' || cmd == 'R' || cmd == 'r') {
		handleCommand(cmd);
		return;
	}

	// A = toggle auto explore
	if (cmd == 'A' || cmd == 'a') {
		// Optionnel : A,0 / A,1 pour forcer un état (évite les désync côté PC)
		char *comma = strchr(line, ',');
		if (comma) {
			int32_t v;
			if (!parseInt32(comma + 1, &v)) return;
			if (v <= 0) {
				mode = MODE_MANUAL;
				hasGoal = false;
				avoiding = false;
				avoidUntilMs = 0;
				setMotion('S');
			} else {
				mode = MODE_AUTO_EXPLORE;
				hasGoal = false;
				avoiding = false;
				avoidUntilMs = 0;
				setMotion('F');
			}
			bt.print(F("OK,A,"));
			bt.print(mode == MODE_AUTO_EXPLORE ? 1 : 0);
			bt.print('\n');
			return;
		}

		if (mode == MODE_AUTO_EXPLORE) {
			mode = MODE_MANUAL;
			avoiding = false;
			setMotion('S');
		} else {
			mode = MODE_AUTO_EXPLORE;
			hasGoal = false;
			avoiding = false;
			avoidUntilMs = 0;
			setMotion('F');
		}
		bt.print(F("OK,A,"));
		bt.print(mode == MODE_AUTO_EXPLORE ? 1 : 0);
		bt.print('\n');
		return;
	}

	// M = toggle streaming telemetry
	if (cmd == 'M' || cmd == 'm') {
		// Optionnel : M,0 / M,1 pour forcer un état (évite les désync côté PC)
		char *comma = strchr(line, ',');
		if (comma) {
			int32_t v;
			if (!parseInt32(comma + 1, &v)) return;
			streamEnabled = (v > 0);
			bt.print(F("OK,M,"));
			bt.print(streamEnabled ? 1 : 0);
			bt.print('\n');
			return;
		}

		streamEnabled = !streamEnabled;
		bt.print(F("OK,M,"));
		bt.print(streamEnabled ? 1 : 0);
		bt.print('\n');
		return;
	}

	// P = one-shot distance
	if (cmd == 'P' || cmd == 'p') {
		uint16_t d = filterDistanceMm(readDistanceMm());
		bt.print('U');
		bt.print(',');
		bt.print(lastEchoUs);
		bt.print('\n');
		bt.print('D');
		bt.print(',');
		bt.print(d);
		bt.print('\n');
		return;
	}

	// V,<mm_per_s> : set linear speed estimate
	if (cmd == 'V' || cmd == 'v') {
		// Commande conservée pour compatibilité (ancienne odométrie). Maintenant ignorée.
		bt.print(F("OK,V,IGNORED"));
		bt.print('\n');
		return;
	}

	// G,<x_mm>,<y_mm> : set goal (world coordinates, mm)
	if (cmd == 'G' || cmd == 'g') {
		char *p1 = strchr(line, ',');
		if (!p1) return;
		char *p2 = strchr(p1 + 1, ',');
		if (!p2) return;
		*p2 = 0;
		int32_t gx, gy;
		if (!parseInt32(p1 + 1, &gx)) return;
		if (!parseInt32(p2 + 1, &gy)) return;

		goalXmm = gx;
		goalYmm = gy;
		hasGoal = true;
		mode = MODE_AUTO_GOTO;
		avoiding = false;
		setMotion('S');
		bt.print(F("OK,G,"));
		bt.print(goalXmm);
		bt.print(',');
		bt.print(goalYmm);
		bt.print('\n');
		return;
	}
}

static void autoExploreStep(uint16_t distMm) {
	unsigned long now = millis();
	if (!avoiding) {
		if (distMm > 0 && distMm < OBSTACLE_MM) {
			avoiding = true;
			avoidUntilMs = now + AVOID_TURN_MS;
			setMotion('R');
			return;
		}
		setMotion('F');
		return;
	}

	// avoiding
	if (now >= avoidUntilMs && (distMm == 0 || distMm > CLEAR_MM)) {
		avoiding = false;
		setMotion('F');
		return;
	}
	setMotion('R');
}

static void autoGotoStep(uint16_t distMm) {
	if (!hasGoal) {
		mode = MODE_MANUAL;
		setMotion('S');
		return;
	}

	// basic obstacle avoidance
	if (distMm > 0 && distMm < OBSTACLE_MM) {
		avoiding = true;
		avoidUntilMs = millis() + AVOID_TURN_MS;
		setMotion('R');
		return;
	}
	if (avoiding) {
		if (millis() >= avoidUntilMs && (distMm == 0 || distMm > CLEAR_MM)) {
			avoiding = false;
		} else {
			setMotion('R');
			return;
		}
	}

	int32_t dx = goalXmm - poseXmm;
	int32_t dy = goalYmm - poseYmm;
	float dist = sqrt((float)dx * (float)dx + (float)dy * (float)dy);
	if (dist < (float)GOAL_TOL_MM) {
		setMotion('S');
		mode = MODE_MANUAL;
		hasGoal = false;
		return;
	}

	float desired = atan2((float)dy, (float)dx) * 57.2957795f;
	float err = angleDiffDeg(desired, yawDeg);
	if (fabs(err) > GOAL_ANGLE_TOL_DEG) {
		// choose turn direction
		if (err > 0) setMotion('L');
		else setMotion('R');
		return;
	}
	setMotion('F');
}

void setup() {
	pinMode(LEFT_IN1, OUTPUT);
	pinMode(LEFT_IN2, OUTPUT);
	pinMode(RIGHT_IN3, OUTPUT);
	pinMode(RIGHT_IN4, OUTPUT);

	pinMode(US_TRIG_PIN, OUTPUT);
	pinMode(US_ECHO_PIN, INPUT);
	digitalWrite(US_TRIG_PIN, LOW);

	driveStop();
	currentMotion = 'S';

	imuInitAndCalibrate();
	encodersInit();
	lastPoseMs = millis();
	lastTelemMs = millis();
	lastDistMs = millis();
	lastDistMm = 0;
	lastDistGoodMm = 0;
	distInvalidStreak = 0;

	bt.begin(9600); // HC-06 est typiquement en 9600 par defaut
}

void loop() {
	static char lastMotionCmd = 'S';
	static char lineBuf[64];
	static uint8_t lineLen = 0;

	imuUpdateYaw();
	updatePose();

	unsigned long now = millis();
	if (now - lastDistMs >= DIST_INTERVAL_MS) {
		lastDistMm = filterDistanceMm(readDistanceMm());
		lastDistMs = now;
	}
	uint16_t distMm = lastDistMm;

	if (mode == MODE_AUTO_EXPLORE) {
		autoExploreStep(distMm);
	} else if (mode == MODE_AUTO_GOTO) {
		autoGotoStep(distMm);
	}

	if (streamEnabled) {
		if (now - lastTelemMs >= 120) {
			sendTelemetry(distMm);
			lastTelemMs = now;
		}
	}

	while (bt.available() > 0) {
		char c = (char)bt.read();

		// On ignore les fins de ligne au cas ou l'appli envoie \r/\n
		if (c == '\r' || c == '\n' || c == ' ') {
			if (c == '\r' || c == '\n') {
				if (lineLen > 0) {
					lineBuf[lineLen] = 0;
					processLine(lineBuf);
					lineLen = 0;
				}
			}
			continue;
		}


		// Beaucoup d'apps envoient la meme lettre en continu pendant l'appui.
		// On ne filtre les doublons QUE pour les commandes de mouvement.
		// Important : ne pas filtrer A/M (toggle), sinon on ne peut plus les stopper.
		char cu = (char)toupper((unsigned char)c);
		if ((cu == 'F' || cu == 'B' || cu == 'S' || cu == 'L' || cu == 'R') && cu == lastMotionCmd) {
			continue;
		}

		// Accumule en ligne pour commandes avec parametres.
		if (lineLen < (sizeof(lineBuf) - 1)) {
			lineBuf[lineLen++] = c;
		}

		// Si c'est une commande simple (F/B/S/L/R/A/M/P) envoyee seule, on traite direct.
		// IMPORTANT : ne pas traiter A/M/P en direct, car l'appli PC envoie souvent A,0|1 et M,0|1.
		// Si on traite 'M' dès le 1er caractère, on casse la commande 'M,1' (le reste ",1" devient une ligne invalide).
		// On garde le traitement direct uniquement pour les commandes de mouvement.
		if (lineLen == 1 && (c == 'F' || c == 'f' || c == 'B' || c == 'b' || c == 'S' || c == 's' || c == 'L' || c == 'l' || c == 'R' || c == 'r')) {
			lineBuf[1] = 0;
			processLine(lineBuf);
			lineLen = 0;
			if (cu == 'F' || cu == 'B' || cu == 'S' || cu == 'L' || cu == 'R')
				lastMotionCmd = cu;
		}
	}
}

