
#include <SoftwareSerial.h>
#include <Wire.h>
#include <math.h>

// HC-06
// SoftwareSerial(rxPin, txPin) : rxPin = pin qui RECOIT (Arduino RX)
static const uint8_t BT_RX_PIN = 7; // relie au TXD du HC-06
static const uint8_t BT_TX_PIN = 8; // relie au RXD du HC-06
SoftwareSerial bt(BT_RX_PIN, BT_TX_PIN);

// HC-SR04
static const uint8_t US_TRIG_PIN = 9;
static const uint8_t US_ECHO_PIN = 10;

// MPU-9250/6500 (I2C)
static const uint8_t IMU_ADDR = 0x68;
static const uint8_t REG_PWR_MGMT_1 = 0x6B;
static const uint8_t REG_GYRO_CONFIG = 0x1B;
static const uint8_t REG_GYRO_ZOUT_H = 0x47;
static const float GYRO_SENS_250DPS = 131.0f; // LSB/(deg/s)

// Filtrage gyro : deadband + adaptation lente de l'offset quand le robot est immobile.
static const float GYRO_DEADBAND_DPS = 0.8f;
static const float GYRO_OFFSET_ADAPT_ALPHA = 0.0020f;

// Robot pose (reference: depart = (0,0), en mm)
static int32_t poseXmm = 0;
static int32_t poseYmm = 0;
static float yawDeg = 0.0f;
static float gyroZOffset = 0.0f;

// Vitesse lineaire (a calibrer)
// Sans encodeurs, c'est une estimation. Ajuste cette valeur pour que la map ait une echelle correcte.
static int32_t LINEAR_SPEED_MM_PER_S = 120; // ~12 cm/s par defaut

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

	if (currentMotion == 'F' || currentMotion == 'f' || currentMotion == 'B' || currentMotion == 'b') {
		int32_t dir = (currentMotion == 'B' || currentMotion == 'b') ? -1 : 1;
		int32_t distMm = (int32_t)((int64_t)LINEAR_SPEED_MM_PER_S * (int64_t)dtMs / 1000LL);
		distMm *= dir;
		float yawRad = yawDeg * 0.01745329252f;
		poseXmm += (int32_t)((float)distMm * cos(yawRad));
		poseYmm += (int32_t)((float)distMm * sin(yawRad));
	}
}

static void sendTelemetry(uint16_t distMm) {
	// Format: T,<ms>,<x_mm>,<y_mm>,<yaw_cdeg>,<dist_mm>,<mode>\n
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
		char *comma = strchr(line, ',');
		if (!comma) return;
		int32_t v;
		if (!parseInt32(comma + 1, &v)) return;
		if (v < 20) v = 20;
		if (v > 500) v = 500;
		LINEAR_SPEED_MM_PER_S = v;
		bt.print(F("OK,V,"));
		bt.print(LINEAR_SPEED_MM_PER_S);
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
		if (lineLen == 1 && (c == 'F' || c == 'f' || c == 'B' || c == 'b' || c == 'S' || c == 's' || c == 'L' || c == 'l' || c == 'R' || c == 'r' || c == 'A' || c == 'a' || c == 'M' || c == 'm' || c == 'P' || c == 'p')) {
			lineBuf[1] = 0;
			processLine(lineBuf);
			lineLen = 0;
			if (cu == 'F' || cu == 'B' || cu == 'S' || cu == 'L' || cu == 'R')
				lastMotionCmd = cu;
		}
	}
}

