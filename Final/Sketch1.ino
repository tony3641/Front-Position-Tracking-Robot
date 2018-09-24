/*
Name:		Sketch1.ino
Created:	2017/3/19 21:13:18
Author:	Tony Han-PC
*/

const short
motorL1 = 8, //L PWM1
motorL2 = 9, //L PWM2
motorR1 = 10, //R PWM1
motorR2 = 11, //R PWM2
dirL1 = 51, //Dir L PWM1
dirL2 = 53, //Dir L PWM2
dirR1 = 38, //Dir R PWM1
dirR2 = 40, //Dir R PWM2

ENA = 7,  //¾íÏßµç»ú
IN1 = 6,
IN2 = 5,

ENCODER_LEFT_A = 18,
ENCODER_LEFT_B = 19,//Integrated Encoder L

ENCODER_RIGHT_A = 3,
ENCODER_RIGHT_B = 24, ////!!!!!!!!!!!!!!!!////

ENCODER_A_A = 2,//!!!!!!!!!!!!!!!
ENCODER_A_B = 22;//!!!!!!!!!!!!!!!

int ENCODER_LEFT_A_LAST;
int ENCODER_RIGHT_A_LAST;
int ENCODER_A_A_LAST;

boolean Direction_L;
int durationL;
boolean Direction_R;
int durationR;
boolean Direction_A;
int durationA;

int Lfwd = 1;
int Lrev = 0;
int Rfwd = 0;
int Rrev = 1;
float lf, rf, lb, rb;

float iniX;
float iniY;

float iniAngel;
float Angel;
float Posture;
float iniPosture;

float X_Output;
float Y_Output;
float Posture_Output;

class vector
{
public:
	float x, y;

	void getPlus(vector a, vector b) {
		x = a.x + b.x;
		y = a.y + b.y;
	}

	void getMinus(vector a, vector b) {
		x = a.x - b.x;
		y = a.y - b.y;
	}

	const float getNorm() {
		return sqrt(x*x + y*y);
	}

	void getNormal(vector a) {
		x = a.y;
		y = -a.x;
	}

	void getMultiply(float n, vector v) {
		x = v.x *n;
		y = v.y*n;
	}

	float getDistance(vector a) {
		return sqrt((a.x - x)*(a.x - x) + (a.y - y)*(a.y - y));
	}
};

class PIDControl
{
public:
	float kp = 0.2, kd = 0, ki = 0.005;
	float PD_Control(int target, int current) {
		error = target - current;
		output = error*kp + (error - last)*kd;
		last = error;
		return output;
	}

	int PID_Control(int target, int current) {
		error = target - current;
		integral += error;
		output = kp*(error + integral*ki + (error - last)*kd);
		last = error;
		return output;
	}

private:
	int error;
	int last;
	int output;
	int integral;
};

class Position
{
public:
	const float RobotWidth = 800.0;
	const float DriverWidth = 1000.0;
	float M[2], Delta;

	void getPosition(float a, float b, float theta) {
		float bSinC, bCosC, d, d1, d3, p;
		vector other, o1, o2, x, nx, temp1, temp2, t, result, x1, x2, dis1, dis2;

		bSinC = b*sin(theta);
		bCosC = b*cos(theta);

		o1.x = RobotWidth / 2 - bCosC;
		o1.y = -bSinC;

		o2.x = -RobotWidth / 2;
		o2.y = 0;

		x.getMinus(o2, o1);
		nx.getNormal(x);

		Delta = 1000 * atan(nx.y / nx.x);

		d = x.getNorm();
		d1 = d / 2 + float((a*a - DriverWidth*DriverWidth)) / (2 * d);
		d3 = sqrt(a*a - d1*d1);
		p = d1 / d;
		temp1.getMultiply(p, o1);
		temp2.getMultiply(1 - p, o2);
		t.getPlus(temp1, temp2);
		result.getMultiply(d3 / nx.getNorm(), nx);
		x1.getPlus(t, result);
		x2.getMinus(t, result);

		other.x = RobotWidth / 2;
		other.y = 0;
		if (x1.getDistance(other) > x2.getDistance(other)) {
			M[0] = (x1.x + o1.x) / 2;
			M[1] = (x1.y + o1.y) / 2;
		}
		else {
			M[0] = (x2.x + o1.x) / 2;
			M[1] = (x2.y + o1.y) / 2;
		}
	}
};



void M(float y, float x, float w) {
	//Calculate
	lf = y + x + w;
	lb = y - x + w;
	rf = y - x - w;
	rb = y + x - w;

	//Standardize

	float i;
	float _max = abs(lf);
	if (abs(lb) > _max) _max = abs(lb);
	if (abs(rf) > _max) _max = abs(rf);
	if (abs(rb) > _max) _max = abs(rb);

	if (_max>80) {
		lf = 150.0 * lf / _max;
		lb = 150.0 * lb / _max;
		rf = 150.0 * rf / _max;
		rb = 150.0 * rb / _max;
	}
}
/*
Mega2560
2  int.0
3  int.1
21 int.2
20 int.3
19 int.4
18 int.5
*/
void ENCODER_R() {
	int Lstate = digitalRead(ENCODER_RIGHT_A);
	if ((ENCODER_RIGHT_A_LAST == LOW) && Lstate == HIGH) {
		int val = digitalRead(ENCODER_RIGHT_B);
		if (val == LOW && Direction_R) Direction_R = false; //Reverse
		else if (val == HIGH && !Direction_R) Direction_R = true;  //Forward
	}
	ENCODER_RIGHT_A_LAST = Lstate;

	if (!Direction_R) durationR++;
	else  durationR--;
}

void ENCODER_L() {
	int Lstate = digitalRead(ENCODER_LEFT_A);
	if ((ENCODER_LEFT_A_LAST == LOW) && Lstate == HIGH) {
		int val = digitalRead(ENCODER_LEFT_B);
		if (val == LOW && Direction_L) Direction_L = false; //Reverse
		else if (val == HIGH && !Direction_L) Direction_L = true;  //Forward
	}
	ENCODER_LEFT_A_LAST = Lstate;

	if (!Direction_L) durationL++;
	else  durationL--;
}

void ENCODER_A() {
	int Lstate = digitalRead(ENCODER_A_A);
	if ((ENCODER_A_A_LAST == LOW) && Lstate == HIGH) {
		int val = digitalRead(ENCODER_A_B);
		if (val == LOW && Direction_A) Direction_A = false; //Reverse
		else if (val == HIGH && !Direction_A) Direction_A = true;  //Forward
	}
	ENCODER_A_A_LAST = Lstate;

	if (!Direction_A) durationA++;
	else  durationA--;
}

void motorRunL1(float power, boolean dir) {
	digitalWrite(dirL1, dir);
	analogWrite(motorL1, power);
}
void motorRunL2(float power, boolean dir) {
	digitalWrite(dirL2, dir);
	analogWrite(motorL2, power);
}
void motorRunR1(float power, boolean dir) {
	digitalWrite(dirR1, !dir);
	analogWrite(motorR1, power);
}
void motorRunR2(float power, boolean dir) {
	digitalWrite(dirR2, !dir);
	analogWrite(motorR2, power);
}
void motorStop() {
	analogWrite(motorL1, 0);
	analogWrite(motorL2, 0);
	analogWrite(motorR1, 0);
	analogWrite(motorR2, 0);
}

void encoderInit() {
	pinMode(2, INPUT_PULLUP);
	pinMode(3, INPUT_PULLUP);//2 3 22 24 18 19;
	pinMode(22, INPUT_PULLUP);
	pinMode(24, INPUT_PULLUP);
	pinMode(18, INPUT_PULLUP);
	pinMode(19, INPUT_PULLUP);
}

void motorInit() {
	pinMode(motorL1, OUTPUT);
	pinMode(motorL2, OUTPUT);
	pinMode(motorR1, OUTPUT);
	pinMode(motorR2, OUTPUT);
	pinMode(dirL1, OUTPUT);
	pinMode(dirL2, OUTPUT);
	pinMode(dirR1, OUTPUT);
	pinMode(dirR2, OUTPUT);
	pinMode(ENA, OUTPUT);
	pinMode(IN1, OUTPUT);
	pinMode(IN2, OUTPUT);
}

int i = 0;
PIDControl XControl;
PIDControl YControl;

PIDControl PostureControl;
Position CurrentPosition;

void setup()
{
	pinMode(7, OUTPUT);
	pinMode(6, OUTPUT);
	pinMode(5, OUTPUT);
	analogWrite(ENA, 110);
	digitalWrite(IN1, HIGH);
	digitalWrite(IN2, LOW);

	/* add setup code here */
	Serial.begin(9600);
	motorInit();
	encoderInit();
	attachInterrupt(0, ENCODER_A, CHANGE);
	attachInterrupt(1, ENCODER_R, CHANGE);
	attachInterrupt(5, ENCODER_L, CHANGE);
	delay(1000);
	durationA = 500;
	durationR = -3000;
	durationL = 3000;
	delay(500);

	iniAngel = durationA;
	CurrentPosition.getPosition(durationL, -durationR, (PI*durationA) / 1000);
	iniX = CurrentPosition.M[0];
	iniY = CurrentPosition.M[1];
	iniPosture = CurrentPosition.Delta;
}

void loop()
{
	Angel = PI*(durationA) / 1000;
	CurrentPosition.getPosition(durationL, -durationR, Angel);
	//newX = CurrentPosition.M[0] * cos(PI / 8.8) - sin(PI / 8.8)*CurrentPosition.M[1];
	//newY = CurrentPosition.M[0] * sin(PI / 8.8) + cos(PI / 8.8)*CurrentPosition.M[1];
	X_Output = -XControl.PD_Control(iniX, CurrentPosition.M[0]);
	Y_Output = YControl.PD_Control(iniY, CurrentPosition.M[1]);
	Posture_Output = PostureControl.PID_Control(iniPosture, CurrentPosition.Delta);

	M(Y_Output, X_Output, Posture_Output);

	motorRunL1(abs(lf), lf > 0);
	motorRunL2(abs(lb), lb > 0);
	motorRunR1(abs(rf), rf > 0);
	motorRunR2(abs(rb), rb > 0);
	//Serial.println(durationA);
	delay(20);
	i++;
	if (i == 20) {
		i = 0;
		Serial.print(CurrentPosition.M[0]);
		Serial.print(",");
		Serial.print(CurrentPosition.M[1]);
		Serial.print(",");
		Serial.println(CurrentPosition.Delta);

		/*Serial.print("length");
		Serial.print(durationL);
		Serial.print(",");
		Serial.println(durationR);*/
	}
}