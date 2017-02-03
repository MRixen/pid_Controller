#define di_encoderPinA 2
#define di_encoderPinB 3
#define do_motorDirection 4
#define do_pwm 9

const double ENCODER_TO_DEGREE = ((double)360 / 34608);
const int MULTIPLICATION_FACTOR = 10;

long encoderValue = 0;
int pwm_motorDirection;

// PDI controller data
double const SAMPLE_TIME = 0.05; // s
double const UPPER_SATURATION_LIMIT = 255;
double const LOWER_SATURATION_LIMIT = -255;
double P = -88.6551165175934;
double I = -233.477221663654;
double D = -5.18336719848569;
double N = 31.522805349829;

double pid_sollValue = 22.5; // deg

double pid_error;
double pid_istValue = 0;
double p_term = 0;
double i_term = 0;
double d_term = 0;
double preSat = 0;
double pid_output = 0;
double d_filter = 0;

void setup()
{
	// Configure serial interface
	Serial.begin(9600);

	// Define I/Os
	pinMode(di_encoderPinA, INPUT);
	pinMode(di_encoderPinB, INPUT);
	pinMode(do_motorDirection, OUTPUT);

	// Attach interrupt for encoder
	attachInterrupt(digitalPinToInterrupt(di_encoderPinA), doEncoderA, CHANGE);
	attachInterrupt(digitalPinToInterrupt(di_encoderPinB), doEncoderB, CHANGE);
}

void loop()
{
	// Convert encoder value to degree
	pid_istValue = encoderValue*ENCODER_TO_DEGREE;

	// Calculate error term (soll - ist)
	pid_error = pid_istValue - pid_sollValue;
	Serial.print("pid_sollValue: ");
	Serial.println(pid_sollValue);

	// Calculate output for motor
	pid_output = pidController(pid_error);
	Serial.print("pid_output: ");
	Serial.println(pid_output);

	// Configure direction value for motor
	// Direction input: when DIR is high (positive) current will flow from OUTA to OUTB, when it is low current will flow from OUTB to OUTA (negative).
	if (pid_output < 0) {
		digitalWrite(do_motorDirection, LOW);
		pid_output = pid_output*(-1);
	}
	else digitalWrite(do_motorDirection, HIGH);

	// Rotate motor
	analogWrite(do_pwm, (int)pid_output);

	delay(SAMPLE_TIME*1000);
}

double pidController(double error) {

	// Calculate p term
	p_term = P * error;

	// Calculate i term with clamping (anti windup)
	if (clamp(preSat, error)) i_term = i_term + 0;
	else i_term = i_term + (error*SAMPLE_TIME);	
	i_term = i_term*I;

	// Calculate d term with filtered derivative
	d_term = (D * error - d_filter) * N;

	// Summarize the p,i,d terms
	preSat = p_term + i_term + d_term;

	// Saturate output for max / min 255 / -255
	if (preSat > 255) pid_output = 255;
	else if (preSat < -255) pid_output = -255;
	else pid_output = preSat;

	d_filter = d_filter + d_term*SAMPLE_TIME;
}

bool clamp(double preSat, double preIntegrator) {

	double deadZone_out = 0;
	int signDeltaU = 0;
	int signPreIntegrator = 0;

	// Check if the presat value is inside the deadzone
	if ((preSat <= UPPER_SATURATION_LIMIT) & (preSat >= LOWER_SATURATION_LIMIT)) deadZone_out = 0;
	else {
		if ((preSat > UPPER_SATURATION_LIMIT)) deadZone_out = preSat - UPPER_SATURATION_LIMIT;
		else if((preSat < LOWER_SATURATION_LIMIT)) deadZone_out = preSat - LOWER_SATURATION_LIMIT;
	}

	// Calculate sign delta u
	if (deadZone_out > 0) signDeltaU = 1;
	else if (deadZone_out == 0) signDeltaU = 0;
	else if(deadZone_out < 0) signDeltaU = -1;

	// Calculate sign pre integrator
	if (preIntegrator > 0) signPreIntegrator = 1;
	else if (preIntegrator == 0) signPreIntegrator = 0;
	else if (preIntegrator < 0) signPreIntegrator = -1;

	// Return true if both signs equal and the preSat outside the dead zone
	if ((signDeltaU == signPreIntegrator) & (deadZone_out != 0)) return true;
}

void doEncoderA() {

	// look for a low-to-high on channel A
	if (digitalRead(di_encoderPinA) == HIGH) {
		// check channel B to see which way encoder is turning
		if (digitalRead(di_encoderPinB) == LOW) {
			encoderValue = encoderValue + 1;         // CW
		}
		else {
			encoderValue = encoderValue - 1;         // CCW
		}
	}
	else   // must be a high-to-low edge on channel A                                       
	{
		// check channel B to see which way encoder is turning  
		if (digitalRead(di_encoderPinB) == HIGH) {
			encoderValue = encoderValue + 1;          // CW
		}
		else {
			encoderValue = encoderValue - 1;          // CCW
		}
	}
}

void doEncoderB() {

	// look for a low-to-high on channel B
	if (digitalRead(di_encoderPinB) == HIGH) {
		// check channel A to see which way encoder is turning
		if (digitalRead(di_encoderPinA) == HIGH) {
			encoderValue = encoderValue + 1;         // CW
		}
		else {
			encoderValue = encoderValue - 1;         // CCW
		}
	}
	// Look for a high-to-low on channel B
	else {
		// check channel B to see which way encoder is turning  
		if (digitalRead(di_encoderPinA) == LOW) {
			encoderValue = encoderValue + 1;          // CW
		}
		else {
			encoderValue = encoderValue - 1;          // CCW
		}
	}
}
