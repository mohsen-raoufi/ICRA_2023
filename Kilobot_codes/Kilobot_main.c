#include <kilolib.h>
#include <math.h>

#define STOP 0
#define FORWARD 1
#define LEFT 2
#define RIGHT 3

// Defining the three major states: exploration, local achieving, and consensus-based chemotaxis/phototaxis
#define DISPERSION 0
#define CONSENSUS 1
#define CHEMOTAXIS 2

// Defining the sub-states for dispersion
// 0: still too tight: keep doing the random walk
// 1: I am good, but my neighbors are still moving, I should wait for them
// 2: I got disconnected from the rest, HALT!! DO NOT MOVE (to make it worse!), show the RED LED
// 3: Seems like we are all good, let's start sharing
// 4: Measuring the light for a few samples
// 5: Sharing + local averaging following DeGroot Model
#define NULL 0 // still dispersing
#define WAIT_FOR_OTHERS 1
#define DISCONNECTED 2
#define READY_TO_SHARE 3
#define MEASURE 4
#define SHARING 5


// Defining a few registers for coding the messages between Kilobots, showing which state they are at
#define DISPERSING_RX 0xAA
#define WAITING4OTHERS_RX 0xAB
#define CONSNSS_SHARING_RX 0xCA


static const uint8_t MAX_DISTANCE = 60; // ~ 60 mm: this is the parameter that tunes the dispersion distance
int current_motion = STOP; 				// Initial value
message_t message;						// message ...
uint32_t last_motion_update = 0;		// Initial value
uint8_t cur_distance = 0;				// Initial Value
distance_measurement_t dist;			// distance estimated from the strength of the message

// ---------------------
// Flag to keep track of message transmission.
int message_sent = 0;
// Flag to keep track of new messages.
int new_message = 0;

long double z_col = 0.0; 				// (local) collective mean
long double z_m = 0;					// posteriori opinion
long double z_s = 0;					// a priori (measured/sensed)
int N = 0;								// Number of neighbors (NOTE: Kilobots should ideally have distinct IDs)

double beta = 0.01;						// Memory (self-weight/decaying factor for averaging)
double g = 0.6;							// 
double lamda = 0.4;						// it is reinitialized later on in the setup, how to update gamma

int counter;

int ids[21];							// a vector containing the IDs of received messages
double vals[21];						// a vector containing the values of received messages

uint32_t tick;							// a counter for time, later used for checking the messages
uint16_t get_averaged_ambient_light();	// a function to check the sensor value
void send_light();						// 
void send_subState(int subState);		//
int contains_id(int id);				// to check if I already counted the ids
void do_random_walk();					//
void chemotaxis();						// chemotaxis and phototaxis are used exchangeably 

int last_light = 0;						// last measured light during chemotaxis
int last_dist2Ref = 0;					// distance to the reference light intensity
int ref_light = 0;						// reference light: in this case, consensus value
int ambient = 0;						// measured current light

int dist2Ref = 0;						// absolute value |l_ref - l_cur|
float diff_dist2Ref = 0;				// difference of the dist2Ref compared to the last sample
int diffThresh = 20;					// threshold defining the decision for chemotaxis
int switch2Exploit = 100;				// Duration of consensus: number of iterations of averaging
int counterConstEnv = 0;				// a heuristic to do the random walk
// ----------------------------------------

int state = DISPERSION; // 0: dispersion, 1: consensus, 2: chemotaxis
// Dispersion : AA: Dispersing 		AB: Waiting for other disperse		CC: Consensus SHARING		DD: Chemotaxis

//int state = 0; // 0: measure, 1: interact/communicate with others ,  2: do chemotaxis!
int subState = NULL;

// Function to handle motion.
void set_motion(int new_motion)
{
	// Only take an an action if the motion is being changed.
	if (current_motion != new_motion)
	{
		current_motion = new_motion;

		if (current_motion == STOP)
		{
			set_motors(0, 0);
		}
		else if (current_motion == FORWARD)
		{
			spinup_motors();
			set_motors(kilo_straight_left, kilo_straight_right);
		}
		else if (current_motion == LEFT)
		{
			spinup_motors();
			set_motors(kilo_turn_left, 0);
		}
		else if (current_motion == RIGHT)
		{
			spinup_motors();
			set_motors(0, kilo_turn_right);
		}
	}
}

void do_random_walk()
{
	// Generate an 8-bit random number (between 0 and 2^8 - 1 = 255).
	int random_number = rand_soft();

	// Compute the remainder of random_number when divided by 4.
	// This gives a new random number in the set {0, 1, 2, 3}.
	int random_direction = (random_number % 4);


	// There is a 50% chance of random_direction being 0 OR 1, in which
	// case set the LED green and move forward.
	if ((random_direction == 0) || (random_direction == 1))
		//            if ((random_direction == 0) || (random_direction == 1))
	{
		//                set_color(RGB(0, 1, 0));
		set_motion(FORWARD);
	}
	// There is a 25% chance of random_direction being 2, in which case
	// set the LED red and move left.
	else if (random_direction == 2)
	{
		//                set_color(RGB(1, 0, 0));
		set_motion(LEFT);
	}
	// There is a 25% chance of random_direction being 3, in which case
	// set the LED blue and move right.
	else if (random_direction == 3)
	{
		//                set_color(RGB(0, 0, 1));
		set_motion(RIGHT);
	}
}

void dispersion()
{
	if (kilo_ticks > tick + 32)
	{
		tick = kilo_ticks;

		// If a message was received within the last second
		if (new_message == 1)
		{
			new_message = 0;
			subState = NULL;
			cur_distance = estimate_distance(&dist);

			// If the distance between the kilobot and its neighbour is too
			// large, stop its motion
			if (cur_distance > MAX_DISTANCE){
				set_motion(STOP);

				subState = WAIT_FOR_OTHERS;

				// check if all the other neighbors finished their dispersion state
				int othersDone = 1;
				for (int i = 0; i < N; i++) {
					if(vals[i]==DISPERSING_RX) // if there is any one that is still dispersing, then I am not ready to share!
					{
						othersDone = 0;
						break;
					}
				}
				if(othersDone==1)
					subState = READY_TO_SHARE;
			}

			else // if the distance is close enough ->> do random walk
				do_random_walk();


			if(subState == READY_TO_SHARE) 
			{
				set_motion(STOP);
				state = CONSENSUS;
				subState = MEASURE;//READY_TO_SHARE;
			}

		}

		// If no messages were received within the last second, set the LED red
		// and stop moving.
		else if(new_message == 0)
		{
			set_motion(STOP);
			subState = DISCONNECTED;
		}

		N = 0;
	}
}

void consensus()
{
	// int switch2Exploit = 100;

	if (subState== MEASURE) {
		counter = 0;
		int NFirstSamples = 20;				// Number of samples to measure before start sharing
		z_m = 0;
		for (int i = 0; i < NFirstSamples; i++)
		{
			z_m += (double) get_averaged_ambient_light();
		}
		z_m /= (double) NFirstSamples;

		delay(1500);
		set_color(RGB(0,0,0));
		delay(100);
		subState = SHARING;
	}

	send_light();

	if(subState == SHARING) { // communicate
		if (kilo_ticks > tick + 40) { // ToDo: do we need this? (now that we have couple of delays)
			tick = kilo_ticks;
			z_col = 0;
			int t = 1;
			for (int i = 0; i < N; i++) {
				z_col += (vals[i] - z_col) / t;
				++t;
				//			counter++; // we can also put it here! so that, those who have more neighbors switch to exploitation earlier
			}

			counter++;

			z_s = (double) get_averaged_ambient_light();

			if(N == 0) z_col = z_s;

			double updatedG;

			if(counter<20)
			{
				updatedG= (double) (counter/switch2Exploit);
				updatedG = g + lamda*updatedG;
			}
			else
			{
				updatedG = 0.99;
			}

			z_m = beta*z_m + updatedG*z_col + (1.0-beta-updatedG)*z_s;

			N = 0;
		}

		// Show the opinion with the LED, just for visualization purposes :)
		int phase;
		phase = ((int) z_m/125)%8; // make the value to be between 0 and 8
		set_color(RGB(phase>>2, phase&0b010, phase&0b001)); 	

		// Get ready for the exploitation phase 
		if(counter==switch2Exploit)
		{
			ref_light = z_m;
			state = CHEMOTAXIS;
			for(int phase=0; phase<8; phase++)
			{
				set_color(RGB(phase>>2, phase&0b010, phase&0b001));
				delay(100);
			}
			counter ++;
		}
	}
}

void chemotaxis()
{
	if (kilo_ticks > tick + 40) { 
		tick = kilo_ticks;

		z_col = 0;
		if(N == 0) z_col = z_m;
		int t = 1;
		for (int i = 0; i < N; i++) {
			z_col += (vals[i] - z_col) / t;
			++t;
			}
		z_m = beta*z_m + (1.0-beta)*z_col; // stil update the collective estimation, but no input from the environment!
		N = 0;
	}

	ref_light = z_m;			// The reference light for chemotaxis is the consensus value

	int timeDuration = 200;		// the time duration for moving forward in ms (between the two sampling/decision making in chemotaxis)

	ambient = get_averaged_ambient_light(); 	// current ambient light
	dist2Ref = fabs(ambient-ref_light);		 	// the distance  to the ref. light (unsigned)
	int checkVal = fabs(last_light-ambient);

	if(dist2Ref<50)								// if it is close enough to the consensus light! if(fabs(ambient-ref_light)<70)
	{
		set_motion(STOP);
		set_color(RGB(1, 1, 1)); 				// White
		delay(100);
		set_color(RGB(0,0,0)); 					// OFF
		delay(200);
		set_color(RGB(1, 1, 1)); 				// White
		delay(100);
		set_color(RGB(0,0,0)); 					// OFF
		delay(200);
		delay(10*timeDuration);
	}
	else 										// still needs to do the chemotaxis
	{
		if( (checkVal>diffThresh) ) 			// if you could sense some difference in the light, compared to the last time you measured it
		{
			set_color(RGB(0,3,3));				//Cyan RGB LED
			delay(100);

			last_light = ambient; 				// update the last light, for the next steps ...

			diff_dist2Ref = dist2Ref - last_dist2Ref;

			last_dist2Ref = dist2Ref;			// update the difference to the ref. light for the next steps

			z_s = ambient;

			if(diff_dist2Ref<0)
			{
				set_motion(FORWARD);
				set_color(RGB(0, 1, 0)); 		// Green
				delay(100);
				set_color(RGB(0, 0, 0)); 		// off
				delay(timeDuration);
			}
			else if(diff_dist2Ref>1)
			{
				int rnd = rand_soft();
				set_motion(STOP);
				set_color(RGB(1, 1, 1)); 		// White
				delay(100);

				if(1) 							// Always turn right! nothing stochastic!!
				{
					set_motion(RIGHT);
					set_color(RGB(0, 0, 1)); 	// Blue
				}
				else
				{
					set_motion(LEFT);
					set_color(RGB(1, 0, 0)); 	// RED
				}

				delay(100);
				set_color(RGB(0, 0, 0)); 		// off
				delay(2000);
				set_motion(FORWARD);
				set_color(RGB(3,0,3));			//turn RGB LED Violet
				delay(50);
				set_color(RGB(0, 0, 0)); 		// off
				delay(2*timeDuration);
			}
		}
		else
		{
			set_motion(FORWARD);
			set_color(RGB(1,1,0));
			delay(100);
			set_color(RGB(0, 0, 0)); 			// off
			delay(timeDuration);
			counterConstEnv++;
		}

		if(counterConstEnv > 40) 				// if the robot is measuring the same value for a while: meaning it stuck, or is lost, or ... nothing good!
		{
			counterConstEnv = 0; 				// reset the counter
			spinup_motors();
			set_motors(180, 180);
			delay(400);
			int rnd = rand_soft();
			if(rnd<180)
			{
				set_motion(RIGHT);
				set_color(RGB(0, 0, 1)); 		// Blue
			}
			else
			{
				set_motion(LEFT);
				set_color(RGB(1, 0, 0)); 		// RED
			}

			delay(100);
			set_color(RGB(0, 0, 0)); 			// off
			delay(2000);
			set_motion(FORWARD);
		}
	}
}

void setup()
{
	// Initialize message:
	// The type is always NORMAL.
	message.type = NORMAL;
	// Some dummy data as an example.
	message.data[0] = 0;
	// It's important that the CRC is computed after the data has been set;
	// otherwise it would be wrong and the message would be dropped by the
	// receiver.
	message.crc = message_crc(&message);

	kilo_ticks += (rand_hard())<<2;

	rand_seed(rand_hard()); // randomly seed the random for soft_rand

	tick = kilo_ticks;
	counter = 1;

	lamda = 1.0 - beta - g; // just to get sure the sum of parameters is equal to 1.0

	delay(2000);
}

void loop()
{

	switch (state) {
	case CONSENSUS:
		consensus();

		delay(50);
		set_color(RGB(0,0,0));
		delay(50);
		break;

	case CHEMOTAXIS:
		chemotaxis();
		break;

	case DISPERSION: 
		dispersion();

		switch (subState) {
		case NULL:
			set_color(RGB(0, 1, 0));
			break;
		case DISCONNECTED:
			set_color(RGB(1, 0, 0));
			break;
		case WAIT_FOR_OTHERS:
			set_color(RGB(0, 0, 1));
			break;
		case READY_TO_SHARE:
			set_color(RGB(0, 1, 1));
			break;
		}

		break;
	}


}

//void message_rx(message_t *m, distance_measurement_t *d)
void message_rx(message_t *message_r, distance_measurement_t *distance)
{
	new_message = 1;
	dist = *distance;

	if(state==DISPERSION)
	{
		send_subState(subState);
		if(subState != NULL)
		{

			{
				int id = message_r->data[0];
				int ind = contains_id(id);
				int checkStateOthers = (int) message_r->data[1];
				if (ind != -1) {
					vals[ind] = checkStateOthers;
				}
				else {
					ids[N] = id;
					vals[N] = checkStateOthers;
					N++;
				}
			}
		}
	}

	else 
	{
		int checkStateOthers = (int) message_r->data[1];
		if(checkStateOthers == DISPERSING_RX)
		{
			state = DISPERSION;
			subState = WAIT_FOR_OTHERS;
			send_subState(subState);
		}
		else // others are also in consensus state
		{
			
			// Reset the flag so the LED is only blinked once per message.
			int id = message_r->data[0];
			int st = message_r->data[1];
			if(st==CONSNSS_SHARING_RX)
			{
				uint32_t light = (unsigned long) message_r->data[2];
				light = light << 8;
				light |= message_r->data[3];
				light = light << 8;
				light |= message_r->data[4];
				light = light << 8;
				light |= message_r->data[5];

				double l = light / (double) 1000;
				int ind = contains_id(id);
				if (ind != -1) {
					vals[ind] = l;
				} else {
					ids[N] = id;
					vals[N] = l;
					N++;
				}
			}

			send_light();
		}
	}
}

message_t* message_tx() {
	if (message_sent == 0) {
		return &message;
	}
	return 0;
}

void message_tx_success() {
	// Set the flag on message transmission.
	message_sent = 1;
}


int main()
{
	// initialize hardware
	kilo_init();
	//	debug_init();
	// Register the message_tx callback function.
	kilo_message_tx = message_tx;
	// Register the message_tx_success callback function.
	kilo_message_tx_success = message_tx_success;
	// Register the message_rx callback function.
	kilo_message_rx = message_rx;
	// start program
	kilo_start(setup, loop);

	return 0;
}

void send_subState(int subState)
{
	uint8_t st;
	//	AA: Dispersing 		BB: Waiting for other disperse		CC: Consensus		DD: Chemotaxis
	switch (subState) {
	case NULL:
		st = DISPERSING_RX;
		break;
	case WAIT_FOR_OTHERS:
		st = WAITING4OTHERS_RX;
		break;
	case READY_TO_SHARE:
		st = CONSNSS_SHARING_RX;
		break;
	case MEASURE:
		st = CONSNSS_SHARING_RX;
		break;
	default:
		st = 0xBB;
		break;
	}
	message.data[0] = kilo_uid;
	message.data[1] = st; // Dispersing


	message.crc = message_crc(&message);
	message_sent = 0;
}

void send_light() {

	uint32_t x = z_m * 1000; // send z_m or z_s
	message.data[0] = kilo_uid;
	message.data[1] = CONSNSS_SHARING_RX;
	message.data[2] = (x >> 24) & 0xFF;
	message.data[3] = (x >> 16) & 0xFF;
	message.data[4] = (x >> 8) & 0xFF;
	message.data[5] = (x) & 0xFF;

	message.crc = message_crc(&message);
	message_sent = 0;

}

int contains_id(int id) {
	for (int i = 0; i < N; i++) {
		if (ids[i] == id)
			return i;
	}
	return -1;
}

//get light intensity
uint16_t get_averaged_ambient_light() {
	static const uint16_t NUM_SAMPLES = 400;
	uint32_t sum = 0;
	uint16_t sample_counter = 0;

	while (sample_counter < NUM_SAMPLES) {
		int16_t sample = get_ambientlight();
		if (sample != -1) {
			sum += sample;
			sample_counter++;
		}
	}
	return (sum / NUM_SAMPLES);
}
