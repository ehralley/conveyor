#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>

#define CLOCK_MULT 4

#define PARTS_TO_SORT 48
#define FORWARD 0b00001000 //Commands for motor direction
#define REVERSE 0b00000100
#define BRAKE	0b00000000
#define BRAKE_TIME 300 //ms
#define IND_RANGE 1500
#define REFL_RANGE 13000

#define STEP_SCALE_SPEED 100.0
#define STEP_HOME_TIME 18
#define MAX_STEP_TIME_EMPTY 14.0 //14
#define MIN_STEP_TIME_EMPTY 4.5 //4.75
#define MAX_STEP_TIME_FULL 14.0 //14 //new 14
#define MIN_STEP_TIME_FULL 5.0//5.5 //new 5.0

#define STEP_ACCEL_LINEAR_EMPTY 0.935 //.92
#define STEP_DECEL_LINEAR_EMPTY 1.10 //1.10
#define STEP_ACCEL_LINEAR_FULL 0.955 //.945 //new .955
#define STEP_DECEL_LINEAR_FULL 1.075 //1.075 //new 1.08
#define STEP_SWITCH_DELAY 40.0
#define STEP_PUMP_ACCEL_RATIO .25
#define STEP_PUMP_DECEL_RATIO .75

#define STEP_FALL_RANGE_STOP_EMPTY 25 //25
#define STEP_FALL_RANGE_STOP_FULL 20 //20
#define STEP_FALL_RANGE_CONT_EMPTY 50 //50
#define STEP_FALL_RANGE_CONT_FULL 39 //39
#define STEP_FALL_RANGE_ACCEL_CONT 10 //10
#define TOO_CLOSE_CONT_RANGE 35 //35
uint8_t step_fall_range_stop = STEP_FALL_RANGE_STOP_EMPTY;
uint8_t step_fall_range_cont = STEP_FALL_RANGE_CONT_EMPTY;

#define MIN_SPEED_FOR_CONT_RANGE 7.0

#define DEQUEUE_HOLD_TIME_STOPPED 320 //ms
uint16_t dequeue_hold_time_reverse = 500; //ms //360


#define BELT_PWM_CRAWL 65
#define BELT_PWM_MAX 166
#define DIST_TOTAL_BELT 750000
#define DIST_OP1_OP2 67000
#define DIST_OP1_OP3 237500
#define DIST_ACCEL_CALIB_SUBTRACT 5000
#define DIST_OP1_IND 30186
#define DIST_OP3_BRAKE 3000 //not used
#define DIST_OP3_FALL 29500

#define ADC_count 3 //how many values are stored for each ADC
#define INTERRUPT_DEBOUNCE_LENGTH 100


// define the global variables that can be used in every function ==========
uint16_t ADC_result = 0;
uint8_t ACD_result_flag;

uint32_t timer2_pear = 0;
uint8_t timer2_fract = 0;
uint32_t timer2_overflow_count = 0;
uint32_t interrupt_debounce_timer =0;
uint32_t dequeue_time = 0;

uint32_t belt_pos = 0;
int16_t belt_accel_counter =0; //now signed, positive for accel, negative for decel
int16_t belt_speed = 0;
int16_t belt_speed_start =0; //speed at start of accel or decel

uint16_t belt_speed_max = 351;
float belt_accel = 2.807;
float belt_decel =-3.426;

uint16_t brake_dist = 15000;
uint32_t ramp_down_pos =0;
uint8_t pause_program_flag=0;
uint8_t unidentified_part_flag =0;
uint8_t amount_sorted =0;

uint8_t stepper_pos = 0;
uint8_t desired_pos = 0;
uint8_t prev_pos =0;
int8_t stepper_command =0;
uint8_t start_stepper_pos = 0;
int8_t stepper_dir = 0;
int8_t desired_dir=0;
uint8_t stepper_switch_flag = 0;
float prev_step_time = 0;
float step_delay = 1;
int8_t stepper_state =0;

float max_step_time = MAX_STEP_TIME_EMPTY;
float min_step_time = MIN_STEP_TIME_EMPTY;

float step_accel_linear = STEP_ACCEL_LINEAR_EMPTY;
float step_decel_linear = STEP_DECEL_LINEAR_EMPTY;

//char Steps[4] = {0b00110000, 0b00000110, 0b00101000, 0b00000101}; //station 4
char Steps[4] = {0b00110000, 0b00000101, 0b00101000,0b00000110}; //station 11

uint32_t op1_pos = 0;
uint32_t op2_pos = 0;
uint32_t op3_pos = 0;
int16_t op_counter1= 0x00;
int16_t op_counter2= 0x00;
int16_t op_counter3= 0x00;
int16_t ind_counter= 0x00;

uint16_t ADC_refl_values[ADC_count]; //Array of size ADC_count
uint16_t ADC_refl_val =1023;

uint16_t refl_theoretical[4] = {960, 150, 959, 576}; //Black, Aluminum, White, Steel //station 4 vals
#define BLACK_REFL_MIN 959
#define WHITE_REFL_MIN 840 //???
#define STEEL_REFL_MAX 839 //???
#define STEEL_REFL_MIN 270
#define AL_REFL_MAX 339
#define AL_REFL_MIN 1 //???

uint8_t interrupt_flag = 0;

typedef struct part{
	int8_t ind;
	uint8_t material;
	uint8_t drop_state;
	uint8_t direction;
	uint16_t next_dist;
	uint16_t refl;
	uint32_t pos1;
	uint32_t pos2;
	uint32_t pos3;

	struct part *next;
	struct part *prev;
} part;

typedef struct dropped_part{
	uint8_t material;
	uint8_t drop_state;
	struct dropped_part *next;
	struct dropped_part *prev;
} dropped_part;

uint8_t too_close_cont_flag =0;
uint8_t list_size=0;
part *head=NULL;
part *tail=NULL;
dropped_part *dhead=NULL;
dropped_part *dtail=NULL;

void enqueue(part **h, part **t);
void dequeue(part **h, part **t);
void checkOPsensor3();
void beltUpdate();
int16_t remainingSteps();
void pauseProgram();
//===========================================END OF VARIABLES=====================================================


void setup() {
	// changing clock speed===============================================
	CLKPR = 0b10000000;
	CLKPR = 0b00000000;

	cli(); // disable all of the interrupt ==========================

	// config the external interrupt ======================================
	EIMSK |= (_BV(INT0));							//enable INT0
	EICRA |= (_BV(ISC00) | _BV(ISC01)); 			// rising edge interrupt

	EIMSK |= (_BV(INT1));							// enable INT1
	EICRA |= (_BV(ISC10) | _BV(ISC11)); 			// rising edge interrupt

	EIMSK |= (_BV(INT2)); 							// enable INT2
	EICRA |= (_BV(ISC20) | _BV(ISC21)); 			// rising edge interrupt

	EIMSK |= (_BV(INT3));							// enable INT3
	EICRA |= (_BV(ISC30) | _BV(ISC31));				// rising edge interrupt

	// config ADC =========================================================
	// by default, the ADC input (analog input is set to be ADC0 / PORTF0
	ADCSRA |= _BV(ADEN); // enable ADC
	ADCSRA |= _BV(ADIE); // enable interrupt of ADC
	ADMUX |= _BV(REFS0) | _BV(MUX1);
	//ADMUX |= _BV(ADLAR) | _BV(REFS0) | _BV(MUX1);

	// set the PORTC as output to display the ADC result ==================
	DDRA = 0xff;
	DDRB = 0xff;
	DDRC = 0xff;
	DDRD = 0x00;
	DDRE = 0xff;
	DDRF = 0x00;

	PORTA = 0x00;
	PORTB = FORWARD;
	PORTC = 0x00;

	// Sets the registers necessary to run pear()========================
	TCCR2A |= _BV(WGM21);				// Sets Waveform Generation Mode for pear()
	//TCCR2B |= _BV(CS20);
	TCCR2B |= _BV(CS21);				// Sets Prescaler value to clk/8
	TIMSK2 |= _BV(TOIE2);				// Sets the interrupt enable for overflow
	TCNT2 = 0x00;

	// sets the Global Enable for all interrupts ==========================
	sei();
	return;
}



unsigned long pear(){
	unsigned long m;
	uint8_t oldSREG = SREG;

	// disable interrupts while we read timer2_pear or we might get inconsistent vals
	cli();
	m = timer2_pear;
	SREG = oldSREG;

	return m;
}


void pearDelay(uint16_t length){
	length=length*CLOCK_MULT;
	uint32_t pear_delay_start = pear();
	while(1){
		if(pear()-pear_delay_start>length) break;
	}
}


void pwm(uint8_t duty_cycle) { //Runs pwm on pin 7 port B.
	TCCR0A |= _BV(WGM00);
	TCCR0A |= _BV(WGM01);
	TCCR0A |= _BV(COM0A1);
	TCCR0B |= _BV(CS01); //Prescaler to 1/8;
	//TIMSK0 |= _BV(OCIE0A);
	OCR0A = duty_cycle;
}

void calculateTimeToDecel(){
	uint8_t steps = (floor)(log(max_step_time/min_step_time)/log(step_decel_linear)+1);
	dequeue_hold_time_reverse = CLOCK_MULT*(min_step_time*(step_fall_range_stop-steps)+(min_step_time*(1-pow(step_decel_linear,steps+1))/(1-step_decel_linear))+32);		
}



void flashLeds() {
	uint32_t counter = pear();
	uint8_t i = 1;
	while(i < 4) {
		if(i%2 == 1) {
			PORTA = 0b10101010;
		}
		else {
			PORTA = 0b01010101;
		}
		if(pear()-counter > 500*CLOCK_MULT) {
			counter = pear();
			i++;
		}
	}
}


void resetADCreflVals(){
	//for(int i=0; i<ADC_count; i++) ADC_refl_values[i] = 1023;
	ADC_refl_val=1023;
}


uint16_t averageADCreflVals(){
	uint16_t ADC_sum = 0;
	for(int i=0; i<ADC_count; i++) ADC_sum += ADC_refl_values[i]; //sum stored reflectance values
	return ADC_sum/ADC_count; //assign reflectance value to part
}


void calibBreak(){
	PORTB = BRAKE;
	uint32_t start_brake_time=pear();
	while(1){
		if(pear()-start_brake_time>BRAKE_TIME*CLOCK_MULT) break;
	}
}


void calibReverse(int t){
	PORTB=REVERSE;
	pwm(BELT_PWM_MAX);
	uint32_t start_rev_time=pear();
	while(1){
		if(pear()-start_rev_time>t*CLOCK_MULT) break;
	}
}

void calibrateMaterials(){
	calibBreak();
	flashLeds();
	flashLeds();
	resetADCreflVals();
	for(int i=0; i<4; i++) { //test each material
		PORTA=i+1; //display what material should be going in
		PORTC = 0x00; //clear C so A can clearly show which material is next
		uint16_t ADC_avg = 0;
		uint8_t j=0;
		while((interrupt_flag & 0b00000100) != 0b00000100) {
			pwm(BELT_PWM_MAX);
			PORTB=FORWARD;
			uint32_t start_time=0;
			while(1){ //check op2 sensor to know when to start reflective sensing
				if((PIND&0b00100000) == 0b00100000){
					if(op_counter1==-1) start_time=pear();
					if(op_counter1==1) break;
					op_counter1++;
				}
				else{
					if(op_counter1<=0) op_counter1=-1;
					else op_counter1=0;
				}
			}
			ADCSRA |= _BV(ADSC); //call ADC conversion once at start of loop
			while(1){
				if (ACD_result_flag){ //if conversion is done
					ACD_result_flag = 0x00; //reset flag

					if(ADC_result<ADC_refl_val) ADC_refl_val = ADC_result;

					/*
					uint16_t temp_refl = 0;
					for(int k=0; k<ADC_count; k++){ //Keep lowest three values per part
						if(ADC_result<ADC_refl_values[k]){ //will re sort list and keep lowest three ADC values
							temp_refl = ADC_refl_values[k];
							ADC_refl_values[k] = ADC_result;
							ADC_result = temp_refl;
						}
					}*/
					ADCSRA |= _BV(ADSC); //call ADC conversion again
				}
				if(pear()-start_time>100*CLOCK_MULT) break; //stop reflective sensing after set amount of time
			}
			//ADC_avg = (ADC_avg*j+averageADCreflVals())/(j+1); //update average each pass
			ADC_avg = (ADC_avg*j+ADC_refl_val)/(j+1);
			resetADCreflVals();
			PORTC = ADC_avg;
			PORTA = ADC_avg>>8;
			calibBreak();
			calibReverse(800);
			calibBreak();
			pearDelay(300); //time to push part back to rail
			j++;
		}
		interrupt_flag &= 0b11111011;
		PORTC = ADC_avg;
		PORTA = ADC_avg>>8;
		refl_theoretical[i] = ADC_avg;
		pearDelay(10000); //delay to record value
	}
}


uint8_t fallStopped(uint8_t switcher, uint16_t fall_reverse_time, uint16_t *fbp){
	if(PORTB== BRAKE && belt_speed == 0 && head!= NULL){
		calibBreak();
		uint8_t op3_pull = (~PIND)&0b01000000;
		head->pos3=0;
		PORTB=REVERSE;
		uint32_t start_rev_time=pear();
		while(1){
			checkOPsensor3();
			if(pear()-start_rev_time>fall_reverse_time*CLOCK_MULT) break;
		}
		calibBreak();
		calibBreak();
		PORTB=FORWARD;
		if(switcher==0){
			if(head->pos3 ==0 && op3_pull==0x00) return 1;
			else{
				*fbp += belt_speed_max;
				head->pos3=0;
				return 0;
			}
		}
		else{
			if(head->pos3 ==0 && op3_pull==0x00) *fbp-=belt_speed_max;
			return 0;
		}
	}
	return 0;
}



void fallBrakeCheck(uint16_t fall_brake_pos){
	if(head->pos3!=0 && belt_pos-head->pos3 > fall_brake_pos){
		PORTB=BRAKE;
	}
}


void calibrationFall(){
	calibBreak();
	flashLeds();
	PORTB=FORWARD;
	uint32_t prev_pear_fall = pear();
	uint16_t fall_brake_pos =0;
	enqueue(&head, &tail);
	while(1){ //run until part first falls off
		if(pear()>prev_pear_fall+CLOCK_MULT-1) {
			prev_pear_fall=pear();
			beltUpdate();
			checkOPsensor3();
			fallBrakeCheck(fall_brake_pos);
			if(fallStopped(0, 1000, &fall_brake_pos)==1) break;
		}
	}
	flashLeds();
	fall_brake_pos-=belt_speed_max;
	while((interrupt_flag & 0b00001000) != 0b00001000) {
		if(pear()>prev_pear_fall+CLOCK_MULT-1) {
			prev_pear_fall=pear();
			beltUpdate();
			checkOPsensor3();
			fallBrakeCheck(fall_brake_pos);
			fallStopped(1, 1000, &fall_brake_pos);
		}
	}
	interrupt_flag &= 0b11110111; //reset flag
	PORTA=fall_brake_pos>>8;
	PORTC=fall_brake_pos;
	pearDelay(15000);
	uint16_t d=-(belt_speed_max*belt_speed_max)/(2*belt_decel) + fall_brake_pos;
	PORTA = d>>8;
	PORTC = d;
	pearDelay(15000);
	dequeue(&head, &tail);
}


void calibrationInd(){
	calibBreak();			//stop the belt
	flashLeds();
	pwm(BELT_PWM_MAX);
	uint16_t time_avg=0;
	uint8_t i=0;
	while((interrupt_flag & 0b00001000) != 0b00001000) {
		PORTB = FORWARD;
		uint32_t start_time=0;
		uint32_t end_time=0;
		uint32_t end_time2=0;
		while(1){
			if(((~PINF)&0b00001000) == 0b00001000){
				if(op_counter1==-1) start_time=pear();
				if(op_counter1==1){
					break;
				}
				op_counter1++;
			}
			else{
				if(op_counter1<=0) op_counter1=-2;
				else op_counter1=0;
			}
		}
		while(1){
			if(((~PIND)&0b00010000) == 0b00010000){
				if(op_counter1==-1) end_time=pear();
				op_counter1++;
			}
			else{
				if(op_counter1<=0){
					op_counter1=-1;
					if(end_time!=0){
						end_time2 = pear();
						break;
					}
				}
				else op_counter1=0;
			}
		}
		time_avg=(time_avg*i+((end_time2-end_time)/2+end_time-start_time)/CLOCK_MULT)/(i+1);
		PORTA=time_avg>>8;
		PORTC=time_avg;
		calibBreak();
		calibReverse(time_avg+500);
		calibBreak();
		i++;
	}
	interrupt_flag &= 0b11110111; //reset flag
	uint16_t temp_ind_dist = belt_speed_max*time_avg; //(10um/ms)
	PORTA=temp_ind_dist>>8;
	PORTC=temp_ind_dist;
	pearDelay(15000);
	//calibrationFall();
}


void calibrationDecel(){
	calibBreak();			//stop the belt
	flashLeds();
	pwm(BELT_PWM_MAX);
	uint16_t time_avg=0;
	uint8_t i=0;
	while((interrupt_flag & 0b00001000) != 0b00001000) { //loop until button is pressed again
		PORTB = FORWARD;
		uint32_t start_time=0;
		uint32_t end_time=0;

		while(1){
			if(((~PINF)&0b00001000) == 0b00001000){
				if(op_counter1==-1) start_time=pear();
				if(op_counter1==1){
					break;
				}
				op_counter1++;
			}
			else{
				if(op_counter1<=0) op_counter1=-1;
				else op_counter1=0;
			}
		}
		while(1){
			if(pear()-start_time>200*CLOCK_MULT){
				calibBreak();
				PORTB=FORWARD;
				break;
			}
		}
		while(1){
			if(((~PIND)&0b01000000) == 0b01000000){
				if(op_counter1==-1) end_time=pear();
				if(op_counter1==1){
					break;
				}
				op_counter1++;
			}
			else{
				if(op_counter1<=0) op_counter1=-1;
				else op_counter1=0;
			}
		}
		time_avg=(time_avg*i+(end_time-start_time)/CLOCK_MULT)/(i+1);
		PORTA=time_avg>>8;
		PORTC=time_avg;
		calibBreak();
		calibReverse(time_avg-BRAKE_TIME+400);
		calibBreak();
		i++;
	}
	interrupt_flag &= 0b11110111; //reset flag
	float time_accel = (belt_speed_max/belt_accel);					// time required to accelerate from stop
	uint16_t time_full= time_avg-BRAKE_TIME-time_accel;						// time at full speed between sensors
	belt_decel = -((float)belt_speed_max*belt_speed_max)/(2.0*((float)DIST_OP1_OP3-(float)time_full*belt_speed_max-0.5*belt_accel*time_accel*time_accel)); // (10um)/ms^2
	belt_decel = -belt_decel*1000;
	PORTC=(int)belt_decel;
	PORTA=(int)belt_decel>>8;
	belt_decel=-belt_decel/1000;
	pearDelay(15000);
	calibrationInd();
}



void calibrationAccel(){
	calibBreak();					//stop the belt
	flashLeds();
	uint16_t time_avg=0;
	uint8_t i=0;
	while((interrupt_flag & 0b00001000) != 0b00001000) {
		pwm(BELT_PWM_CRAWL);
		PORTB=FORWARD;
		uint32_t start_time =0;
		uint32_t end_time=0;
		while(1){
			if(((~PINF)&0b00001000) == 0b00001000){
				calibBreak();
				start_time = pear();
				PORTB= FORWARD;
				pwm(BELT_PWM_MAX);
				break;
			}
		}
		while(1){
			if(((~PIND)&0b01000000) == 0b01000000){
				if(op_counter1==-1) end_time=pear();
				if(op_counter1==1){
					break;
				}
				op_counter1++;
			}
			else{
				if(op_counter1<=0) op_counter1=-1;
				else op_counter1=0;
			}
		}
		time_avg=(time_avg*i+(end_time-start_time)/CLOCK_MULT)/(i+1);
		PORTA=time_avg>>8;
		PORTC=time_avg;
		calibBreak();
		calibReverse(time_avg+250);
		calibBreak();
		i++;
	}
	interrupt_flag &= 0b11110111; //reset flag
	belt_accel = (float)-1.0/((2.0*(DIST_OP1_OP3-DIST_ACCEL_CALIB_SUBTRACT)/((float)belt_speed_max*belt_speed_max))-(2.0*time_avg/belt_speed_max)); // um/ms^2
	belt_accel = belt_accel*1000;
	PORTC=(int)belt_accel;
	PORTA=(int)belt_accel>>8;
	belt_accel = belt_accel/1000;
	pearDelay(15000);
	calibrationDecel();
}


void calibrationVMax(){
	calibBreak();			//stop the belt
	flashLeds();
	pwm(BELT_PWM_MAX);
	uint16_t time_avg=0;
	uint8_t i=0;
	while((interrupt_flag & 0b00001000) != 0b00001000) {
		PORTB = FORWARD;
		uint32_t start_time=0;
		uint32_t end_time=0;

		while(1){
			if(((~PINF)&0b00001000) == 0b00001000){
				if(op_counter1==-1) start_time=pear();
				if(op_counter1==1){
					break;
				}
				op_counter1++;
			}
			else{
				if(op_counter1<=0) op_counter1=-1;
				else op_counter1=0;
			}
		}
		while(1){
			if(((~PIND)&0b01000000) == 0b01000000){
				if(op_counter1==-1) end_time=pear();
				if(op_counter1==1){
					break;
				}
				op_counter1++;
			}
			else{
				if(op_counter1<=0) op_counter1=-1;
				else op_counter1=0;
			}
		}
		time_avg=(time_avg*i+(end_time-start_time)/CLOCK_MULT)/(i+1);
		PORTA=time_avg>>8;
		PORTC=time_avg;
		calibBreak();
		calibReverse(time_avg+500);
		calibBreak();
		i++;
	}
	interrupt_flag &= 0b11110111; //reset flag
	belt_speed_max=DIST_OP1_OP3/time_avg; //(10um/ms)
	PORTA=belt_speed_max>>8;
	PORTC=belt_speed_max;
	pearDelay(15000);
	calibrationAccel();
}




void enqueue(part **h, part **t) {
	part *new_part = malloc(sizeof(part));
	new_part->next = NULL;
	new_part->prev = NULL;
	if (*t != NULL){
		new_part->prev = *t;
		(*t)->next = new_part;
		*t = new_part;
	}
	else{
		*h = new_part;
		*t = new_part;
	}
	list_size++;
	new_part->ind=0;
	new_part->material=0;
	new_part->drop_state=0;
	new_part->direction=0;
	new_part->next_dist=0;
	new_part->refl=0;
	new_part->pos1=0;
	new_part->pos2=0;
	new_part->pos3=0;
	return;
}


void checkTooCloseContinuous(){
	desired_pos = (head->material-1)*50;
	if(head->drop_state==1 && abs(remainingSteps())<TOO_CLOSE_CONT_RANGE) head->drop_state=4;
}


void enqueueDropped(dropped_part **dh, dropped_part **dt) {
	dropped_part *new_part = malloc(sizeof(dropped_part));
	new_part->next = NULL;
	new_part->prev = NULL;
	if (*dt != NULL){
		new_part->prev = *dt;
		(*dt)->next = new_part;
		*dt = new_part;
	}
	else{
		*dh = new_part;
		*dt = new_part;
	}
	new_part->material=head->material;
	new_part->drop_state=head->drop_state;
	amount_sorted++;
	return;
}


void dequeue(part **h, part **t){
	part *dq_part = *h;
	if (*h != NULL){
		checkTooCloseContinuous(); //will set drop state to 4 if too close
		if(stepper_dir==0) head->drop_state = 5;
		enqueueDropped(&dhead, &dtail); //needs to be called after check too close cont to assign correct drop state
		*h = (*h)->next;
		if(*h==NULL) (*t)=NULL; //also set tail to null if list is empty
		else (*h)->prev=NULL; //remove link to old head
	}
	free(dq_part); //free memory
	list_size--;
	dequeue_time = pear();
	return;
}

void dequeueMissing(part **h, part **t){
	part *dq_part = *h;
	if (*h != NULL){
		*h = (*h)->next;
		if(*h==NULL) (*t)=NULL; //also set tail to null if list is empty
		else (*h)->prev=NULL; //remove link to old head
	}
	free(dq_part); //free memory
	list_size--;
	return;
}



void assignInductance(){
	/*PORTC+=1;
	part* iterator = tail;
	uint8_t closest_part = 0;
	int32_t shortest_dist =64000; //largest possible unsigned 16 bit
	int i = 1;
	PORTA=0x00;
	while(iterator!=NULL && iterator->pos1!= 0 && i<3){
		PORTA|=0b10000000;
		int32_t temp_belt_pos = belt_pos/2;
		int32_t temp_pos1 = iterator->pos1/2;
		int32_t temp_op1_ind = DIST_OP1_IND/2;
		if(abs(temp_belt_pos - temp_pos1 - temp_op1_ind) < shortest_dist){
			PORTA|=0b01000000;
			shortest_dist = abs(temp_belt_pos - temp_pos1 - temp_op1_ind);
			closest_part=i;
		}
		iterator= iterator->prev;
		i++;
	}
	part* iterator2=tail;
	PORTA|=closest_part;
	for(uint8_t m=1; m<closest_part; m++) iterator2=iterator2->prev; //assign iterator to the correct node.
	if(iterator2!=NULL && closest_part!=0) iterator2->ind =1;*/
	part* iterator = tail;
	if(tail!= NULL && belt_pos-iterator->pos1 > DIST_OP1_IND -6000) iterator->ind=1;
	else if(tail->next!=NULL) iterator->next->ind=1;

}


void checkInd() {
	//iterate through last 2 blocks. if ind value has not been set and position is between values appropriate for hall, set.
	/*part* iterator = tail;
	for(int i = 0; i<2; i++) {
		if((iterator->ind == 0) && (belt_pos - (iterator->pos1) > DIST_OP1_IND - IND_RANGE) && (belt_pos - (iterator->pos1) < DIST_OP1_IND + IND_RANGE)) {
			if(((~PIND)&0b00010000) == 0b00010000){
				iterator->ind = 1;
			}
			else{
				iterator->ind = -1;
			}
		}
		iterator = iterator->prev;
	}*/
	part* iterator=tail;
	while(iterator!= NULL && iterator->ind==0){
		if(belt_pos-iterator->pos1>DIST_OP1_IND-200){
			if(((~PIND)&0b00010000) == 0b00010000) iterator->ind=1;
			else iterator->ind=-1;
		}
		iterator=iterator->prev;
	}
}


void identifyAndPlan() {
	part* iterator = head;
	while(iterator!=NULL && iterator->material!=0) iterator=iterator->next;

	if(iterator!=NULL){
		iterator->refl = ADC_refl_val;
		resetADCreflVals();
/*
		//OLD CODE
		if(iterator->refl>=refl_theoretical[0]) iterator->material=1;
		else if(iterator->refl>STEEL_REFL_MAX) iterator->material=3;
		else if(iterator->refl>STEEL_REFL_MIN) iterator->material=4;
		else iterator->material=2;*/

		if(iterator->refl>=BLACK_REFL_MIN && iterator->ind!=1) iterator->material=1;
		else if(iterator->refl>WHITE_REFL_MIN && iterator->ind!=1) iterator->material=3;
		else if(iterator->refl>STEEL_REFL_MIN && iterator->refl<STEEL_REFL_MAX && iterator->ind==1) iterator->material=4;
		else if(iterator->refl<AL_REFL_MAX && iterator->refl>AL_REFL_MIN)iterator->material=2;
		else{
			iterator->material=1; //assign a random material so it does not =0
			unidentified_part_flag=1;
			pauseProgram();
			PORTC=iterator->refl;
			PORTA=iterator->refl>>8;
		}
	}

	//if((iterator->material==1 || iterator->material==3) && iterator->ind==1) iterator->material=4; //Set to steel if inductance is high even if reflectance thinks part is plastic
	/*if(iterator!=NULL){
		PORTA=iterator->refl>>8;
		PORTC=iterator->refl;

	}
	else PORTA=0xff;*/

}


void assignRefl(){  //Called after ADC conversion has been completed
	part* iterator = head;
	while(iterator!=NULL && iterator->material!=0) iterator=iterator->next;

	if(iterator!=NULL){
		if(ADC_result<ADC_refl_val) ADC_refl_val=ADC_result;
	}
}


int stepperHome() {
	uint16_t stepper_setup_counter = 0;
	while(stepper_setup_counter<10){
		stepper_command++;
		if(stepper_command > 3) stepper_command = 0;
		PORTE=Steps[stepper_command];
		pearDelay(STEP_HOME_TIME);
		stepper_setup_counter++;
	}
	stepper_setup_counter=0;
	while(stepper_setup_counter < 1){
		stepper_command++;
		if(stepper_command > 3) stepper_command = 0;
		PORTE=Steps[stepper_command];
		pearDelay(STEP_HOME_TIME);
		if(((~PIND)&0b10000000)==0b10000000) stepper_setup_counter++;
	}
	pearDelay(200);
	for(int i=0; i<4; i++){
		stepper_command--;
		if(stepper_command < 0) stepper_command = 3;
		PORTE=Steps[stepper_command];
		pearDelay(STEP_HOME_TIME);
	}
	stepper_pos=0;
	return 0;
}

void decreaseStepperSpeed(){

	max_step_time+=(MAX_STEP_TIME_FULL-MAX_STEP_TIME_EMPTY)/PARTS_TO_SORT;
	min_step_time+=(MIN_STEP_TIME_FULL-MIN_STEP_TIME_EMPTY)/PARTS_TO_SORT;
	step_accel_linear += (STEP_ACCEL_LINEAR_FULL-STEP_ACCEL_LINEAR_EMPTY)/PARTS_TO_SORT;
	step_decel_linear -= (STEP_DECEL_LINEAR_EMPTY-STEP_DECEL_LINEAR_FULL)/PARTS_TO_SORT;
	step_fall_range_cont -= (STEP_FALL_RANGE_CONT_EMPTY-STEP_FALL_RANGE_CONT_FULL)/PARTS_TO_SORT;
	step_fall_range_stop -= (STEP_FALL_RANGE_STOP_EMPTY-STEP_FALL_RANGE_STOP_FULL)/PARTS_TO_SORT;
	//calculateTimeToDecel();

}


uint8_t chooseFallRange(){ //only enters if head has a pos 3, determine if part is continuous or reversal/stop

	if(head!=NULL && head->next!=NULL && head->next->material==head->material) return 0; //stopping
	else if(head!=NULL && head->next!=NULL && head->next->material != 0 && (desired_dir==stepper_dir || desired_dir==0)){ //dont use longer range when slowing to reverse
		desired_pos = ((head->material)-1)*50; //desired pos will not be correctly set if 2nd part was at the sensor when head was dequeued since stepperCheck calls dequeue first
		uint8_t next_pos = ((head->next->material)-1)*50;
		if(stepper_dir==1){
			if(next_pos != desired_pos && next_pos-desired_pos!=150 && next_pos-desired_pos!=-50){ //next part is not a reversal or same material
				if(step_delay>MIN_SPEED_FOR_CONT_RANGE || step_delay<=1.0) return 2; //decrease range if just starting to accelerate
				return 1; //larger range if it is not slowing down to stop or reverse
			}
		}
		else if(stepper_dir==-1){
			if(next_pos != desired_pos && next_pos-desired_pos!=-150 && next_pos-desired_pos!=50){ //next part is not a reversal or same material
				if(step_delay>MIN_SPEED_FOR_CONT_RANGE || step_delay<=1.0) return 2;
				return 1; //larger range if it is not slowing down to stop or reverse
			}
		}
	}
	else if(head!=NULL && head->next==NULL) return 0;
	return 3; //reversing
}



uint8_t stepperDepositCheck(uint8_t material){ //can only enter if head!=NULL
	uint8_t drop_range =0;
	/*
	if(chooseFallRange()==1){
		 drop_range = step_fall_range_cont; //drop range depends on whether stepper will stop of continue through this material
		 head->continuous = 1;
	}
	else if(chooseFallRange()==2) drop_range = STEP_FALL_RANGE_ACCEL_CONT;
	else if(chooseFallRange()==3) drop_range = STEP_FALL_RANGE_REVERSE;
	else drop_range = step_fall_range_stop;*/

	head->drop_state=chooseFallRange();

	if(head->drop_state==1) drop_range = step_fall_range_cont;
	else if(head->drop_state==2) drop_range = STEP_FALL_RANGE_ACCEL_CONT;
	else if(head->drop_state==3) drop_range = step_fall_range_stop;
	else drop_range = step_fall_range_stop;

	if(stepper_dir==1){
		if(material==1){
			if(stepper_pos!= 0 && stepper_pos<200-drop_range) return 0;
		}
		else if(stepper_pos<(material-1)*50-drop_range || stepper_pos>(material-1)*50) return 0;
	}
	else if(stepper_dir==-1){
		if(material==1){
			if(stepper_pos>(material-1)*50+drop_range) return 0;
		}
		if(stepper_pos>(material-1)*50+drop_range || stepper_pos<(material-1)*50) return 0;
	}
	else if(stepper_dir==0 && stepper_pos!=(material-1)*50){
			//PORTC=(material+1)*50;
			return 0;
	}
	return 1;
}



void stepperCheck(){ //check to see if the stepper is in acceptable range to drop part.
	if(head != NULL && head->pos3 != 0 && belt_pos - head->pos3 > DIST_OP3_FALL) {
		dequeue(&head, &tail);
		decreaseStepperSpeed(); //Update stepper min and max step times on each dequeue
	}
	if(head != NULL && head->pos3 != 0) {
		if(belt_pos - head->pos3 > 10500) {
			if(head->material==0){
				 head->material=1; //last minute setting to black if unassigned
				 PORTA=0xff;
			}
			if(stepperDepositCheck(head->material)==0){ PORTB=BRAKE;
			}
		}
	}
	if(pause_program_flag == 1) PORTB=BRAKE;
	else if(head==NULL || stepperDepositCheck(head->material)==1) PORTB = FORWARD;
}



int16_t remainingSteps(){ //called from stepperRun
	int16_t result = desired_pos - stepper_pos;
	if(stepper_dir==1){
		if(dtail!= NULL && head!=NULL && (desired_pos-prev_pos==150 || desired_pos-prev_pos==-50)){
			desired_dir=-1;
		}
	}
	else if(stepper_dir==-1){
		if(dtail!= NULL && head!=NULL && (desired_pos-prev_pos==-150  || desired_pos-prev_pos == 50)){
			desired_dir=1;
		}
	}
	else if(stepper_dir==0 && result != 0){
		stepper_dir = 1;
		if(result == 150 || result == -50) stepper_dir = -1;
		step_delay = max_step_time;
	}
	if(stepper_dir==1 && result <0) result += 200;
	if(stepper_dir==-1 && result >0) result -=200;
	return result;
}



void stepperRun(){ //controls actions of the stepper
	//PORTA=dequeue_hold_time_reverse>>8;
	//PORTC=dequeue_hold_time_reverse;
	if(dtail!=NULL) prev_pos = ((dtail->material)-1)*50;
	/*
	if(amount_sorted>=1 && (step_delay>10.0 || step_delay<=1) && pear()-dequeue_time<DEQUEUE_HOLD_TIME_REVERSE) desired_pos = prev_pos; //reversing
	else if(amount_sorted>=1 && too_close_cont_flag==1 && pear()-dequeue_time<DEQUEUE_HOLD_TIME_REVERSE) desired_pos = prev_pos;
	else if(head!=NULL && head->material != 0) desired_pos = ((head->material)-1)*50;
	else if(dtail!=NULL){
		 desired_pos = prev_pos;
	}
	else desired_pos=stepper_pos;
	*/

	if(dtail!=NULL && dtail->drop_state ==0 && pear()-dequeue_time<dequeue_hold_time_reverse) desired_pos = prev_pos; //stopping
	else if(dtail!=NULL && dtail->drop_state ==2 && pear()-dequeue_time<DEQUEUE_HOLD_TIME_STOPPED) desired_pos = prev_pos; //accelerating from stop
	else if(dtail!=NULL && dtail->drop_state ==3 && pear()-dequeue_time<dequeue_hold_time_reverse) desired_pos = prev_pos; //reversing
	else if(dtail!=NULL && dtail->drop_state ==4 && pear()-dequeue_time<DEQUEUE_HOLD_TIME_STOPPED) desired_pos = prev_pos;//continuous but too close
	else if(dtail!=NULL && dtail->drop_state ==5 && pear()-dequeue_time<DEQUEUE_HOLD_TIME_STOPPED) desired_pos = prev_pos;
	else if(head!=NULL && head->material != 0) desired_pos = ((head->material)-1)*50; //Continuous
	else if(dtail!=NULL) desired_pos = prev_pos; //no parts, stay on last dropped
	else desired_pos=stepper_pos; //start of program

	//if(pear()-dequeue_time>=DEQUEUE_HOLD_TIME_REVERSE) too_close_cont_flag=0;

	int16_t steps_remaining = remainingSteps();
	if((pear()-prev_step_time) >= (floor)(step_delay*CLOCK_MULT+0.5)) { //is it time to step?
		stepper_state=0;
		prev_step_time = pear();
		if(steps_remaining == 0){
			stepper_dir=0;
			step_delay=0.25;
		}
		else{ //not in final position
			if(stepper_switch_flag==1){
				stepper_switch_flag=0;
				step_delay=max_step_time;
			}
			if(stepper_dir == 1) { //forward
				int8_t prev_command = stepper_command;
				stepper_command++;
				if(stepper_command == 4) stepper_command = 0;
				PORTE=Steps[stepper_command];
				stepper_pos++;
				if(stepper_pos>199) stepper_pos=0;
				if(steps_remaining <= (floor)(log(max_step_time/step_delay)/log(step_decel_linear)+1)){ //adjust speed to decelerate
					step_delay = step_delay*step_decel_linear;
					stepper_state = -1;
				}
				else if(desired_dir==-1){
					step_delay = step_delay*step_decel_linear;
					stepper_state = -1;
				}
				else if(step_delay>min_step_time) { //accelerate
					step_delay = step_delay*step_accel_linear;
					stepper_state=1;
				}
				if(desired_dir==-1 && step_delay>=max_step_time){
					 stepper_dir=-1;
					 stepper_switch_flag =1;
					 step_delay=STEP_SWITCH_DELAY;
					 PORTE |= Steps[prev_command];
				}
			}
			else if(stepper_dir == -1) { //reverse
				int8_t prev_command = stepper_command;
				stepper_command--;
				if(stepper_command == -1) stepper_command = 3;
				PORTE=Steps[stepper_command];
				if(stepper_pos==0) stepper_pos = 199;
				else stepper_pos--;
				if(abs(steps_remaining) <= (floor)(log(max_step_time/step_delay)/log(step_decel_linear)+1)){ //adjust speed to decelerate
					step_delay = step_delay*step_decel_linear;
					stepper_state = -1;
				}
				else if(desired_dir==1){
					step_delay = step_delay*step_decel_linear;
					stepper_state = -1;
				}
				else if(step_delay>min_step_time) {
					step_delay = step_delay*step_accel_linear;
					stepper_state=1;
				}
				if(desired_dir==1 && step_delay>=max_step_time){
					stepper_dir=1;
					stepper_switch_flag =1;
					step_delay=STEP_SWITCH_DELAY;
					PORTE |= Steps[prev_command];

				}
			}
		}
	}
}



void stepperPump(){
	if(stepper_switch_flag != 1 && stepper_state == 1 && ((pear()-prev_step_time) >= (floor)(step_delay*STEP_PUMP_ACCEL_RATIO*CLOCK_MULT+0.5))) {
		int8_t temp_command = stepper_command;
		if(stepper_dir==1){
			temp_command++;
			if(temp_command == 4) temp_command = 0;
			PORTE|=Steps[temp_command];
		}
		else if(stepper_dir==-1){
			temp_command--;
			if(temp_command == -1) temp_command = 3;
			PORTE|=Steps[temp_command];
		}
	}
	else if(stepper_switch_flag != 1 && stepper_state == -1 && ((pear()-prev_step_time) <= (floor)(step_delay*STEP_PUMP_DECEL_RATIO*CLOCK_MULT+0.5))) {
		int8_t temp_command = stepper_command;
		if(stepper_dir==1){
			temp_command--;
			if(temp_command == -1) temp_command = 3;
			PORTE|=Steps[temp_command];
		}
		else if(stepper_dir==-1){
			temp_command++;
			if(temp_command == 4) temp_command = 0;
			PORTE|=Steps[temp_command];
		}
	}
	else if(stepper_switch_flag != 1 && stepper_state == -1 && ((pear()-prev_step_time) > (floor)(step_delay*STEP_PUMP_DECEL_RATIO*CLOCK_MULT+0.5))) PORTE=Steps[stepper_command];
}



void checkOPsensor1() {
	if(((~PINF)&0b00001000) == 0b00001000){
		if(op_counter1==-1) op1_pos = belt_pos;
		else if(op_counter1==1){
			enqueue(&head, &tail);
			tail->pos1 = op1_pos;
		}
		op_counter1++;
	}
	else{
		if(op_counter1<=0) op_counter1=-1;
		else op_counter1=0;
	}
}


void checkOPsensor2() {
	if((PIND&0b00100000) == 0b00100000){
		if(op_counter2>=1) ADCSRA |= _BV(ADSC);
		else op_counter2++;
	}
	else{
		if(op_counter2>0) op_counter2=0;
		else if(op_counter2==0){ //two low senses in a row
			op_counter2=-1;
			identifyAndPlan();
		}
		else op_counter2=-1; //makes it so many senses of high are required to start ADC readings
	}
}

void checkOPsensor3() {
	if(((~PIND)&0b01000000) == 0b01000000){
		if(op_counter3==-1) {
			op3_pos = belt_pos;
		}
		else if(op_counter3==1){
			if(head!= NULL){
				if(head->pos3 == 0) {
					head->pos3 = op3_pos;
				}
				else if(head->next != NULL) {
					head->next->pos3 = op3_pos;
				}
			}
		}
		op_counter3 ++;
	}
	else{
		if(op_counter3 <= 0){
			op_counter3 = -1;
		}
		else op_counter3=0;
	}
}


void checkOPsensors() {
	checkOPsensor1();
	checkOPsensor2();
	checkOPsensor3();
}



void beltUpdateAccel(){
	if(belt_accel_counter<=0) {
		belt_accel_counter = 0;
		belt_speed_start = belt_speed;
	}
	belt_accel_counter++;
	belt_speed += (int)belt_accel; 		//floor belt acceleration to lower int and add to speed
	if(belt_accel*belt_accel_counter > (float)belt_speed-belt_speed_start+0.5){
		belt_speed += 1;
	}
	if(belt_speed>=belt_speed_max){
		belt_speed=belt_speed_max;
		belt_accel_counter=0;
	}
}


void beltUpdateDecel(){
	if(belt_accel_counter>=0) {
		belt_accel_counter = 0;
		belt_speed_start = belt_speed;
	}
	belt_accel_counter--;
	belt_speed += (int)belt_decel; 		//floor belt deceleration to higher negative int and add to speed
	if(belt_decel*(-belt_accel_counter) < (float)belt_speed-belt_speed_start-0.5){
		belt_speed -= 1;
	}
	if(belt_speed<=0){
		belt_speed=0;
		belt_accel_counter=0;
	}
}


void calculateBrakeDist(){
	brake_dist = (int)-(belt_speed*belt_speed)/belt_decel;
}

void beltUpdate() {
	if(PORTB==FORWARD && belt_speed<belt_speed_max){ //is it accelerating?
		beltUpdateAccel();
	}
	else if(PORTB==BRAKE && belt_speed!=0){ //Is it slowing down?
		beltUpdateDecel();
	}
	calculateBrakeDist();
	belt_pos += belt_speed;
	return;
}


void rampDown(){
	ramp_down_pos = belt_pos;
}


void pauseProgram(){ //will enter from pause button or unidentified part
	pause_program_flag=1;
}


void pauseForUnidentified(){
	PORTB=BRAKE;
	pause_program_flag=0;
	unidentified_part_flag =0;
	while((interrupt_flag & 0b00000001) != 0b00000001){ //if paused. run continually until 2nd button press
		PORTA=0xff;
		PORTC=0x00;
		pearDelay(500);
		PORTA=0x00;
		PORTC=0xff;
		pearDelay(500);
	}
	interrupt_flag &= 0b11111110;
	PORTA=0x00;
	PORTC=0x00;
}


uint8_t countSorted(uint8_t mat){
	uint8_t temp_counter=0;
	dropped_part *iterator=dhead;
	while(iterator!=NULL){
		if(iterator->material==mat) temp_counter++;
		iterator=iterator->next;
	}
	return temp_counter;
}


void displaySorted(){
	PORTB=BRAKE;
	pause_program_flag=0;
	while((interrupt_flag & 0b00000001) != 0b00000001 || ramp_down_pos!=0){ //if paused. run continually until 2nd button press
		if(ramp_down_pos!=0) interrupt_flag|=0b00000001;
		ramp_down_pos=0; //these two lines cause it to terminate after one loop on rampdown
		for(uint8_t i=1; i<=4; i++){
			PORTA=i;
			PORTC=countSorted(i);
			pearDelay(5000);
		}
		PORTA = 0xff;
		PORTC = list_size;
		pearDelay(5000);
	}
	interrupt_flag &= 0b11111110;
	PORTA=0x00;
	PORTC=0x00;
}

void checkMissingPart(){
	if(head!= NULL && head->pos1!=0 && (belt_pos-head->pos1)>(DIST_OP1_OP3+100000)) dequeueMissing(&head, &tail);	
}



void checkInterrupt() {
	if(interrupt_flag >0) {
		if((interrupt_flag & 0b00000001) == 0b00000001) {
			interrupt_flag &= 0b11111110;
			pauseProgram();
		}
		if((interrupt_flag & 0b00000010) == 0b00000010) {
			interrupt_flag &= 0b11111101;
			rampDown();
		}
		if((interrupt_flag & 0b00000100) == 0b00000100) { //each successive press will iterate through all four materials before leaving
			interrupt_flag &= 0b11111011;
			calibrateMaterials();
		}
		if((interrupt_flag & 0b00001000) == 0b00001000) { //continuing button presses will then move to further calibration functions
			interrupt_flag &= 0b11110111 ;
			calibrationVMax();
			//calibrationInd();
		}

	}
}


void mainLoop() {
	uint32_t prev_pear = 0;
	uint32_t prev_pear2 =0;
	pwm(BELT_PWM_MAX);
	PORTB = FORWARD;

	while(1) { //main while loop of the code
		if(pear()>prev_pear+CLOCK_MULT-1) {
			prev_pear= pear();
			beltUpdate();
			stepperCheck();
			checkInd();
			checkInterrupt();
			checkMissingPart();
			if(belt_speed<=0 && stepper_dir == 0 && pause_program_flag==1){
				if(unidentified_part_flag == 0) displaySorted();
				else pauseForUnidentified();
			}
			if(ramp_down_pos!= 0 && belt_pos-ramp_down_pos>DIST_TOTAL_BELT) break;
		}
		if(pear()>prev_pear2){
			prev_pear2=pear();
			checkOPsensors();
		}

		stepperRun();
		stepperPump();

		if (ACD_result_flag){ //check ADC flag while waiting for next ms to pass
			ACD_result_flag = 0x00; //reset flag
			assignRefl();
		}

	}
	displaySorted();
	return;
}


void terminateFlash(){
	uint8_t d=50;
	PORTA=0B10000000;
	pearDelay(d);
	PORTA=0b11000000;
	for(int j =0; j<2; j++){
		for(int i=0; i<6; i++){
			PORTA=PORTA>>1;
			pearDelay(d);
		}
		PORTA=0b00000001;
		PORTC=0b10000000;
		pearDelay(d);
		PORTA=0X00;
		PORTC=0b11000000;
		pearDelay(d);
		for(int i=0; i<6; i++){
			PORTC=PORTC>>1;
			pearDelay(d);
		}
		for(int i=0; i<6; i++){
			PORTC=PORTC<<1;
			pearDelay(d);
		}
		PORTA=0b00000001;
		PORTC=0b10000000;
		pearDelay(d);
		PORTC=0x00;
		PORTA=0B00000011;
		pearDelay(d);
		for(int i=0; i<6; i++){
			PORTA=PORTA<<1;
			pearDelay(d);
		}
	}
	PORTA=0b10000000;
	pearDelay(d);
	PORTA=0x00;
}


int main(){
	setup();
	calculateTimeToDecel();
	resetADCreflVals();
	stepperHome();
	mainLoop();
	terminateFlash();
	return 0;
}

ISR(INT0_vect) {
	if(pear()-interrupt_debounce_timer>INTERRUPT_DEBOUNCE_LENGTH*CLOCK_MULT){
		interrupt_debounce_timer = pear();
		interrupt_flag |= 0b00000001;
	}
}

ISR(INT1_vect) {
	if(pear()-interrupt_debounce_timer>INTERRUPT_DEBOUNCE_LENGTH*CLOCK_MULT){
		interrupt_debounce_timer = pear();
		interrupt_flag |= 0b00000010;
	}
}

ISR(INT2_vect) {
	if(pear()-interrupt_debounce_timer>INTERRUPT_DEBOUNCE_LENGTH*CLOCK_MULT){
		interrupt_debounce_timer = pear();
		interrupt_flag |= 0b00000100;
	}
}

ISR(INT3_vect) {
	if(pear()-interrupt_debounce_timer>INTERRUPT_DEBOUNCE_LENGTH*CLOCK_MULT){
		interrupt_debounce_timer = pear();
		interrupt_flag |= 0b00001000;
	}
}

// the interrupt will be trigured if the ADC is done ========================
ISR(ADC_vect) {
	ADC_result=ADCL;
	ADC_result+=ADCH<<8;
	ACD_result_flag = 1;
}

ISR(TIMER2_OVF_vect) {
	unsigned long m = timer2_pear;
	unsigned char f = timer2_fract;

	m += 1;
	f += 3;
	if (f >= 125) {
		f -= 125;
		m += 1;
	}

	timer2_fract = f;
	timer2_pear = m;
	timer2_overflow_count++;

	TCNT2 = 0x00;

}


ISR(BADISR_vect) {
	PORTA=0b11111111;
}



