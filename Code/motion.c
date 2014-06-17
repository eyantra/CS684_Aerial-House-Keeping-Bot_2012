#include <avr/io.h>
#include<avr/interrupt.h>
#include<util/delay.h>
#include<stdlib.h>

#define LEFT 1
#define RIGHT 2
#define C1 3
#define C2 4


void moveToDestination(int, int);
void processRecievedIp();

struct movement {
	int distance;
	int wheel;
};

struct coordinateDistance {
	float left;
	float right;
	float c2;
};

struct coordinates {
	int x;
	int y;
};



#define bool int
#define true 1
#define false 0

#define ERROR 15
bool response = false;
bool hasRecievedInput = false;
unsigned char data; //to store received data from UDR1
volatile unsigned int ShaftCountRight = 0, ShaftCountLeft = 0;
volatile unsigned int ShaftCountC1 = 0, ShaftCountC2 = 0;
volatile int currentXCoord = 0;
volatile int currentYCoord = 0;
volatile int noOfIpRecieved;
volatile int index = 0;
volatile int ipX = 0,ipY = 0;

volatile int actualLeftDist,actualRightDist,actualC2Dist;
unsigned char recievedCoordinates[20];
struct coordinateDistance coordToDistance[3][3];
struct coordinateDistance currentCoordinates;
struct coordinates ipCoordinates[5];



/*
	Interrupt service routine for all the four motors
	Note that ISR7 is for C1 wheel but interrupt for C1 has been disabled in Firebird V
*/

ISR(INT5_vect) {
	cli();
	ShaftCountRight++;
	sei();
}


ISR(INT4_vect) {
	cli();
	ShaftCountLeft++;
	sei();
}


ISR(INT6_vect) {
	cli(); 
	ShaftCountC2++;
	sei();
}


ISR(INT7_vect) {
	cli();
	ShaftCountC1++;
	sei();
}


SIGNAL(SIG_USART0_RECV) 		// ISR for receive complete interrupt
{
	data = UDR0; 				//making copy of data from UDR0 in 'data' variable 
	recievedCoordinates[index++] = data;
	if(data == '.') {
		processRecievedIp();
		if(recievedCoordinates[0] == 'u')
			hasRecievedInput = true;
		
	}
}



void processRecievedIp() {

//	now we have recieved the actual position of BB from the camera in recievedCoordinates[]

	volatile int tempx = 0, tempy = 0, i = 0;
	volatile int tempLeft = 0, tempRight = 0,tempC2 = 0;
	if(recievedCoordinates[0] == 'u') {
		i++;
//		buzzer_on();
//		_delay_ms(1000);
//		buzzer_off();
		while(recievedCoordinates[i] != ','){
			tempx = (tempx*10) + (recievedCoordinates[i] - '0');
//			recievedCoordinates[i] = '-';
//			UDR0 = recievedCoordinates[i];
			i++;
		}
		i++;		// skip index with value ','

		while(recievedCoordinates[i] != '.'){
			tempy = (tempy*10) + (recievedCoordinates[i] - '0');
			i++;
		}
		index = 0;
		ipX = tempx;
		ipY = tempy;
	} else {

		while(recievedCoordinates[i] != ','){
			tempLeft = (tempLeft*10) + (recievedCoordinates[i] - '0');
			i++;
		}
		
		i++;		// skip index with value ','

		while(recievedCoordinates[i] != ','){
			tempRight = (tempRight *10) + (recievedCoordinates[i] - '0');
			i++;
		}
		
		i++;		// skip index with value ','

		while(recievedCoordinates[i] != '.'){
			tempC2 = (tempC2*10) + (recievedCoordinates[i] - '0');
			i++;
		}

		index = 0;
		actualLeftDist = tempLeft;
		actualRightDist = tempRight;
		actualC2Dist = tempC2;

		response = true;
	}

}

void Init_Ports(){
	DDRA = 0xFF;
	PORTA = 0x00;

	DDRE = 0x08;
	PORTE = 0x08;

	DDRL = 0x38;
	PORTL = 0x38;

	DDRC = 0xFF;
}



void initInterrupts() {
	DDRE |= 0xF0;
	PORTE |= 0xF0;

}

//Function To Initialize UART0
// desired baud rate:9600
// actual baud rate:9600 (error 0.0%)
// char size: 8 bit
// parity: Disabled

void uart0_init(void)
{
 UCSR0B = 0x00; //disable while setting baud rate
 UCSR0A = 0x00;
 UCSR0C = 0x06;
 UBRR0L = 0x47; //11059200 Hz
// UBRR0L = 0x5F; // 14745600 Hzset baud rate lo
 UBRR0H = 0x00; //set baud rate hi
 UCSR0B = 0x98;
}


void left_position_encoder_interrupt_init (void) //Interrupt 4 enable
{
	cli(); //Clears the global interrupt
	EICRB = EICRB | 0x02; // INT4 is set to trigger with falling edge
	EIMSK = EIMSK | 0x10; // Enable Interrupt INT4 for left position encoder
	sei(); // Enables the global interrupt
}
void right_position_encoder_interrupt_init (void) //Interrupt 5 enable
{
	cli(); //Clears the global interrupt
	EICRB = EICRB | 0x08; // INT5 is set to trigger with falling edge
	EIMSK = EIMSK | 0x20; // Enable Interrupt INT5 for right position encoder
	sei(); // Enables the global interrupt
}

void c1_position_encoder_interrupt_init (void) //Interrupt 7 enable
{
	cli(); //Clears the global interrupt
	EICRB = EICRB | 0x80; // INT7 is set to trigger with falling edge
	EIMSK = EIMSK | 0x80; // Enable Interrupt INT7 for C1 position encoder
	sei(); // Enables the global interrupt
}

void c2_position_encoder_interrupt_init (void) //Interrupt 6 enable
{
	cli(); //Clears the global interrupt
	EICRB = EICRB | 0x20; // INT6 is set to trigger with falling edge
	EIMSK = EIMSK | 0x40; // Enable Interrupt INT6 for C2 position encoder
	sei(); // Enables the global interrupt
}



/*
	Below are all the 9 motion functions of each wheel of the firebird bot.
*/

void Stop () {
	PORTA = 0x00;
}

void rightForward() {
	PORTA = 0x04;		//PA2
}
void rightBackward() {
	PORTA = 0x08;		//PA3
}

void leftForward() {
	PORTA = 0x02;		//PA1
}
void leftBackward() {
	PORTA = 0x01;		//PA0
}

void c1Forward() {
	PORTA = 0x20;		//PA5
}
void c1Backward() {
	PORTA = 0x10;		// PA4
}

void c2Forward() {
	PORTA = 0x80;		//PA7
}
void c2Backward() {
	PORTA = 0x40;		//PA6
}


/*
	linear_distance_mm: takes as parameter the distance and the wheel no which needs to be moved
	by the required distance
	
	continualy checks whether the wheel has moved the required distance, and if yes it simply stops 
	the rotation and returns.
*/

void linear_distance_mm(unsigned int DistanceInMM,int wheel)
{
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;
	unsigned int currentShaftCount = 0;
//	unsigned long int CurrentRotShaftInt = 0;
//	ReqdShaftCount = DistanceInMM / 5.338; // division by resolution to get shaft count

	//circumference of wheel changed to 85 mm	85/30 = 2.83mm
	ReqdShaftCount = DistanceInMM / 2.83; // division by resolution to get shaft count

	ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;

	switch(wheel) {
		case LEFT:	ShaftCountLeft = 0; break;
		case RIGHT:	ShaftCountRight = 0; break;
		case C1:	ShaftCountC1 = 0; break;
		case C2:	ShaftCountC2 = 0;break;
	}
	while(1) {
		switch(wheel) {
			case LEFT:	currentShaftCount = ShaftCountLeft; break;
			case RIGHT:	currentShaftCount = ShaftCountRight; break;
			case C1:	currentShaftCount = ShaftCountC1; break;
			case C2:	currentShaftCount = ShaftCountC2;break;
		}
		if(currentShaftCount > ReqdShaftCountInt)
		{
			break;
		}
	}
	Stop(); //Stop robot
}



void initDevices() {
	Init_Ports();
	uart0_init(); 							//Initailize UART1 for serial communiaction
	cli();									 //Clears the global interrupt
	right_position_encoder_interrupt_init();
	left_position_encoder_interrupt_init();
	c1_position_encoder_interrupt_init();
	c2_position_encoder_interrupt_init();
	sei(); 									// Enables the global interrupt
}


void makecoordToDistance(int x,int y,float left,float right, float c2) {
	coordToDistance[x][y].left = left;
	coordToDistance[x][y].right = right;
	coordToDistance[x][y].c2 = c2;

}


void initiliazeCoordToDistance(){

	makecoordToDistance(0,0,0,800,1264.9);
	makecoordToDistance(1,1,180.3,715.9,1092.0);
	makecoordToDistance(1,2,427.2,427.2,1050);
	makecoordToDistance(1,3,715.9,180.3,1092.0);
	makecoordToDistance(2,1,650,813.9,618.5);
	makecoordToDistance(2,3,813.9,650,618.5);
	makecoordToDistance(3,2,1123.6,1123.6,150);

}



/*
	moveBB : moves the blackBox, by rotating each wheel to the required destination
	Amount of rotation required by each wheel as taken input as parameters
	first wheels with positive distances are moved followed by the 
	wheels with negative distances
*/

void moveBB(int left,int right,int c2){

	int c1 = 0;
	struct movement dist[4];
	dist[0].distance = left; dist[0].wheel= LEFT;
	dist[1].distance = right; dist[1].wheel= RIGHT;
	dist[2].distance = c1; dist[2].wheel= C1;
	dist[3].distance = c2; dist[3].wheel= C2;

	
	struct movement positiveDistances[4];
	struct movement negativeDistances[4];

	int i,j,k;
	for(i=0,j=0,k=0;k<4;k++){
		if(dist[k].distance == 0)
			continue;
		else if(dist[k].distance > 0){
			positiveDistances[i].distance = dist[k].distance;
			positiveDistances[i++].wheel =  dist[k].wheel;
		} else {
			negativeDistances[j].distance = dist[k].distance;
			negativeDistances[j++].wheel =  dist[k].wheel;
		}
	}

	for(k = 0;k<i;k++) {

		switch(positiveDistances[k].wheel) {
			case LEFT:
				leftForward();
				linear_distance_mm(positiveDistances[k].distance,LEFT);
				break;
			case RIGHT:
				rightForward();
				linear_distance_mm(positiveDistances[k].distance,RIGHT);
				break;
			case C1:
				c1Forward();
				linear_distance_mm(positiveDistances[k].distance,C1);
				break;
			case C2:
				c2Forward();
				linear_distance_mm(positiveDistances[k].distance,C2);
				break;
		}
	}
	
	for(k = 0;k<j;k++) {
		switch(negativeDistances[k].wheel) {
			case LEFT:
				leftBackward();
				linear_distance_mm(abs(negativeDistances[k].distance),LEFT);
				break;
			case RIGHT:
				rightBackward();
				linear_distance_mm(abs(negativeDistances[k].distance),RIGHT);
				break;
			case C1:
				c1Backward();
				linear_distance_mm(abs(negativeDistances[k].distance),C1);
				break;
			case C2:
				c2Backward();
				linear_distance_mm(abs(negativeDistances[k].distance),C2);
				break;
		}
	}
	
}

/*
	moveToDestination

	takes as input 2 parameters viz x,y which are coordinates of the required 
	destination where the BB is to be moved

	it first calculates the amount of distance required by each wheel to reach destination,
	thn it calls "moveBB" to move the BB

	Parameters to moveBB is the distances required by each wheel to be moved calculated

	After the movement is done it asks the feedback system to give actual position of BB and 
	if the position is not correct, calculates the error and moves to BB again to reach final destination

*/

void moveToDestination(int x, int y) {

//	check whether the x and y coords are in field range

	int left = coordToDistance[x][y].left - coordToDistance[currentXCoord][currentYCoord].left;
	int right = coordToDistance[x][y].right - coordToDistance[currentXCoord][currentYCoord].right;
	int c2 = coordToDistance[x][y].c2 - coordToDistance[currentXCoord][currentYCoord].c2;
//	int c1 = coordToDistance[x][y].c1 - coordToDistance[currentXCoord][currentYCoord].c1;

	moveBB(left,right,c2);


	do {


		UDR0 = 'r';	//send request to computer to give the actual position of BB using the camera mounted
	
	//	buzzer_on();
		response = false;
		while(!response);

	//	now we have recieved the actual position of BB from the camera in recievedCoordinates[]

		if((abs(actualLeftDist - coordToDistance[x][y].left) > ERROR) ||
			 (abs(actualRightDist - coordToDistance[x][y].right) > ERROR)||
			 (abs(actualC2Dist - coordToDistance[x][y].c2) > ERROR))
		{
			moveBB(coordToDistance[x][y].left - actualLeftDist,coordToDistance[x][y].right - actualRightDist ,
				coordToDistance[x][y].c2 - actualC2Dist);
		} else {
			break;
		}	
	}while(1);

	currentXCoord = x;
	currentYCoord = y;
}



 
int main(){
	initDevices();
	initiliazeCoordToDistance();
	currentXCoord = 0;
	currentYCoord = 0;

	while(1){				//	waiting for input from user
		hasRecievedInput = false;
		while(!hasRecievedInput);
			moveToDestination(ipX,ipY);
	}

	return 0;
}

