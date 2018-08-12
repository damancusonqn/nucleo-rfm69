#include "mbed.h"
#include <RFM69.h>
#include <RFM69registers.h>
#include "x_nucleo_iks01a1.h"

// ------------------------  RFM69  -----------------------------------------
#define IS_RFM69HW   //NOTE: uncomment this ONLY for RFM69HW or RFM69HCW
#define GATEWAY_ID   1     // this is ME, TGateway
#define NODE_ID       9    // node ID

#define NETWORKID     101   //the same on all nodes that talk to each other
#define FREQUENCY     RF69_868MHZ

#define MSGBUFSIZE  128   // message buffersize, but for this demo we only use: 
                        // 1-byte NODEID + 4-bytes for time + 1-byte for temp in C + 2-bytes for vcc(mV)
char msgBuf[MSGBUFSIZE];

//RFM69::RFM69(PinName  PinName mosi, PinName miso, PinName sclk,slaveSelectPin, PinName int)
// RFM69 radio(D11, D12, D13, D10, D8);
RFM69 radio(D11, D12, D13, D10, PA_0);

DigitalOut RFM_RESET(D8);
//D8 --> RFM_RESET

bool promiscuousMode = false; // set 'true' to sniff all packets on the same network
bool requestACK=false;

// --------------------------- X_NUCLEO_IKS01A1 -------------------------------------

/* Instantiate the expansion board */
static X_NUCLEO_IKS01A1 *mems_expansion_board = X_NUCLEO_IKS01A1::Instance(D14, D15);

/* Retrieve the composing elements of the expansion board */
static GyroSensor *gyroscope = mems_expansion_board->GetGyroscope();
static MotionSensor *accelerometer = mems_expansion_board->GetAccelerometer();
static MagneticSensor *magnetometer = mems_expansion_board->magnetometer;
static HumiditySensor *humidity_sensor = mems_expansion_board->ht_sensor;
static PressureSensor *pressure_sensor = mems_expansion_board->pt_sensor;
static TempSensor *temp_sensor1 = mems_expansion_board->ht_sensor;
static TempSensor *temp_sensor2 = mems_expansion_board->pt_sensor;

// ---------------------------------------------------------------------------------- 


Thread rfm69_thread(osPriorityNormal, 0x4000); 


// DigitalOut led1(PA_5);
DigitalOut led1(PA_3);


void rfm69_rx_run()
{
    uint8_t theNodeID;
    int16_t rssi;

    wait(1);

    printf("\n\r----  **********     RFM69HW test    **********       ----\n\r");    
    printf("\n\r---- Initializing RFM69 radio in MikroBUS-2 socket... ----\n\r");    
    radio.initialize(FREQUENCY, GATEWAY_ID, NETWORKID);
    radio.encrypt(0);
    radio.promiscuous(promiscuousMode);
    //DM: radio.promiscuous(promiscuousMode);

    rssi = radio.readRSSI(0);

    printf("RSSI: %d\n\r", rssi); 

    printf("Waiting for an incoming package...\n\r");  
    while(1){
        if (radio.receiveDone()) {
            printf("Received from TNODE: %d ",radio.SENDERID);
            printf((char*)radio.DATA);
            if (radio.ACKRequested()){
                theNodeID = radio.SENDERID;
                radio.sendACK();
                printf(" - ACK sent. Receive RSSI: %d\r\n",radio.RSSI);
            } else printf("Receive RSSI: %d\r\n",radio.RSSI);
        }  
        wait(.5);
        printf("."); 
    }
}

void rfm69_tx_run()
{
    // int16_t rssi;
    int i=0;
    uint8_t result;

    float humidity, temperature, pressure;
  
    RFM_RESET = 1;
    wait(.1);
    RFM_RESET = 0;
    wait(.1);

    printf("\033[2J \n\r----  **********     RFM69HW TX test    **********       ----\n\r");    
    printf("\n\r---- Initializing RFM69 radio in MikroBUS-2 socket... ----\n\r");    
    
    printf("\n\r**\n\r");    

    if(radio.initialize(FREQUENCY, NODE_ID, NETWORKID))
    	printf("\n\rRadio Initialized!\n\r"); 
    else 
    	printf("\n\r Error Initializing radio!!!\n\r");

    
   
    // rssi = radio.readRSSI(0);
    printf("FREQUENCY: %lu\n\r", radio.getFrequency()); 

    printf("RSSI: %d\n\r", radio.readRSSI(0)); 

    
    radio.encrypt(0);
    radio.setPowerLevel(20);
    //DM radio.setPowerLevel(20);
    radio.promiscuous(promiscuousMode);
    radio.setHighPower(true); 

    memset(msgBuf,0,sizeof(msgBuf));
    msgBuf[0] = (uint8_t)NODE_ID;  // load NODEID
 

    while(1){
    	temp_sensor1->get_temperature(&temperature);
    	humidity_sensor->get_humidity(&humidity);
    
    	sprintf((char*)msgBuf, "#%d, T=%2.2fC, H=%2.0f%%", i++, temperature, humidity); 

    	// printf("RSSI: %d \n\r", radio.readRSSI(0)); 
		// sprintf((char*)msgBuf, "#%d,  temp=%dC", i++, radio.readTemperature(-1)); 
		// if(radio.sendWithRetry((uint8_t)GATEWAY_ID, msgBuf, strlen(msgBuf), true)){
		// 	 printf("Packet %d sent, Ack ok!\r\n",i++);
		// }
		// else printf("Packet %d sent, no Ack!\r\n",i++);

		radio.send((uint8_t)GATEWAY_ID, msgBuf, strlen(msgBuf));
		printf("Send: %s\r\n", msgBuf);

    	wait(3);
    }

	// while(1) {
	// 	uint8_t tempC =  radio.readTemperature(-1); // -1 = user cal factor, adjust for correct ambient
	// 	uint8_t tempF = 1.8 * tempC + 32; // 9/5=1.8

	// 	// sprintf((char*)msgBuf,"#%d,  temp=%dF, RSSI=%d ",i,tempC,radio.RSSI); 		
	// 	// if(radio.sendWithRetry((uint8_t)GATEWAY_ID, msgBuf, strlen(msgBuf), true))
	// 	// 	printf("Packet %d sent, Ack ok!\r\n",i++);
	// 	// else printf("Packet %d sent, no Ack!\r\n",i++);

	// 	msgBuf[0]=1;
	// 	msgBuf[1]=2;
	// 	msgBuf[2]=3;

	// 	printf("TempC: %d\n\r", tempC); 
	// 	// if(radio.sendWithRetry((uint8_t)GATEWAY_ID, msgBuf, 3, true))
	// 	if(radio.canSend())
	// 	{
	// 	    printf("Packet %d sent, Ack ok!\r\n",i++);
	// 	    radio.send((uint8_t)GATEWAY_ID, msgBuf, 3);
	// 	}
	// 	else printf("Packet %d sent, no Ack!\r\n",i++);
		
	// 	wait(3);
 //  		led1 = !led1;
	// }

}

// main() runs in its own thread in the OS
int main() {
    rfm69_thread.start(&rfm69_tx_run);
    wait(osWaitForever);
    // while (true) {
    // 	// printf("Hello Worl1d!\n\r");
    //     led1 = !led1;
    //     wait(1);
    // }
}

