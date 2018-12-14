/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# Minimal main function that starts with a call to system_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */
#include <asf.h>
#include <stdio.h>
#include <string.h>

//Debug Serial Port Hardware Definition
#define EDBG_CDC_SERCOM_MUX_SETTING USART_RX_3_TX_2_XCK_3
#define EDBG_CDC_SERCOM_PINMUX_PAD0 PINMUX_UNUSED
#define  EDBG_CDC_SERCOM_PINMUX_PAD1 PINMUX_UNUSED
#define EDBG_CDC_SERCOM_PINMUX_PAD2 PINMUX_PB22D_SERCOM5_PAD2
#define  EDBG_CDC_SERCOM_PINMUX_PAD3 PINMUX_PB23D_SERCOM5_PAD3
#define EDBG_CDC_MODULE SERCOM5

//LIDAR Serial Hardware Configuration
#define LIDAR_CDC_SERCOM_MUX_SETTING USART_RX_3_TX_2_XCK_3
#define LIDAR_CDC_SERCOM_PINMUX_PAD0 PINMUX_UNUSED
#define  LIDAR_CDC_SERCOM_PINMUX_PAD1 PINMUX_UNUSED
#define LIDAR_CDC_SERCOM_PINMUX_PAD2 PINMUX_PA10D_SERCOM2_PAD2
#define  LIDAR_CDC_SERCOM_PINMUX_PAD3 PINMUX_PA11D_SERCOM2_PAD3
#define LIDAR_CDC_MODULE SERCOM2

//TCC Configuration
#define CONF_PWM_MODULE      TCC0
#define CONF_PWM_CHANNEL     0
#define CONF_PWM_OUTPUT      0
#define CONF_PWM_OUT_PIN     PIN_PA08
#define CONF_PWM_OUT_MUX     PINMUX_PA08E_TCC0_WO0

//PID Configuration
#define kp					 -20.0f
#define ki					 0.0f
#define kd					 0.1f
#define target_position		 40
#define MAX_PWM				1000

//Global FreeRTOS object initializations
xSemaphoreHandle rx_semaphore;
xTimerHandle rx_timer_id;
xTaskHandle rx_job_id;
xTaskHandle get_pid_id;
xTaskHandle set_pwm_id;
xQueueHandle lidar_distance_queue;
xQueueHandle pwm_pid_queue;
//Global Queue Size Definition, Arbitrary
#define DEFAULT_QUEUE_SIZE 10
//LIDAR Packet Length according to TFmini Datasheet
#define LIDAR_PACKET_LENGTH 9
//RX Buffer Length, to guarantee packet capture
#define MAX_RX_BUFFER_LENGTH   LIDAR_PACKET_LENGTH*2
//Global RX Buffer for callback mode
volatile uint8_t lidar_buffer[MAX_RX_BUFFER_LENGTH];
//LIDAR Packet Header according to TFmini Datasheet
const uint8_t HEADER=0x59;

//Handlers for Atmel Peripheral objects
struct usart_module usart_instance_debug;
struct usart_module usart_instance_lidar;
struct tcc_module tcc_instance;

//Some prototypes for compiler
void usart_read_callback(struct usart_module *const usart_module);
void configure_usart_callbacks(void);

//Distance Buffer for LIDAR Callback and Queue input
uint16_t distance_buffer;

static void configure_tcc(void)
{
	//Configure PWM, using TCC peripheral
	struct tcc_config config_tcc;
	tcc_get_config_defaults(&config_tcc, CONF_PWM_MODULE);
	config_tcc.counter.clock_prescaler = TCC_CLOCK_PRESCALER_DIV1;
	config_tcc.counter.period = MAX_PWM;
	config_tcc.compare.wave_generation = TCC_WAVE_GENERATION_SINGLE_SLOPE_PWM;
	config_tcc.compare.match[CONF_PWM_CHANNEL] = MAX_PWM/2;
	config_tcc.pins.enable_wave_out_pin[CONF_PWM_OUTPUT] = true;
	config_tcc.pins.wave_out_pin[CONF_PWM_OUTPUT]        = CONF_PWM_OUT_PIN;
	config_tcc.pins.wave_out_pin_mux[CONF_PWM_OUTPUT]    = CONF_PWM_OUT_MUX;
	tcc_init(&tcc_instance, CONF_PWM_MODULE, &config_tcc);
	tcc_enable(&tcc_instance);
}

void configure_usart_debug(void){

	//Configure debugging serial port, not used in final implementation
	struct usart_config config_usart;
	usart_get_config_defaults(&config_usart);
	config_usart.baudrate    = 9600;
	config_usart.mux_setting = EDBG_CDC_SERCOM_MUX_SETTING;
	config_usart.pinmux_pad0 = EDBG_CDC_SERCOM_PINMUX_PAD0;
	config_usart.pinmux_pad1 = EDBG_CDC_SERCOM_PINMUX_PAD1;
	config_usart.pinmux_pad2 = EDBG_CDC_SERCOM_PINMUX_PAD2;
	config_usart.pinmux_pad3 = EDBG_CDC_SERCOM_PINMUX_PAD3;
	stdio_serial_init(&usart_instance_debug,EDBG_CDC_MODULE, &config_usart);
	usart_enable(&usart_instance_debug);
}


void configure_usart_callbacks(void)
{
	//setup callback function to call the handler for the RX buffer
	usart_register_callback(&usart_instance_lidar,
	usart_read_callback, USART_CALLBACK_BUFFER_RECEIVED);
	usart_enable_callback(&usart_instance_lidar, USART_CALLBACK_BUFFER_RECEIVED);
}
	
void configure_usart_lidar(void){

	//Setup LIDAR sensor serial line
	struct usart_config config_usart;
	usart_get_config_defaults(&config_usart);
	config_usart.baudrate    = 115200;
	config_usart.mux_setting = LIDAR_CDC_SERCOM_MUX_SETTING;
	config_usart.pinmux_pad0 = LIDAR_CDC_SERCOM_PINMUX_PAD0;
	config_usart.pinmux_pad1 = LIDAR_CDC_SERCOM_PINMUX_PAD1;
	config_usart.pinmux_pad2 = LIDAR_CDC_SERCOM_PINMUX_PAD2;
	config_usart.pinmux_pad3 = LIDAR_CDC_SERCOM_PINMUX_PAD3;
	while (usart_init(&usart_instance_lidar,LIDAR_CDC_MODULE, &config_usart) != STATUS_OK) {
	}
	usart_enable(&usart_instance_lidar);
	//Configure callbacks in another function
	configure_usart_callbacks();
}

void get_pid_task(void * pvParameters )
{
	//Initialize variables that will be maintained over the entirety of task lifetime
	int integral = 0;
	//Used for derivative calculation
	int last_error = 0;
	int dist = 0;
	
	//Main task loop
	for(;;)
	{
		//Blocking Queue receive function, will unblock when data is ready, allows synchronization
		xQueueReceive(lidar_distance_queue, &dist, portMAX_DELAY);
		//Get error value
		int error = target_position - dist;
		//Add to integral 
		integral += error;
		//Get derivative
		int derivative = error - last_error;
		//Reset last error with current error
		last_error = error;
		//Get PID output
		int pid_value = (kp * error) + (ki * integral) * (kd * derivative);
		//Ensure that PID output is bounded to values within the PWMs range
		if(pid_value > MAX_PWM - 1) pid_value = MAX_PWM -1;
		else if(pid_value < 0) pid_value = 0;
		//Send data to PWM queue, which will allow the PWM task to unblock
		xQueueSend(pwm_pid_queue, &pid_value, portMAX_DELAY);
	}

}

void usart_read_callback(struct usart_module *const usart_module)
{
	int i = 0;
	
	//Increment until packet header is reached
	for(i = 0; i <= LIDAR_PACKET_LENGTH; i++)
	{
		//Check first and second bytes at index for two header bytes
		if(lidar_buffer[i] == HEADER && lidar_buffer[i+1] == HEADER)
		{
			//Calculate Checksum in packet
			uint8_t check =HEADER+HEADER+lidar_buffer[i+2]+lidar_buffer[i+3]+lidar_buffer[i+4]+lidar_buffer[i+5]+lidar_buffer[i+6]+lidar_buffer[i+7];
			//Compare Checksum to packet
			if(check==lidar_buffer[i + 8])
			{
				//Concatenate two bytes into short value for distance
				distance_buffer = lidar_buffer[i+3] << 8;
				distance_buffer += lidar_buffer[i+2];
				//Add new calculated distance to distance data queue
				xQueueSendFromISR(lidar_distance_queue, &distance_buffer, NULL);
			}
		}
	}
}

//Task that checks for active jobs, and calls new ones if none are active
void rx_job_task(void *pvParameters)
{
	for(;;)
	{
		//Checks if job is currently being executed
		if(!(usart_get_job_status(&usart_instance_lidar, USART_TRANSCEIVER_RX) == STATUS_BUSY))
		{
			//Starts knew read job to fill buffer
			usart_read_buffer_job(&usart_instance_lidar, lidar_buffer, MAX_RX_BUFFER_LENGTH);
		}
		//Suspend itself until called by timer 
		vTaskSuspend(NULL);
	}
}

//Task that handles PWM output
void set_pwm_task( void *pvParameters)
{
	int pwm;
	for(;;)
	{
		//Blocking queue, that only unblocks when PWM data is available
		xQueueReceive(pwm_pid_queue, &pwm, portMAX_DELAY);
		//Set pwm pin to provided duty cycle. 
		tcc_set_compare_value(&tcc_instance,CONF_PWM_CHANNEL, pwm);
	}
}

//Callback Function for RX Timer
void rx_timer_callback()
{
	//Re enable Task that starts USART Read Jobs
	vTaskResume(rx_job_id);
}

int main (void)
{
	system_init();
	board_init();
	//Enable Tracing Mode, for Data Acquisition
	vTraceEnable(TRC_START);
	//Configure Serial IO
	configure_usart_debug();
	configure_usart_lidar();
	//Configure PWM Peripheral
	configure_tcc();
	
	//Create Queue Objects
	pwm_pid_queue = xQueueCreate(DEFAULT_QUEUE_SIZE, sizeof (int));
	lidar_distance_queue = xQueueCreate(DEFAULT_QUEUE_SIZE, sizeof (int));
	
	//Create Timer Object
	rx_timer_id = xTimerCreate("RX Timer",25,1,0, rx_timer_callback);
	
	//Create Task Objects
	xTaskCreate(rx_job_task, "Trigger RX", configMINIMAL_STACK_SIZE+1000, NULL, 1, &rx_job_id);
	xTaskCreate(set_pwm_task, "Set PWM", configMINIMAL_STACK_SIZE+1000, NULL, 2, &set_pwm_id);
	xTaskCreate(get_pid_task, "Get PID", configMINIMAL_STACK_SIZE+1000, NULL, 2, &get_pid_id);
	
	//Start Timer Object
	xTimerStart(rx_timer_id, 0);
	
	//Start FreeRTOS
	vTaskStartScheduler();
	
	while (1);
	return 0;
}


