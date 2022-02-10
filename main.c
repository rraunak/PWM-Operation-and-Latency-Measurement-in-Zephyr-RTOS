/*
 * Copyright (c) 2016 Open-RnD Sp. z o.o.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <gpio.h>
#include <pwm.h>
#include <misc/util.h>
#include <misc/printk.h>
#include <misc/__assert.h>
#include <shell/shell.h>
#include <shell/shell_uart.h>
#include <version.h>
#include <pinmux.h>
#include "../boards/x86/galileo/board.h"
#include "../boards/x86/galileo/pinmux_galileo.h"
#include "../drivers/gpio/gpio_dw_registers.h"
#include "../drivers/gpio/gpio_dw.h"
#include "../drivers/pwm/pwm_pca9685.h"
#include <stdlib.h>
#include <string.h>
/* Define Debug to see the Dprint statement to debug */
//#define DEBUG

#if defined(DEBUG) 
	#define DPRINTK(fmt, args...) printk("DEBUG: %s():%d: " fmt, \
   		 __func__, __LINE__, ##args)
#else
 	#define DPRINTK(fmt, args...) /* do nothing if not defined*/
#endif
/*  define rising and falling edge  */
#define EDGE_FALLING    (GPIO_INT_EDGE | GPIO_INT_ACTIVE_LOW)
#define EDGE_RISING		(GPIO_INT_EDGE | GPIO_INT_ACTIVE_HIGH)

/* change this to enable pull-up/pull-down */
#define PULL_UP 0

/* Sleep time */
#define SLEEP_TIME	1000
/* stack size */
#define STACK_SIZE 500
/* priority of different threads */
#define PRIORITYBLOCKING 0
#define PRIORITYSWITCHING 7
#define PRIORITYDELAY 14

/* define stack size for each thread */
K_THREAD_STACK_DEFINE(my_stack_area1, STACK_SIZE);
K_THREAD_STACK_DEFINE(my_stack_area2, STACK_SIZE);
K_THREAD_STACK_DEFINE(my_stack_area3, STACK_SIZE);

/* define mutex for each operation */
struct k_mutex my_mutex;
struct k_mutex mutex_int;
struct k_mutex mutex_cntxt;

/* define semaphore for each operation */
struct k_sem my_sem;
struct k_sem my_sem2;
struct k_sem my_sem3;

/* define struct device variable */
struct device *gpiob;

/* define diiferent variables to measure the two latencies */
int startInt, stopInt, startCtxt, stopCtxt, startDelayInt, stopDelayInt, delayStart, delayStop;
int intno=0;		// number of interrupts
int cntxtno=0;      // number of context switches
int sum=0;          // sum to store total interrupt time
int sumcntxt=0;     // sum to store total context switch time

static struct gpio_callback gpio_cb;         //structure for callback function defined
static struct device *pinmux;                //variable of type struct device defined
struct device *pwm;                          //variable for pwn struct device type defined
int numSamples=0;                            //store total number of interrupt samples to be taken
int numSamplesCntxt=0;                       //store total number of context switches to be taken

void blockingThread(void*, void*, void*);    //Highest priority thread
void switchingThread(void*, void*, void* );  //Medium priority thread
void delayThread(void*, void*, void* );      //Lowest priority thread

struct k_thread my_thread_data1;             //struct kthread for highest priority thread defined
struct k_thread my_thread_data2;             //struct kthread for medium priority thread defined
struct k_thread my_thread_data3;             //struct kthread for lowest priority thread defined

/* x86 read time stamp counter defined */
static __inline__ unsigned long long RDTSC(void)
{
    unsigned hi, lo;
    __asm__ __volatile__ ("rdtsc" : "=a"(lo), "=d"(hi));
    return ( (unsigned long long)lo)|( ((unsigned long long)hi)<<32 );
}

/* Function for Highest priority thread defined */
void blockingThread(void *a, void *b, void *c){

        DPRINTK(" Thread1 start\n");
        k_sem_take(&my_sem, K_FOREVER);                   //wait for signal from switching thread
        DPRINTK(" Thread1 waits for mutex unlock\n");
        k_mutex_lock(&my_mutex, K_FOREVER);               //wait till the switching thread unlocks the mutex
        DPRINTK(" Thread1 unlocks\n");
        stopCtxt = RDTSC();                               //take the final measurement
        DPRINTK(" Thread1 switch to thread 2\n");
        k_sem_give(&my_sem3);                             //give the signal to delayThread to unlock
        k_mutex_unlock(&my_mutex);                        //switch to switching thread


}

/* Function for Medium priority thread defined */
void switchingThread(void *a, void *b, void *c) {

    DPRINTK(" Thread2 start\n");
    DPRINTK(" Thread2 switch from thread 1\n");
    k_mutex_lock(&my_mutex, K_FOREVER);                   //take the mutex at start
    k_sem_give(&my_sem);                                  //give the signal to blocking thread to wait on mutex
    DPRINTK(" Thread2 takes lock\n");
    startCtxt = RDTSC();                                  //take the initial measurement
    DPRINTK(" Thread2 switch to thread 1\n");
    k_mutex_unlock(&my_mutex);                            //unlock and switch to blocking thread
    DPRINTK(" Thread2 waits for thread 1\n");
    k_mutex_lock(&my_mutex, K_FOREVER);                   //wait for blocking thread to finish
    DPRINTK(" Thread2 try to switch to thread 3\n");
    delayStart = RDTSC();                                 //take the initial delay measurement
    k_mutex_unlock(&my_mutex);                            //try to context switch to delay thread but will not due to higher priority
    DPRINTK(" Thread2 remains in execution due to higher priority\n");
    delayStop = RDTSC();                                  //take the final delay measurement

    int interval;
    cntxtno++;                                            //increase te context switch counter
    interval = (((stopCtxt-startCtxt)-(delayStop - delayStart))*1000)/400;  //subtract the delay measurement and calculate the actual measurement
    sumcntxt = sumcntxt + interval;                                         //add the time to sum value
    DPRINTK(" Context switched %d\n", cntxtno);


}

/* Function for Lowest priority thread defined */
void delayThread(void *a, void *b, void *c) {

    DPRINTK(" Thread3 start\n");
    k_sem_take(&my_sem3, K_FOREVER);                        //wait for the signal from blocking thread
    k_mutex_lock(&my_mutex, K_FOREVER);                     //wait till all thread finish
    DPRINTK(" Thread3 unlocks\n");
    k_mutex_unlock(&my_mutex);                              //unlocks and continue
    DPRINTK(" Thread3 signals to shell to continue\n");
    k_sem_give(&my_sem2);                                   //signal the shell to continue to continue the measurement process

}

/* Function for interrupt callback function defined */
void interrupt_cb(struct device *gpiob, struct gpio_callback *cb,
		    u32_t pins)
{
	u32_t val;
	int interval;
    intno++;
    gpio_pin_read(gpiob, 7, &val);                           //read the pin value
    stopInt = RDTSC();                                       //calculate the final measurement
    interval = (((stopInt-startInt)-(stopDelayInt-startDelayInt))*1000)/400;  //subtract the delay and calculate the final measurement
    sum = sum + interval;                                                     //calculate the total sum
    DPRINTK(" Interrupted %d\n", intno);

}


/*Shell function for calculating context switch latency defined */
static int cmd_cs_latency(const struct shell *shell, size_t argc, char **argv)
{
    int res;

    /* Initialize mutex */
    k_mutex_init(&my_mutex);
    k_mutex_init(&mutex_cntxt);

    /* Initialize semaphore */
    k_sem_init(&my_sem, 0, 1);
    k_sem_init(&my_sem2, 0, 1);
    k_sem_init(&my_sem3, 0, 1);

    /* Take the lock so that other measurements should block*/
    k_mutex_lock(&mutex_cntxt, K_FOREVER);
    int num = atoi(argv[1]);                              //take the measurement count
    numSamplesCntxt = num;
    /* Create three threads to measure context switch latency */
    for(int i=0; i<num; i++) {


        k_thread_create(&my_thread_data1, my_stack_area1,
                        K_THREAD_STACK_SIZEOF(my_stack_area1),
                        blockingThread,
                        NULL, NULL, NULL,
                        PRIORITYBLOCKING, 0, K_NO_WAIT);

        k_thread_create(&my_thread_data2, my_stack_area2,
                        K_THREAD_STACK_SIZEOF(my_stack_area2),
                        switchingThread,
                        NULL, NULL, NULL,
                        PRIORITYSWITCHING, 0, K_NO_WAIT);

        k_thread_create(&my_thread_data3, my_stack_area3,
                        K_THREAD_STACK_SIZEOF(my_stack_area3),
                        delayThread,
                        NULL, NULL, NULL,
                        PRIORITYDELAY, 0, K_NO_WAIT);



        k_sem_take(&my_sem2, K_FOREVER);                        //wait till all the threads finish

    }
    /* Calculate and display the average measurement*/
    res = sumcntxt/numSamplesCntxt;
    shell_print(shell," ---- Average Context Switch Latency of %d samples (in ns): %d \n", numSamplesCntxt, res);
    /* Initialize variables to 0 for the next measurement */
    cntxtno=0;
    sumcntxt=0;
    k_mutex_unlock(&mutex_cntxt);                              //unlock the mutex for next shell command
	return 0;
}

/*Shell function for changing the brightness of LED through PWM defined */
static int cmd_RGB_display(const struct shell *shell, size_t argc, char **argv)
{

    /* Calculate the actual cycles from percentage of brightness required */
	int periodR = 100/(atoi(argv[1]));
	periodR = 4096/periodR;
	int periodG = 100/(atoi(argv[2]));
	periodG = 4096/periodG;
	int periodB = 100/(atoi(argv[3]));
	periodB = 4096/periodB;

	/* Set the brightness to required level */
    pwm_pin_set_cycles(pwm, 3, 4096, periodR);
    pwm_pin_set_cycles(pwm, 5, 4096, periodG);
    pwm_pin_set_cycles(pwm, 7, 4096, periodB);

	return 0;
}

/*Shell function for calculating interrupt latency defined */
static int cmd_int_latency(const struct shell *shell, size_t argc, char **argv)
{
    int res;

    /* Initialize the mutex*/
    k_mutex_init(&mutex_int);

    /* Take the lock so that other measurements should block*/
    k_mutex_lock(&mutex_int, K_FOREVER);
    
    
    /* disable callback to take delay */
    gpio_pin_disable_callback(gpiob, 7);
    
    
    /* Calculate the delay measurement for interrupt */
    gpio_pin_write(gpiob, 6, 0);
    startDelayInt = RDTSC();
    gpio_pin_write(gpiob, 6, 1);
    stopDelayInt = RDTSC();
    gpio_pin_write(gpiob, 6, 0);
    
    /*  Enable callback again after the delay measurement */
    gpio_pin_enable_callback(gpiob, 7);

    numSamples = atoi(argv[1]);                              //take the measurement count

    /* Trigger the interrupt the time specified by the user*/
	for(int i=0; i<numSamples; i++) {
        gpio_pin_write(gpiob, 6, 0);
        startInt = RDTSC();
        gpio_pin_write(gpiob, 6, 1);
        gpio_pin_write(gpiob, 6, 0);
    }
    /* Calculate and display the average measurement*/
    res = sum/numSamples;
    shell_print(shell, " ---- Average Interrupt Latency of %d interrupts(in ns): %d \n",numSamples, res);
    /* Initialize variables to 0 for the next measurement */
    intno=0;
    sum=0;
    k_mutex_unlock(&mutex_int);                                  //unlock the mutex for next shell command
	return 0;
}

/* Shell commands for RGB display, interrupt and context switch defined */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_project,
	SHELL_CMD(RGB-display, NULL, "Project-1 RGB-display x y z", cmd_RGB_display),
	SHELL_CMD(int-latency, NULL, "project1 int-latency n", cmd_int_latency),
    SHELL_CMD(cs-latency, NULL, "project1 cs-latency n", cmd_cs_latency),
	SHELL_SUBCMD_SET_END /* Array terminated. */
);
SHELL_CMD_REGISTER(project1, &sub_project, "Project-1 commands", NULL);

/* Start of main function */
void main(void)
{

	int ret;

    pinmux=device_get_binding(CONFIG_PINMUX_NAME);

    struct galileo_data *dev = pinmux->driver_data;    //extract the galileo_data structure from pinmux

    pwm = device_get_binding(CONFIG_PWM_PCA9685_0_DEV_NAME);

    gpiob=dev->gpio_dw; //retrieving gpio_dw driver struct from pinmux driver
                         // Alternatively, gpiob = device_get_binding(PORT);

	if (!gpiob) {
		DPRINTK("error\n");
		return;
	}

    if (!pwm) {
        DPRINTK("error\n");
        return;
    }

    ret=pinmux_pin_set(pinmux,5,PINMUX_FUNC_C); //IO 5 -- gpio8/pwm.led3
    if(ret<0)
        DPRINTK("error setting pin for IO5\n");

    ret=pinmux_pin_set(pinmux,6,PINMUX_FUNC_C); //IO 6  -- gpio9/pwm.led5
    if(ret<0)
        DPRINTK("error setting pin for IO6\n");

    ret=pinmux_pin_set(pinmux,9,PINMUX_FUNC_C); //IO 9  -- gpio_sus2/pwm.led7
    if(ret<0)
        DPRINTK("error setting pin for IO9\n");

    ret=pinmux_pin_set(pinmux,3,PINMUX_FUNC_A); //IO 3 -- gpio6
    if(ret<0)
	DPRINTK("error setting pin for IO3\n");

    gpio_pin_configure(gpiob, 6, GPIO_DIR_OUT); //Configure the gpio6 to out to trigger the interrupt


	ret=pinmux_pin_set(pinmux,12,PINMUX_FUNC_B); //IO12 for input interrupt -- gpio7
	if(ret<0)
		DPRINTK("error setting pin for IO12\n");

	//gpio7 corresponds to input pin for IO12 ..refer boards/x86/galileo/pinmux.c
	ret=gpio_pin_configure(gpiob, 7, GPIO_DIR_IN | GPIO_INT | EDGE_RISING);
	
	gpio_init_callback(&gpio_cb, interrupt_cb, BIT(7));

	/* add callback*/
	ret=gpio_add_callback(gpiob, &gpio_cb);
	if(ret<0)
		DPRINTK("error adding callback\n");

	/* enable callback */
	ret=gpio_pin_enable_callback(gpiob, 7);
	if(ret<0)
		DPRINTK("error enabling callback\n");


	/* Set the LED to 100% initially */
    pwm_pin_set_cycles(pwm, 3, 4096, 4096);
    pwm_pin_set_cycles(pwm, 5, 4096, 4096);
    pwm_pin_set_cycles(pwm, 7, 4096, 4096);

}
