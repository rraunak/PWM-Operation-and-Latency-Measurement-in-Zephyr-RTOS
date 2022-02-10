******************************************************************************
**********************CSE522-Real Time Embedded Systems**********************
*****************************Assignment-1***************************************


Name - Raunak
ASU ID - 1217240245

Description : 

In this assignment, we are configuring the pwm signals to control the brightness of the RGB LED, we are also measuring the interrupt latency and context switch latency in the zephyr rtos. All the functions are performed through the shell command.



******************************************************************************
*******************Steps to compile and execute the code*********************



1. Copy the  RTES-Raunak_01.zip file in the zephyr/samples directory.

2. Unzip the RTES-Raunak_01.zip in the zephyr/samples directory.

3. start the terminal go to zephyr folder and set the environment by the following commands,

	(i)   source zephyr-env.sh
	(ii)  export ZEPHYR_TOOLCHAIN_VARIANT=zephyr
	(iii) export ZEPHYR_SDK_INSTALL_DIR=Zephyr SDK installation directory
	
4. Go to zephyr/samples/project1_45 folder

5. Create a folder named build using mkdir build command.

6. Go inside the folder build using cd build command.

7. Set the cmake environment using the following command,

	cmake -DBOARD=galileo ..
	
8. Then type the following command to make the binary,

	make
	
9. Now from the zephyr directory created inside build copy the zephyr.strip file to the kernel folder of SD card.

10. Now, assuming you sd card is prepared with bootia32.efi file and config file. Insert your sd card to the galileo board.

11. Now boot from SD card and load the zephyr kernel.

12. Now connect the ftdi cable and do chmod 777 tty/USB0.

13. start putty, set serial communication to 115200 and tty/USB0 and load.

14. Load the zephyr kernel, led with full brightness will be on.

15. To observe the change in brightness type the folllowing command,

	project1 RGB-display x y z,  where x y z aare brightness percentages
	
16. To observe interrupt latency type the following command,

	project1 int-latency n,  where n is the number of samples of measurement to be taken
	
17. To observe context switch latency type the following command,

	project1 cs-latency n,  where n is the number of samples of measurement to be taken
	
	
	
*******************************************************************************
******************************Sample Output**********************************



uart:~$ project1 RGB-display 10 10 10
uart:~$ project1 RGB-display 10 10 90
uart:~$ project1 RGB-display 10 90 10
uart:~$ project1 int-latency 2
 ---- Average Interrupt Latency of 2 interrupts(in ns): 2533

uart:~$ project1 int-latency 7
 ---- Average Interrupt Latency of 7 interrupts(in ns): 2094

uart:~$ project1 int-latency 10
 ---- Average Interrupt Latency of 10 interrupts(in ns): 1973

uart:~$ project1 int-latency 11
 ---- Average Interrupt Latency of 11 interrupts(in ns): 2343

uart:~$ project1 int-latency 3
 ---- Average Interrupt Latency of 3 interrupts(in ns): 3784

uart:~$ project1 cs-latency 3
 ---- Average Context Switch Latency of 3 samples (in ns): 3494

uart:~$ project1 cs-latency 2
 ---- Average Context Switch Latency of 2 samples (in ns): 3304

uart:~$ project1 cs-latency 9
 ---- Average Context Switch Latency of 9 samples (in ns): 3349

uart:~$ project1 cs-latency 11
 ---- Average Context Switch Latency of 11 samples (in ns): 3353

