/* Copyright 2014, Mariano Cerdeiro
 * Copyright 2014, Pablo Ridolfi
 * Copyright 2014, Juan Cecconi
 * Copyright 2014, Gustavo Muro
 *
 * This file is part of CIAA Firmware.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/** \brief Blinking_echo example source file
 **
 ** This is a mini example of the CIAA Firmware to test the periodical
 ** task excecution and serial port funcionality.
 ** To run this sample in x86 plataform you must enable the funcionality of
 ** uart device setting a value of une or more of folowing macros defined
 ** in header file modules/plataforms/x86/inc/ciaaDriverUart_Internal.h
 **/

/** \addtogroup CIAA_Firmware CIAA Firmware
 ** @{ */
/** \addtogroup Examples CIAA Firmware Examples
 ** @{ */
/** \addtogroup Blinking Blinking_echo example source file
 ** @{ */

/*
 * Initials     Name
 * ---------------------------
 * MaCe         Mariano Cerdeiro
 * PR           Pablo Ridolfi
 * JuCe         Juan Cecconi
 * GMuro        Gustavo Muro
 */

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * 20141019 v0.0.2   JuCe add printf in each task,
 *                        remove trailing spaces
 * 20140731 v0.0.1   PR   first functional version
 */

/*==================[inclusions]=============================================*/
#include "os.h"               /* <= operating system header */
//#include "ciaaPOSIX_stdio.h"  /* <= device handler header */
//#include "ciaaPOSIX_string.h" /* <= string header */
//#include "ciaak.h"            /* <= ciaa kernel header */
#include "blinking_echo_sapi.h"         /* <= own header */
#include "sAPI.h"

/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/
/** \brief File descriptor for digital input ports
 *
 * Device path /dev/dio/in/0
 */
uint8_t dato  = 0;
static int32_t fd_in;

/** \brief File descriptor for digital output ports
 *
 * Device path /dev/dio/out/0
 */
static int32_t fd_out;

/** \brief File descriptor of the USB uart
 *
 * Device path /dev/serial/uart/1
 */
static int32_t fd_uart1;

/** \brief File descriptor of the RS232 uart
 *
 * Device path /dev/serial/uart/2
 */
static int32_t fd_uart2;

/** \brief Periodic Task Counter
 *
 */
static uint32_t Periodic_Task_Counter;

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/
/** \brief Main function
 *
 * This is the main entry point of the software.
 *
 * \returns 0
 *
 * \remarks This function never returns. Return value is only to avoid compiler
 *          warnings or errors.
 */
int main(void)
{
   /* Starts the operating system in the Application Mode 1 */
   /* This example has only one Application Mode */
   StartOS(AppMode1);

   /* StartOs shall never returns, but to avoid compiler warnings or errors
    * 0 is returned */
   return 0;
}

/** \brief Error Hook function
 *
 * This fucntion is called from the os if an os interface (API) returns an
 * error. Is for debugging proposes. If called this function triggers a
 * ShutdownOs which ends in a while(1).
 *
 * The values:
 *    OSErrorGetServiceId
 *    OSErrorGetParam1
 *    OSErrorGetParam2
 *    OSErrorGetParam3
 *    OSErrorGetRet
 *
 * will provide you the interface, the input parameters and the returned value.
 * For more details see the OSEK specification:
 * http://portal.osek-vdx.org/files/pdf/specs/os223.pdf
 *
 */
void ErrorHook(void)
{
   ciaaPOSIX_printf("ErrorHook was called\n");
   ciaaPOSIX_printf("Service: %d, P1: %d, P2: %d, P3: %d, RET: %d\n", OSErrorGetServiceId(), OSErrorGetParam1(), OSErrorGetParam2(), OSErrorGetParam3(), OSErrorGetRet());
   ShutdownOS(0);
}

/** \brief Initial task
 *
 * This task is started automatically in the application mode 1.
 */
TASK(InitTask)
{

   /* activate example tasks */
	 /* Inicializar la placa */
	   boardConfig();

	   /* Configuración de pines de entrada para Teclas de la CIAA-NXP */
	      digitalConfig( TEC1, INPUT );
	      digitalConfig( TEC2, INPUT );
	      digitalConfig( TEC3, INPUT );
	      digitalConfig( TEC4, INPUT );

	      /* Configuración de pines de salida para Leds de la CIAA-NXP */
	         digitalConfig( LEDR, OUTPUT );
	         digitalConfig( LEDG, OUTPUT );
	         digitalConfig( LEDB, OUTPUT );
	         digitalConfig( LED1, OUTPUT );
	         digitalConfig( LED2, OUTPUT );
	         digitalConfig( LED3, OUTPUT );

	         /* Inicializar UART_USB a 115200 baudios */
	            uartConfig( UART_USB, 115200 );



   //Periodic_Task_Counter = 0;
   //SetRelAlarm(SerialEchoTask, 200, 200);

   /* Activates the SerialEchoTask task */
   ActivateTask(SerialEchoTask);

   /* end InitTask */
   TerminateTask();
}

/** \brief Serial Echo Task
 *
 * This tasks waits for input data from fd_uart1 and writes the received data
 * to fd_uart1 and fd_uart2. This taks alos blinkgs the output 5.
 *
 */
TASK(SerialEchoTask)
{
   int8_t buf[20];   /* buffer for uart operation              */
   uint8_t outputs;  /* to store outputs status                */
   int32_t ret;      /* return value variable for posix calls  */


   while(1)
   {


       uint8_t word[4];
	   /* Recibir byte de la UART_USB y guardarlo en la variable dato */
	        dato = uartReadByte( UART_USB );

	        if( dato ){
	                uartWriteByte( UART_USB, dato ); //hago eco de lo que recibe la uart


	             }





            //enciendo leds de acuerdo a la letra ingresada (letras son las iniciales de cada led)
	        if( dato== 'a' ){
	        	digitalWrite( LEDB, ON );
	        	digitalWrite (LEDR, OFF);
	        	digitalWrite (LEDG, OFF);
	        	digitalWrite(LED1, OFF);
	        	digitalWrite(LED2, OFF);
	        	digitalWrite(LED3, OFF);
	        }
	        if( dato== 'r' ){
	        	digitalWrite( LEDR, ON );
	        	digitalWrite(LEDB, OFF);
	        	digitalWrite(LEDG, OFF);
	        	digitalWrite(LED1, OFF);
	        	digitalWrite(LED2, OFF);
	        	digitalWrite(LED3, OFF);
	        }

	        if( dato== 'v' ){
	       	       digitalWrite( LEDG, ON );
	       	       digitalWrite(LEDB, OFF);
	       	       digitalWrite(LEDR, OFF);
	       	       digitalWrite(LED1, OFF);
	       	       digitalWrite(LED2, OFF);
	       	       digitalWrite(LED3, OFF);
	       	}

	        if( dato== '1' ){
	              digitalWrite(LED1, ON );
	              digitalWrite(LED2, OFF);
	              digitalWrite(LED3, OFF);
	        	  digitalWrite(LEDB, OFF);
	        	  digitalWrite(LEDR, OFF);
	        	  digitalWrite(LEDG, OFF);
	         }

	        if( dato== '2' ){
	        	  digitalWrite(LED2, ON );
	        	  digitalWrite(LED1, OFF);
	        	  digitalWrite(LED3, OFF);
	        	  digitalWrite(LEDB, OFF);
	        	   digitalWrite(LEDR, OFF);
	        	   digitalWrite(LEDG, OFF);
	          }
	        if( dato== '3' ){
	     	        	  digitalWrite(LED3, ON );
	     	        	  digitalWrite(LED2, OFF);
	     	        	  digitalWrite(LED1, OFF);
	     	        	  digitalWrite(LEDB, OFF);
	     	        	   digitalWrite(LEDR, OFF);
	     	        	   digitalWrite(LEDG, OFF);
	     	          }




	   /* wait for any character ... */
     // ret = ciaaPOSIX_read(fd_uart1, buf, 20);

     // if(ret > 0)
      //{
         /* ... and write them to the same device */
        // ciaaPOSIX_write(fd_uart1, buf, ret);

         /* also write them to the other device */
         //ciaaPOSIX_write(fd_uart2, buf, ret);
    //  }

      /* blink output 5 with each loop */
      //ciaaPOSIX_read(fd_out, &outputs, 1);
      //outputs ^= 0x20;
      //ciaaPOSIX_write(fd_out, &outputs, 1);
   }
}

/** \brief Periodic Task
 *
 * This task is activated by the Alarm ActivatePeriodicTask.
 * This task copies the status of the inputs bits 0..3 to the output bits 0..3.
 * This task also blinks the output 4
 */


/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

