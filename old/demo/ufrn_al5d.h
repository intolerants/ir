/*************************************************************/
/*       UNIVERSIDADE FEDERAL DO RIO GRANDE DO NORTE         */
/*   DEPARTAMENTO DE ENGENHARIA DE COMPUTAÇÃO E AUTOMAÇÃO    */
/*							     */
/*   DRIVER DO BRAÇO ROBÓTICO LYNX AL5D PARA A PORTA SERIAL  */
/*							     */
/*   DESENVOLVEDORES:					     */
/*	- ENG. M.SC. DESNES AUGUSTO NUNES DO ROSÁRIO	     */
/*	- ENG. DANILO CHAVES DE SOUSA ICHIHARA		     */
/*************************************************************/

#ifndef _UFRN_AL5D_H_
#define _UFRN_AL5D_H_

	#include <stdio.h>
	#include <string.h>
	#include <unistd.h>
	#include <fcntl.h>
	#include <termios.h>

	#define BUFSIZE 1024

	// SERVOS //

	// Servo da base HS-485HB //
	#define BAS_SERVO 0

	// Servo do ombro HS-805BB //
	#define SHL_SERVO 1

	// Servo do cotovelo HS-755HB//
	#define ELB_SERVO 2

	// Servo do punho HS-645MG //
	#define WRI_SERVO 3
	
	// Servo da garra HS-322HD //
	#define GRI_SERVO 4

	int configurar_porta(int);
	int abrir_porta(void);
	int enviar_comando(char*,int);

	void fechar_porta(int);
	void ufrn_header(void);

#endif

