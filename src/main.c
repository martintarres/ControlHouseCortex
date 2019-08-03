
/**
  ******************************************************************************
  * @file    appControlHouseM3.c
  * @author  Tarres Martin
  * @version V1.0
  * @date    29-June-2019
  ******************************************************************************
*/

#include "stm32f10x.h"
#include "stdio.h"
#include <string.h>

/** palabras de configuracion
  ******************************************************************************
  * conf -> configuracion claves
  * outc -> salgo modo configuracion
  * tim1 -> 3 segundos
  * tim2 -> 5 segundos
  * tim3 -> 8 segundos
  * tim4 -> 10 segundos
  * tim5 -> 15 segundos
  * s1ac -> activo sensor 1
  * s1de -> desactivo sensor 1
  * alac -> activo alarma total
  * alde -> desactivo alarma total
  * maac -> activo alarma magnetico
  * made -> desactivo alarma magnetico
  * l1pr -> prendo luces 1
  * l1ap -> apago luces 1
  * l2pr -> prendo luces 2
  * l2ap -> apago luces 2
  * okok -> saber si esta conectado a BT
  *
  * LOS SENSORES MAGNETICOS VAN CONECTADOS A POSITIVO 3.3V
  ******************************************************************************
*/

//funciones configuracion
void configClock(void);
void configPines(void);
void configUsart(void);
void cambioLed(void);
void configDMA(void);
void configTimer(void);
void configPinesExternalInterrupt(void);
void configSysTick(void);



//funciones acciones
void configuracionTiempoSonar(char*);
int stateChange(int);
void activarSirena();

//variables
int a;
volatile char bufferRX[80] = {'\0'};			// BUFFER RECEPCION DMA
volatile char bufferTX[80] = {'\0'};			//BUFFER TRANSIMISON DMA
int flagConf=0;									//BANDERA QUE INDICA QUE ESTOY EN MODO CONFIGURACION
int flagAlarmaActivaMovimiento=0;				//BANDERA QUE INDICA SI LA ALARMA ESTA ACTIVA O NO
int flagAlarmaActivaMagnetico=0;				//BANDERA QUE INDICA SI LA ALARMA ESTA ACTIVA O NO
int tiempoTimer;								//TIEMPO QUE SE LE SETEA AL TIMER
int tiempoTimer3;

//ESTADO DE LOS PUERTOS
int a0=0;
int a1=0;
int a2=0;
int a3=0;
int a4=0;

int main(){

	configClock();
	configPines();
	configUsart();
	configDMA();
	configTimer();
	configPinesExternalInterrupt();
	configSysTick();

	while(1){
		a++;
		a++;
		a++;
		a++;
		a++;
	}
}

void configClock(){
	// HABILITAMOS CLOCK DEL PUERTO C
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
	//HABILITO CLOCK PUERTO A
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	//HABILITO CLOCK PUERTO B
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
	// HABILITO CLOCK PARA USART2
	RCC -> APB2ENR |= RCC_APB2ENR_USART1EN;
	// HABILITO CLOCK DE AFIO
	RCC -> APB2ENR |= RCC_APB2ENR_AFIOEN;
	// HABILITO CLOCK DMA
	RCC -> AHBENR |= RCC_AHBPeriph_DMA1;
	//HABILITO CLOCK PARA TIMER2
	RCC -> APB1ENR |= RCC_APB1ENR_TIM2EN;
	//HABILITO CLOCK PARA TIMER3
	RCC -> APB1ENR |= RCC_APB1ENR_TIM3EN;
	/*Divido el AHB por 4. Por default viene en 72MHz.*/
	RCC -> CFGR |= RCC_CFGR_HPRE_3;
	RCC -> CFGR |= RCC_CFGR_HPRE_1;
}

void configPines(){
	// PONGO COMO SALIDA PIN 13
	GPIOC->CRH |= GPIO_CRH_MODE13_0;
	// PONGO COMO SALIDA PIN B5
	GPIOB->CRL |= GPIO_CRL_MODE5_0;
	GPIOB->CRL &= ~GPIO_CRL_CNF5_0;
	GPIOB->CRL &= ~GPIO_CRL_CNF5_1;
	// PONGO COMO SALIDA PIN B6
	GPIOB->CRL |= GPIO_CRL_MODE6_0;
	GPIOB->CRL &= ~GPIO_CRL_CNF6_0;
	GPIOB->CRL &= ~GPIO_CRL_CNF6_1;
	// CONFIGURO PA10 COMO ENTRADA
	GPIOA->CRH &= ~GPIO_CRH_MODE10_0;
	GPIOA->CRH &= ~GPIO_CRH_MODE10_1;
	// PA10 COMO PULL UP
	GPIOA->CRH &= ~GPIO_CRH_CNF10_1;
	GPIOA->CRH |= GPIO_CRH_CNF10_0;
	// PA9 COMO SALIDA 50 MHz
	GPIOA->CRH |= GPIO_CRH_MODE9_0;
	GPIOA->CRH |= GPIO_CRH_MODE9_1;
	// PA9 COMO ALTNERATE FUNCTION PUSH PULL
	GPIOA->CRH &= ~GPIO_CRH_CNF9_0;
	GPIOA->CRH |= GPIO_CRH_CNF9_1;
	// CONFIGURO PINES 0,1,2,3,4 DEL PUERTO A COMO ENTRADA
	GPIOA->CRL &= ~GPIO_CRL_MODE0_0;
	GPIOA->CRL &= ~GPIO_CRL_MODE0_1;
	GPIOA->CRL |= GPIO_CRL_CNF0_1;
	GPIOA->CRL &= ~GPIO_CRL_CNF0_0;

	GPIOA->CRL &= ~GPIO_CRL_MODE1_0;
	GPIOA->CRL &= ~GPIO_CRL_MODE1_1;
	GPIOA->CRL |= GPIO_CRL_CNF1_1;
	GPIOA->CRL &= ~GPIO_CRL_CNF1_0;

	GPIOA->CRL &= ~GPIO_CRL_MODE2_0;
	GPIOA->CRL &= ~GPIO_CRL_MODE2_1;
	GPIOA->CRL |= GPIO_CRL_CNF2_1;
	GPIOA->CRL &= ~GPIO_CRL_CNF2_0;

	GPIOA->CRL &= ~GPIO_CRL_MODE3_0;
	GPIOA->CRL &= ~GPIO_CRL_MODE3_1;
	GPIOA->CRL |= GPIO_CRL_CNF3_1;
	GPIOA->CRL &= ~GPIO_CRL_CNF3_0;

	GPIOA->CRL &= ~GPIO_CRL_MODE4_0;
	GPIOA->CRL &= ~GPIO_CRL_MODE4_1;
	GPIOA->CRL |= GPIO_CRL_CNF4_1;
	GPIOA->CRL &= ~GPIO_CRL_CNF4_0;
	//APAGO LA SIRENA
    GPIO_SetBits(GPIOC, GPIO_Pin_13);

}

void configUsart(){
    USART_InitTypeDef usart1_init_struct;
    USART_Cmd(USART1, ENABLE);
    usart1_init_struct.USART_BaudRate = 9600;
    usart1_init_struct.USART_WordLength = USART_WordLength_8b;
    usart1_init_struct.USART_StopBits = USART_StopBits_1;
    usart1_init_struct.USART_Parity = USART_Parity_No;
    usart1_init_struct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    usart1_init_struct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART1, &usart1_init_struct);
    // HABILITAR USART PARA RECIBIR POR DMA
    USART1->CR3 |= USART_CR3_DMAR;
    NVIC_EnableIRQ(USART1_IRQn);
}

void configDMA(){
	//configuracion para RX
	//AGREGO DIRECCION DE DATOS DE UART1
	DMA1_Channel5 ->CPAR = (uint32_t)&(USART1->DR);
	//MAPEO AL BUFFER PARA RX
	DMA1_Channel5 ->CMAR = (uint32_t)&bufferRX[0];
	//TAMANO DE LA PALABRA QUE VOY A RECIBIR
	DMA1_Channel5 ->CNDTR= 6;        // EJEMPLO : (L1ON- L1OF - S1ON - CONF) -> MAS \r\n
	//HABILITO INTERRUPCIONES POR TRANSIMISON COMPLETA
	DMA1_Channel5 ->CCR |=DMA_CCR5_TCIE;
	//MEMORIA INCREMENTARL EN EL BUFFER AL RECIBIR
	DMA1_Channel5 ->CCR |=DMA_CCR5_MINC;
	//HABILITO CANAL 5
	DMA1_Channel5 ->CCR |=DMA_CCR5_EN;
	NVIC_EnableIRQ(DMA1_Channel5_IRQn);

	//configuracion para TX
	DMA1_Channel4 ->CPAR = (uint32_t)&(USART1->DR);
	DMA1_Channel4 ->CMAR = (uint32_t)&bufferTX[0];
	DMA1_Channel4 ->CCR |=DMA_CCR4_TCIE;
	DMA1_Channel4 ->CCR |=DMA_CCR4_MINC;
	DMA1_Channel4 ->CCR |=DMA_CCR4_DIR;
	USART1->CR3 |= USART_CR3_DMAT;
	NVIC_EnableIRQ(DMA1_Channel4_IRQn);
}

void configTimer(){

	//TIMER 2 USADO PARA TIEMPO PARA DESACTIVAR ALARMA AL SER DETECTADO

	//clock division
	TIM2->CR1 &= ~TIM_CR1_CKD_0;
	TIM2->CR1 &= ~TIM_CR1_CKD_1;
	//CUENTA HACIA ARRIBA
	TIM2->CR1 &= ~TIM_CR1_DIR;
	//preload enable
	TIM2->CR1 |= TIM_CR1_ARPE;
	//NO PARA AL DESBORDAR
	TIM2 ->CR1 &= ~TIM_CR1_OPM;
	TIM2->CR1 |= TIM_CR1_URS;
	//HABILITO INTERRUPCIONES DE ACTUALIZACION
	TIM2->DIER |= TIM_DIER_UIE;
	//CARGAMOS VALOR DE PRELOAD, CONTADOR Y PRESCALER
	TIM2->PSC = 10000;
	//HABILITAMOS INTERRUPCION NVIC
	NVIC_EnableIRQ(TIM2_IRQn);
	//TIMER PARA EN MODO DEBUG
	DBGMCU->CR |= DBGMCU_CR_DBG_TIM2_STOP;

	//TIMER 3 USADO PARA DELAY PARA ACTIVAR ALARMA
	//clock division
	TIM3->CR1 &= ~TIM_CR1_CKD_0;
	TIM3->CR1 &= ~TIM_CR1_CKD_1;
	//CUENTA HACIA ARRIBA
	TIM3->CR1 &= ~TIM_CR1_DIR;
	//preload enable
	TIM3->CR1 |= TIM_CR1_ARPE;
	// no PARA AL DESBORDAR
	TIM3 ->CR1 &= ~TIM_CR1_OPM;
	TIM3->CR1 |= TIM_CR1_URS;
	//HABILITO INTERRUPCIONES DE ACTUALIZACION
	TIM3->DIER |= TIM_DIER_UIE;
	//CARGAMOS VALOR DE PRELOAD, CONTADOR Y PRESCALER
	TIM3->PSC = 10000;
	TIM3->ARR =27024;
	TIM3->CNT =27024;
	//HABILITAMOS INTERRUPCION NVIC
	NVIC_EnableIRQ(TIM3_IRQn);
	//TIMER PARA EN MODO DEBUG
	DBGMCU->CR |= DBGMCU_CR_DBG_TIM3_STOP;


}

void configSysTick(){
	/*CARGO VALOR SYSTICK*/
	SysTick->LOAD = 4500000;
	/*HABILITO SYSTICK*/
	SysTick->CTRL |= SysTick_CTRL_ENABLE;
	/*selecciono source sysTick. Si lo pongo en 1, trabajo con el valor de AHB. En 0 divide AHB/8*/
	SysTick->CTRL |= SysTick_CTRL_CLKSOURCE;


}

void configPinesExternalInterrupt(){
	// CONFIGURACION PUERTO A0
	AFIO -> EXTICR[1] |= AFIO_EXTICR1_EXTI0_PA;
	EXTI->IMR |= EXTI_IMR_MR0;
	EXTI-> RTSR |= EXTI_RTSR_TR0;
	EXTI-> FTSR |= EXTI_FTSR_TR0;
	NVIC_EnableIRQ(EXTI0_IRQn);
	// CONFIGURACION PUERTO A1
	AFIO -> EXTICR[1] |= AFIO_EXTICR1_EXTI1_PA;
	EXTI->IMR |= EXTI_IMR_MR1;
	EXTI-> RTSR |= EXTI_RTSR_TR1;
	EXTI-> FTSR |= EXTI_FTSR_TR1;
	NVIC_EnableIRQ(EXTI1_IRQn);
	// CONFIGURACION PUERTO A2
	AFIO -> EXTICR[1] |= AFIO_EXTICR1_EXTI2_PA;
	EXTI->IMR |= EXTI_IMR_MR2;
	EXTI-> RTSR |= EXTI_RTSR_TR2;
	EXTI-> FTSR |= EXTI_FTSR_TR2;
	NVIC_EnableIRQ(EXTI2_IRQn);
	// CONFIGURACION PUERTO A3
	AFIO -> EXTICR[1] |= AFIO_EXTICR1_EXTI3_PA;
	EXTI->IMR |= EXTI_IMR_MR3;
	EXTI-> RTSR |= EXTI_RTSR_TR3;
	EXTI-> FTSR |= EXTI_FTSR_TR3;
	NVIC_EnableIRQ(EXTI3_IRQn);
	// CONFIGURACION PUERTO A4
	AFIO -> EXTICR[2] |= AFIO_EXTICR2_EXTI4_PA;
	EXTI->IMR |= EXTI_IMR_MR4;
	EXTI-> RTSR |= EXTI_RTSR_TR4;
	EXTI-> FTSR |= EXTI_FTSR_TR4;
	NVIC_EnableIRQ(EXTI4_IRQn);
}


void DMA1_Channel5_IRQHandler(void)
{
	char palabraRecibida[6]={'\0'};
	strcpy(palabraRecibida,bufferRX);

	if(flagConf==0){
		// NO ESTOY EN MODO CONFIGURACION
		if(strcmp(palabraRecibida,"conf\r\n")== 0){
			flagConf=1;
			strcpy(bufferTX, "conf");
			DMA_Cmd(DMA1_Channel4, DISABLE);
			DMA1_Channel4->CNDTR = 6;
			DMA_Cmd(DMA1_Channel4, ENABLE);
		}

		if(strcmp(palabraRecibida,"alac\r\n")== 0){
			TIM3->CR1 |= TIM_CR1_CEN;
			TIM3->EGR |= TIM_EGR_UG;
			strcpy(bufferTX, "acti\r\n");
			DMA_Cmd(DMA1_Channel4, DISABLE);
			DMA1_Channel4->CNDTR = 6;
			DMA_Cmd(DMA1_Channel4, ENABLE);

		}

		if(strcmp(palabraRecibida,"alde\r\n")== 0){
			flagAlarmaActivaMovimiento=0;
			flagAlarmaActivaMagnetico=0;
			//APAGO LA SIRENA
	        GPIO_SetBits(GPIOC, GPIO_Pin_13);
			//DESACTIVO TIMER PARA QUE NO SUENE LA ALARMA
			//COUNTER DISABLED
			TIM2->CR1 &= ~TIM_CR1_CEN;
			//REINICIO TIEMPO DE TIMER
			TIM2->CNT =tiempoTimer;
			strcpy(bufferTX, "alde\r\n");
			DMA_Cmd(DMA1_Channel4, DISABLE);
			DMA1_Channel4->CNDTR = 6;
			DMA_Cmd(DMA1_Channel4, ENABLE);
		}

		if(strcmp(palabraRecibida,"maac\r\n")== 0){
			flagAlarmaActivaMagnetico=1;
			strcpy(bufferTX, "maac\r\n");
			DMA_Cmd(DMA1_Channel4, DISABLE);
			DMA1_Channel4->CNDTR = 6;
			DMA_Cmd(DMA1_Channel4, ENABLE);
			/*strcpy(bufferTX, "alac\r\n");
			DMA_Cmd(DMA1_Channel4, DISABLE);
			DMA1_Channel4->CNDTR = 6;
			DMA_Cmd(DMA1_Channel4, ENABLE);*/
		}

		if(strcmp(palabraRecibida,"made\r\n")== 0){
			flagAlarmaActivaMagnetico=0;
			//APAGO LA SIRENA
	        GPIO_SetBits(GPIOC, GPIO_Pin_13);
			//DESACTIVO TIMER PARA QUE NO SUENE LA ALARMA
			//COUNTER DISABLED
			TIM2->CR1 &= ~TIM_CR1_CEN;
			//REINICIO TIEMPO DE TIMER
			TIM2->CNT =tiempoTimer;
			strcpy(bufferTX, "made\r\n");
			DMA_Cmd(DMA1_Channel4, DISABLE);
			DMA1_Channel4->CNDTR = 6;
			DMA_Cmd(DMA1_Channel4, ENABLE);
		}

		if(strcmp(palabraRecibida,"l1pr\r\n")== 0){
			GPIO_SetBits(GPIOB, GPIO_Pin_5);
			strcpy(bufferTX, "l1pr\r\n");
			DMA_Cmd(DMA1_Channel4, DISABLE);
			DMA1_Channel4->CNDTR = 6;
			DMA_Cmd(DMA1_Channel4, ENABLE);
		}

		if(strcmp(palabraRecibida,"l1ap\r\n")== 0){
			GPIO_ResetBits(GPIOB, GPIO_Pin_5);
			strcpy(bufferTX, "l1ap\r\n");
			DMA_Cmd(DMA1_Channel4, DISABLE);
			DMA1_Channel4->CNDTR = 6;
			DMA_Cmd(DMA1_Channel4, ENABLE);
		}

		if(strcmp(palabraRecibida,"l2pr\r\n")== 0){
			GPIO_SetBits(GPIOB, GPIO_Pin_6);
			strcpy(bufferTX, "l2pr\r\n");
			DMA_Cmd(DMA1_Channel4, DISABLE);
			DMA1_Channel4->CNDTR = 6;
			DMA_Cmd(DMA1_Channel4, ENABLE);
		}

		if(strcmp(palabraRecibida,"l2ap\r\n")== 0){
			GPIO_ResetBits(GPIOB, GPIO_Pin_6);
			strcpy(bufferTX, "l2ap\r\n");
			DMA_Cmd(DMA1_Channel4, DISABLE);
			DMA1_Channel4->CNDTR = 6;
			DMA_Cmd(DMA1_Channel4, ENABLE);
		}

		if(strcmp(palabraRecibida,"okok\r\n")== 0){
			/*strcpy(bufferTX, "oack\r\n");
			DMA_Cmd(DMA1_Channel4, DISABLE);
			DMA1_Channel4->CNDTR = 6;
			DMA_Cmd(DMA1_Channel4, ENABLE);*/
		}

		if(strcmp(palabraRecibida,"andr\r\n")== 0){
			strcpy(bufferTX, "lack\r\n");
			cambioLed();
			DMA_Cmd(DMA1_Channel4, DISABLE);
			DMA1_Channel4->CNDTR = 6;
			DMA_Cmd(DMA1_Channel4, ENABLE);
		}




	}else{
		//ESTOY EN MODO CONFIGURACION
		if(strcmp(palabraRecibida,"outc\r\n")== 0){
			flagConf=0;
			strcpy(bufferTX, "saliya");
			DMA_Cmd(DMA1_Channel4, DISABLE);
			DMA1_Channel4->CNDTR = 6;
			DMA_Cmd(DMA1_Channel4, ENABLE);
		}
		//hago otra cosa
		else{
			configuracionTiempoSonar(palabraRecibida);
		}
	}

	DMA1-> IFCR |= DMA_ISR_GIF5;
	DMA1_Channel5 ->CCR &=~DMA_CCR5_EN;
	DMA1_Channel5 ->CNDTR= 6;
	DMA1_Channel5 ->CCR |=DMA_CCR5_EN;
}

void DMA1_Channel4_IRQHandler(void)
{
	DMA_ClearITPendingBit(DMA1_IT_TC4);
	DMA_Cmd(DMA1_Channel4, DISABLE);
}

void TIM2_IRQHandler(void){

	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET){
		activarSirena();
		//BAJO BANDERA DE INTERRUPCION POR DESBORDAMIENTO
		TIM2->SR &= ~TIM_SR_UIF;
		//APAGO EL TIMER PARA QUE NO SIGA CONTANDO
		TIM2->CR1 &= ~TIM_CR1_CEN;
	}
}

void TIM3_IRQHandler(void){

	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET){
		flagAlarmaActivaMovimiento=1;
		flagAlarmaActivaMagnetico=1;
		strcpy(bufferTX, "alac\r\n");
		DMA_Cmd(DMA1_Channel4, DISABLE);
		DMA1_Channel4->CNDTR = 6;
		DMA_Cmd(DMA1_Channel4, ENABLE);
		//cambioLed();
		//BAJO BANDERA DE INTERRUPCION POR DESBORDAMIENTO
		TIM3->SR &= ~TIM_SR_UIF;
		//APAGO EL TIMER PARA QUE NO SIGA CONTANDO
		TIM3->CR1 &= ~TIM_CR1_CEN;
	}
}

void SysTick_Handler(){
	//cambioLed();
	activarSirena();

	/*deshabilito interrupcion por sysTick*/
	SysTick->CTRL &= ~SysTick_CTRL_TICKINT;

}



/**
 * interrupcion para sensor movimiento 1
 */
void EXTI0_IRQHandler(void){
	if(!flagConf){
		if(EXTI_GetFlagStatus(EXTI_Line0)!= RESET){

			if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)==1){
				strcpy(bufferTX, "s1ac\r\n");
				DMA_Cmd(DMA1_Channel4, DISABLE);
				DMA1_Channel4->CNDTR = 6;
				DMA_Cmd(DMA1_Channel4, ENABLE);
			}

			if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)==0){
				if(flagAlarmaActivaMovimiento==1){
					TIM2->CR1 |= TIM_CR1_CEN;
					TIM2->EGR |= TIM_EGR_UG;
					strcpy(bufferTX, "tdes\r\n");
					DMA_Cmd(DMA1_Channel4, DISABLE);
					DMA1_Channel4->CNDTR = 6;
					DMA_Cmd(DMA1_Channel4, ENABLE);

				}
				strcpy(bufferTX, "s1de\r\n");
				DMA_Cmd(DMA1_Channel4, DISABLE);
				DMA1_Channel4->CNDTR = 6;
				DMA_Cmd(DMA1_Channel4, ENABLE);
			}
			EXTI_ClearFlag(EXTI_Line0);
		}
	}

}
/**
 * interrupcion para sensor movimiento 2
 */
void EXTI1_IRQHandler(void){
	if(!flagConf){
		if(EXTI_GetFlagStatus(EXTI_Line1)!= RESET){
			if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1)==1){
				strcpy(bufferTX, "s2ac\r\n");
				DMA_Cmd(DMA1_Channel4, DISABLE);
				DMA1_Channel4->CNDTR = 6;
				DMA_Cmd(DMA1_Channel4, ENABLE);
			}

			if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1)==0){
				if(flagAlarmaActivaMovimiento==1){
					TIM2->CR1 |= TIM_CR1_CEN;
					TIM2->EGR |= TIM_EGR_UG;
					strcpy(bufferTX, "tdes\r\n");
					DMA_Cmd(DMA1_Channel4, DISABLE);
					DMA1_Channel4->CNDTR = 6;
					DMA_Cmd(DMA1_Channel4, ENABLE);
				}

				strcpy(bufferTX, "s2de\r\n");
				DMA_Cmd(DMA1_Channel4, DISABLE);
				DMA1_Channel4->CNDTR = 6;
				DMA_Cmd(DMA1_Channel4, ENABLE);
			}
			EXTI_ClearFlag(EXTI_Line1);
		}
	}

}

/**
 * interrupcion para sensor magnetico 1
 */
void EXTI2_IRQHandler(void){
	if(!flagConf){
		if(EXTI_GetFlagStatus(EXTI_Line2)!= RESET){
			if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_2)==1){
				strcpy(bufferTX, "m1de\r\n");
				DMA_Cmd(DMA1_Channel4, DISABLE);
				DMA1_Channel4->CNDTR = 6;
				DMA_Cmd(DMA1_Channel4, ENABLE);
			}

			if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_2)==0){
				if(flagAlarmaActivaMovimiento==1 && flagAlarmaActivaMagnetico==1){
					TIM2->CR1 |= TIM_CR1_CEN;
					TIM2->EGR |= TIM_EGR_UG;

				}

				strcpy(bufferTX, "m1ac\r\n");
				DMA_Cmd(DMA1_Channel4, DISABLE);
				DMA1_Channel4->CNDTR = 6;
				DMA_Cmd(DMA1_Channel4, ENABLE);

				if(flagAlarmaActivaMagnetico==1 && flagAlarmaActivaMovimiento==0){
					/*habilito interrupcion por sysTick*/
					SysTick->CTRL |= SysTick_CTRL_TICKINT;
				}


			}
			EXTI_ClearFlag(EXTI_Line2);
		}
	}


}

/**
 * interrupcion para sensor magnetico 2
 */
void EXTI3_IRQHandler(void){

	if(!flagConf){
		if(EXTI_GetFlagStatus(EXTI_Line3)!= RESET){
				if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_3)==1){
					strcpy(bufferTX, "m2de\r\n");
					DMA_Cmd(DMA1_Channel4, DISABLE);
					DMA1_Channel4->CNDTR = 6;
					DMA_Cmd(DMA1_Channel4, ENABLE);
				}

				if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_3)==0){
					strcpy(bufferTX, "m2ac\r\n");
					DMA_Cmd(DMA1_Channel4, DISABLE);
					DMA1_Channel4->CNDTR = 6;
					DMA_Cmd(DMA1_Channel4, ENABLE);

					if(flagAlarmaActivaMovimiento==1 && flagAlarmaActivaMagnetico==1){
						TIM2->CR1 |= TIM_CR1_CEN;
						TIM2->EGR |= TIM_EGR_UG;

					}

					if(flagAlarmaActivaMagnetico==1 && flagAlarmaActivaMovimiento==0){
						/*habilito interrupcion por sysTick*/
						SysTick->CTRL |= SysTick_CTRL_TICKINT;
					}

				}
				EXTI_ClearFlag(EXTI_Line3);
			}

	}

}

/**
 * interrupcion para sensor magnetico 3
 */
void EXTI4_IRQHandler(void){
	if(!flagConf){
		if(EXTI_GetFlagStatus(EXTI_Line4)!= RESET){
				if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)==1){
					strcpy(bufferTX, "m3de\r\n");
					DMA_Cmd(DMA1_Channel4, DISABLE);
					DMA1_Channel4->CNDTR = 6;
					DMA_Cmd(DMA1_Channel4, ENABLE);
				}

				if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)==0){
					if(flagAlarmaActivaMovimiento==1&& flagAlarmaActivaMagnetico==1){
						TIM2->CR1 |= TIM_CR1_CEN;
						TIM2->EGR |= TIM_EGR_UG;

					}

					strcpy(bufferTX, "m3ac\r\n");
					DMA_Cmd(DMA1_Channel4, DISABLE);
					DMA1_Channel4->CNDTR = 6;
					DMA_Cmd(DMA1_Channel4, ENABLE);

					for(int i=0;i<100000;i++){
									a++;
								}

					if(flagAlarmaActivaMagnetico==1 && flagAlarmaActivaMovimiento==0){
						/*habilito interrupcion por sysTick*/
						SysTick->CTRL |= SysTick_CTRL_TICKINT;
					}

				}
				EXTI_ClearFlag(EXTI_Line4);
			}
	}

}


void cambioLed(void) {
	uint8_t led_bit = GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_13);

	if(led_bit == (uint8_t)Bit_SET)
	    {
	        GPIO_ResetBits(GPIOC, GPIO_Pin_13);
	    }
	    else
	    {
	        GPIO_SetBits(GPIOC, GPIO_Pin_13);
	    }
}

void configuracionTiempoSonar(char *tiempoSetear){

	if(strcmp(tiempoSetear,"tim1\r\n")== 0){
		tiempoTimer=2703;
		TIM2->ARR =tiempoTimer;
		TIM2->CNT =tiempoTimer;
		strcpy(bufferTX, "3seg\r\n");
		DMA_Cmd(DMA1_Channel4, DISABLE);
		DMA1_Channel4->CNDTR = 6;
		DMA_Cmd(DMA1_Channel4, ENABLE);
	}

	if(strcmp(tiempoSetear,"tim2\r\n")== 0){
		tiempoTimer=4504;
		TIM2->ARR =tiempoTimer;
		TIM2->CNT =tiempoTimer;
		strcpy(bufferTX, "5seg\r\n");
		DMA_Cmd(DMA1_Channel4, DISABLE);
		DMA1_Channel4->CNDTR = 6;
		DMA_Cmd(DMA1_Channel4, ENABLE);
	}

	if(strcmp(tiempoSetear,"tim3\r\n")== 0){
		tiempoTimer=7207;
		TIM2->ARR =tiempoTimer;
		TIM2->CNT =tiempoTimer;
		strcpy(bufferTX, "8seg\r\n");
		DMA_Cmd(DMA1_Channel4, DISABLE);
		DMA1_Channel4->CNDTR = 6;
		DMA_Cmd(DMA1_Channel4, ENABLE);
	}

	if(strcmp(tiempoSetear,"tim4\r\n")== 0){
		tiempoTimer=9009;
		TIM2->ARR =tiempoTimer;
		TIM2->CNT =tiempoTimer;
		strcpy(bufferTX, "10se\r\n");
		DMA_Cmd(DMA1_Channel4, DISABLE);
		DMA1_Channel4->CNDTR = 6;
		DMA_Cmd(DMA1_Channel4, ENABLE);
	}

	if(strcmp(tiempoSetear,"tim5\r\n")== 0){
		tiempoTimer=13513;
		TIM2->ARR =tiempoTimer;
		TIM2->CNT =tiempoTimer;
		strcpy(bufferTX, "15se\r\n");
		DMA_Cmd(DMA1_Channel4, DISABLE);
		DMA1_Channel4->CNDTR = 6;
		DMA_Cmd(DMA1_Channel4, ENABLE);
	}
	/*tiempoTimer3=27024;
	TIM3->ARR =tiempoTimer3;
	TIM3->CNT =tiempoTimer3;*/

}

/**
 * recibe el puerto del cual se debe saber si cambio de estado o no, devolviendo 1 en caso afirmativo
 */
int stateChange(int puerto){
	switch (puerto){
		case 0:
			if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0) == a0){
				//a0= GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0);
				return 0;
			}else{
				//a0= GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0);
				return 1;
			}
			break;
		case 1:
			if(GPIO_ReadInputDataBit(GPIOA,1) == a1){
				return 1;
			}else return 0;
			break;
		case 2:
			if(GPIO_ReadInputDataBit(GPIOA,2) == a2){
				return 1;
			}else return 0;
			break;
		case 3:
			if(GPIO_ReadInputDataBit(GPIOA,3) == a3){
				return 1;
			}else return 0;
			break;
		case 4:
			if(GPIO_ReadInputDataBit(GPIOA,4) == a4){
				return 1;
			}else return 0;
			break;
	}
}

void activarSirena(){
	strcpy(bufferTX, "sire\r\n");
	DMA_Cmd(DMA1_Channel4, DISABLE);
	DMA1_Channel4->CNDTR = 6;
	DMA_Cmd(DMA1_Channel4, ENABLE);
	GPIO_ResetBits(GPIOC, GPIO_Pin_13);
}
