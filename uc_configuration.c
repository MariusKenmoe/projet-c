#include "main.h"
#include "uc_configuration.h"

#define TRIGGER_PIN 9  // PA9
#define PRESCALER 4
#define WAIT_TIME 500


void ConfigureMicroController() {
    // Default configuration (should not be modified) 
    SystemInit();
    EnableClockToPeripherals();

    // User configuration (stuff can be added or removed)
      ConfigureGPIO();
      SetupGPIOsPmw();
      InitializeUltrasonicSensor();
      InitializePulseTimer();
      InitializeEchoTimer();
      InitializeMeasurementTimer();
      //InitializePWM();
      InitializeTimer2();
      SetupGPIOs2();
      EnableInterrupts();
      //SetupGPIOsPmw2();
     // InitializePWM2();

       ConfigureGPIO_ADC();
       //ConfigureADC();
       //ConfigureTimer2();
       //EnableInterrupts2();
       //InitializeTimer66();
       //InitializeTimer77();

       //ConfigureModulationTimer();
}


void SystemInit(void)
{
  /* FPU settings: Give full access to FPU ------------------------------------*/
  SCB->CPACR |= (3UL << 20U)|(3UL << 22U);  /* set CP10 and CP11 Full Access */
}


/*!
 * Enable clock for the main used peripherals
 */
void EnableClockToPeripherals() {

	//! Enable DMA CLOCK
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMA2EN;

    //! Enable Clock for all GPIOs
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN
    			  | RCC_AHB2ENR_GPIODEN | RCC_AHB2ENR_GPIOEEN | RCC_AHB2ENR_GPIOFEN
				  | RCC_AHB2ENR_GPIOGEN | RCC_AHB2ENR_GPIOHEN;


    //! Enable ADC clock
    RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;

    //! Enable clock to TIM2, TIM3, TIM4, TIM5, TIM6, TIM6
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN | RCC_APB1ENR1_TIM3EN | RCC_APB1ENR1_TIM4EN
    		       | RCC_APB1ENR1_TIM5EN | RCC_APB1ENR1_TIM6EN | RCC_APB1ENR1_TIM7EN;

    //! Enable UART2, all I2C and DAC1 clock
    RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN | RCC_APB1ENR1_I2C1EN | RCC_APB1ENR1_I2C2EN
    		      | RCC_APB1ENR1_I2C3EN | RCC_APB1ENR1_DAC1EN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN|RCC_APB2ENR_TIM8EN;


    //! Select Sysclk for ADC
    RCC->CCIPR |= RCC_CCIPR_ADCSEL_0 | RCC_CCIPR_ADCSEL_1;

    //! Enable clock to SYSCFG
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
}



void ConfigureGPIO() {

    GPIOA->MODER &= ~(3 << (2 * TRIGGER_PIN)); // on réinitialiser
    GPIOA->MODER |= (1 << (2 * TRIGGER_PIN));  // Mode sortie par défaut
    GPIOA->ODR &= ~(1 << TRIGGER_PIN);         // Initialiser à LOW
}

void SetupGPIOsPmw() {

	// Configure le buzzer sur (PA5) en alternate function (PWM from TIM2 channel 1),  voir le tuto du cours pour comprendre la cofiguration
	// Reset PA5's mode of operation
	GPIOA->MODER &= ~GPIO_MODER_MODE5_Msk;
	// Set PA5 as alternate function
	GPIOA->MODER |= 0b10UL << GPIO_MODER_MODE5_Pos;
	//! Reset alternate function selection for PA5
	GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL5_Msk;
	//! Specify alternate function 1 for PA5
	GPIOA->AFR[0] |= (1UL << GPIO_AFRL_AFSEL5_Pos);
}
/*void SetupGPIOsPmw2() {

	// Configure le buzzer sur (PA7) en alternate function (PWM from TIM8 channel 1),  voir le tuto du cours pour comprendre la cofiguration
	// Reset PA7's mode of operation
	GPIOA->MODER &= ~GPIO_MODER_MODE7_Msk;
	// Set PA5 as alternate function
	GPIOA->MODER |= 0b10UL << GPIO_MODER_MODE7_Pos;
	//! Reset alternate function selection for PA7
	GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL7_Msk;
	//! Specify alternate function 3 for PA7
	GPIOA->AFR[0] |= (3UL << GPIO_AFRL_AFSEL7_Pos);// AF3 pour TIM8

	// Configuration PB0 en Alternate Function (AF) pour TIM8_CH1N
	GPIOB->MODER &= ~GPIO_MODER_MODE0_Msk;
		// Set PB0 as alternate function
	GPIOB->MODER |= 0b10UL << GPIO_MODER_MODE0_Pos;
		//! Reset alternate function selection for PB0
	GPIOB->AFR[0] &= ~GPIO_AFRL_AFSEL0_Msk;
		//! Specify alternate function 3 for PB0
	GPIOB->AFR[0] |= (3UL << GPIO_AFRL_AFSEL0_Pos); // AF3 pour TIM8


}*/




void SetupGPIOs2()
{

	//Configuration Bouton Echelle
		GPIOC->MODER &= ~GPIO_MODER_MODE0_Msk;//Set PC0 en Input ('00')
		GPIOC->PUPDR &= ~GPIO_PUPDR_PUPD0_Msk;
		GPIOC->PUPDR |= GPIO_PUPDR_PUPD0_1;//Resistance de pull down pour l'input ('10')
	//--------------------------------------------------

	//Configure Bouton Mesures
		GPIOC->MODER &= ~GPIO_MODER_MODE3_Msk;//Set PC3 en Input ('00')
		GPIOC->PUPDR &= ~GPIO_PUPDR_PUPD3_Msk;
		GPIOC->PUPDR |= GPIO_PUPDR_PUPD3_1;//Resistance de pull down pour l'input ('10')
	//--------------------------------------------------

	//Configure Rotary Encoder calibre de mesure
		//------Configure le signal A sur PC10
		GPIOC->MODER &= ~GPIO_MODER_MODE10_Msk;//Set PC10 en Input ('00')
		GPIOC->PUPDR &= ~GPIO_PUPDR_PUPD10_Msk;
		GPIOC->PUPDR |= GPIO_PUPDR_PUPD10_0;//Resistance de pull up pour l'input ('01')
		//------Configure le signal B sur PC12
		GPIOC->MODER &= ~GPIO_MODER_MODE12_Msk;//Set PC12 en Input ('00')
		GPIOC->PUPDR &= ~GPIO_PUPDR_PUPD12_Msk;
		GPIOC->PUPDR |= GPIO_PUPDR_PUPD12_0;//Resistance de pull up pour l'input ('01')
	//--------------------------------------------------

	//Configure PC10 en EXTI (interruption externe)
		SYSCFG->EXTICR[2] &= ~SYSCFG_EXTICR3_EXTI10_Msk;//Clear EXTI10
		SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI10_PC;//Set EXTI source sur PC10

		EXTI->FTSR1 |= EXTI_FTSR1_FT10;//Configure le front descendant en EXTI

		EXTI->IMR1 |= EXTI_IMR1_IM10;//Unmask EXTI10

		NVIC_EnableIRQ(EXTI15_10_IRQn);//Active l'interruption EXTI15_10 in NVIC (gestion des interruptions emboitées)
	//--------------------------------------------------


	//Configuration de l'écran LCD
	//--------------------------------------------------
		//Configuration de D2 (PA10) comme sortie
		GPIOA->MODER &= ~GPIO_MODER_MODE10_Msk;//RESET les bits dans MODE10
		GPIOA->MODER |= GPIO_MODER_MODE10_0;//SET le bit 20 sur 1 (output mode '01')
		//Configuration de D3 (PB3) comme sortie
		GPIOB->MODER &= ~GPIO_MODER_MODE3_Msk;//RESET les bits dans MODE3
		GPIOB->MODER |= GPIO_MODER_MODE3_0;//SET le bit 6 sur 1 (output mode '01')
		//Configuration de D4 (PB5) comme sortie
		GPIOB->MODER &= ~GPIO_MODER_MODE5_Msk;//RESET les bits dans MODE5
		GPIOB->MODER |= GPIO_MODER_MODE5_0;//SET le bit 10 sur 1 (output mode '01')
		//Configuration de D5 (PB4)comme sortie
		GPIOB->MODER &= ~GPIO_MODER_MODE4_Msk;//RESET les bits dans MODE4
		GPIOB->MODER |= GPIO_MODER_MODE4_0;//SET le bit 8 sur 1 (output mode '01')
		//Configuration de D6 (PB10) comme sortie
		GPIOB->MODER &= ~GPIO_MODER_MODE10_Msk;//RESET les bits dans MODE10
		GPIOB->MODER |= GPIO_MODER_MODE10_0;//SET le bit 20 sur 1 (output mode '01')
		//Configuration de D7 (PA8) comme sortie
		GPIOA->MODER &= ~GPIO_MODER_MODE8_Msk;//RESET les bits dans MODE5
		GPIOA->MODER |= GPIO_MODER_MODE8_0;//SET le bit 16 sur 1 (output mode '01')
	//--------------------------------------------------

	//Configuration de l'entrée analogique
		GPIOA->MODER |= GPIO_MODER_MODE0_Msk;//Selection du mode analogique ('11')
		GPIOA->ASCR  |= GPIO_ASCR_ASC0;//Connection entre l'adc et la pin d'entrée
	//--------------------------------------------------
}

// Configuration de EXTI pour PA9
void InitializeUltrasonicSensor() {

    SYSCFG->EXTICR[1] |= SYSCFG_EXTICR3_EXTI9_PA;  // Associer GPIOA à EXTI9 !!!!

    EXTI->IMR1 |= (1 << TRIGGER_PIN);  // On active interruption pour PA9
    EXTI->RTSR1 |= (1 << TRIGGER_PIN); // Front montant
    EXTI->FTSR1 |= (1 << TRIGGER_PIN); // Front descendant
    NVIC_EnableIRQ(EXTI9_5_IRQn);     // on active EXTI9 dans le NVIC
   // NVIC_EnableIRQ(EXTI9_5_IRQn);
}

// Initialisation du timer trigger pour la génération de pulse
 // avec nous aurons un signal d'horloge de 1 MHz, ce qui incrémente le compteur pour chaque microséconde
void InitializePulseTimer() {
// a changer pour prendre un autre timer moins important
    TIM4->PSC = PRESCALER - 1;          // 1 µs par incrémentation
    TIM4->ARR = 5;                      // Pulse de 5 µs au total
    TIM4->CR1 |= TIM_CR1_OPM;           // One Pulse Mode
}

// Initialisation du timer echo max
void InitializeEchoTimer() {

    TIM3->PSC = PRESCALER - 1;          // 1 µs
    TIM3->ARR = 18500;                  // tIN_MAX (18.5 ms)
    TIM3->DIER |= TIM_DIER_UIE;         // On active interruptions débordement

}

//Initialisation du timer delay avant la prochaine mesure
void InitializeMeasurementTimer() {

    TIM5->PSC = PRESCALER - 1;          //  1 µs
    TIM5->ARR = 200;                    // 200 µs
    TIM5->DIER |= TIM_DIER_UIE;         // On activer interruptions débordement
    TIM5->CR1 |= TIM_CR1_CEN;           // On démarrer TIM5

}

/*void InitializePWM() {
    // La fréquence de signal d'horloge est de 4 MHz
    TIM2->PSC = 4000-1; // 4 MHz/4000 = 1 kHz, donc à chaque milli seconde le compteur s'incrémentera
    TIM2->ARR = 1000;  // Période totale = 1000, donc le compteur se réinitialisera après 1000 * 1 ms
    TIM2->CCR1 = ;       // Rapport cyclique initial à 0% (pas de signal)

    // on configure PWM Mode 1 sur le canal 1
    TIM2->CCMR1 &= ~TIM_CCMR1_OC1M_Msk;
    TIM2->CCMR1 |= (0b0110UL << TIM_CCMR1_OC1M_Pos);
    TIM2->CCMR1 |= TIM_CCMR1_OC1PE;

    // On activer la sortie PWM sur le canal 1
    TIM2->CCER |= TIM_CCER_CC1E;

    // On activer le timer
    TIM2->CR1 |= TIM_CR1_CEN;
}*/

void InitializeTimer2() {
	//! Prescaler for a counting period of 1ms (internal clock: 4MHz => Prescaler : /4000)
	//! By default, AHB Prescaler and APBx Prescalers are configured to /1
	TIM2->PSC = 4-1;

	// Specify the ARR value to 1000, for the counter to reset after 1 second
	TIM2->ARR = WAIT_TIME * 2;
	// Specify the compare value of channel 1 to 500 (half of the period)
	TIM2->CCR1 =WAIT_TIME ;

	// Specify that TIM2 should function as a PWM generator
	TIM2->CCMR1 &= ~TIM_CCMR1_OC1M_Msk;
	TIM2->CCMR1 |= 0b0110UL << TIM_CCMR1_OC1M_Pos;

	TIM2->CCMR1 &= ~TIM_CCMR1_OC1PE_Msk;
	TIM2->CCMR1 |= TIM_CCMR1_OC1PE;

	// Enable Channel 1
	TIM2->CCER |= TIM_CCER_CC1E;

	// Enable counter
	TIM2->CR1 |= TIM_CR1_CEN_Msk;
}



/*void InitializePWM2() {

    TIM8->PSC = 4000 - 1;
    TIM8->ARR = 1000;         // La période totale : 1000 ms (1 seconde)
	 TIM8->CCR1 = 0;         // Rapport cyclique initial : 0%

	 // Configuration du mode PWM classique sur le canal 1
	 TIM8->CCMR1 &= ~TIM_CCMR1_OC1M_Msk;
	 TIM8->CCMR1 |= (0b0110UL << TIM_CCMR1_OC1M_Pos);
	 TIM8->CCMR1 |= TIM_CCMR1_OC1PE;

	  // Configuration des sorties classique et complémentaire
	  TIM8->CCER |= TIM_CCER_CC1E;  // Active la sortie normale (PWM classique)
	  TIM8->CCER |= TIM_CCER_CC1NE; // Active la sortie complémentaire (PWM inversé)

	    // Configuration de la polarité de la sortie normale et complémentaire
	  TIM8->CCER &= ~TIM_CCER_CC1P;  // Polarité normale : actif haut
	  TIM8->CCER |= TIM_CCER_CC1NP;  // Polarité complémentaire : actif bas (inverse)

	    //Configuration de la gestion automatique des états "break" et "dead time"
	  TIM8->BDTR |= TIM_BDTR_MOE;   // Active les sorties principales
	   TIM8->BDTR |= (100 << TIM_BDTR_DTG_Pos); // Dead-time (optionnel, ici 100 cycles)

	   // On active le timer 8
	  TIM8->CR1 |= TIM_CR1_CEN; // Active le timer 8
	}*/

void EnableInterrupts(){

	NVIC_EnableIRQ(TIM5_IRQn);
	NVIC_EnableIRQ(TIM3_IRQn);
	NVIC_EnableIRQ(EXTI9_5_IRQn);
	NVIC_EnableIRQ(TIM2_IRQn);
	NVIC_EnableIRQ(TIM4_IRQn);
	NVIC_EnableIRQ(TIM7_IRQn);
	//NVIC_EnableIRQ(TIM6_DAC_IRQn);
	NVIC_SetPriority(TIM6_DAC_IRQn, 1);
}



// Partie pour la configuration de la température
// Configuration GPIO : PA0 (entrée analogique pour TMP37)
void ConfigureGPIO_ADC(void) {

    GPIOA->MODER |= (3 << (0 * 2));        // PA0 en mode analogique
    GPIOA->PUPDR &= ~(3 << (0 * 2));       // Pas de pull-up, pull-down
}

// Configuration de l'ADC1
/*void ConfigureADC(void) {
    RCC->APB2ENR |= RCC_AHB2ENR_ADCEN;    // Activer l'horloge ADC1

    ADC1->CR = 0;                         // Réinitialiser CR1
    ADC1->CR |= ADC_IER_EOCIE;            // Activer interruption fin de conversion

    ADC1->CR = 0;                         // Réinitialiser CR2
    ADC1->CR |= ADC_CR_ADEN;             // Activer l'ADC

    ADC1->CFGR &= ~ADC_CFGR_RES;           // Réinitialiser la résolution
    ADC1->CFGR |= 0x0;                     // Résolution 12 bits (par défaut)

    ADC1->SMPR2 |= (3 << (0 * 3));         // Temps d'échantillonnage 56 cycles (canal 0)

    ADC1->SQR1 &= ~ADC_SQR1_L;             // Une seule conversion
    ADC1->SQR3 = 0;                        // Canal 0 (PA0)
}*/

// Configuration du Timer 2 pour déclencher la conversion
/*void ConfigureTimer2(void) {
    RCC->AHB1ENR |= RCC_APB1ENR1_TIM2EN;    // Activer l'horloge TIM2

    TIM2->PSC = 4000-1;                      // Prescaler : diviser l'horloge (4 MHz) par 4000 => 1 kHz (1 ms)
    TIM2->ARR = 1000;                      // Compteur : 1000 ms (1 Hz, conversion toutes les secondes)
    TIM2->DIER |= TIM_DIER_UIE;            // Activer interruption de mise à jour
    TIM2->CR1 |= TIM_CR1_CEN;              // Démarrer le timer
}*/


//Initialisation de TIM6 pour gérer la période ON
/*void InitializeTimer66() {
    TIM6->PSC = 4000-1;
    //TIM6->ARR = 100;  //Durée ON initiale en ms
    TIM6->DIER |= TIM_DIER_UIE;
    //NVIC_EnableIRQ(TIM6_DAC_IRQn);
    TIM6->CR1 &= ~TIM_CR1_CEN_Msk;
}

// Initialisation de TIM7 pour gérer la période OFF
void InitializeTimer77() {
    TIM7->PSC = 4000-1;
    //TIM7->ARR = 100;
    TIM7->DIER |= TIM_DIER_UIE;
    //NVIC_EnableIRQ(TIM7_IRQn);
    TIM7->CR1 &= ~TIM_CR1_CEN_Msk;        // on Désactive initialement
}*/

// Configuration des interruptions
void EnableInterrupts2(void) {
    NVIC_EnableIRQ(TIM2_IRQn);
    NVIC_EnableIRQ(TIM7_IRQn);
    //NVIC_EnableIRQ(TIM6_DAC_IRQn);
    // Activer l'interruption Timer 2
    /*NVIC_SetPriority(TIM2_IRQn, 1);        // Priorité pour Timer 2
    NVIC_EnableIRQ(ADC3_IRQn);              // Activer l'interruption ADC
    NVIC_SetPriority(ADC3_IRQn, 2); */        // Priorité pour l'ADC
}










