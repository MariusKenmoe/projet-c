#include "main.h"
#include "uc_configuration.h"
#define TRIGGER_PIN 9  // PA9
#define PRESCALER 4

// Buzzer = PA5


/*! 
 * Entry point of your source code
 */
// Mes variables marius
 uint8_t start_measurement = 1; // Flag pour démarrer une nouvelle mesure
 volatile uint32_t distance_cm =0;     // Distance calculée
 uint8_t echo_valid = 0;       // Indique qu'une distance valide est prête
 uint8_t sensor_error = 0;     // Erreur (défaillance ou hors portée)

 float temperature = 0.0;  // Stocke la température mesurée
 uint16_t adc_value = 0;   // Valeur brute de l'ADC
 uint8_t buzzer_enable = 0; // État du buzzer (1 = activé, 0 = désactivé)
 volatile uint16_t modulation_period=0;
 uint32_t distance_max = 300;
 uint32_t distance_min = 20;


// Les variables de grec
uint8_t Compteur=1; //Permet de connaitre l'état du rotary encoder
	uint8_t Echelle=0; //Communique au programme que le bouton poussoir du menu echelle a été enfoncé (via une interruption)
	uint8_t Mesure=0; //Communique au programme que le bouton poussoir du menu mesure a été enfoncé (via une interruption)
    uint8_t j=0; //indice utilisé dans la gestion de l'interruption pour le bouton mesures. Permet de figer l'affichage tant qu'une nouvelle mesure n'a pas ete faite
    uint8_t p=0; //indice utilisé dans la gestion de l'interruption pour le bouton echelle. Permet de figer l'affichage tant que l'échelle n'est pas changée
    uint8_t calibre=1; //Variable utilisée pour la sélection du calibre de mesure (3.3V ou 33V)
    uint8_t Clear_mesure=0; //indique si il faut clear l'affichage du lcd (lors du changement de menu par exemple)
    uint8_t Clear_Scale=0; //idem que pour Clear_mesure
    uint8_t	Scan_btn=0; //Utilisé pour compter le nombres de mesures de l'adc et déclancher une interruption quand le tableau ADC_Value est plein
    uint8_t  Modif_compteur = 0; //Indique que le rotary encoder à été tourné
    uint8_t test=0;//Permet de ne faire qu'une seule fois la routine d'initialisation du LCD
	uint32_t ADC_Value[60]; //tableau contenant les 60 dernières mesures faites par l'adc. Réinitialisé à la fréquence du TIM3
	uint32_t Indice_Value=0; //Indice utilisé pour suivre la position actuelle dans le tableau ADC_Value
	double ADC_Affichage[60];//tableau pour stocker les valeurs converties de l'adc
	double ADC_Min=4095; //Différence de potentiel minimale mesurée par l'adc, initialisé à 4095 (nombre décimal max sur 12 bits)
	double ADC_Max; //KDifférence de potentiel maximale mesurée par l'adc
	double dc_component; //variable reprenant la valeur de la composante dc du signal



	void TIM4_IRQHandler();

	int main()
	{


	    //! Configuration du microcontrolleur (GPIOs/Peripheriques/Horloge)
		ConfigureMicroController();
		EnableInterrupts();






	    while(1)
	    {



	    	if(test==0)
	    	{
				LCD_Configuration(0b00000011);
				for(int i = 0; i<1e4;i++);
				LCD_Configuration(0b00000011);
				for(int i = 0; i<1e4;i++);
				LCD_Configuration(0b00000011);
				for(int i = 0; i<1e4;i++);
				LCD_Configuration(0b00000010);

				LCD_Configuration(0b00000010);
				LCD_Configuration(0b00001000);

				LCD_Configuration(0b00000000);
				LCD_Configuration(0b00001000);

				LCD_Configuration(0b00000000);
				LCD_Configuration(0b00000001);

				LCD_Configuration(0b00000000);
				LCD_Configuration(0b00000111);
				for(unsigned i =0; i<1e5;i++);//delay
				LCD_Configuration(0b00000000);//Return Home
				LCD_Configuration(0b00000010);
				for(unsigned i =0; i<1e5;i++);//delay
				LCD_Configuration(0b00000000);//Display ON/OFF Control
				LCD_Configuration(0b00001100);

				LCD_Adress(8);
				for(unsigned i =0; i<1e1;i++);
				LCD_Adress(5);
				for(unsigned i =0; i<1e1;i++);
				RCC->AHB1ENR |= RCC_AHB2ENR_GPIOCEN; // Activation de l'horloge pour GPIOC
				// Configuration de PC13 en mode entrée (bouton utilisateur)
				GPIOC->MODER &= ~GPIO_MODER_MODE13_Msk; // Réinitialise le mode de PC13 (par défaut en entrée)

				//menu_demarrage();
				//fonction de sonar





				test++;
	    	}

	    	fonction_chargement();
	    	//ModulationPeriod(distance_cm);


	    	//temperature
	    	//ADC_IRQHandler();
	    						for(unsigned i =0; i<1e1;i++);
	    						LCD_Adress(8);
	    						for(unsigned i =0; i<1e1;i++);
	    						LCD_Adress(0);
	    						char str[20];
	    						sprintf(str, "%d", temperature);
	    						affichage_mot(str);


	    }
	}

	void fonction_chargement(){
		//croissance

		while(1) {

			 for (unsigned i = 0; i < 100000; i++);
			// Vérifie si PC13 est à l'état haut (non pressé, car le bouton tire vers 0 lorsqu'appuyé)
				 if(GPIOC->IDR & GPIO_IDR_ID13){}
				 else// PC13 à l'état bas
				        {
				            // Appel de la fonction lorsque le bouton n'est pas pressé (PC13 haut)
				            menu_demarrage();

				            // Délai pour éviter des appels multiples inutiles
				            for (unsigned i = 0; i < 1000; i++); // Attendre ~quelques ms
				        }




		calcul_dist();

							//for(unsigned i =0; i<1;i++);

							//LCD_Adress(8);
							//for(unsigned i =0; i<1e1;i++);
							//LCD_Adress(0);
							//TIM2->CR1 |= TIM_CR1_CEN_Msk;

							//char str[20];
							//uint32_t timer_value = TIM2->CNT;
							//sprintf(str, "%d", timer_value);
							//affichage_mot(str);
							//TIM2->CNT = 0;
		calcul_dist();

	}
		//PWMDutyCycle();
	}




// Code de gestion du signal du bip sonore
/*void PWMDutyCycle(uint16_t distance_cm) {
    uint16_t distance_min = 2;  // Distance minimale (cm)
    uint16_t distance_max = 300; // Distance maximale (cm)

    uint16_t dutyCycle = 0;  // on commence avec un rapport cyclique initiale nul, après on évoluera en fonction de la distance

    if (distance_cm <= distance_min) {
        dutyCycle = 100; // bip maximal pour le rapport cyclique à 100%
    } else if (distance_cm >= distance_max) {
        dutyCycle = 0;   // Pas de bip
    } else {
        // on calcul le rapport cyclique proportionnel
        dutyCycle = (distance_max - distance_cm) * 100 / (distance_max - distance_min); // formule de calcul du rapport cyclique en %
    }

    // On met à jour le rapport cyclique dans CCR1 (la largeur de l'impulsion)
    TIM2->CCR1 = (TIM2->ARR) * dutyCycle / 100;
}*/

	/*void TIM7_IRQHandler(void) {
	    if (TIM7->SR & TIM_SR_UIF) {
	        TIM7->SR &= ~TIM_SR_UIF;


	        if (distance_cm <= distance_min) {
	            buzzer_enable = 1; // Bip continu
	        } else if (distance_cm  >= distance_max ) {
	            buzzer_enable = 0; // Pas de bip
	        } else {
	            buzzer_enable = !buzzer_enable; // Modulation
	        }

	        if (buzzer_enable) {
	            GPIOA->MODER |= (2 << (5 * 2));  // on activer la sortie PWM sur PA5
	        } else {
	            GPIOA->MODER &= ~(3 << (5 * 2)); // on désactiver la sortie PWM (GPIO en entrée)
	        }
	    }
	}*/

	/*void ModulationPeriod(uint32_t distance) {
	    if (distance_cm <= distance_min) {
	        modulation_period = 0; // Bip continu
	    } else if (distance_cm >= distance_max) {
	    	TIM2->CR1 &=  ~TIM_CR1_CEN_Msk; // Pas de bip
	    } else {
	        // Modulation proportionnelle à la distance
	        modulation_period = 0 + ((distance_cm - distance_min) * 300) / (distance_max - distance_min);
	    }

	    // On mettre à jour TIM2->ARR
	    TIM7->ARR = modulation_period;
	}*/

	/*void ModulationPeriod(uint32_t distance) {

		                TIM7->PSC = 4000-1;
		      		            			 //TIM7->ARR = 45; //// 45 MIN MAX 200
		      		 TIM7->DIER |= TIM_DIER_UIE;
		      		            			 TIM7->CR1 |= TIM_CR1_CEN;
		        // Modulation proportionnelle à la distance
		        modulation_period = 0 + ((distance_cm - distance_min) * 300) / (distance_max - distance_min);

		    TIM7->ARR = modulation_period;
		}*/

	// programme d'interruption pour moduler le bip sonor JJJJJJJJJJJJJJJJJJJJJJJJJ
	/*void TIM7_IRQHandler() {
	    if (TIM7->SR & TIM_SR_UIF) {
	        TIM7->SR &= ~TIM_SR_UIF;

	        // Alterner l'état du buzzer
	        buzzer_enable = !buzzer_enable;
	        if (buzzer_enable) {
	           // GPIOA->MODER |= (2 << (5 * 2));  // on active la sortie PWM sur PA5
	        	 TIM2->CR1 |=  TIM_CR1_CEN_Msk;
	        } else {
	            // on désactive le TMR PWM
	        	 TIM2->CR1 &=  ~TIM_CR1_CEN_Msk;
	        }
	    }
	}*/

	// Configuration du timer qui s'occupe de la modulation du bip
	/*void ConfigureModulationTimer() {

      if(distance_cm < distance_max)
      {

		if (distance_cm <=distance_min) {
			 TIM7->ARR = 0;      // on active la sortie PWM sur PA8
			        } else if (distance_cm <=50) {
			        	 TIM7->ARR = 45; // on désactive la sortie PWM (GPIO en entrée)
			        }
	                   else if (distance_cm <=100){
	                	   TIM7->ARR = 90; // on désactive la sortie PWM (GPIO en entrée)
				          }

		                   else if (distance_cm <=150){
		                	   TIM7->ARR = 150; // on désactive la sortie PWM (GPIO en entrée)
					          }
		                   else if (distance_cm <=200){
		                   		    TIM7->ARR = 200; // on désactive la sortie PWM (GPIO en entrée)
		                   		 }
		                   else if (distance_cm <=250){
		                   		      TIM7->ARR = 250; // on désactive la sortie PWM (GPIO en entrée)
		                   					          }
		                   else{
		                	   TIM7->ARR = 300;

		                   }
      }
      else{
    	  TIM2->CR1 &=  ~TIM_CR1_CEN_Msk;

      }
	}*/

      // Configuration du timer qui s'occupe de la modulation du bip JJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJ
      	/*void ConfigureModulationTimer() {

      		TIM2->CCER |= TIM_CCER_CC1E;
      		TIM7->PSC = 4000-1;
      		            			 //TIM7->ARR = 45; //// 45 MIN MAX 200
      		 TIM7->DIER |= TIM_DIER_UIE;
    		 TIM7->CR1 |= TIM_CR1_CEN; //uint32_t local_distance = distance_cm ;



      		if (distance_cm <=distance_min) {
      			TIM2->CCER |= TIM_CCER_CC1E;
      			 TIM7->ARR = 0;      // on active la sortie PWM sur PA8
      			        } else if (distance_cm <distance_min*2) {
      			        	TIM2->CCER |= TIM_CCER_CC1E;
      			        	 TIM7->ARR = 45; // on désactive la sortie PWM (GPIO en entrée)
      			        }
      	                   else if (distance_cm <=distance_min*3){
      	                	 TIM2->CCER |= TIM_CCER_CC1E;
      	                	   TIM7->ARR = 90; // on désactive la sortie PWM (GPIO en entrée)
      				          }

      		                   else if (distance_cm <=distance_min*4){
      		                	 TIM2->CCER |= TIM_CCER_CC1E;
      		                	   TIM7->ARR = 150; // on désactive la sortie PWM (GPIO en entrée)
      		                   }
      		                 else if (distance_cm <=distance_min*6){
      		                      		                	 TIM2->CCER |= TIM_CCER_CC1E;
      		                      		                	   TIM7->ARR = 200; // on désactive la sortie PWM (GPIO en entrée)
      		                      		                   }

      		               else if (distance_cm <=distance_min*10){
      		                    		                      		                	 TIM2->CCER |= TIM_CCER_CC1E;
      		                    		                      		                	   TIM7->ARR = 250; // on désactive la sortie PWM (GPIO en entrée)
      		                    		                      		                   }
      		                   else{
      		                	 TIM2->CR1 &=  ~TIM_CR1_CEN_Msk;


      		                   }



            }*/


		    //TIM7->PSC = 4000-1;
		    //TIM7->ARR = 45; //// 45 MIN MAX 200
		    //TIM7->DIER |= TIM_DIER_UIE;
		   // TIM7->CR1 |= TIM_CR1_CEN;







// STM32 OUT
void SetGPIOAsOutput() {
    GPIOA->MODER &= ~(3 << (2 * TRIGGER_PIN));
    GPIOA->MODER |= (1 << (2 * TRIGGER_PIN)); // Mode sortie
}



// STM32 IN
void SetGPIOAsInput() {
    GPIOA->MODER &= ~(3 << (2 * TRIGGER_PIN)); // Mode entrée
}




// gestion de pulse
void GeneratePulse() {
    SetGPIOAsOutput();           // on configurer PA9 en sortie
    GPIOA->ODR |= (1 << TRIGGER_PIN); // on met PA9 à HIGH
    TIM4->CNT = 0;               // on réinitialise le compteur
    TIM4->CR1 |= TIM_CR1_CEN;    // on démarrer TIM4 pour 5 µs
    while (!(TIM4->SR & TIM_SR_UIF)); // Attendre débordement
    TIM4->SR &= ~TIM_SR_UIF;     // on effacer le drapeau
    GPIOA->ODR &= ~(1 << TRIGGER_PIN); // on met PA9 à LOW
    SetGPIOAsInput();            // Configurer PA9 en entrée
}




// Programme d'interruption sur la ligne 9 (gestion des FM et FD)
void EXTI9_5_IRQHandler() {
    if (EXTI->PR1 & (1 << TRIGGER_PIN)) {
        if (GPIOA->IDR & (1 << TRIGGER_PIN)) {
            // Front montant : on démarre le timer pour mesurer l’écho
            TIM3->CNT = 0;             // On réinitialiser le compteur
            TIM3->ARR = 18500;         // On configurer tIN_MAX
            TIM3->CR1 |= TIM_CR1_CEN;  // On démarrer TIM3
        } else {
            // Front descendant : on arrêter le timer et on calcule la durée
            TIM3->CR1 &= ~TIM_CR1_CEN;
            uint32_t echo_duration = TIM3->CNT;

            if (echo_duration >= 115 && echo_duration <= 20000) {
            	uint32_t distance_temp = (echo_duration * 343) / (2 * 10000); // Calcul distance
                distance_cm=distance_temp;

                echo_valid = 1;

                // Ensuite on démarre le timer pour la prochaine mesure
                TIM5->CNT = 0;
                TIM5->CR1 |= TIM_CR1_CEN;
            } else {
                sensor_error = 2; // Distance hors portée
            }
        }
        EXTI->PR1 |= (1 << TRIGGER_PIN); // On éfface le drapeau d’interruption
    }
}




//Programme d'interruption sur TIM3
void TIM3_IRQHandler() {
    if (TIM3->SR & TIM_SR_UIF) {
    	TIM3->CR1 &= ~TIM_CR1_CEN;  // On arrête de compter
        TIM3->SR &= ~TIM_SR_UIF;    // Effacer le drapeau
        start_measurement = 1;      // Si débordement avant réception du front montant
        if (TIM3->ARR == 750) {
            sensor_error = 1;       // Pas de front montant (capteur défaillant)
        } else if (TIM3->ARR == 18500) {
            sensor_error = 2;      // Pas de front descendant (distance max dépassée)
        }
    }
}


// Interruption sur TMR5 (delay avant la prochaine mesure)
void TIM5_IRQHandler() {
    if (TIM5->SR & TIM_SR_UIF) {
    	TIM5->CR1 &= ~TIM_CR1_CEN;	// on arrête de compter
        TIM5->SR &= ~TIM_SR_UIF;  // Effacer le drapeau
        start_measurement = 1;   // Déclencher une nouvelle mesure
    }
}

/*// Interruption sur TMR5 (delay avant la prochaine mesure)
void TIM5_IRQHandler() {
	TIM5->CR1 &= ~TIM_CR1_CEN;	// on arrête de compter
	TIM5->SR &= ~TIM_SR_UIF;    // On efface le drapeau
	GeneratePulse();            // Générer le pulse de 5 µs
    TIM3->CNT = 0;              // Réinitialiser pour tHOLDOFF
	 TIM3->ARR = 750;           // Configure tHOLDOFF (750 µs)
	TIM3->CR1 |= TIM_CR1_CEN;  // Démarrer le timer

}   */





// Calcul de la température
float CalculateTemperature(uint16_t adc_value) {
    float voltage = ((float)adc_value / 4095.0) * 3.3;  // On Convertir la valeur ADC en tension
    return (voltage - 0.5) / 0.02;                      // TMP37 : 500 mV à 25°C, 20 mV par °C
}

// Gestionnaire d'interruption pour TIM2
/*void TIM2_IRQHandler(void) {
    if (TIM2->SR & TIM_SR_UIF) {
        TIM2->SR &= ~TIM_SR_UIF;           // on Efface le drapeau
        ADC1->CR |= ADC_CR_ADSTART;      // on Lance une conversion ADC
    }
}*/

// Gestionnaire d'interruption pour ADC1
/*void ADC_IRQHandler(void) {
    if (ADC1->ISR & ADC_ISR_EOC) {           // Vérifier fin de conversion
        adc_value = ADC1->DR;              // Lire la valeur brute de l'ADC
        temperature = CalculateTemperature(adc_value); // Calculer la température
    }
}*/



// Les codes de grec !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

void LCD_Configuration(uint8_t mot)
{
	for (int i = 0; i<= 7; i++)
	{
		uint8_t bit = (mot >> i) & 0x01;

		if(i==0)//DB4 on PB5
		{
			if (bit == 1)
			{
				GPIOB->BSRR = GPIO_BSRR_BS5;
			}
			else
			{
				GPIOB->BSRR = GPIO_BSRR_BR5;
			}
		}
		else if(i==1)//DB5 on PB4
		{
			if (bit == 1)
			{
				GPIOB->BSRR = GPIO_BSRR_BS4;
			}
			else
			{
				GPIOB->BSRR = GPIO_BSRR_BR4;
			}
		}
		else if(i==2)//DB6 on PB10
		{
			if (bit == 1)
			{
				GPIOB->BSRR = GPIO_BSRR_BS10;
			}
			else
			{
				GPIOB->BSRR = GPIO_BSRR_BR10;
			}
		 }
		else if(i==3)//DB7 on PA8
		{
			if (bit == 1)
			{
				GPIOA->BSRR = GPIO_BSRR_BS8;
			}
			else
			{
				GPIOA->BSRR = GPIO_BSRR_BR8;
			}
		}
		else if(i==5)//RS on PA10
		{
			GPIOA->BSRR = GPIO_BSRR_BR10;//0 during the configuration
		}
	}

		//Creation of signal Enable
		GPIOB->BSRR = GPIO_BSRR_BS3;
		GPIOB->BSRR = GPIO_BSRR_BR3;
}

void LCD_Communication(uint8_t mot)
{
	for (int i = 0; i<= 7; i++)
		{
			uint8_t bit = (mot >> i) & 0x01;

			if(i==0)//DB4 on PB5
			{
				if (bit == 1)
				{
					GPIOB->BSRR = GPIO_BSRR_BS5;
				}
				else
				{
					GPIOB->BSRR = GPIO_BSRR_BR5;
				}
			}
			else if(i==1)//DB5 on PB4
			{
				if (bit == 1)
				{
					GPIOB->BSRR = GPIO_BSRR_BS4;
				}
				else
				{
					GPIOB->BSRR = GPIO_BSRR_BR4;
				}
			}
			else if(i==2)//DB6 PB10
			{
				if (bit == 1)
				{
					GPIOB->BSRR = GPIO_BSRR_BS10;
				}
				else
				{
					GPIOB->BSRR = GPIO_BSRR_BR10;
				}
			 }
			else if(i==3)//DB7 on PA8
			{
				if (bit == 1)
				{
					GPIOA->BSRR = GPIO_BSRR_BS8;
				}
				else
				{
					GPIOA->BSRR = GPIO_BSRR_BR8;
				}
			}
			else if(i==5)//RS on PA10
			{
				GPIOA->BSRR = GPIO_BSRR_BS10;//1 during the configuration
			}
		}

			//Creation of signal Enable
			GPIOB->BSRR = GPIO_BSRR_BS3;
			GPIOB->BSRR = GPIO_BSRR_BR3;
}

void LCD_Adress(uint8_t adress)
{
		for (int i = 0; i<= 7; i++)
		{
			uint8_t bit = (adress >> i) & 0x01;

			if(i==0)//DB4 on PB5
			{
				if (bit == 1)
				{
					GPIOB->BSRR = GPIO_BSRR_BS5;
				}
				else
				{
					GPIOB->BSRR = GPIO_BSRR_BR5;
				}
			}
			else if(i==1)//DB5 on PB4
			{
				if (bit == 1)
				{
					GPIOB->BSRR = GPIO_BSRR_BS4;
				}
				else
				{
					GPIOB->BSRR = GPIO_BSRR_BR4;
				}
			}
			else if(i==2)//DB6 PB10
			{
				if (bit == 1)
				{
					GPIOB->BSRR = GPIO_BSRR_BS10;
				}
				else
				{
					GPIOB->BSRR = GPIO_BSRR_BR10;
				}
			 }
			else if(i==3)//DB7 on PA8
			{
				if (bit == 1)
				{
					GPIOA->BSRR = GPIO_BSRR_BS8;
				}
				else
				{
					GPIOA->BSRR = GPIO_BSRR_BR8;
				}

			}
			else if(i==5)//RS on PA10
			{
				GPIOA->BSRR = GPIO_BSRR_BR10;//0 during Adressing
			}
		}
		//Creation of signal Enable
		GPIOB->BSRR = GPIO_BSRR_BS3;
		GPIOB->BSRR = GPIO_BSRR_BR3;
}

void affichage_mot(const char *mot ){

	    for (int i = 0; mot[i] != '\0'; i++) {  // Parcours jusqu'au caractère nul '\0'
	    	if (mot[i]=='a')
	    	{
	    		LCD_Communication(4);
	    		LCD_Communication(1);
	    	}
	    else if (mot[i]=='b'){
	    	LCD_Communication(4);
	        LCD_Communication(2);
	    }
	    else if (mot[i]=='c'){
	    	    	LCD_Communication(4);
	    	        LCD_Communication(3);
	    	    }
	    else if (mot[i]=='d'){
	    	    	LCD_Communication(4);
	    	        LCD_Communication(4);
	    	    }
	    else if (mot[i]=='e'){
	    	    	LCD_Communication(4);
	    	        LCD_Communication(5);
	    	    }
	    else if (mot[i]=='f'){
	    	    	LCD_Communication(4);
	    	        LCD_Communication(6);
	    	    }
	    else if (mot[i]=='g'){
	    	    	LCD_Communication(4);
	    	        LCD_Communication(7);
	    	    }
	    else if (mot[i]=='h'){
	    	    	    	LCD_Communication(4);
	    	    	        LCD_Communication(8);
	    	    	    }
	    else if (mot[i]=='i'){
	    	    	    	LCD_Communication(4);
	    	    	        LCD_Communication(9);
	    	    	    }
	    else if (mot[i]=='j'){
	    	    	    	LCD_Communication(4);
	    	    	        LCD_Communication(10);
	    	    	    }
	    else if (mot[i]=='k'){
	    	    	    	LCD_Communication(4);
	    	    	        LCD_Communication(11);
	    	    	    }
	    else if (mot[i]=='l'){
	    	    	    	LCD_Communication(4);
	    	    	        LCD_Communication(12);
	    	    	    }
	    else if (mot[i]=='m'){
	    	    	    	LCD_Communication(4);
	    	    	        LCD_Communication(13);
	    	    	    }
	    else if (mot[i]=='n'){
	    	    	    	LCD_Communication(4);
	    	    	        LCD_Communication(14);
	    	    	    }
	    else if (mot[i]=='o'){
	    	    	    	LCD_Communication(4);
	    	    	        LCD_Communication(15);
	    	    	    }
	    else if (mot[i]=='p'){
	    	    	    	LCD_Communication(5);
	    	    	        LCD_Communication(0);
	    	    	    }
	    else if (mot[i]=='q'){
	    	    	    	LCD_Communication(5);
	    	    	        LCD_Communication(1);
	    	    	    }
	    else if (mot[i]=='r'){
	    	    	    	LCD_Communication(5);
	    	    	        LCD_Communication(2);
	    	    	    }
	    else if (mot[i]=='s'){
	    	    	    	LCD_Communication(5);
	    	    	        LCD_Communication(3);
	    	    	    }
	    else if (mot[i]=='t'){
	    	    	    	LCD_Communication(5);
	    	    	        LCD_Communication(4);
	    	    	    }
	    else if (mot[i]=='u'){
	    	    	    	LCD_Communication(5);
	    	    	        LCD_Communication(5);
	    	    	    }
	    else if (mot[i]=='v'){
	    	    	    	LCD_Communication(5);
	    	    	        LCD_Communication(6);
	    	    	    }
	    else if (mot[i]=='w'){
	    	    	    	LCD_Communication(5);
	    	    	        LCD_Communication(7);
	    	    	    }
	    else if (mot[i]=='x'){
	    	    	    	LCD_Communication(5);
	    	    	        LCD_Communication(8);
	    	    	    }
	    else if (mot[i]=='y'){
	    	    	    	LCD_Communication(5);
	    	    	        LCD_Communication(9);
	    	    	    }
	    else if (mot[i]=='z'){
	    	    	    	LCD_Communication(5);
	    	    	        LCD_Communication(10);
	    	    	    }
	    else if (mot[i]=='0'){
	    	    	    	    	LCD_Communication(3);
	    	    	    	        LCD_Communication(0);
	    	    	    	    }
	    else if (mot[i]=='1'){
	    	    	    	    	    	LCD_Communication(3);
	    	    	    	    	        LCD_Communication(1);
	    	    	    	    	    }
	    else if (mot[i]=='2'){
	    	    	    	    	    	LCD_Communication(3);
	    	    	    	    	        LCD_Communication(2);
	    	    	    	    	    }
	    else if (mot[i]=='3'){
	    	    	    	    	    	LCD_Communication(3);
	    	    	    	    	        LCD_Communication(3);
	    	    	    	    	    }
	    else if (mot[i]=='4'){
	    	    	    	    	    	LCD_Communication(3);
	    	    	    	    	        LCD_Communication(4);
	    	    	    	    	    }
	    else if (mot[i]=='5'){
	    	    	    	    	    	LCD_Communication(3);
	    	    	    	    	        LCD_Communication(5);
	    	    	    	    	    }
	    else if (mot[i]=='6'){
	    	    	    	    	    	LCD_Communication(3);
	    	    	    	    	        LCD_Communication(6);
	    	    	    	    	    }
	    else if (mot[i]=='7'){
	    	    	    	    	    	LCD_Communication(3);
	    	    	    	    	        LCD_Communication(7);
	    	    	    	    	    }
	    else if (mot[i]=='8'){
	    	    	    	    	    	LCD_Communication(3);
	    	    	    	    	        LCD_Communication(8);
	    	    	    	    	    }
	    else if (mot[i]=='9'){
	    	    	    	    	    	    	LCD_Communication(3);
	    	    	    	    	    	        LCD_Communication(9);
	    	    	    	    	    	    }
	    else if (mot[i]=='0'){
	    	    	    	    	    	LCD_Communication(3);
	    	    	    	    	        LCD_Communication(0);
	    	    	    	    	    }
	    	    else if (mot[i]==1){
	    	    	    	    	    	    	LCD_Communication(3);
	    	    	    	    	    	        LCD_Communication(1);
	    	    	    	    	    	    }
	    	    else if (mot[i]==2){
	    	    	    	    	    	    	LCD_Communication(3);
	    	    	    	    	    	        LCD_Communication(2);
	    	    	    	    	    	    }
	    	    else if (mot[i]==3){
	    	    	    	    	    	    	LCD_Communication(3);
	    	    	    	    	    	        LCD_Communication(3);
	    	    	    	    	    	    }
	    	    else if (mot[i]==4){
	    	    	    	    	    	    	LCD_Communication(3);
	    	    	    	    	    	        LCD_Communication(4);
	    	    	    	    	    	    }
	    	    else if (mot[i]==5){
	    	    	    	    	    	    	LCD_Communication(3);
	    	    	    	    	    	        LCD_Communication(5);
	    	    	    	    	    	    }
	    	    else if (mot[i]==6){
	    	    	    	    	    	    	LCD_Communication(3);
	    	    	    	    	    	        LCD_Communication(6);
	    	    	    	    	    	    }
	    	    else if (mot[i]==7){
	    	    	    	    	    	    	LCD_Communication(3);
	    	    	    	    	    	        LCD_Communication(7);
	    	    	    	    	    	    }
	    	    else if (mot[i]==8){
	    	    	    	    	    	    	LCD_Communication(3);
	    	    	    	    	    	        LCD_Communication(8);
	    	    	    	    	    	    }
	    	    else if (mot[i]==9){
	    	    	    	    	    	    	    	LCD_Communication(3);
	    	    	    	    	    	    	        LCD_Communication(9);
	    	    	    	    	    	    	    }
	    else if (mot[i]==':'){
	    	    	    	    	    	    	LCD_Communication(3);
	    	    	    	    	    	        LCD_Communication(10);
	    	    	    	    	    	    }
	    else if (mot[i]==')'){
	    	    	    	    	    	    	    	LCD_Communication(2);
	    	    	    	    	    	    	        LCD_Communication(3);
	    	    	    	    	    	    	    }
	    else if (mot[i]=='$'){
	   	    	    	    	    	    	    	    	LCD_Communication(2);
	   	    	    	    	    	    	    	        LCD_Communication(0);
	   	    	    	    	    	    	    	    }


	    }}


void aff(const char* str){
	for(unsigned i =0; i<1;i++);
	LCD_Adress(8);
	for(unsigned i =0; i<1e1;i++);
	LCD_Adress(0);
	affichage_mot(str);
}





void calcul_dist(){

char vraiment_pulse[4] = "non";      // Variable pour vérifier si un pulse est détecté

// Fonction d'affichage (à adapter selon ton écran)
void affichage(uint32_t counter) {
    char str[20];
    sprintf(str, "%lu", counter);  // Convertit le compteur en chaîne de caractères
    aff(str);
    // Code spécifique pour afficher `str` sur ton écran (ex. LCD)
}
//ConfigureModulationTimer();
if (start_measurement) {


            start_measurement = 0;
            GeneratePulse();          // Générer le pulse de 5 µs
            TIM3->CNT = 0;            // Réinitialiser pour tHOLDOFF
            TIM3->ARR = 750;          // Configure tHOLDOFF (750 µs)
            TIM3->CR1 |= TIM_CR1_CEN; // Démarrer le timer
        }

        if (echo_valid) {
            echo_valid = 0;
            //ConfigureModulationTimer();

            for(unsigned i =0; i<1;i++);
                        LCD_Adress(8);
                        for(unsigned i =0; i<1e1;i++);
                        LCD_Adress(0);
                        affichage_mot("$$$$$$$$$$$$$$");
                        char str[20];

                        	sprintf(str, "%d", distance_cm);
            				aff(str);  // Affiche le nombre de µs du pulse détecté
            				for(unsigned i =0; i<1;i++);
            				LCD_Adress(8);
            				for(unsigned i =0; i<1e1;i++);
            				LCD_Adress(3);
            				affichage_mot("cm");
            				// on clear l'affichage de la mesure ////////////////////////////////////////////////////




            				//ModulationPeriod(distance_cm);
            				AdjustModulation();
            				maj_progression();
            				progression(distance_cm);


            				 //PWMDutyCycle(distance_cm);

                       }






            return;
}









void progression(int distance){		//affichage distance en petits carrés
	if (distance > 10) {
		for(unsigned i =0; i<1;i++);
		LCD_Adress(12);
		for(unsigned i =0; i<1e1;i++);
		LCD_Adress(0);
		affichage_mot(")");
	}
	else if (distance > 20){
		for(unsigned i =0; i<1;i++);
		LCD_Adress(12);
		for(unsigned i =0; i<1e1;i++);
		LCD_Adress(1);
	    affichage_mot(")");
	}
	else if (distance > 30){
			for(unsigned i =0; i<1;i++);
			LCD_Adress(12);
			for(unsigned i =0; i<1e1;i++);
			LCD_Adress(2);
		    affichage_mot(")");
		}
	else if (distance > 40){
				for(unsigned i =0; i<1;i++);
				LCD_Adress(12);
				for(unsigned i =0; i<1e1;i++);
				LCD_Adress(3);
			    affichage_mot(")");
			}
	else if (distance > 50){
				for(unsigned i =0; i<1;i++);
				LCD_Adress(12);
				for(unsigned i =0; i<1e1;i++);
				LCD_Adress(4);
			    affichage_mot(")");
			}
	else if (distance > 60){
				for(unsigned i =0; i<1;i++);
				LCD_Adress(12);
				for(unsigned i =0; i<1e1;i++);
				LCD_Adress(5);
			    affichage_mot(")");
			}
	else if (distance > 70){
				for(unsigned i =0; i<1;i++);
				LCD_Adress(12);
				for(unsigned i =0; i<1e1;i++);
				LCD_Adress(6);
			    affichage_mot(")");
			}
	else if (distance > 80){
				for(unsigned i =0; i<1;i++);
				LCD_Adress(12);
				for(unsigned i =0; i<1e1;i++);
				LCD_Adress(7);
			    affichage_mot(")");
			}
	else if (distance > 90){
				for(unsigned i =0; i<1;i++);
				LCD_Adress(12);
				for(unsigned i =0; i<1e1;i++);
				LCD_Adress(8);
			    affichage_mot(")");
			}
	else if (distance > 100){
				for(unsigned i =0; i<1;i++);
				LCD_Adress(12);
				for(unsigned i =0; i<1e1;i++);
				LCD_Adress(9);
			    affichage_mot(")");
			}
	else if (distance > 110){
				for(unsigned i =0; i<1;i++);
				LCD_Adress(12);
				for(unsigned i =0; i<1e1;i++);
				LCD_Adress(10);
			    affichage_mot(")");
			}
	else if (distance > 120){
					for(unsigned i =0; i<1;i++);
					LCD_Adress(12);
					for(unsigned i =0; i<1e1;i++);
					LCD_Adress(11);
				    affichage_mot(")");
				}
	else if (distance > 130){
					for(unsigned i =0; i<1;i++);
					LCD_Adress(12);
					for(unsigned i =0; i<1e1;i++);
					LCD_Adress(12);
				    affichage_mot(")");
				}
	else if (distance > 140){
					for(unsigned i =0; i<1;i++);
					LCD_Adress(12);
					for(unsigned i =0; i<1e1;i++);
					LCD_Adress(13);
				    affichage_mot(")");
				}
	else if (distance > 150){
					for(unsigned i =0; i<1;i++);
					LCD_Adress(12);
					for(unsigned i =0; i<1e1;i++);
					LCD_Adress(14);
				    affichage_mot(")");
				}
	else if (distance > 160){
					for(unsigned i =0; i<1;i++);
					LCD_Adress(12);
					for(unsigned i =0; i<1e1;i++);
					LCD_Adress(15);
				    affichage_mot(")");
				}
	else if (distance > 170){
						for(unsigned i =0; i<1;i++);
						LCD_Adress(13);
						for(unsigned i =0; i<1e1;i++);
						LCD_Adress(0);
					    affichage_mot(")");
					}
	else if (distance > 180){
							for(unsigned i =0; i<1;i++);
							LCD_Adress(13);
							for(unsigned i =0; i<1e1;i++);
							LCD_Adress(1);
						    affichage_mot(")");
						}
	else if (distance > 190){
							for(unsigned i =0; i<1;i++);
							LCD_Adress(13);
							for(unsigned i =0; i<1e1;i++);
							LCD_Adress(2);
						    affichage_mot(")");
						}
	                 else{                                            //(distance > 200)
							for(unsigned i =0; i<1;i++);
							LCD_Adress(13);
							for(unsigned i =0; i<1e1;i++);
							LCD_Adress(3);
						    affichage_mot(")");
						 }
	//ConfigureModulationTimer(distance);
}




void maj_progression(){
	for(unsigned place =0; place<16;place++){
									for(unsigned i =0; i<1;i++);
									LCD_Adress(12);
									for(unsigned i =0; i<1e1;i++);
									LCD_Adress(place);
								    affichage_mot("$");
	}
	for(unsigned place =0; place<4;place++){
										for(unsigned i =0; i<1;i++);
										LCD_Adress(13);
										for(unsigned i =0; i<1e1;i++);
										LCD_Adress(place);
									    affichage_mot("$");
		}

}

void menu_demarrage(){
	while(1){



	for(unsigned i =0; i<1e1;i++);
	LCD_Adress(12);
	for(unsigned i =0; i<1e1;i++);
	LCD_Adress(1);
	affichage_mot("lancer$le$compteur");

	for(unsigned i =0; i<1e1;i++);
	LCD_Adress(13);
	for(unsigned i =0; i<1e1;i++);
	LCD_Adress(5);
	affichage_mot("parametres");

}
}


// Gestion de TIM6 pour la période ON
void TIM6_DAC_IRQHandler() {
    if (TIM6->SR & TIM_SR_UIF) {
        TIM6->SR &= ~TIM_SR_UIF;
        TIM6->CR1 &= ~TIM_CR1_CEN_Msk;    //on Désactive TIM6

        TIM2->CR1 &= ~TIM_CR1_CEN_Msk;    // on Désactive le PWM (buzzer OFF)
        TIM7->CR1 |= TIM_CR1_CEN_Msk;     // on Active TIM7 pour la période OFF
    }
}

// Gestion de TIM7 pour la période OFF
void TIM7_IRQHandler() {
    if (TIM7->SR & TIM_SR_UIF) {
        TIM7->SR &= ~TIM_SR_UIF;
        TIM7->CR1 &= ~TIM_CR1_CEN_Msk;    // on Désactive TIM7

        TIM2->CR1 |= TIM_CR1_CEN_Msk;     // on Active le PWM (buzzer ON)
        TIM6->CR1 |= TIM_CR1_CEN_Msk;     // on Active TIM6 pour la période ON
    }
}

// Ajustement de la modulation en fonction de la distance
void AdjustModulation() {

	TIM7->PSC = 4000-1;
					    //TIM7->ARR = 100;
					    TIM7->DIER |= TIM_DIER_UIE;
					    //NVIC_EnableIRQ(TIM7_IRQn);
					    TIM7->CR1 |= TIM_CR1_CEN_Msk;

					    TIM6->PSC = 4000-1;
					        //TIM6->ARR = 100;  //Durée ON initiale en ms
					        TIM6->DIER |= TIM_DIER_UIE;
					        //NVIC_EnableIRQ(TIM6_DAC_IRQn);
					        TIM6->CR1 |= TIM_CR1_CEN_Msk;




    if (distance_cm <= distance_min) {
        TIM6->ARR = 0;
        TIM7->ARR = 0;
        //TIM6->CR1 |= TIM_CR1_CEN_Msk;
    } else if (distance_cm >= distance_max) {
        TIM6->ARR = 0;    // Pas de période ON
        TIM7->ARR = 0;    // Pas de période OFF
        TIM2->CR1 &= ~TIM_CR1_CEN_Msk; // on Désactive complètement le buzzer
    }


    else if (distance_cm <= 50 ) {
       TIM6->ARR = 20;
       TIM7->ARR = 20;
       TIM2->CR1 &= ~TIM_CR1_CEN_Msk;
   }

    else if (distance_cm <= 100 ) {
           TIM6->ARR = 35;
           TIM7->ARR = 75;
           TIM2->CR1 &= ~TIM_CR1_CEN_Msk;
       }

    else if (distance_cm <= 150 ) {
           TIM6->ARR = 50;
           TIM7->ARR = 100;
           TIM2->CR1 &= ~TIM_CR1_CEN_Msk;
       }
    else if (distance_cm <= 200 ) {
               TIM6->ARR = 80;
               TIM7->ARR = 160;
               TIM2->CR1 &= ~TIM_CR1_CEN_Msk;
           }


    else if (distance_cm <= 250 ) {
           TIM6->ARR = 120;
           TIM7->ARR = 240;
           TIM2->CR1 &= ~TIM_CR1_CEN_Msk;
       }
    else {
        //uint32_t modulation_period = 100 + ((distance_cm -distance_min ) * 900) / (distance_max - distance_min);
        //TIM6->ARR = modulation_period / 2; // Période ON
        //TIM7->ARR = modulation_period / 2; // Période OFF
        //TIM6->CR1 |= TIM_CR1_CEN_Msk; // Redémarrer la modulation
    	TIM2->CR1 &= ~TIM_CR1_CEN_Msk; // on Désactiver complètement le buzzer

    }
}
















