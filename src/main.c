/*
 * 	EMRE MALTAS 12.02.2022
 *  Register kodlamasi ile servo motor  kontrol.
 */

#include "stm32f4xx.h"
#include "stm32f4_discovery.h"

uint16_t adc_degeri=0;
uint32_t bekle=0;


void CLOCK_Config()
{
	RCC->CR &= ~(1<<0);  										//dahili osilat�r kapat�ld�.
	while(!(RCC->CR & (1<<1)));									//aktifle�mesi beklendi.

	RCC->CR |= (1<<16);											//harici osilat�r a��ld�.
	while(!(RCC->CR & (1<<17)));								//aktifle�mesi beklendi.

	RCC->CR |= (1<<19);											//sistem g�venlik ayar� a��ld�.

	RCC->PLLCFGR |= (1<<3);										//pllm:8
	RCC->PLLCFGR &= ~(1<<4);									//168 Mhz ayarlamas� i�in gerekli g�r�ld�.
	RCC->PLLCFGR &= ~(1<<13);									//168 Mhz ayarlamas� i�in gerekli g�r�ld�.
	RCC->PLLCFGR |= (336<<6); 									//plln:336
	RCC->PLLCFGR &=  ~(3<<16);									//pllp:2
	RCC->PLLCFGR |= (1<<22);									//pll kana��:hse

	RCC->CR |= (1<<24);											//pll on.

	while(!(RCC->CR & (1<<25)));								//pll bekleniyor.

	RCC->CFGR |= (1<<1);										//sistem kayna�� se�iliyor.
	while(!(RCC->CFGR & (1<<3)));								//se�imin do�rulu�u bekleniyor.
}


void GPIO_Config()
{
	RCC->AHB1ENR |= (1<<0);   									//a portunun clock hatt� aktif edildi.
	//adc okuma icin;

	GPIOA->MODER |= 0X00000003;  								//a0 analog yap�ld�.
	GPIOA->OSPEEDR |= 0X00000003; 								//y�ksek h�z yap�ld�.
	GPIOA->PUPDR &= ~(1<<0) & ~(1<<1); 						   //no pull up-down.

	//pwm icin;

	GPIOA->MODER |= (1<<3); 								    //pin(a1) alternatif fonk. ayarland�.
	GPIOA->OSPEEDR |=(1<<2) | (1<<3); 							//y�ksek h�z yap�ld�.
	GPIOA->PUPDR &= ~(1<<2) & ~(1<<3); 							//no pull up-down.
	GPIOA->AFR[0] |= (1<<5); 									//af2 ayarland�.

	//bekleme fonk. i�in;

	SysTick_Config(SystemCoreClock/1000);

}

void SysTick_Handler()
{
	if(bekle>0)
		bekle--;
}
void delay_ms(uint32_t sure)										//ms t�r�nden bekleme fonk. olu�turuldu.
{
	bekle=sure;
	while(bekle);
}
void ADC_Config()
{
	RCC->APB2ENR |= (1<<8);  										//adc1 clock hatt� aftif edildi.

	ADC->CCR &= ~(1<<16) & (1<<17); 								//adc clock hatt� 4'e b�l�nd�

	ADC1->CR1 &= ~(1<<24) & (1<<25); 								//adc 12 bitlik ��z�n�rl�k se�ildi.

	ADC1->CR2 |= (1<<1); 											//s�rekli cevrim modu secildi.
	ADC1->CR2 |= (1<<8); 											//dma secildi.
	ADC1->CR2 |= (1<<9); 											//dma s�rekli istek secildi.
	ADC1->CR2 |= (1<<10); 											//eoc bayra��n�n kalkmas� istendi
	ADC1->CR2 &= ~(1<<11); 										   //sa�a dayal� yaz�m

	ADC1->SMPR2 |= 0X00000003; 										//56 cycle �rneklem ayarland�.
	ADC1->SQR1 &= ~(1<<20) & ~(1<<21) & ~(1<<22) & ~(1<<23);  		//okunacak adc say�s� (1)
	ADC1->SQR3 &= ~(1<<0) & ~(1<<1) & ~(1<<2) & ~(1<<3) & ~(1<<4); 	//kanal 0 �ncelik s�ras� 1 ayarland�.

	ADC1->CR2 |= (1<<0);
}

void DMA_Config()
{
	RCC->AHB1ENR |= (1<<22); 									//dma2 clock hatt� aktif edildi.
	while(DMA2_Stream0->CR & (1<<0)); 							//dma2 etkinle�mesi beklendi.

	DMA2_Stream0->CR &= ~(1<<6) & ~(1<<7); 						//dma i�lemi cevresel birimden haf�zaya
	DMA2_Stream0->CR |= (1<<8); 								//s�rekli dma aktar�m� ayarland�.
	DMA2_Stream0->CR &= ~(1<<9); 								//cevresel birimin adresinin sabit oldugunu belirttik.
	DMA2_Stream0->CR |= (1<<10); 								//degerin yaz�lacak yeri de�i�imi istendi
	DMA2_Stream0->CR |= (1<<11); 								//cevresel birimin 16 bit oldugu belirtildi
	DMA2_Stream0->CR |= (1<<13); 								//haf�za biriminde 16 bit yer ay�rt�ld�
	DMA2_Stream0->CR |= (1<<17) | (1<<17); 						// y�ksek �ncelik verildi.
	DMA2_Stream0->CR &= ~(1<<25) & ~(1<<26) & ~(1<<27); 		//o. kanaldan dma oluca�� belirtildi.

	DMA2_Stream0->NDTR = 1; 									//1 tane dma aktar�m� olucak
	DMA2_Stream0->PAR = (uint32_t) &ADC1->DR ; 					//verinin okunaca�� adres
	DMA2_Stream0->M0AR = (uint32_t) &adc_degeri; 				//okunan de�erin yaz�laca�� adres

	DMA2_Stream0->FCR |= (1<<0) | (1<<1); 						//tam dolu oldugunda yaz�l�m ger�ekle�icek

	DMA2_Stream0->CR |= (1<<0); 								//dma2 ba�lat�ld�.
}

uint32_t map(uint32_t adc_degeri,uint32_t okunacak_min,uint32_t okunacak_max,uint32_t cevrilecek_min,uint32_t cevrilecek_max)
{
	return cevrilecek_min + ((cevrilecek_max - cevrilecek_min) /  (okunacak_max / adc_degeri));
}

void TIMER5_Config()
{
	RCC->APB1ENR |= (1<<3); 					//timer'in clock hatt� aktif edildi

	TIM5->CR1 &= ~(1<<4);   					//yukar� y�nl� sayma secildi
	TIM5->CR1 &= ~(1<<5) & ~(1<<6); 			// dir registerine g�re i�lem yap�lcak
	TIM5->CR1 &= ~(1<<8) & ~(1<<9);	 			//clock hatt� b�l�nmedi
	TIM5->EGR |= (1<<0); 						//her ta�ma sonras� s�f�rlama olucak.
	TIM5->CCMR1 &= ~(1<<8) & ~(1<<9);   		//kanal 2 ��k�s olarak se�ildi
	TIM5->CCMR1 |= (1<<13) | (1<<14);   		// pwm1 olarak mod se�ildi
	TIM5->CCER |= (1<<4); 						//kanal2 ��k��� aktif edildi.
	TIM5->ARR = 9999;     						//10000'e kadar say�m
	TIM5->PSC = 167;	  						//b�lme oran�
	TIM5->CCR2 = 500;							//pulse de�eri
	TIM5->CR1 |= (1<<0); 						//sayma ba�lat�ld�.
}

int main(void)
{
	CLOCK_Config(); 						    //saat ayarlamak i�in fonksiyon �a��r�ld�.
	GPIO_Config();								//gp�o ayarlar� i�in fonksiyon �a��r�ld�.
	ADC_Config();								//adc ayarlar� i�in fonksiyon �a��r�ld�.
	DMA_Config();								//dma ayarlar� i�in fonksiyon �a��r�ld�.
	TIMER5_Config();							//timer ayarlar� i�in fonksiyon �a��r�ld�.

	ADC1->CR2 |= (1<<30); 						//adc �evrim i�lmemi ba�lat�ld�.

	while (1)
	{

		TIM5->CCR2 = map(adc_degeri,0,4095,500,3000);  //adc degeri 500-3000 aral�g�na d�n��t�r�l�p pulse degeri olarak atand�.
		delay_ms(1000);								   //servo d�n��� i�in s�re verildi.
	}

}

void EVAL_AUDIO_TransferComplete_CallBack(uint32_t pBuffer, uint32_t Size){

	return;
}

uint16_t EVAL_AUDIO_GetSampleCallBack(void){

	return -1;
}
