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
	RCC->CR &= ~(1<<0);  										//dahili osilatör kapatýldý.
	while(!(RCC->CR & (1<<1)));									//aktifleþmesi beklendi.

	RCC->CR |= (1<<16);											//harici osilatör açýldý.
	while(!(RCC->CR & (1<<17)));								//aktifleþmesi beklendi.

	RCC->CR |= (1<<19);											//sistem güvenlik ayarý açýldý.

	RCC->PLLCFGR |= (1<<3);										//pllm:8
	RCC->PLLCFGR &= ~(1<<4);									//168 Mhz ayarlamasý için gerekli görüldü.
	RCC->PLLCFGR &= ~(1<<13);									//168 Mhz ayarlamasý için gerekli görüldü.
	RCC->PLLCFGR |= (336<<6); 									//plln:336
	RCC->PLLCFGR &=  ~(3<<16);									//pllp:2
	RCC->PLLCFGR |= (1<<22);									//pll kanaðý:hse

	RCC->CR |= (1<<24);											//pll on.

	while(!(RCC->CR & (1<<25)));								//pll bekleniyor.

	RCC->CFGR |= (1<<1);										//sistem kaynaðý seçiliyor.
	while(!(RCC->CFGR & (1<<3)));								//seçimin doðruluðu bekleniyor.
}


void GPIO_Config()
{
	RCC->AHB1ENR |= (1<<0);   									//a portunun clock hattý aktif edildi.
	//adc okuma icin;

	GPIOA->MODER |= 0X00000003;  								//a0 analog yapýldý.
	GPIOA->OSPEEDR |= 0X00000003; 								//yüksek hýz yapýldý.
	GPIOA->PUPDR &= ~(1<<0) & ~(1<<1); 						   //no pull up-down.

	//pwm icin;

	GPIOA->MODER |= (1<<3); 								    //pin(a1) alternatif fonk. ayarlandý.
	GPIOA->OSPEEDR |=(1<<2) | (1<<3); 							//yüksek hýz yapýldý.
	GPIOA->PUPDR &= ~(1<<2) & ~(1<<3); 							//no pull up-down.
	GPIOA->AFR[0] |= (1<<5); 									//af2 ayarlandý.

	//bekleme fonk. için;

	SysTick_Config(SystemCoreClock/1000);

}

void SysTick_Handler()
{
	if(bekle>0)
		bekle--;
}
void delay_ms(uint32_t sure)										//ms türünden bekleme fonk. oluþturuldu.
{
	bekle=sure;
	while(bekle);
}
void ADC_Config()
{
	RCC->APB2ENR |= (1<<8);  										//adc1 clock hattý aftif edildi.

	ADC->CCR &= ~(1<<16) & (1<<17); 								//adc clock hattý 4'e bölündü

	ADC1->CR1 &= ~(1<<24) & (1<<25); 								//adc 12 bitlik çözünürlük seçildi.

	ADC1->CR2 |= (1<<1); 											//sürekli cevrim modu secildi.
	ADC1->CR2 |= (1<<8); 											//dma secildi.
	ADC1->CR2 |= (1<<9); 											//dma sürekli istek secildi.
	ADC1->CR2 |= (1<<10); 											//eoc bayraðýnýn kalkmasý istendi
	ADC1->CR2 &= ~(1<<11); 										   //saða dayalý yazým

	ADC1->SMPR2 |= 0X00000003; 										//56 cycle örneklem ayarlandý.
	ADC1->SQR1 &= ~(1<<20) & ~(1<<21) & ~(1<<22) & ~(1<<23);  		//okunacak adc sayýsý (1)
	ADC1->SQR3 &= ~(1<<0) & ~(1<<1) & ~(1<<2) & ~(1<<3) & ~(1<<4); 	//kanal 0 öncelik sýrasý 1 ayarlandý.

	ADC1->CR2 |= (1<<0);
}

void DMA_Config()
{
	RCC->AHB1ENR |= (1<<22); 									//dma2 clock hattý aktif edildi.
	while(DMA2_Stream0->CR & (1<<0)); 							//dma2 etkinleþmesi beklendi.

	DMA2_Stream0->CR &= ~(1<<6) & ~(1<<7); 						//dma iþlemi cevresel birimden hafýzaya
	DMA2_Stream0->CR |= (1<<8); 								//sürekli dma aktarýmý ayarlandý.
	DMA2_Stream0->CR &= ~(1<<9); 								//cevresel birimin adresinin sabit oldugunu belirttik.
	DMA2_Stream0->CR |= (1<<10); 								//degerin yazýlacak yeri deðiþimi istendi
	DMA2_Stream0->CR |= (1<<11); 								//cevresel birimin 16 bit oldugu belirtildi
	DMA2_Stream0->CR |= (1<<13); 								//hafýza biriminde 16 bit yer ayýrtýldý
	DMA2_Stream0->CR |= (1<<17) | (1<<17); 						// yüksek öncelik verildi.
	DMA2_Stream0->CR &= ~(1<<25) & ~(1<<26) & ~(1<<27); 		//o. kanaldan dma olucaðý belirtildi.

	DMA2_Stream0->NDTR = 1; 									//1 tane dma aktarýmý olucak
	DMA2_Stream0->PAR = (uint32_t) &ADC1->DR ; 					//verinin okunacaðý adres
	DMA2_Stream0->M0AR = (uint32_t) &adc_degeri; 				//okunan deðerin yazýlacaðý adres

	DMA2_Stream0->FCR |= (1<<0) | (1<<1); 						//tam dolu oldugunda yazýlým gerçekleþicek

	DMA2_Stream0->CR |= (1<<0); 								//dma2 baþlatýldý.
}

uint32_t map(uint32_t adc_degeri,uint32_t okunacak_min,uint32_t okunacak_max,uint32_t cevrilecek_min,uint32_t cevrilecek_max)
{
	return cevrilecek_min + ((cevrilecek_max - cevrilecek_min) /  (okunacak_max / adc_degeri));
}

void TIMER5_Config()
{
	RCC->APB1ENR |= (1<<3); 					//timer'in clock hattý aktif edildi

	TIM5->CR1 &= ~(1<<4);   					//yukarý yönlü sayma secildi
	TIM5->CR1 &= ~(1<<5) & ~(1<<6); 			// dir registerine göre iþlem yapýlcak
	TIM5->CR1 &= ~(1<<8) & ~(1<<9);	 			//clock hattý bölünmedi
	TIM5->EGR |= (1<<0); 						//her taþma sonrasý sýfýrlama olucak.
	TIM5->CCMR1 &= ~(1<<8) & ~(1<<9);   		//kanal 2 çýkýs olarak seçildi
	TIM5->CCMR1 |= (1<<13) | (1<<14);   		// pwm1 olarak mod seçildi
	TIM5->CCER |= (1<<4); 						//kanal2 çýkýþý aktif edildi.
	TIM5->ARR = 9999;     						//10000'e kadar sayým
	TIM5->PSC = 167;	  						//bölme oraný
	TIM5->CCR2 = 500;							//pulse deðeri
	TIM5->CR1 |= (1<<0); 						//sayma baþlatýldý.
}

int main(void)
{
	CLOCK_Config(); 						    //saat ayarlamak için fonksiyon çaðýrýldý.
	GPIO_Config();								//gpýo ayarlarý için fonksiyon çaðýrýldý.
	ADC_Config();								//adc ayarlarý için fonksiyon çaðýrýldý.
	DMA_Config();								//dma ayarlarý için fonksiyon çaðýrýldý.
	TIMER5_Config();							//timer ayarlarý için fonksiyon çaðýrýldý.

	ADC1->CR2 |= (1<<30); 						//adc çevrim iþlmemi baþlatýldý.

	while (1)
	{

		TIM5->CCR2 = map(adc_degeri,0,4095,500,3000);  //adc degeri 500-3000 aralýgýna dönüþtürülüp pulse degeri olarak atandý.
		delay_ms(1000);								   //servo dönüþü için süre verildi.
	}

}

void EVAL_AUDIO_TransferComplete_CallBack(uint32_t pBuffer, uint32_t Size){

	return;
}

uint16_t EVAL_AUDIO_GetSampleCallBack(void){

	return -1;
}
