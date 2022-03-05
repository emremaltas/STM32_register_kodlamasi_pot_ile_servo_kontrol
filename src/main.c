/*
 * 	EMRE MALTAS 12.02.2022
 *  Register kodlamasi ile servo motor  kontrol.
 */

#include "stm32f4xx.h"
#include "stm32f4_discovery.h"

uint16_t adc_degeri=0;
uint32_t bekle=0;


void  SysTick1_Config()
{
	SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk ;
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
	SysTick->VAL = 0 ;
	SysTick->LOAD = 168000;

	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}

void CLOCK_Config()
{
	RCC->CR &= ~(1<<0);  									//dahili osilatör kapatıldı.
	while(!(RCC->CR & (1<<1)));								//aktifleşmesi beklendi.

	RCC->CR |= (1<<16);									//harici osilatör açıldı.
	while(!(RCC->CR & (1<<17)));								//aktifleşmesi beklendi.

	RCC->CR |= (1<<19);									//sistem güvenlik ayarı açıldı.

	RCC->PLLCFGR |= (1<<3);									//pllm:8
	RCC->PLLCFGR &= ~(1<<4);								//168 MHz ayarlaması için gerekli görüldü.
	RCC->PLLCFGR &= ~(1<<13);								//168 MHz ayarlaması için gerekli görüldü.
	RCC->PLLCFGR |= (336<<6); 								//plln:336
	RCC->PLLCFGR &=  ~(3<<16);								//pllp:2
	RCC->PLLCFGR |= (1<<22);								//pll kanağı:hse

	RCC->CR |= (1<<24);									//pll on.

	while(!(RCC->CR & (1<<25)));								//pll bekleniyor.

	RCC->CFGR |= (1<<1);									//sistem kaynağı seçiliyor.
	while(!(RCC->CFGR & (1<<3)));								//seçimin doğruluğu bekleniyor.
}


void GPIO_Config()
{
	RCC->AHB1ENR |= (1<<0);   								//a portunun clock hattı aktif edildi.
	//adc okuma icin;

	GPIOA->MODER |= 0X00000003;  								//a0 analog yapıldı.
	GPIOA->OSPEEDR |= 0X00000003; 								//yüksek hız yapıldı.
	GPIOA->PUPDR &= ~(1<<0) & ~(1<<1); 						        //no pull up-down.

	//pwm icin;

	GPIOA->MODER |= (1<<3); 								//pin(a1) alternatif fonk. ayarlandı.
	GPIOA->OSPEEDR |=(1<<2) | (1<<3); 							//yüksek hız yapıldı.
	GPIOA->PUPDR &= ~(1<<2) & ~(1<<3); 							//no pull up-down.
	GPIOA->AFR[0] |= (1<<5); 							        //af2 ayarlandı.

}

void SysTick_Handler()
{
	if(bekle>0)
		bekle--;
}
void delay_ms(uint32_t sure)									//ms türünden bekleme fonk. oluşturuldu.
{
	bekle=sure;
	while(bekle);
}
void ADC_Config()
{
	RCC->APB2ENR |= (1<<8);  								//adc1 clock hattı aftif edildi.

	ADC->CCR |= (1<<16); 									//adc clock hattı 4'e bölündü

	ADC1->CR1 &= ~(1<<24) & (1<<25); 							//adc 12 bitlik çözünürlük seçildi.

	ADC1->CR2 |= (1<<1); 									//sürekli cevrim modu secildi.
	ADC1->CR2 |= (1<<8); 									//dma secildi.
	ADC1->CR2 |= (1<<9); 									//dma sürekli istek secildi.
	ADC1->CR2 |= (1<<10); 									//eoc bayrağının kalkması istendi
	ADC1->CR2 &= ~(1<<11); 									//sağa dayalı yazım

	ADC1->SMPR2 |= 0X00000003; 								//56 cycle örneklem ayarlandı.
	ADC1->SQR1 &= ~(1<<20) & ~(1<<21) & ~(1<<22) & ~(1<<23);  		                //okunacak adc sayısı (1)
	ADC1->SQR3 &= ~(1<<0) & ~(1<<1) & ~(1<<2) & ~(1<<3) & ~(1<<4); 	                        //kanal 0 öncelik sırası 1 ayarlandı.

	ADC1->CR2 |= (1<<0);
}

void DMA_Config()
{
	RCC->AHB1ENR |= (1<<22); 								//dma2 clock hattı aktif edildi.
	while(DMA2_Stream0->CR & (1<<0)); 							//dma2 etkinleşmesi beklendi.

	DMA2_Stream0->CR &= ~(1<<6) & ~(1<<7); 						        //dma işlemi cevresel birimden hafızaya
	DMA2_Stream0->CR |= (1<<8); 								//sürekli dma aktarımı ayarlandı.
	DMA2_Stream0->CR &= ~(1<<9); 								//cevresel birimin adresinin sabit oldugunu belirttik.
	DMA2_Stream0->CR |= (1<<10); 								//degerin yazılacak yeri değişimi istendi
	DMA2_Stream0->CR |= (1<<11); 								//cevresel birimin 16 bit oldugu belirtildi
	DMA2_Stream0->CR |= (1<<13); 								//hafıza biriminde 16 bit yer ayırtıldı
	DMA2_Stream0->CR |= (1<<17) | (1<<17); 						        // yüksek öncelik verildi.
	DMA2_Stream0->CR &= ~(1<<25) & ~(1<<26) & ~(1<<27); 		                        //o. kanaldan dma olucağı belirtildi.

	DMA2_Stream0->NDTR = 1; 							        //1 tane dma aktarımı olucak
	DMA2_Stream0->PAR = (uint32_t) &ADC1->DR ; 					        //verinin okunacağı adres
	DMA2_Stream0->M0AR = (uint32_t) &adc_degeri; 				                //okunan değerin yazılacağı adres

	DMA2_Stream0->FCR |= (1<<0) | (1<<1); 						        //tam dolu oldugunda yazılım gerçekleşicek

	DMA2_Stream0->CR |= (1<<0); 								//dma2 başlatıldı.
}

uint32_t map(uint32_t adc_degeri,uint32_t okunacak_min,uint32_t okunacak_max,uint32_t cevrilecek_min,uint32_t cevrilecek_max)
{
	return cevrilecek_min + ((cevrilecek_max - cevrilecek_min) /  (okunacak_max / adc_degeri));
}
void TIMER5_Config()
{
	RCC->APB1ENR |= (1<<3); 					                      //timer'in clock hattı aktif edildi

	TIM5->CR1 &= ~(1<<4);   					        	      //yukarı yönlü sayma secildi
	TIM5->CR1 &= ~(1<<5) & ~(1<<6); 			                              // dir registerine göre işlem yapılcak
	TIM5->CR1 &= ~(1<<8) & ~(1<<9);	 						      //clock hattı bölünmedi
	TIM5->EGR |= (1<<0); 								      //her taşma sonrası sıfırlama olucak.
	TIM5->CCMR1 &= ~(1<<8) & ~(1<<9);   						      //kanal 2 çıkıs olarak seçildi
	TIM5->CCMR1 |= (1<<13) | (1<<14);   					              // pwm1 olarak mod seçildi
	TIM5->CCER |= (1<<4); 							  	      //kanal2 çıkışı aktif edildi.
	TIM5->ARR = 9999;     								      //10000'e kadar sayım
	TIM5->PSC = 167;	  							      //bölme oranı
	TIM5->CCR2 = 500;								      //pulse değeri
	TIM5->CR1 |= (1<<0); 								      //sayma başlatıldı.
}

int main(void)
{	SysTick1_Config();								    //ms ekleme ayarlamaları	
	CLOCK_Config(); 						    		    //saat ayarlamak için fonksiyon çağırıldı.
	GPIO_Config();									    //gpıo ayarları için fonksiyon çağırıldı.
	ADC_Config();									    //adc ayarları için fonksiyon çağırıldı.
	DMA_Config();									    //dma ayarları için fonksiyon çağırıldı.
	TIMER5_Config();								    //timer ayarları için fonksiyon çağırıldı.

	ADC1->CR2 |= (1<<30); 								    //adc çevrim işlmemi başlatıldı.

	while (1)
	{
		TIM5->CCR2 = map(adc_degeri,0,4095,500,3000);  				   //adc degeri 500-3000 aralıgına dönüştürülüp pulse degeri olarak atandı.
		delay_ms(1000);								   //servo dönüşü için süre verildi.
	}
}

void EVAL_AUDIO_TransferComplete_CallBack(uint32_t pBuffer, uint32_t Size){

	return;
}
uint16_t EVAL_AUDIO_GetSampleCallBack(void){
	return -1;
}
