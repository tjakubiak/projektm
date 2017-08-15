/*
 * GccApplication1.c
 *
 * Created: 2017-02-18 10:34:48
 * Author : SMT1
 */ 
#define F_CPU 8000000UL


#define DDR_WYSW1 DDRA
#define DDR_WYSW2 DDRC

#define MASTER_IN1 (1<<PA6)
#define PIN_MASTER_IN1 PINA
#define PORT_MASTER_IN1 PORTA
#define IF_MASTER_IN1_ON (PIN_MASTER_IN1 & MASTER_IN1)
#define IF_MASTER_IN1_OFF (!(PIN_MASTER_IN1 & MASTER_IN1))

#define MASTER_IN2 (1<<PC5)
#define PIN_MASTER_IN2 PINC
#define PORT_MASTER_IN2 PORTC
#define IF_MASTER_IN2_ON (PIN_MASTER_IN2 & MASTER_IN2)
#define IF_MASTER_IN2_OFF (!(PIN_MASTER_IN2 & MASTER_IN2))



#define DANE_WYSW1 (1<<PA7)
#define PORT_DANE_WYSW1 PORTA
#define ON_DANE_WYSW1 PORT_DANE_WYSW1 |= DANE_WYSW1
#define OFF_DANE_WYSW1 PORT_DANE_WYSW1 &=~ DANE_WYSW1

#define DANE_WYSW2 (1<<PC4)
#define PORT_DANE_WYSW2 PORTC
#define ON_DANE_WYSW2 PORT_DANE_WYSW2 |= DANE_WYSW2
#define OFF_DANE_WYSW2 PORT_DANE_WYSW2 &=~ DANE_WYSW2



#define SCLK_WYSW1 (1<<PA5)
#define PORT_SCLK_WYSW1 PORTA
#define ON_SCLK_WYSW1 PORT_SCLK_WYSW1 |= SCLK_WYSW1
#define OFF_SCLK_WYSW1 PORT_SCLK_WYSW1 &=~ SCLK_WYSW1

#define SCLK_WYSW2 (1<<PC6)
#define PORT_SCLK_WYSW2 PORTC
#define ON_SCLK_WYSW2 PORT_SCLK_WYSW2 |= SCLK_WYSW2
#define OFF_SCLK_WYSW2 PORT_SCLK_WYSW2 &=~ SCLK_WYSW2



#define MASTER_OUT1 (1<<PA4)
#define PORT_MASTER_OUT1 PORTA
#define ON_MASTER_OUT1 PORT_MASTER_OUT1 |= MASTER_OUT1
#define OFF_MASTER_OUT1 PORT_MASTER_OUT1 &=~ MASTER_OUT1

#define MASTER_OUT2 (1<<PC7)
#define PORT_MASTER_OUT2 PORTC
#define ON_MASTER_OUT2 PORT_MASTER_OUT2 |= MASTER_OUT2
#define OFF_MASTER_OUT2 PORT_MASTER_OUT2 &=~ MASTER_OUT2


#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/interrupt.h> 
#include <stdlib.h> 
#include <string.h> // correct header

#include "diskio.h"
#include "pff.h"


uint16_t licznikKoncaAnimacji=0;
volatile uint8_t licznikPlikWyswietlacz1=0;
volatile uint8_t licznikPlikWyswietlacz2=0;

volatile uint16_t odczytanyCzas=0;

volatile uint16_t odczytajCzas(char *odczytanyTekst){

	odczytanyCzas=0;
	uint8_t licznik=0;
	
	if(odczytanyTekst[0] <= '9' && odczytanyTekst[0] >= '0'){	//sprawdzam pierwszy znak
		while(licznik < 5){	//maksymalny czas bedzie skladal sie z 5 cyfr
				if(odczytanyTekst[licznik] <= '9' && odczytanyTekst[licznik] >= '0'){
					odczytanyCzas *= 10;				
					odczytanyCzas = odczytanyCzas + (odczytanyTekst[0] - '0');
				}else{			
					return odczytanyCzas;
				}
			licznik++;
		}		
	}else{		
		return 15; //jezeli pierwszy znak to nie cyfra
	}
}

void nadaj_bajt_do_wyswietlacza_1(char c)
{
	unsigned char i=0x80;
 	while (i) {
		if (c & i) {
			ON_DANE_WYSW1;
		}
		else {
			OFF_DANE_WYSW1;
		}

		OFF_SCLK_WYSW1;
		_delay_us(100);
		ON_SCLK_WYSW1;
		_delay_ms(1);;

		i >>= 1;
	}
 	OFF_DANE_WYSW1;
	
}
uint8_t pomocnicza=0;

//nadaje napis bez znakow specjalnych (enter, tab itp.)
uint8_t nadaj_tekst_do_wyswietlacza_1(char *napis)
{
	ON_SCLK_WYSW1;					//na wszelki wypadek podciagam do gory przed wejsciem do funkcji wysylania
	OFF_MASTER_OUT1;				//informujemy o danych do wyslania

	licznikKoncaAnimacji=0;
	
	while(IF_MASTER_IN1_OFF){		// czekamy na koniec animacji	

	if ((PINA & (1<<PA2)) != 0) {	// karta jest wyciagnieta
		return 1;
	}

									//Maksymalny czas najdluzszej animacji
	licznikKoncaAnimacji++;
		if(licznikKoncaAnimacji==0){
			return 1;
		}
		_delay_ms(1);
	}
	if ((PINA & (1<<PA2)) != 0) {	// karta jest wyciagnieta
		return 1;
	}
	
 	unsigned int k=0;		// licznik

	while (napis[k]) {
		if (32 <= napis[k] && napis[k] <= 127) {
			nadaj_bajt_do_wyswietlacza_1(napis[k]);	// wyslij pojedynczy znak
		}
		k++;
	}
 	nadaj_bajt_do_wyswietlacza_1(0);	// koniec napisu

 	_delay_ms(5);	// po chwili powinno byc zero na linii zwrotnej = potwierdzenie ok

	uint8_t ret = 0;
	if (PINA & (1<<PA6)) {
		ret = 1;
	}
	
	ON_MASTER_OUT1;
	return ret;

}

void nadaj_bajt_do_wyswietlacza_2(char c)
{
	unsigned char i=0x80;
	while (i) {
		if (c & i) {
			ON_DANE_WYSW2;
		}
		else {
			OFF_DANE_WYSW2;
		}

		OFF_SCLK_WYSW2;
		_delay_us(100);
		ON_SCLK_WYSW2;
		_delay_ms(1);;

		i >>= 1;
	}
	OFF_DANE_WYSW2;
}

// nadaje napis bez znakow specjalnych (enter, tab itp.)
// zwraca: 0 ok, 1 error
uint8_t nadaj_tekst_do_wyswietlacza_2(char *napis)
{		
	ON_SCLK_WYSW2;					//na wszelki wypadek podciagam do gory przed wejsciem do funkcji wysylania
	OFF_MASTER_OUT2;				//informujemy o danych do wyslania

	licznikKoncaAnimacji=0;
	
	while(IF_MASTER_IN2_OFF){		// czekamy na koniec animacji
	
		if ((PINA & (1<<PA2)) != 0) {	// karta jest wyciagnieta
			return 1;
		}	
									//Maksymalny czas najdluzszej animacji
		licznikKoncaAnimacji++;
		if(licznikKoncaAnimacji==0){
			return 1;
		}
		_delay_ms(1);	
			
	}
	
	unsigned int k=0;		// licznik

	while (napis[k]) {
		if (32 <= napis[k] && napis[k] <= 127) {
			nadaj_bajt_do_wyswietlacza_2(napis[k]);	// wyslij pojedynczy znak
		}
		k++;
	}
	nadaj_bajt_do_wyswietlacza_2(0);	// koniec napisu

	_delay_ms(5);	// po chwili powinno byc zero na linii zwrotnej = potwierdzenie ok

	uint8_t ret = 0;
// 	if (PINA & (1<<PA6)) {
// 		ret = 1;
// 	}
	
	ON_MASTER_OUT2;
	return ret;
}


char bufor_na_tekst[240];

char bufor_parametry[240];

BYTE res;
WORD s1;
FATFS fs;

//do pliku conf
BYTE res2;
WORD s2;
FATFS fs2;

//-----------------START TOMEK---------------------------------------
volatile uint16_t globalLicznik=0;
volatile uint8_t przepelnienieLicznika=0;
uint8_t err1=2, err2=2;

uint8_t errConf=2;	//b??dy zwi?zane z plikiem conf

uint8_t error=0;

uint8_t i;
uint8_t k;
uint8_t licznik_obecnosci_karty = 0;

char file_name_11[]="11.txt";
char file_name_12[]="12.txt";
char file_name_13[]="13.txt";
char file_name_14[]="14.txt";
char file_name_15[]="15.txt";
char file_name_16[]="16.txt";
char file_name_17[]="17.txt";

char* tablicaNazw1[]={file_name_11, file_name_12, file_name_13, file_name_14, file_name_15, file_name_16, file_name_17};

char file_name_21[]="21.txt";
char file_name_22[]="22.txt";
char file_name_23[]="23.txt";
char file_name_24[]="24.txt";
char file_name_25[]="25.txt";
char file_name_26[]="26.txt";
char file_name_27[]="27.txt";
char* tablicaNazw2[]={file_name_21, file_name_22, file_name_23, file_name_24, file_name_25, file_name_26, file_name_27};
	
char *ktoryPlik; 

char conf_file_name[]= "conf.txt";

void buforCzysc(void){
	for(uint8_t i=0; i<240; i++){
		bufor_na_tekst[i]=0x00;
	}
	
}

void timer16bitInitialize(void){  //tryb CTC, 1024 prescaler, ustawiony na 1 sekunde

	TCCR1B |= (1<<WGM12) | (1<<CS12) | (1<<CS10);
	TIMSK |= (1<<OCIE1A);
	OCR1A = 7812;
}

uint8_t wyslijDoWyswietlacza(char * nazwaPliku, uint8_t ktoryWyswietlacz){	
 		if ((PINA & (1<<PA2)) != 0) {	// karta jest wlozona
		 return 0;
		 }

		if ((PINA & (1<<PA2)) != 0){
			err1=2;
			err2=2;
		}
		
		if(ktoryWyswietlacz == 1){
			PORTC |= (1<<PC2); 	// led zielony 1 on
		}else{
			PORTC |= (1<<PC3); 	// led zielony 2 on
		}
			
			//inicjalizacja karty SD
			i = 255;	//timeout
			while (i-- && (res = disk_initialize()));

			
			if (res == FR_OK) {

				// pierwszy plik dla pierwszego wyswietlacza
				res = pf_mount(&fs);

				if (res == FR_OK) {

					res = pf_open(nazwaPliku);	//	otwiera konretny plik

					if (res == FR_OK) {
						
						buforCzysc();						
						res = pf_read(bufor_na_tekst, 240, &s1);	// czyta z pliku

						if (res == FR_OK) {	// jesli operacja sie powiodla wyslij zawartosc pliku
							bufor_na_tekst[s1]=0;
							
							if(ktoryWyswietlacz == 1){
								if(IF_MASTER_IN1_OFF){
										err1 = nadaj_tekst_do_wyswietlacza_1(bufor_na_tekst);
								}else{
										err1 =1;
								}
							}else{
								if(IF_MASTER_IN2_OFF){
									err2 = nadaj_tekst_do_wyswietlacza_2(bufor_na_tekst);
									}else{
									err2 =1;
								}
							}
						}
						else {
							//nadajstring("read error");
							if(ktoryWyswietlacz == 1)	err1 = 5;
							else						err2 = 5;

						}
					}
					else {
						//nadajstring("open file error");
							if(ktoryWyswietlacz == 1)	err1 = 4;
							else						err2 = 4;
							return 1;
					}
				}
				else {
					//nadajstring("mount error");
							if(ktoryWyswietlacz == 1)	err1 = 3;
							else						err2 = 3;
				}
				_delay_ms(100);
				pf_mount(NULL);

				// drugi plik dla drugiego wyswietlacza
			}
	
			PORTC &= (~(1<<PC1)); 	// led czerwony off
			PORTC &= (~(1<<PC2)); 	// led zielony 1 off
			PORTC &= (~(1<<PC3)); 	// led zielony 2 off
			_delay_ms(100);		

		if(ktoryWyswietlacz == 1){
				// wyswietla stan odczytu 1.txt: (zielona_1 + czerwona * err1)
				PORTC |= (1<<PC2); 	// led zielony 1 on
				_delay_ms(300);
				for (i=0; i<err1; i++) {	// ktory error? pierwszego wyswietlacza
					PORTC |= (1<<PC1); 	// led czerwony on
					_delay_ms(100);
					PORTC &= (~(1<<PC1)); 	// led czerwony off
					_delay_ms(200);
				}
				PORTC &= (~(1<<PC2)); 	// led zielony 1 off
		_delay_ms(100);			
				
		}else{
				// wyswietla stan odczytu 2.txt: (zielona_2 + czerwona * err2)
				PORTC |= (1<<PC3); 	// led zielony 2 on
				_delay_ms(300);
				for (i=0; i<err2; i++) {	// ktory error? drugiego wyswietlacza
					PORTC |= (1<<PC1); 	// led czerwony on
					_delay_ms(100);
					PORTC &= (~(1<<PC1)); 	// led czerwony off
					_delay_ms(200);
				}
				PORTC &= (~(1<<PC3)); 	// led zielony 2 off
				_delay_ms(100);						
		}
		
		return 0;
}
//-----------------END TOMEK---------------------------------------


void resetujLicznik(void);


int main(void)
{
	//inicjalizacja SPI
	#define SCK PB7
	#define MOSI PB5
	#define MISO PB6
	#define CS PB4

	DDRB |=(1<<CS)|(1<<MOSI)|(1<<SCK);	// 1 out 0 in
	PORTB |=(1<<CS);
	PORTB |=(1<<MISO); //podciagamy linie miso do vcc
	SPCR |=(1<<SPE)|(1<<MSTR);
	// port od LED jako wyjscie
	DDRB |=(1<<PB1); 

	DDRC |= (1<<PC1) | (1<<PC2) | (1<<PC3); 	// ledy

	DDRA |= (1<<PA5) | (1<<PA7); 	// OUT1	do wyswietlaczy
	DDRC |= (1<<PC4) | (1<<PC6); 	// OUT2	do wyswietlaczy

	DDRA &= (~(1<<PA6)); 	// IN1	z wyswietlaczy
	DDRC &= (~(1<<PC5)); 	// IN2	z wyswietlaczy

	PORTA &= ~((1<<PA5) | (1<<PA7)); 	// OUT1
	PORTC &= ~((1<<PC4) | (1<<PC6)); 	// OUT2

	PORTA |= (1<<PA6); 	// IN1 pull-up
	PORTC |= (1<<PC5); 	// IN2 pull-up

	DDRA &= (~(1<<PA2)); 	// IN1	wykrywanie obecnosci karty SD
	PORTA |= (1<<PA2); 	// IN1 pull-up
	
	//inicjalizacja UARTA
	DDRD |=(1<<PD1);
	_delay_ms(100);
	_delay_ms(100);	

//	nadaj_bajt_do_wyswietlacza_1(1); 
	
	
	//TJ SPI
	//WYSW1
	DDR_WYSW1 &=~ MASTER_IN1;
	PORT_MASTER_IN1 |= MASTER_IN1;


	DDR_WYSW1 |= SCLK_WYSW1;
	ON_SCLK_WYSW1;

	DDR_WYSW1 |= DANE_WYSW1;

	DDR_WYSW1 |= MASTER_OUT1;

	ON_MASTER_OUT1;

	//WYSW2
	DDR_WYSW2  &=~ MASTER_IN2 ;
	PORT_MASTER_IN2 |= MASTER_IN2;


	DDR_WYSW2 |= SCLK_WYSW2;
	ON_SCLK_WYSW2;

	DDR_WYSW2 |= DANE_WYSW2;

	DDR_WYSW2 |= MASTER_OUT2;

	ON_MASTER_OUT2;

	timer16bitInitialize();

	uint8_t licznik_obecnosci_karty = 0;	
	uint8_t licznikBrakuKarty=0;
	odczytanyCzas=10;
	
	//END TOMEK
	sei();	
	while(1) {	
		_delay_ms(100);
		if ((PINA & (1<<PA2)) == 0) {	// karta jest wlozona
			if (licznik_obecnosci_karty < 10) {
				licznik_obecnosci_karty ++;
			}

		}
		else {							// nie ma karty
			licznik_obecnosci_karty = 0;
			PORTC |= (1<<PC1); 	// switch LED CZERWONY		
			PORTC &= (~(1<<PC2)); 	// led zielony 1 off
			PORTC &= (~(1<<PC3)); 	// led zielony 2 off			
			cli();			
		}

		if (licznik_obecnosci_karty == 3) {	// jesli jest od paru sekund
	
			
			PORTC &=~ (1<<PC1);
			resetujLicznik();
			//od razu po wlozeniu karty wysylamy pliki 11.txt i 21.txt;
			
			wyslijDoWyswietlacza(tablicaNazw1[0],1);
			wyslijDoWyswietlacza(tablicaNazw2[0],2);			
			licznikPlikWyswietlacz1=1;
			licznikPlikWyswietlacz2=1;			
			
			//inicjalizacja karty SD
			k = 255;	//timeout
			while (k-- && (res2 = disk_initialize()));

			
			if (res2 == FR_OK) {

				// pierwszy plik dla pierwszego wyswietlacza
				res2 = pf_mount(&fs2);

				if (res2 == FR_OK) {

					res2 = pf_open(conf_file_name);	//	otwiera konretny plik

					if (res2 == FR_OK) {
						res2 = pf_read(bufor_parametry, 240, &s2);	// czyta z pliku

						if (res2 == FR_OK) {	// jesli operacja sie powiodla wyslij zawartosc pliku
							bufor_parametry[s2]=0;
							odczytanyCzas = odczytajCzas(bufor_parametry);						
						
//							err1 = nadaj_tekst_do_wyswietlacza_1(bufor_na_tekst);
						}
						else {
							//nadajstring("read error");
							errConf = 5;
						}
					}
					else {
						//nadajstring("open file error");
						errConf = 4;
					}
				}
				else {
					//nadajstring("mount error");
					errConf = 3;
				}
				_delay_ms(100);
				pf_mount(NULL);

			}
			else {
				//nadajstring("disk init error");
// 				err1 = 2;
// 				err2 = 2;
			}
			_delay_ms(100);
			pf_mount(NULL);
			
			sei();
			
		}


	}
}


void resetujLicznik(void){
	globalLicznik=0;	//inicjalizujemy poczkatkowymi wartosciami
	licznikPlikWyswietlacz1=0;
	licznikPlikWyswietlacz2=0;
	TCNT1=0;
}

uint8_t pomocnicza1=0;
uint8_t pomocnicza2=0;

ISR(TIMER1_COMPA_vect){
	globalLicznik++;	
	if(globalLicznik >= odczytanyCzas){
		cli();
		globalLicznik=0;
		
		pomocnicza1=0;
		pomocnicza2=0;
		
		if ((PINA & (1<<PA2)) == 0){
			while(wyslijDoWyswietlacza(tablicaNazw1[licznikPlikWyswietlacz1],1)){
				licznikPlikWyswietlacz1++;
				pomocnicza1++;
				if(licznikPlikWyswietlacz1==7) licznikPlikWyswietlacz1=0;
				if(pomocnicza1 == 7) break;
			}
		}
			
			PORTC &= (~(1<<PC1)); 	// led czerwony off
			PORTC &= (~(1<<PC2)); 	// led zielony 1 off
			PORTC &= (~(1<<PC3)); 	// led zielony 2 off
	
		if(pomocnicza1 == 7){
				// wyswietla stan odczytu 1.txt: (zielona_1 + czerwona * err1)
				PORTC |= (1<<PC2); 	// led zielony 1 on
				_delay_ms(300);
				for (i=0; i<err1; i++) {	// ktory error? pierwszego wyswietlacza
					PORTC |= (1<<PC1); 	// led czerwony on
					_delay_ms(100);
					PORTC &= (~(1<<PC1)); 	// led czerwony off
					_delay_ms(200);
				}
				PORTC &= (~(1<<PC2)); 	// led zielony 1 off
				_delay_ms(100);
		}
	
			PORTC &= (~(1<<PC1)); 	// led czerwony off
			PORTC &= (~(1<<PC2)); 	// led zielony 1 off
			PORTC &= (~(1<<PC3)); 	// led zielony 2 off	
	
		if ((PINA & (1<<PA2)) == 0){
			while(wyslijDoWyswietlacza(tablicaNazw2[licznikPlikWyswietlacz2],2)){
				licznikPlikWyswietlacz2++;
				pomocnicza2++;
				if(licznikPlikWyswietlacz2==7) licznikPlikWyswietlacz2=0;
				if(pomocnicza2 == 7) break;
			}
		}
			PORTC &= (~(1<<PC1)); 	// led czerwony off
			PORTC &= (~(1<<PC2)); 	// led zielony 1 off
			PORTC &= (~(1<<PC3)); 	// led zielony 2 off
		
		if(pomocnicza2 == 7){			
			// wyswietla stan odczytu 2.txt: (zielona_2 + czerwona * err2)
			PORTC |= (1<<PC3); 	// led zielony 2 on
			_delay_ms(300);
			for (i=0; i<4; i++) {	// ktory error? drugiego wyswietlacza
				PORTC |= (1<<PC1); 	// led czerwony on
				_delay_ms(100);
				PORTC &= (~(1<<PC1)); 	// led czerwony off
				_delay_ms(200);
			}
			PORTC &= (~(1<<PC3)); 	// led zielony 2 off
			_delay_ms(100);
		}
		
			PORTC &= (~(1<<PC1)); 	// led czerwony off
			PORTC &= (~(1<<PC2)); 	// led zielony 1 off
			PORTC &= (~(1<<PC3)); 	// led zielony 2 off		
		
		  licznikPlikWyswietlacz1++;
		  licznikPlikWyswietlacz2++;
		  	  
		  if(licznikPlikWyswietlacz1 == 7) licznikPlikWyswietlacz1= 0;
		  if(licznikPlikWyswietlacz2 == 7) licznikPlikWyswietlacz2= 0;
		  	  		
		TCNT1=0;
		sei();
	}
}