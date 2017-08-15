/*
 * m16_marix.c
 *
 * Created: 2017-06-12 09:38:16
 * Author : Dawid
 */ 

#ifndef F_CPU
#define F_CPU 8000000UL
#endif

#define BREAKPOINT asm volatile("nop");

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include <avr/pgmspace.h>
#include "font1.h"


#define _NOP() __asm__ __volatile__("nop")
//#define _NOP() asm volatile("nop")


#define delay_l()	for (x=0; x<4; x++) y++
#define delay_s()	y++//for (x=0; x<3; x++) y++


void SzpilkaNaB2();
void SzpilkaNaB3();
void SzpilkaNaB4();

void efekt1();

void EEPROM_write(unsigned int uiAddress, unsigned char ucData);
unsigned char EEPROM_read(unsigned int uiAddress);

#define POWT_MAX 15
#define BUF_LEN_MAX 200

uint8_t x, y=0;	// dla delay_l()
//y=0;

#define ZapalBiala() PORTB|=0x04		// B2
#define ZgasBiala() PORTB&=0xfb			// B2

#define ZapalNiebieska() PORTB|=0x08	// B3
#define ZgasNiebieska() PORTB&=0xf7		// B3

#define ZapalZielona() PORTB|=0x10		// B4
#define ZgasZielona() PORTB&=0xef		// B4


// initialize timer, interrupt and variable
void timer1_init()
{
	// set up timer with prescaler = 64 and CTC mode
	TCCR1B |= (1 << WGM12)|(1 << CS11)|(1 << CS10);
	// initialize counter
	TCNT1 = 0;
	// initialize compare value
	OCR1A = 4;
	// enable compare interrupt
	TIMSK |= (1 << OCIE1A);
	// enable global interrupts
}

// TJ
#define MASTER_IN (1<<PD1)
#define PORT_MASTER_IN PORTD
#define ON_MASTER_IN  PORT_MASTER_IN |= MASTER_IN
#define OFF_MASTER_IN PORT_MASTER_IN &=~ MASTER_IN

#define MASTER_OUT (1<<PD3)
#define PIN_MASTER_OUT PIND
#define PORT_MASTER_OUT PORTD
#define IF_MASTER_OUT_ON PIN_MASTER_OUT & MASTER_OUT
#define IF_MASTER_OUT_OFF (!(PIN_MASTER_OUT & MASTER_OUT))

#define DANE (1<<PD0)
#define PORT_DANE PORTD
#define PIN_DANE PIND
#define IF_DANE_ON (PIN_DANE & (DANE))
#define IF_DANE_OFF (!(PIN_DANE & (DANE)))

#define SCLK_WYSW1 (1<<PD2)
#define PORT_SCLK_WYSW1 PORTD
#define IF_SCLK_WYSW1_ON (PIND & (1<<PD2))
#define IF_SCLK_WYSW1_OFF (!(PIND & (1<<PD2)))
//END TJ

void PinsInit()	// gasi wszystko
{ 
	// wybor 1/8:        A 01110000	(1 to jest)
	// wybor scalaka238: A 10001111
	// wybor scalaka238: C 11100000
	// wybor scalaka238: D 00110000
	// wybor LED:        A 00000000
	// wybor LED:        C 00011111
	// wybor LED:        D 11000000
	// RS:               D 00001111 (INT1 D3, INT0 D2, TX D1, RX D0)
	// debug:            B 00011100
	
	DDRA = 0xff;	// [0 in, 1 out]
	DDRB = 0x1f;	// tylko MOSI, MISO, SCK in
	DDRC = 0xff;	//
	DDRD = 0xfa;	// tylko INT0, RX in

	PORTA = 0x00;	// gasi wszystko (wyswietlacz 80x7 i pomocnicze ledy)
	PORTB = 0x00;
	PORTC = 0x00;
	PORTD = 0x0f;
	
//START TJ	
	DDRD |= MASTER_IN;
	DDRD &=~ (MASTER_OUT | SCLK_WYSW1);
	DDRD &=~ DANE;
		
	PORT_DANE |= DANE;
	ON_MASTER_IN;
	PORT_MASTER_OUT |= MASTER_OUT;
//END TJ
}

void ZgasWyswietlacz()
{
	//	ZapalBiala();	// B2

	PORTC = 0x00;	// gasi 5x led i 3x 238
	PORTD &= 0x0f;	// gasi 2x led i 2x 238 (RS bez zmian)
	PORTA = 0x00;	// gasi 5x 238 i zeruje wybor 1/8
	//	PORTB &= 0xff;	// pomocnicze ledy (B2 B3 B4) zostaja jak byly, I2C (B0 B1) zostaje bez zmian, SPI(B5 B6 B7) bez zmian
}

void ZgasStary_ZapalKolumneRzad(uint8_t kolumna, uint8_t rzad)
{

	uint8_t portA = 0, portC = 0, portD = 0;

	// wybiera kolumne (1 z 8)
	if (kolumna & 0b001) portA |= 0b00010000;		// A4	// 4.4us
	if (kolumna & 0b010) portA |= 0b00100000;		// A5
	if (kolumna & 0b100) portA |= 0b01000000;		// A6

	// zapala kolumne (ktory scalak -> 1/8)
	kolumna = kolumna >> 3;	// zsuwa 3 najmniejsze bity
	switch (kolumna) {
		case 0: portA |= 0b00000001; break;		// A0
		case 1: portA |= 0b00000010; break;		// A1
		case 2: portA |= 0b00000100; break;		// A2
		case 3: portA |= 0b00001000; break;		// A3
		case 4: portA |= 0b10000000; break;		// A7
		case 5: portC |= 0b10000000; break;		// C7
		case 6: portC |= 0b01000000; break;		// C6
		case 7: portC |= 0b00100000; break;		// C5
		case 8: portD |= 0b00010000; break;		// D4
		case 9: portD |= 0b00100000; break;		// D5
	}

	// zapala rzad (ledy)
	if (rzad & 0b00000001) portC |= 0b00010000;	// C4
	if (rzad & 0b00000010) portC |= 0b00001000;	// C3
	if (rzad & 0b00000100) portC |= 0b00000100;	// C2
	if (rzad & 0b00001000) portC |= 0b00000010;	// C1
	if (rzad & 0b00010000) portC |= 0b00000001;	// C0

	if (rzad & 0b00100000) portD |= 0b10000000;	// D5
	if (rzad & 0b01000000) portD |= 0b01000000;	// D4
	//	ZapalBiala();	// B2

	// gaszenie starych
	PORTC = 0x00;	// gasi 5x led i 3x 238
	PORTD &= 0x0f;	// gasi 2x led i 2x 238 (RS bez zmian)
	PORTA = 0x00;	// gasi 5x 238 i zeruje wybor 1/8

	delay_s();
	//	delay_l();

	// zapalanie nowych
	PORTA = portA;
	PORTC = portC;
	PORTD |= portD;	// tu port RS
}

static const uint8_t PROGMEM mapa[] = {
	0x3b, 0x4e, 0x3a, 0x49, 0x39, 0x4d, 0x38, 0x4b,
	0x28, 0x4f, 0x29, 0x4a, 0x2a, 0x4c, 0x2b, 0x48,
	0x2c, 0x3f, 0x2d, 0x3e, 0x2e, 0x3d, 0x2f, 0x3c,
	0x30, 0x40, 0x31, 0x44, 0x32, 0x42, 0x33, 0x46,
	0x34, 0x41, 0x35, 0x45, 0x36, 0x47, 0x37, 0x43,
	0x18, 0x07, 0x1c, 0x06, 0x1a, 0x05, 0x1e, 0x04,
	0x19, 0x00, 0x1d, 0x01, 0x1b, 0x02, 0x1f, 0x03,
	0x20, 0x16, 0x21, 0x12, 0x22, 0x14, 0x23, 0x10,
	0x24, 0x0f, 0x25, 0x0b, 0x26, 0x0d, 0x27, 0x09,
	0x11, 0x0e, 0x15, 0x0a, 0x13, 0x0c, 0x17, 0x08,
};

uint8_t wysw[81];

volatile uint8_t tekst0[BUF_LEN_MAX];//="71TEST1";
//volatile uint8_t tekst1[BUF_LEN_MAX];
volatile uint8_t * tekst = tekst0;	// wskaznik na tekst0 lub tekst1

volatile uint8_t tekst1_wyswietlany;	// ==0 tekst0 wyswietlany, !=0 tekst1 wyswietlany,
volatile uint8_t zmiana_bufora;	// ==0 tekst0 wyswietlany, !=0 tekst1 wyswietlany,

uint8_t rx_ktory_bit;	// uzywane tylko w przerwaniu, bitowe odbieranie, nie sprzetowe!
uint8_t rx_ktory_bajt;	// na opadajacym zboczu INT0 (D2), pobiera jedna wartosc z RX (D0)
uint8_t rx_jest_error;
uint8_t rx_nowy_bajt;
uint8_t rx_startTransmisji=1;

	uint8_t ktora_litera;	// potrzebne by zapelnic 80 kolumn co przesuniecie
	uint8_t kolumna_litery;	// kolumna litery (0 1 2 3 4 5, potem od poczatku)

	uint8_t litera;	// chwilowa zmienna pomocnicza

	// faza, faza_kol sprawiaja ze tekst sie przesowa
	uint8_t faza_kol = 0;	// faza kolumny (0 1 2 3 4 5, potem od poczatku)
	// 5 to wolna kolumna miedzy literami
	uint8_t faza = 0;		// +1 w momecie wyzerowania faza_kol, =0 gdy >= dlugosc_tekstu


	uint8_t kol;
	uint8_t kol_popr, ledy;
	uint8_t m;
	uint8_t powt, powtMax = POWT_MAX;		// powtMax = spowolnienie, wieksze = wolniej przesowa sie
	uint16_t a;
	


	uint8_t wybor_szybkosci = 2;	// jeszcze zmieni
	uint8_t dlugosc_tekstu = 3;	// jeszcze przeliczy


// START ****************************************************TOMEK ZMIANA**************************************************************
/*INFORMACJE
-maksymalnie 13 znakow w jednej linii
-kolumna zerowa jest od lewej strony
*/

uint16_t licznikKoncaCzasu=0;



uint8_t volatile przerwijAnimacje=0;
uint8_t MatrycaBufor1[81];
uint8_t MatrycaBufor2[81];
uint8_t* wskaznik=MatrycaBufor2;

uint8_t matrycaNegujF=0;
uint8_t ErrorFlag=0; 

uint8_t licznik=0;
uint8_t licznikPomocniczy=0;

uint8_t tekstTJ[200]="@@@@@ drugieSlowo";//EKSTREMALNIEEEEEEEEEE DLUUUUUUUUUGI TEKST !!!!!!!!!!!!!!!!!!";
uint8_t pozycjaSlow[30][2];
uint8_t dlugoscTekstu;
uint8_t liczbaSlow;
uint8_t buforSlowa1[14];
uint8_t buforSlowa2[14];
uint16_t vPredkosc=0;
uint8_t wyborAnimacji=1;

void opoznienie(uint16_t zwloka){
	for(uint16_t i=0; i < zwloka; i++)	_delay_us(800);
}

/* Funkcje dotyczace napisow i stringow */
uint8_t wyznaczLiczbeZnakow(uint8_t* wskaznikTekst){
	uint8_t dlugosc=0;
	while(wskaznikTekst[dlugosc]){
		dlugosc++;
		if(dlugosc>=200){
			dlugosc=0;
			return dlugosc;
		}
	}
	return dlugosc;
}

uint8_t wyznaczLiczbeSlow(uint8_t* wskaznikTekst){
	
	uint8_t pomocnicza=0;
	uint8_t liczbaSlow=1;
	
	while(wskaznikTekst[pomocnicza]){
		if(wskaznikTekst[pomocnicza]==' '){			
				while(wskaznikTekst[pomocnicza+1]==' '){
					pomocnicza++;
				}						
			liczbaSlow++;	
			
		} 
		pomocnicza++;
		if(pomocnicza>200) break;
	}
	
	if(wskaznikTekst[pomocnicza-1]==' '){
		liczbaSlow--;
	}
	
	return liczbaSlow;
}

//Funkcja zapisuje pozycje slow w tablicy globalnej pozycjaSlow[][2] - pierwszy bajt to poczatek, a drugi to koniec slowa
void wyznaczPozycjeSlow(uint8_t* tekstWew, uint8_t tekstDlugosc){

	uint8_t slowo=0;
	pozycjaSlow[0][0]=0;
	//uint8_t flagaSlowo=1;
	
	for(uint8_t kursor=1; kursor <= tekstDlugosc; kursor++){
		if(tekstWew[kursor]== 0) pozycjaSlow[slowo][1]=kursor;
		
		if(tekstWew[kursor]==' '){	
			
			pozycjaSlow[slowo][1]=kursor-1;
			while(tekstWew[kursor+1]==' '){
				kursor++;
			}
			kursor++;
			slowo++;
			pozycjaSlow[slowo][0]=kursor;
		}
	}
}


//ERROR START
	//Sprawdz Czy Jest Tekst
uint8_t ErrorNiewlasciwyTekst(uint8_t* wskaznikTekstu, uint8_t tekstDlugosc){
	
		if(tekstDlugosc == 0) return 1;
		if(wskaznikTekstu[0] == ' ') return 1;
	
// 		for(uint8_t i=0; i <tekstDlugosc; i++) {
// 			if(wskaznikTekstu[i]=='\t') return 1;
// 			if(wskaznikTekstu[i]=='\n') return 1;			
// 		}
				
		for(uint8_t i=0; i < tekstDlugosc; i++){
				if(wskaznikTekstu[i]!=0 && wskaznikTekstu[i]!=' ') return 0;	
		}	
		
		return 1;
}


//ERROR END

//mozna kopiowac to samo slowo ale od lewej do prawej
void kopiujString(uint8_t* tekstKopia, uint8_t* tekstOryginalny, uint8_t pozycjaPoczatek,
uint8_t pozycjaKoniec){
	
	for(int i=pozycjaPoczatek, k=0; i <= pozycjaKoniec; i++, k++){
		tekstKopia[k]=tekstOryginalny[i];
		if(i == pozycjaKoniec) tekstKopia[k+1]=0;
	}	
}

void czyscTekst(uint8_t* tablicaTekstu){
	
	for(uint8_t i=0; i <BUF_LEN_MAX-1; i++){
		tablicaTekstu[i]=0x00;
	}
}

//pracuje na globalnej tablicy pozycjaSlow[][]
//Dodano zabezpieczenie dla dlugosci slow (MAX 13 znakow)
void kopiujSlowo(uint8_t* tekstKopia, uint8_t*tekstOryginalny, uint8_t slowo){
	
	uint8_t dlugoscSlowa=0;
	dlugoscSlowa = pozycjaSlow[slowo-1][1] - pozycjaSlow[slowo-1][0];
	
	if( dlugoscSlowa >= 13){
		pozycjaSlow[slowo-1][1]=pozycjaSlow[slowo-1][0]+12;
	}

	kopiujString(tekstKopia, tekstOryginalny, pozycjaSlow[slowo-1][0], pozycjaSlow[slowo-1][1]);	
}

/* Funkcje dotycz¹ce matrycy */
void matrycaZapal(uint8_t* matrycaWynik){
	for(int i=0; i<80; i++){		
		if(matrycaNegujF==1) matrycaWynik[i]=0x00;
		else matrycaWynik[i]=0x7f;
	}
}

void matrycaZgas(uint8_t* matrycaWynik){
	for(int i=0; i<80; i++){
		if(matrycaNegujF==1) matrycaWynik[i]=0x7f;
		else matrycaWynik[i]=0x00;
	}
}

void matrycaNeguj(uint8_t* matryca){
	for(int i=0; i<80; i++) matryca[i]=~matryca[i];
}

void matrycaKopiuj(uint8_t* matrycaKopia, uint8_t* matrycaOryginal){
	
	for(uint8_t i=0; i<80; i++)	matrycaKopia[i] = matrycaOryginal[i];			
}

// pozycja od -5 do 79 (od -5 bo litera moze zaczynac sie czesciowo po lewej stronie
void literkaMatryca(uint8_t* matryca, uint8_t literka, int8_t pozycja){
		if(pozycja >= 0){
			for (int8_t i=0; (i < 5)&&( i+pozycja <80 ); i++) {
// 				if(i+pozycja < 80){
// 					matryca[i+pozycja] = pgm_read_byte(&(tablica_font[literka-32][i]));
// 					}
				if(matrycaNegujF==1){
					matryca[i+pozycja] = ~pgm_read_byte(&(tablica_font[literka-32][i]));
				}else{
					matryca[i+pozycja] = pgm_read_byte(&(tablica_font[literka-32][i]));
				}
				
		}
		}else{
			for (int8_t i=0; ( i < 5+pozycja )&&( i<80) ; i++) {
				
				if(matrycaNegujF==1){
					matryca[i] = ~pgm_read_byte(&(tablica_font[literka-32][i-pozycja]));
					}else{
					matryca[i] = pgm_read_byte(&(tablica_font[literka-32][i-pozycja]));
				}
								
// 				if(i < 80){
// 					matryca[i] = pgm_read_byte(&(tablica_font[literka-32][i-pozycja]));
// 				}
			}
		}		
}

void zgas_od_do(uint8_t* matryca, uint8_t pozycjaPoczatek, uint8_t pozycjaKoniec){
	for (int i=pozycjaPoczatek; ( i < pozycjaKoniec )&&( i< 80) ; i++) {
		
		if(matrycaNegujF==1) matryca[i]=0x7f;
		else matryca[i]=0x00;
	}
}

void zgasLiterke(uint8_t* matryca, uint8_t pozycja){
	for (int i=pozycja; ( i < pozycja+5 )&&( i<80 ); i++) {
		if(matrycaNegujF==1) matryca[i]=0x7f;
		else matryca[i]=0x00;
	}
}

//kolumna od 0 do 4 - wyswietla okreslona kolumne literki na pozycji od 0 do 79
void literkaKolumnaMatryca(uint8_t* matryca, uint8_t literka, int16_t pozycja, uint8_t kolumna){
	
			if(matrycaNegujF==1){
				if( (pozycja>=0) && (pozycja < 80)) matryca[pozycja] = ~pgm_read_byte(&(tablica_font[literka-32][kolumna]));
			}else{
				if( (pozycja>=0) && (pozycja < 80)) matryca[pozycja] = pgm_read_byte(&(tablica_font[literka-32][kolumna]));
			}
}

//pozycja od -200*5-200 do 79
void napisMatryca2wersja(uint8_t* matryca, uint8_t* tekstWew, uint8_t dlugoscTekstu, int16_t pozycja){
	
	if(pozycja >= 0){
		for(uint8_t i=0; (i < dlugoscTekstu) && (i<200); i++){
			for(uint8_t k=0; k<5; k++){
				if(pozycja+i*5+i+k < 80 ){
						if(przerwijAnimacje == 1){
							matrycaZgas(wysw);
							return;
						}
					literkaKolumnaMatryca(matryca, tekstWew[i], pozycja+i*5+i+k, k);
				}
			}
		}
		}else{
		for(uint8_t i=0; (i < dlugoscTekstu) && (i<200) ; i++){
			for(uint8_t k=0; k<5; k++){
				if((int16_t)(pozycja+i*5+i+k) >= 0){	
					if(przerwijAnimacje == 1){
						matrycaZgas(wysw);
						return;
					}						
					literkaKolumnaMatryca(matryca, tekstWew[i], (int16_t)(pozycja+i*5+i+k), k);
				}
			}
		}
	}
}


//najlepiej nie wysylac dluzszych slow niz 13 znakow
void napisSrodek(uint8_t* matryca, uint8_t* slowo){

	uint8_t dlugoscSlowa=0;

	while (slowo[dlugoscSlowa]) dlugoscSlowa++;
	
	uint8_t prawdziwaDlugosc =dlugoscSlowa*5+dlugoscSlowa-1;
	uint8_t przerwa = (80-prawdziwaDlugosc)/2;
	
	napisMatryca2wersja(matryca, slowo, dlugoscSlowa, przerwa);
}

void animacjaLiterka_od_do(uint8_t* matryca, uint8_t literka, uint8_t pozycjaPoczatek,
uint8_t pozycjaKoniec, uint16_t predkosc){
	
	if(pozycjaKoniec > pozycjaPoczatek)
	{
		for(uint8_t i = pozycjaPoczatek; i<=pozycjaKoniec; i++){
			literkaMatryca(matryca, literka, i);
			opoznienie(predkosc);
				if(przerwijAnimacje == 1){
					matrycaZgas(wysw);
					return;
				}			
			zgasLiterke(matryca, i);
		}
		}else if(pozycjaPoczatek > pozycjaKoniec){
		for(int16_t i = pozycjaPoczatek; i>=pozycjaKoniec; i--){
			literkaMatryca(matryca, literka, i);
			opoznienie(predkosc);
				if(przerwijAnimacje == 1){
					matrycaZgas(wysw);
					return;
				}
			zgasLiterke(matryca, i);
		}
		
		}else{
		literkaMatryca(matryca, literka, pozycjaPoczatek);
				if(przerwijAnimacje == 1){
					matrycaZgas(wysw);
					return;
				}
	}
}

void animacjaPoLiterceSlowoSrodek(uint8_t* matryca, uint8_t* matrycaBufor, uint8_t* slowo,  uint16_t predkosc){
	uint8_t dlugoscSlowa=0;
	
	while (slowo[dlugoscSlowa]) dlugoscSlowa++;
	
	uint8_t prawdziwaDlugosc = dlugoscSlowa*5+dlugoscSlowa-1;
	uint8_t przerwa = (80-prawdziwaDlugosc)/2;
	
	uint8_t pozycjaLiterki=przerwa;
	
	for(uint8_t i=0; i<dlugoscSlowa; i++){
			if(przerwijAnimacje == 1){
				matrycaZgas(wysw);
				return;
			}
		animacjaLiterka_od_do(matryca, slowo[i], 79, pozycjaLiterki, predkosc);
		literkaMatryca(wysw, slowo[i], pozycjaLiterki);
		pozycjaLiterki=przerwa+(i+1)*5+(i+1);
	}		
}

void animacjaPoLiterceSlowoOdBoku(uint8_t* matryca, uint8_t* matrycaBufor, uint8_t* slowo,  uint16_t predkosc){
	uint8_t dlugoscSlowa=0;
	uint8_t pozycjaLiterki=0;
	
	while (slowo[dlugoscSlowa]) dlugoscSlowa++;
	
	for(uint8_t i=0; i<dlugoscSlowa; i++){
		animacjaLiterka_od_do(matryca, slowo[i], 79, pozycjaLiterki, predkosc);
		literkaMatryca(wysw, slowo[i], pozycjaLiterki);
		pozycjaLiterki = (i+1)*5+(i+1);
	}	
}

//ToDo Sprawdzic
void animacjaNapisPrzesunPrawo(uint8_t* matryca, uint8_t* napis, uint8_t dlugosc, uint16_t predkosc){
	uint16_t liczbaPetli;
	liczbaPetli = dlugosc + 80;
	for(uint16_t i=0; i < liczbaPetli; i++){
		//napisMatryca(matryca, napis, dlugosc, 79)
		opoznienie(predkosc);
		matrycaZgas(matryca);
	}
}

#ifndef MATRYCA_GLOBAL
#define MATRYCA_GLOBAL wysw
#endif
	void animacjaMiganieGlobal(uint8_t ilosc_migniec, uint16_t predkosc){
		for(int i=0; i<ilosc_migniec; i++)
		{
			matrycaZapal(MATRYCA_GLOBAL);
			opoznienie(predkosc*10);
			matrycaZgas(MATRYCA_GLOBAL);
			opoznienie(predkosc*10);
		}
	}
#undef MATRYCA_GLOBAL


// START - NIE UZYWANE ALE MOGA BYC PRZYDATNE
		/*
		poziom od -6 do 6 cos widac jeszcze
		-6 - do gory
		 6 - do dolu
		 */

		void napisPoziom(uint8_t* matryca, uint8_t* matrycaBufor, uint8_t* slowo, int8_t poziom){
	
			napisSrodek(matrycaBufor, slowo);
	
			//Specjalnie nie jest wygaszana matryca
			for(uint8_t i=0; i<80; i++){
				if(poziom >= 0) matryca[i] |= matrycaBufor[i] << poziom;
				else			matryca[i] |= matrycaBufor[i] >> (-poziom);
			}	
			matrycaZgas(matrycaBufor);
		}

		void dwaNapisyGoraDol(uint8_t* matryca, uint8_t* matrycaBufor, uint8_t* tekstGora,
		uint8_t* tekstDol, int8_t poziomZeroOsiem){

			matrycaZgas(wysw);
			matrycaZgas(matrycaBufor);
			napisPoziom(matryca, matrycaBufor, tekstGora, -8+poziomZeroOsiem);
			napisPoziom(matryca, matrycaBufor, tekstDol, poziomZeroOsiem);
		}

		void animacjaDwaNapisyGoraDol(uint8_t* matryca, uint8_t* matrycaBufor,
		uint8_t* tekstGora, uint8_t* tekstDol, uint16_t predkosc){
	
			for(uint8_t i=0; i<9; i++){
				dwaNapisyGoraDol(matryca, matrycaBufor, tekstGora, tekstDol, i);	
					if(przerwijAnimacje == 1){
						matrycaZgas(wysw);
						return;
					}
				opoznienie(predkosc);			
				if( i==0) opoznienie(predkosc*5);
			}
			
			matrycaZgas(matrycaBufor);
		}
// END - NIE UZYWANE ALE MOGA BYC PRZYDATNE

void napisKursor(uint8_t* matryca, uint8_t* matrycaBufor1, uint8_t* matrycaBufor2, uint8_t* slowo, uint8_t kursor){
	
	matrycaZgas(matrycaBufor1);
	napisSrodek(matrycaBufor1, slowo);	
	matrycaZgas(matrycaBufor2);
	
	
	for(uint8_t i=0; i <=kursor; i++){
		
		if(i < 80){			
				matrycaBufor2[i]= matrycaBufor1[i];
					if(przerwijAnimacje == 1){
						matrycaZgas(wysw);
						return;
					}
				
				if(matrycaNegujF==1){
					if(i==kursor) matrycaBufor2[i]= 0x00;
					}else{
					if(i==kursor) matrycaBufor2[i]= 0xff;
				}
		}else{
			matrycaBufor2[i]= matrycaBufor1[i];
					if(przerwijAnimacje == 1){
						matrycaZgas(wysw);
						return;
					}
			
		}		
	}
	matrycaKopiuj(matryca,matrycaBufor2);
	
}

void animacjaKursor(uint8_t* matryca, uint8_t* matrycaBufor1, uint8_t* matrycaBufor2, 
					uint8_t* slowo, uint16_t predkosc){
	
	for(uint8_t i=0; i <= 80; i++){
									if(przerwijAnimacje == 1){  
										matrycaZgas(wysw);
										return;										
									}
		napisKursor(matryca, matrycaBufor1, matrycaBufor2, slowo, i);
		opoznienie(predkosc);
	}
}

//zwraca pozycje ostatniego od lewej znaku

int16_t napisMatrycaOdKonca(uint8_t* matryca, uint8_t* tekstWew, uint8_t dlugosc, int16_t pozycja){	
	
		int16_t wynikFunkcji=0;
		
		for(int16_t i=0 ; i < dlugosc; i++){
			
			for(int8_t k=0; k<5; k++){
				wynikFunkcji=pozycja-i*5-i-k;
				if( wynikFunkcji >=0){					
						literkaKolumnaMatryca(matryca, tekstWew[dlugosc-i-1], wynikFunkcji, 4-k);						
				}
				if(wynikFunkcji <= 0) return wynikFunkcji=0;
			}
			
		}			
		return wynikFunkcji;
}
				
//blizniacza funkcja
int16_t napisMatrycaDoPrzodu(uint8_t* matryca, uint8_t* tekstWew, uint8_t dlugosc, int16_t pozycja){
	
	int16_t wynikFunkcji=0;
	
	for(int8_t i=0 ; i < dlugosc; i++){
		for(int8_t k=0; k<5; k++){
			wynikFunkcji=pozycja+i*5+i+k;
			if( wynikFunkcji <80){				
				literkaKolumnaMatryca(matryca, tekstWew[i], wynikFunkcji, k);
			}
			if(wynikFunkcji >= 79) return wynikFunkcji=79;
		}
	}
	
	return wynikFunkcji;
}


//pozycja od 0 do 79
void napisMatrycaPetla(uint8_t* matryca, uint8_t* tekstWew, uint8_t dlugoscTekstu, int16_t pozycja){
		
	int16_t ostatniaPozycja=0;
		
	ostatniaPozycja= napisMatrycaOdKonca(matryca, tekstWew, dlugoscTekstu, pozycja-2);
	
	//wypisuje tekst od prawej do lewej
	while(ostatniaPozycja !=0){
			ostatniaPozycja=napisMatrycaOdKonca(matryca, tekstWew, dlugoscTekstu, ostatniaPozycja-2);
		}
		
	//od lewej do prawej
	ostatniaPozycja = napisMatrycaDoPrzodu(matryca, tekstWew, dlugoscTekstu, pozycja);

	while(ostatniaPozycja != 79){
		ostatniaPozycja = napisMatrycaDoPrzodu(matryca, tekstWew, dlugoscTekstu, ostatniaPozycja+2);
	}	
	
}

void animacjaCalyTekstLewo(uint8_t* matryca, uint8_t* matrycaBufor1, uint8_t* matrycaBufor2,
uint8_t* calyTekst, uint8_t dlugoscTekstu, uint16_t predkosc){

	int16_t prawdziwaDlugosc = dlugoscTekstu*5+dlugoscTekstu;

	for(int16_t i=79; i>-prawdziwaDlugosc+1; i--){		
		matrycaZgas(matrycaBufor1);
		
		napisMatryca2wersja(matrycaBufor1, tekstTJ, dlugoscTekstu, i);
		matrycaKopiuj(matryca,matrycaBufor1);
				if(przerwijAnimacje == 1){
					matrycaZgas(wysw);
					return;
				}	
		 opoznienie(predkosc);
		 opoznienie(predkosc);

	}
	matrycaZgas(matrycaBufor1);	//inaczej w innych animacjach smieci
}

void animacjaCalyTekstPrawo(uint8_t* matryca, uint8_t* matrycaBufor1, uint8_t* matrycaBufor2,
uint8_t* calyTekst, uint8_t dlugoscTekstu, uint16_t predkosc){
		
	int16_t prawdziwaDlugosc = dlugoscTekstu*5+dlugoscTekstu;
	
	
	for(int16_t i=-prawdziwaDlugosc; i<80; i++){
		matrycaZgas(matrycaBufor1);
		napisMatryca2wersja(matrycaBufor1, tekstTJ, dlugoscTekstu, i);
		
		matrycaKopiuj(matryca,matrycaBufor1);

				if(przerwijAnimacje == 1){
					matrycaZgas(wysw);
					return;
				}	
		opoznienie(predkosc);
		opoznienie(predkosc);
	}
	matrycaZgas(matrycaBufor1);	//inaczej w innych animacjach smieci
}

//
void zapisDoEEPROM(uint8_t * tekstWew, uint16_t dlugoscTekstuWew){
	
//zapisywanie do EEPROM - start
			cli();		// Disable Global Interrupts
			EEPROM_write(0x000, 0xa3);	// oznakowanie, ze sa istotne dane w EEPROM

			for (a=0; a<dlugoscTekstuWew; a++) {
				EEPROM_write(a+1, tekstWew[a]);
			}
			EEPROM_write(dlugoscTekstuWew+1, 0x00);	// na koniec zapisuje 0
			sei();		// Enable Global Interrupts
			// zapisywanie do EEPROM - koniec
}


//zmodyfikowany oryginal
void matrycaPrzesunOryginal(uint8_t* tekstWew, uint16_t tekstDlugosc, uint16_t predkosc)
{
	dlugosc_tekstu = 0;
	while (tekstWew[dlugosc_tekstu])	{	// wyznacza dlugosc tekstu
		dlugosc_tekstu++;
	}	
	uint16_t prawdziwa_dlugosc;
	prawdziwa_dlugosc=dlugosc_tekstu*5+dlugosc_tekstu-1;
	uint16_t powtorzen = prawdziwa_dlugosc+80;	
		
	for(int i=0; i<=powtorzen; i++)
	{
		
		kolumna_litery = faza_kol;
		ktora_litera = faza;


		// wypelnia 80 kolumn wyswietlacza danymi (przesuniecie o 1 kolumne)
		for (m=0; m<80; m++) {
			if (ktora_litera >= dlugosc_tekstu) {
				ktora_litera = 0;
			}

			if (kolumna_litery < 5) {
				litera = tekstWew[ktora_litera];
				wysw[m] = pgm_read_byte(&(tablica_font[litera-32][kolumna_litery]));
			}
			else {	// kolumna_litery == 5
				wysw[m] = 0;	// ledy zgaszone, czarna kolumna miedzy literami
			}

			kolumna_litery++;
			if (kolumna_litery > 5) {
				kolumna_litery = 0;
				ktora_litera++;
			}
		}

		faza_kol++;
		if (faza_kol > 5) {
			faza_kol = 0;
			faza++;
			if (faza >= dlugosc_tekstu) {
				faza = 0;
			}
		}
			
			opoznienie(predkosc);
	}	
}

//funkcja pracuje na globalnych zmiennych oprocz argumentow
void wszystkieAnimacje(uint8_t wyborAnimacji, int16_t Vpredkosc){
		
	switch (wyborAnimacji) {
		case 1:    //pojawia sie tekst po kolei na srodku
			for(uint8_t i=1; i <= liczbaSlow; i++){
				kopiujSlowo(buforSlowa1, tekstTJ, i);
				matrycaZgas(wysw);
				opoznienie(Vpredkosc*70);
					if(przerwijAnimacje == 1){
						matrycaZgas(wysw);
						return;
					}
				napisSrodek(wysw, buforSlowa1);
				opoznienie(Vpredkosc*180);
			}
					
		break;
		
		case 2:	//animacja kursor
			for(uint8_t i=1; i <= liczbaSlow; i++){
				kopiujSlowo(buforSlowa1, tekstTJ, i);
				animacjaKursor(wysw, MatrycaBufor1, MatrycaBufor2, buforSlowa1, Vpredkosc*2);
				matrycaZgas(MatrycaBufor1);
				matrycaZgas(MatrycaBufor2);
					if(przerwijAnimacje == 1){
						matrycaZgas(wysw);
						return;
					}
				opoznienie(Vpredkosc*300);
				matrycaZgas(wysw);
			}
		break;
		
		case 3:	//animacja po literce SRODEK			
			for(uint8_t i=1; i <= liczbaSlow; i++){
				kopiujSlowo(buforSlowa1, tekstTJ, i);
				animacjaPoLiterceSlowoSrodek(wysw, MatrycaBufor1, buforSlowa1, Vpredkosc);
					if(przerwijAnimacje == 1){
						matrycaZgas(wysw);
						return;
					}
				opoznienie(Vpredkosc*230);
				matrycaZgas(MatrycaBufor1);
				matrycaZgas(wysw);
			}		
		break;

		
		case 4:	//animacja po literce LEWY BOK				
			for(uint8_t i=1; i <= liczbaSlow; i++){
				kopiujSlowo(buforSlowa1, tekstTJ, i);
				animacjaPoLiterceSlowoOdBoku(wysw, MatrycaBufor1, buforSlowa1, Vpredkosc);
					if(przerwijAnimacje == 1){
						matrycaZgas(wysw);
						return;
					}
				opoznienie(Vpredkosc*230);
				matrycaZgas(wysw);
				matrycaZgas(MatrycaBufor1);
			}
		break;
		
		
		case 5:	//animacja od prawej do lewej
				animacjaCalyTekstLewo(wysw, MatrycaBufor1, MatrycaBufor2, tekstTJ, dlugoscTekstu, Vpredkosc*2);
				matrycaZgas(wysw);
				matrycaZgas(MatrycaBufor1);
				matrycaZgas(MatrycaBufor2);
		break;		
		
		case 6:		//animacja od lewej do prawej
				animacjaCalyTekstPrawo(wysw, MatrycaBufor1, MatrycaBufor2, tekstTJ, dlugoscTekstu, Vpredkosc*2);
				matrycaZgas(wysw);
				matrycaZgas(MatrycaBufor1);
				matrycaZgas(MatrycaBufor2);

		break;
		
		case 7:		//tekst gora dol
				for(uint8_t i=1; i<=liczbaSlow; i++){
						if(i != liczbaSlow){
								if(przerwijAnimacje == 1){
									matrycaZgas(wysw);
									return;
								}									
							kopiujSlowo(buforSlowa1, tekstTJ, i);
							kopiujSlowo(buforSlowa2, tekstTJ, i+1);
								if(i==1){
									napisSrodek(wysw, buforSlowa1);
									opoznienie(Vpredkosc*20);
								}
							animacjaDwaNapisyGoraDol(wysw, MatrycaBufor1, buforSlowa2, buforSlowa1, Vpredkosc*20);
						
						}
						else{
								if(przerwijAnimacje == 1){
									matrycaZgas(wysw);
									return;
								}
							kopiujSlowo(buforSlowa1, tekstTJ, liczbaSlow);
							kopiujSlowo(buforSlowa2, tekstTJ, 1);
							animacjaDwaNapisyGoraDol(wysw,MatrycaBufor1, buforSlowa2, buforSlowa1, Vpredkosc*20);
						}
					}		
					opoznienie(Vpredkosc*20);
					opoznienie(Vpredkosc*20);	
					matrycaZgas(MatrycaBufor1);
					matrycaZgas(MatrycaBufor2);
		break;
	}
}

//ToDo: kalibracja predkosci
//Podaj Predkosc od 1 do 10
uint16_t przeliczaPredkosc(uint8_t Predkosc){
	uint16_t wynikFunkcji=0;

	 switch (Predkosc) {
		 case  9:  wynikFunkcji=1;  break;		 
		 case  8:  wynikFunkcji=2;  break;
		 case  7:  wynikFunkcji=3;  break;
		 case  6:  wynikFunkcji=4;  break;
		 case  5:  wynikFunkcji=5;  break;
		 case  4:  wynikFunkcji=6;  break;
		 case  3:  wynikFunkcji=7;  break;
		 case  2:  wynikFunkcji=8;  break;
		 case  1:  wynikFunkcji=9;  break;
		 case  0:  wynikFunkcji=10;  break;	 
	  
		 default:  wynikFunkcji=8;  break;
	 }

return wynikFunkcji;
}

volatile uint8_t chwilowa=0;
uint8_t czyDaneEEPROM=0;

/*END TOMEK ZMIANA*/
int main()
{
	tekst1_wyswietlany = 1;	// zaraz domyslnie zostanie zmieniony na tekst0 po "if (zmiana_bufora) ..."


	PinsInit();

	//GICR = 1<<INT0;					// Enable INT0
//	MCUCR = 1<<ISC01 ;	// Trigger INT0 on falling edge

	ZgasZielona();
	ZgasNiebieska();
	ZgasBiala();
	czyDaneEEPROM = EEPROM_read(0x000);
	
	if (czyDaneEEPROM == 0xa3) {	// sa istotne dane w EEPROM
		for (a=0; a<BUF_LEN_MAX; a++) {
			m = EEPROM_read(a+1);
			tekst[a] = m;
			if (m == 0) {	// koniec danych
				break;
			}
		}
		tekst0[BUF_LEN_MAX-1] = 0;

		ZapalZielona();	// udalo sie odczytac
	}

// START TOMEK ZMIANA
matrycaZgas(MatrycaBufor1);
matrycaZgas(MatrycaBufor2);
// END TOMEK ZMIANA

timer1_init();
sei();		// Enable Global Interrupts

uint8_t poczatekProgramu=1;
OFF_MASTER_IN;
zmiana_bufora = 0;

	while(1) {
		/*TOMEK PROGRAM*/
			matrycaNegujF=0;	
			if (zmiana_bufora==1 || czyDaneEEPROM == 0xa3) {	
						czyDaneEEPROM = 0x00;
						poczatekProgramu=0;
						zmiana_bufora=0;
						zapisDoEEPROM(tekst, wyznaczLiczbeZnakow(tekst));
					    czyscTekst(tekstTJ);
						kopiujString(tekstTJ, tekst, 0, 199);	
						
						litera = tekstTJ[0];

						if (litera >= '1' && litera <='9') {	// jesli pierwsza litera to cyfra - odpowiada za numer animacji
							vPredkosc = litera - '0';			
							dlugoscTekstu= wyznaczLiczbeZnakow(tekstTJ);
							kopiujString(tekstTJ, tekstTJ, 1, dlugoscTekstu);
						}else{
							vPredkosc = 7;	
						}
				
						litera = tekstTJ[0];
										
						if (litera >= '1' && litera <='7') {	// jesli pierwsza litera to cyfra - odpowiada za numer animacji
							wyborAnimacji = litera - '0';			
							dlugoscTekstu= wyznaczLiczbeZnakow(tekstTJ);
							kopiujString(tekstTJ, tekstTJ, 1, dlugoscTekstu);
						}else{
							wyborAnimacji =1;			
						}
	
						dlugoscTekstu = wyznaczLiczbeZnakow(tekstTJ);
						liczbaSlow = wyznaczLiczbeSlow(tekstTJ);
						wyznaczPozycjeSlow(tekstTJ, dlugoscTekstu);
						_delay_ms(10);
						
				}	
				if(poczatekProgramu ==1){
						poczatekProgramu=0;
						dlugoscTekstu = wyznaczLiczbeZnakow(tekstTJ);
						liczbaSlow = wyznaczLiczbeSlow(tekstTJ);
						wyznaczPozycjeSlow(tekstTJ, dlugoscTekstu);
						vPredkosc = 7;
						_delay_ms(10);																									
					
				}							
		przerwijAnimacje = 0;
			//matrycaZapal(wysw);
			//_delay_ms(10000);
		wszystkieAnimacje(wyborAnimacji, przeliczaPredkosc(vPredkosc));
		
	 		
 		if(IF_MASTER_OUT_OFF){ //jesli sa dane do odebrania			
			
			cli();
			ZgasStary_ZapalKolumneRzad(pgm_read_byte(&(mapa[0])),wysw[0x00]);	// led
			matrycaZgas(wysw);
			czyscTekst(tekst);
			licznikKoncaCzasu=0;
			ON_MASTER_IN;	//daj znac, ze jestes gotowy do odebrania
			
 			while(1){
				 licznikKoncaCzasu=0;
 					while(IF_SCLK_WYSW1_ON){//zbocze narastajce sa dane!!
 						licznikKoncaCzasu++;
						if(licznikKoncaCzasu == 0){	//okolo 8ms maks oczekiwanie
							rx_jest_error = 5;
							break;
						}
					} 
 					
 					rx_nowy_bajt <<= 1;
 
 					if(IF_DANE_ON){
 						rx_nowy_bajt |= 1;
					}
 
 					rx_ktory_bit++;
 					
 					if (rx_ktory_bit == 1) {
 						if ((rx_nowy_bajt & 1) == 1) {
 							rx_jest_error = 1;

 						}
 					}	
 
 					if (rx_ktory_bit == 8) {
				
 						rx_ktory_bit = 0;
 						tekst[rx_ktory_bajt] = rx_nowy_bajt;
 						rx_ktory_bajt++;
 
 						if (rx_nowy_bajt > 127) {	// error, znak poza zakresem wyswietlania
							rx_jest_error = 2;
						}
						if (rx_nowy_bajt > 0 && rx_nowy_bajt < ' ') {	// error, znak poza zakresem wyswietlania
							rx_jest_error = 3;
						}
						if (rx_nowy_bajt == 0 && rx_ktory_bajt == 1) {	// za malo znakow (pierwszy znak to zero == koniec napisu?!)
							rx_jest_error = 4;
//														TIMSK &=~ (1<<OCIE1A);
//														ZgasStary_ZapalKolumneRzad(pgm_read_byte(&(mapa[40])),0x55);
						}
						if (rx_ktory_bajt >= BUF_LEN_MAX-1) {
							//rx_jest_error = 5;	// jest blad czy nie?!

							rx_nowy_bajt = 0;	// jednak wpisuje ze jest dobrze i koncze
							rx_ktory_bajt = BUF_LEN_MAX-1;

							tekst[rx_ktory_bajt] = rx_nowy_bajt;
						}
 					}

					if (rx_jest_error) {
						rx_jest_error = 0;
						rx_ktory_bajt = 0;
						rx_ktory_bit = 0;
						rx_nowy_bajt = 0;					
						break;
						
						
					} else {	// nie ma error, wiec to moze juz zakonczenie odbierania
						if (rx_ktory_bajt > 1) {	// jest przynajmniej jeden znak != 0
							if (rx_nowy_bajt == 0) {	// i zero na koncu

								zmiana_bufora = 1;	// udalo sie odebrac nowy bufor!

								rx_jest_error = 0;
								rx_ktory_bajt = 0;
								rx_ktory_bit = 0;
								rx_nowy_bajt = 0;

								//	PORTD &= (~(1<<PD1));	// info zwrotne

								ZapalNiebieska();	// udalo sie

								rx_startTransmisji =1;			

								break;
							}
						}
					}
 
				_delay_us(200); 
				}// end while(1) do transmisji
					
			OFF_MASTER_IN;
			sei();
		}	//end if - koniec transmisji	
			
		/*END TOMEK PROGRAM*/
		
	}//END petla programowa					
}//END MAIN


void SzpilkaNaB2()
{
	DDRB |= 0x04;
	PORTB |= 0x04;
	delay_s();
	delay_s();
	PORTB &= 0xfb;
}

void SzpilkaNaB3()
{
	DDRB |= 0x08;
	PORTB |= 0x08;
	delay_s();
	delay_s();
	PORTB &= 0xf7;
}

void SzpilkaNaB4()
{
	DDRB |= 0x10;
	PORTB |= 0x10;
	delay_s();
	delay_s();
	PORTB &= 0xef;
}

//Oryginalna animacja
void efekt1()
{
	kolumna_litery = faza_kol;
	ktora_litera = faza;

	// wypelnia 80 kolumn wyswietlacza danymi (przesuniecie o 1 kolumne)
	for (m=0; m<80; m++) {
		if (ktora_litera >= dlugosc_tekstu) {
			ktora_litera = 0;
		}

		if (kolumna_litery < 5) {
			litera = tekst[ktora_litera];
			wysw[m] = pgm_read_byte(&(tablica_font[litera-32][kolumna_litery]));
		}
		else {	// kolumna_litery == 5
			wysw[m] = 0;	// ledy zgaszone, czarna kolumna miedzy literami
		}

		kolumna_litery++;
		if (kolumna_litery > 5) {
			kolumna_litery = 0;
			ktora_litera++;
		}
	}

	faza_kol++;
	if (faza_kol > 5) {
		faza_kol = 0;
		faza++;
		if (faza >= dlugosc_tekstu) {
			faza = 0;
		}
	}

}

// odbiera od plytki z SD, RS, RF itp.
// znaki ASCII od SPACJI (32)...ABC...abc...(127) i zero na koniec
// wiec najstarszy bit (pierwszy odebrany w kazdym bajcie) musi byc zero
// (mozna zresetowac kazde odbieranie wysylajac min. osiem jedynek)
// pierwszy bajt != 0,
// ostatni bajt == 0

volatile uint8_t pomocnicza=0;


ISR(INT0_vect)	// wyzwalany gdy PD0 idzie w dol (zbocze opadajace)
{
}

void EEPROM_write(unsigned int uiAddress, unsigned char ucData)
{
	/* Wait for completion of previous write */
	while(EECR & (1<<EEWE))
	;
	/* Set up address and data registers */
	EEAR = uiAddress;
	EEDR = ucData;
	/* Write logical one to EEMWE */
	EECR |= (1<<EEMWE);
	/* Start eeprom write by setting EEWE */
	EECR |= (1<<EEWE);
}

unsigned char EEPROM_read(unsigned int uiAddress)
{
	/* Wait for completion of previous write */
	while(EECR & (1<<EEWE))	;
	/* Set up address register */
	EEAR = uiAddress;
	/* Start eeprom read by writing EERE */
	EECR |= (1<<EERE);
	/* Return data from data register */
	return EEDR;
}

//Odswiezanie matrycy co okreslony czas - Dawid
ISR (TIMER1_COMPA_vect)
{
	//ZgasWyswietlacz();
	// toggle led here
	PORTB ^= (1 << 0);

	kol++;
	if(kol>80)kol=0;
		//ledy = wysw[kol];
		//kol_popr = mapa[kol];
	ZgasStary_ZapalKolumneRzad(pgm_read_byte(&(mapa[kol])),wysw[kol]);	// ledy
}

