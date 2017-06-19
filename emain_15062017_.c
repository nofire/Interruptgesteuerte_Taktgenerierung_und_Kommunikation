#include "simuc.h"

///////////////////////////////////////////////////////////////////////////////////////
//#define V3_Aufgabe_1
#define V3_Aufgabe_2_und_3
//#define nachrichtenempfang_ueber_ports
///////////////////////////////////////////////////////////////////////////////////////

#ifdef V3_Aufgabe_1

#define BIT_POS_IST_OBEN            0
#define BIT_POS_IST_UNTEN           1
#define BIT_POS_NACH_OBEN           4
#define BIT_POS_NACH_UNTEN          5
#define BIT_POS_FAHRE_NACH_OBEN     9
#define BIT_POS_MOTOR_AN            8
#define BIT_POS_FAHRE_NACH_UNTEN	10
#define ZT_MAXW 2  //20
#define SS_MAXW 60 //60
#define MM_MAXW 2  //60
#define HH_MAXW 1  //24
#define laenge 100
#define eins 0x01
#define oi 0xFF00
#define io 0x00FF
#define d "%d:%d:%d"

typedef enum {hoch, runter, steht} STATE;


typedef struct {UCHAR zt, hh, mm, ss;} uhrzeit;

uhrzeit akt_zeit, hoch_zeit, runter_zeit;

void steuerungsfunktion (USHORT ist_oben, USHORT ist_unten, USHORT nach_oben, USHORT nach_unten, USHORT nach_oben_wegen_zeit,USHORT nach_unten_wegen_zeit,USHORT* p_fahre_nach_oben,USHORT* p_fahre_nach_unten,STATE* p_state){

    if      (nach_oben_wegen_zeit  != 0)  {*p_state = hoch;}
    else if (nach_unten_wegen_zeit != 0)  {*p_state = runter;}
    else{
        switch (*p_state) {
            case steht:     *p_fahre_nach_unten= 0; *p_fahre_nach_oben = 0;
                            if    ((ist_unten == 0) && (nach_unten == 1) && (nach_oben == 0))       {*p_state = runter;}
                            if    ((ist_oben == 0) && (nach_oben == 1))                            {*p_state = hoch;}
                            if (!((ist_unten == 0) && (nach_unten == 1) && (nach_oben == 0)) ||
                                  ((ist_oben == 0) && (nach_oben == 1))){
                                *p_state = steht;
                            }
                            break;

            case runter:    *p_fahre_nach_unten = 1; *p_fahre_nach_oben = 0;
                            if (ist_unten == 1) {*p_state = steht;}
                            if (ist_unten == 0){*p_state = runter;}
            break;

            case hoch:      *p_fahre_nach_unten = 0; *p_fahre_nach_oben = 1;
                            if (ist_oben == 1) {*p_state = steht;}
                            if (ist_oben == 0){*p_state = hoch;}

                            break;
            default:        *p_state = runter;
       }
    }
}

                                                                                // Aufgabe 1b Timer als Taktsignalgenerator

USHORT zt, hh, mm, ss = 0;                                                      // Globale Variablen zur Kommunikation mit der ISR

void timer_init(){
    USHORT buf = 0;

    buf = (1<<PS11) | (1<<TM10);                                                // TCRA1 -> Timer Control Register A // Clock Source auf intern/8 Prescaler // Timer Modus Clear-Time-On-Compare-Match
    io_out16(TCRA1,buf);

    buf = (1<<CE1);                                                             // TCRB1 -> Timer Control Register B // Counter Enable
    io_out16(TCRB1,buf);

    buf = (1<<OCIE1);                                                           // TIMR1 // Compare Match Interrupt enable
    io_out16(TIMR1,buf);

    buf = 5000;                                                                // CMPA1 //Compare Match Register auf 5000 einstellen
    io_out16(CMPA1,buf);

}

timer_isr(){                                                                    // Interrupt Service Routine
    UCHAR stringbuf[laenge], generierte_Uhrzeit[laenge];
    USHORT buf;
    zt++;
    if(zt==ZT_MAXW){zt=0; ss++;}
    if(ss==SS_MAXW){ss=0; mm++;}
    if(mm==MM_MAXW){mm=0; hh++;}
    if(hh==HH_MAXW){hh=0;}

    sprintf(stringbuf,"zt=%d Uhrzeit: %d:%d:%d\n",zt,hh,mm,ss);
    putstring(stringbuf);                                                       // Bei der Simulation des SimuC fuehrt der Aufruf von putstring() innerhalb einer ISR // ggf. zu einem Dead-Lock . Dies liegt aber nur am Simulator.

    buf = io_in16(TIFR1);                                                       // Zuruecksetzen des Interrupt-Flags
    buf = buf & ~(1 << OCIF1);
    io_out16(TIFR1, buf);

    akt_zeit.hh = hh; akt_zeit.mm = mm; akt_zeit.ss = ss;                       // Generierte Uhrzeit in akt_zeit speichern

    sprintf(generierte_Uhrzeit,"Generierte Uhrzeit: %d:%d:%d\n", akt_zeit.hh,akt_zeit.mm,akt_zeit.ss);
    putstring(generierte_Uhrzeit);

}


void emain(void* arg) {

    akt_zeit.hh = 0;
    akt_zeit.mm = 0;
    akt_zeit.ss = 0;

    hoch_zeit.hh = 0;
    hoch_zeit.mm = 1;
    hoch_zeit.ss = 0;

    runter_zeit.hh = 0;
    runter_zeit.mm = 1;
    runter_zeit.ss = 55;

    STATE cstate;

    USHORT		nach_oben, nach_unten, ist_oben, ist_unten,                      // Eingabesignale.
                fahre_nach_oben, fahre_nach_unten,                               // Ausgabesignale.
                input, output, last_output,
                nach_oben_wegen_zeit, nach_unten_wegen_zeit, buf = 0;

    UCHAR stringbuf[laenge];

    INIT_BM_WITH_REGISTER_UI;                                                    // Simulation: Bandmodell und UserInterface

    buf = io_in16(PICC);                                                         // Zur Sicherheit vor Initialisierung den Interupt des PIC generell deaktivieren
    buf = buf & ~(1 << PICE);
    io_out16(PICC, buf);

    timer_init();                                                                // Timer 1 initialisieren

    setInterruptHandler(IVN_OC1, timer_isr);                                     // ISR registrieren

   /* putstring("Bitte die aktuelle Uhrzeit im Format hh:mm:ss eingeben:\n");      // Uhrzeiterfragen ohne weitere Ueberpruefung
    getstring(stringbuf);
    sscanf(stringbuf,d,&hh, &mm, &ss);
*/
    // Interrupt des PIC jetzt zulassen
    buf = buf | (1 << PICE);
    io_out16(PICC, buf);

    io_out16(DIR1, oi);                                                         // 1.)	Hardware konfigurieren // Ausgang: Bits 15 bis 8   Eingang: Bits 7 bis 0
    cstate = runter;                                                            // 2.)  Definition des Startzustandes. Entspricht dem asynchronen Reset in VHDL.

    while (1) {                                                                 // 3.) Unendliche Schleife. Ein Schleifendurchlauf entspricht einem Zyklus des Automaten

        SYNC_SIM;

        input = io_in16(IN1);                                                   // 4.)	Einlesen der Eingabesignale einmal je Zyklus
        ist_oben = (input >> BIT_POS_IST_OBEN) & eins;                          // extrahieren von "ist_oben" (BIT_POS_IST_OBEN)
        ist_unten = (input >> BIT_POS_IST_UNTEN) & eins;                        // extrahieren von "ist_unten" (BIT_POS_IST_UNTEN)
        nach_oben = (input >> BIT_POS_NACH_OBEN) & eins;                        // extrahieren von "nach_oben" (BIT_POS_NACH_OBEN)
        nach_unten = (input >> BIT_POS_NACH_UNTEN) & eins;                      // extrahieren von "nach_unten" (BIT_POS_NACH_UNTEN)

        /*Aufgabe 1a: Ermittlung der Eingangsgrößen*/

        if((akt_zeit.hh == hoch_zeit.hh) &&
           (akt_zeit.mm == hoch_zeit.mm) &&
           (akt_zeit.ss == hoch_zeit.ss)){
              nach_oben_wegen_zeit = 1;
        }else nach_oben_wegen_zeit = 0;

        if((akt_zeit.hh == runter_zeit.hh) &&
           (akt_zeit.mm == runter_zeit.mm) &&
           (akt_zeit.ss == runter_zeit.ss)){
              nach_unten_wegen_zeit = 1;
        }else nach_unten_wegen_zeit = 0;


        steuerungsfunktion (ist_oben, ist_unten, nach_oben, nach_unten,nach_oben_wegen_zeit,nach_unten_wegen_zeit, &fahre_nach_oben, &fahre_nach_unten, &cstate);

        // 7b.) Ausgabesignale ausgeben
        output=fahre_nach_unten<<BIT_POS_FAHRE_NACH_UNTEN;
        output=output | (fahre_nach_oben<< BIT_POS_FAHRE_NACH_OBEN);


        // Nur wirklich ausgeben wenn notwendig
        // Optimierung mittels bedigter Portausgabe
        if (last_output != output) {
            output=output | (1<< BIT_POS_MOTOR_AN);   // Nur fuer Bandmodell
            io_out16(OUT1, io_in16(OUT1) & io);
            io_out16(OUT1, io_in16(OUT1) |  output);
            last_output = output;
        }

    }
}
#endif

#ifdef V3_Aufgabe_2_und_3
#include "user_conf.h"
#define LEN 300
#define COM_SIGNAl_PIN				0		// Pin ueber den der Interrupts ausgeloest wird
#define COM_DATA_IN_REGISTER		IN0		// Register ueber den das Byte eingelesen wird
#define MAX_MESSAGE_SIZE			100		// Maximale Laenge einer Nachricht
#define STARTBYTE					0x23	// Wert des Start-Bytes #
#define ENDBYTE						0    	// Wert des Ende-Bytes /0
#define READ                        0x00FF


typedef struct {UCHAR hh, mm, ss;} uhrzeit;

typedef enum {warte_auf_start_byte, warte_auf_end_byte} STATE_N;

UCHAR		nachricht[MAX_MESSAGE_SIZE];
UCHAR		flag_ready;
STATE_N		comstate=warte_auf_start_byte;
ULONG       byte_counter;


uhrzeit     akt_zeit, hoch_zeit, runter_zeit;

void do_param(UCHAR* auszuwertende_nachricht, uhrzeit* akt, uhrzeit* hoch, uhrzeit* runter){


UCHAR i;
unsigned short stunde = 0;
unsigned short minute = 0;
unsigned short sekunde = 0;


    switch (auszuwertende_nachricht[1]){

    case 'A':
            i = 0;
            stunde = 10*(auszuwertende_nachricht[i+2]-0x30);
            stunde = stunde + (auszuwertende_nachricht[i+3]-0x30);

            minute = 10* (auszuwertende_nachricht[i+4]-0x30);
            minute = minute + (auszuwertende_nachricht[i+5]-0x30);

            sekunde = 10*(auszuwertende_nachricht[i+6]-0x30);
            sekunde  = sekunde + (auszuwertende_nachricht[i+7]-0x30);

            akt->hh = stunde;
            akt->mm = minute;
            akt->ss = sekunde;

         break;

    case 'B':
            i = 0;
            stunde = 10*(auszuwertende_nachricht[i+2]-0x30);
            stunde = stunde + (auszuwertende_nachricht[i+3]-0x30);

            minute = 10* (auszuwertende_nachricht[i+4]-0x30);
            minute = minute + (auszuwertende_nachricht[i+5]-0x30);

            sekunde = 10*(auszuwertende_nachricht[i+6]-0x30);
            sekunde  = sekunde + (auszuwertende_nachricht[i+7]-0x30);

            hoch->hh = stunde;
            hoch->mm = minute;
            hoch->ss = sekunde;
        break;

    case 'C':
            i = 0;
            stunde = 10*(auszuwertende_nachricht[i+2]-0x30);
            stunde = stunde + (auszuwertende_nachricht[i+3]-0x30);

            minute = 10* (auszuwertende_nachricht[i+4]-0x30);
            minute = minute + (auszuwertende_nachricht[i+5]-0x30);

            sekunde = 10*(auszuwertende_nachricht[i+6]-0x30);
            sekunde  = sekunde + (auszuwertende_nachricht[i+7]-0x30);

            runter->hh = stunde;
            runter->mm = minute;
            runter->ss = sekunde;
        break;

    }
}

void ISR(){
    USHORT hilfe = 0;
    UCHAR buf;
    buf=(UCHAR) (io_in8(SPDR2) & READ);                                                 // Einlesen des Datenbytes in Slave
    steuerungsfunktion(buf, &byte_counter, &(nachricht[0]), &flag_ready, &comstate);    // Aufruf der Steuerungsfunktion

    hilfe = io_in8(SPSR2) & (~(1<<SPIF2));                                              // Zureucksetzen des Interrupt-Flags

    io_out8(SPSR2, hilfe);
    if(flag_ready == 1){
        do_param(nachricht, &akt_zeit, &hoch_zeit, &runter_zeit);
    }
    return;
}



void steuerungsfunktion(UCHAR byte_received, ULONG* byte_zaehler, UCHAR* empfangene_nachricht, UCHAR* ready, STATE_N* state){

    switch (*state) {

        case warte_auf_start_byte:

            if ( byte_received == STARTBYTE) {                      // Uebergang nach warte_auf_end_byte
                *byte_zaehler = 0;                                    // Etwas tun am Uebergang.
                empfangene_nachricht[*byte_zaehler]=byte_received;
                *byte_zaehler=*byte_zaehler+1;
                *state = warte_auf_end_byte;                        // Zustandswechsel

            }
            break;

        case warte_auf_end_byte:
            if (byte_received == ENDBYTE)  {                        // Uebergang nach warte_auf_start_byte

                empfangene_nachricht[*byte_zaehler]=byte_received;
                *byte_zaehler=*byte_zaehler+1;
                *ready=1;
                *state = warte_auf_start_byte;

            }

            if (*byte_zaehler == MAX_MESSAGE_SIZE-1) {
                    *state = warte_auf_start_byte;                  // Die Nachricht ist zu lang und kann daher nicht gueltig sein!
            }

            if (byte_received == STARTBYTE) {                       // Uebergang auf sich selbst nur damit etwas getan wird.
                *byte_zaehler=0;
                empfangene_nachricht[*byte_zaehler]=byte_received;
                *byte_zaehler=*byte_zaehler+1;
                *state = warte_auf_end_byte;                        // Ist ueberfluessing dient aber hoffentlich
                                                                    // dem Verstaendnis
            }

            if ((byte_received != STARTBYTE) && (byte_received != ENDBYTE)
                && (*byte_zaehler < MAX_MESSAGE_SIZE-1)) {
                empfangene_nachricht[*byte_zaehler]=byte_received;
                *byte_zaehler=*byte_zaehler+1;
                *state = warte_auf_end_byte;
            }

            break;

        default:    *state = warte_auf_start_byte;
    }
}







void init_spi1(){                                                                   // emain-sender Sender SPI-Master
    io_out8(SPCR1, 0);                                                              // SPI ist Master, clock rate = 1/4,
    io_out8(SPCR1, ((1<<SPE1) | (1<<MSTR1)));
}


void emain(void* arg){
    char string[LEN];
    USHORT buf;
    INIT_BM_WITH_REGISTER_UI;

    init_spi2();

    // Zur Sicherheit vor Initialisierung den Interrupt des PIC generell deaktivieren
        buf = io_in16(PICC);
        buf = buf &  ~(1 << PICE);
        io_out16(PICC, buf);

        // Registrieren der ISRs in der Interupt-Vektor-Tabelle
        setInterruptHandler(IVN_SPI2, ISR);

        // Interrupt des PIV jetzt zulassen.
        buf = buf | (1 << PICE);
        io_out16(PICC, buf);

    while(1) {

#ifndef USER_PROG_2
        putstring("Sie haben USER_PROG_2 nicht definiert\n");
#endif
        if (flag_ready==1){
            putstring((char*)nachricht);
            putstring("\n");

            sprintf(string, "Akt:%d:%d:%d  Hoch:%d:%d:%d  Runter:%d:%d:%d\n", akt_zeit.hh,akt_zeit.mm,akt_zeit.ss, hoch_zeit.hh,hoch_zeit.mm,hoch_zeit.ss, runter_zeit.hh,runter_zeit.mm,runter_zeit.ss);
            putstring(string);
            flag_ready=0;
        }
    }
}


//################AB HIER STEHT ALLES FUER DAS SENDER-PROGRAMM #################################################

void init_spi2(){ //emain Empfänger SPI-Slave

//SPI ist Slave
    io_out8(SPCR2, 0);
    io_out8(SPCR2, ((1<<SPE2) | (1<<SPIE2)));
}


void emain_sender(void* arg){
     UCHAR i;
     UCHAR parametriere_akt_zeit   [] = "#A000005";
     UCHAR parametriere_hoch_zeit  [] = "#B000105";
     UCHAR parametriere_runter_zeit[] = "#C000159";

     init_spi1();

     while(1) {
         i = 0;
         do{
            io_out8(SPCR1, io_in8(SPCR1) & (~(1<<notSS1)));

            for(; i<=8; i++){ // Ein ASCII-Zeichen versenden
                io_out8(SPDR1, parametriere_akt_zeit[i]);

            }
            if(i == 10){
                for(i=0; i<=8; i++){ // Ein ASCII-Zeichen versenden
                    io_out8(SPDR1, parametriere_hoch_zeit[i]);

                  }
                i++;
            }

            if(i == 11){
                for(i=0; i<=8; i++){ // Ein ASCII-Zeichen versenden
                    io_out8(SPDR1, parametriere_runter_zeit[i]);
                }

                i+=2;
             }
             ms_wait(100);

             io_out8(SPCR1,(io_in8(SPCR1) | (1<<notSS1)));

             i++;

         } while(i<127);

      }


}
#endif //V3_Aufgabe_2_und_3

