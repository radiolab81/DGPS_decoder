/*
 * main.c - DGPS-Beacon-Decoder (RTCM SC-104 auf MSK, Langwelle) fuer ATmega328P
 * ==============================================================================
 *
 * Reines avr-gcc / avr-libc, KEIN Arduino-Core, kein printf, kein float.
 * Ausgabe: Klartext auf USART0 (9600 8N1).
 *
 * WAS WIRD EMPFANGEN?
 * -------------------
 * Die maritimen DGPS-Baken (283.5 ... 325 kHz) senden Korrekturdaten nach
 * RTCM SC-104 v2 als MSK (Minimum Shift Keying) mit 50, 100 oder 200 Bit/s -
 * jeder Sender darf seine Baudrate selbst waehlen (Helgoland: 100 Bit/s,
 * Gilze-Rijen: 200 Bit/s).  Der Empfaenger (SSB/CW-Stellung) setzt das Signal
 * mit seinem BFO auf eine Audiofrequenz um.
 * MSK ist im Grunde FSK mit minimalem Hub: zwei Toene im Abstand Baud/2,
 * also  f0 = fc - Baud/4  und  f1 = fc + Baud/4  (100 Bit/s: 725/775 Hz).
 * Die Phase ist beim Tonwechsel stetig - dadurch ist das Spektrum schmal.
 *
 * Wichtig fuer den Decoder:
 *   - Das Tonvorzeichen (hoeherer/tieferer Ton) IST das Datenbit.
 *   - RTCM-Woerter sind polaritaetsfest (D29/D30-Stern-Regel + Paritaet), wir
 *     muessen also NICHT wissen, welcher Ton "1" ist.  Ein gedrehtes
 *     Seitenband oder ein anderer BFO-Versatz ist damit egal.
 *   - Es gibt keinen Restraeger.  Ein klassischer Costas-Loop (BPSK/QPSK)
 *     hat hier nichts zum Einrasten; siehe "Traegernachfuehrung" unten.
 *
 * SIGNALKETTE (alles Festkomma, ganzzahlig)
 * -----------------------------------------
 *
 *   ADC0 --4 kHz--> [DC-Abzug] --> [NCO-Mischer cos/sin] --> [CIC3 / 4]
 *                                        ^                       |
 *                                        |                    1 kHz komplex (I,Q)
 *                     Traegernachfuehrung (FLL)                  |
 *                     stellt die NCO-Frequenz nach               v
 *                                        ^          [Begrenzer: |z| = konstant]
 *                                        |                       |
 *                                        +---- Diskriminator  Im(z[n]*conj(z[n-1]))
 *                                                                |
 *                            +-----------------------+-----------+-----------+
 *                            |                       |                       |
 *                        Lane 50 Bit/s          Lane 100 Bit/s         Lane 200 Bit/s
 *                      (Nulldurchgangs-DPLL   +  Integrate&Dump  +  RTCM-Sync)
 *                                                    |
 *                                    (die Lane, die zuerst gueltige RTCM-
 *                                     Rahmen liefert, gewinnt = Baudrate erkannt)
 *                                                    |
 *                                       RTCM-Parser --> Klartext-Ausgabe
 *
 * WARUM ABTASTRATE 4 kHz UND 1 kHz KOMPLEX?
 * ------------------------------------------
 *   - Das Nutzsignal liegt bei 500 ... 1000 Hz (Toene bis ca. 850 Hz plus
 *     Flanken).  Ein einfacher analoger Tiefpass bei 1.5 kHz vor dem ADC
 *     genuegt als Anti-Alias-Filter, denn Nyquist liegt bei 2 kHz.
 *   - 4000/4 = 1000 Hz komplexe Rate: 20 / 10 / 5 Abtastwerte pro Bit bei
 *     50 / 100 / 200 Bit/s - alle ganzzahlig.  Das ist kein Zufall: so bleibt
 *     der Bittakt-Zaehler exakt und jede Baudrate faellt "glatt" auf das Raster.
 *   - Timer1 teilt 16 MHz exakt durch 4000 (OCR1A = 3999): kein Rundungsfehler.
 *
 * TRAEGERNACHFUEHRUNG ("BFO wandert")
 * ------------------------------------
 * Der Mischer arbeitet mit einem NCO (Phasenakkumulator + Sinustabelle).
 * Wandert der BFO des Empfaengers (Drift, Nachstimmen), wandert die Mitte
 * des MSK-Spektrums im Audio mit.  Wuerde der NCO stehen bleiben, rutschte
 * der Ton-Mittelpunkt aus dem Filter und die Entscheidungsschwelle (0 Hz im
 * Basisband) laege schief.  Deshalb regelt eine Frequenzschleife den NCO nach:
 *
 *   e = Mittelwert der Momentanfrequenz im Basisband
 *
 * Bei MSK sind beide Toene symmetrisch zur Mitte (+/- Baud/4).  Bei
 * ausgeglichenen Daten mittelt sich die Datenmodulation heraus, uebrig
 * bleibt genau der Versatz der Mitte.  Das ist die FSK/MSK-Entsprechung des
 * Costas-Gedankens: Modulation herausrechnen, Rest als Fehler nutzen.  Da die
 * Daten nur *fast* ausgeglichen sind, ist die Schleife traege (Zeitkonstante
 * ca. 2 s im eingerasteten Zustand, schneller beim Suchen).
 * Weil der Diskriminator nur die Frequenz (nicht die Phase) auswertet, ist
 * keine Phasenverriegelung noetig; die Detektion ist nichtkoharent.
 * Fangbereich: +/- NCO_RANGE_HZ um AUDIO_CENTER_HZ (Vorgabe +/-100 Hz).
 *
 * BAUDRATENERKENNUNG
 * ------------------
 * Drei Bit-Lanes (50/100/200) laufen parallel auf demselben Diskriminator.
 * Jede hat einen eigenen Bittakt und sucht RTCM-Rahmen.  Ein Rahmen ist erst
 * "echt", wenn Praeambel 0x66 UND die 6-Bit-Paritaet von Wort 1 UND Wort 2
 * stimmen (zufaellig: 1 : 2^26).  Die Lane, die das schafft, wird aktiv;
 * die anderen werden ignoriert.  Bleiben ~8 s gueltige Rahmen aus, wird
 * wieder in allen drei Lanes gesucht (Sender hat Baudrate gewechselt / weg).
 *
 * BITTAKT (Nulldurchgangs-DPLL)
 * -----------------------------
 * Bei MSK wechselt die Momentanfrequenz nur an Bitgrenzen.  Der Diskriminator-
 * Ausgang kreuzt dabei die Nulllinie.  Jede Lane fuehrt einen Phasenakkumulator
 * (Q16 Bit pro Abtastwert = Baud/1000); jeder Nulldurchgang (auf Bruchteile
 * eines Abtastwerts interpoliert) zieht die Phase Richtung "Bitgrenze".
 * Zwischen Nulldurchgaengen (lange gleiche Bits) laeuft der Takt frei weiter.
 *
 * RTCM-SC-104 v2 IN KUERZE
 * ------------------------
 *   Wort = 30 Bit: 24 Datenbits + 6 Paritaetsbits (wie GPS-Navigationsdaten).
 *   Ist D30 des Vorwortes 1, sind die 24 Datenbits des aktuellen Wortes
 *   invertiert uebertragen.
 *   Wort 1: Praeambel 01100110 | Typ (6) | Stations-ID (10) | Parity
 *   Wort 2: Z-Zaehler (13, je 0.6 s) | Folgenr (3) | Laenge in Woertern (5)
 *           | Stationszustand (3) | Parity
 *   dann "Laenge" Datenwoerter.
 *   Typ 1/9  : GPS-Korrekturen,  Typ 31/34 : GLONASS-Korrekturen
 *              (je Satellit 40 Bit: Skala 1, UDRE 2, SatID 5, PRC 16, RRC 8, IOD 8)
 *   Typ 3    : Position der Referenzstation (ECEF X,Y,Z je 32 Bit, 1 cm)
 *   Typ 16   : Textnachricht (8-Bit-ASCII)
 *
 * SPEICHER (ATmega328P: 2 KB RAM)
 * -------------------------------
 * Die schwere Rechnung laeuft in der Hauptschleife mit 1 kHz; der Interrupt
 * macht nur das Noetigste (Mischer + CIC) und legt (I,Q) in einen Ringpuffer.
 * Ausgaben laufen ueber einen interruptgetriebenen UART-Ringpuffer und werden
 * zeilenweise abgesetzt, so dass die DSP-Kette nie auf die serielle
 * Schnittstelle warten muss.
 *
 * PC-TEST
 * -------
 * Mit -DHOST_SIM laesst sich dieselbe Datei auf dem PC uebersetzen (siehe
 * host_sim.c / "make test"): dann werden die Abtastwerte aus einer Datei
 * gelesen statt vom ADC.  So wurde die Kette gegen die Mitschnitte in der Entwicklungsphase vor
 * dem ersten Live-Einsatz geprueft.
 */

#ifdef HOST_SIM
/* ---- PC-Testumgebung: AVR-Spezialitaeten wegdefinieren ---------------- */
#include <stdint.h>
#include <stdio.h>
#define PROGMEM
#define PSTR(s)            (s)
#define pgm_read_byte(p)   (*(const uint8_t *)(p))
#define pgm_read_dword(p)  (*(const uint32_t *)(p))
#define cli()              do { } while (0)
#define sei()              do { } while (0)
#else
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdint.h>
#endif

/* ------------------------------------------------------------------ */
/* Konfiguration                                                      */
/* ------------------------------------------------------------------ */

#define UART_BAUD        9600UL
#define AUDIO_CENTER_HZ  750      /* Mitte des MSK-Signals im Audio (BFO!)   */
#define NCO_RANGE_HZ     100      /* Nachfuehrbereich +/- um die Mitte       */
#define STATUS_PERIOD_S  30       /* Statuszeile alle n s (0 = aus)          */
#define LOSS_TIMEOUT_S   8        /* so lange ohne gueltigen Rahmen = "weg"  */
#define RECENTER_S       40       /* ohne Sync: NCO wieder auf Mitte setzen  */

/* Feste Raster - siehe Kommentar oben, nicht ohne Nachdenken aendern */
#define FS_HZ            4000UL   /* ADC-Rate (Timer1)                       */
#define DECIM            4        /* CIC-Dezimation -> 1 kHz komplex         */
#define FB_HZ            1000UL   /* komplexe Rate nach der Dezimation       */

/* NCO: 16-Bit-Phasenakkumulator, 1 LSB = 4000/65536 Hz = 0.061 Hz */
#define HZ_TO_INC(h)     ((uint16_t)(((uint32_t)(h) * 65536UL + FS_HZ / 2) / FS_HZ))
#define NCO_INC_NOM      HZ_TO_INC(AUDIO_CENTER_HZ)
#define NCO_INC_RANGE    HZ_TO_INC(NCO_RANGE_HZ)

#define NLANES           3
#define ZAMP             4096     /* Betrag des Begrenzer-Ausgangs           */
#define FLL_BLOCK        256      /* Abtastwerte je Frequenzschleifen-Schritt */
#define TXBUF            256      /* UART-Sendepuffer (Zweierpotenz!)        */
#define TXLINE_MAX       120      /* laengste Zeile (Platzbedarf im Puffer)  */
#define RXRING           32       /* Ringpuffer ISR -> Hauptschleife         */

/* ------------------------------------------------------------------ */
/* UART-Ausgabe (interruptgetrieben, ohne stdio)                       */
/* ------------------------------------------------------------------ */

#ifndef HOST_SIM
static uint8_t txbuf[TXBUF];
#endif
static volatile uint8_t tx_head, tx_tail;

#ifndef HOST_SIM
static void uart_init(void)
{
    uint16_t ubrr = (uint16_t)((F_CPU / (16UL * UART_BAUD)) - 1);
    UBRR0H = (uint8_t)(ubrr >> 8);
    UBRR0L = (uint8_t)ubrr;
    UCSR0B = (1 << TXEN0);                       /* nur Sender               */
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);      /* 8N1                      */
}

/* Sendeinterrupt: solange Daten im Ring liegen, Byte fuer Byte nachladen. */
ISR(USART_UDRE_vect)
{
    if (tx_tail == tx_head) {
        UCSR0B &= (uint8_t)~(1 << UDRIE0);       /* Ring leer: IRQ aus       */
        return;
    }
    UDR0 = txbuf[tx_tail];
    tx_tail = (uint8_t)((tx_tail + 1) & (TXBUF - 1));
}
#endif

/* Freier Platz im Sendepuffer (fuer die Zeilenlogik). */
static uint8_t tx_free(void)
{
#ifdef HOST_SIM
    return 255;
#else
    return (uint8_t)((tx_tail - tx_head - 1) & (TXBUF - 1));
#endif
}

/* Ein Zeichen in den Sendepuffer.  Voller Puffer: Zeichen verwerfen -
 * die Zeilenlogik prueft vorher tx_free(), das passiert also nie. */
static void tx_put(char c)
{
#ifdef HOST_SIM
    putchar(c);
#else
    uint8_t n = (uint8_t)((tx_head + 1) & (TXBUF - 1));
    if (n == tx_tail) {
        return;
    }
    txbuf[tx_head] = (uint8_t)c;
    tx_head = n;
    UCSR0B |= (1 << UDRIE0);
#endif
}

/* String aus dem FLASH (immer als puts_P(PSTR("...")) aufrufen,
 * sonst landen Literale im knappen RAM). */
static void puts_P(const char *s)
{
    char c;
    while ((c = (char)pgm_read_byte(s++)) != '\0') {
        tx_put(c);
    }
}

static void put_udec(uint32_t v)
{
    char buf[11];
    uint8_t i = 10;
    buf[i] = '\0';
    do {
        buf[--i] = (char)('0' + (v % 10));
        v /= 10;
    } while (v != 0);
    while (buf[i]) {
        tx_put(buf[i++]);
    }
}

static void put_2d(uint8_t v)                    /* zweistellig, fuehrende 0 */
{
    tx_put((char)('0' + v / 10));
    tx_put((char)('0' + v % 10));
}

/* Festkomma: v ist der Wert * 10^dec, z.B. put_fixed(-266, 2) -> "-2.66" */
static void put_fixed(int32_t v, uint8_t dec)
{
    uint32_t p = 1, u, f;
    uint8_t i;
    if (v < 0) {
        tx_put('-');
        u = (uint32_t)(-v);
    } else {
        u = (uint32_t)v;
    }
    for (i = 0; i < dec; i++) {
        p *= 10;
    }
    put_udec(u / p);
    tx_put('.');
    f = u % p;
    for (i = 0; i < dec; i++) {                  /* Nachkommastellen mit 0   */
        p /= 10;
        tx_put((char)('0' + (f / p) % 10));
    }
}

/* ------------------------------------------------------------------ */
/* Sinustabelle (1 Periode, 256 Werte, Amplitude 127)                  */
/* cos(x) = sin(x + 64).  Die Phasenquantisierung von 8 Bit erzeugt    */
/* Nebenlinien ca. 48 dB unter dem Nutzton - im Basisband harmlos.     */
/* ------------------------------------------------------------------ */

static const int8_t sin_tab[256] PROGMEM = {
0,    3,    6,    9,   12,   16,   19,   22,   25,   28,   31,   34,   37,   40,   43,   46,
      49,   51,   54,   57,   60,   63,   65,   68,   71,   73,   76,   78,   81,   83,   85,   88,
      90,   92,   94,   96,   98,  100,  102,  104,  106,  107,  109,  111,  112,  113,  115,  116,
     117,  118,  120,  121,  122,  122,  123,  124,  125,  125,  126,  126,  126,  127,  127,  127,
     127,  127,  127,  127,  126,  126,  126,  125,  125,  124,  123,  122,  122,  121,  120,  118,
     117,  116,  115,  113,  112,  111,  109,  107,  106,  104,  102,  100,   98,   96,   94,   92,
      90,   88,   85,   83,   81,   78,   76,   73,   71,   68,   65,   63,   60,   57,   54,   51,
      49,   46,   43,   40,   37,   34,   31,   28,   25,   22,   19,   16,   12,    9,    6,    3,
       0,   -3,   -6,   -9,  -12,  -16,  -19,  -22,  -25,  -28,  -31,  -34,  -37,  -40,  -43,  -46,
     -49,  -51,  -54,  -57,  -60,  -63,  -65,  -68,  -71,  -73,  -76,  -78,  -81,  -83,  -85,  -88,
     -90,  -92,  -94,  -96,  -98, -100, -102, -104, -106, -107, -109, -111, -112, -113, -115, -116,
    -117, -118, -120, -121, -122, -122, -123, -124, -125, -125, -126, -126, -126, -127, -127, -127,
    -127, -127, -127, -127, -126, -126, -126, -125, -125, -124, -123, -122, -122, -121, -120, -118,
    -117, -116, -115, -113, -112, -111, -109, -107, -106, -104, -102, -100,  -98,  -96,  -94,  -92,
     -90,  -88,  -85,  -83,  -81,  -78,  -76,  -73,  -71,  -68,  -65,  -63,  -60,  -57,  -54,  -51,
     -49,  -46,  -43,  -40,  -37,  -34,  -31,  -28,  -25,  -22,  -19,  -16,  -12,   -9,   -6,   -3,};

/* ------------------------------------------------------------------ */
/* Front-End im Interrupt: DC-Abzug, NCO-Mischer, CIC3-Dezimation      */
/* ------------------------------------------------------------------ */

typedef struct { int16_t i, q; } cplx16_t;

static cplx16_t rx_ring[RXRING];
static volatile uint8_t rx_head, rx_tail;

static uint16_t nco_phase;                       /* 0..65535 = 0..360 Grad   */
static volatile uint16_t nco_inc = NCO_INC_NOM;  /* Phasenschritt je Abtast. */

/* CIC-Filter (Cascaded Integrator-Comb) 3. Ordnung, Dezimation 4:
 * drei Integratoren mit 4 kHz, dann jeder 4. Wert durch drei Kammfilter
 * (Differenzen).  Vorteile: nur Additionen, keine Koeffizienten.  Die
 * Nullstellen liegen bei 1000/2000/... Hz - genau dort, wo beim Herunter-
 * tasten auf 1 kHz sonst Stoerungen ins Basisband faltet.  Die Verstaerkung
 * ist 4^3 = 64.  Unsigned-Arithmetik: Ueberlaeufe sind hier gewollt und
 * heben sich in den Kammfiltern wieder auf (Modulo-2^32-Trick). */
static uint32_t ci1, ci2, ci3, cd1, cd2, cd3;    /* I-Kanal */
static uint32_t cq1, cq2, cq3, ce1, ce2, ce3;    /* Q-Kanal */
static uint8_t  dec_cnt;
static int32_t  dc_acc;                          /* DC-Schaetzer             */

static void dsp_sample(int16_t x)                /* wird mit 4 kHz gerufen   */
{
    /* 1) Gleichanteil (ADC-Bias) mit sehr langsamem Tiefpass (~0.6 Hz)
     *    schaetzen und abziehen.  Sonst liefe ein DC-Rest als Ton bei
     *    -f_NCO durch den Mischer. */
    dc_acc += x - (dc_acc >> 10);
    x -= (int16_t)(dc_acc >> 10);

    /* 2) Komplexer Mischer:  z = x * e^(-j*phi)  ->  I = x*cos, Q = -x*sin */
    uint8_t ph = (uint8_t)(nco_phase >> 8);
    int8_t  c  = (int8_t)pgm_read_byte(&sin_tab[(uint8_t)(ph + 64)]);
    int8_t  s  = (int8_t)pgm_read_byte(&sin_tab[ph]);
    nco_phase += nco_inc;
    int32_t mi =  (int32_t)x * c;
    int32_t mq = -(int32_t)x * s;

    /* 3) Integratoren (laufen mit jeder Abtastung) */
    ci1 += (uint32_t)mi;  ci2 += ci1;  ci3 += ci2;
    cq1 += (uint32_t)mq;  cq2 += cq1;  cq3 += cq2;

    if (++dec_cnt < DECIM) {
        return;
    }
    dec_cnt = 0;

    /* 4) Kammfilter (nur jeden 4. Wert) */
    uint32_t t, u;
    t = ci3 - cd1;  cd1 = ci3;  u = t - cd2;  cd2 = t;  t = u - cd3;  cd3 = u;
    int32_t oi = (int32_t)t;
    t = cq3 - ce1;  ce1 = cq3;  u = t - ce2;  ce2 = t;  t = u - ce3;  ce3 = u;
    int32_t oq = (int32_t)t;

    /* Verstaerkung 64 * (127/2) -> mit >>7 bleibt |I|,|Q| < 32768 bei
     * voller ADC-Aussteuerung. */
    uint8_t nh = (uint8_t)((rx_head + 1) & (RXRING - 1));
    if (nh != rx_tail) {                         /* Ring voll: Wert verlieren */
        rx_ring[rx_head].i = (int16_t)(oi >> 7);
        rx_ring[rx_head].q = (int16_t)(oq >> 7);
        rx_head = nh;
    }
}

#ifndef HOST_SIM
/* Timer1 (CTC, 4000 Hz) startet jede ADC-Wandlung, der ADC-Interrupt holt
 * den Wert.  (bewusst keine Auto-Trigger-Magie.) */
ISR(TIMER1_COMPA_vect)
{
    ADCSRA |= (1 << ADSC);
}

ISR(ADC_vect)
{
    uint16_t raw = ADC;
    dsp_sample((int16_t)raw - 512);              /* Bias Vcc/2 -> 0          */
}

static void adc_init(void)
{
    ADMUX  = (1 << REFS0);                       /* AVcc-Referenz, Kanal ADC0 */
    ADCSRA = (1 << ADEN) | (1 << ADIE)
           | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);   /* 125 kHz Takt   */
}

static void timer1_init(void)
{
    TCCR1A = 0;
    TCCR1B = (1 << WGM12) | (1 << CS10);         /* CTC, Vorteiler 1         */
    OCR1A  = (uint16_t)(F_CPU / FS_HZ - 1);      /* 3999 -> exakt 4000 Hz    */
    TIMSK1 = (1 << OCIE1A);
}
#endif

/* ------------------------------------------------------------------ */
/* Hilfsroutinen der Hauptschleife                                     */
/* ------------------------------------------------------------------ */

static uint8_t rx_pop(cplx16_t *out)
{
    if (rx_tail == rx_head) {
        return 0;
    }
    *out = rx_ring[rx_tail];
    rx_tail = (uint8_t)((rx_tail + 1) & (RXRING - 1));
    return 1;
}

static void nco_set_inc(uint16_t v)              /* 16 Bit sind nicht atomar */
{
    cli();
    nco_inc = v;
    sei();
}

/* ------------------------------------------------------------------ */
/* RTCM-Woerter: Paritaet                                               */
/* ------------------------------------------------------------------ */

/* Hamming-Masken der 6 Paritaetsbits (identisch zur GPS-Schnittstelle).
 * Das 32-Bit-Wort enthaelt oben D29* und D30* des Vorwortes, darunter die
 * 24 Daten- und 6 Paritaetsbits des aktuellen Wortes. */
static const uint32_t ham[6] PROGMEM = {
    0xBB1F3480UL, 0x5D8F9A40UL, 0xAEC7CD00UL,
    0x5763E680UL, 0x6BB1F340UL, 0x8B7A89C0UL
};

static uint8_t par32(uint32_t x)                 /* XOR aller 32 Bit         */
{
    x ^= x >> 16;
    x ^= x >> 8;
    x ^= x >> 4;
    return (uint8_t)((0x6996u >> (x & 0xF)) & 1);
}

/* Prueft ein Wort; entzieht dabei die D30*-Inversion.  Rueckgabe 1 = ok.
 * *w enthaelt danach die korrigierten Daten in Bit 29..6. */
static uint8_t word_check(uint32_t *w)
{
    uint32_t x = *w;
    uint8_t i, p = 0;
    if (x & 0x40000000UL) {
        x ^= 0x3FFFFFC0UL;                       /* 24 Datenbits zurueckdrehen */
    }
    *w = x;
    for (i = 0; i < 6; i++) {
        p = (uint8_t)((p << 1) | par32(x & pgm_read_dword(&ham[i])));
    }
    return p == (uint8_t)(x & 0x3F);
}

/* ------------------------------------------------------------------ */
/* Bit-Lanes und RTCM-Rahmenempfang                                     */
/* ------------------------------------------------------------------ */

enum { ST_HUNT = 0, ST_W2, ST_BODY };            /* Rahmenzustand einer Lane */

typedef struct {
    int32_t  phase;      /* Bittakt-Phase, Q16 Bit (65536 = 1 Bit)          */
    int32_t  acc;        /* Integrator: Summe der Diskriminatorwerte im Bit */
    uint32_t reg;        /* 32-Bit-Schieberegister der Bits                 */
    uint16_t inc;        /* Phasenschritt je Abtastwert = Baud/1000 * 65536 */
    uint16_t baud;
    uint8_t  st;         /* ST_HUNT / ST_W2 / ST_BODY                       */
    uint8_t  nb;         /* Bits im aktuellen Wort                          */
} lane_t;

static lane_t lane[NLANES];
static int8_t active = -1;                       /* gewonnene Lane oder -1  */

/* Kopf und Nutzlast des gerade empfangenen Rahmens */
static uint8_t  fr_type, fr_seq, fr_len, fr_health, fr_words;
static uint16_t fr_sid, fr_z;
static uint8_t  msg[31 * 3];                     /* max. 31 Datenwoerter     */

/* Zaehler / Zeitbasis */
static uint16_t frames_ok, frames_bad, frames_skipped;
static uint16_t uptime_s, last_ok_s, ms_cnt;

/* Ausgabeauftraege (siehe Printer weiter unten) */
#define EV_LOCK  0x01
#define EV_LOSS  0x02
#define EV_STAT  0x04
static volatile uint8_t ev_flags;
static uint8_t  pj_active, pj_line;              /* Rahmen-Ausgabeauftrag    */

static uint32_t msg_bits(uint16_t pos, uint8_t n)  /* n Bit ab Bitposition pos */
{
    uint32_t v = 0;
    while (n--) {
        v = (v << 1) | ((msg[pos >> 3] >> (7 - (pos & 7))) & 1);
        pos++;
    }
    return v;
}

static void lanes_to_hunt(void)
{
    uint8_t i;
    for (i = 0; i < NLANES; i++) {
        lane[i].st = ST_HUNT;
    }
}

/* Ein Bit einer Lane in den RTCM-Empfaenger schieben. */
static void lane_bit(uint8_t li, uint8_t bit)
{
    lane_t *L = &lane[li];
    uint32_t w;

    L->reg = (L->reg << 1) | bit;

    if (L->st == ST_HUNT) {
        /* Praeambel 01100110 in den oberen 8 Datenbits; bei gesetztem D30*
         * (Bit 30) waeren sie invertiert uebertragen. */
        uint8_t pre = (uint8_t)(L->reg >> 22);
        if (L->reg & 0x40000000UL) {
            pre ^= 0xFF;
        }
        if (pre != 0x66) {
            return;
        }
        w = L->reg;
        if (!word_check(&w)) {
            return;
        }
        fr_type = (uint8_t)((w >> 16) & 0x3F);
        fr_sid  = (uint16_t)((w >> 6) & 0x3FF);
        L->st = ST_W2;
        L->nb = 0;
        return;
    }

    if (++L->nb < 30) {
        return;                                  /* Wort noch nicht komplett */
    }
    L->nb = 0;
    w = L->reg;

    if (!word_check(&w)) {                       /* Paritaetsfehler          */
        if (active == (int8_t)li) {
            frames_bad++;
        }
        L->st = ST_HUNT;
        return;
    }

    if (L->st == ST_W2) {                        /* Wort 2 = Kopf Teil 2     */
        if (active >= 0 && active != (int8_t)li) {
            L->st = ST_HUNT;                     /* andere Lane ist schon aktiv */
            return;
        }
        if (active < 0) {                        /* erste Lane mit 2 guten Woertern */
            active = (int8_t)li;
            ev_flags |= EV_LOCK;
        }
        fr_z      = (uint16_t)((w >> 17) & 0x1FFF);
        fr_seq    = (uint8_t)((w >> 14) & 7);
        fr_len    = (uint8_t)((w >> 9) & 0x1F);
        fr_health = (uint8_t)((w >> 6) & 7);
        fr_words  = 0;
        last_ok_s = uptime_s;
        L->st = ST_BODY;
    } else {                                     /* Datenwort                */
        if (!pj_active) {                        /* nur ablegen, wenn Drucker frei */
            uint8_t k = (uint8_t)(fr_words * 3);
            msg[k]     = (uint8_t)(w >> 22);
            msg[k + 1] = (uint8_t)(w >> 14);
            msg[k + 2] = (uint8_t)(w >> 6);
        }
        fr_words++;
    }

    if (L->st == ST_BODY && fr_words >= fr_len) {   /* Rahmen komplett        */
        frames_ok++;
        last_ok_s = uptime_s;
        if (pj_active) {
            frames_skipped++;                    /* Drucker noch beschaeftigt */
        } else {
            pj_active = 1;
            pj_line = 0;
        }
        L->st = ST_HUNT;
    }
}

/* ------------------------------------------------------------------ */
/* Klartext-Ausgabe (eine Zeile je Aufruf, blockiert nie)               */
/* ------------------------------------------------------------------ */

static void put_typename(uint8_t t)
{
    switch (t) {
    case 1:  puts_P(PSTR("DGPS-Korrekturen"));            break;
    case 2:  puts_P(PSTR("Delta-Korrekturen"));           break;
    case 3:  puts_P(PSTR("Referenzstation-Position"));    break;
    case 5:  puts_P(PSTR("Satelliten-Zustand"));          break;
    case 6:  puts_P(PSTR("Fuellrahmen"));                 break;
    case 7:  puts_P(PSTR("Baken-Almanach"));              break;
    case 9:  puts_P(PSTR("GPS-Teilkorrekturen"));         break;
    case 16: puts_P(PSTR("Textnachricht"));               break;
    case 31: puts_P(PSTR("GLONASS-Korrekturen"));         break;
    case 34: puts_P(PSTR("GLONASS-Teilkorrekturen"));     break;
    default: puts_P(PSTR("(Typ nicht dekodiert)"));       break;
    }
}

static void put_health(uint8_t h)
{
    switch (h) {
    case 0:  puts_P(PSTR("ok"));                          break;
    case 6:  puts_P(PSTR("nicht ueberwacht"));            break;
    case 7:  puts_P(PSTR("AUSSER BETRIEB"));              break;
    default: puts_P(PSTR("UDRE-Skala ")); put_udec(h);    break;
    }
}

/* Gibt Zeile "line" des aktuellen Rahmens aus, Rueckgabe 1 = es folgen mehr. */
static uint8_t print_line(uint8_t line)
{
    uint8_t t = fr_type;

    if (line == 0) {                             /* Kopfzeile: immer         */
        uint32_t t10 = (uint32_t)fr_z * 6;       /* Z-Zaehler in 0.1 s       */
        tx_put('[');  put_udec(fr_sid);  puts_P(PSTR("] Typ "));  put_udec(t);
        puts_P(PSTR(" "));  put_typename(t);
        puts_P(PSTR(", Z="));
        put_2d((uint8_t)(t10 / 600));  tx_put(':');
        put_2d((uint8_t)((t10 % 600) / 10));  tx_put('.');
        put_udec(t10 % 10);
        puts_P(PSTR(", Nr "));  put_udec(fr_seq);
        puts_P(PSTR(", "));  put_udec(fr_len);  puts_P(PSTR(" Wo, Stat: "));
        put_health(fr_health);
        puts_P(PSTR("\r\n"));
        return 1;
    }

    if (t == 1 || t == 9 || t == 31 || t == 34) {
        /* Je Satellit 40 Bit am Stueck (Woerter nahtlos aneinandergereiht):
         * Skala(1) UDRE(2) SatID(5) PRC(16, signed) RRC(8, signed) IOD(8) */
        uint8_t nsat = (uint8_t)((fr_len * 24U) / 40U);
        uint8_t k = (uint8_t)(line - 1);
        if (k >= nsat || k >= fr_words * 24U / 40U) {
            return 0;
        }
        uint16_t p = (uint16_t)k * 40;
        uint8_t  sc   = (uint8_t)msg_bits(p, 1);
        uint8_t  udre = (uint8_t)msg_bits(p + 1, 2);
        uint8_t  id   = (uint8_t)msg_bits(p + 3, 5);
        int16_t  prc  = (int16_t)msg_bits(p + 8, 16);
        int8_t   rrc  = (int8_t)msg_bits(p + 24, 8);
        uint8_t  iod  = (uint8_t)msg_bits(p + 32, 8);
        if (id == 0) {
            id = 32;                             /* ID 0 steht fuer PRN 32   */
        }
        uint8_t mul = sc ? 32 : 2;               /* PRC 0.02 m / 0.32 m; RRC 0.002 / 0.032 m/s */
        puts_P((t == 1 || t == 9) ? PSTR("  G") : PSTR("  R"));
        put_2d(id);
        puts_P(PSTR("  PRC "));  put_fixed((int32_t)prc * mul, 2);
        puts_P(PSTR(" m  RRC "));  put_fixed((int32_t)rrc * mul, 3);
        puts_P(PSTR(" m/s  IOD "));  put_udec(iod);
        puts_P(PSTR("  UDRE "));
        puts_P(udre == 0 ? PSTR("<=1 m") : udre == 1 ? PSTR("<=4 m")
             : udre == 2 ? PSTR("<=8 m") : PSTR(">8 m (nicht nutzen)"));
        puts_P(PSTR("\r\n"));
        return (uint8_t)(k + 1 < nsat);
    }

    if (t == 3 && fr_len >= 4) {                 /* ECEF-Position, 3 x 32 Bit, 1 cm */
        puts_P(PSTR("  Referenzposition ECEF  X "));
        put_fixed((int32_t)msg_bits(0, 32), 2);
        puts_P(PSTR("  Y "));  put_fixed((int32_t)msg_bits(32, 32), 2);
        puts_P(PSTR("  Z "));  put_fixed((int32_t)msg_bits(64, 32), 2);
        puts_P(PSTR(" m\r\n"));
        return 0;
    }

    if (t == 16) {                               /* Text: 3 Zeichen je Wort  */
        uint8_t n = (uint8_t)(fr_words * 3), i;
        puts_P(PSTR("  Text: "));
        for (i = 0; i < n && i < 90; i++) {
            char c = (char)msg[i];
            if (c == 0) {
                break;                           /* 0 beendet den Text       */
            }
            tx_put((c >= 32 && c < 127) ? c : '.');
        }
        puts_P(PSTR("\r\n"));
        return 0;
    }

    return 0;                                    /* sonstige Typen: nur Kopf */
}

/* Ereigniszeilen (Sync, Verlust, Status) */
static void put_freq_tenth(void)                 /* aktuelle NCO-Frequenz    */
{
    uint16_t inc;
    cli();
    inc = nco_inc;
    sei();
    put_fixed(((int32_t)inc * 625L) / 1024L, 1); /* 0.061035 Hz je LSB       */
    puts_P(PSTR(" Hz"));
}

static uint16_t level_avg;                       /* mittlerer Betrag vor Begrenzer */

static void print_event(uint8_t ev)
{
    if (ev & EV_LOCK) {
        puts_P(PSTR("*** SYNC: "));
        put_udec(active >= 0 ? lane[(uint8_t)active].baud : 0);
        puts_P(PSTR(" Bit/s, NF-Mitte "));
        put_freq_tenth();
        puts_P(PSTR(" ***\r\n"));
    } else if (ev & EV_LOSS) {
        puts_P(PSTR("*** Sync verloren - suche 50/100/200 Bit/s ***\r\n"));
    } else {
        puts_P(PSTR("--- Status: NF-Mitte "));
        put_freq_tenth();
        puts_P(PSTR(", Pegel ~"));
        put_udec(level_avg / 32);                /* ca. ADC-Counts (Spitze)  */
        puts_P(PSTR(", Rahmen ok "));  put_udec(frames_ok);
        puts_P(PSTR(", fehlerhaft "));  put_udec(frames_bad);
        puts_P(PSTR(", uebersprungen "));  put_udec(frames_skipped);
        if (active >= 0) { puts_P(PSTR(" ---\r\n")); } else { puts_P(PSTR(", KEIN SYNC ---\r\n")); }
    }
}

/* Wird staendig aus der Hauptschleife gerufen: gibt hoechstens eine Zeile
 * aus, und nur wenn der Sendepuffer sie ganz aufnehmen kann. */
static void printer_pump(void)
{
    if (tx_free() < TXLINE_MAX) {
        return;
    }
    if (ev_flags) {
        uint8_t e;
        cli();
        e = ev_flags & (uint8_t)(-(int8_t)ev_flags);   /* niedrigstes Bit  */
        ev_flags &= (uint8_t)~e;
        sei();
        print_event(e);
        return;
    }
    if (pj_active) {
        if (!print_line(pj_line++)) {
            pj_active = 0;
        }
    }
}

/* ------------------------------------------------------------------ */
/* 1-kHz-Verarbeitung: Begrenzer, Diskriminator, FLL, Bittakt          */
/* ------------------------------------------------------------------ */

static int16_t  zi_p, zq_p;                      /* letzter Begrenzer-Ausgang */
static int32_t  di_p, ds_p, abs_avg;
static int32_t  fll_acc;
static uint16_t fll_n;

static void fll_update(void)
{
    /* Mittelwert der Momentanfrequenz -> Frequenzfehler des NCO.
     *   e[rad/Abtastwert] = fll_acc / (FLL_BLOCK * 2^18)
     *   Delta f [Hz]       = e * 1000 / 2pi
     *   Delta NCO-LSB      = Delta f / 0.061 Hz  ~  fll_acc / (FLL_BLOCK * 100)
     * Schleifenverstaerkung 1/2 (Suchen) bzw. 1/8 (eingerastet). */
    uint8_t sh = (active >= 0) ? 3 : 1;
    int32_t d = fll_acc / ((int32_t)FLL_BLOCK * 100L << sh);
    int32_t v = (int32_t)nco_inc + d;
    if (v > (int32_t)NCO_INC_NOM + NCO_INC_RANGE) v = (int32_t)NCO_INC_NOM + NCO_INC_RANGE;
    if (v < (int32_t)NCO_INC_NOM - NCO_INC_RANGE) v = (int32_t)NCO_INC_NOM - NCO_INC_RANGE;
    nco_set_inc((uint16_t)v);
    fll_acc = 0;
    fll_n = 0;
}

/* Einmal je Sekunde: Zeitueberwachung. */
static void second_tick(void)
{
    uptime_s++;
    if (active >= 0 && (uint16_t)(uptime_s - last_ok_s) > LOSS_TIMEOUT_S) {
        active = -1;
        lanes_to_hunt();
        ev_flags |= EV_LOSS;
    }
    if (active < 0 && (uint16_t)(uptime_s - last_ok_s) > RECENTER_S) {
        nco_set_inc(NCO_INC_NOM);                /* wieder in der Mitte suchen */
        last_ok_s = uptime_s;
    }
#if STATUS_PERIOD_S
    if ((uptime_s % STATUS_PERIOD_S) == 0) {
        ev_flags |= EV_STAT;
    }
#endif
}

static void process_sample(int16_t I, int16_t Q)
{
    uint8_t i;

    /* --- 0) Schmalband-Nachfilter: zwei Einpol-Tiefpaesse (je alpha = 1/2,
     *        fc ~ 110 Hz bei 1 kHz) auf I und Q.  Das CIC laesst noch ~500 Hz
     *        Rauschen durch; der Diskriminator verstaerkt Rauschen stark.
     *        Gemessen: ohne diese Stufe ging bei 100 Bit/s (Helgoland) ein
     *        Grossteil der Rahmen verloren, mit ihr 100 von 100 gueltig.
     *        Zu schmal (alpha = 1/4) macht dagegen 200 Bit/s kaputt. ------ */
    static int32_t a1i, a1q, a2i, a2q;
    a1i += ((int32_t)I - a1i) >> 1;   a1q += ((int32_t)Q - a1q) >> 1;
    a2i += (a1i - a2i) >> 1;          a2q += (a1q - a2q) >> 1;
    I = (int16_t)a2i;                 Q = (int16_t)a2q;

    /* --- 1) Begrenzer (wie im FM-Empfaenger): Betrag auf ZAMP normieren.
     *        |z| ~ max + 3/8*min (Fehler < 7 %).  Ergebnis: nur noch die
     *        Phase zaehlt, Schwund und Pegel sind egal. ---------------- */
    uint16_t ai = (uint16_t)(I < 0 ? -I : I);
    uint16_t aq = (uint16_t)(Q < 0 ? -Q : Q);
    uint16_t mx = ai > aq ? ai : aq;
    uint16_t mn = ai > aq ? aq : ai;
    uint16_t m  = (uint16_t)(mx + (mn >> 2) + (mn >> 3));
    level_avg = (uint16_t)(level_avg + (((int32_t)m - level_avg) >> 6));
    if (m < 4) {
        m = 4;
    }
    int32_t g  = ((int32_t)ZAMP << 8) / m;       /* eine Division je Abtastwert */
    int16_t zi = (int16_t)(((int32_t)I * g) >> 8);
    int16_t zq = (int16_t)(((int32_t)Q * g) >> 8);

    /* --- 2) Frequenzdiskriminator (Verzoegerung um 1 Abtastwert):
     *        Im(z[n] * conj(z[n-1])) ~ sin(Phasenschritt) ~ Momentanfrequenz.
     *        Positiv = Signal liegt ueber der NCO-Frequenz. ---------------- */
    int32_t di = (int32_t)zq * zi_p - (int32_t)zi * zq_p;
    zi_p = zi;
    zq_p = zq;

    /* --- 3) Traegernachfuehrung: Mittelwert des Diskriminators -------- */
    fll_acc += di >> 6;
    if (++fll_n >= FLL_BLOCK) {
        fll_update();
    }

    /* --- 4) Nulldurchgang des (2-Punkt-geglaetteten) Diskriminators ---
     *        = Bitgrenze, wenn sich der Ton aendert.  Nur "steile"
     *        Durchgaenge zaehlen (Rauschen in flachen Stuecken nicht). --- */
    int32_t ds = di + di_p;
    int32_t adiff = ds - ds_p;
    uint8_t cross = 0;
    uint16_t fr16 = 0;
    if ((ds >= 0) != (ds_p >= 0)) {
        if ((adiff < 0 ? -adiff : adiff) > (abs_avg >> 3)) {
            /* Lage des Durchgangs zwischen den beiden Abtastwerten, 0..65536 */
            int32_t den = adiff >> 10;
            if (den == 0) {
                den = (adiff < 0) ? -1 : 1;
            }
            int32_t f = (-(ds_p >> 10) * 65536L) / den;
            if (f < 0) f = 0;
            if (f > 65536L) f = 65536L;
            fr16 = (uint16_t)f;
            cross = 1;
        }
    }
    abs_avg += ((ds < 0 ? -ds : ds) - abs_avg) >> 5;
    di_p = di;
    ds_p = ds;

    /* --- 5) Bittakt-DPLL und Integrate&Dump in jeder Lane -------------- */
    uint8_t kp = (active >= 0) ? 4 : 3;          /* Schleifenverstaerkung 1/16 bzw. 1/8 */
    for (i = 0; i < NLANES; i++) {
        lane_t *L = &lane[i];
        if (cross) {
            /* Bittakt-Phase im Moment des Nulldurchgangs (1 Abtastwert
             * Glaettungs-/Diskriminatorversatz eingerechnet).  Nahe 0 heisst
             * "Takt stimmt"; sonst Phase entsprechend verschieben. */
            int32_t pc = L->phase + (((int32_t)L->inc * ((int32_t)fr16 - 65536L)) >> 16);
            int16_t e = (int16_t)pc;
            L->phase -= (e >> kp);
        }
        L->phase += L->inc;
        L->acc += di >> 8;
        if (L->phase >= 65536L) {                /* Bitgrenze                */
            L->phase -= 65536L;
            uint8_t bit = (L->acc > 0);          /* Vorzeichen = Ton = Bit   */
            L->acc = 0;
            lane_bit(i, bit);
        }
    }

    /* --- 6) Zeitbasis -------------------------------------------------- */
    if (++ms_cnt >= 1000) {
        ms_cnt = 0;
        second_tick();
    }
}

/* ------------------------------------------------------------------ */
/* Initialisierung und Hauptschleife                                    */
/* ------------------------------------------------------------------ */

static void dgps_init(void)
{
    static const uint16_t bauds[NLANES] = { 50, 100, 200 };
    uint8_t i;
    for (i = 0; i < NLANES; i++) {
        lane[i].baud  = bauds[i];
        lane[i].inc   = (uint16_t)(((uint32_t)bauds[i] * 65536UL + FB_HZ / 2) / FB_HZ);
        lane[i].phase = 0;
        lane[i].st    = ST_HUNT;
    }
}

/* Alles, was die Hauptschleife je Durchlauf tut. */
static void main_step(void)
{
    cplx16_t z;
    while (rx_pop(&z)) {
        process_sample(z.i, z.q);
    }
    printer_pump();
}

#ifndef HOST_SIM
int main(void)
{
    dgps_init();
    uart_init();
    timer1_init();
    adc_init();
    sei();

    puts_P(PSTR("DGPS-Decoder (RTCM SC-104/MSK) ATmega328P, 4 kHz ADC, 50/100/200 Bit/s\r\n"));
    puts_P(PSTR("Erwarte NF-Signal um "));
    put_udec(AUDIO_CENTER_HZ);
    puts_P(PSTR(" Hz an ADC0 ...\r\n"));

    for (;;) {
        main_step();
    }
}
#endif
