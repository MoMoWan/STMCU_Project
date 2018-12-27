// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + www.MikroKopter.com
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + Software Nutzungsbedingungen (english version: see below)
// + der Fa. HiSystems GmbH, Flachsmeerstrasse 2, 26802 Moormerland - nachfolgend Lizenzgeber genannt -
// + Der Lizenzgeber r�umt dem Kunden ein nicht-ausschlie�liches, zeitlich und r�umlich* unbeschr�nktes Recht ein, die im den
// + Mikrocontroller verwendete Firmware f�r die Hardware Flight-Ctrl, Navi-Ctrl, BL-Ctrl, MK3Mag & PC-Programm MikroKopter-Tool 
// + - nachfolgend Software genannt - nur f�r private Zwecke zu nutzen.
// + Der Einsatz dieser Software ist nur auf oder mit Produkten des Lizenzgebers zul�ssig.
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + Die vom Lizenzgeber gelieferte Software ist urheberrechtlich gesch�tzt. Alle Rechte an der Software sowie an sonstigen im
// + Rahmen der Vertragsanbahnung und Vertragsdurchf�hrung �berlassenen Unterlagen stehen im Verh�ltnis der Vertragspartner ausschlie�lich dem Lizenzgeber zu.
// + Die in der Software enthaltenen Copyright-Vermerke, Markenzeichen, andere Rechtsvorbehalte, Seriennummern sowie
// + sonstige der Programmidentifikation dienenden Merkmale d�rfen vom Kunden nicht ver�ndert oder unkenntlich gemacht werden.
// + Der Kunde trifft angemessene Vorkehrungen f�r den sicheren Einsatz der Software. Er wird die Software gr�ndlich auf deren
// + Verwendbarkeit zu dem von ihm beabsichtigten Zweck testen, bevor er diese operativ einsetzt.
// + Die Haftung des Lizenzgebers wird - soweit gesetzlich zul�ssig - begrenzt in H�he des typischen und vorhersehbaren
// + Schadens. Die gesetzliche Haftung bei Personensch�den und nach dem Produkthaftungsgesetz bleibt unber�hrt. Dem Lizenzgeber steht jedoch der Einwand 
// + des Mitverschuldens offen.
// + Der Kunde trifft angemessene Vorkehrungen f�r den Fall, dass die Software ganz oder teilweise nicht ordnungsgem�� arbeitet.
// + Er wird die Software gr�ndlich auf deren Verwendbarkeit zu dem von ihm beabsichtigten Zweck testen, bevor er diese operativ einsetzt.
// + Der Kunde wird er seine Daten vor Einsatz der Software nach dem Stand der Technik sichern.
// + Der Kunde ist dar�ber unterrichtet, dass der Lizenzgeber seine Daten im zur Vertragsdurchf�hrung erforderlichen Umfang
// + und auf Grundlage der Datenschutzvorschriften erhebt, speichert, verarbeitet und, sofern notwendig, an Dritte �bermittelt.
// + *) Die r�umliche Nutzung bezieht sich nur auf den Einsatzort, nicht auf die Reichweite der programmierten Software.
// + #### ENDE DER NUTZUNGSBEDINGUNGEN ####'
// +  Hinweis: Informationen �ber erweiterte Nutzungsrechte (wie z.B. Nutzung f�r nicht-private Zwecke) sind auf Anfrage per Email an info(@)hisystems.de verf�gbar.
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + Software LICENSING TERMS
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + of HiSystems GmbH, Flachsmeerstrasse 2, 26802 Moormerland, Germany - the Licensor -
// + The Licensor grants the customer a non-exclusive license to use the microcontroller firmware of the Flight-Ctrl, Navi-Ctrl, BL-Ctrl, and MK3Mag hardware 
// + (the Software) exclusively for private purposes. The License is unrestricted with respect to time and territory*.
// + The Software may only be used with the Licensor's products.
// + The Software provided by the Licensor is protected by copyright. With respect to the relationship between the parties to this
// + agreement, all rights pertaining to the Software and other documents provided during the preparation and execution of this
// + agreement shall be the property of the Licensor.
// + The information contained in the Software copyright notices, trademarks, other legal reservations, serial numbers and other
// + features that can be used to identify the program may not be altered or defaced by the customer.
// + The customer shall be responsible for taking reasonable precautions
// + for the safe use of the Software. The customer shall test the Software thoroughly regarding its suitability for the
// + intended purpose before implementing it for actual operation. The Licensor's liability shall be limited to the extent of typical and
// + foreseeable damage to the extent permitted by law, notwithstanding statutory liability for bodily injury and product
// + liability. However, the Licensor shall be entitled to the defense of contributory negligence.
// + The customer will take adequate precautions in the case, that the software is not working properly. The customer will test
// + the software for his purpose before any operational usage. The customer will backup his data before using the software.
// + The customer understands that the Licensor collects, stores and processes, and, where required, forwards, customer data
// + to third parties to the extent necessary for executing the agreement, subject to applicable data protection and privacy regulations.
// + *) The territory aspect only refers to the place where the Software is used, not its programmed range.
// + #### END OF LICENSING TERMS ####
// + Note: For information on license extensions (e.g. commercial use), please contact us at info(@)hisystems.de.
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include "main.h"
#include "eeprom.h"
volatile int  Aktuell_Nick,Aktuell_Roll,Aktuell_Gier,Aktuell_ax, Aktuell_ay,Aktuell_az, UBat = 100;
volatile int  AdWertNickFilter = 0, AdWertRollFilter = 0, AdWertGierFilter = 0;
volatile int  HiResNick = 2500, HiResRoll = 2500;
volatile int  AdWertNick = 0, AdWertRoll = 0, AdWertGier = 0;
volatile int  AdWertAccRoll = 0,AdWertAccNick = 0,AdWertAccHoch = 0;
volatile long Luftdruck = 32000;
volatile long SummenHoehe = 0;
volatile int  StartLuftdruck;
volatile unsigned int  MessLuftdruck = 1023;
unsigned char DruckOffsetSetting;
signed char ExpandBaro = 0;
volatile int VarioMeter = 0;
volatile unsigned int ZaehlMessungen = 0;
unsigned char AnalogOffsetNick = 115,AnalogOffsetRoll = 115,AnalogOffsetGier = 115;
volatile unsigned char AdReady = 1;

//#######################################################################################
void ADC_Init(void)
//#######################################################################################
{
    ADMUX = 0;//Referenz ist extern
    ANALOG_ON;
}

#define DESIRED_H_ADC 800

void SucheLuftruckOffset(void)
{
 unsigned int off;
 ExpandBaro = 0;

#if (defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__))
 {
  unsigned char off2;
  OCR0A = 150;
  off2 = GetParamByte(PID_PRESSURE_OFFSET);
  if(off2 < 230) off2 += 10;
  OCR0B = off2;
  Delay_ms_Mess(100);
  if(MessLuftdruck > DESIRED_H_ADC) off2 = 240;
  for(; off2 >= 5; off2 -= 5)
   {
   OCR0B = off2;
   Delay_ms_Mess(50);
   printf("*");
   if(MessLuftdruck > DESIRED_H_ADC) break;
   }
   SetParamByte(PID_PRESSURE_OFFSET, off2);
  if(off2 >= 15) off = 140; else off = 0;
  for(; off < 250;off++)
   {
   OCR0A = off;
   Delay_ms_Mess(50);
   printf(".");
   if(MessLuftdruck < DESIRED_H_ADC) break;
   }
   DruckOffsetSetting = off;
 }
#else
  off = GetParamByte(PID_PRESSURE_OFFSET);
  if(off > 20) off -= 10;
  OCR0A = off;
  Delay_ms_Mess(100);
  if(MessLuftdruck < DESIRED_H_ADC) off = 0;
  for(; off < 250;off++)
   {
   OCR0A = off;
   Delay_ms_Mess(50);
   printf(".");
   if(MessLuftdruck < DESIRED_H_ADC) break;
   }
   DruckOffsetSetting = off;
   SetParamByte(PID_PRESSURE_OFFSET, off);
#endif
 if((EE_Parameter.GlobalConfig & CFG_HOEHENREGELUNG) && (DruckOffsetSetting < 10 || DruckOffsetSetting >= 245)) VersionInfo.HardwareError[0] |= FC_ERROR0_PRESSURE;
 OCR0A = off;
 Delay_ms_Mess(300);
}


void SucheGyroOffset(void)
{
 unsigned char i, ready = 0;
 int timeout;
 timeout = SetDelay(2000);
 for(i=140; i != 0; i--)
  {
   if(ready == 3 && i > 10) i = 9;
   ready = 0;
   if(AdWertNick < 1020) AnalogOffsetNick--; else if(AdWertNick > 1030) AnalogOffsetNick++; else ready++;
   if(AdWertRoll < 1020) AnalogOffsetRoll--; else if(AdWertRoll > 1030) AnalogOffsetRoll++; else ready++;
   if(AdWertGier < 1020) AnalogOffsetGier--; else if(AdWertGier > 1030) AnalogOffsetGier++; else ready++;
   I2C_Start(TWI_STATE_GYRO_OFFSET_TX);
   if(AnalogOffsetNick < 10)  { VersionInfo.HardwareError[0] |= FC_ERROR0_GYRO_NICK; AnalogOffsetNick = 10;}; if(AnalogOffsetNick > 245) { VersionInfo.HardwareError[0] |= FC_ERROR0_GYRO_NICK; AnalogOffsetNick = 245;};
   if(AnalogOffsetRoll < 10)  { VersionInfo.HardwareError[0] |= FC_ERROR0_GYRO_ROLL; AnalogOffsetRoll = 10;}; if(AnalogOffsetRoll > 245) { VersionInfo.HardwareError[0] |= FC_ERROR0_GYRO_ROLL; AnalogOffsetRoll = 245;};
   if(AnalogOffsetGier < 10)  { VersionInfo.HardwareError[0] |= FC_ERROR0_GYRO_YAW;  AnalogOffsetGier = 10;}; if(AnalogOffsetGier > 245) { VersionInfo.HardwareError[0] |= FC_ERROR0_GYRO_YAW;  AnalogOffsetGier = 245;};
   while(twi_state) if(CheckDelay(timeout)) {printf("\n\r DAC or I2C ERROR! Check I2C, 3Vref, DAC and BL-Ctrl"); break;}
   AdReady = 0;
   ANALOG_ON;
   while(!AdReady);
   if(i<10) Delay_ms_Mess(10);
  }
   Delay_ms_Mess(70);
}

/*
0  n
1  r
2     g
3     y
4     x
5  n
6  r
7     u
8     z
9     L
10 n
11 r
12    g
13    y
14    x
15 n
16 r
17    L
*/

//#######################################################################################
//
ISR(ADC_vect)
//#######################################################################################
{
    static unsigned char kanal=0,state = 0;
	static signed char subcount = 0;
    static signed int gier1, roll1, nick1, nick_filter, roll_filter;
	static signed int accy, accx;
	static long tmpLuftdruck = 0;
	static char messanzahl_Druck = 0;
    switch(state++)
        {
        case 0:
            nick1 = ADC;
            kanal = AD_ROLL;
            break;
        case 1:
            roll1 = ADC;
		    kanal = AD_GIER;
            break;
        case 2:
            gier1 = ADC;
            kanal = AD_ACC_Y;
            break;
        case 3:
            Aktuell_ay = NeutralAccY - ADC;
            accy = Aktuell_ay;
		    kanal = AD_ACC_X;
            break;
        case 4:
            Aktuell_ax = ADC - NeutralAccX;
            accx =  Aktuell_ax;
            kanal = AD_NICK;
            break;
        case 5:
            nick1 += ADC;
            kanal = AD_ROLL;
            break;
        case 6:
            roll1 += ADC;
            kanal = AD_UBAT;
            break;
        case 7:
#if (defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__))
            if(EE_Parameter.ExtraConfig & CFG_3_3V_REFERENCE) UBat = (3 * UBat + (11 * ADC) / 30) / 4; // there were some single FC2.1 with 3.3V reference
			else   
#endif
			UBat = (3 * UBat + ADC / 3) / 4;
		    kanal = AD_ACC_Z;
            break;
       case 8:
            AdWertAccHoch =  (signed int) ADC - NeutralAccZ;
            if(AdWertAccHoch > 1)
             {
              if(NeutralAccZ < 750)
               {
                subcount += 5;
                if(modell_fliegt < 500) subcount += 10;
               }
              if(subcount > 100) { NeutralAccZ++; subcount -= 100;}
             }
             else if(AdWertAccHoch < -1)
             {
              if(NeutralAccZ > 550)
                {
                 subcount -= 5;
                 if(modell_fliegt < 500) subcount -= 10;
                 if(subcount < -100) { NeutralAccZ--; subcount += 100;}
                }
             }
//            messanzahl_AccHoch = 1;
            Aktuell_az = ADC;
            Mess_Integral_Hoch += AdWertAccHoch;      // Integrieren
            Mess_Integral_Hoch -= Mess_Integral_Hoch / 1024; // d�mfen
 	        kanal = AD_DRUCK;
            break;
   // "case 9:" fehlt hier absichtlich
        case 10:
            nick1 += ADC;
            kanal = AD_ROLL;
            break;
        case 11:
            roll1 += ADC;
		    kanal = AD_GIER;
            break;
        case 12:
            if(PlatinenVersion == 10)  AdWertGier = (ADC + gier1 + 1) / 2;
            else
            if(PlatinenVersion >= 20)  AdWertGier = 2047 - (ADC + gier1);
			else 					   AdWertGier = (ADC + gier1);
            kanal = AD_ACC_Y;
            break;
        case 13:
            Aktuell_ay = NeutralAccY - ADC;
            AdWertAccRoll = (Aktuell_ay + accy);
            kanal = AD_ACC_X;
            break;
        case 14:
            Aktuell_ax = ADC - NeutralAccX;
            AdWertAccNick =  (Aktuell_ax + accx);
            kanal = AD_NICK;
            break;
        case 15:
            nick1 += ADC;
            if(PlatinenVersion == 10) nick1 *= 2; else nick1 *= 4;
            AdWertNick = nick1 / 8;
            nick_filter = (nick_filter + nick1) / 2;
            HiResNick = nick_filter - AdNeutralNick;
            AdWertNickFilter = (AdWertNickFilter + HiResNick) / 2;
            kanal = AD_ROLL;
            break;
        case 16:
            roll1 += ADC;
            if(PlatinenVersion == 10) roll1 *= 2; else roll1 *= 4;
            AdWertRoll = roll1 / 8;
            roll_filter = (roll_filter + roll1) / 2;
            HiResRoll = roll_filter - AdNeutralRoll;
            AdWertRollFilter = (AdWertRollFilter + HiResRoll) / 2;
 	        kanal = AD_DRUCK;
            break;
        case 17:
            state = 0;
			AdReady = 1;
            ZaehlMessungen++;
            // "break" fehlt hier absichtlich
        case 9:
        	MessLuftdruck = ADC;
            tmpLuftdruck += MessLuftdruck;
            if(++messanzahl_Druck >= 16) // war bis 0.86 "18"
            {
			    signed int tmp;
				Luftdruck = (7 * Luftdruck + tmpLuftdruck - (16 * 523) * (long)ExpandBaro + 4) / 8;  // -523.19 counts per 10 counts offset step
				HoehenWert = StartLuftdruck - Luftdruck;
				SummenHoehe -= SummenHoehe/SM_FILTER;
				SummenHoehe += HoehenWert;
				tmp = (HoehenWert - SummenHoehe/SM_FILTER);
				if(tmp > 1024) tmp = 1024; 	else if(tmp < -1024) tmp = -1024; 
                if(abs(VarioMeter) > 700) VarioMeter = (15 * VarioMeter + 8 * tmp)/16;
				else VarioMeter = (31 * VarioMeter + 8 * tmp)/32;
                tmpLuftdruck /= 2;
                messanzahl_Druck = 16/2;
            }
            kanal = AD_NICK;
            break;
        default:
            kanal = 0; state = 0; kanal = AD_NICK;
            break;
        }
    ADMUX = kanal;
    if(state != 0) ANALOG_ON;
}

