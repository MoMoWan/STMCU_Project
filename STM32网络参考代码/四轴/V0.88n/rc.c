/*#######################################################################################
Decodieren eines RC Summen Signals
#######################################################################################*/
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

#include "rc.h"
#include "main.h"
// Achtung: ACT_S3D_SUMMENSIGNAL wird in der Main.h gesetzt

volatile int PPM_in[26];
volatile int PPM_diff[26];  // das diffenzierte Stick-Signal
volatile char Channels,tmpChannels = 0;
volatile unsigned char NewPpmData = 1;
unsigned int PPM_Neutral = 466;

//############################################################################
// Clear the values
void rc_sum_init (void)
//############################################################################
{
 unsigned char i;
 for(i=0;i<26;i++)
  {
   if(i < 5) PPM_in[i] = 0; else PPM_in[i] = -126;
   PPM_diff[i] = 0;
  }
    AdNeutralGier = 0;
    AdNeutralRoll = 0;
    AdNeutralNick = 0;
    return;
}

#ifndef ACT_S3D_SUMMENSIGNAL
//############################################################################
// Interrupt function for the PPM-Input
ISR(TIMER1_CAPT_vect)
//############################################################################
{
if(!(EE_Parameter.ExtraConfig & CFG_SENSITIVE_RC))
 {
	static unsigned int AltICR=0;
    signed int signal = 0,tmp;
	static int index;

	signal = (unsigned int) ICR1 - AltICR;
	AltICR = ICR1;
    //Syncronisationspause? (3.52 ms < signal < 25.6 ms)
	if((signal > 1100) && (signal < 8000))
        {
        Channels = index;
        if(index >= 4)  NewPpmData = 0;  // Null bedeutet: Neue Daten
        index = 1;
        }
 	else
        {
        if(index < 13)
            {
            if((signal > 250) && (signal < 687))
                {
                signal -= PPM_Neutral;
                // Stabiles Signal
			  	  if(EE_Parameter.FailsafeChannel == 0 || PPM_in[EE_Parameter.FailsafeChannel] < 100)  // forces Failsafe if the receiver doesn't have 'signal loss' on Failsafe
				  {
                    if(abs(signal - PPM_in[index]) < 6) { if(SenderOkay < 200) SenderOkay += 10; else SenderOkay = 200;}
				  }	
                tmp = (3 * (PPM_in[index]) + signal) / 4;
                if(tmp > signal+1) tmp--; else
                if(tmp < signal-1) tmp++;
                if(SenderOkay >= 195)  PPM_diff[index] = ((tmp - PPM_in[index]) / 3) * 3;
                else PPM_diff[index] = 0;
                PPM_in[index] = tmp;
                }
            index++;
		   if(PlatinenVersion < 20)
            {	
             if(index == 5) J3High; else J3Low;  // Servosignal an J3 anlegen
             if(index == 6) J4High; else J4Low;  // Servosignal an J4 anlegen
             if(index == 7) J5High; else J5Low;  // Servosignal an J5 anlegen
			}
        }
	}
 }
 else
 {
	static unsigned int AltICR=0;
    static int ppm_in[13];
    static int ppm_diff[13];
    static int old_ppm_in[13];
    static int old_ppm_diff[13];
    signed int signal = 0,tmp;
	static unsigned char index, okay_cnt = 0;
	signal = (unsigned int) ICR1 - AltICR;
	AltICR = ICR1;
    //Syncronisationspause? (3.52 ms < signal < 25.6 ms)
	if((signal > 1100) && (signal < 8000))
        {
        tmpChannels = index;
        if(tmpChannels >= 4 && Channels == tmpChannels)
		 {
          if(okay_cnt > 10)
		   {
		   NewPpmData = 0;  // Null bedeutet: Neue Daten
		   for(index = 0; index < 13; index++)
		    {
			 if(okay_cnt > 30)
			  {
			   old_ppm_in[index] = PPM_in[index];
			   old_ppm_diff[index] = PPM_diff[index];
			  }
		     PPM_in[index] = ppm_in[index];
		     PPM_diff[index] = ppm_diff[index];
		    }
		   }
          if(okay_cnt < 255) okay_cnt++;
		 }
         else
		  {
		   if(okay_cnt > 100) okay_cnt = 10; else okay_cnt = 0;
		   ROT_ON;
		  }
        index = 1;
        if(!MotorenEin) Channels = tmpChannels;
        }
 	else
        {
        if(index < 13)
            {
            if((signal > 250) && (signal < 687))
                {
                signal -= PPM_Neutral;
                // Stabiles Signal
                if((abs(signal - ppm_in[index]) < 6))
				 {
			  	  if(EE_Parameter.FailsafeChannel == 0 || PPM_in[EE_Parameter.FailsafeChannel] < 100)  // forces Failsafe if the receiver doesn't have 'signal loss' on Failsafe
				  {
				   if(okay_cnt > 25)  SenderOkay += 10;
				   else
				   if(okay_cnt > 10)  SenderOkay += 2;
				   if(SenderOkay > 200) SenderOkay = 200;
				  } 
				 }
                tmp = (3 * (ppm_in[index]) + signal) / 4;
                if(tmp > signal+1) tmp--; else
                if(tmp < signal-1) tmp++;
                if(SenderOkay >= 190)  ppm_diff[index] = ((tmp - ppm_in[index]) / 3) * 3;
                else ppm_diff[index] = 0;
                ppm_in[index] = tmp;
                }
			else ROT_ON;
		   if(PlatinenVersion < 20)
            {	
             if(index == 5) J3High; else J3Low;  // Servosignal an J3 anlegen
             if(index == 6) J4High; else J4Low;  // Servosignal an J4 anlegen
             if(index == 7) J5High; else J5Low;  // Servosignal an J5 anlegen
			}
          }
		  if(index < 20) index++;
          else
		  if(index == 20)
		  {
            unsigned char i;
            ROT_ON;
		    index = 30;
            for(i=0;i<13;i++) // restore from older data
			 {
		      PPM_in[i] = old_ppm_in[i];
		      PPM_diff[i] = 0;
//			  okay_cnt /= 2;
 	         }
		  }
	    }
 }
}

#else
//############################################################################
// Interrupt function for the PPM-Input
ISR(TIMER1_CAPT_vect)
//############################################################################

{
	static unsigned int AltICR=0;
    signed int signal = 0,tmp;
	static int index;

	signal = (unsigned int) ICR1 - AltICR;
	signal /= 2;
	AltICR = ICR1;
    //Syncronisationspause?
	if((signal > 1100*2) && (signal < 8000*2))
        {
        if(index >= 4)  NewPpmData = 0;  // Null bedeutet: Neue Daten
        index = 1;
        }
 	else
        {
        if(index < 13)
            {
            if((signal > 250) && (signal < 687*2))
                {
                signal -= 962;
                // Stabiles Signal
                if(abs(signal - PPM_in[index]) < 6) { if(SenderOkay < 200) SenderOkay += 10;}
                tmp = (3 * (PPM_in[index]) + signal) / 4;
                if(tmp > signal+1) tmp--; else
                if(tmp < signal-1) tmp++;
                if(SenderOkay >= 195)  PPM_diff[index] = ((tmp - PPM_in[index]) / 3) * 3;
                else PPM_diff[index] = 0;
                PPM_in[index] = tmp;
                }
            index++;
        }
	}
}
#endif



