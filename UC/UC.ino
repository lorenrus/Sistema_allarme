                             
#include <EEPROM.h>
#include <Keypad.h>
#include <Key.h>
#include <SPI.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>


#define REED_SENSOR 2
#define Reed_Sensor_Eprom_Addr 7

#define SET_HC12 4

#define LED_VERDE 10
#define LED_ROSSO 8

#define SIRENA  11
#define BUZZER  A15

#define STATO_ALLARME 0
#define STATO_ALIMENTAZIONE 8


#define RIGHE   4
#define COLONNE 4


char tasti[RIGHE][COLONNE] =
{
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
};


byte pinRighe[RIGHE] = {48,46,44,42};
byte pinColonne[COLONNE] = {49,47,45};


Keypad Tastiera = Keypad( makeKeymap(tasti), pinRighe,pinColonne,RIGHE,COLONNE);


LiquidCrystal_I2C   Schermo(0x27, 20, 4);

void(* resetFunc) (void) = 0;

struct HC12 {

  int HC12_Command_Send_to_Box = 0;
  int HC12_Command_Receive_from_Box = 0;

  bool Enable_Receive_HC12 = false;
};

struct HC12 Modulo_HC12;


struct Variabili_Sistema_Allarme {

  bool Box_Camera_Lorenzo = false;
  bool Box_Camera_Mamma = false;
  bool Box_1_Sala = false;
  bool Box_2_Sala = false;
  bool Box_3_Sala = false;
  bool Box_Cucina = false;

  bool Enable_Camera_Lorenzo = false;
  bool Enable_Camera_Mamma = false;
  bool Enable_Sala_1 = false;
  bool Enable_Sala_2 = false;
  bool Enable_Sala_3 = false;
  bool Enable_Cucina = false;

  uint8_t Vec_Psw_Inserita[8];

  uint8_t Psw_Allarme[8] = {1,1,1,1,1,1,1,1};

  uint8_t Num_Max_Tentativi = 3;

  uint8_t Controllo_Riavvio_Sistema;

  uint8_t Allarm_Activated = 1;
  uint8_t Allarm_Disactivated = 0;

  uint8_t SI_Effrazione = 1;
  uint8_t NO_Effrazione = 0;

  bool Effrazione_Attiva_from_Box = false;

  bool Effrazione_Attiva_from_Reed_Interno = false;
};

struct Variabili_Sistema_Allarme Sistema_Allarme;


enum Enum_Box {

  No_Box = 0,
  Box_Camera_Lorenzo = 1,
  Box_Camera_Mamma = 2,
  Box_Cucina = 3,
  Box_Number_4 = 4,
  Box_Number_5 = 5,
  Box_Number_6 = 6
};

enum Box_Activation {

  Active_Box_Camera_Lorenzo = 11,
  Active_Box_Camera_Mamma = 12,
  Active_Box_Cucina = 13,
  Active_Box_Number_4 = 14,
  Active_Box_Number_5 = 15,
  Active_Box_Number_6 = 16
};

enum Box_Effrazione {

  Effrazione_Box_Camera_Lorenzo = 11,
  Effrazione_Box_Camera_Mamma = 12,
  Effrazione_Box_Cucina = 13,
  Effrazione_Box_Number_4 = 14,
  Effrazione_Box_Number_5 = 15,
  Effrazione_Box_Number_6 = 16
};

uint8_t Vettore_Rilevamento_Effrazione[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};



bool Enable_ISR_Func_from_ISR = false;

char Command = 0;
uint8_t Comando_Inserito;
uint8_t indice_Main = 0;
uint8_t indice_Check_Psw = 0;
uint8_t indice_Check_Effrazione = 1;
uint8_t Count_Psw = 0;


unsigned long t = 0;


uint8_t Vettore_Allarme_Box_Camera_Lorenzo = 0;
uint8_t Vettore_Allarme_Box_Camera_Mamma = 0;
uint8_t Vettore_Allarme_Box_Sala_1 = 0;
uint8_t Vettore_Allarme_Box_Sala_2 = 0;
uint8_t Vettore_Allarme_Box_Sala_3 = 0;
uint8_t Vettore_Allarme_Box_Cucina = 0;


int PinBatt = A9;


void setup() {

  Serial.begin(9600);
  Serial1.begin(9600);
  Serial2.begin(19200);
  Serial3.begin(9600);

  pinMode(BUZZER, OUTPUT);
  pinMode(SIRENA, OUTPUT);

  digitalWrite(SIRENA, LOW);

  pinMode(LED_VERDE, OUTPUT);
  pinMode(LED_ROSSO, OUTPUT);

  pinMode(PinBatt, INPUT);

  pinMode(SET_HC12, OUTPUT);
  digitalWrite(SET_HC12, HIGH);

  pinMode(REED_SENSOR, INPUT_PULLUP);

  pinMode(11, INPUT_PULLUP);
  pinMode(13, INPUT_PULLUP);

  // Da commentare quando si fanno le prove programmando arduino
  // altrimenti ogni volta che lo programmiamo che lui si riavvia entrerà sempre nell'if
  // dei controllo di questa variabile
  Sistema_Allarme.Controllo_Riavvio_Sistema = 1;
  EEPROM.write(STATO_ALIMENTAZIONE, Sistema_Allarme.Controllo_Riavvio_Sistema);

  Schermo.init();
  Schermo.backlight();
  Estetica_Display_LCD();
  Schermo.setCursor(4, 1);
  Schermo.print("Casa Urbani");
  Schermo.setCursor(1, 2);
  Schermo.print("Sistema d'allarme");
}


void loop() {

  Main_Task();

  Reed_Sensor();

  Check_Voltage();

  Receive_Message_From_Box();
}


void Main_Task() {

  /* In attesa dei comandi provenienti dalla tastiera */

  Command = Tastiera.getKey();

  Comando_Inserito = Command - '0';

  if(Command && indice_Main < 8) {

    Serial.println(Comando_Inserito);

    if(indice_Main < 8) {

      Sistema_Allarme.Vec_Psw_Inserita[indice_Main] =  Comando_Inserito;
    }

    indice_Main ++;
  }
  else if(indice_Main >= 8) {

    indice_Main = 0;

    Check_Pws_Inserita();
  }


  if(EEPROM.read(STATO_ALIMENTAZIONE) == 1) {

    Serial.println("RIAVVIATO PER MANCATA ALIMENTAZIONE");

    if(EEPROM.read(STATO_ALLARME) == Sistema_Allarme.Allarm_Disactivated) {

      detachInterrupt(digitalPinToInterrupt(REED_SENSOR));

      Serial.println("ALLARME RIATTIVATO IN STATO DISATTIVATO");

      digitalWrite(LED_VERDE, HIGH);
      digitalWrite(LED_ROSSO, LOW);

      Modulo_HC12.Enable_Receive_HC12 = false;
    }
    else if(EEPROM.read(STATO_ALLARME) == Sistema_Allarme.Allarm_Activated) {

      attachInterrupt(digitalPinToInterrupt(REED_SENSOR), ISR_Func_Reed, HIGH);

      Serial.println("ALLARME RIATTIVATO IN STATO ATTIVATO");

      digitalWrite(LED_VERDE, LOW);
      digitalWrite(LED_ROSSO, HIGH);

      for(indice_Check_Effrazione = 1; indice_Check_Effrazione <= 6; indice_Check_Effrazione ++) {
        
        if(EEPROM.read(indice_Check_Effrazione) == 1) {

          Serial.println("EFFRAZIONE BOX ATTIVA DOPO IL POWER CYCLE");

          Sistema_Allarme.Effrazione_Attiva_from_Box = true;

          digitalWrite(SIRENA, HIGH);
          tone(BUZZER, 120);
        }
      }

      if(EEPROM.read(Reed_Sensor_Eprom_Addr) == 1) {

          Serial.println("EFFRAZIONE REED ATTIVA DOPO IL POWER CYCLE");

          Sistema_Allarme.Effrazione_Attiva_from_Reed_Interno = true;

          digitalWrite(SIRENA, HIGH);
          tone(BUZZER, 120);
        }

      Modulo_HC12.Enable_Receive_HC12 = true;
    }

    Sistema_Allarme.Controllo_Riavvio_Sistema = 0;
    EEPROM.write(STATO_ALIMENTAZIONE, Sistema_Allarme.Controllo_Riavvio_Sistema);
  }
}


void Check_Pws_Inserita() {

  for(indice_Check_Psw = 0; indice_Check_Psw <= 7; indice_Check_Psw ++) {

    if(Sistema_Allarme.Psw_Allarme[indice_Check_Psw] == Sistema_Allarme.Vec_Psw_Inserita[indice_Check_Psw]) {

      Count_Psw ++;
    }
  }

  if(Count_Psw == 8) {

    //Serial.println("Password corretta");

    Sistema_Allarme.Enable_Camera_Lorenzo = true;

    if(EEPROM.read(STATO_ALLARME) == Sistema_Allarme.Allarm_Disactivated) { // L'allarme era precedentemente disattivato quindi lo attivo

      Serial.println("ABILITO LA RICEZIONE");

      Schermo.clear();
      Estetica_Display_LCD();
      Schermo.setCursor(2, 1);
      Schermo.print("ATTIVAZIONE");
      Schermo.setCursor(11, 2);
      Schermo.print("ALLARME");

      digitalWrite(LED_VERDE, LOW);

      attachInterrupt(digitalPinToInterrupt(REED_SENSOR), ISR_Func_Reed, HIGH);

      EEPROM.write(STATO_ALLARME, Sistema_Allarme.Allarm_Activated);

      Send_Message_To_Box();

      digitalWrite(LED_ROSSO, HIGH);

      Modulo_HC12.Enable_Receive_HC12 = true;

      Schermo.clear();
      Estetica_Display_LCD();
      Schermo.setCursor(2, 1);
      Schermo.print("ALLARME");
      Schermo.setCursor(12, 2);
      Schermo.print("ATTIVO");

      delay(5000);

      Schermo.clear();
      Estetica_Display_LCD();
      Schermo.setCursor(4, 1);
      Schermo.print("Casa Urbani");
      Schermo.setCursor(1, 2);
      Schermo.print("Sistema d'allarme");
    }
    else if(EEPROM.read(STATO_ALLARME) == Sistema_Allarme.Allarm_Activated) { // L'allarme era precedentemente attivato quindi lo disattivo

      Serial.println("DISABILITO LA RICEZIONE");

      Schermo.clear();
      Estetica_Display_LCD();
      Schermo.setCursor(2, 1);
      Schermo.print("DISATTIVAZIONE");
      Schermo.setCursor(11, 2);
      Schermo.print("ALLARME");

      if(Sistema_Allarme.Effrazione_Attiva_from_Box) {

        Serial.println("Sirena fermata");
        digitalWrite(SIRENA, LOW);

        noTone(BUZZER);

        /* Reset delle vettore effrazioni*/
        for(indice_Check_Effrazione = 1; indice_Check_Effrazione <= 6; indice_Check_Effrazione ++) {
        
          EEPROM.write(indice_Check_Effrazione, Sistema_Allarme.NO_Effrazione);
        }

        Sistema_Allarme.Effrazione_Attiva_from_Box = false;
      }
      else if(Sistema_Allarme.Effrazione_Attiva_from_Reed_Interno) {

        Serial.println("Sirena fermata");
        digitalWrite(SIRENA, LOW);

        noTone(BUZZER);

        Sistema_Allarme.Effrazione_Attiva_from_Reed_Interno = false;
      }
      else {

        Serial.println("Sirena fermata");
        digitalWrite(SIRENA, LOW);

        noTone(BUZZER);
      }

      digitalWrite(LED_ROSSO, LOW);

      detachInterrupt(digitalPinToInterrupt(REED_SENSOR));

      EEPROM.write(STATO_ALLARME, Sistema_Allarme.Allarm_Disactivated);

      Modulo_HC12.Enable_Receive_HC12 = false;

      Send_Message_To_Box();

      digitalWrite(LED_VERDE, HIGH);

      Schermo.clear();
      Estetica_Display_LCD();
      Schermo.setCursor(2, 1);
      Schermo.print("ALLARME");
      Schermo.setCursor(7, 2);
      Schermo.print("DISATTIVATO");

      delay(5000);

      Schermo.clear();
      Estetica_Display_LCD();
      Schermo.setCursor(4, 1);
      Schermo.print("Casa Urbani");
      Schermo.setCursor(1, 2);
      Schermo.print("Sistema d'allarme");
    }

    Count_Psw = 0;

  }
  else {

    Serial.println("Password errata");

    Sistema_Allarme.Num_Max_Tentativi -= 1;

    if(Sistema_Allarme.Num_Max_Tentativi == 0) {

      Serial.println("PWS INSERITE ERRATE -- EFFRAZIONE -- ATTIVO LA SIRENA");

      digitalWrite(SIRENA, HIGH);
      tone(BUZZER, 120);

      /*
      * qui devo prevedere l'invio del messaggio sms
      * verso il proprietario della casa.
      */

      Count_Psw = 0;
    }
    else {

      Serial.print("Tentativi rimasti : ");
      Serial.println(Sistema_Allarme.Num_Max_Tentativi);

      Schermo.clear();
      Estetica_Display_LCD();
      Schermo.setCursor(2, 1);
      Schermo.print("TENTATIVI");
      Schermo.setCursor(6, 2);
      Schermo.print("RIMASTI : ");
      Schermo.setCursor(16, 2);
      Schermo.print(Sistema_Allarme.Num_Max_Tentativi);

      delay(5000);

      Schermo.clear();
      Estetica_Display_LCD();
      Schermo.setCursor(4, 1);
      Schermo.print("inseriscila");
      Schermo.setCursor(1, 2);
      Schermo.print("di nuovo");
    }
  }
}


/*
 *
 * QUESTA FUNZIONE SERVE PER ABILITARE/DISABILITARE L'ALLARME
 * NEI RISPETTIVI BOX
 *
 */
void Send_Message_To_Box() {

  /* Box Camera Lorenzo */

  if(Sistema_Allarme.Enable_Camera_Lorenzo) {
    
    delay(40);

    Serial.println("PRIMO VALORE BOX LORENZO: ");
  
    Serial3.write(Box_Camera_Lorenzo);

    delay(10);

    Sistema_Allarme.Enable_Camera_Lorenzo = false;

    Sistema_Allarme.Box_Camera_Lorenzo = true;
  }

  while(Sistema_Allarme.Box_Camera_Lorenzo) {

    if(Serial3.available() ) {

      Modulo_HC12.HC12_Command_Receive_from_Box = Serial3.read();

      Serial.print("SECONDO VALORE BOX LORENZO: ");
      Serial.println(Modulo_HC12.HC12_Command_Receive_from_Box);

      if(Modulo_HC12.HC12_Command_Receive_from_Box == Box_Camera_Lorenzo) {

        Serial.print("Box Camera Lorenzo SENT : ");
        Serial.println(Modulo_HC12.HC12_Command_Receive_from_Box);

        delay(40);

        if(EEPROM.read(STATO_ALLARME) == Sistema_Allarme.Allarm_Activated) {

          Serial3.write(Active_Box_Camera_Lorenzo);
        }
        else {

          Serial3.write(EEPROM.read(STATO_ALLARME));
        }

        delay(10);

        Sistema_Allarme.Box_Camera_Lorenzo = false;

        Sistema_Allarme.Enable_Camera_Mamma = true;
      }
    }
  }


  delay(1000);


  /* Box Camera Mamma */

  if(Sistema_Allarme.Enable_Camera_Mamma) {
    
    delay(40);

    Serial.println("PRIMO VALORE BOX MAMMA: ");
  
    Serial3.write(Box_Camera_Mamma);

    delay(10);

    Sistema_Allarme.Enable_Camera_Mamma = false;

    Sistema_Allarme.Box_Camera_Mamma = true;
  }

  while(Sistema_Allarme.Box_Camera_Mamma) {

    if(Serial3.available() ) {

      Modulo_HC12.HC12_Command_Receive_from_Box = Serial3.read();

      Serial.print("SECONDO VALORE BOX MAMMA: ");
      Serial.println(Modulo_HC12.HC12_Command_Receive_from_Box);

      if(Modulo_HC12.HC12_Command_Receive_from_Box == Box_Camera_Mamma) {

        Serial.print("Box Camera Mamma SENT : ");
        Serial.println(Modulo_HC12.HC12_Command_Receive_from_Box);

        delay(40);

        if(EEPROM.read(STATO_ALLARME) == Sistema_Allarme.Allarm_Activated) {

          Serial3.write(Active_Box_Camera_Mamma);
        }
        else {

          Serial3.write(EEPROM.read(STATO_ALLARME));
        }

        delay(10);

        Sistema_Allarme.Box_Camera_Mamma = false;

        Sistema_Allarme.Enable_Cucina = true;
      }
    }
  }

  delay(1000);


  /* Box Camera Cucina */

  if(Sistema_Allarme.Enable_Cucina) {
    
    delay(40);

    Serial.println("PRIMO VALORE BOX Cucina: ");
  
    Serial3.write(Box_Cucina);

    delay(10);

    Sistema_Allarme.Enable_Cucina = false;

    Sistema_Allarme.Box_Cucina = true;
  }

  while(Sistema_Allarme.Box_Cucina) {

    if(Serial3.available() ) {

      Modulo_HC12.HC12_Command_Receive_from_Box = Serial3.read();

      Serial.print("SECONDO VALORE BOX CUCINA: ");
      Serial.println(Modulo_HC12.HC12_Command_Receive_from_Box);

      if(Modulo_HC12.HC12_Command_Receive_from_Box == Box_Cucina) {

        Serial.print("Box Cucina SENT : ");
        Serial.println(Modulo_HC12.HC12_Command_Receive_from_Box);

        delay(40);

        if(EEPROM.read(STATO_ALLARME) == Sistema_Allarme.Allarm_Activated) {

          Serial3.write(Active_Box_Cucina);
        }
        else {

          Serial3.write(EEPROM.read(STATO_ALLARME));
        }

        delay(10);

        Sistema_Allarme.Box_Cucina = false;
      }
    }
  }


  /* Box Sala 1 */

  /*Serial3.write();

  while(!Box_1_Sala) {

    if(Serial3.available() ) {

      Command = Serial3.read();

      if(Command == ) {

        SERIAL(Command);

        vTaskDelay(50);

        Serial3.write();

        Box_1_Sala = true;
      }
    }
  }*/


  /* Box Sala 2 */

  /*Serial3.write();

  while(!Box_2_Sala) {

    if(Serial3.available() ) {

      Command = Serial3.read();

      if(Command == ) {

        SERIAL(Command);

        vTaskDelay(50);

        Serial3.write();

        Box_2_Sala = true;
      }
    }
  }*/


  /* Box Sala 3 */

  /*Serial3.write();

  while(!Box_3_Sala) {

    if(Serial3.available() ) {

      Command = Serial3.read();

      if(Command == ) {

        SERIAL(Command);

        vTaskDelay(50);

        Serial3.write();

        Box_3_Sala = true;
      }
    }
  }*/

}


/*
 * FUNZIONI DI CONTROLLO PER IL SENSORE REED INTERNO LETTO ATTRAVERSO L'INTERRUPT
 */
void ISR_Func_Reed() {

  //invio sms

  digitalWrite(SIRENA, HIGH);
  tone(BUZZER, 120);

  Enable_ISR_Func_from_ISR = true;

  EEPROM.write(Reed_Sensor_Eprom_Addr, Sistema_Allarme.SI_Effrazione);

  Sistema_Allarme.Effrazione_Attiva_from_Reed_Interno = true;
}

void Reed_Sensor() {

  if(Enable_ISR_Func_from_ISR) {

    delay(60);

    detachInterrupt(digitalPinToInterrupt(REED_SENSOR));

    Enable_ISR_Func_from_ISR = false;
  }
}


/*
 * FUNZIONE CHE VA A CONTROLLARE CON UNA CADENZA DI 2 MINUTI O SE LA BATTERIA E CARICA E QUINDI
 * DISABILITARE LA CARICA O SE LA BATTERIA E' SCARICA QUINDI ABILIATRE IL FLAG DELLA CARICA UNA
 * VOLTA CHE SI RITORNA CON L'ALIMENTAZIONE DA PRESA E SE TROPPO SCARICA AGGIUNGERE FUNZIONE DA DECIDERE
*/

void Check_Voltage() {

  if(millis()-t >= 60000) {

    Serial.println(millis()-t);

    int val = analogRead(PinBatt);  // read the input pin
    float milliVolts = map (val, 0, 1023, 0, 12000);

    Serial1.println(milliVolts);
    Serial.println(milliVolts);

    if(milliVolts < 10000) {

      // inviare messaggio con indicazione
      // della batteria scarica
    }

    t = millis();

    Serial.println(t);
  }
}



void Receive_Message_From_Box() {

  /* In attesa dei comandi da parte dei sensori delle rispettive Box */

  if(Serial3.available() && Modulo_HC12.Enable_Receive_HC12) {

    Serial.println("PASSO 1 Receive from Box");

    Modulo_HC12.HC12_Command_Receive_from_Box = Serial3.read();

    Serial.println(Modulo_HC12.HC12_Command_Receive_from_Box);

    if(Modulo_HC12.HC12_Command_Receive_from_Box == 11) {

      Serial.print("Box Camera Lorenzo RECEIVED : ");
      Serial.println(Modulo_HC12.HC12_Command_Receive_from_Box);

      EEPROM.write(Box_Camera_Lorenzo, Sistema_Allarme.SI_Effrazione);

      Serial.println("Box Camera Lorenzo - EFFRAZIONE");

      digitalWrite(SIRENA, HIGH);
      tone(BUZZER, 120);
    }
    else if(Modulo_HC12.HC12_Command_Receive_from_Box == Effrazione_Box_Camera_Mamma) {

      Serial.print("Box Camera Mamma RECEIVED : ");
      Serial.println(Modulo_HC12.HC12_Command_Receive_from_Box);

      EEPROM.write(Box_Camera_Mamma, Sistema_Allarme.SI_Effrazione);

      Serial.println("Box Camera Mamma - EFFRAZIONE");

      digitalWrite(SIRENA, HIGH);
      tone(BUZZER, 120);
    }
    else if(Modulo_HC12.HC12_Command_Receive_from_Box == Box_Cucina) {

      Serial.print("Box Cucina RECEIVED : ");
      Serial.println(Modulo_HC12.HC12_Command_Receive_from_Box);

      EEPROM.write(Box_Cucina, Sistema_Allarme.SI_Effrazione);

      Serial.println("Box Cucina - EFFRAZIONE");

      digitalWrite(SIRENA, HIGH);
      tone(BUZZER, 120);
    }
    /*else if(HC12_Command == Modulo_HC12.Box_Number_2 && Enable) {

        Serial.print("Box Camera Mamma RECEIVED : ");
        Serial.println(HC12_Command);

        vTaskDelay(50);

        Serial3.write(Modulo_HC12.Box_Number_2);

        Box_Camera_Mamma_Receive = false;
      }
      else if(HC12_Command == Modulo_HC12.Box_Number_3 && Enable) {

        Serial.print("Box 1 Sala RECEIVED : ");
        Serial.println(HC12_Command);

        vTaskDelay(50);

        Serial3.write(Modulo_HC12.Box_Number_3);

        Box_1_Sala_Receive = false;
      }
      else if(HC12_Command == Modulo_HC12.Box_Number_4 && Enable) {

        Serial.print("Box 2 Sala RECEIVED : ");
        Serial.println(HC12_Command);

        vTaskDelay(50);

        Serial3.write(Modulo_HC12.Box_Number_4);

        Box_2_Sala_Receive = false;
      }
      else if(HC12_Command == Modulo_HC12.Box_Number_5 && Enable) {

        Serial.print("Box 3 Sala RECEIVED : ");
        Serial.println(HC12_Command);

        vTaskDelay(50);

        Serial3.write(Modulo_HC12.Box_Number_5);

        Box_3_Sala_Receive = false;
      }
      else if(HC12_Command == Modulo_HC12.Box_Number_6 && Enable) {

        Serial.print("Box Cucina RECEIVED : ");
        Serial.println(HC12_Command);

        vTaskDelay(50);

        Serial3.write(Modulo_HC12.Box_Number_6);

        Box_Cucina_Receive = false;
      }
      else if(HC12_Command == Modulo_HC12.Box_PIR_Number_7 && Enable) {

        Serial.print("Box PIR Ingresso RECEIVED ALLARM : ");
        Serial.println(HC12_Command);

        vTaskDelay(1);

        Serial3.write(Modulo_HC12.Box_PIR_Number_7);

        Box_PIR_Porta_Ingresso_Receive = false;
        Enable = false;
      }
    }*/

  }
}


void Estetica_Display_LCD() {

  // PRIMA RIGA IN ALTO DEL DISPLY LCD
  Schermo.setCursor(0,0);
  Schermo.print("*");
  Schermo.setCursor(1,0);
  Schermo.print("*");
  Schermo.setCursor(2,0);
  Schermo.print("*");
  Schermo.setCursor(3,0);
  Schermo.print("*");
  Schermo.setCursor(4,0);
  Schermo.print("*");
  Schermo.setCursor(5,0);
  Schermo.print("*");
  Schermo.setCursor(6,0);
  Schermo.print("*");
  Schermo.setCursor(7,0);
  Schermo.print("*");
  Schermo.setCursor(8,0);
  Schermo.print("*");
  Schermo.setCursor(9,0);
  Schermo.print("*");
  Schermo.setCursor(10,0);
  Schermo.print("*");
  Schermo.setCursor(11,0);
  Schermo.print("*");
  Schermo.setCursor(12,0);
  Schermo.print("*");
  Schermo.setCursor(13,0);
  Schermo.print("*");
  Schermo.setCursor(14,0);
  Schermo.print("*");
  Schermo.setCursor(15,0);
  Schermo.print("*");
  Schermo.setCursor(16,0);
  Schermo.print("*");
  Schermo.setCursor(17,0);
  Schermo.print("*");
  Schermo.setCursor(18,0);
  Schermo.print("*");
  Schermo.setCursor(19,0);
  Schermo.print("*");

  // ULTIMA RIGA IN BASSO DEL DISPLY LCD
  Schermo.setCursor(0,3);
  Schermo.print("*");
  Schermo.setCursor(1,3);
  Schermo.print("*");
  Schermo.setCursor(2,3);
  Schermo.print("*");
  Schermo.setCursor(3,3);
  Schermo.print("*");
  Schermo.setCursor(4,3);
  Schermo.print("*");
  Schermo.setCursor(5,3);
  Schermo.print("*");
  Schermo.setCursor(6,3);
  Schermo.print("*");
  Schermo.setCursor(7,3);
  Schermo.print("*");
  Schermo.setCursor(8,3);
  Schermo.print("*");
  Schermo.setCursor(9,3);
  Schermo.print("*");
  Schermo.setCursor(10,3);
  Schermo.print("*");
  Schermo.setCursor(11,3);
  Schermo.print("*");
  Schermo.setCursor(12,3);
  Schermo.print("*");
  Schermo.setCursor(13,3);
  Schermo.print("*");
  Schermo.setCursor(14,3);
  Schermo.print("*");
  Schermo.setCursor(15,3);
  Schermo.print("*");
  Schermo.setCursor(16,3);
  Schermo.print("*");
  Schermo.setCursor(17,3);
  Schermo.print("*");
  Schermo.setCursor(18,3);
  Schermo.print("*");
  Schermo.setCursor(19,3);
  Schermo.print("*");

  // RIGA LATO SINISTRO
  Schermo.setCursor(0,1);
  Schermo.print("*");
  Schermo.setCursor(0,2);
  Schermo.print("*");

  // RIGA LATO DESTRO
  Schermo.setCursor(19,1);
  Schermo.print("*");
  Schermo.setCursor(19,2);
  Schermo.print("*");
}


/*
 * DI SEGUITO TUTT LE FUNZIONI LEGATE AL MODULO GSM
 */

/*void SIM900power() {

  digitalWrite(SET, HIGH);
  delay(1000);
  digitalWrite(SET, LOW);
  delay(1000);
}*/


/*void Impostazioni_GSM() {

  Serial2.println("AT+CSQ"); //Signal quality test, value range is 0-31 , 31 is the best
  updateSerial();

  Serial2.println("AT+CCID"); //Read SIM information to confirm whether the SIM is plugged
  updateSerial();

  Serial2.println("AT+CREG?"); //Check whether it has registered in the network
  updateSerial();

  Serial2.println("AT+CMGF=1"); // Configuring TEXT mode
  updateSerial();

  Serial2.println("AT+CNMI=1,2,0,0,0"); // Decides how newly arrived SMS messages should be handled
  updateSerial();
}*/


/*void updateSerial() {

  //vTaskDelay(pdMS_TO_TICKS(1000));

  while (Serial.available())
  {
    Serial2.write(Serial.read());//Forward what Serial received to Software Serial Port
  }
  while(Serial2.available())
  {
    Serial.println(Serial2.read());//Forward what Software Serial received to Serial Port
  }
}*/


/*void SendMessage_Lorenzo() {

  Serial2.println("AT+CMGF=1");    //Sets the GSM Module in Text Mode
  delay(1000);  // Delay of 1000 milli seconds or 1 second

  Serial2.println("AT+CMGS=\"+393275548933\"\r"); // Replace x with mobile number
  delay(1000);

  Serial2.println("ALLARME SCATTATO - CASA URBANI");// The SMS text you want to send
  delay(100);

  Serial2.println((char)26);// ASCII code of CTRL+Z
  delay(1000);
}*/


/*void SendMessage_Mamma_Lavoro() {

  Serial3.println("AT+CMGF=1");    //Sets the GSM Module in Text Mode
  //vTaskDelay(pdMS_TO_TICKS(1000));  // Delay of 1000 milli seconds or 1 second

  Serial3.println("AT+CMGS=\"+393487746691\"\r"); // Replace x with mobile number
  //vTaskDelay(pdMS_TO_TICKS(1000));

  Serial3.println("ALLARME SCATTATO - CASA URBANI");// The SMS text you want to send
  //vTaskDelay(pdMS_TO_TICKS(100));

  Serial3.println((char)26);// ASCII code of CTRL+Z
  //vTaskDelay(pdMS_TO_TICKS(1000));
}*/


/*void SendMessage_Mamma_Privato() {

  Serial3.println("AT+CMGF=1");    //Sets the GSM Module in Text Mode
  //vTaskDelay(pdMS_TO_TICKS(1000));  // Delay of 1000 milli seconds or 1 second

  Serial3.println("AT+CMGS=\"+393483020148\"\r"); // Replace x with mobile number
  //vTaskDelay(pdMS_TO_TICKS(1000));

  Serial3.println("ALLARME SCATTATO - CASA URBANI");// The SMS text you want to send
  //vTaskDelay(pdMS_TO_TICKS(100));

  Serial3.println((char)26);// ASCII code of CTRL+Z
  //vTaskDelay(pdMS_TO_TICKS(1000));
}*/


/*void SendMessage_Rocco() {

  Serial3.println("AT+CMGF=1");    //Sets the GSM Module in Text Mode
  vTaskDelay(pdMS_TO_TICKS(1000));  // Delay of 1000 milli seconds or 1 second

  Serial3.println("AT+CMGS=\"+393351219483\"\r"); // Replace x with mobile number
  vTaskDelay(pdMS_TO_TICKS(1000));

  Serial3.println("ALLARME SCATTATO - CASA URBANI");// The SMS text you want to send
  vTaskDelay(pdMS_TO_TICKS(100));

  Serial3.println((char)26);// ASCII code of CTRL+Z
  vTaskDelay(pdMS_TO_TICKS(1000));
}*/
