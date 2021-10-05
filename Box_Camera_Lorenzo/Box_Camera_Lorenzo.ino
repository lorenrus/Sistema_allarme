#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <SPI.h>


#define STATO_ALLARME 0
#define STATO_ALIMENTAZIONE 1

#define SENSORE_REED_ESTERNO          2
#define SENSORE_REED_INTERNO          3


#define LED_VERDE_ALLARME_DISATTIVO   A0       
#define LED_ROSSO_ALLARME_ATTIVO      A1 


#define MAX_BATTERY_VOLTAGE     5
#define BATTERY_CHARGE_SWITCH   6


#define SET_HC12 8


struct HC12 {
  
  int HC12_Command_Receive_from_UC = 0;

  bool Enable_Receive_HC12 = true;
};

struct HC12 Modulo_HC12;


struct Variabili_Sistema_Allarme {

  uint8_t Box_Camera_Lorenzo = 1;
  
  uint8_t Allarm_Activated = 11;
  uint8_t Allarm_Disactivated = 0;
  
  uint8_t Allarm_Status;
  
  uint8_t SI_Effrazione = 11;
  uint8_t NO_Effrazione = 0;
  
  uint8_t Controllo_Riavvio_Sistema;

  bool Func_HC12 = false;
};

struct Variabili_Sistema_Allarme Sistema_Allarme;


SoftwareSerial HC12(10, 11); // HC-12 TX Pin, HC-12 RX Pin

bool Enable_ISR_Func_From_ISR_Reed_Interno = false;
bool Enable_ISR_Func_From_ISR_Reed_Esterno = false;


void setup() {

  Serial.begin(115200);
  HC12.begin(9600);               


  pinMode(SET_HC12, OUTPUT);
  digitalWrite(SET_HC12, HIGH);


  pinMode(SENSORE_REED_ESTERNO, INPUT);
  pinMode(SENSORE_REED_INTERNO, INPUT);

  
  pinMode(LED_VERDE_ALLARME_DISATTIVO, OUTPUT);
  pinMode(LED_ROSSO_ALLARME_ATTIVO, OUTPUT);


  pinMode(MAX_BATTERY_VOLTAGE , INPUT);
  pinMode(BATTERY_CHARGE_SWITCH, OUTPUT);

  // Da commentare quando si fanno le prove programmando arduino
  // altrimenti ogni volta che lo programmiamo che lui si riavvia entrerà sempre nell'if
  // dei controllo di questa variabile
  Sistema_Allarme.Controllo_Riavvio_Sistema = 1;
  EEPROM.write(STATO_ALIMENTAZIONE, Sistema_Allarme.Controllo_Riavvio_Sistema);
}


void loop() {

  MainFunction();
  
  Reed_Sensor();
}


void ISR_Func_Reed_Interno() {

  Enable_ISR_Func_From_ISR_Reed_Interno = true;

  delay(40);

  HC12.write(Sistema_Allarme.SI_Effrazione);

  delay(10);
}


void ISR_Func_Reed_Esterno() {

  Enable_ISR_Func_From_ISR_Reed_Esterno = true;

  delay(40);

  HC12.write(Sistema_Allarme.SI_Effrazione);

  delay(10);
}


void Reed_Sensor() {

  /* Box Camera Lorenzo */

  if(Enable_ISR_Func_From_ISR_Reed_Interno) {

    delay(10);

    detachInterrupt(digitalPinToInterrupt(SENSORE_REED_INTERNO));

    Enable_ISR_Func_From_ISR_Reed_Interno = false;
  }
  else if(Enable_ISR_Func_From_ISR_Reed_Esterno) {

    delay(10);

    detachInterrupt(digitalPinToInterrupt(SENSORE_REED_ESTERNO));

    Enable_ISR_Func_From_ISR_Reed_Esterno = false;
  }
}


/*void Read_Voltage_Battery() {

  while(1) {

    if(digitalRead(MAX_BATTERY_VOLTAGE) == HIGH) { // LA BATTERIA è CARICA

      digitalWrite(BATTERY_CHARGE_SWITCH, HIGH); // Non stiamo ricaricando la batteria
    }
    else { // LA TENSIONE DELLA BATTERIA E' SCESA AL DI SOTTO DELLA SOGLIA

      digitalWrite(BATTERY_CHARGE_SWITCH, LOW); // Stiamo ricaricando la batteria
    }
  }
}*/


void MainFunction() {
  
  if(HC12.available() && Modulo_HC12.Enable_Receive_HC12) {
        
    Modulo_HC12.HC12_Command_Receive_from_UC = HC12.read();

    Serial.print("PRIMO : ");
    Serial.println(Modulo_HC12.HC12_Command_Receive_from_UC);

    if(Modulo_HC12.HC12_Command_Receive_from_UC == Sistema_Allarme.Box_Camera_Lorenzo) {

      Serial.print("SECONDO : ");
      Serial.println(Modulo_HC12.HC12_Command_Receive_from_UC);

      delay(40);
        
      HC12.write(Sistema_Allarme.Box_Camera_Lorenzo);

      delay(10);

      Sistema_Allarme.Func_HC12 = true;

      Modulo_HC12.Enable_Receive_HC12 = false;
    }
  }

  while(Sistema_Allarme.Func_HC12) {

    if(HC12.available()) {

      Sistema_Allarme.Allarm_Status = HC12.read();

      Serial.print("TERZO : ");
      Serial.println(Sistema_Allarme.Allarm_Status);

      EEPROM.write(STATO_ALLARME, Sistema_Allarme.Allarm_Status);

      Check_Allarm_Status();

      Sistema_Allarme.Func_HC12 = false;

      Modulo_HC12.Enable_Receive_HC12 = true;
    }
  }
       
  if(EEPROM.read(STATO_ALIMENTAZIONE) == 1) {

    Serial.println("RIAVVIATO PER MANCATA ALIMENTAZIONE");
       
    if(EEPROM.read(STATO_ALLARME) == Sistema_Allarme.Allarm_Disactivated) {  // Sistema di allarme disattivato

        Serial.println("ALLARME DISATTIVATO");

        digitalWrite(LED_ROSSO_ALLARME_ATTIVO, LOW);

        digitalWrite(LED_VERDE_ALLARME_DISATTIVO, HIGH);
        
        detachInterrupt(digitalPinToInterrupt(SENSORE_REED_INTERNO));
        detachInterrupt(digitalPinToInterrupt(SENSORE_REED_ESTERNO));
      }
      else if(EEPROM.read(STATO_ALLARME) == Sistema_Allarme.Allarm_Activated) {   // Sistema di allarme attivato

        Serial.println("ALLARME ATTIVATO");

        digitalWrite(LED_ROSSO_ALLARME_ATTIVO, HIGH);

        digitalWrite(LED_VERDE_ALLARME_DISATTIVO, LOW);
        
        attachInterrupt(digitalPinToInterrupt(SENSORE_REED_INTERNO), ISR_Func_Reed_Interno, HIGH);
        attachInterrupt(digitalPinToInterrupt(SENSORE_REED_ESTERNO), ISR_Func_Reed_Esterno, HIGH);
      }

      Sistema_Allarme.Controllo_Riavvio_Sistema = 0;
      EEPROM.write(STATO_ALIMENTAZIONE, Sistema_Allarme.Controllo_Riavvio_Sistema);
  }
}


void Check_Allarm_Status() {

  if(EEPROM.read(STATO_ALLARME) == Sistema_Allarme.Allarm_Disactivated) {  // Sistema di allarme disattivato

//    Serial.println("ALLARME DISATTIVATO");

    digitalWrite(LED_ROSSO_ALLARME_ATTIVO, LOW);

    digitalWrite(LED_VERDE_ALLARME_DISATTIVO, HIGH);
        
    detachInterrupt(digitalPinToInterrupt(SENSORE_REED_INTERNO));
    detachInterrupt(digitalPinToInterrupt(SENSORE_REED_ESTERNO));
  }
  else if(EEPROM.read(STATO_ALLARME) == Sistema_Allarme.Allarm_Activated) {   // Sistema di allarme attivato

//    Serial.println("ALLARME ATTIVATO");

    digitalWrite(LED_ROSSO_ALLARME_ATTIVO, HIGH);

    digitalWrite(LED_VERDE_ALLARME_DISATTIVO, LOW);
        
    attachInterrupt(digitalPinToInterrupt(SENSORE_REED_INTERNO), ISR_Func_Reed_Interno, LOW);
    attachInterrupt(digitalPinToInterrupt(SENSORE_REED_ESTERNO), ISR_Func_Reed_Esterno, LOW);
  }
}
