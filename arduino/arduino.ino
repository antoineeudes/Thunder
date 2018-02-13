//CONSTANTES
const int FLASH_IMPULSE_DELAY = 10; //Durée (en ms) de l'impulsion donnée au stroboscope

int incomingByte = 0;

void setup() {
  //Connexion série
  Serial.begin(9600);

  //LED
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

}

void loop() {

    if(Serial.available() > 0)
    {
        //Pourra être utile pour distinguer quel projecteur allumer
        //(Switch sur incomingByte)
        incomingByte = Serial.read();   //Purge également les données en attente

        flash();
    }
}

void flash() {
    //Déclenche le flash du stroboscope
    digitalWrite(13, HIGH);
    delay(FLASH_IMPULSE_DELAY);
    digitalWrite(13, LOW);
}
