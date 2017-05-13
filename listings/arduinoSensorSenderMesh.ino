//Libraries
#include <DHT.h>       //Temperatur und Luftfeuchtigkeit
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <LowPower.h>
#include <EEPROM.h>
#include <AESLib.h>
#include <math.h>
#include <Base64.h>
#include <printf.h>
#include <BH1750.h>
#include <OneWire.h>
#include <DallasTemperature.h>

//Konstanten und Variablen

  //ID des Arduinos
  unsigned int arduinoId = 9999;

  //Energiesparen
  #define ENERGY_SAVING true 
  #define SLEEP_TIME 8 //mal 8s
  #define RADIO_RETRY_ATTEMPTS 7
  #define DEBUG_AUSGABE true

  //DHT22
  #define DHTPIN 4     // what pin we're connected to
  #define DHTTYPE DHT11  // DHT 22  (AM2302)
  DHT dht(DHTPIN, DHTTYPE); //// Initialize DHT sensor for normal 16mhz Arduino
  float humidity;
  float temperatur;
  
  BH1750 LightSensor;
  float lux;
    
  //BPM 180
  Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
  int pressure;
  boolean bpm180Connected;

  //Bewegungsmelder
  #define INTERRUPT_PIN_MOTION_DETECT 3
  boolean motion;
  boolean firstTimeMotion;

  //Bodenfeuchtigkeit
  boolean soilMoistureDigital;
  #define soilMoistureDigitalPin 5
  #define soilMoistureAnalogPin 6
  float soilMoistureAnalog;

  //OneWire
  #define ONE_WIRE_BUS 6
  OneWire oneWire(ONE_WIRE_BUS);
  DallasTemperature oneWireDallas(&oneWire);

  //Reed Kontakt kein Interrupt
  #define REED_CONTACT_NOINT_PIN 7
  boolean reedContactNoInterrupt;
  boolean firstTimeConntectNoInt;

  //Reed Kontakt mit Interrupt
  #define REED_CONTACT_WITHINT_PIN 2
  boolean firstTimeConntectWithInt;
  boolean reedContactWithInterrupt;
  boolean interruptHappendReedContact;



  // NRF24L01
  RF24 radio(9,10);
  uint8_t addresses[][6] = {"1Node"}; //Pipe

   
  unsigned long timeId;
  long addressTimeId;

  unsigned int messageId;
  int addressMessageId;

  #define SIZE_OF_BLACKLIST 100
  unsigned long blacklist[SIZE_OF_BLACKLIST];
  int listPointer = 0;
  

  //Sensor Daten
  struct sensorData{
    unsigned  int id;
    float value;
    int unit;
    unsigned long timeId;
  };

  //Mesh Daten Paket
  struct dataPacket{
    unsigned int destinationAddr;
    unsigned int originAddr;
    unsigned int lastHopAddr;
    unsigned int messageId;
    sensorData data; 
  };
  //Mesh Daten Paket
  struct dataPacketEncoded{
    char message[28];
  };


  //
  long startTime; // millis-Wert beim ersten Drücken der Taste
  long duration;  // Variable für die Dauer

  //Normalbetrieb kein Energiesparmodus
  int countTimer = 0; 
  boolean interruptHappened;

  
void setup() { 
  #if DEBUG_AUSGABE 
    Serial.begin(9600);  
  #endif
/*
  EEPROMWriteInt(12, 100);
  delay(500);
*/
  interruptHappened = false; 
  
  arduinoId = EEPROMReadInt(12);
  delay(500);

  
  //DHT22
  dht.begin();
  lux = NULL;

  //BMP 180
  if(!bmp.begin())
  {
    bpm180Connected = false; 
  }else{
    bpm180Connected = true;
  }
  
  //Bewegungsmelder
  motion = false;
  firstTimeMotion = false;

  //Bodenfeuchtigkeit
   pinMode(soilMoistureDigitalPin, INPUT); 
   soilMoistureAnalog = 0;

  //OneWire
  oneWire.reset();
  oneWireDallas.begin();

  //Reed Kontakt
  firstTimeConntectNoInt = false;
  firstTimeConntectWithInt = false;
  interruptHappendReedContact = false;
  pinMode(REED_CONTACT_WITHINT_PIN, INPUT);

  #if DEBUG_AUSGABE 
    Serial.println("init Radio");
  #endif

  initRadio();
  
  //EEPROM komplett loeschen
  /*
   for (int i = 0 ; i < EEPROM.length() ; i++) {
    EEPROM.write(i, 0);
   } 
   */
  
  addressTimeId = 1;
  timeId = EEPROMReadlong(addressTimeId);
  delay(1000);
  addressMessageId = 8;
  messageId = (int) EEPROMReadInt(addressMessageId);
  
  #if DEBUG_AUSGABE 
    Serial.print("timeId ID: \t");
    Serial.println(timeId);
    Serial.print("Message ID: \t");
    Serial.println(messageId);
  #endif

  #if !ENERGY_SAVING 
    collectAndSendSensorData();
    radio.startListening();
  
    // initialize timer1 
    noInterrupts();           // disable all interrupts
    TCCR1A = 0;
    TCCR1B = 0;
  
    TCNT1 = 0;            // preload timer 65536-16MHz/256/2Hz
    TCCR1B |= (1 << CS12);    // 256 prescaler 
    TCCR1B |= (1 << CS10); 
    TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
    interrupts();             // enable all interrupts
  #endif
  
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN_MOTION_DETECT), interruptMotionDetector, RISING);
  attachInterrupt(digitalPinToInterrupt(REED_CONTACT_WITHINT_PIN), interruptContactDetector, CHANGE);
    
}


ISR(TIMER1_OVF_vect)          // timer compare interrupt service routine
{
  if(countTimer <= 6){
    TCNT1 = 0; 
    countTimer++;
  }else{
    // Serial.print("INTERRUPT: ");
    //  duration = millis() - startTime;
    //Serial.println(duration);
    
    TCNT1 = 0; 
    countTimer = 0;  
    interruptHappened = true;
    startTime = millis(); 
    
  }
}

void interruptMotionDetector(){  
  motion = true;
  firstTimeMotion = true;
}
void interruptContactDetector(){
  
  boolean firstRead = digitalRead(REED_CONTACT_WITHINT_PIN);
  delay(10);
  boolean secondRead = digitalRead(REED_CONTACT_WITHINT_PIN);
  
  if(firstRead == secondRead){
    interruptHappendReedContact = true;
    detachInterrupt(digitalPinToInterrupt(REED_CONTACT_WITHINT_PIN));   
    reedContactWithInterrupt = firstRead;
    firstTimeConntectWithInt = true;
  }
}



void loop() {
    #if ENERGY_SAVING 
        #if DEBUG_AUSGABE 
            long start;
            long dauer;
            start = millis();
        #endif
                
        radio.powerUp();
        collectAndSendSensorData();
        radio.powerDown();
        
        #if DEBUG_AUSGABE 
           dauer = millis() - start;
           Serial.print("Dauer: ");
           Serial.print(dauer);
           Serial.println(" ms");
           delay(400);
        #else
           delay(15);
        #endif
        
        for(int i = 0; i < SLEEP_TIME; i++)
        {
         LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
        }
            
    #else
        dataPacket t_DataPacket_Loop;
        dataPacketEncoded t_dataPacketEncoded;
        
       if(interruptHappened){
        //radio.stopListening();
        collectAndSendSensorData();
        interruptHappened = false;
       }
        
        while( radio.available()){
          radio.read( &t_dataPacketEncoded.message, sizeof(t_dataPacketEncoded.message) );
          t_DataPacket_Loop = decodingData(t_dataPacketEncoded);
          #if DEBUG_AUSGABE 
            Serial.print("Paket erhalten: \t");
            ausgabeDataPacket(t_DataPacket_Loop);
          #endif
          processData(t_DataPacket_Loop);
        }
   #endif
}

void collectAndSendSensorData(){
  #if !ENERGY_SAVING 
    radio.stopListening();
    delay(100);
  #endif
  
  timeId++;
  EEPROMWritelong(addressTimeId, timeId);
  
  getTemperatureHumidty();
  sendDataPacket(createSensorDataPacket(temperatur, 1));
  sendDataPacket(createSensorDataPacket(humidity, 2));
  
  getLightIntensity();
  if(54612 != lux){
    sendDataPacket(createSensorDataPacket(lux, 3));
  }
  if(true == bpm180Connected){
    getPressure();
    sendDataPacket(createSensorDataPacket(pressure, 4));
  }
  if(true == firstTimeMotion){
    sendDataPacket(createSensorDataPacket(getAndResetMotionSensor(), 5));
  }
  getSoilMoisture();
  if(soilMoistureAnalog){
    sendDataPacket(createSensorDataPacket(soilMoistureAnalog, 6));
    sendDataPacket(createSensorDataPacket(soilMoistureDigital, 7));
  }
  getOneWireTemperature();
  for(int i = 100; i < oneWireDallas.getDeviceCount() + 100 && i <= 199 ;i++){
    sendDataPacket(createSensorDataPacket(oneWireDallas.getTempCByIndex(i-100), (i) ));
  }
  
  getReedContactNoInt();
  if(false == firstTimeConntectNoInt){
    sendDataPacket(createSensorDataPacket(reedContactNoInterrupt, 8));
  }
  if (true == firstTimeConntectWithInt){
    sendDataPacket(createSensorDataPacket(getAndResetReedContactSensor(), 9));
  }
  
  #if !ENERGY_SAVING
    delay(100);
    radio.startListening();
  #endif
}

void getTemperatureHumidty(){  
  humidity = dht.readHumidity(); 
  temperatur = dht.readTemperature();  
}
void getLightIntensity(){  
  LightSensor.begin(BH1750_ONE_TIME_HIGH_RES_MODE);
  delay(200);
  lux = LightSensor.readLightLevel(); 
  LightSensor.powerDown();
}
boolean getAndResetMotionSensor(){ 
  boolean motionSensor = motion;
  motion = false;
  return motionSensor;
}

void getPressure(){
  sensors_event_t event;
  bmp.getEvent(&event);
  pressure = event.pressure; 
}
void getSoilMoisture(){  
  soilMoistureAnalog = analogRead(soilMoistureAnalogPin); 
  soilMoistureDigital = digitalRead(soilMoistureDigitalPin);  
}
void getOneWireTemperature(){
  oneWireDallas.requestTemperatures();
  delay(10);
}
void getReedContactNoInt(){  
   reedContactNoInterrupt = digitalRead(REED_CONTACT_NOINT_PIN);
   if(false == reedContactNoInterrupt){
    firstTimeConntectNoInt = true;  
   }
}
boolean getAndResetReedContactSensor(){ 
  if(interruptHappendReedContact){
    interruptHappendReedContact = false;
    attachInterrupt(digitalPinToInterrupt(REED_CONTACT_WITHINT_PIN), interruptContactDetector, CHANGE);
  }
  return reedContactWithInterrupt;
}


dataPacket createSensorDataPacket(float value, int unit){

  //Sensor Daten uebernehmen
  sensorData t_sensorData;

  t_sensorData.timeId = timeId;
  t_sensorData.id = arduinoId;
  t_sensorData.value = value;
  t_sensorData.unit = unit;

  //Packet erstellen
  dataPacket t_dataPacket;
  t_dataPacket.destinationAddr = 1;
  t_dataPacket.originAddr = arduinoId;
  messageId++;
  EEPROMWriteInt(addressMessageId, messageId);
  t_dataPacket.messageId = messageId;
  t_dataPacket.lastHopAddr = 0;
  t_dataPacket.data = t_sensorData; 
  
  return t_dataPacket;
}

void sendDataPacket(dataPacket t_dataPacket){
  #if DEBUG_AUSGABE 
    Serial.print("Paket senden: \t \t");
    ausgabeDataPacket(t_dataPacket);
  #endif
  
  dataPacketEncoded encodedMessage;
  encodedMessage = encodiereDaten(t_dataPacket);
  radio.SensorknotenIoT_resetRegister();

  radio.write( &encodedMessage.message, sizeof(encodedMessage.message));
  for(int retry = 0; retry <= RADIO_RETRY_ATTEMPTS; retry++){
    radio.write( &encodedMessage.message, sizeof(encodedMessage.message));
    //radio.reUseTX();
   delayMicroseconds(160);
  } 

   
}

dataPacketEncoded encodiereDaten(dataPacket  t_dataPacket){
  char encoded[base64_enc_len(sizeof(t_dataPacket))];
  char dataToEncode[sizeof(t_dataPacket)]; 
  memcpy(&dataToEncode, &t_dataPacket, sizeof(t_dataPacket)); 
  base64_encode(encoded, dataToEncode, sizeof(dataToEncode));
  dataPacketEncoded t_dataPacketEncoded;
  memcpy(&t_dataPacketEncoded.message, &encoded, sizeof(encoded));
  return t_dataPacketEncoded;
}


dataPacket decodingData(dataPacketEncoded  t_dataPacketEncoded){
  int input2Len = sizeof(t_dataPacketEncoded.message);
  int decodedLen = base64_dec_len(t_dataPacketEncoded.message, input2Len);
  char decoded[decodedLen];
  base64_decode(decoded, t_dataPacketEncoded.message, input2Len);
  dataPacket  t_dataPacket;
  memcpy(&t_dataPacket, &decoded, sizeof(decoded));
  return t_dataPacket;
}

void processData(dataPacket t_dataPacket){
  if(!(t_dataPacket.destinationAddr == arduinoId)){
     unsigned long uniqueMessageId = 
	 (unsigned long) ((unsigned long) t_dataPacket.originAddr << 16)
	 +  (unsigned long) t_dataPacket.messageId;
     
     if(!compareToList(uniqueMessageId)){
         insertInList(uniqueMessageId);
         t_dataPacket.lastHopAddr = arduinoId;
         sendDataPacket(t_dataPacket);
     }else{
        #if DEBUG_AUSGABE 
          //Message verwefen
          Serial.println("Message verworfen");
        #endif
     }    
  }else{
    #if DEBUG_AUSGABE 
      //Arduino ist Empfaenger
      Serial.print("Arduino ist Empfaenger: \t");
      ausgabeDataPacket(t_dataPacket);
    #endif
  }
}

void decrementPoiter() {
    if(listPointer < SIZE_OF_BLACKLIST )
        listPointer++;
    else
        listPointer=0;       
}

boolean compareToList(unsigned long uniqueMessageId){
   for (int i=0; i<  (SIZE_OF_BLACKLIST - 1); i++) {
        if (uniqueMessageId==blacklist[i])
            return true;
    }
  return false;
  
}
void insertInList(unsigned long uniqueMessageId){
    blacklist[listPointer]= uniqueMessageId;
    decrementPoiter();
}


void EEPROMWritelong(int address, long value){
      //Decomposition from a long to 4 bytes by using bitshift.
      //One = Most significant -> Four = Least significant byte
      byte four = (value & 0xFF);
      byte three = ((value >> 8) & 0xFF);
      byte two = ((value >> 16) & 0xFF);
      byte one = ((value >> 24) & 0xFF);

      //Write the 4 bytes into the eeprom memory.
      EEPROM.write(address, four);
      EEPROM.write(address + 1, three);
      EEPROM.write(address + 2, two);
      EEPROM.write(address + 3, one);
}

long EEPROMReadlong(int address)
{
      //Read the 4 bytes from the eeprom memory.
      long four = EEPROM.read(address);
      long three = EEPROM.read(address + 1);
      long two = EEPROM.read(address + 2);
      long one = EEPROM.read(address + 3);

      //Return the recomposed long by using bitshift.
      return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
}
void EEPROMWriteInt(int address, int value){
      //Decomposition from a long to 4 bytes by using bitshift.
      //One = Most significant -> Four = Least significant byte
      byte two = (value & 0xFF);
      byte one = ((value >> 8) & 0xFF);

      //Write the 4 bytes into the eeprom memory.
      EEPROM.write(address, two);
      EEPROM.write(address + 1, one);
}

int EEPROMReadInt(int address)
{
      //Read the 4 bytes from the eeprom memory.
      long four = EEPROM.read(address);
      long three = EEPROM.read(address + 1);

      //Return the recomposed long by using bitshift.
      return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF);
}



void serielleAusgabe(){
    Serial.print("Luftfeuchte: ");
    Serial.print(humidity);
    Serial.print(" %\t");
    Serial.print("Temperatur: ");
    Serial.print(temperatur);
    Serial.print(" C\t");
    Serial.print("Pressure: ");
    Serial.print(pressure);
    Serial.print(" hPa\t");
    Serial.print(" TimeId: ");
    Serial.println(timeId);
    
}


void ausgabeSensorData(sensorData t_sensorData){
        Serial.print("ID: ");
        Serial.print(t_sensorData.id);
        Serial.print(" \t");
        Serial.print("Value: ");
        Serial.print(t_sensorData.value);
        Serial.print(" \t");
        Serial.print("Unit: ");
        Serial.print(t_sensorData.unit);// Get the payload
        Serial.print(" \t");
        Serial.print("TimeId: ");
        Serial.println(t_sensorData.timeId);// Get the payload
}
void ausgabeDataPacket(dataPacket t_dataPacket){
    Serial.print("Destination Addr: ");
    Serial.print(t_dataPacket.destinationAddr);
    Serial.print(" \t");
    Serial.print("Origin Addr: ");
    Serial.print(t_dataPacket.originAddr);
    Serial.print(" \t");
    Serial.print("Last Hop Addr: ");
    Serial.print(t_dataPacket.lastHopAddr);// Get the payload
    Serial.print(" \t");
    Serial.print("MessageID: ");
    Serial.print(t_dataPacket.messageId);// Get the payload
    Serial.print(" \t");
    ausgabeSensorData(t_dataPacket.data);
}
 void initRadio(){
      radio.begin();
      radio.setPayloadSize(base64_enc_len(sizeof(dataPacket)));  //Groesse der gesendeten Daten
      radio.enableDynamicPayloads();
      radio.setAutoAck(true); 
      radio.setDataRate(RF24_250KBPS); //250kbs
      radio.setPALevel(RF24_PA_HIGH);
      radio.setChannel(90);
      radio.setRetries(15,15);
      radio.setCRCLength(RF24_CRC_16);
      radio.openWritingPipe(addresses[0]);
      radio.openReadingPipe(1,addresses[0]); //1
      #if DEBUG_AUSGABE 
        printf_begin();
        radio.printDetails();
      #endif
  }










