/*
 * Ce code est destiné à être utilisé sur un MKRFOX1200. Arduino intégrant le module SigFox.
 * Pour faire fonctionner ce code, il faut :  
 *   - Un module GPS permettant de sortir des trames NMEA GMRMC
 *   - Une alimentation électrique 5V ou 3.3V
 *   - Deux résistances de valeurs équivalentes pour faire un pont diviseur (cf : https://fr.wikipedia.org/wiki/Diviseur_de_tension)
 * 
 * Récupération des trames GPS : 
 *   La sortie (TX) du GPS doit être reliée à l'entrée série de l'arduino (PIN 13 : RX)
 *   Une fois sous tension, le GPS envoie les trames au port série, il reste donc à les lire et les traiter.
 *   Tant qu'aucune donnée n'est reçu, la lecture du port série retourne -1.
 *   Sinon, les caractères sont lus bit à bit. Le programme ajoute les informations reçues dans une chaîne de caractères (linea)
 *   jusqu'à obtenir une fin de ligne <CR><LF> : 0x13 0x10 - Une protection est ajoutée au cas où la chaîne dépasserait le buffer (limité à 300 caractères).
 *   Une fois le caractère de fin de ligne obtenu :
 *     * On vérifie que la trame commence par GPRMC
 *     * On recherche les virgules et l'étoile pour connaître la position des différents éléments
 *     * On vérifie le checksum en comparant celui de la trace avec celui que l'on calcule
 *     * On vérifie qu'il s'agit d'une trame de position valide (Si le GPS n'a pas suffisemment de satellites pour avoir la position il marque la trame non valide)
 *     * On extrait les différentes données pour les mettre dans une structure spécifique pour l'envoi SigFox
 * 
 * Lecture du voltage :
 *   Le pont diviseur permet de réduire la valeur du voltage à lire, mais n'est pas strictement nécessaire.
 *   Le voltage est récupéré sur la PIN A0 de l'arduino.
 *   Le code lit la valeur (comprise entre 0 et 1023 - où 1023 est la valeur max donc 3.3V sur un MRKFOX1200).
 *   La valeur récupérée est ensuite ajoutée à la structure SigFox
 *   
 * Envoi des informations :
 *   Les informations sont envoyées par la fonction SendSigFox() de façon standard.
 *   Si le module retourne une erreur d'envoi, on essaye 5 fois et on arrête
 *   
 * Sources :
 *  Igor Gonzalez Martin. 05-04-2007 - http://forum.arduino.cc/index.php?topic=150925.0
 *  Arnaud Boudou - https://github.com/aboudou/SmartEverything_SigFox_GPS 
 *  Arduino - https://www.arduino.cc/en/Tutorial/ReadAnalogVoltage 
 *  
 *  * Official repository for this code : https://github.com/jmalghem/mkrfox1200_with_gps
 * 
 * Feel free to clone it, modify it, improve it… The following code is licensed under the BSD 2-Clauses License.
 * 
 * Copyright (c) 2017, Julien Malghem
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this list 
 * of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list 
 * of conditions and the following disclaimer in the documentation and/or other materials 
 * provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS 
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY 
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER 
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT 
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 *  
*/
 
 
 #include <ctype.h>
 #include "SigFox.h"
 // mode debug pour sortir du log
#define DEBUG true     

 unsigned long triggerInterval = 3600000; // Interval entre deux réveils = 2 minutes
 int deviceid = 7; // Numéro du device ou toute autre info sur 4 octets
 //unsigned long triggerInterval = 120000; // Interval entre deux réveils = 2 minutes
 unsigned long killTimer = 60000; // Temps limite pour avoir une position valide = 1 minute
 unsigned long previousInterval = 0;
 unsigned long currentMillis = 0; // Va prendre la valeur du nombre de millisecondes depuis le démarrage de l'arduino
 char comandoGPR[7] = "$GPRMC"; // Filtre de la ligne
 int byteGPS=-1; // Récupère les valeurs reçues par le port série (donc le GPS)
 char linea[300] = ""; // Ligne à traiter
 int cont=0;
 int bien=0;
 int conta=0;
 int indices[14]; // Enregistre la position des virgules dans la chaine de caractères à traiter
 boolean startLoop = true; // permet de lancer la boucle de recherche de la position au démarrage et gérer son état ensuite

 typedef struct __attribute__ ((packed)) sigfox_message {
  float latitude;
  float longitude;
  uint16_t deviceid;
  uint16_t voltage;
 } SigfoxMessage;
 
SigfoxMessage msg;

void setup() {
  // initialisation des ports série
  //Serial.begin(9600); // Sortie écran
  Serial1.begin(9600); // GPS (PIN 13)
  
  // Initialisation du buffer de réception
  for (int i=0;i<300;i++){       
     linea[i]=' ';
   }   
  // Initialisation du module SigFox
  if (!SigFox.begin()) {
    //Serial.println("gros soucis on reboot");
    //reboot;
  }

  if (DEBUG) {
    // Enable DEBUG prints and LED indication if we are testing
    SigFox.debug();
  }

  // On passe le module en standby
  SigFox.end();

  // On attends 5 secondes que tout soit initialisé
  delay(5000);
}

void loop() {
  currentMillis = millis(); // Récup le temps courant
  if ((unsigned long)(currentMillis - previousInterval) <= triggerInterval) {
    if (startLoop) {
      loopPosition(currentMillis + killTimer);
      startLoop = false;
    }
  } else {
    // Le triggerInterval est écoulé, on peut autoriser la boucle à se relancer
    previousInterval = millis(); // On récupère l'heure du dernier lancement
    startLoop = true; // On autorise le lancement de la boucle
    delay(1); // On ajoute un retardateur de 1ms sinon il relance la boucle tout de suite (currentMillis = previousInterval)
  }
}

void loopPosition(int endDate) {
  do {
    if(!startLoop) { break; } // Si on a récupéré une position valide alors on arrête la récup des données en sortant de la boucle
  } while (checkPosition() < endDate); // Si on est pas sorti plus tôt, on sort après le killTimer
}

unsigned long checkPosition() {
     byteGPS=Serial1.read();         // Read a byte of the serial port
   if (byteGPS == -1) {           // See if the port is empty yet
     delay(50); 
   } else {
     int res = conta < 300 ? linea[conta]=byteGPS : byteGPS = 13; // Vérifie que les données reçues ne dépassent pas le buffer
     conta++;                      
     // Serial.write(byteGPS); 
     if (byteGPS==13){            // On recherche une fin de ligne (0x13)
       cont=0;
       bien=0;
       // La boucle commence à 1 car le premier caractère est <LF> (0x10) qui suit le 0x13 de la dernière transmission.
       for (int i=1;i<7;i++){     // On vérifie si les 7 premiers caractères de la chaîne sont égales au filtre 
         if (linea[i]==comandoGPR[i-1]){
           bien++;
         }
       }
       
       if(bien==6){               // If on est sur la bonne chaine alors on traite les data
        // Serial.write(linea);
         for (int i=0;i<300;i++){
           if (linea[i]==','){    // On recherche la position des "," dans la chaîne pour repérer les différents champs
             // Empeche le dépassement du buffer en cas de trop nombreux parametres
             if (cont < 12) { indices[cont]=i; }
             cont++;
           }
           if (linea[i]=='*'){    // On recherche le "*" qui précède le checksum
             indices[12]=i;
             cont++;
           }
         }

          // Calcul du CRC checksum de la trame reçue (Boucle XOR avec le caractère précédent)
          byte CRC = 0;
          for (int k=2;k<(indices[12]);k++) {
            CRC = CRC ^ linea[k];
          }
          //Serial.println("");
          //Serial.print("Calculated CRC : ");
          //Serial.println(CRC);

          // Conversion du cheksum reçu en décimal
          char recvchk[4] = {0};
          recvchk[0] = '0';
          recvchk[1] = 'X';
          recvchk[2] = linea[indices[12]+1];
          recvchk[3] = linea[indices[12]+2];
          int number = strtol(recvchk, NULL, 16);
          //Serial.print("CRC Reçu en décimal : ");
          //Serial.println(number);

          // Vérification que le CRC Calculé correspond au CRC Reçu :
          if (CRC == number) { 
            //Serial.println("CRC Valide"); 
            indices[13]=indices[12]+3;
            //Serial.println("---------------");
            int posValid = indices[1];
            if ((char)linea[posValid+1] == 'A' && cont == 13) { // On vérifie si la position fournie est valide (A si valide, V si non valide)
              // Récup de la valeur de la latitude
              char Lat[11] = {0};
              int l = indices[2];
              for (int k=indices[2];k<indices[3]-1;k++) {
                Lat[k - l] = linea[k+1];
              }
              // Récup de la valeur de la longitude
              char Lon[11] = {0};
              l = indices[4];
              for (int k=indices[4];k<indices[5]-1;k++) {
                Lon[k - l] = linea[k+1];
              }
              // Conversion des char[] en float
              msg.latitude = atof(Lat) / 100;
              msg.longitude = atof(Lon) / 100;
              
              // Inverse le signe si on ne se trouve pas dans la partie Nord / Est
              msg.latitude = linea[indices[3] + 1] == 'N' ? msg.latitude : -msg.latitude;
              msg.longitude = linea[indices[5] + 1] == 'E' ? msg.longitude : -msg.longitude;

              // On ajoute les param supplémentaires
              msg.deviceid = deviceid;
              int sensorValue = analogRead(A0);
              msg.voltage = (sensorValue * (3.27 / 1023) * 100) * 2;
              //Serial.println(msg.deviceid);
              //Serial.println(msg.voltage);
              // On tente 5 fois de transmettre le message
              bool resTX = false;
              for (int i=0;i=4;i++) {
                resTX = SendSigFox();
                if (resTX == true) break;
              }
              //Serial.println(latitude);
              //Serial.println(longitude);
              //for (int i=0;i<13;i++){
              //  switch(i){
              //    case 0 :Serial.print("Time in UTC (HhMmSs): ");break;
              //    case 1 :Serial.print("Status (A=OK,V=KO): ");break;
              //    case 2 :Serial.print("Latitude: ");break;
              //    case 3 :Serial.print("Direction (N/S): ");break;
              //    case 4 :Serial.print("Longitude: ");break;
              //    case 5 :Serial.print("Direction (E/W): ");break;
              //    case 6 :Serial.print("Velocity in knots: ");break;
              //    case 7 :Serial.print("Heading in degrees: ");break;
              //    case 8 :Serial.print("Date UTC (DdMmAa): ");break;
              //    case 9 :Serial.print("Magnetic degrees: ");break;
              //    case 10 :Serial.print("Variation (E/W): ");break;
              //    case 11 :Serial.print("Mode: ");break;
              //    case 12 :Serial.print("Checksum: ");break;
              //  }
              //  for (int j=indices[i];j<(indices[i+1]-1);j++){
              //    Serial.print(linea[j+1]); 
              //  }
              //  Serial.println("");
              //Serial.println("On a une position valide on arrête la boucle d'acquisition");
              startLoop = false; // On arrête l'acquisition de données car les données sont transmises.
              //}
            } else {
              //Serial.println("Invalid data received.");
              //Serial.println(linea);
            }
          } else {
            //Serial.println("*** CHECKSUM ERROR ***");
          }
       }
       conta=0;                    // On remet à zéro les compteurs
       for (int i=0;i<300;i++){    
         linea[i]=' ';             
       }                 
     }
   }
   return millis();
}

boolean SendSigFox() {
  SigFox.begin(); // On prépare le SigFox à la transmission des données
  delay(100);
  SigFox.status();
  delay(1);
  SigFox.beginPacket(); // Transmission des informations -- Les float sont codés en little-endian
  SigFox.write((uint8_t*)&msg, 12); // Envoi du message. On passe l'instance de la structure
  int ret = SigFox.endPacket(); // Fin de la transmission on vérifie si ça s'est bien passé (on ne peut pas vérifier si l'info a été reçue car pas de callback)
  SigFox.end(); // On endort le SigFox
  if (ret == 0) {
    //Serial.println("OK : Transmission terminée");
    return true;
  } else {
    //Serial.println("KO : Erreur de transmission");
    return false;
  }
}

