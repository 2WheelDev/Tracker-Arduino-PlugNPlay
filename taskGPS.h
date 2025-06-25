#include "HardwareSerial.h"

#ifndef TASK_GPS_H
#define TASK_GPS_H
#include <Arduino.h>
#include "dataStructures.h"
#include <TinyGPS++.h>  
#include "taskSQLITE.h" 
#include <math.h>
#include <Adafruit_NeoPixel.h>



// Definir el pin donde está conectado el DHT11

#define UMBRAL_MOV 5.0

// Instancias de sensores

// Última vez que se tomó una medida válida
unsigned long ultimaMedidaOK = 0;
void taskGPS(void *param);
double calcularDistanciaKm(double lat1, double lon1, double lat2, double lon2);

// Intervalo máximo permitido sin medidas (en milisegundos)
const unsigned long pausaMaxMs = 10000;
unsigned long deltams;

//  Definir el pin donde está conectado el GPS
#define GPS_RX 4
#define GPS_TX 5
#define TOLERANCIA_GPS 5000
#define TIMEOUT_GPS 5000

#define LED_PIN     48   // Pin de datos
#define NUM_LEDS     1   // Número de LEDs conectados
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);


// Filtro de media móvil para suavizar velocidad GPS
const int NUM_VELOCIDADES = 20;
double historialVelocidades[NUM_VELOCIDADES] = {0};
int indiceVelocidad = 0;
bool bufferVelocidadLleno = false;
double velocidad = 0.0;
double distancia=0.0;
HardwareSerial gpsSerial(1);
TinyGPSPlus gps;
GPSData gpsData;

int syncTries = 0;

//  Variables  para llevar control global
unsigned long ultimoTiempo = 0;
static double ultimaLat = 0.0;
static double ultimaLon = 0.0;

void taskGPS(void *parameter);
void procesarGPS();



void calcularHz() {
  static unsigned long ultimaMuestra = 0;
  static unsigned long tiempoActivoMs = 0;
  static unsigned long totalMuestras   = 0;

  static const int   VENTANA_SEGUNDOS     = 30;
  static const int   MAX_MUESTRAS_VENTANA = 300;  // para hasta 10 Hz
  static unsigned long marcasTiempo[MAX_MUESTRAS_VENTANA];
  static int        indiceVentana  = 0;
  static int        totalVentana   = 0;
  static bool       bufferLleno    = false;

  const unsigned long MAX_DT_VALIDO = 500;  // ms; umbral para salto (pausa)

  unsigned long ahora = millis();

  if (ultimaMuestra > 0) {
    unsigned long dt = ahora - ultimaMuestra;

    if (dt <= MAX_DT_VALIDO) {
      // 
      tiempoActivoMs += dt;
      totalMuestras++;

     
      marcasTiempo[indiceVentana] = ahora;
      indiceVentana = (indiceVentana + 1) % MAX_MUESTRAS_VENTANA;
      if (totalVentana < MAX_MUESTRAS_VENTANA) totalVentana++;
      else bufferLleno = true;
    }
    else {
      
    }
  }

  ultimaMuestra = ahora;

  
  if (tiempoActivoMs > 0) {
    hzMedia = (totalMuestras * 1000.0) / tiempoActivoMs;
  }


  int contador = 0;
  for (int i = 0; i < totalVentana; i++) {
    if ((ahora - marcasTiempo[i]) <= (VENTANA_SEGUNDOS * 1000)) {
      contador++;
    }
  }
  hzReciente = contador / (float)VENTANA_SEGUNDOS;
}


void procesarGPS() {
    double nuevaLat = gps.location.lat();
    double nuevaLon = gps.location.lng();

        // Calcular delta de tiempo entre dos medidas de procesar gps
    unsigned long ahora = millis();
    deltams = ahora - ultimoTiempo;

 // Evitar cálculo si aún no se ha registrado la primera posición válida
    if (ultimaLat == 0.0 && ultimaLon == 0.0) {
      ultimaLat = nuevaLat;
      ultimaLon = nuevaLon;
        ultimoTiempo = ahora;           // ← lo pones aquí

      Serial.println("🔄 Coordenadas iniciales registradas");
      return;
    }




    ultimoTiempo = ahora;
    tiempoTotalMs += deltams;

   



    distancia = gps.distanceBetween(ultimaLat, ultimaLon, nuevaLat, nuevaLon);
distanciaKm=distancia/1000.0;
velocidadKmh=gps.speed.kmph();

if (velocidadKmh>=UMBRAL_MOV){
 estaEnMovimiento = true;
    tiempoMovimientoMs += deltams;  
    } // acumula tiempo en movimiento}
else{
    estaEnMovimiento = false;
    tiempoParadoMs    += deltams;   // acumula tiempo parado
    return;
}
          distanciaTotalKm += distanciaKm;

    ultimaLat = nuevaLat;
    ultimaLon = nuevaLon;

        // Preparar datos GPS
    gpsData.lat = nuevaLat;
    gpsData.lon = nuevaLon;
    gpsData.ele = gps.altitude.meters();
    gpsData.speed = gps.speed.kmph();
    gpsData.course = gps.course.deg();
    gpsData.satelites = gps.satellites.value();
    gpsData.hdop= gps.hdop.hdop();


    xQueueOverwrite(gpsQueue, &gpsData);

    // Guardar en buffer circular
    if (writingBufferA) {
      if (bufferIndexA < BUFFER_SIZE) {
        gpsBufferA[bufferIndexA++] = gpsData;
      } else {
        Serial.println("⚠️ Buffer A lleno → cambiando a B");
        writingBufferA = false;
        xTaskNotifyGive(TaskSQLITEHandle);
      }
    } else {
      if (bufferIndexB < BUFFER_SIZE) {
        gpsBufferB[bufferIndexB++] = gpsData;
      } else {
        Serial.println("⚠️ Buffer B lleno → cambiando a A");
        writingBufferA = true;
        xTaskNotifyGive(TaskSQLITEHandle);
      }
    }
    
    
   calcularHz();  
}







void checkGPSConnection() {
  while (gpsSerial.available()) {
  gps.encode(gpsSerial.read());
}
  sats = gps.satellites.value();
  hdops = gps.hdop.hdop();
  vel = gps.speed.kmph();

  unsigned long ahora = millis();

  if (sats >= 4) {
      strip.setPixelColor(0, strip.Color(0, 255, 0));
  strip.show();
    lastUpdate = ahora;
    desconexion = 0;
  } else if (ahora - lastUpdate > TIMEOUT_GPS && sats<4) {
          strip.setPixelColor(0, strip.Color(255, 0, 0));
  strip.show();
    desconexion = (ahora - lastUpdate) / 1000;
    fixValido=false;
  }
}
bool esBisiesto(int agno) {
  return (agno % 4 == 0 && agno % 100 != 0) || (agno % 400 == 0);
}
// Función para ajustar día/mes/año correctamente
void ajustarDiaMesAgno() {
  const int diasPorMes[] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
  int diasDelMes = diasPorMes[relojMes - 1];

  if (relojMes == 2 && esBisiesto(relojAgno)) {
    diasDelMes = 29;
  }

  if (relojDia > diasDelMes) {
    relojDia = 1;
    relojMes++;
    if (relojMes > 12) {
      relojMes = 1;
      relojAgno++;
    }
  }
}
void syncClockWithGPS() {

while (gpsSerial.available()) {
  gps.encode(gpsSerial.read());
  
}
          strip.setPixelColor(0, strip.Color(0, 0, 10));
          strip.show();


  sats=gps.satellites.value();
  hdops=gps.hdop.hdop();

  if ( gps.time.isValid() && gps.time.age() < 30000 
  && gps.date.isValid() && gps.satellites.value()>=4) {
    relojHora = gps.time.hour();
    relojMinuto = gps.time.minute();
    relojSegundo = gps.time.second();
    relojMilisegundos = millis() % 1000;  // Ajustar milisegundos al tiempo actual
    relojDia = gps.date.day();
    relojMes = gps.date.month();
    relojAgno = gps.date.year();
    lastMillis = millis();
    Serial.println(" ✅ GPS sincronizado");

    Serial.printf("⏳ Reloj interno calibrado con GPS: %04d/%02d/%02d %02d:%02d:%02d.%03d\n",
                  relojAgno, relojMes, relojDia, relojHora, relojMinuto, relojSegundo, relojMilisegundos);
    sprintf(logFileName, "/%04d%02d%02d-%02d%02d%02d.db",
            relojAgno, relojMes, relojDia, relojHora, relojMinuto, relojSegundo);

    sprintf(timestampInicioRuta, "%04d-%02d-%02dT%02d:%02d:%02dZ",
            relojAgno, relojMes, relojDia, relojHora, relojMinuto, relojSegundo);

    Serial.printf("📁 Nombre de la base de datos: %s\n", logFileName);
    gpsSync = true;
    tiempoTotalMs      = 0;
    tiempoMovimientoMs = 0;
    tiempoParadoMs     = 0;



  } else {
    syncTries++;
    Serial.printf("⚠️ GPS no sincronizado. Esperando datos válidos... intento Nº: %d \n",syncTries);
       Serial.printf("📁 Numero de Satelites: %d\n", gps.satellites.value());

    vTaskDelay(pdMS_TO_TICKS(100));  
  strip.clear();
  strip.show();         
              vTaskDelay(pdMS_TO_TICKS(100));  

  }
}


void updateInternalClock() {
  unsigned long currentMillis = millis();
  unsigned long elapsedMillis = currentMillis - lastMillis;
  lastMillis = currentMillis;



  // Convertir milisegundos a segundos
  relojMilisegundos += elapsedMillis;
  while (relojMilisegundos >= 1000) {
    relojSegundo++;
    relojMilisegundos -= 1000;
  }

  //  Ajustar minutos
  while (relojSegundo >= 60) {
    relojMinuto++;
    relojSegundo -= 60;
  }

  //  Ajustar horas
  while (relojMinuto >= 60) {
    relojHora++;
    relojMinuto -= 60;
  }

  //  Ajustar días y meses
  if (relojHora >= 24) {
    relojHora = 0;
    relojDia++;
    ajustarDiaMesAgno();  //  Corrige cambios de mes y año
  }
  //  Guardar fecha en gpsData (para registros)
  gpsData.agno = relojAgno;
  gpsData.mes = relojMes;
  gpsData.dia = relojDia;
  gpsData.hora = relojHora;
  gpsData.minuto = relojMinuto;
  gpsData.segundo = relojSegundo;
  gpsData.milisegundos = relojMilisegundos;
  xQueueOverwrite(gpsQueue, &gpsData);  // Enviar SIEMPRE la hora al `taskDisplay`
}
void taskGPS(void *parameter) {
  gpsSerial.begin(115200, SERIAL_8N1, GPS_RX, GPS_TX);
 strip.begin();
   strip.show();

 vTaskDelay(pdMS_TO_TICKS(100)); // <-- IMPORTANTE para FreeRTOS

strip.setBrightness(255);
  strip.setPixelColor(0, strip.Color(0, 0, 0));
  strip.show();
 // pequeña pausa para que lo visualices

   

  while (!gpsSync) {
         
    syncClockWithGPS();
          xQueueOverwrite(gpsQueue, &gpsData);  // Actualizar solo la hora en pantalla

    vTaskDelay(pdMS_TO_TICKS(40));  //  Deja que otras tareas corran

  }

  for (;;) {
    //  Actualizar el reloj interno basado en `millis()`
    updateInternalClock();  // Actualizar el reloj interno
    checkGPSConnection();//comprueba la cantidad de satelites
    
//  Esperar activamente a que haya un fix GPS válido y actualizado
    if (gps.location.isUpdated() && gps.satellites.value() >= 4) {
       //  Si llegamos aquí, hay un fix GPS nuevo y válido
  //Serial.println(" ✅señal de gps adquirida✅");
      fixValido=true; 
      procesarGPS();
      totalMuestrasGPS++;
    vTaskDelay(pdMS_TO_TICKS(75));
    }
    else{
//Serial.println("Esperando señal de gps");
      xQueueOverwrite(gpsQueue, &gpsData);  // Actualizar solo la hora en pantalla
    vTaskDelay(pdMS_TO_TICKS(50));
    }     
        vTaskDelay(pdMS_TO_TICKS(5));
  }
}


#endif
