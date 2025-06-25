#ifndef TASK_DISPLAY_H
#define TASK_DISPLAY_H
#include <Arduino.h>

#include <math.h>
#include "dataStructures.h"  
#include "taskGPS.h"
#include "taskSQLITE.h"
#include "taskWeb.h"

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>





#define SCREEN_WIDTH 128 // Ancho del display en píxeles
#define SCREEN_HEIGHT 64 // Alto del display en píxeles

#define OLED_RESET     -1 // Reset no conectado
#define SDA_PIN         3 // Cambia si usas otros pines
#define SCL_PIN         1

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void flushAndCloseDatabase();






/*
void iniciarModoGPS() {
  // GPS
  if (TaskGPSHandle == NULL) {
    xTaskCreatePinnedToCore(taskGPS, "TaskGPS", stackSizeTaskGPS  ,  NULL, 4, &TaskGPSHandle, 1);
    Serial.println("✅ taskGPS creada");
  } else {
    vTaskResume(TaskGPSHandle);
    Serial.println("▶️ taskGPS reanudada");
  }


  // SQLITE
  if (TaskSQLITEHandle == NULL) {
    xTaskCreatePinnedToCore(taskSQLITE,"TaskSQLITE" ,stackSizeTaskSQLITE, NULL, 3, &TaskSQLITEHandle, 0);
    Serial.println("✅ taskSQLITE creada");
  } else {
    vTaskResume(TaskSQLITEHandle);
    Serial.println("▶️ taskSQLITE reanudada");
  }


}*/
void detenerModoGPS() {
    gpsData = {};
  if (TaskGPSHandle != NULL) {
    TaskGPSHandle=NULL;
    vTaskDelete(TaskGPSHandle);
  }
  if (TaskSQLITEHandle != NULL){
    TaskSQLITEHandle=NULL;
    vTaskDelete(TaskSQLITEHandle);
  } 

}

void IniciarWeb() {
  // 🚫 Apagar pantalla y touch temporalmente
 
  if (TaskSQLITEHandle != NULL) vTaskSuspend(TaskSQLITEHandle);

  vTaskDelay(pdMS_TO_TICKS(1000));  // ⏳ pequeña espera para liberar SPI

  if (TaskWebHandle == NULL) {
    Serial.println("🟢 Iniciando WebServer...");
  xTaskCreatePinnedToCore(taskWebServer, "taskWeb", 10192, NULL, 1, &TaskWebHandle, 1);// task para crear servidor
  } else {
    Serial.println("ℹ️ taskWeb ya estaba creada");
  }
}

void detenerWeb() {

  if (TaskWebHandle != NULL) {
    WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
    vTaskDelete(TaskWebHandle);
    TaskWebHandle = NULL;
    Serial.println("🛑 taskWeb eliminada");
  }
}
void taskDisplay(void *){
   Wire.begin(SDA_PIN, SCL_PIN); // Iniciar I2C en pines definidos
  Serial.begin(115200);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3D)) {
    Serial.println(F("❌ No se detectó pantalla OLED"));
    while (true); // Parar ejecución
  }
  display.clearDisplay();
  display.setTextSize(1);      // Tamaño del texto
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);     // Coordenadas (x,y)
  display.println("Hola, ESP32-S3!");
  display.println("OLED 128x64 listo!");
  display.display();           // Mostrar contenido

 
    GPSData displayData; 


  for(;;){
     if(xQueuePeek(gpsQueue, &displayData, 0) == pdTRUE) {
    gpsData = displayData;
     }
     display.clearDisplay();
  display.setTextSize(1);      // Tamaño del texto
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);     // Coordenadas (x,y)
  /*display.printf("Hora: %02d:%02d:%02d.%03d\n", gpsData.hora, gpsData.minuto, gpsData.segundo, gpsData.milisegundos);
  display.printf("Fecha: %04d/%02d/%02d\n", gpsData.agno, gpsData.mes, gpsData.dia);

  display.printf("Lt:%.4f/", gpsData.lat);display.printf("Ln:%.4f\n", gpsData.lon);
  display.printf("Sat:%d/", sats); display.printf("HDOP:%.3f\n",hdops);
  display.printf("Vel:%.2f/", gpsData.speed);  display.printf("Ele:%.2f\n", gpsData.ele);
  display.printf("Dir:%.2f/", gpsData.course);display.printf("Km:%.2f\n", distanciaTotalKm);
    display.printf("Buffer A: %d / %d\n", bufferIndexA, BUFFER_SIZE);
     display.printf("Buffer B: %d / %d\n", bufferIndexB, BUFFER_SIZE);*/

       display.printf("SD: %s", sdReady ? "OK" : "FALLO");
  display.printf("BD: %s\n", dbReady ? "OK" : "FALLO");
  display.printf("ICM: %s\n", icmReady ? "OK" : "FALLO");
  display.printf("Bd Corrupta: %s\n", bdCorrupta ? "SI" : "NO");
   display.printf("Buffer listo: %s\n", bufferReadyForSD ? "SI" : "NO");

  display.printf("Buffer A: %d / %d\n", bufferIndexA, BUFFER_SIZE);
  display.printf("Buffer B: %d / %d\n", bufferIndexB, BUFFER_SIZE);
  display.printf("FIX: %s / \n", fixValido ? "SI" : "No"); 

  display.display();           // Mostrar contenido
        vTaskDelay(pdMS_TO_TICKS(300));
     
}
}
#endif
