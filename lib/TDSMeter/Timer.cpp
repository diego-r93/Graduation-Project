#include "Timer.h"

#include "CN0411.h"

// Instância global para o timer
FreeRTOSTimer* xTimer = nullptr;

// Função de callback que será chamada quando o timer expirar
void vTimerCallback(TimerHandle_t xTimerHandle) {
   if (xTimerHandle == nullptr) {
      Serial.println("Erro: xTimerHandle é nulo!");
      return;
   }

   CN0411_pwm_gen();
}

// Função para iniciar o timer de software
void timer_start(void) {
   if (xTimer == nullptr) {
      // Cria o timer de software usando sua classe
      xTimer = new FreeRTOSTimer("Timer", pdMS_TO_TICKS(1), pdTRUE, nullptr, vTimerCallback);
      if (xTimer == nullptr) {
         Serial.println("Erro ao criar xTimer!");
         return;
      }
      xTimer->start();
   } else {
      Serial.println("xTimer já foi criado.");
   }
}

// Função para suspender a execução por um número de milissegundos
void timer_sleep(timer_ticks_t ticks) {
   vTaskDelay(pdMS_TO_TICKS(ticks));
}

// Função para suspender a execução por um intervalo de 5 microsegundos
void timer_sleep_5uS(timer_ticks_t ticks) {
   for (timer_ticks_t i = 0; i < ticks; i++) {
      CN0411_pwm_gen();
      ets_delay_us(5);  // Precise delay for ESP32
   }
}
