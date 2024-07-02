#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <string.h>

// Resposta do elevador
// eIa -> terreo = 0
// eIb -> 1 andar = 1
// eIc -> 2 andar = 2
// ...
// eIp -> 15 andar = 15
// a  - p
// 97 - 112
// 



// Comandos do microcontrolador
// Sobe        es\r
// Desce       ed\r
// Inicializa  er\r
// Para        ep\r
// Abre        ea\r
// Fecha       ef\r



// Define o tamanho máximo da string
#define MAX_COMMAND_LENGTH 20

/* Variáveis para armazenamento do handle das tasks */
TaskHandle_t task1Handle = NULL;
TaskHandle_t task2Handle = NULL;
TaskHandle_t task3Handle = NULL;

// Handle para a fila
QueueHandle_t xQueueRx;
QueueHandle_t xQueueTx;

//Status da porta 
#define FECHADA 0
#define ABERTA 1

int volatile statusPorta = FECHADA;

//Status Elevador
#define SUBINDO 0
#define DESCENDO 1
#define PARADO 2

int volatile statusElevador = PARADO;

//Tipos de mensagens recebidas
#define BTN_SUBIDA 0
#define BTN_DESCIDA 1
#define BTN_INTERNO 2
#define STATUS_ANDAR 3
#define STATUS_PORTA 4


void SerialTask(void *parameter) {

  //char buffer[MAX_COMMAND_LENGTH];

  while (true) {
    if (Serial.available() > 0) {
      //Lê até encontrar o LF
      // Aloca memória para uma string
      char *buffer = (char *)pvPortMalloc(10 * sizeof(char));
      String receivedData = Serial.readStringUntil('\n');
      // Copia a string para a memória alocada
      // transforma o comando recebido em padrão C e  copia para o buffer alocado
      strcpy(buffer, receivedData.c_str());
      // Envia para a fila
      xQueueSend(xQueueRx, &buffer, portMAX_DELAY);
      // Enviar o buffer alocado com o valor recebido para a fila
      //String receivedData = Serial.readString();
      //Serial.print("Recebido: ");
      //Serial.println(receivedData);
    }

    //Fazer logica para envio de comandos~
    
    char *sendCommand;
    //Verifica se tem comandos na fila, mas sem bloquear a task, pois esta task verifica se há dados recebidos também
    //Ou seja, não pode ser bloqueada dessa maneira
    if (xQueueReceive(xQueueTx, &sendCommand, 0) == pdPASS)
    {
            // Envia o comando
            Serial.printf("%s", sendCommand);//o <CR> já vem do comando na fila

            // Libera a memória da string recebida
            free(sendCommand);
    }

    vTaskDelay(10 / portTICK_PERIOD_MS); // Delay para evitar consumir muito processamento
  }

}

void SendElevatorCommandTask(void *parameter){

  char *receivedCommand;
  //char buffer[MAX_COMMAND_LENGTH];

  while(true){
    //char *str = (char *)parameter; 
    // Recebe o ponteiro da fila
    if (xQueueReceive(xQueueRx, &receivedCommand, portMAX_DELAY) == pdPASS)
    {
        // Processa o comando
        //Serial.printf("Processing command: %s\n", receivedCommand);
        /*** 
        // Verificando o tipo de mensagem
        if(strlen(receivedCommand) == 4 && receivedCommand[1] == 'I'){
          //Se o tamanho da string for 4 e for da forma <elevador = e>I<andar = a ... p><CR> ->Obs.: A captura ira armazenadar <CR> também
          //Então é uma requisição de andar, pode ser a = Térreo ou b = 1 ... até p =15
        }
        if(strlen(receivedCommand) == 4 && receivedCommand[1] != 'I'){
          //Se o tamanho da string for 4 e for da forma <elevador = e><andar = 10 a 15><CR> ->Obs.: A captura ira armazenadar <CR> também
          //Então é o andar atual, com 4 caracteres poder ser de e10<CR> a e15<CR>
        }

        if(strlen(receivedCommand) == 3){
        //Se o tamanho da string for 3 e for da forma <elevador = e><andar = 0 a 9><CR> ->Obs.: A captura ira armazenadar <CR> também
        //Então é o andar atual, com 3 caracteres poder ser de e0<CR> a e9<CR>
          if(receivedCommand[1] == 'F' || receivedCommand[1] == 'A'){
            //Se for na forma <andar><F>. Então as portas estão fechadas
            //Se for na forma <andar><A>. Então as portas estão abertas
        }
        if(strlen(receivedCommand) == 6 && receivedCommand[1] == 'E'){
          //Se o tamanho da string for 6 e for da forma <elevador>E<andar_dezena><andar_unidade><direção do botâo><CR> ->Obs.: A captura ira armazenadar <CR> também
          //<andar_dezena><andar_unidade> = 00 a 15
          //<direção do botâo> = s -> Sobe ou <direção do botâo> = d -> Desce

        }

        ***/

        int tamanahoComando = strlen(receivedCommand);
        char *sendCommand = (char *)pvPortMalloc(10 * sizeof(char));
        //Váriaveis de status do elevador
        int botoesInternosCabine; //Botão interno pressionado na cabine
        volatile static int andarAtual = 0; // Andar atual do elevador
        int porta; //Status da porta, pode estar fechada ou aberta
        volatile static int andarChamandoElevador = -1; // Andar em que alguém está requisitando o elevador para subir ou descer
        int tipoMensagem; //Tipo da mensagem recebida, possibilidades -> botao de subida/descida pressionadao (Ex>eE10s)->Botão interno pressionado(Ex>eIa)
                                                      //              -> Andar atual do elevador              (Ex>e10)
                                                      //              -> Status da porta                      (Ex>eF ou eA)
        switch(tamanahoComando){
          case 3://Andar em que a cabine está, andar = e0 a e9 ou status portas  = eF e eA
            if(receivedCommand[1]>='0' && receivedCommand[1]<='9'){
              andarAtual = receivedCommand[1]-48;
              tipoMensagem = STATUS_ANDAR;
            }else{
              if(receivedCommand[1] == 'F' || receivedCommand[1] == 'A'){
                porta = (receivedCommand[1] == 'F') ? FECHADA : ABERTA;
                tipoMensagem = STATUS_PORTA;
              }
            }
            break;
          case 4://Botão da cabine pressionado  = eIa a eIp
            if(receivedCommand[1] == 'I'){
              botoesInternosCabine = receivedCommand[2]-97;//Transforma de ascii para um formato inteiro de 0 a 15
              tipoMensagem = BTN_INTERNO;
            }else{
              //Andar em que a cabine está, andar e10 a e15
              andarAtual = (receivedCommand[1]-48)*10+(receivedCommand[2]-48);
              tipoMensagem = STATUS_ANDAR;
            }
            break;
          case 6://Botão externo de subida ou descida pressionado  = <elevador>E<andar_dezena><andar_unidade><direção do botâo><CR>
            if(receivedCommand[1] == 'E'){//eE10s ou eE10d
              if(receivedCommand[4] == 's'){//Subida
                
                andarChamandoElevador = (receivedCommand[2]-48)*10+(receivedCommand[3]-48);
                tipoMensagem = BTN_SUBIDA;
              }
              if(receivedCommand[4] == 'd'){//Descida
                
                andarChamandoElevador = (receivedCommand[2]-48)*10+(receivedCommand[3]-48);
                Serial.printf("%i",andarChamandoElevador);
                tipoMensagem = BTN_DESCIDA;
              }
            }
            break;
        }
        int difAndar;
        switch (tipoMensagem)
        {
            case STATUS_PORTA:
              /* PASS */
              break;
            case STATUS_ANDAR:
              if(andarChamandoElevador != -1){
                difAndar =  andarAtual - andarChamandoElevador;//Quantidade de andares faltantes
                Serial.printf("andarAtual %i andarChamandoElevador %i \n", andarAtual, andarChamandoElevador);
                Serial.printf("dif %i \n",difAndar);
                if(difAndar == 0){
                  //Enviar comando de parada
                  strcpy(sendCommand, "er\r");
                  andarChamandoElevador = -1;
                  statusElevador = PARADO;
                }else{
                  if(difAndar>0){
                    //Enviar comando de descida se o elevador não estiver descendo
                    if(statusElevador != DESCENDO){
                      strcpy(sendCommand, "ed\r");
                      statusElevador = DESCENDO;
                    }
                  }
                  if(difAndar<0){
                    //Enviar comando de subida se o elevador não estiver subindo
                    if(statusElevador != SUBINDO){
                      strcpy(sendCommand, "es\r");
                      statusElevador = SUBINDO;
                    }
                  }
                }
              }
              
              break;
            case BTN_SUBIDA:
               if(andarChamandoElevador != -1){
                difAndar =  andarAtual - andarChamandoElevador;//Quantidade de andares faltantes

                if(difAndar == 0){
                  //Enviar comando de parada
                  strcpy(sendCommand, "er\r");
                  andarChamandoElevador = -1;
                  statusElevador = PARADO;
                }else{
                  if(difAndar>0){
                    //Enviar comando de descida se o elevador não estiver descendo
                    if(statusElevador != DESCENDO){
                      strcpy(sendCommand, "ed\r");
                      statusElevador = DESCENDO;
                    }
                  }
                  if(difAndar<0){
                    //Enviar comando de subida se o elevador não estiver subindo
                    if(statusElevador != SUBINDO){
                      strcpy(sendCommand, "es\r");
                      statusElevador = SUBINDO;
                    }
                  }
                }
              }
              break;
            case BTN_DESCIDA:
              /* PASS */
              break;
            case BTN_INTERNO:
              /* PASS */
              break;
        }


        // Envia para a fila
        xQueueSend(xQueueTx, &sendCommand, portMAX_DELAY);
        // Libera a memória da string recebida
        free(receivedCommand);
      }
  }
}


void CommandTask(void *parameter){
  //Recebera as solicitações do elevador
  while(true){
    
  }

}



void setup() {
  // Inicializando a Serial
  Serial.begin(115200);
  while (!Serial) {
    ; // Espera pela inicialização da Serial
  }
   
  // Cria a fila com capacidade para 10 ponteiros para strings
  // Enfileirar quantos comandos ?
  xQueueRx = xQueueCreate(10, sizeof(char *));
  xQueueTx = xQueueCreate(10, sizeof(char *));

  // Criando a tarefa
  xTaskCreate(
    SerialTask,   // Função da tarefa
    "Read and write Serial Task", // Nome da tarefa
    2048,              // Tamanho da stack da tarefa
    NULL,              // Parâmetro da tarefa
    1,                 // Prioridade da tarefa
    &task1Handle             // Handle da tarefa
  );
  xTaskCreate(
    SendElevatorCommandTask,   // Função da tarefa
    "Read Serial Task", // Nome da tarefa
    2048,              // Tamanho da stack da tarefa
    NULL,              // Parâmetro da tarefa
    1,                 // Prioridade da tarefa
    &task2Handle               // Handle da tarefa
  );
  /***xTaskCreate(
    CommandTask,   // Função da tarefa
    "Read Serial Task", // Nome da tarefa
    2048,              // Tamanho da stack da tarefa
    NULL,              // Parâmetro da tarefa
    1,                 // Prioridade da tarefa
    NULL               // Handle da tarefa
  );***/
}

void loop() {
  // O loop principal pode permanecer vazio se tudo for tratado pelas tarefas
}
