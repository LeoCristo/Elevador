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
QueueHandle_t xQueueMensagens;

// Mutex para acesso da requisicao de andar
SemaphoreHandle_t xMutexRequisicaoAndar;


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
#define UNDEFINED 10
//Status chamando Andar
#define NAO_CHAMANDO -1

//Status da requisição de andar
#define SEM_REQUISICAO 1
#define EM_ANDAMENTO 0

// Indica se alguma requisicao de andar esta sendo tratada, utilizada pela task 3
static int requisicaoAndar = SEM_REQUISICAO; // Váriavel de controle das requisições de andar


//Andar atual do elevador
volatile static int andarAtual = 0; 

//Andar solicitado externamente ou internamente na cabine
volatile static int andarSolicitado;

/***
 * Recebe dados da serial e envia dados para o elevado a pedido da task de comandos
 * ***/
void SerialTask(void *parameter) {

  //char buffer[MAX_COMMAND_LENGTH];

  while (true) {
    if (Serial.available() > 0) {
      //Lê até encontrar o LF
      // Aloca memória para uma string
      char *buffer = (char *)pvPortMalloc(30 * sizeof(char));
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
            if(strcmp(sendCommand,"ef\r") == 0 || strcmp(sendCommand,"ea\r") == 0){
              vTaskDelay(700 / portTICK_PERIOD_MS);
            }
            // Libera a memória da string recebida
            free(sendCommand);
    }

    vTaskDelay(10 / portTICK_PERIOD_MS); // Delay para evitar consumir muito processamento
  }

}
/***
 *  Pega as mensagens recebidas da task da serial
 *  e envia requisições de andar para a task de comando
 *  também atualiza o estado do andar atual
 * ***/
void SendElevatorCommandTask(void *parameter){

  char *receivedCommand;
  //char buffer[MAX_COMMAND_LENGTH];

  while(true){
    //char *str = (char *)parameter; 
    // Recebe o ponteiro da fila
    if (xQueueReceive(xQueueRx, &receivedCommand, portMAX_DELAY) == pdPASS)
    {
        int tamanahoComando = strlen(receivedCommand);
        //char *sendCommand = (char *)pvPortMalloc(10 * sizeof(char));
        int sendAndar; //Andar solicitado, tanto externamente ou internamente na cabine
        //Váriaveis de status do elevador
        int botoesInternosCabine; //Botão interno pressionado na cabine
        //volatile static int andarAtual = 0; // Andar atual do elevador
        int porta; //Status da porta, pode estar fechada ou aberta
        volatile static int andarChamandoElevador = -1; // Andar em que alguém está requisitando o elevador para subir ou descer
        int tipoMensagem = UNDEFINED; //Tipo da mensagem recebida, possibilidades -> botao de subida/descida pressionadao (Ex>eE10s)->Botão interno pressionado(Ex>eIa)
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
                //Serial.printf("%i",andarChamandoElevador);
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
      
              break;
            case BTN_SUBIDA:
         
              if(andarChamandoElevador != NAO_CHAMANDO){
                sendAndar = andarChamandoElevador;
                andarChamandoElevador = NAO_CHAMANDO;
              }
              break;
            case BTN_DESCIDA:
              if(andarChamandoElevador != NAO_CHAMANDO){
                sendAndar = andarChamandoElevador;
                andarChamandoElevador = NAO_CHAMANDO;
              }
              break;
            case BTN_INTERNO:
              sendAndar = botoesInternosCabine;
              break;
        }
        //Só envia o andar pedido, se foi uma solicitação de andar 
        if(tipoMensagem == BTN_SUBIDA || tipoMensagem == BTN_DESCIDA || tipoMensagem == BTN_INTERNO){
          // Envia para a fila da task 3
          xQueueSend(xQueueMensagens, &sendAndar, portMAX_DELAY);
          //xQueueSend(xQueueTx, &receivedCommand, portMAX_DELAY);

        }
        
        // Libera a memória da string recebida da task 1
      free(receivedCommand);
      }
  }
}

/***
 * Recebe as requisições de andar
 * Ao pegar uma requisição de andar, Verifica se deve subir, descer ou parar
 * Ao pegar uma requisição de andar, outra requisição é pega apenas quando o elevador parar
 * ou seja, uma requisição por vez é atendida da fila
 * ***/
void CommandTask(void *parameter){
  //Recebera as solicitações do elevador em andares. Ex 0,1,2,...,15
  int andarSolicitado1;
  char *sendCommand1;
  char *sendCommand2; 
  char *sendCommand3; 
  int difAndar;

  while(true){
    //Só pega requisição da fila, se o elevador for parado
    //initialized<CR><LF>
    if (xSemaphoreTake(xMutexRequisicaoAndar, portMAX_DELAY) == pdTRUE) {
      if(requisicaoAndar == SEM_REQUISICAO){ 
        if (xQueueReceive(xQueueMensagens, &andarSolicitado1, portMAX_DELAY) == pdPASS){
            andarSolicitado = andarSolicitado1;
            difAndar = andarAtual - andarSolicitado1;
            if(difAndar == 0){
              sendCommand1 = (char *)pvPortMalloc(30 * sizeof(char));
              sendCommand2 = (char *)pvPortMalloc(30 * sizeof(char));
              sendCommand3 = (char *)pvPortMalloc(30 * sizeof(char));
            //Elevador deve parar, abrir a porta e fechar
              strcpy(sendCommand1, "ep\r");
              xQueueSend(xQueueTx, &sendCommand1, portMAX_DELAY);
              strcpy(sendCommand2, "ea\r");
              xQueueSend(xQueueTx, &sendCommand2, portMAX_DELAY);
              strcpy(sendCommand3, "ef\r");
              xQueueSend(xQueueTx, &sendCommand3, portMAX_DELAY);
              requisicaoAndar = SEM_REQUISICAO; 
              
            }else{
              sendCommand1 = (char *)pvPortMalloc(30 * sizeof(char));
              sendCommand2 = (char *)pvPortMalloc(30 * sizeof(char));
              if(difAndar<0){//Elevador deve fechar a porta e subir
                strcpy(sendCommand1, "ef\r");
                xQueueSend(xQueueTx, &sendCommand1, portMAX_DELAY);
                strcpy(sendCommand2, "es\r");
                xQueueSend(xQueueTx, &sendCommand2, portMAX_DELAY);
                requisicaoAndar = EM_ANDAMENTO;
              }
              if(difAndar>0){//Elevador deve fehcar a porta e descer
                strcpy(sendCommand1, "ef\r");
                xQueueSend(xQueueTx, &sendCommand1, portMAX_DELAY);
                strcpy(sendCommand2, "ed\r");
                xQueueSend(xQueueTx, &sendCommand2, portMAX_DELAY);
                requisicaoAndar = EM_ANDAMENTO;
              }
            }
        }
      
      }
    // Libera o mutex
    xSemaphoreGive(xMutexRequisicaoAndar);
    }
    vTaskDelay(10 / portTICK_PERIOD_MS); // Delay para evitar consumir muito processamento

  }
}

/***
 * Verifica se o andar solicitado foi atingido
 * E envia comandos para parar, abrir porta e fechar
 * ***/
void stopTask(void *parameter){
  char *sendCommand1;
  char *sendCommand2; 
  char *sendCommand3; 
  int difAndar;
  while(true){

    if (xSemaphoreTake(xMutexRequisicaoAndar, portMAX_DELAY) == pdTRUE) {
      difAndar = andarAtual - andarSolicitado;
      if(difAndar == 0){
        sendCommand1 = (char *)pvPortMalloc(30 * sizeof(char));
        sendCommand2 = (char *)pvPortMalloc(30 * sizeof(char));
        sendCommand3 = (char *)pvPortMalloc(30 * sizeof(char));
      //Elevador deve parar, abrir a porta e fechar
        strcpy(sendCommand1, "ep\r");
        xQueueSend(xQueueTx, &sendCommand1, portMAX_DELAY);
        strcpy(sendCommand2, "ea\r");
        xQueueSend(xQueueTx, &sendCommand2, portMAX_DELAY);
        strcpy(sendCommand3, "ef\r");
        xQueueSend(xQueueTx, &sendCommand3, portMAX_DELAY);
        requisicaoAndar = SEM_REQUISICAO; 
      }
    // Libera o mutex
    xSemaphoreGive(xMutexRequisicaoAndar);
    }
    vTaskDelay(10 / portTICK_PERIOD_MS); // Delay para evitar consumir muito processamento
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
  xQueueRx = xQueueCreate(10, sizeof(char *)); //Recebe as mensagens do elevador, tais como andar atual, solicitação de andar, status da porta
  xQueueTx = xQueueCreate(10, sizeof(char *)); //Recebe os comandos a serem enviados para o elevador
  xQueueMensagens = xQueueCreate(10, sizeof(int)); //Recebe as solicitacoes de andar - Nesse caso são inteiros de 0 a 15

  //Mutex
  // Cria o mutex da requisicao andar, duas tasks acessam essa variavel, task 3 e task 4
  xMutexRequisicaoAndar = xSemaphoreCreateMutex();

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
  
  xTaskCreate(
    CommandTask,   // Função da tarefa
    "Envia comandos", // Nome da tarefa
    4096,              // Tamanho da stack da tarefa
    NULL,              // Parâmetro da tarefa
    1,                 // Prioridade da tarefa
    NULL               // Handle da tarefa
  );
  xTaskCreate(
    stopTask,   // Função da tarefa
    "Para o elevador", // Nome da tarefa
    2048,              // Tamanho da stack da tarefa
    NULL,              // Parâmetro da tarefa
    1,                 // Prioridade da tarefa
    NULL               // Handle da tarefa
  );
}

void loop() {
  // O loop principal pode permanecer vazio se tudo for tratado pelas tarefas
}
