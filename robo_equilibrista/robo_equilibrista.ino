/*
 * =============================================================================
 *   ROBÔ EQUILIBRISTA
 *   Arduino Nano + MPU6050 (GY-521) + MX1508 — um único motor move as 2 rodas
 * =============================================================================
 *
 *   A IDEIA EM UMA FRASE
 *   --------------------
 *   O MPU6050 diz quanto o robô está inclinado; o PID decide com que força e
 *   para que lado girar o motor para colocar as rodas de volta embaixo do
 *   centro de massa — do mesmo jeito que você equilibra uma vassoura na mão.
 *
 *   O QUE ACONTECE 200 VEZES POR SEGUNDO
 *   ------------------------------------
 *     0. O MPU6050 termina uma medição e avisa pelo pino INT (interrupção).
 *     1. Ler acelerômetro e giroscópio do MPU6050.
 *     2. Juntar as duas leituras num ângulo confiável (filtro complementar).
 *     3. Calcular a correção com o PID.
 *     4. Mandar a correção para o motor através do MX1508.
 *
 *   LIGAÇÕES
 *   --------
 *     MPU6050 VCC ........ Nano 5V
 *     MPU6050 GND ........ Nano GND
 *     MPU6050 SDA ........ Nano A4
 *     MPU6050 SCL ........ Nano A5
 *     MPU6050 INT ........ Nano D2   (interrupção)
 *     MX1508  IN1 ........ Nano D9   (PWM)
 *     MX1508  IN2 ........ Nano D10  (PWM)
 *     MX1508  MOTOR-A .... os dois fios do motor
 *     Bateria + (via chave) ... MX1508 "+"  e  Nano VIN
 *     Bateria − ............... MX1508 "−"  e  Nano GND   (terra comum!)
 *
 *   MONTAGEM DO SENSOR
 *   ------------------
 *     Módulo deitado (na horizontal), o mais perto possível do eixo das
 *     rodas, com a seta "X" impressa na placa apontando para a FRENTE.
 *
 *   CONVENÇÃO DE SINAIS USADA EM TODO O CÓDIGO
 *   ------------------------------------------
 *     ângulo positivo  = robô tombando para a frente
 *     comando positivo = rodas andando para a frente
 *   Se o seu robô fizer o contrário, não mexa nas contas: use as chaves
 *   INVERTER_SENTIDO_DO_ANGULO e INVERTER_SENTIDO_DO_MOTOR, mais abaixo.
 *
 *   AJUSTE PELO MONITOR SERIAL (115200 baud, final de linha "Nova linha")
 *   ---------------------------------------------------------------------
 *     kp 20                   muda o ganho proporcional
 *     ki 40                   muda o ganho integral
 *     kd 0.8                  muda o ganho derivativo
 *     angulo_equilibrio -1.5  muda o ângulo em que o robô fica em pé
 *     marcar_equilibrio       usa o ângulo atual como ângulo de equilíbrio
 *     pwm_minimo 40           muda o menor PWM que faz o motor girar
 *     angulo_queda 35         muda a inclinação a partir da qual o robô "caiu"
 *     peso_giroscopio 0.98    muda quanto o filtro confia no giroscópio (0 a 1)
 *     telemetria              liga/desliga o envio de números para o computador
 *     valores                 mostra os valores atuais
 *     ajuda                   mostra esta lista
 *
 *   Não precisa instalar biblioteca: o MPU6050 é lido direto pelo I2C (Wire).
 * =============================================================================
 */

#include <Wire.h>


// =============================================================================
//  1. PINOS
// =============================================================================

const int PINO_MOTOR_IN1 = 9;    // MX1508 IN1 — precisa ser um pino com PWM (~)
const int PINO_MOTOR_IN2 = 10;   // MX1508 IN2 — precisa ser um pino com PWM (~)
const int PINO_LED       = 13;   // LED da própria placa: aceso = equilibrando
const int PINO_INT_MPU   = 2;    // MPU6050 INT — no Nano, só D2 e D3 têm interrupção externa

// SDA e SCL do MPU6050 ficam em A4 e A5. No Nano esses pinos são fixos para o
// I2C, por isso não aparecem aqui.


// =============================================================================
//  2. GANHOS DO PID  (é aqui que se mexe quando for ajustar o robô)
// =============================================================================
//
//  A saída do PID é um comando de motor entre -255 (ré total) e +255 (frente
//  total). Cada ganho diz quanto de comando cada "sintoma" vale:
//
//  Kp — "quanto estou inclinado?"
//       Kp = 20 significa: 1 grau de inclinação vira 20 de comando.
//       Pouco Kp: o robô cai sem reagir direito.
//       Muito Kp: ele treme para a frente e para trás cada vez mais forte.
//
//  Ki — "há quanto tempo estou inclinado?"
//       Ki = 40 significa: 1 grau mantido por 1 segundo acumula 40 de comando.
//       Corrige o robô que vai "andando" para um lado até cair.
//       Muito Ki: oscilações lentas e largas. Comece em zero.
//
//  Kd — "com que velocidade estou caindo?"
//       Kd = 0.8 significa: caindo a 100 graus/s vira 80 de comando.
//       Funciona como um freio que amortece o tremor causado pelo Kp.
//       Muito Kd: vibração rápida e zumbido no motor.

float Kp = 20.0;
float Ki = 0.0;
float Kd = 0.8;

// Ângulo em que o robô fica em pé sozinho. Quase nunca é exatamente 0: depende
// de onde está o centro de massa e de quão reto o sensor foi fixado.
// O jeito mais fácil de achar: segure o robô equilibrado e mande
// "marcar_equilibrio" pelo Monitor Serial.
float anguloDeEquilibrio = 0.0;


// =============================================================================
//  3. OUTROS AJUSTES
// =============================================================================

// Descubra com o robô no ar (veja o README). Troque se estiver ao contrário.
const bool INVERTER_SENTIDO_DO_ANGULO = false;
const bool INVERTER_SENTIDO_DO_MOTOR  = false;

// Motores pequenos não giram com PWM baixo — só zumbem. Qualquer comando
// diferente de zero começa a partir deste valor ("zona morta").
int pwmMinimo = 40;
const int PWM_MAXIMO = 255;

// Limite do termo integral (em unidades de comando). Sem limite, enquanto o
// robô está inclinado a soma cresce sem parar e ele "dispara" depois.
const float LIMITE_DO_TERMO_INTEGRAL = 120.0;

// Segurança. O ângulo de queda pode ser mudado pela serial ("angulo_queda 35").
// Passou disso longe do equilíbrio: o robô caiu, desliga o motor. Muito baixo,
// ele "desiste" em inclinações que ainda daria para salvar; muito alto, o motor
// fica girando com o robô deitado no chão.
float anguloDeQueda = 35.0;
const float ANGULO_DE_QUEDA_MINIMO = 5.0;    // limites aceitos pelo comando
const float ANGULO_DE_QUEDA_MAXIMO = 80.0;
const float ANGULO_PARA_REARMAR = 3.0;       // volta a equilibrar quando for levantado até aqui

// Filtro complementar: quanto confiar no giroscópio, de 0 a 1. Pode ser mudado
// pela serial ("peso_giroscopio 0.98"). Veja a explicação em atualizarAngulo().
float pesoDoGiroscopio = 0.98;

// Ritmo do controle: quem manda é o próprio MPU6050, que mede 200 vezes por
// segundo e avisa cada medição nova pelo pino INT. Este é o intervalo esperado
// entre dois avisos: 5000 µs = 5 ms. Para mudar, mude também o divisor de
// amostragem em iniciarMPU6050().
const unsigned long INTERVALO_ENTRE_AMOSTRAS_US = 5000;

// Passou este tempo sem nenhum aviso: o fio do INT soltou ou o sensor travou.
// O motor é desligado. 50 ms = 10 avisos perdidos seguidos.
const unsigned long TEMPO_MAXIMO_SEM_AVISO_US = 50000;

// De quanto em quanto tempo mandar os números para o computador.
const unsigned long INTERVALO_DA_TELEMETRIA_MS = 100;


// =============================================================================
//  4. MPU6050 — endereço e registradores (valores tirados do datasheet)
// =============================================================================

const uint8_t ENDERECO_MPU6050 = 0x68;   // com o pino AD0 solto ou em GND

const uint8_t REG_DIVISOR_DE_AMOSTRAGEM = 0x19;  // SMPLRT_DIV
const uint8_t REG_FILTRO_PASSA_BAIXAS   = 0x1A;  // CONFIG
const uint8_t REG_ESCALA_GIROSCOPIO     = 0x1B;  // GYRO_CONFIG
const uint8_t REG_ESCALA_ACELEROMETRO   = 0x1C;  // ACCEL_CONFIG
const uint8_t REG_CONFIGURACAO_DO_INT   = 0x37;  // INT_PIN_CFG
const uint8_t REG_INTERRUPCOES_LIGADAS  = 0x38;  // INT_ENABLE
const uint8_t REG_PRIMEIRO_DADO         = 0x3B;  // ACCEL_XOUT_H: início das leituras
const uint8_t REG_ENERGIA               = 0x6B;  // PWR_MGMT_1

// Na escala de ±500 graus/s, cada 65,5 unidades lidas equivalem a 1 grau/s.
const float UNIDADES_POR_GRAU_POR_SEGUNDO = 65.5;


// =============================================================================
//  5. ESTADO DO ROBÔ (valores que mudam enquanto ele funciona)
// =============================================================================

// Uma leitura completa do sensor, em unidades "cruas" (ainda sem conversão).
struct LeituraDoSensor {
  int16_t acelX, acelY, acelZ;
  int16_t giroX, giroY, giroZ;
};

float anguloAtual        = 0.0;  // graus;    positivo = tombado para a frente
float velocidadeAngular  = 0.0;  // graus/s;  positivo = tombando para a frente
float desvioDoGiroscopio = 0.0;  // graus/s que o giroscópio marca mesmo parado

float termoIntegral = 0.0;       // a "memória" do PID, já em unidades de comando
float ultimoTermoP  = 0.0;       // P e D guardados só para aparecer na telemetria
float ultimoTermoD  = 0.0;
float ultimoComando = 0.0;       // o que foi pedido ao motor por último

bool equilibrando = false;       // false = caído, esperando ser levantado

unsigned long instanteDoUltimoCiclo      = 0;  // em micros(): quando chegou a amostra usada por último
unsigned long instanteDaUltimaTelemetria = 0;  // em millis()
bool telemetriaLigada = true;

// Preenchidas dentro da interrupção, por isso "volatile": avisa o compilador
// que elas podem mudar a qualquer momento, fora do fluxo normal do loop().
volatile bool          chegouAmostraNova   = false;
volatile unsigned long instanteDaAmostraNova = 0;   // em micros()
bool sensorAvisando = true;      // false depois que os avisos pararam de chegar

char bufferDaSerial[32];         // guarda o comando enquanto ele chega letra a letra
int  tamanhoDoBuffer = 0;


// =============================================================================
//  SETUP — roda uma vez, ao ligar
// =============================================================================

void setup() {
  Serial.begin(115200);

  pinMode(PINO_MOTOR_IN1, OUTPUT);
  pinMode(PINO_MOTOR_IN2, OUTPUT);
  pinMode(PINO_LED, OUTPUT);
  pararMotor();

  Serial.println(F("\n# Robo equilibrista iniciando..."));

  Wire.begin();
  Wire.setClock(400000);             // I2C no modo rápido (400 kHz)
  Wire.setWireTimeout(3000, true);   // se um fio soltar, o I2C desiste em vez de travar

  if (!iniciarMPU6050()) {
    travarComErro(F("# ERRO: o MPU6050 nao respondeu. Confira VCC, GND, SDA (A4) e SCL (A5)."));
  }

  // A partir daqui, cada medição nova do sensor chama receberAvisoDoMPU6050().
  // RISING = na subida do pulso do INT (de 0 V para 5 V).
  pinMode(PINO_INT_MPU, INPUT);
  attachInterrupt(digitalPinToInterrupt(PINO_INT_MPU), receberAvisoDoMPU6050, RISING);

  if (!esperarAmostraNova()) {
    travarComErro(F("# ERRO: o MPU6050 responde, mas nao avisa pelo INT. Confira o fio INT -> D2."));
  }

  calibrarGiroscopio();

  // O filtro precisa de um ponto de partida: usamos o ângulo que o
  // acelerômetro enxerga agora.
  LeituraDoSensor leitura;
  if (lerMPU6050(leitura)) {
    anguloAtual = calcularAnguloPeloAcelerometro(leitura);
  }

  mostrarAjustes();
  Serial.println(F("# Coloque o robo em pe para comecar. Digite \"ajuda\" para ver os comandos."));

  // Joga fora um aviso que tenha chegado durante as mensagens acima: o primeiro
  // ciclo do loop() usa uma amostra fresca.
  unsigned long descartado;
  instanteDoUltimoCiclo = micros();
  pegarAmostraNova(descartado);
}


// =============================================================================
//  LOOP — roda sem parar
// =============================================================================

void loop() {
  // Comandos do computador e telemetria podem ser atendidos a qualquer hora...
  lerComandosDaSerial();
  enviarTelemetria();

  // ...mas o controle só roda quando o MPU6050 avisa que mediu de novo. Assim
  // cada ciclo usa uma medição nova, nem repetida nem atrasada, no ritmo fixo
  // do relógio do próprio sensor.
  unsigned long instanteDaAmostra;
  if (!pegarAmostraNova(instanteDaAmostra)) {
    verificarSeOSensorSumiu();
    return;   // nada novo ainda
  }

  // O tempo entre as amostras vem do instante em que cada aviso chegou, e não
  // de quando o loop() conseguiu atender: é o intervalo real entre as medições.
  unsigned long microssegundosDesdeOUltimoCiclo = instanteDaAmostra - instanteDoUltimoCiclo;
  instanteDoUltimoCiclo = instanteDaAmostra;

  // Voltando de um sumiço, o intervalo seria enorme e o ângulo guardado está
  // velho: recomeçamos o ângulo pelo acelerômetro, como no setup().
  bool voltouDeUmSumico = !sensorAvisando ||
                          microssegundosDesdeOUltimoCiclo > TEMPO_MAXIMO_SEM_AVISO_US;
  if (voltouDeUmSumico) {
    microssegundosDesdeOUltimoCiclo = INTERVALO_ENTRE_AMOSTRAS_US;
  }
  sensorAvisando = true;
  float segundosDesdeOUltimoCiclo = microssegundosDesdeOUltimoCiclo / 1000000.0;

  // --- Passos 1 e 2: onde estou? ---------------------------------------------
  LeituraDoSensor leitura;
  if (!lerMPU6050(leitura)) {
    // Sem sensor não dá para equilibrar: melhor desligar do que chutar.
    if (equilibrando) {
      Serial.println(F("# Falha na leitura do MPU6050. Motor desligado."));
    }
    equilibrando = false;
    pararMotor();
    digitalWrite(PINO_LED, LOW);
    return;
  }
  if (voltouDeUmSumico) {
    anguloAtual = calcularAnguloPeloAcelerometro(leitura);
  }
  atualizarAngulo(leitura, segundosDesdeOUltimoCiclo);

  // --- Passos 3 e 4: o que fazer? --------------------------------------------
  float distanciaDoEquilibrio = fabs(anguloAtual - anguloDeEquilibrio);

  if (equilibrando) {
    if (distanciaDoEquilibrio > anguloDeQueda) {
      // Inclinado demais: não tem mais como salvar. Desliga tudo.
      equilibrando = false;
      pararMotor();
      Serial.println(F("# Caiu. Levante o robo para recomecar."));
    } else {
      float comando = calcularPID(segundosDesdeOUltimoCiclo);
      acionarMotor(comando);
    }
  } else if (distanciaDoEquilibrio < ANGULO_PARA_REARMAR) {
    // Alguém levantou o robô até perto do equilíbrio: começa do zero.
    zerarPID();
    equilibrando = true;
    Serial.println(F("# Em pe! Equilibrando."));
  }

  digitalWrite(PINO_LED, equilibrando ? HIGH : LOW);
}

// Sem avisos há tempo demais, o controle não roda — e o motor ficaria preso no
// último comando. Melhor desligar.
void verificarSeOSensorSumiu() {
  if (!sensorAvisando || micros() - instanteDoUltimoCiclo <= TEMPO_MAXIMO_SEM_AVISO_US) {
    return;
  }
  sensorAvisando = false;
  equilibrando = false;
  pararMotor();
  digitalWrite(PINO_LED, LOW);
  Serial.println(F("# Falha: o MPU6050 parou de avisar pelo INT (D2). Motor desligado."));
}

// Pisca rápido para sempre: algo está errado e não dá para continuar.
void travarComErro(const __FlashStringHelper *mensagem) {
  Serial.println(mensagem);
  while (true) {
    digitalWrite(PINO_LED, !digitalRead(PINO_LED));
    delay(100);
  }
}


// =============================================================================
//  ÂNGULO — FILTRO COMPLEMENTAR
// =============================================================================
//
//  Nenhum dos dois sensores, sozinho, entrega um ângulo bom:
//
//   • ACELERÔMETRO: sente a gravidade, então sempre sabe onde é "para baixo" e
//     nunca se perde com o tempo. Mas qualquer tranco ou vibração do motor
//     também aparece como se fosse inclinação.
//     → bom a longo prazo, ruim no instante.
//
//   • GIROSCÓPIO: mede a velocidade de giro, liso e imune a trancos. Somando
//     essa velocidade a cada ciclo temos o ângulo — só que qualquer errinho
//     também vai sendo somado, e o ângulo "escorrega" aos poucos.
//     → bom no instante, ruim a longo prazo.
//
//  Misturando 98% do giroscópio com 2% do acelerômetro, ficamos com o melhor
//  dos dois: o giroscópio acompanha os movimentos rápidos e o acelerômetro,
//  devagarinho, puxa o ângulo de volta para o valor verdadeiro.
//
//  Quão devagar? A cada ciclo, só (1 − peso) da diferença entre os dois é
//  corrigida. Isso dá uma "constante de tempo" de
//
//      intervalo × peso / (1 − peso)  =  5 ms × 0,98 / 0,02  ≈  0,25 s
//
//  Movimentos mais rápidos que isso vêm do giroscópio; tendências mais lentas,
//  do acelerômetro.
//     peso 0,95  → ~0,1 s: corrige o escorregamento rápido, mas deixa passar
//                  mais trancos do motor para o ângulo.
//     peso 0,995 → ~1 s:   ângulo bem liso, mas um giroscópio que escorrega
//                  demora a ser corrigido.
//     peso 1     → nunca corrige: o ângulo escorrega até o robô cair.

void atualizarAngulo(const LeituraDoSensor &leitura, float segundosDesdeOUltimoCiclo) {
  float anguloPeloAcelerometro = calcularAnguloPeloAcelerometro(leitura);

  // Com o sensor deitado e o X para a frente, tombar para a frente é girar em
  // torno do eixo Y do sensor.
  velocidadeAngular = leitura.giroY / UNIDADES_POR_GRAU_POR_SEGUNDO - desvioDoGiroscopio;
  if (INVERTER_SENTIDO_DO_ANGULO) {
    velocidadeAngular = -velocidadeAngular;
  }

  float anguloPeloGiroscopio = anguloAtual + velocidadeAngular * segundosDesdeOUltimoCiclo;

  anguloAtual = pesoDoGiroscopio         * anguloPeloGiroscopio
              + (1.0 - pesoDoGiroscopio) * anguloPeloAcelerometro;
}

float calcularAnguloPeloAcelerometro(const LeituraDoSensor &leitura) {
  // Parado e reto, o acelerômetro sente toda a gravidade no eixo Z. Quando o
  // robô tomba para a frente, parte dela "passa" para o eixo X. O arco-tangente
  // da relação entre as duas partes é o ângulo de inclinação.
  float angulo = atan2(-(float)leitura.acelX, (float)leitura.acelZ) * RAD_TO_DEG;

  if (INVERTER_SENTIDO_DO_ANGULO) {
    angulo = -angulo;
  }
  return angulo;
}

// Todo giroscópio marca uma pequena velocidade mesmo parado. Medimos esse
// desvio ao ligar, com o robô imóvel, e descontamos dele em todas as leituras.
void calibrarGiroscopio() {
  const int QUANTIDADE_DE_AMOSTRAS = 400;   // 400 amostras × 5 ms = 2 segundos

  Serial.println(F("# Calibrando o giroscopio: deixe o robo PARADO por 2 segundos..."));

  float soma = 0.0;
  int leiturasValidas = 0;

  for (int i = 0; i < QUANTIDADE_DE_AMOSTRAS; i++) {
    LeituraDoSensor leitura;
    // Espera cada medição nova, para não somar a mesma leitura duas vezes.
    if (esperarAmostraNova() && lerMPU6050(leitura)) {
      soma += leitura.giroY / UNIDADES_POR_GRAU_POR_SEGUNDO;
      leiturasValidas++;
    }
    if (i % 20 == 0) {
      digitalWrite(PINO_LED, !digitalRead(PINO_LED));   // pisca enquanto calibra
    }
  }

  if (leiturasValidas > 0) {
    desvioDoGiroscopio = soma / leiturasValidas;
  }
  digitalWrite(PINO_LED, LOW);

  Serial.print(F("# Desvio do giroscopio: "));
  Serial.print(desvioDoGiroscopio, 3);
  Serial.println(F(" graus/s"));
}


// =============================================================================
//  PID
// =============================================================================
//
//  Entra: o ângulo atual (já filtrado) e a velocidade com que ele muda.
//  Sai:   um comando para o motor, de -255 (ré total) a +255 (frente total).

float calcularPID(float segundosDesdeOUltimoCiclo) {
  // ERRO: quantos graus estamos longe do equilíbrio.
  // Positivo = tombando para a frente = as rodas precisam ir para a frente
  // para "passar por baixo" do robô. Por isso o erro é (atual − alvo), e não
  // (alvo − atual) como costuma aparecer nos livros: assim a correção já sai
  // com o sinal certo e todos os ganhos ficam positivos.
  float erro = anguloAtual - anguloDeEquilibrio;

  // P — PROPORCIONAL: quanto maior a inclinação, mais força.
  float termoP = Kp * erro;

  // I — INTEGRAL: vai somando o erro ao longo do tempo. Guardamos a soma já
  // multiplicada pelo Ki, assim mudar o Ki pela serial não dá um solavanco.
  // O limite impede que a soma cresça sem fim (o chamado "anti-windup").
  termoIntegral += Ki * erro * segundosDesdeOUltimoCiclo;
  termoIntegral = constrain(termoIntegral, -LIMITE_DO_TERMO_INTEGRAL, LIMITE_DO_TERMO_INTEGRAL);

  // D — DERIVATIVO: reage à velocidade com que o erro muda. Como o alvo é
  // fixo, essa velocidade é exatamente o que o giroscópio mede. Usar o
  // giroscópio direto dá um sinal muito mais limpo do que fazer
  // (erro atual − erro anterior) / tempo, que amplifica o ruído.
  float termoD = Kd * velocidadeAngular;

  ultimoTermoP = termoP;
  ultimoTermoD = termoD;

  float comando = termoP + termoIntegral + termoD;
  return constrain(comando, -PWM_MAXIMO, PWM_MAXIMO);
}

void zerarPID() {
  termoIntegral = 0.0;
}


// =============================================================================
//  MOTOR — MX1508
// =============================================================================
//
//  O MX1508 não tem pino "enable": cada motor usa dois pinos de entrada, e é a
//  combinação deles que decide o sentido.
//
//      IN1    IN2    motor
//      PWM    0      gira para a frente, com força proporcional ao PWM
//      0      PWM    gira para trás
//      0      0      solto (roda livre)

void acionarMotor(float comando) {
  ultimoComando = comando;

  if (INVERTER_SENTIDO_DO_MOTOR) {
    comando = -comando;
  }

  int forca = (int)fabs(comando);   // de 0 a 255, sem o sinal
  if (forca == 0) {
    pararMotor();
    return;
  }

  // Zona morta: espalha 1..255 dentro de pwmMinimo..255, para que até o
  // menor comando já faça a roda se mexer.
  int pwm = map(forca, 1, PWM_MAXIMO, pwmMinimo, PWM_MAXIMO);

  if (comando > 0) {
    analogWrite(PINO_MOTOR_IN1, pwm);   // para a frente
    analogWrite(PINO_MOTOR_IN2, 0);
  } else {
    analogWrite(PINO_MOTOR_IN1, 0);     // para trás
    analogWrite(PINO_MOTOR_IN2, pwm);
  }
}

void pararMotor() {
  analogWrite(PINO_MOTOR_IN1, 0);
  analogWrite(PINO_MOTOR_IN2, 0);
  ultimoComando = 0.0;
}


// =============================================================================
//  MPU6050 — conversa pelo I2C
// =============================================================================

// Grava um valor num registrador. Devolve true se o sensor confirmou.
bool escreverRegistrador(uint8_t registrador, uint8_t valor) {
  Wire.beginTransmission(ENDERECO_MPU6050);
  Wire.write(registrador);
  Wire.write(valor);
  return Wire.endTransmission() == 0;   // 0 = recebido com sucesso
}

bool iniciarMPU6050() {
  // O MPU6050 liga "dormindo". Acordamos e mandamos usar o relógio do
  // giroscópio, que é mais estável que o oscilador interno.
  if (!escreverRegistrador(REG_ENERGIA, 0x01)) {
    return false;
  }
  delay(100);

  bool tudoCerto = true;

  // Filtro passa-baixas interno de ~44 Hz: tira boa parte da vibração do motor.
  tudoCerto &= escreverRegistrador(REG_FILTRO_PASSA_BAIXAS, 0x03);

  // Com esse filtro o sensor mede 1000 vezes por segundo; dividindo por 5
  // chegamos a 200 por segundo, o mesmo ritmo do nosso controle.
  tudoCerto &= escreverRegistrador(REG_DIVISOR_DE_AMOSTRAGEM, 4);

  // Giroscópio em ±500 graus/s e acelerômetro em ±2 g.
  tudoCerto &= escreverRegistrador(REG_ESCALA_GIROSCOPIO, 0x08);
  tudoCerto &= escreverRegistrador(REG_ESCALA_ACELEROMETRO, 0x00);

  // Pino INT: sobe para nível alto num pulso curto (50 µs) a cada medição nova.
  // Nesse modo o pulso vem mesmo que a leitura anterior tenha se perdido; no
  // modo "travado" o pino ficaria alto esperando uma leitura, e o robô, parado.
  tudoCerto &= escreverRegistrador(REG_CONFIGURACAO_DO_INT, 0x00);
  // Liga só o aviso de "dado pronto" (DATA_RDY_EN).
  tudoCerto &= escreverRegistrador(REG_INTERRUPCOES_LIGADAS, 0x01);

  return tudoCerto;
}

// Chamada pela interrupção a cada pulso do INT. Tem que ser rápida: só anota
// que chegou e quando. A leitura pelo I2C fica para o loop(), porque o Wire
// depende de outras interrupções que ficam paradas enquanto esta roda.
void receberAvisoDoMPU6050() {
  instanteDaAmostraNova = micros();
  chegouAmostraNova = true;
}

// Devolve true se chegou amostra nova desde a última consulta, e o instante
// em que ela chegou. As interrupções ficam paradas por um instante durante a
// cópia: sem isso, um aviso no meio poderia deixar os dois valores trocados.
bool pegarAmostraNova(unsigned long &instante) {
  noInterrupts();
  bool chegou = chegouAmostraNova;
  instante = instanteDaAmostraNova;
  chegouAmostraNova = false;
  interrupts();
  return chegou;
}

// Espera o próximo aviso do MPU6050. Devolve false se ele não vier em 100 ms.
bool esperarAmostraNova() {
  unsigned long inicio = millis();
  unsigned long instante;
  while (!pegarAmostraNova(instante)) {
    if (millis() - inicio > 100) {
      return false;
    }
  }
  return true;
}

// Lê acelerômetro e giroscópio de uma vez. Devolve false se algo falhar.
bool lerMPU6050(LeituraDoSensor &leitura) {
  // Os dados ficam em 14 bytes seguidos a partir do registrador 0x3B, nesta
  // ordem (2 bytes cada): acelX, acelY, acelZ, temperatura, giroX, giroY, giroZ.
  Wire.beginTransmission(ENDERECO_MPU6050);
  Wire.write(REG_PRIMEIRO_DADO);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom(ENDERECO_MPU6050, (uint8_t)14) != 14) {
    return false;
  }

  leitura.acelX = lerDoisBytes();
  leitura.acelY = lerDoisBytes();
  leitura.acelZ = lerDoisBytes();
  lerDoisBytes();                      // temperatura: não usamos
  leitura.giroX = lerDoisBytes();
  leitura.giroY = lerDoisBytes();
  leitura.giroZ = lerDoisBytes();
  return true;
}

// Cada valor chega em 2 bytes, o mais significativo primeiro.
int16_t lerDoisBytes() {
  uint8_t byteAlto  = Wire.read();
  uint8_t byteBaixo = Wire.read();
  return (int16_t)((byteAlto << 8) | byteBaixo);
}


// =============================================================================
//  SERIAL — ajuste ao vivo e telemetria
// =============================================================================

// Junta as letras que chegam até formar uma linha, e então executa o comando.
void lerComandosDaSerial() {
  while (Serial.available() > 0) {
    char letra = Serial.read();

    if (letra == '\n' || letra == '\r') {
      if (tamanhoDoBuffer > 0) {
        bufferDaSerial[tamanhoDoBuffer] = '\0';
        executarComando(bufferDaSerial);
        tamanhoDoBuffer = 0;
      }
    } else if (tamanhoDoBuffer < (int)sizeof(bufferDaSerial) - 1) {
      bufferDaSerial[tamanhoDoBuffer] = letra;
      tamanhoDoBuffer++;
    }
  }
}

// Recebe uma linha como "kp 25" ou "marcar_equilibrio" e faz o que ela pede.
void executarComando(char *linha) {
  // Separa a linha no primeiro espaço: "kp 25" vira nome "kp" e valor "25".
  char *nome = linha;
  char *valorEmTexto = strchr(linha, ' ');
  bool veioComValor = false;
  float valor = 0.0;

  if (valorEmTexto != NULL) {
    *valorEmTexto = '\0';          // o nome termina onde estava o espaço
    valorEmTexto++;                // e o valor começa logo depois
    veioComValor = (strpbrk(valorEmTexto, "0123456789") != NULL);
    valor = atof(valorEmTexto);
  }

  // Comandos que mudam um número precisam vir com o número. Sem ele, avisamos
  // em vez de zerar o valor sem querer.
  bool precisaDeValor = nomeDoComandoE(nome, "kp") ||
                        nomeDoComandoE(nome, "ki") ||
                        nomeDoComandoE(nome, "kd") ||
                        nomeDoComandoE(nome, "angulo_equilibrio") ||
                        nomeDoComandoE(nome, "pwm_minimo") ||
                        nomeDoComandoE(nome, "angulo_queda") ||
                        nomeDoComandoE(nome, "peso_giroscopio");
  if (precisaDeValor && !veioComValor) {
    Serial.print(F("# Faltou o numero depois de \""));
    Serial.print(nome);
    Serial.println(F("\". Exemplo: kp 20"));
    return;
  }

  if (nomeDoComandoE(nome, "kp")) {
    Kp = valor;
  } else if (nomeDoComandoE(nome, "ki")) {
    Ki = valor;
  } else if (nomeDoComandoE(nome, "kd")) {
    Kd = valor;
  } else if (nomeDoComandoE(nome, "angulo_equilibrio")) {
    anguloDeEquilibrio = valor;
  } else if (nomeDoComandoE(nome, "marcar_equilibrio")) {
    anguloDeEquilibrio = anguloAtual;
  } else if (nomeDoComandoE(nome, "pwm_minimo")) {
    pwmMinimo = constrain((int)valor, 0, PWM_MAXIMO - 1);
  } else if (nomeDoComandoE(nome, "angulo_queda")) {
    anguloDeQueda = constrain(valor, ANGULO_DE_QUEDA_MINIMO, ANGULO_DE_QUEDA_MAXIMO);
  } else if (nomeDoComandoE(nome, "peso_giroscopio")) {
    pesoDoGiroscopio = constrain(valor, 0.0, 1.0);
  } else if (nomeDoComandoE(nome, "telemetria")) {
    telemetriaLigada = !telemetriaLigada;
  } else if (nomeDoComandoE(nome, "valores")) {
    // nada a mudar: só mostra os valores logo abaixo
  } else if (nomeDoComandoE(nome, "ajuda")) {
    mostrarAjuda();
    return;
  } else {
    Serial.print(F("# Comando desconhecido: \""));
    Serial.print(nome);
    Serial.println(F("\""));
    mostrarAjuda();
    return;
  }

  mostrarAjustes();
}

// Compara o nome digitado com o esperado, sem ligar para maiúsculas.
bool nomeDoComandoE(const char *nomeDigitado, const char *nomeEsperado) {
  return strcasecmp(nomeDigitado, nomeEsperado) == 0;
}

void mostrarAjuda() {
  Serial.println(F("# Comandos:"));
  Serial.println(F("#   kp 20                   ganho proporcional"));
  Serial.println(F("#   ki 40                   ganho integral"));
  Serial.println(F("#   kd 0.8                  ganho derivativo"));
  Serial.println(F("#   angulo_equilibrio -1.5  angulo em que o robo fica em pe"));
  Serial.println(F("#   marcar_equilibrio       usa o angulo atual como equilibrio"));
  Serial.println(F("#   pwm_minimo 40           menor PWM que faz o motor girar"));
  Serial.println(F("#   angulo_queda 35         inclinacao a partir da qual o robo caiu (5 a 80)"));
  Serial.println(F("#   peso_giroscopio 0.98    quanto o filtro confia no giroscopio (0 a 1)"));
  Serial.println(F("#   telemetria              liga/desliga o envio de numeros"));
  Serial.println(F("#   valores                 mostra os valores atuais"));
  Serial.println(F("#   ajuda                   mostra esta lista"));
}

// O programa ferramentas/pid-robot lê esta linha e a da telemetria: se mudar o
// formato de alguma delas, ajuste o programa também.
void mostrarAjustes() {
  Serial.print(F("# kp="));                  Serial.print(Kp, 2);
  Serial.print(F("  ki="));                  Serial.print(Ki, 2);
  Serial.print(F("  kd="));                  Serial.print(Kd, 3);
  Serial.print(F("  angulo_equilibrio="));   Serial.print(anguloDeEquilibrio, 2);
  Serial.print(F("  pwm_minimo="));          Serial.print(pwmMinimo);
  Serial.print(F("  angulo_queda="));        Serial.print(anguloDeQueda, 1);
  Serial.print(F("  peso_giroscopio="));     Serial.print(pesoDoGiroscopio, 3);
  Serial.print(F("  telemetria="));          Serial.println(telemetriaLigada ? F("ligada") : F("desligada"));
}

// Formato "nome:valor,nome:valor" — abre direto no Plotter Serial da IDE.
void enviarTelemetria() {
  if (!telemetriaLigada) return;
  if (millis() - instanteDaUltimaTelemetria < INTERVALO_DA_TELEMETRIA_MS) return;
  instanteDaUltimaTelemetria = millis();

  Serial.print(F("angulo:"));              Serial.print(anguloAtual, 2);
  Serial.print(F(",angulo_equilibrio:"));  Serial.print(anguloDeEquilibrio, 2);
  Serial.print(F(",termo_P:"));            Serial.print(ultimoTermoP, 1);
  Serial.print(F(",termo_I:"));            Serial.print(termoIntegral, 1);
  Serial.print(F(",termo_D:"));            Serial.print(ultimoTermoD, 1);
  Serial.print(F(",comando_motor:"));      Serial.println(ultimoComando, 0);
}
