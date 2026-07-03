# Calibracao

Calibre sempre com o motor desconectado.

## 1. Verifique O Receptor

Abra o Serial Monitor em 115200 baud e observe:

```text
rudderUs=1500 throttleUs=1500
```

Valores normais:

- stick no minimo: aproximadamente 1000 us
- stick no centro: aproximadamente 1500 us
- stick no maximo: aproximadamente 2000 us

Se o seu receptor usar limites diferentes, ajuste:

```cpp
const int RC_MIN_US = 1000;
const int RC_CENTER_US = 1500;
const int RC_MAX_US = 2000;
```

## 2. Calibre O Servo 270

O firmware parte de:

```cpp
const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2500;
const float SERVO_TOTAL_DEGREES = 270.0f;
```

Nem todo servo 270 usa exatamente essa faixa. Se ele bater no limite antes do esperado, reduza `SERVO_MIN_US` e `SERVO_MAX_US` para algo mais conservador, como 700 e 2300.

## 3. Ajuste A Geometria

Configuracao padrao:

```cpp
const float POD_LEFT_LIMIT_DEG = 0.0f;
const float POD_FORWARD_CENTER_DEG = 45.0f;
const float POD_REVERSE_CENTER_DEG = 225.0f;
const float POD_RIGHT_LIMIT_DEG = 270.0f;
const float RUDDER_MAX_DEFLECTION_DEG = 45.0f;
```

Isso usa praticamente todo o curso de 270 graus:

- frente: 0 a 90 graus
- re: 180 a 270 graus

Alinhe mecanicamente o pod para que 45 graus seja "frente". Se o seu servo nao gostar dos extremos, reduza `SERVO_MIN_US` e `SERVO_MAX_US` antes de mexer na geometria.

## 4. ESC

Por padrao:

```cpp
const int ESC_STOP_US = 1000;
const int ESC_MAX_US = 2000;
```

Esse modo considera que o ESC recebe apenas comando positivo, e a re vem da rotacao do pod.

Se seu ESC precisa de neutro em 1500 us, adapte `computeTargetEscUs()` para mapear a magnitude para a faixa esperada pelo ESC.
