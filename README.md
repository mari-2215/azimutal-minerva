# Azimuth Tug Mega

Firmware profissional para controlar um propulsor azimutal de rebocador RC com Arduino Mega, servo posicional de 270 graus e radio FlySky.

Autor: `mari-2215`

O projeto foi pensado para um conjunto onde o pod precisa cobrir:

- 45 graus para bombordo
- 45 graus para boreste
- 180 graus para manobra de re

Em vez de exigir um servo posicional de 360 graus, o firmware usa um servo comum de 270 graus e escolhe automaticamente entre o quadrante de vante e o quadrante de re conforme o comando de aceleracao.

## Recursos

- Leitura PWM direta de receptor RC FlySky ou equivalente.
- Compativel com Arduino Mega 2560.
- Controle de servo azimutal de 270 graus por microssegundos.
- Controle de ESC/motor usando magnitude do acelerador.
- Fail-safe por perda de sinal.
- Suavizacao de servo e acelerador.
- Calibracao centralizada por constantes.
- Codigo separado em funcoes pequenas e comentadas onde importa.

## Hardware Alvo

- Arduino Mega 2560
- Receptor FlySky com saidas PWM por canal
- Radio FlySky, por exemplo FS-i4, FS-i6 ou similar
- Servo posicional 270 graus com engrenagens metalicas
- ESC para motor DC/brushless
- Fonte/BEC adequado para o servo
- Propulsor azimutal mecanico

> Importante: o Arduino nao deve alimentar servo forte diretamente pelo pino 5V. Use BEC/fonte separada com GND comum.

## Mapa De Controle

O firmware assume:

- Canal 1: leme, esquerda/direita
- Canal 2: acelerador, frente/re

Com acelerador para frente:

```text
servo = centro_frente +/- ate 45 graus
motor = proporcional ao acelerador
```

Com acelerador para re:

```text
servo = centro_re +/- ate 45 graus
motor = proporcional ao modulo do acelerador
```

Isso permite usar um ESC configurado para motor sempre em sentido positivo, enquanto a direcao fisica do empuxo vem da rotacao do pod.

## Estrutura

```text
azimuth-tug-mega/
  firmware/
    AzimuthTugMega/
      AzimuthTugMega.ino
  docs/
    calibration.md
    control-model.md
    wiring.md
  .gitignore
  LICENSE
  README.md
```

## Como Usar

1. Abra `firmware/AzimuthTugMega/AzimuthTugMega.ino` na Arduino IDE.
2. Selecione `Arduino Mega or Mega 2560`.
3. Ligue o receptor FlySky aos pinos configurados.
4. Ajuste os limites em `User configuration`.
5. Envie para a placa.
6. Faca a calibracao mecanica com o motor desconectado.

## Pinos Padrao

| Funcao | Pino Mega |
| --- | ---: |
| PWM leme do receptor | 2 |
| PWM acelerador do receptor | 3 |
| Servo azimutal | 9 |
| ESC/motor | 10 |

Os pinos 2 e 3 foram escolhidos porque suportam interrupcao externa no Mega.

## Estado Do Projeto

Versao inicial funcional para bancada. Antes de navegar, valide:

- sentido mecanico do servo
- limites fisicos do pod
- fail-safe do receptor
- corrente do servo sob carga
- estanqueidade do conjunto

## Licenca

MIT. Use, modifique e navegue feliz.
