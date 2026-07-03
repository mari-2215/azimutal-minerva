# Ligacoes

## Arduino Mega

| Funcao | Pino |
| --- | ---: |
| Entrada PWM leme do receptor | D2 |
| Entrada PWM acelerador do receptor | D3 |
| Sinal do servo azimutal | D9 |
| Sinal do ESC | D10 |

## Receptor FlySky

Em radios FlySky, o nome exato depende do kit:

- FS-i4 costuma ser o radio/transmissor.
- O receptor pode ser FS-iA4B, FS-iA6, FS-iA6B ou parecido.

Se o receptor tiver saidas PWM separadas por canal, ligue:

- CH1 no pino D2 do Mega
- CH2 no pino D3 do Mega
- GND do receptor no GND do Mega

## Alimentacao

Use GND comum entre:

- Arduino Mega
- receptor
- BEC/fonte do servo
- ESC

Nao alimente servo de alto torque pelo 5V do Arduino. Um servo de 20 kg.cm pode puxar picos altos de corrente e reiniciar a placa.

## Checklist Antes De Ligar Motor

- Remova a helice ou desconecte o motor.
- Ligue o sistema e veja se o servo centraliza sem bater no fim mecanico.
- Abra o Serial Monitor em 115200 baud.
- Confirme que os pulsos ficam perto de 1000, 1500 e 2000 us.
- Inverta `INVERT_RUDDER_INPUT`, `INVERT_THROTTLE_INPUT` ou `INVERT_SERVO_OUTPUT` se necessario.
