# Modelo De Controle

O objetivo e fazer o comportamento de um rebocador azimutal sem precisar de servo posicional 360 graus.

## Entrada

O firmware usa dois comandos normalizados:

```text
rudder   = -1.0 .. +1.0
throttle = -1.0 .. +1.0
```

O leme vira deflexao:

```text
deflection = rudder * 45 graus
```

## Frente

Com acelerador positivo:

```text
pod = 45 graus + deflection
motor = abs(throttle)
```

Faixa resultante:

```text
0 a 90 graus
```

## Re

Com acelerador negativo:

```text
pod = 225 graus - deflection
motor = abs(throttle)
```

Faixa resultante:

```text
180 a 270 graus
```

## Por Que O Motor Usa Abs(throttle)?

Porque o propulsor azimutal faz a re mudando a direcao do empuxo. O motor pode girar sempre no mesmo sentido e o pod aponta o empuxo para onde voce quer.

Essa abordagem e amigavel para ESCs simples e reduz a chance de confusao com marcha a re eletronica.

## Ajustes Futuros

Ideias naturais para evoluir:

- terceiro canal para modo manual/automatico
- mistura para dois propulsores azimutais
- leitura iBUS em vez de PWM por canal
- calibracao por Serial
- EEPROM para salvar ajustes sem recompilar
