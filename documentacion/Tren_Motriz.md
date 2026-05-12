# Subsistema — Tren Motriz

## Descripción

> _Función del subsistema dentro del AMR1._

## Modos de firmware

| Entorno PlatformIO    | Uso                              |
|-----------------------|----------------------------------|
| `tren_motriz_can`     | Integración completa por CAN bus |
| `tren_motriz_serial`  | Calibración y pruebas por Serial |

## Hardware

| Componente      | Pin / Interfaz | Notas                        |
|-----------------|----------------|------------------------------|
| Motor PWM       | A1             |                              |
| Relé habilitador| 4              |                              |
| Dirección REV   | 11             |                              |
| CONFIG L1       | 12             | Transistor ON permanente     |
| CONFIG L2       | 13             | Transistor OFF permanente    |
| CAN CS          | PIN_CAN_CS     | Definido por la placa        |

## IDs CAN

| ID (hex) | Dirección | Descripción       |
|----------|-----------|-------------------|
| 0x300    | Entrada   | Comando PWM (%)   |
| 0x301    | Salida    | Feedback PWM      |
| 0x310    | Entrada   | Comando relé      |
| 0x311    | Salida    | Feedback relé     |
| 0x320    | Entrada   | Comando dirección |
| 0x321    | Salida    | Feedback dirección|
| 0x210    | Entrada   | Paro emergencia   |

## Lógica de control

> _Describir rampa PWM, rango de valores (0–100 %), comportamiento en emergencia, etc._

## Calibración (modo Serial)

> _Procedimiento de calibración: comandos, valores esperados, cómo verificar._

## Notas

> _Observaciones de hardware, bugs conocidos, revisiones de PCB, etc._
