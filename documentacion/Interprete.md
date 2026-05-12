# Subsistema — Intérprete

## Descripción

> _Función del intérprete: nodo maestro del bus CAN, recibe comandos externos y los distribuye a los subsistemas._

## Modos de firmware

| Entorno PlatformIO    | Uso                              |
|-----------------------|----------------------------------|
| `interprete_can`      | Integración completa por CAN bus |
| `interprete_serial`   | Calibración y pruebas por Serial |

## Hardware

| Componente | Pin / Interfaz | Notas |
|------------|----------------|-------|
|            |                |       |

## IDs CAN que genera

| ID (hex) | Destino      | Descripción |
|----------|--------------|-------------|
| 0x210    | Broadcast    | Paro emergencia |
|          |              |             |

## IDs CAN que escucha

| ID (hex) | Origen       | Descripción |
|----------|--------------|-------------|
|          |              |             |

## Fuente de comandos

> _De dónde recibe instrucciones el intérprete: Serial, RC, ROS, joystick, etc._

## Lógica de arbitraje / emergencia

> _Cómo decide cuándo enviar el paro de emergencia, prioridades, timeouts._

## Calibración (modo Serial)

> _Procedimiento de calibración: comandos, valores esperados, cómo verificar._

## Notas

> _Observaciones de hardware, bugs conocidos, revisiones de PCB, etc._
