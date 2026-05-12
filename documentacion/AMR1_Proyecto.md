# AMR1 — Documentación General

## Descripción del proyecto

> _Breve descripción del robot / vehículo AMR1 y su propósito._

## Arquitectura del sistema

> _Diagrama o descripción general de cómo se conectan los subsistemas entre sí._

```
[ Intérprete ] ──CAN──┬── [ Tren Motriz ]
                      ├── [ Freno ]
                      └── [ Dirección ]
```

## Subsistemas

| Subsistema   | Archivo de documentación          |
|--------------|-----------------------------------|
| Tren Motriz  | [Tren_Motriz.md](Tren_Motriz.md)  |
| Freno        | [Freno.md](Freno.md)              |
| Dirección    | [Direccion.md](Direccion.md)      |
| Intérprete   | [Interprete.md](Interprete.md)    |

## Bus CAN — Tabla de IDs

| ID (hex) | Dirección     | Descripción              |
|----------|---------------|--------------------------|
| 0x210    | Broadcast     | Paro de emergencia       |
| 0x300    | → Tren Motriz | Comando PWM              |
| 0x301    | ← Tren Motriz | Feedback PWM             |
| 0x310    | → Tren Motriz | Comando relé             |
| 0x311    | ← Tren Motriz | Feedback relé            |
| 0x320    | → Tren Motriz | Comando dirección        |
| 0x321    | ← Tren Motriz | Feedback dirección       |
|          |               | _(completar otros IDs)_  |

## Hardware común

- **Microcontrolador:** Adafruit Feather RP2040 CAN
- **CAN transceiver:** MCP2515 (integrado en la placa)
- **Pantalla:** OLED SSD1306 128×64 (I2C, 0x3C)
- **Velocidad CAN:** 250 kbps
- **Velocidad Serial:** 115200 baud

## Herramientas y dependencias

- PlatformIO + framework Arduino
- Plataforma: `maxgerhardt/platform-raspberrypi`
- Librerías: Adafruit MCP2515, Adafruit SSD1306, Adafruit GFX, Adafruit BusIO

## Notas generales

> _Decisiones de diseño, restricciones conocidas, versiones de hardware, etc._
