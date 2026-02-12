# Mega2 Debug-Level: SIM vs. HW (Elektrische Eisenbahn)

Diese Datei beschreibt das 2-Level Debug-/Betriebsmodell für Mega2
inklusive Analog-Sensorik (Strom & Trafo).

---

# 1. Betriebsmodi

Mega2 kennt zwei klare Betriebsarten:

- **SIM (Simulation)**  
  Keine reale Anlage nötig. Sensorik, Belegung und Strom können per Serial simuliert werden.

- **HW (Hardware)**  
  Reale Anlage aktiv. Simulation ist deaktiviert. Nur echte Sensorwerte zählen.

Ziel:
✔ deterministisches Testen im SIM  
✔ sicherer Realbetrieb im HW  
✔ keine unbeabsichtigten Eingriffe im Echtbetrieb  

---

# 2. Build-Flags

## 2.1 Grundlegende Flags

| Flag | Bedeutung |
|------|-----------|
| `MEGA2_DEBUG` | Aktiviert Debug-Ausgaben + Serial-Interface |
| `MEGA2_SIM_MODE` | 1 = SIM / 0 = HW |

---

## 2.2 Neues Analog-Debug-Flag

| Flag | Bedeutung |
|------|-----------|
| `MEGA2_DEBUG_ANALOG_TICK` | Gibt 1×/Sekunde Analog-Messwerte aus |

Empfohlen nur temporär aktivieren.

Beispiel `platformio.ini`:

```ini
build_flags =
  -DMEGA2_DEBUG=1
  -DMEGA2_DEBUG_ANALOG_TICK=1

3. Sensorik-Architektur
3.1 Stromsensoren (ZMCT103C)

Typ:

Stromwandler 1000:1

AC-only

5A Nennbereich

sehr kleine Signalpegel bei 200–500mA

Messprinzip:

ADC-Abtastung

Offset-Tracking

RMS über Abweichung

Threshold-Vergleich (overThreshold())

Wichtig:

Default-Threshold wurde auf 8 Counts gesetzt

typische sinnvolle Range: 5–15 Counts

keine mA-Kalibrierung im ersten Schritt nötig

Block-Belegung nutzt stromAktiv() (bool)

Signalpfad:
SensorStrom.update()
   → RMS-Bildung
   → overThreshold()
   → Block.stromAktiv()
   → BlockController
   → Payload
   → ESP

3.2 Trafo-Spannung (ZMPT101B)

Typ:

AC-Spannungswandler

Peak-to-Peak Messung

RMS-Annäherung via Vpp * 0.35355

Exponentielle Glättung

API:

rms() → Spannung in Volt

isPowered() → Trafo EIN/AUS (Threshold)

Primär genutzt für:

Power-Freigabe

Safety-Bedingungen

Diagnose

4. Serial Debug Commands
4.1 Immer erlaubt (SIM & HW)
Command	Funktion
p	Power ON
n	Power OFF
a	ACK
d	Diagnose Dump
t	Trafo Diagnose
4.2 Nur im SIM-Modus
Command	Funktion
T	Trafo unten FORCE ON/OFF
o<id>	Block belegt
O<id>	Block frei
i<id>	Strom aktiv
I<id>	Strom inaktiv
k<id>	Kurzschluss auslösen
5. Neues Analog-Debug (1×/s)

Bei aktiviertem MEGA2_DEBUG_ANALOG_TICK erscheint:

[AN] I1 raw=xxx off=xxx rms=xx act=x |
     ISB1 raw=xxx off=xxx rms=xx act=x |
     TOben=xx.xV pow=x |
     TUnten=xx.xV pow=x

Bedeutung:

raw = ADC Rohwert

off = Offset

rms = RMS Counts

act = overThreshold()

TOben/TUnten = Vrms

pow = isPowered()

Zweck:

Threshold-Feintuning

Erkennen von Noise

Diagnose bei Fehltriggern

Empfehlung:
Nach erfolgreichem Abgleich wieder deaktivieren.

6. Safety: EMERG_NOTHALT_SBHF

Unverändert gültig:

Auslösung wenn:

Trafo unten powered

Nothalt aktiv

Block 6 belegt

kein Strom

Wirkung:

SSR OFF

Lock gesetzt

ACK bleibt blockiert,
bis Block frei + Nothalt frei.

7. Empfohlener Workflow
Entwicklung

mega2-sim

Safety + Blocklogik testen

Threshold anpassen

commit

Abnahme

mega2-hw

reale Stromwerte prüfen

Analog-Debug temporär aktivieren

finale Threshold fixieren

8. Nächste Safety-Phase

Geplant:

EMERG_SHORT_CONFIRMED_BLOCK_x

EMERG_SSR_STUCK_ON

Echte mA-basierte Kurzschluss-Schwellen