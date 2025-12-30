# Mega2 Debug-Level: SIM vs. HW (Elektrische Eisenbahn)

Diese Datei beschreibt das neue **2‑Level Debug-/Betriebsmodell** für **Mega2**:

- **SIM** (*ohne Hardware / rein virtuell*): Sensoren, Belegung und Strom lassen sich per Serial‑Commands simulieren. Zusätzlich kann „Trafo unten powered“ per Debug erzwungen werden.
- **HW** (*mit Hardware / Anlage*): Simulation ist **deaktiviert**, damit im Realbetrieb keine „virtuellen“ Eingriffe passieren. Diagnose/Power/Safety bleiben nutzbar.

Ziel: **deterministisches Testen** (SIM) und **sicherer Realbetrieb** (HW) – ohne Seiteneffekte.

---

## 1. Begriffe & Flags

### Build-Flags
- `MEGA2_DEBUG`  
  Aktiviert Debug-Ausgaben und Serial-Debug-Interface (grundsätzlich).

- `MEGA2_SIM_MODE`  
  Umschalter für Simulation:
  - `1` = SIM (Debug-Kommandos zur Simulation erlaubt)
  - `0` = HW (Simulation deaktiviert)

### SysFlags-Ausgabe
Die Zeile `SYS flags=...` wird **nur bei Änderung** ausgegeben und im Format **HEX**:

Beispiel: `SYS flags=0x0065`

Damit wird das Serial-Log deutlich übersichtlicher.

---

## 2. PlatformIO: Environments

Empfohlen: zwei Environments in `platformio.ini`:

- `mega2-sim`  → `MEGA2_SIM_MODE=1`
- `mega2-hw`   → `MEGA2_SIM_MODE=0`

### Build / Upload (CLI)
```bash
pio run -e mega2-sim
pio run -e mega2-hw

pio run -e mega2-sim -t upload
pio run -e mega2-hw  -t upload

pio device monitor -e mega2-sim
pio device monitor -e mega2-hw
```

### VS Code / PlatformIO UI
Im PlatformIO-Tab das jeweilige Environment auswählen (SIM oder HW) und dann **Build/Upload/Monitor** starten.

---

## 3. Serial Debug Commands

> **Wichtig:** In **HW** sind SIM-Commands absichtlich deaktiviert.
> Dadurch sind Fehlbedienungen im Realbetrieb ausgeschlossen.

### 3.1 Commands (immer erlaubt, SIM & HW)

| Command | Funktion |
|---|---|
| `p` | Power ON (Hauptstrom) |
| `n` | Power OFF (Hauptstrom) |
| `a` | ACK (Safety-Reset, falls möglich) |
| `d` | Diagnose-Dump (Blocks, Safety, SBHF-Status) |
| `t` | Trafo-Diagnose: Vrms + `powered` (oben/unten) + Force-Status (Trafo unten) |

> Hinweis: `a` kann **ACK OK** oder **ACK BLOCKED** melden (siehe Kapitel 5).

### 3.2 Commands (nur SIM)

| Command | Funktion |
|---|---|
| `T` | Toggle: **Trafo unten „powered“ erzwingen** (`FORCE ON/OFF`) |
| `1..6` | SBHF Sensor-Events (S11..S16) simulieren |
| `r` | SBHF Reset/Ack (nur SBHF-State) |
| `o<id>` | Block `<id>` **occupied = true** (z.B. `o6`) |
| `O<id>` | Block `<id>` **occupied = false** (z.B. `O6`) |
| `i<id>` | Block `<id>` **strom aktiv** (z.B. `i6`) |
| `I<id>` | Block `<id>` **strom inaktiv** (z.B. `I6`) |

Beispiele:
- `o6` → Block 6 belegt (Kontaktgleis belegt)
- `I6` → Block 6 „kein Strom“

### 3.3 Parser-Hinweis (wichtig)
Der Serial-Parser ist so angepasst, dass **`o6`/`I6` nicht mehr versehentlich `6` als Single-Key (S16) auslöst**.
Damit sind Debug-Tests reproduzierbar.

---

## 4. SBHF: Nothaltgleis (S15 / S16)

Die Sensoren S15/S16 sind **kein “Not-Aus Taster”**, sondern steuern das **Nothaltgleis** (Stopzone) im Block 6:

- `S15` → **Nothalt frei** → Nothaltgleis **EIN** (Zug kann passieren)
- `S16` → **Nothalt aktiv** → Nothaltgleis **AUS** (Stopzone scharf)

Im Serial-Debug (SIM) entsprechen dem:
- `5` → S15
- `6` → S16

---

## 5. Servicefall: EMERG_NOTHALT_SBHF (Reverse-Entry Block 6)

### 5.1 Auslösebedingung
Der Safety-Trigger **EMERG_NOTHALT_SBHF** wird ausgelöst, wenn:

- **Trafo unten powered** (`g_trafoUnten.isPowered()`), *oder im SIM per `T FORCE ON`*
- **Nothalt aktiv** (Stopzone = AUS)
- **Block 6 belegt** (Kontaktgleis)
- **kein Strom in Block 6** (SIM: `I6` bzw. initial 0)

### 5.2 Wirkung
- SSR für Trafo unten (**SSR_B**) wird **OFF**
- **SafetyLock** wird gesetzt (latched)
- Serial: `[SAFETY] EMERG_NOTHALT_SBHF -> SSR_B OFF, LOCK`

### 5.3 ACK-Blockade (wichtig)
ACK (`a`) hebt den Lock **nicht** sofort auf.

ACK wird **BLOCKED**, solange **mindestens eine** der Bedingungen gilt:
- Block 6 noch belegt (`o6` nicht aufgehoben durch `O6`)
- Nothalt noch aktiv (Stopzone noch AUS, erst `S15`/`5` muss Freigabe setzen)

Beispielmeldung:
`[SAFETY] ACK blocked (EMERG_NOTHALT_SBHF): Block6 still occupied Nothalt still active`

### 5.4 SIM-Testablauf (ohne Hardware)

1. `a` → Boot-Lock weg (ACK OK)
2. `T` → Trafo unten FORCE ON
3. `5` → Nothalt frei
4. `p` → Power ON
5. `6` → Nothalt aktiv (Stopzone AUS)
6. `o6` → Block 6 belegt  → **Lock muss kommen**
7. `a` → **ACK BLOCKED**
8. `O6` → Block 6 frei
9. `5` → Nothalt frei
10. `a` → **ACK OK**
11. `p` → Power ON OK
12. `T` → FORCE OFF (Gegencheck: dann darf es nicht mehr auslösen)

---

## 6. Empfehlung für Workflow

### 6.1 Entwicklung (ohne Anlage)
- Environment: **`mega2-sim`**
- Safety/Logik entwickeln, alles über Serial testen.
- Danach committen.

### 6.2 Abnahme (mit Anlage)
- Environment: **`mega2-hw`**
- SIM-Commands sind deaktiviert, nur echte Sensorik zählt.
- Abnahme: Trigger/Reset/Power-Verhalten mit echter Trafo-Spannung und echten Sensoren prüfen.

### 6.3 Git
Nach größeren Safety-Schritten (z.B. EMERG_NOTHALT_SBHF, Parser, Debug-Level) immer:
```bash
git status
git add -A
git commit -m "Mega2: <kurze Beschreibung>"
git push
```

---

## 7. Nächster Schritt: Block-Safety (Plan)

Sobald SIM/HW sauber ist (diese Datei), folgt Block-Safety Phase 1 (SIM-testbar):
1. **EMERG_SHORT_CONFIRMED_BLOCK_x** (über Short-Event/Debug) → Trafo-SSR OFF + Lock
2. **EMERG_SSR_STUCK_ON_TRAFO_x** (SSR OFF befohlen, aber Strom bleibt) → Lock

Messwerte (mA/Thresholds) werden danach in Phase 2 mit echter Hardware finalisiert.

---


---

## 8. Hardware-Abnahme morgen (HW-Modus) – Checkliste

Diese Schritte sind für die Abnahme **an der Anlage** gedacht, wenn echte Sensorik angeschlossen ist.

### 8.1 Vorbereitung
1. In PlatformIO das Environment **`mega2-hw`** auswählen (oder `MEGA2_SIM_MODE=0` setzen).
2. Flashen + Serial Monitor starten.
3. Hinweis: In HW sind SIM-Commands (z. B. `T`, `o6`, `I6`, `1..6`) **deaktiviert**. Das ist Absicht.

### 8.2 Grunddiagnose: Trafosensoren (ZMPT101B)
Im Serial Monitor:

- `t`

Erwartung:
- **Trafo oben/unten** zeigen sinnvolle `Vrms`-Werte.
- `powered=YES`, wenn der jeweilige Trafo eingeschaltet ist.
- Bei ausgeschaltetem Trafo: `powered=NO`, Vrms nahe 0 (typisch wenige 0.01 V).

**Wenn `powered` falsch ist:**
- Prüfe Verkabelung (ZMPT-Modul, ADC-Pin, GND).
- Prüfe, ob der ADC-Pin “floating” ist (saubere Masseführung).
- Der Default-Schwellwert in `SensorTrafoAC` ist i. d. R. **~2.0 Vrms** (siehe `m_powerThreshold`). Bei Bedarf später feinjustieren.

### 8.3 Abnahmetest: EMERG_NOTHALT_SBHF (Realbetrieb)
Ziel: Safety-Lock soll **nur** im echten Servicefall auslösen.

**A) Normalfall (darf NICHT locken)**
1. Zug fährt regulär in Block 6 (korrekte Richtung).
2. S15 schaltet Nothaltgleis **EIN** → Zug kann weiterfahren.
3. S16 schaltet Nothaltgleis **AUS** (Stopzone scharf).
4. Ergebnis: **kein** Safety-Lock.

**B) Servicefall / Gegenrichtung (MUSS locken)**
Voraussetzung: Trafo unten ist **an** (ZMPT unten `powered=YES`).

1. Block 6 ist **belegt** (Kontaktgleis meldet belegt).
2. Nothalt ist **aktiv** (Stopzone AUS) – in echt: S16 wurde ausgelöst.
3. Es fließt **kein Strom** in Block 6 (typisch: Zug steht gegen Stopzone / falsche Richtung, Traktion bekommt keinen Strom).
4. Ergebnis: **EMERG_NOTHALT_SBHF** → SSR_B OFF + LOCK.

**Wichtig:** Der Lock ist **latched**. Er bleibt aktiv, bis der Bediener den Servicefall wirklich beseitigt.

### 8.4 Abnahmetest: ACK-Blockade (Service muss wirklich erledigt sein)
Wenn der Lock aktiv ist:

1. `a` (ACK) drücken.
   - Erwartung: ACK wird **BLOCKED**, solange Block 6 belegt **oder** Nothalt noch aktiv ist.
2. Service erledigen:
   - Block 6 freimachen (Zug entfernen oder aus dem Block herausfahren).
   - Nothalt wieder freigeben (im Ablauf normalerweise über S15 / Freigabe).
3. `a` erneut:
   - Erwartung: **ACK OK**, Lock wird aufgehoben.
4. `p`:
   - Erwartung: Power ON wieder möglich.

### 8.5 Typische Fehlerbilder & schnelle Diagnose
- **Lock triggert zu früh / Phantom-Lock:**  
  `t` prüfen: Ist `Trafo unten powered=YES` obwohl Trafo aus ist? → Threshold/Noise/ADC floating.
- **Lock triggert nie:**  
  `t` prüfen: Kommt `powered=YES` wenn Trafo wirklich an ist? Falls nein → ZMPT/ADC prüfen.
- **ACK bleibt blockiert obwohl Service erledigt:**  
  Prüfe, ob Block 6 wirklich “frei” meldet (Kontaktgleis) und ob Nothalt wirklich “frei” ist (Stopzone EIN).

---


## Änderungslog (kurz)
- 2 Debug-Level: `MEGA2_SIM_MODE` (SIM vs HW)
- Serial-SIM-Commands im HW-Modus deaktiviert
- `SYS flags` nur on-change, Ausgabe in HEX
- Trafo unten: Debug-Force via `T` (SIM)
- EMERG_NOTHALT_SBHF: Lock + ACK-Blockade bis Service erledigt
