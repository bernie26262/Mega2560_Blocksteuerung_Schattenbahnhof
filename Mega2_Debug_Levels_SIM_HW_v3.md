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
| `k<id>` | **Kurzschluss** am Block `<id>` auslösen (z.B. `k6`) → Safety-Lock + SSR AUS |

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

## Änderungslog (kurz)
- 2 Debug-Level: `MEGA2_SIM_MODE` (SIM vs HW)
- Serial-SIM-Commands im HW-Modus deaktiviert
- `SYS flags` nur on-change, Ausgabe in HEX
- Trafo unten: Debug-Force via `T` (SIM)
- EMERG_NOTHALT_SBHF: Lock + ACK-Blockade bis Service erledigt


## Zusatz: SSR-Force (nur SIM)

- `x`  → SSR_B FORCE OFF
- `X`  → SSR_B FORCE ON
- `y`  → SSR_A FORCE OFF
- `Y`  → SSR_A FORCE ON

Damit kann man die SSR-Stuck-Detektion im SIM-Mode testen (SSR AUS, Trafo-Spannung bleibt anliegend).
