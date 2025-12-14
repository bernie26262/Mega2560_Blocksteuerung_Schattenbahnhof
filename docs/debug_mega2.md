# Mega2 – Debug-System Dokumentation (D2-Stand)

## 1. Zweck des Debug-Systems

Das Debug-System dient dazu,

- interne Zustände transparent zu machen
- die Logik ohne reale Anlage testbar zu halten
- Fehler reproduzierbar zu analysieren
- Software-Fehler klar von Hardware-Fehlern zu trennen
- eine stabile Basis für D3 (I2C / ESP / Web-UI) zu schaffen

Das Debug-System ist **rein diagnostisch**.  
Es verändert **niemals** das Sicherheits- oder Steuerungsverhalten.

---

## 2. Inhalt des Debugs

### 2.1 ShadowYard / Block-bezogene Informationen

Ausgegeben werden u. a.:

- aktueller ShadowYard-State  
  (`Idle`, `PrepareExit`, `SettingWeichen`, `WaitBlock6`, `ExitRunning`, `Error`)
- aktuelles Ausfahr-Gleis
- Weichen-Sequenz:
  - aktueller Index
  - OK-Meldungen
  - Soll/Ist-Abweichungen
  - Timeout
- Hard-Error:
  - Fehlergrund (`HardErrorReason`)
  - betroffene Weiche
- Blockzustände (z. B. Block 6 belegt / frei)

### 2.2 Beispiel: Normalbetrieb

[SBHF] init
[SBHF] S11 -> Gleis 2
[SBHF] Weiche OK Index=0
[SBHF] Weiche OK Index=1
[SBHF] Alle Weichen OK -> WaitBlock6
[SBHF] Block 6 frei -> ExitRunning

shell
Code kopieren

### 2.3 Beispiel: Fehlerfall

[SBHF] WEICHE TIMEOUT Index=1
[SBHF] HARD ERROR, reason=WEICHE_TIMEOUT

yaml
Code kopieren

---

## 3. Serial-Debug vs. Anlagen-Debug

### 3.1 Serial-Debug (ohne Anlage)

**Ziel:** Test der reinen Logik ohne reale Hardware.

Merkmale:
- Ausgabe über USB / Serial Monitor
- Rückmeldungen können simuliert werden
- keine reale Spannung an Gleisen oder Weichen
- ideal für:
  - State-Machine-Tests
  - Weichen-Sequenzen
  - Fehler- und Timeout-Pfade

Typische Umgebung:
- PlatformIO / VS Code
- USB-Verbindung
- 115200 Baud

Debug-Ausgaben dürfen hier **ausführlich** sein.

---

### 3.2 Anlagen-Debug (Live-Betrieb)

**Ziel:** Überprüfung des Zusammenspiels von Software und Hardware.

Merkmale:
- reale Weichen und Rückmelder
- reale Relais / SSR
- Debug ist **rein beobachtend**
- Hard-Error führt zu echter Abschaltung

Wichtig:
- Debug greift nicht aktiv ein
- Debug ersetzt keine Sicherheit
- Debug ändert keine Logik

---

## 4. Debug aktivieren / deaktivieren

Das Debug-System wird zentral über Makros gesteuert (`mega2_debug.h`).

### 4.1 Beispiel: Debug aktiv

```cpp
#define DBG_PRINT(x)    Serial.print(x)
#define DBG_PRINTLN(x)  Serial.println(x)
4.2 Beispiel: Debug deaktiviert (Release)
cpp
Code kopieren
#define DBG_PRINT(x)
#define DBG_PRINTLN(x)
Es ist keine Codeänderung an den Modulen notwendig.

5. Erlaubte Zeichen und Format
5.1 Erlaubt
ASCII-Zeichen

Zahlen

kurze Schlüsselwörter

5.2 Nicht erlaubt / nicht empfohlen
Sonderzeichen außerhalb ASCII

Binärdaten

JSON im Serial-Debug (kommt erst in D3)

5.3 Formatkonvention
csharp
Code kopieren
[MODUL] Nachricht
Beispiele:

csharp
Code kopieren
[SBHF] init
[BLOCK] Block 6 occupied
Die Ausgabe ist damit:

menschenlesbar

maschinenlesbar

stabil für Log-Auswertung

6. Erwartete Systemreaktion
6.1 Normalfall
Zustände wechseln logisch

Debug meldet Fortschritt

keine Seiteneffekte

6.2 Grenzfälle
verzögerte Rückmeldungen werden toleriert

Debug zeigt Wartezustände

kein Fehlalarm

6.3 Fehlerfall
Debug meldet Fehlerursache

Hard-Error wird ausgelöst

Strom wird abgeschaltet

System friert ein

expliziter Reset erforderlich

Beispiel:

csharp
Code kopieren
[SBHF] WEICHE SOLL/IST FEHLER
[SBHF] HARD ERROR, reason=WEICHE_SOLL_IST
7. Hard-Error-Verhalten
Nach einem Hard-Error gilt:

alle Events werden ignoriert

keine Zustandsänderungen mehr

Debug-Zustand bleibt stabil

kein automatischer Neustart

Der Hard-Error-Zustand kann nur explizit verlassen werden.

8. Reset-Verhalten
Der Reset erfolgt bewusst und explizit:

cpp
Code kopieren
shadowController.resetError();
Nach dem Reset:

Fehlerstatus wird gelöscht

State wird auf Idle gesetzt

keine Weichenbewegung

keine Gleisspannung

Beispiel-Debug:

csharp
Code kopieren
[SBHF] RESET ERROR
[SBHF] State -> Idle
9. D2-Abschlussbewertung
✔ Debug-System vollständig
✔ Block- und ShadowYard-Debug stabil
✔ Serial- und Anlagenbetrieb klar getrennt
✔ Sicherheit nicht beeinträchtigt
✔ D3-fähig (I2C / ESP / UI)

10. Ausblick (D3)
Ab D3 werden die hier dokumentierten Zustände:

über I2C übertragen

vom ESP ausgewertet

im Web-UI visualisiert

Das Debug-System bleibt dabei die Referenzquelle für den Systemzustand.