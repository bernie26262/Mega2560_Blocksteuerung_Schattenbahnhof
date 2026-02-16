📄 Mega2 – Analog Abnahme Testplan v1

Stand: mega2-analog-precalib-2026-02-12
Ziel: Validierung Strom (ZMCT103C) + Trafo (ZMPT101B) im HW-Modus

1. Vorbereitung
Build

Environment: mega2-hw

Debug aktiv:

-DMEGA2_DEBUG=1
-DMEGA2_DEBUG_ANALOG_TICK=1
Monitor öffnen
pio device monitor -e mega2-hw

Erwartete Ausgabe (1×/s):

[AN] I1 raw=xxx off=xxx rms=xx act=x | ...
2. Stromsensor – Grundprüfung
2.1 Anlage komplett leer (kein Zug)

Warte 10 Sekunden.

Messgröße	Wert
rms (Block 1)	______
rms (SBHF1)	______
act	0 / 1
Soll:

rms ≤ 5

act = 0

2.2 Eine Lok fährt (mittlere Geschwindigkeit)
Messgröße	Wert
rms	______
act	0 / 1
Soll:

rms deutlich größer als Leerzustand

act = 1 stabil

2.3 Threshold validieren

Berechne:

Threshold = Leer_RMS + 40% der Differenz

Beispiel:

Leer 3

Zug 14
→ Threshold ≈ 7

Wenn:

act flackert → Threshold -1

act bei leer = 1 → Threshold +2

3. Grenztests
3.1 Nur Beleuchtung aktiv (kein Zug)

Soll:

act = 0

kein Fehltrigger

3.2 Extrem langsame Fahrt

Soll:

act bleibt stabil = 1

kein Flackern

3.3 Zug verlässt Block

Soll:

act wird nach < 1 s wieder 0

keine Hysterese-Probleme

4. Trafo-Spannung (ZMPT101B)
4.1 Trafo AUS
Messgröße	Soll
Vrms	~0–0.5V
powered	0
4.2 Trafo EIN
Messgröße	Soll
Vrms	stabil
powered	1

Falls Vrms stark schwankt:
→ Trimmer leicht nachstellen.

5. Safety-Test (real)
EMERG_NOTHALT_SBHF

Trafo EIN

Nothalt aktivieren

Block 6 belegen

Lok stromlos

Erwartung:

SSR OFF

Lock gesetzt

ACK blockiert

Danach:

Block frei

Nothalt frei

ACK OK

6. Dokumentation der Messwerte
Zustand	rms (leer)	rms (Zug)	Threshold	OK
Block 1	___	___	___	☐
SBHF 1	___	___	___	☐
7. Abschluss

Nach erfolgreichem Test:

MEGA2_DEBUG_ANALOG_TICK deaktivieren

Commit:

git commit -m "Mega2: analog sensors validated on hardware"
git push

Optional Tag:

git tag mega2-analog-validated-2026-02-12
git push origin mega2-analog-validated-2026-02-12
8. Abnahmekriterien (Go/No-Go)

✔ Kein Fehltrigger bei leerer Anlage
✔ Sichere Erkennung bei Fahrt
✔ Trafo-Erkennung stabil
✔ Safety korrekt reagierend

Wenn alle 4 erfüllt sind → Produktionsreif für Betrieb