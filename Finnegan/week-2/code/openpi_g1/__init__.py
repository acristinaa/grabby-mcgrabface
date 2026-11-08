"""openpi-Anbindung fuer G1 + BrainCo Revo2 (Trainingsstack entschieden 2026-09-14, Stufe 2).

Laeuft in der openpi-Umgebung (~/dev/g1-ext/openpi/.venv bzw. auf dem GPU-Rechner), nicht in der Sim-Umgebung.

  policy.py    Umrechnung Beobachtung/Aktion <-> Eingang von π₀.₅ - dieselbe im Training und im Policy-Server
  config.py    Trainingskonfiguration pi05_g1_brainco_lora, zur Laufzeit in openpi registriert
  convert.py   Rohaufnahmen (.npz, g1sim/episode.py) -> LeRobot-Datensatz v2.1
  stand_in.py  Ersatz-Policy-Server ohne Gewichte, fuer den Leitungstest Sim <-> Server
  run.py       Einstieg: check, stand-in, norm-stats, train, serve

Die Sim-Seite (openpi-client) liegt in g1sim/openpi_env.py, die Schleife in scripts/60_openpi_closed_loop.py.
"""
