"""Phase 1c — BrainCo Revo2 Haende an die G1 anbauen und Szene schreiben.

Die eigentliche Bau-Logik liegt in dashboard/scene.py, damit das Dashboard die
Szene mit veraenderter Montage neu bauen kann, ohne den Code zu duplizieren.
Dieses Skript ist der Kommandozeilen-Weg dorthin.

Quelle der Handmodelle (ohne Login herunterladbar):
  https://www.brainco-hz.com/docs/revolimb-hand/en/revo2/download.html
  -> Revo2_xml.zip, entpackt nach models/revo2/{xml_left,xml_right}/

Pro Hand: 11 Gelenke, davon 6 aktuiert. Die uebrigen haengen ueber
`equality`-Constraints an den aktuierten (unteraktuierte Mechanik) -> 6 Steuerwerte
pro Hand, 12 fuer beide. Passt exakt zu den "12 hand DOF" des 26-DOF-Aktionsraums.
Das rechte Herstellermodell hat 5 lose Fingerspitzen-Gelenke mehr; attach_revo2 entfernt sie.

Ausgabe: models/g1_brainco/scene_table_hands.xml
"""
import pathlib
import sys
import warnings

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

import mujoco
from dashboard import scene

warnings.filterwarnings("ignore", message=".*Attach conflict.*")


def main():
    before = mujoco.MjModel.from_xml_path(str(scene.SCENE_IN))
    print(f"vorher:  nq={before.nq} nv={before.nv} nu={before.nu} ncam={before.ncam}")

    spec = scene.build_with_hands()
    model = spec.compile()
    print(f"nachher: nq={model.nq} nv={model.nv} nu={model.nu} ncam={model.ncam}")

    hand = scene.hand_actuator_ids(model)
    print(f"\nHand-Aktuatoren: {len(hand)} (erwartet 12)")
    for i in hand:
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        lo, hi = model.actuator_ctrlrange[i]
        print(f"  [{i:2d}] {name:38s} ctrlrange {lo:.3f} .. {hi:.3f} rad")

    n, dst = scene.copy_hand_assets()
    print(f"\n{n} Hand-Meshes kopiert -> {dst}")

    scene.SCENE_OUT.write_text(spec.to_xml())
    print("geschrieben:", scene.SCENE_OUT)

    check = mujoco.MjModel.from_xml_path(str(scene.SCENE_OUT))
    print(f"Reload OK: nq={check.nq} nv={check.nv} nu={check.nu} ncam={check.ncam}")

    if len(hand) != 12:
        raise SystemExit(f"FEHLER: {len(hand)} Hand-Aktuatoren statt 12.")


if __name__ == "__main__":
    main()
