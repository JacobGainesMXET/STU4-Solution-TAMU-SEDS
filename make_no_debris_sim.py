
"""Create a separate, explicitly labelled debris-free controller test simulation."""
import argparse
import copy
import json
from pathlib import Path
import re


def build_no_debris_config(original):
    config = copy.deepcopy(original)
    entities = config["DataManager"]["entities"]

    def is_debris(entity):
        return re.fullmatch(r"Debris\d+", entity.get("#Required", {}).get("Name", "")) is not None

    removed = sum(is_debris(entity) for entity in entities)
    if removed == 0:
        raise ValueError("No debris found. Check the source file.")
    config["DataManager"]["entities"] = [entity for entity in entities if not is_debris(entity)]
    config["DataManager"]["SimEntity"].pop("Debris", None)
    config["DataManager"]["SimEntity"]["DoSpawning"] = False
    systems = config["SystemManager"]["Systems"]
    for system in systems:
        if system.get("Nametag") in ("Radar", "SpawnDebris"):
            system["Active"] = False
    controllers = [system for system in systems if system.get("Nametag") == "Controller"]
    if len(controllers) != 1:
        raise ValueError("Expected one Controller system in the source simulation.")
    controller = controllers[0]
    controller["Inst_Parameters"]["NoDebrisTestMode"] = True
    # This flag is supported by this repository's controller, not arbitrary controllers.
    controller["Source"] = f"Local/Repos/{Path(__file__).resolve().parent.name}/Controller.py"
    return config, removed


def main(argv=None):
    # Resolve relative to this checkout, not whichever Windows account launches Python.
    sim_dir = Path(__file__).resolve().parent.parent / "STU4-Sim" / "Sims"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=sim_dir / "COMPETITION_SIM.simconf")
    parser.add_argument("--output", type=Path, default=sim_dir / "COMPETITION_SIM_NO_DEBRIS.simconf")
    args = parser.parse_args(argv)
    if args.source.resolve() == args.output.resolve():
        raise ValueError("Source and output must differ; preserve the competition simulation.")
    original = json.loads(args.source.read_text(encoding="utf-8-sig"))
    config, removed = build_no_debris_config(original)
    args.output.write_text(json.dumps(config, indent=2) + "\n", encoding="utf-8")
    print(f"Removed {removed} debris objects.")
    print(f"Created no-debris TEST simulation: {args.output}")


if __name__ == "__main__":
    main()
