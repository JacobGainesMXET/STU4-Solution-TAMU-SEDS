from contextlib import redirect_stdout
import io
import json
from pathlib import Path
import tempfile
import unittest

from make_no_debris_sim import build_no_debris_config, main


class NoDebrisSimulationTests(unittest.TestCase):
    def setUp(self):
        self.sim_dir = Path(__file__).resolve().parents[2] / 'STU4-Sim/Sims'
        self.source = self.sim_dir / 'COMPETITION_SIM.simconf'

    def test_generation_removes_debris_and_sets_only_test_overrides(self):
        original = json.loads(self.source.read_text(encoding='utf-8-sig'))
        before = json.dumps(original, sort_keys=True)
        generated, removed = build_no_debris_config(original)
        self.assertEqual(json.dumps(original, sort_keys=True), before)
        self.assertEqual(removed, 500)
        self.assertEqual(len(generated['DataManager']['entities']), 14)
        self.assertNotIn('Debris', generated['DataManager']['SimEntity'])
        self.assertFalse(generated['DataManager']['SimEntity']['DoSpawning'])
        for old, new in zip(original['SystemManager']['Systems'], generated['SystemManager']['Systems']):
            name = old['Nametag']
            if name in ('Radar', 'SpawnDebris'):
                expected = dict(old, Active=False)
                self.assertEqual(new, expected)
            elif name == 'Controller':
                self.assertTrue(new['Inst_Parameters']['NoDebrisTestMode'])
                self.assertNotIn('NoDebrisTestMode', old['Inst_Parameters'])
                self.assertTrue(new['Source'].endswith('STU4-Solution-TAMU-SEDS/Controller.py'))
            else:
                self.assertEqual(new, old)  # gravity, allocator, scoring stay identical
        self.assertEqual(len(generated['SystemManager']['Systems']), len(original['SystemManager']['Systems']))
        self.assertEqual({k: v for k, v in generated['SystemManager'].items() if k != 'Systems'},
                         {k: v for k, v in original['SystemManager'].items() if k != 'Systems'})
        self.assertEqual({k: v for k, v in generated.items() if k not in ('DataManager', 'SystemManager')},
                         {k: v for k, v in original.items() if k not in ('DataManager', 'SystemManager')})

    def test_cli_writes_a_separate_valid_test_config_and_preserves_source(self):
        before = self.source.read_bytes()
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'no_debris.simconf'
            with redirect_stdout(io.StringIO()):
                main(['--source', str(self.source), '--output', str(output)])
            generated = json.loads(output.read_text())
            controller = next(s for s in generated['SystemManager']['Systems'] if s['Nametag'] == 'Controller')
            self.assertTrue(controller['Inst_Parameters']['NoDebrisTestMode'])
        self.assertEqual(self.source.read_bytes(), before)

    def test_cli_refuses_to_overwrite_its_source(self):
        before = self.source.read_bytes()
        with self.assertRaises(ValueError):
            main(['--source', str(self.source), '--output', str(self.source)])
        self.assertEqual(self.source.read_bytes(), before)


if __name__ == '__main__':
    unittest.main()
