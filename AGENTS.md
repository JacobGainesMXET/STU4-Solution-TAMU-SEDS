# Project instructions

## Mission and assignment
- This TSGC team project uses Escape from Phobos in Space Teams PRO: navigate a shuttle through debris to safety while balancing time, damage, and fuel.
- Jacob's assignment: implement short/long range sensor fusion by merging dynamic radar readings with close-range camera depth maps. Exact ranges, performance targets, and teammate interfaces remain unspecified.
- Do not invent requirements, sensor properties, algorithms, or additional TSGC constraints.

## References and competition constraints
- Authoritative rules: https://github.com/SimDynamX/SpaceTeamsPRO/discussions/218
- Model and frames: https://github.com/SimDynamX/STU4-Solution/wiki/1.-vehicle-specs-and-model-details
- Scoring: https://github.com/SimDynamX/STU4-Solution/wiki/2.-mission-success,-failure,-and-scoring
- Allowed interfaces: https://github.com/SimDynamX/STU4-Solution/wiki/3.-competitor-python-api
- API reference: https://simdynamx.github.io/stp-python-api-docs/
- Observe debris through radar/cameras; do not query debris entity truth. Use permitted thruster commands. Do not alter scoring, physics, timescale, or prohibited simulator configuration. Recheck applicable rules before implementation.

## Repository architecture
- Python with spaceteams, NumPy, and Pillow. Controller.py is the documented entry point; verify the active simulator configuration.
- Jacob reports this checkout is not connected to Space Teams. The installed version is unverified; do not assume it matches current online documentation.
- UserAPI.py demonstrates API access; BasicController.py demonstrates beeline control. All three scripts connect and loop at module scope; importing them can start simulator interaction.
- Preserve the marked connection boilerplate and protected helpers. Follow existing conventions and keep changes focused; avoid duplicate systems and unnecessary dependencies.
- SetCommand submits body-frame linear/angular accelerations. No keyboard handler is present in this checkout; manual-control wiring is unverified.

## Engineering and validation
- Shuttle state may be treated as truth under the official model. Radar observations are noisy and distance-sorted; array positions are not persistent object identities.
- Verify units, frames, camera offsets/orientation, capture timestamps, and measurement age before fusion. Range rate alone is not full 3D velocity.
- Documented Depth_cm output uses float32 centimeters; local examples extract RGB only. Verify installed API compatibility, projection/depth semantics, invalid pixels, sensor ranges, and uncertainty before implementing geometry or weighting.
- Test geometry, association, motion, missing/delayed observations, outliers, and sensor overlap when implementation is authorized. Separate mathematical, recorded-data, simulator, and hardware evidence; report only checks actually performed.

## Collaboration and learning
- Explain substantial changes before editing. Teach Python, mathematics, data flow, assumptions, and verification so Jacob can maintain the work.
- Initial scope permits reading and root AGENTS.md only. Obtain explicit approval before implementation, other project edits, dependency installation, project execution/tests, or hardware commands; honor later authorization.
- Verify Git root, branch, remotes, and status before changes. Use a feature branch for implementation; preserve unrelated teammate work.
- Never commit, push, merge, rebase, or open pull requests without explicit approval. Do not assume origin is the shared team repository.
- Suggest persistent instruction updates when warranted; do not record temporary tasks automatically.
