# Licence for hardware and model files

The code, configuration and documentation in this repository are under the Apache License 2.0 (`LICENSE`).

**Hardware files are different.** CAD, meshes, print files, and robot models that embed them (everything under
`hardware/` and the meshes under `robot/`, once they are published here) are under
**Creative Commons Attribution-NonCommercial-ShareAlike 4.0** (`LICENSES/CC-BY-NC-SA-4.0.txt`). You may build, modify
and share the robot with attribution and under the same terms, but not sell it.

Why: Microduck Nano derives from Pollen Robotics' Microduck, whose 3D models are published under CC BY-NC-SA 4.0
(`LICENSE-MODELS.md` in `pollen-robotics/microduck_rl`), by way of Luwu Dynamics' XGO-Duck. XGO-Duck's hardware files
carry no licence of their own yet; its maintainers state an intent to allow personal and educational replication.
Parts here that derive from XGO-Duck's shells are published only once that permission is confirmed.

Third-party model weights used by the benchmark scripts (for example Ultralytics YOLO, AGPL-3.0) are downloaded by
those scripts and are not stored in this repository.
