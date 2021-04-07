# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.0.0] Public Release
### Added
- New attribute: image resolution
- Estimator dependencies to dockerfile
- Added AAUCNS License File.

### Changed
- Added own implementation of rosbag timestamp (#12).
- Added image resolution to launch command line arguments.

### Fixed
- Fixed `CHANGELOG.md` links.
- Fixed OpenCV4 porting issues (#10 and aau-cns/openvins-based-multi-sensor-fusion#1).
- Fixed Python3.8 porting issues (#11).
- Issue with using rosparam for resolution in command line.
- Fixed wrong usage of gyro reading (#17).

## [0.2.1] - Install Documentation Hotfix
### Added
- Known installation issues to `README.md`

### Changed
- Updated the installation dependencies.

### Fixed
- Wrong tracking branch of submodules (#15).

## [0.2.0] - ICRA Submission Version
### Added
- Added GitLab MR Templates.
- Automation of estimator running.
- Noisy IMU input
- Input BAG/CSV parser.
- Added estimator: ROVIO
- Added estimator: LARVIO

### Changed
- Updated GitLab Issue Templates.

### Fixed
- Scene name in logic handler.
- Fixed focal length calculation.

### Removed
- Removed estimator: OKVIS

## [0.1.1]
### Added
- Added empty license file.

### Changed
- Updated and fixed the `Changelog.md`.

## [0.1.0]
### Added
- Added Changelog following the [Keep a Changelog](https://keepachangelog.com/en/1.0.0/) Guide
- Added logichandler, that handles the evaluation input logic.
- Added eval_tools, that perform the performance evaluation.
- Added bagrecorder, that records the evaluation rosbags.
- Added estimator: OpenVins
- Added estimator: VINS-Mono
- Added GitLab issue templates.
- Added trajectory files.

### Changed
- Updated flightgoggles data to track the internal CNS project.

### Fixed
- Added missing ros source `/opt/ros/melodic/devel/setup.bash`



[Unreleased]: https://gitlab.aau.at/aau-cns/vio_eval/eval_cws/compare/main...develop
[1.0.0]: https://gitlab.aau.at/aau-cns/vio_eval/eval_cws/compare/v0.2.1...v1.0.0
[0.2.1]: https://gitlab.aau.at/aau-cns/vio_eval/eval_cws/compare/v0.2.0...v0.2.1
[0.2.0]: https://gitlab.aau.at/aau-cns/vio_eval/eval_cws/compare/v0.1.0...v0.2.0
[0.1.1]: https://gitlab.aau.at/aau-cns/vio_eval/eval_cws/compare/v0.1.0...v0.1.1
[0.1.0]: https://gitlab.aau.at/aau-cns/vio_eval/eval_cws/-/tree/v0.1.0
