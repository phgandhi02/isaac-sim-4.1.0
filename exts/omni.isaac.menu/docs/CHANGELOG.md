# Changelog

## [0.5.0] - 2024-07-10
### Removed
- Deprecated omni.isaac.dofbot and removed its usage.

## [0.4.1] - 2024-05-28
### Fixed
- Robotiq asset links

## [0.4.0] - 2024-05-24
### Changed
- Added leatherback
### Fixed
- broken USD paths

## [0.3.1] - 2024-05-11
### Changed
- Renamed Transporter to iw.hub

## [0.3.0] - 2024-05-07
### Added
- Added menu options for the new robots
- Added humanoid robot section

## [0.2.3] - 2024-04-22
### Fixed
- Sensor menu tests failing due to sensor extension now creates sensors on play, play timeline first before checking for values

## [0.2.2] - 2024-03-07
### Fixed
- Tests failing due to short delay when clicking, increased delay from 10 to 32 (default)

## [0.2.1] - 2024-02-20
### Fixed
- Extra dependencies from tests being incorrectly imported

## [0.2.0] - 2024-01-30
### Changed
- Changed sensor menu tests to verify sensor functionality through the sensor interfaces
- Added golden image test for the environment

## [0.1.0] - 2024-01-19
### Added
- Initial version of extension, moving menu code out of omni.isaac.utils extension
