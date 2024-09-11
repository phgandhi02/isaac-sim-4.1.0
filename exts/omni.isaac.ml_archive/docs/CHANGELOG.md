# Changelog

## [2.0.1] - 2024-05-16
### Fixed
- Make platform specific

## [2.0.0] - 2024-04-19
### Changed
- removed omni.pip.torch dependency, torch is now directly part of this archive
- added torch==2.2.2+cu118, torchvision==0.17.2+cu118, torchaudio==2.2.2+cu118, filelock==3.13.4, fsspec==2024.3.1, mpmath==1.3.0, networkx==3.3, sympy==1.12
- added nvidia lib pip packages

## [1.1.3] - 2023-08-10
### Changed
- Added version to omni.pip.torch dependency

## [1.1.2] - 2023-07-05
### Changed
- Removed unused omni.kit.pipapi dependency

## [1.1.1] - 2023-06-12
### Changed
- no longer depend on omni.pip.torch

## [1.1.0] - 2022-07-11

### Changed
- Use pip_torch extension instead of packaging torch directly. 
- make extension kit sdk version specific

## [1.0.0] - 2022-07-01

### Changed
- Updating version for publishing


## [0.1.0] - 2022-01-13

### Added
- Initial version of Isaac Pip Archive
