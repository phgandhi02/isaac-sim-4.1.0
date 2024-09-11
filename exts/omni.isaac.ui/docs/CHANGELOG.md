# Changelog
## [0.16.0] - 2024-07-18
### Added
- Added ability to trigger Button and StateButton clicks programmatically.

## [0.15.2] - 2024-07-12
### Fixed
- Fixed scrolling window set_num_lines() function argument type.

## [0.15.1] - 2024-04-13
### Added
- optional tooltip entry for SelectPrimWidget and ParamWidget

## [0.15.0] - 2024-03-14
### Added
- Added function to trigger user callback manually for DropDown widget.
- Added test of expected behavior when populating DropDown widge with Articulations on the stage.

## [0.14.1] - 2024-02-27
### Changed
- Use action registry in make_menu_item_description by deregistering actions first

## [0.14.0] - 2024-02-02
### Added
- SelectPrimWidget and ParamWidget to simplify populating a popup dialog for collecting paramaters. 

## [0.13.7] - 2024-02-02
### Changed
- Updated path to the nucleus extension

## [0.13.6] - 2024-01-29
### Fixed
- Fix bug where XYPlot widget could display the wrong axis values for ragged data.

## [0.13.5] - 2023-11-13
### Fixed
- Updated documentation link

## [0.13.4] - 2023-08-26
### Fixed
- Error when user setup function creates a new stage

## [0.13.3] - 2023-08-25
### Fixed
- Error when using load button due to physics context initialization issue

## [0.13.2] - 2023-08-22
### Fixed
- Fixed possible errors in UI Plotting Tool where x_min is set arbitrarily close to x_max beyond what was a hard-coded constant.

## [0.13.1] - 2023-08-22
### Fixed
- Fixed possible errors in UI Plotting Tool that can be caused by spamming updates to violate x_max > x_min on X axis. Fixed error when floats are overflowed, which allowed x_min to equal x_max for high numbers

## [0.13.0] - 2023-08-15
### Fixed
- Added extra frame of waiting when using LoadButton and ResetButton after core.World.reset(). The extra frame resolves possible differences in rendered object position and queried position.

## [0.12.1] - 2023-08-09
### Changed
- Updated ScrollingWindow wrapper to use only omni.ui as a dependency

## [0.12.0] - 2023-08-08
### Added
- Added ScrollingWindow UI wrapper now that Kit 105.1 windows do not automatically have a scrollbar.

### Changed
- Update build_header() function to use constant colors

## [0.11.1] - 2023-07-26
### Fixed
- Fixed unused on event function for string builder

## [0.11.0] - 2023-07-11
### Added
- Added test cases for UI widget wrappers
- Added test cases for UI Core Connectors

### Fixed
- Fixed small bugs found in UI widget wrappers

## [0.10.0] - 2023-06-27
### Fixed
- Enforce argument types in setter functions for UI Widget Wrappers by casting [int,float,bool] args to their required type explicitly. This handles undesirable behavior that arises when numpy types such as np.float are passed instead of primitive types. For example, the FloatField displays the value 0.0:np.float as 1.0.

## [0.9.0] - 2023-05-03
### Added
- Added UIElementWrapper XYPlot with significantly more useful plotting utility than the base omni.ui.Plot

## [0.8.0] - 2023-03-13
### Fixed
- Added missing imports in element_wrappers __init__.py
- Fixed bad formatting on CheckBox docstring

### Added
- Added get_value() function to CheckBox UIElementWrapper

## [0.7.0] - 2023-02-28

### Changed
- Breaking Change: Removed UIFrameWrapper and replaced with CollapsableFrame UIElementWrapper instance
- Breaking Change: UIElementWrapper get_ui_element() function replaced with .container_frame property that gives the user a UI frame that contains everything in the UIElementWrapper (label,buttons,fields,etc.)

### Fixed
- FloatField UIElementWrapper was giving an error when no joint limits were being set
- FloatField setters cast arguments to floats to avoid problems with np.float types

### Added
- Completed basic UI element wrappers in omni.isaac.ui.element_wrappers
- Added accessors to each UIElementWrapper instance to get each omni.ui.Widget that is used.

## [0.6.2] - 2023-02-16

### Added
- Added detailed docstrings to all omni.isaac.ui.element_wrappers.* __init__ and public member functions

## [0.6.1] - 2023-02-15

### Added
- Added DropDown and FloatField UI wrappers in omni.issac.ui/element_wrappers

## [0.6.0] - 2023-02-13

### Added
- Added omni.isaac.ui/element_wrappers with helpful wrappers around UI elements that simplify button and frame creation and management.

## [0.5.2] - 2023-01-19

### Fixed
- split calback tests to reduce errors
### Changed
- rename startup to test_ui

## [0.5.1] - 2023-01-11

### Fixed
- revert to old menu click function to fix hot reload errors

## [0.5.0] - 2023-01-06

### Added
- make_menu_item_description to old menus can use the new action registry

## [0.4.3] - 2022-12-11
### Fixed
- Fixed issue with error when outputting warning message for opening web browser

## [0.4.2] - 2022-12-02
### Fixed
- Fixed bug: File Picker adding extra slash to file path when selecting non-existing files

## [0.4.1] - 2022-11-22
### Fixed
- Missing min/max limits for int field

## [0.4.0] - 2022-11-16
### Added
- Dpad Controller Class

## [0.3.1] - 2022-09-10
### Fixed
- Fix screen print test

## [0.3.0] - 2022-07-18
### Added
- Add a class to print directly onto the screen using Omnigraph.

## [0.2.1] - 2022-06-02
### Changed
- expose labels for file/folder picker

## [0.2.0] - 2022-05-25
### Added
- Windows support to open vscode and folders

## [0.1.3] - 2022-01-18
### Fixed
- Fixes layout issues

## [0.1.2] - 2021-12-14
### Changed
- adjust tooltip for ui Buttons with changed background color

## [0.1.1] - 2021-09-28
### Fixed
- General bugfixes

## [0.1.0] - 2021-07-22

### Added
- Initial version of Isaac Sim UI
