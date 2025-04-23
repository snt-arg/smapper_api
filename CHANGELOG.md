# Changelog

All notable changes to this project will be documented in this file.

## [0.2.0] - 2025-04-23

### 🚀 Features

- Change configuration into a unified settings file
- Add additional parameters from config to topic monitor

## [0.1.0] - 2025-04-23

### 🚀 Features

- Create exception in case ros is not available
- Ensure there is only unique tags when saving a new rosbag

### 🐛 Bug Fixes

- Temporary fix for sourcing ros distro for rosbag

### ⚙️ Miscellaneous Tasks

- Update changelog

## [1.0.3] - 2025-04-15

### 🚀 Features

- Improve way of determining topic frequency

## [1.0.2] - 2025-04-09

### 🐛 Bug Fixes

- Return [] if no runner available
- Wait until rosbag folder has been created
- Replace empty spaces by _
- Verify a process exists
- Add missing CORS for production app server

### 💼 Other

- Add numpy missing deps

### 🚜 Refactor

- *(config)* Use the correct paths for smapper

### ⚙️ Miscellaneous Tasks

- Bump version

## [1.0.1] - 2025-04-08

### 🐛 Bug Fixes

- Make RosbagMetadataUpdate tags field match other schemes

### 🧪 Testing

- Adapt tests for new changes

### ⚙️ Miscellaneous Tasks

- Update changelog
- Update changelog

## [1.0.0] - 2025-04-07

### 🚀 Features

- Create a logger for processes
- Create a process handler class
- Creat tests for process_handler
- Possible configuration of device
- Create basic routers
- Create a global configuration for device
- Change name from process to service and added better error handling
- Create a service manager that manages all services
- Create routes for services management
- Create services from configuration and add newly created routes
- Move out dependencies function to a dedicted module
- Create a config for the API metadata
- Create a single logger.py module
- Handle exception
- Improve main.py
- Implement a better solution for defining api settings
- Convert device config to use the same style as APISettings
- Move previous config BaseModels into schemas
- Annotate endpoints
- Improve exceptions
- Add new schemas for bags and for service state
- Create a polling thread for the service_manager
- Properly load correct type of service
- Basic implementation of bag manager to read bags
- Change state of service to ERROR if return code not 0
- Create a basic rosbag sercice and rosbags manager
- Implement stopping the recording
- Implement functions to get state schema
- Reimplemente service to take care of process children
- Create a thred lock to avoid racing conditions
- Basic implementation of ros topic monitor
- Add missing is_running method
- Surround destroy node by a try
- Allow the API to run if ros is not available
- Ensure the rosbag service does not auto start nor restart on failure
- Initial code to use sqlite3
- Implementation of a proper database for rosbags
- Create a schema for updating rosbag metadata
- *(schemas)* Improve service schemas
- *(schemas)* Improve sensor schemas
- Include openapi tags to endpoints
- Add a settings router for future
- Improve exceptions
- Have #subscribers instead of message count

### 🐛 Bug Fixes

- Add missing imports
- Adapt test for the new changes made to project
- Add good command for rosbag
- Env variables should be defined as dict not list
- Source ws too and use . for sourcing .sh
- Add stdin to a PIPE to avoid process getting access to current TTY
- Add missing ws argument
- Use SIGNINT instead of terminat() otherwise we do no get the correct exit code
- Create storage dir if not existant
- Safely remove delete subscribers
- Remove srv_type from RosbagService
- Create bags storage directory if non existant
- Make sure the first element of services is the name
- Expand variables in path
- Change to get method

### 💼 Other

- Update project deps
- Add coloredlogs package
- Add pybase62 dependency
- Add lark as part of deps for ros testing
- Add sqlalchemy

### 🚜 Refactor

- Change sensors routes for newly created configuration
- Bring the Config load function out of the class
- Change logging format
- Use logger
- Change excpetion message
- Rename schemas classes to include Schema
- Move lifespan related function into it's own module
- Add missing Exception to NotYetImplemented exception
- Improve exception handling from psutil process
- Adding docstrings and endpoint descriptions
- Change topic status to online and offline
- Move settings out from config/ subdir
- Improve service schemas
- *(config)* Improve services names and ids
- Remove unused schemas and organize them
- Add logging to service manager
- Create a terminate function for service manager
- Improve way of getting topics statuses
- Update database create to initialize

### 🧪 Testing

- Create tests for service manager
- Fix service and service_manager new implementations
- Create very simple tests for api endpoints
- Adapt service test bad_command for new service implementation

### ⚙️ Miscellaneous Tasks

- Remove unused test
- Initial draft of processpool class
- Move process specific logger to core
- Delete old and unused schemas
- Move device config out of the app
- Move system services together with service.py impl
- Add todo and use global logger
- Delete .env
- Remove old requirements.txt
- Improve api description
- *(config)* Add correct topics to monitor
- Ignore sqlite db files
- Project structure improvement
- Improve core/ subfolder structure
- Generate changelog with git-cliff
- Bump version to 1.0.0

<!-- generated by git-cliff -->
