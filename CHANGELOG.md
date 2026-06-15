## v1.2.1 (2026-06-15)

### 🐛 Bug Fixes

- **main**: pin tusb_device_task to CPU0 for dedicated GPIO correctness *(Jaroslav Burian - 7933745)*
- **build**: support updated ESP-IDF header layout *(Jaroslav Burian - c82c088)*
- **serial_handler**: let external tools control shared boot and reset lines *(AvNavSim - 7534bb5)*

### 📖 Documentation

- move ESP Launchpad link to top of page *(Jaroslav Burian - 94d5825)*

### 🔧 Code Refactoring

- Update driver dependency *(Jaroslav Safka - 01e97f2)*


## v1.2.0 (2025-09-23)

### ✨ New Features

- **msc**: ensure MD5 check is enabled *(Jaroslav Burian - bac9aa6)*

### 🐛 Bug Fixes

- **serial_handler**: Add RX pull up to avoid floating pin *(Jaroslav Burian - 5580383)*
- **debug**: use correct pins for TDO and TDI *(Jaroslav Burian - 0f04472)*
- **msc**: flash only real payload when UF2_FLAG_MD5_PRESENT flag is set *(Jaroslav Burian - 8eae57a)*
- **debug**: missing driver for Windows *(Jaroslav Burian - b56915c)*

### 🔧 Code Refactoring

- Separate pins and other Kconfig options into components *(Jaroslav Burian - 83dd070)*
- Add activity callbacks to debug_probe and serial_handler components *(Jaroslav Burian - 10c22e3)*
- Move debug specific code into debug_probe component *(Jaroslav Burian - c17f235)*
- Separate debug interface into separate component *(Jaroslav Burian - 28eee02)*
- Separate serial communication into components *(Jaroslav Burian - af83608)*


## v1.1.0 (2025-04-23)

### ✨ New Features

- **uf2**: Add UF2 IDs for C5, C61, H21, and H4 *(Radim Karniš - 3db3d07)*

### 🐛 Bug Fixes

- **tinyusb**: fix zero-length packet handling *(Jaroslav Burian - 974fb04)*
- **msc**: Fix flashing when blocks are not in numerical order *(Jaroslav Burian - 382530e)*
