Change log for PC Engines fork of coreboot
==========================================

Releases 4.0.x are based on PC Engines 20160304 release.
Releases 4.5.x and 4.6.x are based on mainline support submitted in
[this gerrit ref](https://review.coreboot.org/#/c/14138/).
Releases 4.8.0.x/4.9.0.x and later are based on continuous synchronization with
official [coreboot repository](https://review.coreboot.org/cgit/coreboot.git)

## Quick build instructions

Please use [pce-fw-builder](https://github.com/pcengines/pce-fw-builder)

## [Unreleased]

## [v4.16.0.2] - 2022-03-11
### Changed
- Rebased with official coreboot repository commit 66f99f7f

## [v4.16.0.1] - 2022-02-11
### Changed
- Rebased with official coreboot repository commit b4ba289f
- Disabled loglevel prefixes introduced in coreboot 4.16
- Disabled ANSI escape sequences introduced in coreboot 4.16

### Fixed
- [AMD PSP CCP as entropy source](https://github.com/pcengines/apu2-documentation/issues/112)

## [v4.15.0.3] - 2022-02-11
### Changed
- Rebased with official coreboot repository commit 36425312ee
- Added checking hardware matrix before regression tests

### Fixed
- The hard disk not visible in the Seabios Boot Menu

## [v4.15.0.2] - 2021-12-27
### Changed
- rebased with official coreboot repository commit 3990da0b
- disabled SMM
- enabled parallel AP initialization for apu2-6 for faster boot time

## [v4.15.0.1] - 2021-11-26
### Changed
- rebased with official coreboot repository commit 6973a3e7

## [v4.14.0.6] - 2021-10-29
### Changed
- rebased with official coreboot repository commit d06c0917

## [v4.14.0.5] - 2021-10-08
### Changed
- rebased with official coreboot repository commit d4c55353
- Updated CPU declarations in ACPI to comply with newer ACPI standard
- Removed GPIO bindings to fix conflict with OS drivers

## [v4.14.0.4] - 2021-09-06
### Changed
- rebased with official coreboot repository commit d9f5d90
- enabled EHCI controller by default on apu3-apu6 platforms
- [updated sortbootorder to v4.6.22](https://github.com/pcengines/sortbootorder/blob/master/CHANGELOG.md#v4622---2021-09-06)

### Added
- safeguard against setting watchdog timeout too low

## [v4.14.0.3] - 2021-07-29
### Changed
- rebased with official coreboot repository commit c049c80

## [v4.14.0.2] - 2021-06-28
### Changed
- rebased with official coreboot repository commit 1c43d92

## [v4.14.0.1] - 2021-05-27
### Changed
- rebased with official coreboot repository commit 6a936fc
- [updated sortbootorder to v4.6.21](https://github.com/pcengines/sortbootorder/blob/master/CHANGELOG.md#v4621---2021-05-27)
- [updated SeaBIOS to rel-1.14.0.1](https://github.com/pcengines/seabios/blob/apu_support/CHANGELOG.md#rel-11401---2021-05-27)

### Added
- persistent bootorder and runtime configuration in separate FMAP region for
  apu2-apu6 platforms

### Fixed
- inconsistent MTRRs warning in Linux

## [v4.13.0.6] - 2021-04-29
### Changed
- rebased with official coreboot repository commit a4c09c5

## [v4.13.0.5] - 2021-04-07
### Changed
- rebased with official coreboot repository commit e7a68ec

## [v4.13.0.4] - 2021-02-24
### Changed
- rebased with official coreboot repository commit c79fc47

## [v4.13.0.3] - 2021-01-27
### Changed
- rebased with official coreboot repository commit 5e6e5c1

## [v4.13.0.2] - 2020-12-28
### Changed
- rebased with official coreboot repository commit 8edb48b

### Fixed
- [Serial number calculation on APU1](https://github.com/pcengines/coreboot/issues/436)

## [v4.13.0.1] - 2020-11-25
### Changed
- rebased with official coreboot repository commit 9b7dc76

## [v4.12.0.6] - 2020-10-29
### Changed
- rebased with official coreboot repository commit 43439f6

### Fixed
- the option in runtime config to reverse PCI addressing order, now not only
  mPCIe devices, but NICs are reversed as well. The WoL capable NIC should be
  the first booting in iPXE when the reverse option is enabled.

## [v4.12.0.5] - 2020-09-25
### Changed
- rebased with official coreboot repository commit da3375e
- added apu6 build target

## [v4.12.0.4] - 2020-08-28
### Changed
- rebased with official coreboot repository commit 81a2f45
- updates sortbootorder to v4.6.20 for minor build fix

### Fixed
- TPM2 visibility in OS for apu3d and apu4d
- issues with IRQ vectors reported in Xen and Linux dmesg

## [v4.12.0.3] - 2020-07-29
### Fixed
- [memory speed values in the DMI table](https://github.com/pcengines/apu2-documentation/issues/176)
- [Cbmem error on apu1 caused by wrong ACPI table](https://github.com/pcengines/coreboot/issues/411)

### Changed
- rebased with official coreboot repository commit 8d5cedf
- [updated sortbootorder to v4.6.19](https://github.com/pcengines/sortbootorder/blob/master/CHANGELOG.md#v4619---2020-07-29)
- [extended flashing documentation with FreeBSD/pfSense guides](https://github.com/pcengines/apu2-documentation/blob/master/docs/firmware_flashing.md)

### Added
- [option in runtime config to reverse PCI addressing order](https://github.com/pcengines/coreboot/issues/392)

## [v4.12.0.2] - 2020-06-28
### Fixed
- [incorrrect serial number in dmidecode for apu1](https://github.com/pcengines/coreboot/issues/402)

### Changed
- rebased with official coreboot repository commit f183626
- enabled TPM2 for apu3d and apu4d supporting a TPM module connection

### Added
- DRTM ACPI table (holds information about TPM log area for TrenchBoot DRTM)

## [v4.12.0.1] - 2020-05-29
### Changed
- rebased with official coreboot repository commit 8952d1c
- Memtest86+ revision updated to 0b756257276729c1a12bc1d95e7a1f044894bda2

### Fixed
- [random coreboot hangs on apu1](https://github.com/pcengines/coreboot/issues/394)
- [AGESA assertion errors on apu1](https://github.com/pcengines/coreboot/issues/397)


## [v4.11.0.6] - 2020-04-26
### Changed
- rebased with official coreboot repository commit d6f7ec5
- [updated sortbootorder to v4.6.18](https://github.com/pcengines/sortbootorder/blob/master/CHANGELOG.md#v4618---2020-04-26)
- reverted changes to ACPI CPU definitions causing BSD systems to
  [not probe CPU frequency driver](https://github.com/pcengines/coreboot/issues/389)
- reverted changes with PCIe reset logic causing
  [mPCIe2 slot connected modules to not appear in OS](https://github.com/pcengines/coreboot/issues/388)

### Added
- PCIe power management features runtime configuration
- [IOMMU IVRS generation expanded with IVHD type 11h for newer Xen](https://review.coreboot.org/c/coreboot/+/40042)

### Fixed
- [memtest hang on apu1](https://github.com/pcengines/coreboot/issues/395)
- [TPM2 detection on FreeBSD 12.1](https://review.coreboot.org/c/coreboot/+/39699)

## [v4.11.0.5] - 2020-03-27
### Changed
- rebased with official coreboot repository commit 90557f4

### Fixed
- Processors definitions and scope in ACPI
- Missing GNB IOAPIC initialization

### Added
- Enabled PCI Express power management capabilities: ASPM L0s and L1,
  CommonClock, ClockPowerManagement
- PCI Express endpoint reset logic using GPIOs for apu2
- Thermal zone definition in ACPI
- SMBIOS memory tables for apu1
- GNB IOAPIC to MP table

## [v4.11.0.4] - 2020-02-26
### Changed
- rebased with official coreboot repository commit e53f8c9

### Fixed
- microSD card boot order on apu5

### Added
- ARM management controller interaction on apu5

## [v4.11.0.3] - 2020-01-29
### Changed
- rebased with official coreboot repository commit 536799d
- [updated sortbootorder to v4.6.17](https://github.com/pcengines/sortbootorder/blob/master/CHANGELOG.md#v4617---2020-01-28)

## [v4.11.0.2] - 2019-12-30
### Changed
- rebased with official coreboot repository commit 536799d
- [updated sortbootorder to v4.6.16](https://github.com/pcengines/sortbootorder/blob/master/CHANGELOG.md#v4616---2019-12-30)

### Added
- IOMMU runtime configuration (IOMMU is disabled by default)

## [v4.11.0.1] - 2019-12-08
### Changed
- rebased with official coreboot repository commit 9f56eed

### Fixed
- issue with temperature not showing on pfSense dashboard

## [v4.10.0.3] - 2019-11-08
### Changed
- rebased with official coreboot repository commit 2d90cb1

## [v4.10.0.2] - 2019-10-08
### Changed
- rebased with official coreboot repository commit 64c14b5

### Added
- soft reset for SuperIO GPIO
- IO port range for SuperIO GPIO functions

## [v4.10.0.1] - 2019-09-10
### Changed
- watchdog is now available on APU3
- rebased with official coreboot repository commit 22d66ef

## [v4.10.0.0] - 2019-08-09
### Changed
- [ACPI support for GPIOs](https://github.com/pcengines/apu2-documentation/blob/master/docs/gpios.md)
- rebased with official coreboot repository commit 2a20d13

## [v4.9.0.7] - 2019-07-09
### Changed
- [updated SeaBIOS to rel-1.12.1.3](https://github.com/pcengines/seabios/blob/apu_support/CHANGELOG.md#rel-11213---2019-07-05)
- [updated sortbootorder to v4.6.15](https://github.com/pcengines/sortbootorder/blob/master/CHANGELOG.md#v4615---2019-07-05)
- disabled IPv6 in iPXE that often caused the dhcp/autoboot command to time out
- removed incorrectly assigned clock request mappings
- rebased with official coreboot repository commit c32ccb7

### Added
- [prepared integration of tianocore payload allowing to boot UEFI aware systems](https://github.com/pcengines/apu2-documentation/blob/master/docs/tianocore_build.md)

## [v4.9.0.6] - 2019-06-08
### Changed
- [updated SeaBIOS to rel-1.12.1.2](https://github.com/pcengines/seabios/blob/apu_support/CHANGELOG.md#rel-11212---2019-06-04)
- [updated sortbootorder to v4.6.14](https://github.com/pcengines/sortbootorder/blob/master/CHANGELOG.md#v4614---2019-06-04)
- SD 3.0 mode is now runtime configurable
- watchdog is now runtime configurable
- rebased with official coreboot repository commit 3b4d0e0

### Added
- [vboot support with measured boot for apu2](https://github.com/pcengines/apu2-documentation/blob/master/docs/apu2_vboot.md)
- [serial ports are now decribed in ACPI](http://pcengines.info/forums/?page=post&fid=6D8DBBA4-9D40-4C87-B471-80CB5D9BD945&lastp=1&id=776921E8-222D-45F7-A234-910DEBBDA767)

## [v4.9.0.5] - 2019-05-08
### Changed
- rebased with official coreboot repository commit fe80bf2

### Fixed
- MP table creation: fixed SDHCI, xHCI interrupt entries
- redundant SVI2 information string in sign-of-life

### Added
- MP table entries for PCIe bridges, endpoints and IOMMU

## [v4.9.0.4] - 2019-04-03
### Changed
- rebased with official coreboot repository commit 28def8b
- [updated SeaBIOS to rel-1.12.1.1 with new TPM menu option](https://github.com/pcengines/apu2-documentation/blob/master/docs/tpm_menu.md)

### Added
- possibility to reboot platform with coldboot path to ensure full platform
  reset during [remote firmware update](https://github.com/pcengines/apu2-documentation/blob/master/docs/firmware_flashing.md#corebootrom-flashing),
  option is intended to mitigate reboot issue when migrating from BIOS version
  older than v4.9.0.x

## [v4.9.0.3] - 2019-03-08
### Changed
- rebased with official coreboot repository commit 7a732b4
- sortbootorder updated to v4.6.13 introducing CPU boost runtime configuration
- firmware is now built with coreboot SDK 1.52
- configured pull-ups on WLAN_DISABLE# pins on the mPCIe connectors which could
  cause issues with certain modems when floating

### Fixed
- [microcode update option](https://github.com/pcengines/apu2-documentation/issues/75#issuecomment-462979810)
- [memtest86+ and iPXE revisions for fully reproducible builds](https://github.com/pcengines/coreboot/issues/267)

### Added
- CPU boost runtime configuration in coreboot
- information about ECC memory capability in SMBIOS tables on 4GB platforms
- interrupt configuration entries for PCIe bridge devices 2.4 and 2.5

## [v4.9.0.2] - 2019-02-04
### Changed
- rebased with official coreboot repository commit 2b2325e

### Fixed
- removed AGESA headers changes introduced with 1.0.0.A

### Added
- enabled Core Performance Boost feature

## [v4.9.0.1] - 2019-01-09
### Changed
- rebased with official coreboot repository commit cd26f08
- updated SeaBIOS to rel-1.12.0.1
- prepare apu1 platform for TPM usage
- enabled TPM2.0 module on apu1, apu2 and apu5 in default configuration

### Fixed
- disabled waiting for SVI2 command completion causing reboot hangs
- introduced reproducible builds by passing build ID to iPXE
- AGESA AmdInitLate error caused by AGESA bugs

## [v4.8.0.7] - 2018-12-03
### Changed
- updated SeaBIOS to rel-1.11.0.7
- updated sortbootorder to v4.6.12

### Added
- [experimental option for adding microcode update](https://github.com/pcengines/apu2-documentation/blob/master/docs/microcode_patching.md)
- enabled PCIe ACS and AER capabilities
- [COM2 redirection runtime configuration](https://github.com/pcengines/apu2-documentation/blob/master/docs/serial_console.md)

### Fixed
- generating serial number from MAC address of first NIC

## [v4.8.0.6] - 2018-11-08
### Changed
- Updated SeaBIOS to rel-1.11.0.6
- Synced and rebased with coreboot official repo commit de46280

### Added
- [Console output redirection to COM2 option](https://github.com/pcengines/apu2-documentation/blob/master/docs/serial_console.md)

## [v4.8.0.5] - 2018-10-04
### Changed
- Updated sortbootorder to v4.6.11
- Removed custom libpayload modifications
- Synced and rebased with coreboot official repo commit b7b085d

### Fixed
- ECC exclusion range, ECC now works properly
- apu1 bootorder PCI paths and letter assignments

## [v4.8.0.4] - 2018-08-31
### Changed
- Synced and rebased with coreboot official repo commit e27c096
- iPXE revision to master

### Fixed
- iPXE autoboot is enabled for all interfaces

## [v4.8.0.3] - 2018-08-07
### Changed
- Synced and rebased with coreboot official repo commit c8e974f

### Fixed
- Change of date format in sign of life string now applies only to PC Engines
- Use CONFIG_LOCALVERSION as a coreboot version string
- Double sign-of-life during power on

# Added
- Enabled IOMMU

## [v4.8.0.2] - 2018-07-08
### Changed
- Synced and rebased with coreboot official repo commit afc74ca

## [v4.8.0.1] - 2018-06-08
### Changed
- Rebased coreboot repository to official coreboot 4.8.0

## [v4.6.10] - 2018-06-08
### Changed
- Updated SeaBIOS to 1.11.0.5
- Updated sortbootorder to v4.6.9

### Added
- S1 button support for apu5b

## [v4.6.9] - 2018-05-11
### Added
- Infrastructure to include microcode update
- Support for new building system pce-fw-builder

### Fixed
- Building is now possible with official coreboot-sdk container
- Retrieving board serial number

## [v4.6.8] - 2018-04-06
### Added
- Full feature and build support for APU1
- Serial console enable with S1 button feature

### Changed
- Updated sortbootorder to v4.6.8
- Switched from IDE to AHCI mode for SATA controller
- Updated SeaBIOS to 1.11.0.4

## [v4.6.7] - 2018-03-01
### Fixed
- SD cards performance drop
- SMBIOS part number format

### Changed
- xHCI has been enabled

## [v4.6.6] - 2018-01-31
### Changed
- Updated SeaBIOS to 1.11.0.3

### Fixed
- Memtest86+ screen refresh for serial

### Added
- Enabled ATA UDMA in SeaBIOS

## [v4.6.5] - 2017-12-29
### Added
- Apu features default values to `bootorder_def` file

### Changed
- Updated SeaBIOS to 1.11.0.2
- Updated sortbootorder to v4.6.5
- Forced EHCI controller for front USB ports
- Disabled xHCI controller in SeaBIOS

### Fixed
- Setting correct PCI_ROM_ID for iPXE depending on platform

## [v4.6.4] - 2017-11-30
### Added
- SPI support
- Security register support for reading serial number

### Changed
- Upgrade SeaBIOS to 1.11.0.1
- Update sortbootorder to v4.6.4
- Switch Memtest86+ to mainline

## [v4.6.3] - 2017-10-30
### Added
- Runtime configuration of UARTc/d, EHCI and mPCIe2 CLK for APU2/3/5

## [v4.6.2] - 2017-09-29
### Added
- "Disable retrieving SPD info" feature in Memtest86+
- Piotr Król and Kamil Wcisło as PC Engines platforms maintainers

### Changed
- APU3 and APU5 targets are now available as APU2 variants. This change removes
redundant code which was similar for APU2/3/5 boards.

### Fixed
- Date format in sign of life string

## [v4.6.1] - 2017-08-30
### Added
- APU5 target

## [v4.6.0] - 2017-07-21
### Added
- Allow to force GPP3 PCIe CLK attached to mPCIe2 slot based on Konfig option
  This helps in some cases, one example could be
  [mPCIe Ethernet extension card](https://github.com/pcengines/apu2-documentation/blob/master/docs/debug/mpcie_ethernet.md)

### Changed
- coreboot updated to [mainline 4.6 tag](https://github.com/coreboot/coreboot/tree/4.6)
- Update sortbootorder to v4.5.7

## [v4.5.8] - 2017-06-30
### Changed
- Moved sign of life strings (except for memory information) from `mainboard.c`
  to `romstage.c`. They are printed ~0.3s after power on, instead of over 2s.
- Update sortbootorder to v4.5.6
- Use the same sortbootorder for both mainline and legacy
- Enabled sortbootorder payload compression
- Update SeaBIOS to rel-1.10.2.1

### Fixed
- Fixed SeaBIOS debug level option in Kconfig

## [v4.5.7] - 2017-06-08
### Added
- print BIOS version on startup
- correct BIOS version in `SMBIOS`

### Changed
- set board names prefix to "PC Engines" (Kconfig)

## [v4.5.6] - 2017-05-30
### Changed
- updated sortbootorder to v4.5.5

## [v4.5.5.2] - 2017-03-03
### Changed
- (APU3 only) GPIO33 (SIMSWAP) set to output/low by default

## [v4.5.5.1] - 2017-03-02
### Changed
- (APU3 only) GPIO33 (SIMSWAP) set to output/high by default

## [v4.5.5] - 2017-02-24
### Added
- sgabios video oprom is added by default (v.1.0.0 pcengines version)

### Changed
- SeaBIOS up to 1.10.0.1
- sortbootorder up to 4.5.4
- coreboot rebased to the latest mainline

### Fixed
- Asmedia ASM106x controllers work in the mPCIe1 slot
- Memory size shown during boot corrected in 2GB SKU devices

### Known issues
- Asmedia ASM106x controllers are not working in the mPCIe2 slot
- Some XHCI USB booting stability issues after soft-reset

## [v4.5.4] - 2017-01-24
### Added
- add target for APU3 board (EHCI0 is enabled)

### Changed
- add coreboot build string message during boot
- change SeaBIOS version to 1.9.2.4
- optimizations for SeaBIOS config, faster boot

### Fixed
- fix `WARNING - Timeout at i8042_flush:71!` message during boot
- fix `ASSERTION ERROR: file 'src/northbridge/amd/pi/agesawrapper.c', line 305`
  message during boot
- fix disabling/enabling iPXE rom by `sortbootorder`
- fix custom menu/config options for iPXE rom

## [v4.5.3] - 2017-01-12
### Changed
- bootorder file align to 4K in flash
- add legacy logs on boot
- PCEngines SeaBIOS enabled by default
- turn off D4 and D5 leds on boot
- enable power on after power failure

[Unreleased]: https://github.com/pcengines/coreboot/compare/v4.16.0.2...develop
[v4.16.0.2]: https://github.com/pcengines/coreboot/compare/v4.16.0.1...v4.16.0.2
[v4.16.0.1]: https://github.com/pcengines/coreboot/compare/v4.15.0.3...v4.16.0.1
[v4.15.0.3]: https://github.com/pcengines/coreboot/compare/v4.15.0.2...v4.15.0.3
[v4.15.0.2]: https://github.com/pcengines/coreboot/compare/v4.15.0.1...v4.15.0.2
[v4.15.0.1]: https://github.com/pcengines/coreboot/compare/v4.14.0.6...v4.15.0.1
[v4.14.0.6]: https://github.com/pcengines/coreboot/compare/v4.14.0.5...v4.14.0.6
[v4.14.0.5]: https://github.com/pcengines/coreboot/compare/v4.14.0.4...v4.14.0.5
[v4.14.0.4]: https://github.com/pcengines/coreboot/compare/v4.13.0.3...v4.14.0.4
[v4.14.0.3]: https://github.com/pcengines/coreboot/compare/v4.13.0.2...v4.14.0.3
[v4.14.0.2]: https://github.com/pcengines/coreboot/compare/v4.13.0.1...v4.14.0.2
[v4.14.0.1]: https://github.com/pcengines/coreboot/compare/v4.13.0.6...v4.14.0.1
[v4.13.0.6]: https://github.com/pcengines/coreboot/compare/v4.13.0.5...v4.13.0.6
[v4.13.0.5]: https://github.com/pcengines/coreboot/compare/v4.13.0.4...v4.13.0.5
[v4.13.0.4]: https://github.com/pcengines/coreboot/compare/v4.13.0.3...v4.13.0.4
[v4.13.0.3]: https://github.com/pcengines/coreboot/compare/v4.13.0.2...v4.13.0.3
[v4.13.0.2]: https://github.com/pcengines/coreboot/compare/v4.13.0.1...v4.13.0.2
[v4.13.0.1]: https://github.com/pcengines/coreboot/compare/v4.12.0.6...v4.13.0.1
[v4.12.0.6]: https://github.com/pcengines/coreboot/compare/v4.12.0.5...v4.12.0.6
[v4.12.0.5]: https://github.com/pcengines/coreboot/compare/v4.12.0.4...v4.12.0.5
[v4.12.0.4]: https://github.com/pcengines/coreboot/compare/v4.12.0.3...v4.12.0.4
[v4.12.0.3]: https://github.com/pcengines/coreboot/compare/v4.12.0.2...v4.12.0.3
[v4.12.0.2]: https://github.com/pcengines/coreboot/compare/v4.12.0.1...v4.12.0.2
[v4.12.0.1]: https://github.com/pcengines/coreboot/compare/v4.11.0.6...v4.12.0.1
[v4.11.0.6]: https://github.com/pcengines/coreboot/compare/v4.11.0.5...v4.11.0.6
[v4.11.0.5]: https://github.com/pcengines/coreboot/compare/v4.11.0.4...v4.11.0.5
[v4.11.0.4]: https://github.com/pcengines/coreboot/compare/v4.11.0.3...v4.11.0.4
[v4.11.0.3]: https://github.com/pcengines/coreboot/compare/v4.11.0.2...v4.11.0.3
[v4.11.0.2]: https://github.com/pcengines/coreboot/compare/v4.11.0.1...v4.11.0.2
[v4.11.0.1]: https://github.com/pcengines/coreboot/compare/v4.10.0.3...v4.11.0.1
[v4.10.0.3]: https://github.com/pcengines/coreboot/compare/v4.10.0.2...v4.10.0.3
[v4.10.0.2]: https://github.com/pcengines/coreboot/compare/v4.10.0.1...v4.10.0.2
[v4.10.0.1]: https://github.com/pcengines/coreboot/compare/v4.10.0.0...v4.10.0.1
[v4.10.0.0]: https://github.com/pcengines/coreboot/compare/v4.9.0.7...v4.10.0.0
[v4.9.0.7]: https://github.com/pcengines/coreboot/compare/v4.9.0.6...v4.9.0.7
[v4.9.0.6]: https://github.com/pcengines/coreboot/compare/v4.9.0.5...v4.9.0.6
[v4.9.0.5]: https://github.com/pcengines/coreboot/compare/v4.9.0.4...v4.9.0.5
[v4.9.0.4]: https://github.com/pcengines/coreboot/compare/v4.9.0.3...v4.9.0.4
[v4.9.0.3]: https://github.com/pcengines/coreboot/compare/v4.9.0.2...v4.9.0.3
[v4.9.0.2]: https://github.com/pcengines/coreboot/compare/v4.9.0.1...v4.9.0.2
[v4.9.0.1]: https://github.com/pcengines/coreboot/compare/v4.8.0.7...v4.9.0.1
[v4.8.0.7]: https://github.com/pcengines/coreboot/compare/v4.8.0.6...v4.8.0.7
[v4.8.0.6]: https://github.com/pcengines/coreboot/compare/v4.8.0.5...v4.8.0.6
[v4.8.0.5]: https://github.com/pcengines/coreboot/compare/v4.8.0.4...v4.8.0.5
[v4.8.0.4]: https://github.com/pcengines/coreboot/compare/v4.8.0.3...v4.8.0.4
[v4.8.0.3]: https://github.com/pcengines/coreboot/compare/v4.8.0.2...v4.8.0.3
[v4.8.0.2]: https://github.com/pcengines/coreboot/compare/v4.8.0.1...v4.8.0.2
[v4.8.0.1]: https://github.com/pcengines/coreboot/compare/v4.6.9...v4.8.0.1
[v4.6.10]: https://github.com/pcengines/coreboot/compare/v4.6.9...v4.6.10
[v4.6.9]: https://github.com/pcengines/coreboot/compare/v4.6.8...v4.6.9
[v4.6.8]: https://github.com/pcengines/coreboot/compare/v4.6.7...v4.6.8
[v4.6.7]: https://github.com/pcengines/coreboot/compare/v4.6.6...v4.6.7
[v4.6.6]: https://github.com/pcengines/coreboot/compare/v4.6.5...v4.6.6
[v4.6.5]: https://github.com/pcengines/coreboot/compare/v4.6.4...v4.6.5
[v4.6.4]: https://github.com/pcengines/coreboot/compare/v4.6.3...v4.6.4
[v4.6.3]: https://github.com/pcengines/coreboot/compare/v4.6.2...v4.6.3
[v4.6.2]: https://github.com/pcengines/coreboot/compare/v4.6.1...v4.6.2
[v4.6.1]: https://github.com/pcengines/coreboot/compare/v4.6.0...v4.6.1
[v4.6.0]: https://github.com/pcengines/coreboot/compare/v4.5.8...v4.6.0
[v4.5.8]: https://github.com/pcengines/coreboot/compare/v4.5.7...v4.5.8
[v4.5.7]: https://github.com/pcengines/coreboot/compare/v4.5.6...v4.5.7
[v4.5.6]: https://github.com/pcengines/coreboot/compare/v4.5.5.2...v4.5.6
[v4.5.5.2]: https://github.com/pcengines/coreboot/compare/v4.5.5.1...v4.5.5.2
[v4.5.5.1]: https://github.com/pcengines/coreboot/compare/v4.5.5...v4.5.5.1
[v4.5.5]: https://github.com/pcengines/coreboot/compare/v4.5.4...v4.5.5
[v4.5.4]: https://github.com/pcengines/coreboot/compare/v4.5.3...v4.5.4
[v4.5.3]: https://github.com/pcengines/coreboot/compare/v4.5.2...v4.5.3
