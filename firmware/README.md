# mLRS - Firmware #

This folder contains pre-compiled firmware binaries.

## Version Numbers ##

mLRS uses versions with format major.minor.patch (e.g., 1.3.06), where the patch number has the following meaning:

* patch = 00: official release (e.g., v1.3.00)
* patch = even: official pre-release (e.g., v1.3.06)
* patch = odd: development version (e.g., v1.3.07)


## Notes for the Devs ##

### Pre-releases (even version numbers)

- pre-releases need to be tagged, such as "v1.3.06-pre-release"
- the tag needs to be entered into the file tools/web/mlrs_firmware_urls.json, so that the flash tools can find it

### Releases (00 version numbers)

- releases go into an extra branch, e.g. "1.3", and need to be tagged, such as "v1.3-release"
- the tag needs to be entered into the file tools/web/mlrs_firmware_urls.json, so that the flash tools can find it
