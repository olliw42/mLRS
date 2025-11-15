# mLRS - Firmware #

This folder contains pre-compiled firmware binaries.

## Version Numbers ##

mLRS uses version numbers with format major.minor.patch (e.g., 1.3.06), where the patch number has the following meaning:

* patch = 00: official release (e.g., v1.3.00)
* patch = even: official pre-release (e.g., v1.3.06)
* patch = odd: development version (e.g., v1.3.07)


## Notes for Devs ##

### Releases (00 version numbers)

- the binaries of a release can go into whatever subfolders, for v1.3.00 they had been release-stm32, release-matek, release-esp
- releases go into an extra branch, with name "1.3", commit message "v1.3 (release)", and tag "v1.3-release"
- the tag needs to be added to the file tools/web/mlrs_firmware_urls.json, so that flash tools can find it

### Pre-releases (even version numbers)

- the binaries of a pre-release go into folders pre-release-stm32 and pre-release-esp
- the pre-release needs to be commited with commit message "v1.3.06 (pre-release)" and tagged with "v1.3.06-pre-release"
- the tag needs to be added to the file tools/web/mlrs_firmware_urls.json, so that flash tools can find it

### Dev releases (odd version numbers)

- the binaries of a dev version go into the pre-release-stm32 and pre-release-esp folders (overwriting pre-release versions)
- the filenames include the SHA with a "-@", such as "-@ae667b78", this is used by flash tools to identify dev versions
- dev release need NOT be tagged, they are identified by being in main and having the SHA in the file name
