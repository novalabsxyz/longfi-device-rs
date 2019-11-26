<!--
M.m.p (YYYY-MM-DD)
==================
Add a summary of this release.

**BREAKING CHANGES**:

* Some change which breaks API or ABI compatiblity with.


Feature enhancements:

* [Link to github PR]():
  A new feature.

Bug fixes:

* [Link to github PR]():
  A bugfix.
-->
0.1.2 (2019-11-26)
==================
* [Workaround for SX126x](https://github.com/helium/longfi-device/pull/26)
Bump longfi-device submodule to allow SX126x to work
* [Include only used radio in binary output](https://github.com/helium/longfi-device-rs/pull/23)
API is changed so that its easier for rustc to only include relevant radio


0.1.1 (2019-11-13)
==================
Fixed bug where fingerprints failed due to memory misalignment with C bindings

0.1.0 (2019-10-28)
==================
Initial release.
