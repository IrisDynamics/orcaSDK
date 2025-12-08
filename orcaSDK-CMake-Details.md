# orcaSDK CMake Details

orcaSDK supports CMake as a config package. It is available for use through the common `find_package()` `target_link_libraries()` method for including dependencies.

We don't go into the details on how to set up a custom installation, but here are the relevant details if you wish to install the SDK and you wish to supply your own dependencies.

## Dependencies

- [ASIO standalone](https://think-async.com/Asio/)
  - Expected project name: `asio`
  - Expected library name: `asio::asio`
- [orcaAPI](https://github.com/IrisDynamics/orca-api)
  - Expected project name: `orcaAPI`
  - Expected library name: `orcaAPI::orcaAPI`

## Disabling Automatic Dependency Downloads

We define two cache variables which determine whether the SDK will attempt to download its dependencies or whether it will simply use `find_package` and `target_link_libraries`. The variables are:

- `orcaSDK_VENDOR_ASIO` - Default to `ON`. Will automatically download ASIO and define its own library for it to link against.
- `orcaSDK_VENDOR_ORCAAPI` - Default to `ON`. Will automatically download the orca API and link itself against it.

Setting these variables to `OFF` will cause the SDK to search for locally installed versions of the relevant dependency.