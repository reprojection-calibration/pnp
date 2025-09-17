# Pnp

## Repository design principle

Clear delineation between the public and private interfaces of a library is required to effectively manage change at
scale.

In this repository we have chosen a directory and file structure that emphasizes this seperation by placing the public
interface header files in the [include/pnp/](code/include/pnp/) folder. The headers defining the internal private
interface are found in the [src/](code/src/) folder directly next to their associated implementation and unit test
files. This structure helps the developer clearly understand what the public interface actually is and forces the us to
consciously think about breaking changes.

* Benefit #1) Hidden private API - There is a rule in software engineering that says the entire accessible part of an
  API, given enough users, will be entirely used. Even parts of the API which the development team did not intend for
  others to use. Separating the includes as we do, and only allowing the instalation of the public headers found
  in [include/pnp/](code/include/pnp/) physically prevents consumers of the library from using the private API, because
  it will never even be installed.
  Another way to achieve this is by putting the private internal API into a subnamespace like `::internal{}`.
  This is a effective, but is still liable to be abused by developers ignoring your intention and use functions in the
  private namespace anyway. Our preference is not even temp a consumer of the libray with the private API and simply not
  install it. We should not however that for header only libraries, a common way to deliver C++ code (ex. Eigen), the
  private namespace method is the only available option to hide the internal API.

## Open implementation questions
