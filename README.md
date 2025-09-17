# Pnp

## Repository design principle

Clear delineation between the public and private interfaces of a library is required to effectively manage change at
scale.

In this repository we have chosen a directory and file structure that emphasizes this separation by placing the public
interface header files in the [include/pnp/](code/include/pnp/) folder. The headers defining the internal private
interface are found in the [src/](code/src/) folder directly next to their associated implementation and unit test
files. During library development both the public and private headers are available (i.e. part of the `BUILD_INTERFACE`)
but when the library is installed only the public headers are present and available (i.e. part of the
`INSTALL_INTERFACE`).

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
* Benefit #2) Easier maintenance of the public API - The entire public API is described in full by the header files
  in [include/pnp/](code/include/pnp/) and their associated tests in [test/](code/test/). Therefore, a developer can
  easily recognize when they are making changes to the public API that will break the interface for downstream
  consumers. This makes the connection between [semantic versioning](https://semver.org/) and the source code a literal
  structural component of the repository.
* Benefit #3) Reduced source code dependencies - Dependencies on source code can be especially hard to manage. In C++
  source code dependencies on other projects are introduced via the `#include` directive. All the included files in
  the public API (i.e. those found at the top of the files found in [include/pnp/](code/include/pnp/)) must be
  present on the system of the user consuming the library. Therefore, there is a premium on reducing the number of
  includes in the public API and in some cases entirely redesigning the public API to remove "exotic" includes. If for
  example in the public API of your header there is a requirement for an exotic include, for example a JSON class
  defined in a third-party Github repository - and there is no mechanism to automatically install it - you can basically
  count on the fact that your libray is an order of magnitude harder to use than it could be. Thoughtfully designing and
  only installing a clearly defined public API can dramatically reduce the source code dependencies of an installed
  library.
* Benefit #4) Tight coupling of source code and unit testing - Searching for corresponding .hpp, .cpp and the
  corresponding unit test files often consume an unnecessary amount of time. By putting these all directly adjacent to
  one another in the [src/](code/src/) folder we make it easy to find corresponding files and make the relationship
  between files a structural part of the repository itself.

## Open implementation questions

* What to do with K
* How to handle noise and covariance estimation
