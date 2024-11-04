##This file is based off the sample from GoogleTest's Github repo. See https://github.com/google/googletest/blob/main/googletest/README.md
include(GoogleTest)

include(FetchContent)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  # Specify the commit you depend on and update it regularly.
  GIT_TAG b514bdc898e2951020cbdca1304b75f5950d1f59
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)