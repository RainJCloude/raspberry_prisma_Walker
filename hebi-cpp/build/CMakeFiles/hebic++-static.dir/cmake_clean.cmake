file(REMOVE_RECURSE
  "libhebic++-static.a"
  "libhebic++-static.pdb"
)

# Per-language clean rules from dependency scanning.
foreach(lang CXX)
  include(CMakeFiles/hebic++-static.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
