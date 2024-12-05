file(REMOVE_RECURSE
  "libhebic++.pdb"
  "libhebic++.so"
  "libhebic++.so.3.11.1"
)

# Per-language clean rules from dependency scanning.
foreach(lang CXX)
  include(CMakeFiles/hebic++.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
