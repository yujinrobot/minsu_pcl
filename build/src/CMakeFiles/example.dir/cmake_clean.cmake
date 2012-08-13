FILE(REMOVE_RECURSE
  "CMakeFiles/example.dir/example.o"
  "../../bin/example.pdb"
  "../../bin/example"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang CXX)
  INCLUDE(CMakeFiles/example.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
