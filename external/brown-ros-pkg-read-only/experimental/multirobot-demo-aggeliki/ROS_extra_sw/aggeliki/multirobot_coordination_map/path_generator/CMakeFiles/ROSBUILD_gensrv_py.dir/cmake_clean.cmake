FILE(REMOVE_RECURSE
  "CMakeFiles/ROSBUILD_gensrv_py"
  "src/path_generator/srv/__init__.py"
  "src/path_generator/srv/_generatePath.py"
  "src/path_generator/srv/_availableNextHops.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
