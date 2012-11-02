FILE(REMOVE_RECURSE
  "CMakeFiles/ROSBUILD_genmsg_py"
  "src/path_navigator/msg/__init__.py"
  "src/path_navigator/msg/_Waypoints.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
