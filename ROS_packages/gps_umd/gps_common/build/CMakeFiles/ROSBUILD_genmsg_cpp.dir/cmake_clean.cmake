FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/gps_common/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/gps_common/GPSFix.h"
  "../msg_gen/cpp/include/gps_common/GPSStatus.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
