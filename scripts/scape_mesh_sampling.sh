#!/bin/bash
for f in *.ply; do
  pcl_mesh_sampling $f "${f%.ply}.pcd" -leaf_size "$@" -write_normals 1 -no_vis_result 1
done
