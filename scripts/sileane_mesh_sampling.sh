#bin/bash
for d in *; do
  echo $d/mesh.ply
  pcl_mesh_sampling $d/mesh.ply $d/mesh.pcd -leaf_size 0.001 -write_normals 1 -no_vis_result 1
done
