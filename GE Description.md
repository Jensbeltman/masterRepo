### Terms

- **VisiblePoints **: Number of point visible from all the selected OC's when rendered from a top view.

- **VisibleInliers** : Number of VOP point that has and inlier in the point cloud.

- **UniqueVisibleInliers**: Number Visible Inliers that does not share indexes in the Point Cloud. Only  if more than two OC's are colliding  is .

- **InliersThreshold**: Theshold to determine what the ration between VOP and VOI needs to be for cost to increase. Used differently in GEIC  than in  GEICS and GEUICS.

- **PointCloudPoints**: Number of points in the pointcloud(the one captured by scape).

- **CollisionDepth:** The maximum penetration depth for a OC in a given hypothesis.

- **OCScore**: Score for a given OC, in the scape dataset this is given.

- $\mathbf{\sigma(x)}$: function used for discounting inliers based on collision
  $$
  \sigma(x) =\frac{1}{1+e^{-g*(x-c)}}
  $$
  

  - **g**: sigmoid growth rate
  - **c**: sigmoid center

- **PenalizedVisibleInliers**: 
  $$
  \text{PenalizedVisibleInliers}=\sum_{SelectedOC(SOC)} \text{VisiblePoints}_{\text{SOC}}\cdot \sigma(\text{CollisionDepth}_{\text{SOC}})
  $$
  

### GEIC: Genetic Evaluator Inlier Collision


$$
\text{cost} = \text{VisiblePoints} - \frac{\text{PenalizedVisibleInliers}}{\text{InliersThreshold}}
$$



### GEICS: Genetic Evaluator Inlier Collision Scaled

$$
\text{cost} = 1- \frac{\text{VisibleInliers}}{\text{PointCloudPoints}+\max(\text{InlierThreshold} * \text{VisiblePoints}-\text{PenalizedVisibleInliers},0)}
$$

### GEUICS: Genetic Evaluator Unique Inlier Collision Scaled 

$$
\text{cost} = 1- \frac{\text{UniqueVisibleInliers}}{\text{PointCloudPoints}+\max(\text{InlierThreshold} * \text{VisiblePoints}-\text{PenalizedVisibleInliers},0)}
$$

### GESC: Genetic Evaluator Score Collision

$$
cost = -\sum_{SelectedOC(SOC)} \text{OCScore}_\text{SOC}*(1-2*\sigma(\text{CollisionDepth}_\text{SOC}))
$$



