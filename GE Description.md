## Terms and functions

- **VisiblePoints **: Number of point visible from all the selected OCs when rendered from a top view.

- **VisibleInliers** : Number of VisiblePoints point that has an inlier in the point cloud(the one captured by scape).

- **UniqueVisibleInliers**: Number Visible Inliers that does not share indexes in the Point Cloud. If an OC is in collision with with multiple other the one with with most shared inliers are used.

- **PointCloudPoints**: Number of points in the point cloud.

- **CollisionDepth:** The maximum penetration depth for a OC in a given hypothesis.

- **OCScore**: The ones given by scape.

- **PenalizedVisibleInliers**: 
  $$
  \text{PenalizedVisibleInliers}=\sum_{SelectedOC(SOC)} \text{VisiblePoints}_{\text{SOC}}\cdot \sigma(\text{CollisionDepth}_{\text{SOC}})
  $$

  $$
  \sigma(x) =\frac{1}{1+e^{-g*(x-c)}}
  $$

  


  - ***g***: sigmoid growth rate, usually from 5-10, ***c***: sigmoid center, usually around 2-5 mm.

- ***InliersThreshold***(0-1): Theshold to determine what the ration between VisiblePoints and PenalizedVisibleInliers needs to be for cost to increase. Used differently in GEIC  than in  GEICS and GEUICS.

  


## Genetic Evaluators(GEs)

### Genetic Evaluator Inlier Collision(GEIC)


$$
\text{cost} = \text{VisiblePoints} - \frac{\text{PenalizedVisibleInliers}}{\text{InliersThreshold}}
$$



### Genetic Evaluator Inlier Collision Scaled(GEICS)

$$
\text{cost} = 1- \frac{\text{VisibleInliers}}{\text{PointCloudPoints}+\max(\text{InlierThreshold} * \text{VisiblePoints}-\text{PenalizedVisibleInliers},0)}
$$

### Genetic Evaluator Unique Inlier Collision Scaled(GEUICS) 

$$
\text{cost} = 1- \frac{\text{UniqueVisibleInliers}}{\text{PointCloudPoints}+\max(\text{InlierThreshold} * \text{VisiblePoints}-\text{PenalizedVisibleInliers},0)}
$$

### Genetic Evaluator Score Collision(GESC)

$$
cost = -\sum_{SelectedOC(SOC)} \text{OCScore}_\text{SOC}*(1-2*\sigma(\text{CollisionDepth}_\text{SOC}))
$$



