# Miscellaneous

Besides the main collision detection and mesh generation related features,
**ncollide** exposes a number of unclassified operations that are used
internally by the library. Those operations are exported by the `utils` module.
Here are listed some of the most useful operations:

| Function                    | Description                                          |
|--                           | --                                                   |
| `center(pts)`               | Computes the center of the points `pts`.             |
| `circumcircle(a, b, c)`     | Computes the circumcircle of the triangle `a, b, c`. |
| `cov(pts)`                  | Computes the covariance matrix of the points `pts`.  |
| `is_point_in_triangle(...)` | Tests that a point is inside of a triangle.          |
| `sort3(a, b, c)`            | Sorts in increasing order a set of three values.     |
| `triangulate(pts)`          | Triangulates the points `pts`.                       |

See the [API documentation](../rustdoc/ncollide3d/utils/index.html) for an
exhaustive list.
