#include "student_code.h"
#include "mutablePriorityQueue.h"

using namespace std;

namespace CGL
{

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (class member).
   *
   * @param points A vector of points in 2D
   * @return A vector containing intermediate points or the final interpolated vector
   */
  std::vector<Vector2D> BezierCurve::evaluateStep(std::vector<Vector2D> const &points)
  { 
    // TODO Part 1.
      
      std::vector<Vector2D> nextPoints;

      for (size_t i = 0; i < points.size() - 1; ++i)
      {
        Vector2D newPoint = (1 - t) * points[i] + t * points[i + 1];
        nextPoints.push_back(newPoint);
      }

      return nextPoints;
//    return std::vector<Vector2D>();
  }

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (function parameter).
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return A vector containing intermediate points or the final interpolated vector
   */
  std::vector<Vector3D> BezierPatch::evaluateStep(std::vector<Vector3D> const &points, double t) const
  {
    // TODO Part 2.
      std::vector<Vector3D> nextPoints;

      for (size_t i = 0; i < points.size() - 1; ++i)
      {
          Vector3D newPoint = (1 - t) * points[i] + t * points[i + 1];
          nextPoints.push_back(newPoint);
      }

      return nextPoints;
//    return std::vector<Vector3D>();
  }

  /**
   * Fully evaluates de Casteljau's algorithm for a vector of points at scalar parameter t
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> const &points, double t) const
  {
    // TODO Part 2.
      std::vector<Vector3D> currentPoints = points;

      while (currentPoints.size() > 1)
      {
          currentPoints = evaluateStep(currentPoints, t);
      }

      return currentPoints.front();
//    return Vector3D();
  }

  /**
   * Evaluates the Bezier patch at parameter (u, v)
   *
   * @param u         Scalar interpolation parameter
   * @param v         Scalar interpolation parameter (along the other axis)
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate(double u, double v) const 
  {  
    // TODO Part 2.
      std::vector<Vector3D> intermediatePoints;

      //
      for (const auto &row : controlPoints)
      {
          intermediatePoints.push_back(evaluate1D(row, u));
      }

      return evaluate1D(intermediatePoints, v);
//    return Vector3D();
  }

  Vector3D Vertex::normal( void ) const
  {
    // TODO Part 3.
    // Returns an approximate unit normal at this vertex, computed by
    // taking the area-weighted average of the normals of neighboring
    // triangles, then normalizing.
      
      
      Vector3D sumNormals(0, 0, 0);
      HalfedgeCIter h = halfedge();

      do {
          Vector3D vertex1 = h->vertex()->position;
          Vector3D vertex2 = h->next()->vertex()->position;
          Vector3D vertex3 = h->next()->next()->vertex()->position;

          Vector3D edge1 = vertex2 - vertex1;
          Vector3D edge2 = vertex3 - vertex1;

          Vector3D normal = cross(edge1, edge2);
          sumNormals += normal;

          h = h->twin()->next();
      } while (h != halfedge());

      return sumNormals.unit();
//    return Vector3D();
  }


EdgeIter HalfedgeMesh::flipEdge(EdgeIter e0) {
    HalfedgeIter h0 = e0->halfedge();
    HalfedgeIter h1 = h0->next();
    HalfedgeIter h2 = h1->next();
    HalfedgeIter h3 = h0->twin()->next();
    HalfedgeIter h4 = h3->next();
    HalfedgeIter h0_twin = h0->twin();
    HalfedgeIter h1_twin = h1->twin();
    HalfedgeIter h2_twin = h2->twin();
    HalfedgeIter h3_twin = h3->twin();
    HalfedgeIter h4_twin = h4->twin();
    
    FaceIter f0 = h0->face();
    FaceIter f1 = h0->twin()->face();

    VertexIter v0 = h0->vertex();
    VertexIter v1 = h0_twin->vertex();
    VertexIter v2 = h2->vertex();
    VertexIter v3 = h4->vertex();

    EdgeIter e1 = h1->edge();
    EdgeIter e2 = h2->edge();
    EdgeIter e3 = h0_twin->next()->edge();
    EdgeIter e4 = h3->next()->edge();


    h0->setNeighbors(h1, h0_twin, v3, e0, f0);
    h1->setNeighbors(h2, h2_twin, v2, e2, f0);
    h2->setNeighbors(h0, h3_twin, v0, e3, f0);
    h3->setNeighbors(h4, h4_twin, v3, e4, f1);
    h4->setNeighbors(h0_twin, h1_twin, v1, e1, f1);
    
    h0_twin->setNeighbors(h3, h0, v2, e0, f1);
    h1_twin->setNeighbors(h1_twin->next(), h4, v2, e1, h1_twin->face());
    h2_twin->setNeighbors(h2_twin->next(), h1, v0, e2, h2_twin->face());
    h3_twin->setNeighbors(h3_twin->next(), h2, v3, e3, h3_twin->face());
    h4_twin->setNeighbors(h4_twin->next(), h3, v1, e4, h4_twin->face());

    v0->halfedge() = h2;
    v1->halfedge() = h4;
    v2->halfedge() = h0_twin;
    v3->halfedge() = h0;

    e0->halfedge() = h0;
    e1->halfedge() = h4;
    e2->halfedge() = h1;
    e3->halfedge() = h2;
    e4->halfedge() = h3;

    f0->halfedge() = h0;
    f1->halfedge() = h0_twin;

    return e0;
    }


  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
  {
    // TODO Part 5.
    // This method should split the given edge and return an iterator to the newly inserted vertex.
    // The halfedge of this vertex should point along the edge that was split, rather than the new edges.
    return VertexIter();
  }



  void MeshResampler::upsample( HalfedgeMesh& mesh )
  {
    // TODO Part 6.
    // This routine should increase the number of triangles in the mesh using Loop subdivision.
    // One possible solution is to break up the method as listed below.

    // 1. Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
    // and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
    // a vertex of the original mesh.
    
    // 2. Compute the updated vertex positions associated with edges, and store it in Edge::newPosition.
    
    // 3. Split every edge in the mesh, in any order. For future reference, we're also going to store some
    // information about which subdivide edges come from splitting an edge in the original mesh, and which edges
    // are new, by setting the flat Edge::isNew. Note that in this loop, we only want to iterate over edges of
    // the original mesh---otherwise, we'll end up splitting edges that we just split (and the loop will never end!)
    
    // 4. Flip any new edge that connects an old and new vertex.

    // 5. Copy the new vertex positions into final Vertex::position.

  }
}
