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

    if (e0->isBoundary()) {
        return e0;
    }
    HalfedgeIter h0 = e0->halfedge();
    HalfedgeIter h1 = h0->next();
    HalfedgeIter h2 = h1->next();
    HalfedgeIter h3 = h0->twin()->next();
    HalfedgeIter h4 = h3->next();

    FaceIter f0 = h0->face();
    FaceIter f1 = h0->twin()->face();

    VertexIter v0 = h0->vertex();
    VertexIter v1 = h0->twin()->vertex();
    VertexIter v2 = h2->vertex();
    VertexIter v3 = h4->vertex();

    EdgeIter e1 = h1->edge();
    EdgeIter e2 = h2->edge();
    EdgeIter e3 = h0->twin()->next()->edge();
    EdgeIter e4 = h3->next()->edge();


    h0->setNeighbors(h1, h0->twin(), v3, e0, f0);
    h1->setNeighbors(h2, h2->twin(), v2, e2, f0);
    h2->setNeighbors(h0, h3->twin(), v0, e3, f0);
    h3->setNeighbors(h4, h4->twin(), v3, e4, f1);
    h4->setNeighbors(h0->twin(), h1->twin(), v1, e1, f1);
    
    h0->twin()->setNeighbors(h3, h0, v2, e0, f1);
    h1->twin()->setNeighbors(h1->twin()->next(), h4, v2, e1, h1->twin()->face());
    h2->twin()->setNeighbors(h2->twin()->next(), h1, v0, e2, h2->twin()->face());
    h3->twin()->setNeighbors(h3->twin()->next(), h2, v3, e3, h3->twin()->face());
    h4->twin()->setNeighbors(h4->twin()->next(), h3, v1, e4, h4->twin()->face());

    v0->halfedge() = h2;
    v1->halfedge() = h4;
    v2->halfedge() = h0->twin();
    v3->halfedge() = h0;

    e0->halfedge() = h0;
    e1->halfedge() = h4;
    e2->halfedge() = h1;
    e3->halfedge() = h2;
    e4->halfedge() = h3;

    f0->halfedge() = h0;
    f1->halfedge() = h0->twin();

    return e0;
    }
    


VertexIter HalfedgeMesh::splitEdge(EdgeIter e0) {
  if (e0->isBoundary()) {
    // Ignore boundary case
    return e0->halfedge()->vertex();
  }

    HalfedgeIter h0 = e0->halfedge();
    HalfedgeIter h1 = h0->next();
    HalfedgeIter h2 = h1->next();
    HalfedgeIter h3 = h0->twin()->next()->next();
    HalfedgeIter h4 = h0->twin()->next();

    HalfedgeIter h5 = newHalfedge();
    HalfedgeIter h6 = newHalfedge();
    HalfedgeIter h7 = newHalfedge();
    HalfedgeIter h8 = newHalfedge();
    HalfedgeIter h9 = newHalfedge();
    HalfedgeIter h10 = newHalfedge();
    
    EdgeIter e1 = h1->edge();
    EdgeIter e2 = h2->edge();
    EdgeIter e3 = h4->edge();
    EdgeIter e4 = h3->edge();
    EdgeIter e5 = newEdge();
    EdgeIter e6 = newEdge();
    EdgeIter e7 = newEdge();

VertexIter ret_v = newVertex();
    VertexIter v0 = h0->vertex();
    VertexIter v1 = h0->twin()->vertex();
    VertexIter v2 = h2->vertex();
    VertexIter v3 = h3->vertex();


    FaceIter f0 = h0->face();
    FaceIter f1 = h0->twin()->face();
    FaceIter f2 = newFace();
    FaceIter f3 = newFace();

    h0->setNeighbors(h1, h0->twin(), ret_v, e0, f3);
    h1->setNeighbors(h2, h1->twin(), v1, e1, f3);
    h2->setNeighbors(h0, h5, v2, e7, f3);
    h0->twin()->setNeighbors(h4, h0, v1, e0, f2);
    h4->setNeighbors(h3, h10, ret_v, e6, f2);
    h3->setNeighbors(h0->twin(), h3->twin(), v3, e4, f2);
    h1->twin()->setNeighbors(h1->twin()->next(), h1, v2, e1, h1->twin()->face());
    h2->twin()->setNeighbors(h2->twin()->next(), h6, v0, e2, h2->twin()->face());
    h4->twin()->setNeighbors(h4->twin()->next(), h9, v3, e3, h4->twin()->face());
    h3->twin()->setNeighbors(h3->twin()->next(), h3, v1, e4, h3->twin()->face());
    h5->setNeighbors(h6, h2, ret_v, e7, f0);
    h6->setNeighbors(h7, h2->twin(), v2, e2, f0);
    h7->setNeighbors(h5, h8, v0, e5, f0);
    h8->setNeighbors(h9, h7, ret_v, e5, f1);
    h9->setNeighbors(h10, h4->twin(), v0, e3, f1);
    h10->setNeighbors(h8, h4, v3, e6, f1);



    v0->halfedge() = h7;
    v1->halfedge() = h0->twin();
    v2->halfedge() = h2;
    v3->halfedge() = h10;
    ret_v->halfedge() = h0;
    ret_v->position = 0.5f * v3->position + 0.5f * v2->position;


    e0->halfedge() = h0;
    e1->halfedge() = h1;
    e2->halfedge() = h2->twin();
    e3->halfedge() = h4->twin();
    e4->halfedge() = h3;
    e5->halfedge() = h7;
    e6->halfedge() = h4;
    e7->halfedge() = h2;


    f0->halfedge() = h5;
    f1->halfedge() = h8;
    f2->halfedge() = h4;
    f3->halfedge() = h1;

    return ret_v;

    

}



void Vertex::computeCentroid() {
    HalfedgeCIter start = halfedge();
    start = start->twin();
    HalfedgeCIter current = start;
    Vector3D sum;
    size_t counter = 0;

    do {
        sum += current->vertex()->position;
        current = current->next()->twin();
        counter++;
    } while (current != start);

    if (counter != degree()) {
        cerr << "Error: Neighbours count not equal to degree" << endl;
        return;
    }

    centroid = sum;
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
      

      pointer = mesh.verticesBegin()
      while(pointer != mesh.verticesEnd()){
          VertexIter next = pointer;
          next++;
          
          pointer->isNew = false;
          
          float n = pointer-->degree();
          float u = (n == 3) ? 3.0f / 16.0f : 3.0f / (8.0f * n);

          pointer->computeCentroid();
          pointer->newPosition = (1 - (u*n)) * pointer->position + u * pointer->centroid;
          pointer = next;
          
        }

      EdgeIter pointer = mesh.edgesBegin();
      vector<EdgeIter> old;
      while(pointer != mesh.edgesEnd()){
          Vector3D p1 = e->halfedge()->vertex()->position;
          Vector3D p2 = e->halfedge()->twin()->vertex()->position;
          Vector3D p3 = e->halfedge()->next()->next()->vertex()->position;
          Vector3D p4 = e->halfedge()->twin()->next()->next()->vertex()->position;
          e->newPosition = 0.375f * (p1 + p2) + 0.125f * (p3 + p4);
        }

        for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); ++e) {
          originalEdges.push_back(e);
        }
        for (EdgeIter e : originalEdges) {
          VertexIter newV = mesh.splitEdge(e);
          newV->isNew = true;
          newV->position = e->newPosition; // Assign the computed position to the new vertex
        }

        for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); ++e) {
          if (e->isNew && ((e->halfedge()->vertex()->isNew) != (e->halfedge()->twin()->vertex()->isNew))) {
            mesh.flipEdge(e);
          }
        }

        for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); ++v) {
          if (!v->isNew) { // Only update positions of the original vertices
            v->position = v->newPosition;
          }
        }
    


  }
}
