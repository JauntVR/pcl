/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#ifndef VTK_MESH_QUADRIC_CLUSTERING_H_
#define VTK_MESH_QUADRIC_CLUSTERING_H_

#include <pcl/surface/processing.h>
#include <pcl/surface/vtk_smoothing/vtk.h>

namespace pcl
{
  /** \brief PCL mesh decimation based on vtkQuadricClustering from the VTK library.
    * Please check out the original documentation for more details on the inner workings of the algorithm
    * Warning: This wrapper does two fairly computationally expensive conversions from the PCL PolygonMesh
    * data structure to the vtkPolyData data structure and back.
    */
  class PCL_EXPORTS MeshQuadricClusteringVTK : public MeshProcessing
  {
    public:
      /** \brief Empty constructor */
      MeshQuadricClusteringVTK ();

      /** \brief Set whether clustering will use auto devisions
        * \param[in] auto_divisions auto division flag
        */
      inline void
      setAutoAdjustNumberOfDivisions(bool auto_divisions)
      {
        auto_adjust_divisions_ = auto_divisions;
      }
      
      /** \brief Get the auto division status*/
      inline bool
      getAutoAdjustNumberOfDivisions()
      {
        return auto_adjust_divisions_;
      }

      /** \brief Set to use input point
        * \param[in] use_points use points
        */
      inline void
      setUseInputPoints(bool use_points)
      {
        use_input_points_ = use_points;
      }
      
      /** \brief Get the use points status*/
      inline bool
      getUseInputPoints()
      {
        return use_input_points_;
      }

      /** \brief Set the x divisions
        * \param[in] xdivision X divisions
        */
      inline void
      setNumberOfXDivisions(int xdivision)
      {
        x_divisions_ = xdivision;
      }

      /** \brief Get the x division */
      inline int
      getNumberOfXDivisions()
      {
        return x_divisions_;
      }

      /** \brief Set the Y divisions
        * \param[in] xdivision Y divisions
        */
      inline void
      setNumberOfYDivisions(int ydivision)
      {
        y_divisions_ = ydivision;
      }
      
      /** \brief Get the y division */
      inline int
      getNumberOfYDivisions()
      {
        return y_divisions_;
      }
 
      /** \brief Set the Z divisions
        * \param[in] xdivision Z divisions
        */
      inline void
      setNumberOfZDivisions(int zdivision)
      {
        z_divisions_ = zdivision;
      }
      
      /** \brief Get the z division */
      inline int
      getNumberOfZDivisions()
      {
        return z_divisions_;
      }

    protected:
      void
      performProcessing (pcl::PolygonMesh &output);

    private:
      bool use_input_points_;
      bool auto_adjust_divisions_;
      int x_divisions_;
      int y_divisions_;
      int z_divisions_;

      vtkSmartPointer<vtkPolyData> vtk_polygons_;
  };
}
#endif /* VTK_MESH_QUADRIC_CLUSTERING_H_ */
