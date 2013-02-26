#ifndef __MESH_FILTER_MESH_FILTER_BASE_H__
#define	__MESH_FILTER_MESH_FILTER_BASE_H__

#include <map>
#include <mesh_filter/gl_renderer.h>
#include <mesh_filter/sensor_model.h>
#include <boost/function.hpp>
#include <eigen3/Eigen/Eigen>

//forward declarations
namespace shapes
{
  class Mesh;
}

namespace mesh_filter
{

class GLMesh;

typedef unsigned int MeshHandle;
class MeshFilterBase
{
    // inner types and typedefs
  public:
    typedef boost::function<bool (MeshHandle, Eigen::Affine3d&)> TransformCallback;
    enum {Background = 0, Shadow, FirstLabel = 16};
  public:
    /**
     * \brief Constructor
     * \author Suat Gedikli (gedikli@willowgarage.com)
     * \param[in] transform_callback Callback function that is called for each mesh to obtain the current transformation.
     * \note the callback expects the mesh handle but no time stamp. Its the users responsibility to return the correct transformation.
     */
    MeshFilterBase (const TransformCallback& transform_callback,
                    const SensorModel::Parameters& sensor_parameters,
                    const std::string& render_vertex_shader = "", const std::string& render_fragment_shader = "",
                    const std::string& filter_vertex_shader = "", const std::string& filter_fragment_shader = "");
    
    /** \brief Desctructor */
    ~MeshFilterBase ();
    
    /**
     * \brief adds a mesh to the filter object.
     * \author Suat Gedikli (gedikli@willowgarage.com)
     * \param[in] mesh the mesh to be added
     * \return handle to the mesh. This handle is used in the transform callback function to identify the mesh and retrieve the correct transformation.
     */
    MeshHandle addMesh (const shapes::Mesh& mesh);
    
    /**
     * \brief removes a mesh given by its handle
     * \author Suat Gedikli (gedikli@willowgarage.com)
     * \param[in] the handle of the mesh to be removed.
     */
    void removeMesh (MeshHandle);

    /**
     * \brief label/remove pixels from input depth-image
     * \author Suat Gedikli (gedikli@willowgarage.com)
     * \param[in] sensor_data pointer to the input depth image from sensor readings.
     * \param[in] width width of the input data in pixels
     * \param[in] height height of the input data in pixels
     * \param[in] fx focal length in x-direction of the camera in pixels
     * \param[in] fy focal length in y-direction of the camera in pixels
     * \param[in] cx x-component of the principal point in pixels
     * \param[in] cy y-component of the principal point in pixels
     */
    void filter (const float* sensor_data) const;

    
    /**
     * \brief retrieves the labels of the input data
     * \author Suat Gedikli (gedikli@willowgarage.com)
     * \param[out] labels pointer to buffer to be filled with labels
     * \note labels are corresponding 1-1 to the mesh handles. 0 and 1 are reserved indicating either background (0) or shadow (1)
     *       The upper 8bit of a label is filled with the user given flag (see addMesh)
     */
    void getFilteredLabels (unsigned* labels) const;
    
    /**
     * \brief retrieves the filtered depth values
     * \author Suat Gedikli (gedikli@willowgarage.com)
     * \param[out] depth pointer to buffer to be filled with depth values.
     */
    void getFilteredDepth (float* depth) const;
    
    /**
     * \brief retrieves the labels of the rendered model
     * \author Suat Gedikli (gedikli@willowgarage.com)
     * \param[out] labels pointer to buffer to be filled with labels
     * \note labels are corresponding 1-1 to the mesh handles. 0 and 1 are reserved indicating either background (0) or shadow (1)
     *       The upper 8bit of a label is filled with the user given flag (see addMesh)
     */
    void getModelLabels (unsigned* labels) const;
    
    /**
     * \brief retrieves the depth values of the rendered model
     * \author Suat Gedikli (gedikli@willowgarage.com)
     * \param[out] depth pointer to buffer to be filled with depth values.
     */
    void getModelDepth (float* labels) const;
    
    /**
     * \brief set the shadow threshold. points that are further away than the rendered model are filtered out.
     *        Except they are further away than this threshold. Then these points are kept, but its label is set to
     *        1 indicating that it is in the shadow of the model
     * \author Suat Gedikli (gedikli@willowgarage.com)
     * \param[in] threshold shadow threshold in meters
     */
    void setShadowThreshold (float threshold);
    
    /** 
     * \brief set the callback for retrieving transformations for each mesh.
     * \author Suat Gedikli (gedikli@willowgarage.com)
     * \param[in] transform_callback the callback
     */
    void setTransformCallback (const boost::function<bool (MeshHandle, Eigen::Affine3d&)>& transform_callback);
    
    /**
     * \brief set the scale component of padding used to multiply with sensor-specific padding coefficients to get final coefficients.
     * \param[in] scale the scale value
     */
    void setPaddingScale (float scale);
    
    /**
     * \brief set the offset component of padding. This value is added to the scaled sensor-specific constant component.
     * \param[in] offset the offset value
     */
    void setPaddingOffset (float offset);
  protected:
  /**
   * \brief sets the size of the fram buffers
   * \author Suat Gedikli (gedikli@willowgarage.com)
   * \param[in] width width of frame buffers in pixels
   * \param[in] height height of frame buffers in pixels
   */
    void setSize (unsigned width, unsigned height);
    
    /**
     * \brief merges vertices that are close to each other to a single vertex.
     * \param[in] mesh the mesh to be merged.
     * \param[out] compressed the compressed mesh with unique vertices
     */
    static void mergeVertices (const shapes::Mesh& mesh, shapes::Mesh& compressed);
    
    /** \brief storage for meshed to be filtered */
    std::map<MeshHandle, GLMesh*> meshes_;    

    /** \brief the parameters of the used sensor model*/
    boost::shared_ptr<SensorModel::Parameters> sensor_parameters_;
    
    /** \brief first pass renderer for rendering the mesh*/
    mutable GLRenderer mesh_renderer_;
    
    /** \brief second pass renderer for filtering the results of first pass*/
    mutable GLRenderer depth_filter_;
    
    /** \brief canvas element (screen-filling quad) for second pass*/
    GLuint canvas_;
    
    /** \brief handle depth texture from sensor data*/
    GLuint sensor_depth_texture_;
    
    /** \brief handle to GLSL location of shadow threshold*/
    GLuint shadow_threshold_location_;
    
    /** \brief next handle to be used for next mesh that is added*/
    MeshHandle next_handle_;
    
    /** \brief callback function for retrieving the mesh transformations*/
    boost::function<bool (MeshHandle, Eigen::Affine3d&)> transform_callback_;
    
    /** \brief padding scale*/
    float padding_scale_;
    
    /** \brief padding offset*/
    float padding_offset_;

    /** \brief threshold for shadowed pixels vs. filtered pixels*/
    float shadow_threshold_;
    
};
} // namespace mesh_filter

#endif	/* __MESH_FILTER_MESH_FILTER_BASE_H__ */

