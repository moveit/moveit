#ifndef __MESH_FILTER_STEREO_CAMERA_MODEL_H__
#define __MESH_FILTER_STEREO_CAMERA_MODEL_H__
#include <mesh_filter/sensor_model.h>
#include <string>

namespace mesh_filter
{
/**
 * \brief Model for Disparity based devices. E.g stereo camera systems or OpenNI compatible devices
 * \author Suat Gedikli <gedikli@willowgarage.com>
 */
class StereoCameraModel : public SensorModel
{
public:
  /**
   * \brief Parameters for Stereo-like devices
   * \author Suat Gedikli <gedikli@willowgarage.com>
   */
  class Parameters : public SensorModel::Parameters
  {
    public:
      /**
       * \brief Constructor
       * \param[in] width width of generated depth maps from this device
       * \param[in] height height of generated depth maps from this device
       * \param[in] near_clipping_plane_distance distance of near clipping plane
       * \param[in] far_clipping_plane_distance distance of far clipping plene
       * \param[in] fx focal length in x-direction
       * \param[in] fy focal length in y-direction
       * \param[in] cx x component of principal point
       * \param[in] cy y component of principal point
       * \param[in] base_line the distance in meters used to determine disparity values
       * \param[in] disparity_resolution resolution/quantization of disparity values in pixels
       */
      Parameters (unsigned width, unsigned height,
                         float near_clipping_plane_distance, float far_clipping_plane_distance,
                         float fx, float fy, float cx, float cy, float base_line, float disparity_resolution);
      /** \brief Descturctor*/
      ~Parameters ();

      /**
       * \brief polymorphic clone method
       * \return deep copied Parameters of type StereoCameraModel::Parameters
       */
      SensorModel::Parameters* clone () const;
      
      /**
       * \brief set the shader parameters required for the model rendering
       * \param[in] renderer the renderer that holds the rendering shader.
       */
      void setRenderParameters (GLRenderer& renderer) const;
      
      /**
       * \brief set the shader parameters required for the mesh filtering
       * @param[in] renderer the renderer that holds the filtering shader
       */
      void setFilterParameters (GLRenderer& renderer) const;

      /**
       * \brief sets the camera parameters of the pinhole camera where the disparities were obtained. Usually the left camera
       * \param[in] fx focal length in x-direction
       * \param[in] fy focal length in y-direction
       * \param[in] cx x component of principal point
       * \param[in] cy y component of principal point
       */
      void setCameraParameters (float fx, float fy, float cx, float cy);
      
      /**
       * \brief sets the base line = distance of the two projective devices (camera, projector-camera)
       * \param[in] base_line the distance in meters
       */
      void setBaseline (float base_line);
      
      /**
       * \brief the quantization of disparity values in pixels. Usually 1/16th or 1/8th for OpenNI compatible devices
       * \param disparity_resolution
       */
      void setDisparityResolution (float disparity_resolution);
      
      /**
       * \brief returns the coefficients that are required for obtaining the padding for meshes
       * \return the padding coefficients
       */
      const Eigen::Vector3f& getPaddingCoefficients () const;
    private:
        /** \brief focal length in x-direction*/
        float fx_;
        
        /** \brief focal length in y-direction*/
        float fy_;
        
        /** \brief x component of principal point*/
        float cx_;
        
        /** \brief y component of principal point*/
        float cy_;
        
        /** \brief distance of the two projective devices that are used to determine the disparities*/
        float base_line_;
        
        /** \brief resolution/quantization of disparity values*/
        float disparity_resolution_;
        
        /** 
         * \brief padding coefficients
         * \note absolute padding in meters = coeff[0] * z^2 + coeff[1] * z + coeff[2]
         */
        const Eigen::Vector3f padding_coefficients_;
  };
  
  /** \brief predefined sensor model for OpenNI compatible devices*/
  static const StereoCameraModel::Parameters& RegisteredKinectParams;
  
  /** \brief source code of the vertex shader used to render the meshes*/
  static const std::string renderVertexShaderSource;
  
  /** \brief source code of the fragment shader used to render the meshes*/
  static const std::string renderFragmentShaderSource;
  
  /** \brief source code of the vertex shader used to filter the depth map*/
  static const std::string filterVertexShaderSource;
  
  /** \brief source code of the fragment shader used to filter the depth map*/
  static const std::string filterFragmentShaderSource;
};
} //namespace mesh_filter
#endif
