#include <mesh_filter/sensor_model.h>
#include <mesh_filter/utilities.h>
#include <stdexcept>

mesh_filter::SensorModel::~SensorModel ()
{
}

mesh_filter::SensorModel::Parameters::Parameters (unsigned width, unsigned height, float near_clipping_plane_distance, float far_clipping_plane_distance)
: width_ (width)
, height_ (height)
, far_clipping_plane_distance_ (far_clipping_plane_distance)
, near_clipping_plane_distance_ (near_clipping_plane_distance)
{
}

mesh_filter::SensorModel::Parameters::~Parameters ()
{
}

void mesh_filter::SensorModel::Parameters::setImageSize (unsigned width, unsigned height)
{
  width_ = width;
  height_ = height;
}

void mesh_filter::SensorModel::Parameters::setDepthRange (float near, float far)
{
  if (near <= 0)
    throw std::runtime_error ("Near clipping plane distance needs to be larger than zero!");
  
  if (far <= near)
    throw std::runtime_error ("Far clipping plane distance must be larger than the near clipping plane distance!");
  
  near_clipping_plane_distance_ = near;
  far_clipping_plane_distance_ = far;
}

unsigned mesh_filter::SensorModel::Parameters::getWidth () const
{
  return width_;
}

unsigned mesh_filter::SensorModel::Parameters::getHeight () const
{
  return height_;
}

float mesh_filter::SensorModel::Parameters::getNearClippingPlaneDistance () const
{
  return near_clipping_plane_distance_;
}

float mesh_filter::SensorModel::Parameters::getFarClippingPlaneDistance () const
{
  return far_clipping_plane_distance_;
}

void mesh_filter::SensorModel::Parameters::transformModelDepthToMetricDepth (float* depth) const
{
#if HAVE_SSE_EXTENSIONS
  const __m128 mmNear = _mm_set1_ps (near_clipping_plane_distance_);
  const __m128 mmFar = _mm_set1_ps (far_clipping_plane_distance_);
  const __m128 mmNF = _mm_mul_ps (mmNear, mmFar);
  const __m128 mmF_N = _mm_sub_ps (mmFar, mmNear);
  static const __m128 mmOnes = _mm_set1_ps (1);
  static const __m128 mmZeros = _mm_set1_ps (0);

  float* depthEnd = depth + width_ * height_;
  if (!isAligned16 (depth))
  {
    // first depth value without SSE until we reach aligned data
    unsigned first = 16 - alignment16 (depth);
    unsigned idx;
    const float near = near_clipping_plane_distance_;
    const float far = far_clipping_plane_distance_;
    const float nf = near * far;
    const float f_n = far - near;

    while (depth < depthEnd && idx++ < first)
      if (*depth != 0 && *depth != 1)
        *depth = nf / (far - *depth * f_n);
      else
        *depth = 0;
    
    //rest of unaligned data at the end
    unsigned last = (width_ * height_ - first) & 15;
    float* depth2 = depthEnd - last;
    while (depth2 < depthEnd)
      if (*depth2 != 0 && *depth2 != 1)
        *depth2 = nf / (far - *depth2 * f_n);
      else
        *depth2 = 0;
    
    depthEnd -= last;
  }
  
  const __m128* mmEnd = (__m128*) depthEnd;
  __m128* mmDepth = (__m128*) depth;
  // rest is aligned
  while (mmDepth < mmEnd)
  {
    __m128 mask = _mm_and_ps (_mm_cmpneq_ps (*mmDepth, mmOnes),  _mm_cmpneq_ps (*mmDepth, mmZeros));
    *mmDepth = _mm_mul_ps (*mmDepth, mmF_N);
    *mmDepth = _mm_sub_ps (mmFar, *mmDepth);
    *mmDepth = _mm_div_ps (mmNF, *mmDepth);
    *mmDepth = _mm_and_ps (*mmDepth, mask);
    ++mmDepth;
  }
  
#else
  // calculate metric depth values from OpenGL normalized depth buffer
  const float near = near_clipping_plane_distance_;
  const float far = far_clipping_plane_distance_;
  const float nf = near * far;
  const float f_n = far - near;
  
  const float* depthEnd = depth + width_ * height_;
  while (depth < depthEnd)
  {
    if (*depth != 0 && *depth != 1)
      *depth = nf / (far - *depth * f_n);
    else
      *depth = 0;
    
    ++depth;
  }
#endif
}

void mesh_filter::SensorModel::Parameters::transformFilteredDepthToMetricDepth (float* depth) const
{
#if HAVE_SSE_EXTENSIONS
  //* SSE version
  const __m128 mmNear = _mm_set1_ps (near_clipping_plane_distance_);
  const __m128 mmFar = _mm_set1_ps (far_clipping_plane_distance_);
  const __m128 mmScale = _mm_sub_ps (mmFar, mmNear);
  float *depthEnd = depth + width_ * height_;
  
  if (!isAligned16 (depth))
  {
    // first depth value without SSE until we reach aligned data
    unsigned first = 16 - alignment16 (depth);
    unsigned idx;
    const float scale = far_clipping_plane_distance_ - near_clipping_plane_distance_;
    const float offset = near_clipping_plane_distance_;
    while (depth < depthEnd && idx++ < first)
      if (*depth != 0 && *depth != 1.0)
        *depth = *depth * scale + offset;
      else
        *depth = 0;    
    
    //rest of unaligned data at the end
    unsigned last = (width_ * height_ - first) & 15;
    float* depth2 = depthEnd - last;
    while (depth2 < depthEnd)
      if (*depth2 != 0 && *depth != 1.0)
        *depth2 = *depth2 * scale + offset;
      else
        *depth2 = 0;    
    
    depthEnd -= last;
  }
  
  const __m128* mmEnd = (__m128*) depthEnd;
  __m128* mmDepth = (__m128*) depth;
  // rest is aligned
  while (mmDepth < mmEnd)
  {
    *mmDepth = _mm_mul_ps (*mmDepth, mmScale);
    *mmDepth = _mm_add_ps (*mmDepth, mmNear);
    *mmDepth = _mm_and_ps (*mmDepth, _mm_and_ps (_mm_cmpneq_ps (*mmDepth, mmNear), _mm_cmpneq_ps (*mmDepth, mmFar)));
    ++mmDepth;
  }
#else
  const float* depthEnd = depth + width_ * height_;
  const float scale = far_clipping_plane_distance_ - near_clipping_plane_distance_;
  const float offset = near_clipping_plane_distance_;
  while (depth < depthEnd)
  {
    // 0 = on near clipping plane -> we used 0 to mark invalid points -> not visible
    // points on far clipping plane needs to be removed too
    if (*depth != 0 && *depth != 1.0)
      *depth = *depth * scale + offset;
    else
      *depth = 0;
    
    ++depth;
  }
  #endif

}
