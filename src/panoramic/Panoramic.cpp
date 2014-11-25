#include <panoramic/Panoramic.hpp>

using namespace nurc;

Panoramic::Panoramic() :
  nh_()
{
  image_stitcher_ = nh_.advertiseService<SphericalStitchRequest, SphericalStitchResponse>("/stitch", boost::bind(&Panoramic::stitch, this, _1, _2));

}

Panoramic::~Panoramic()
{

}

bool Panoramic::stitch(SphericalStitchRequest& req, SphericalStitchResponse& res)
{
  // Algorithm
  // Select the first image to be the center
  // Warp each image
  // Incrementally add elements to the spherial warping
  //   Each time an element is added run an image alignment over it
  
  return true;
}

void Panoramic::generate_mapping_x()
{

}

void Panoramic::generate_mapping_y()
{

}

void Panoramic::generate_mapping()
{
  generate_mapping_x();
  generate_mapping_y();
}

void Panoramic::generate_spherical_stitching()
{

}
