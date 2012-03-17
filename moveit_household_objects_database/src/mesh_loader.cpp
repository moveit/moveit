/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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
 *   * Neither the name of the Willow Garage nor the names of its
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
 *********************************************************************/

#include "mesh_loader.h"

#include <iostream>

namespace moveit_household_objects_database {

const char *elem_names[] = { /* list of the kinds of elements in the user's object */
  "vertex", "face"
};

PlyProperty vert_props[] = { /* list of property information for a vertex */
  {"x", Float32, Float32, offsetof(Vertex,x), 0, 0, 0, 0},
  {"y", Float32, Float32, offsetof(Vertex,y), 0, 0, 0, 0},
  {"z", Float32, Float32, offsetof(Vertex,z), 0, 0, 0, 0},
};

PlyProperty face_props[] = { /* list of property information for a face */
  {"vertex_indices", Int32, Int32, offsetof(Face,verts),
   1, Uint8, Uint8, offsetof(Face,nverts)},
};


inline void PLYModelLoader::endian_swap(void* p)
{
	unsigned int* x = (unsigned int*)p;

	*x = (*x>>24) |
			((*x<<8) & 0x00FF0000) |
			((*x>>8) & 0x0000FF00) |
			(*x<<24);
}


int PLYModelLoader::readFromFile(const string& filename, std::vector<double> &vertices, 
				 std::vector<int> &triangles)
{
	int i,j;
	int elem_count;
	char *elem_name;

	/*** Read in the original PLY object ***/
	vertices.clear();
	triangles.clear();

	FILE* fin = fopen(filename.c_str(), "rb");

	if (fin==NULL)  {
          std::cerr << "Cannot read file: " << filename << "\n";
		return -1;
	}

	in_ply = read_ply (fin);

	float version;
	int file_type;
	get_info_ply(in_ply, &version, &file_type);

	/* examine each element type that is in the file (vertex, face) */

	for (i = 0; i < in_ply->num_elem_types; i++) {

		/* prepare to read the i'th list of elements */
		elem_name = setup_element_read_ply (in_ply, i, &elem_count);

		if (equal_strings ((char*)"vertex", elem_name)) {
			nverts = elem_count;
			vertices.reserve(3*nverts);

			/* set up for getting vertex elements */
			/* (we want x,y,z) */

			setup_property_ply (in_ply, &vert_props[0]);
			setup_property_ply (in_ply, &vert_props[1]);
			setup_property_ply (in_ply, &vert_props[2]);

			/* we also want normal information if it is there (nx,ny,nz) */

			//		      for (j = 0; j < in_ply->elems[i]->nprops; j++) {
			//			PlyProperty *prop;
			//			prop = in_ply->elems[i]->props[j];
			//			if (equal_strings ("nx", prop->name)) {
			//			  setup_property_ply (in_ply, &vert_props[3]);
			//			  has_nx = 1;
			//			}
			//			if (equal_strings ("ny", prop->name)) {
			//			  setup_property_ply (in_ply, &vert_props[4]);
			//			  has_ny = 1;
			//			}
			//			if (equal_strings ("nz", prop->name)) {
			//			  setup_property_ply (in_ply, &vert_props[5]);
			//			  has_nz = 1;
			//			}
			//		      }

			/* also grab anything else that we don't need to know about */

			vert_other = get_other_properties_ply (in_ply,
					offsetof(Vertex,other_props));

			/* grab the vertex elements*/
			for (j = 0; j < elem_count; j++) {
				get_element_ply (in_ply, (void *) &vertex);
				if (file_type==PLY_BINARY_BE) {
					endian_swap(&vertex.x);
					endian_swap(&vertex.y);
					endian_swap(&vertex.z);
				}
				vertices.push_back( vertex.x );
				vertices.push_back( vertex.y );
				vertices.push_back( vertex.z );
			}
		}
		else if (equal_strings ((char*)"face", elem_name)) {

			nfaces = elem_count;
			triangles.reserve(nfaces*3);

			/* set up for getting face elements */
			/* (all we need are vertex indices) */

			setup_property_ply (in_ply, &face_props[0]);
			face_other = get_other_properties_ply (in_ply,
					offsetof(Face,other_props));

			/* grab all the face elements and place them in our list */

			for (j = 0; j < elem_count; j++) {
				get_element_ply (in_ply, (void *) &face);
				if (file_type==PLY_BINARY_BE) {
					for (int k=0;k<face.nverts;++k) {
						endian_swap(&face.verts[k]);
					}
				}


				if (face.nverts!=3) {
                                  std::cerr << "Mesh contains non triangle faces!\n";
				}
				else {
					triangles.push_back(face.verts[0]);
					triangles.push_back(face.verts[1]);
					triangles.push_back(face.verts[2]);
				}
			}
		}
		else  /* all non-vertex and non-face elements are grabbed here */
			get_other_element_ply (in_ply);
	}

	close_ply (in_ply);

	return 0;
}

}
