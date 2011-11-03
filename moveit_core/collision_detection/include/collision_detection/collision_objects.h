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

/** \author Ioan Sucan, E. Gil Jones */

#ifndef COLLISION_DETECTION_COLLISION_OBJECTS_
#define COLLISION_DETECTION_COLLISION_OBJECTS_

#include <vector>
#include <string>
#include <map>

#include <geometric_shapes/shapes.h>
#include <LinearMath/btTransform.h>

namespace collision_detection
{
    
    /** \brief List of objects contained in the world (not including robot links) */
    class CollisionObjects
    {
    public:
	CollisionObjects(void)
	{
	}
	
	~CollisionObjects(void)
	{
	    clearObjects();
	}
	
	struct NamespaceObjects
	{
	    /** \brief An array of static shapes */
	    std::vector< shapes::StaticShape* > static_shape;
	    
	    /** \brief An array of shapes */
	    std::vector< shapes::Shape* >       shape;
	    
	    /** \brief An array of shape poses */
	    std::vector< btTransform >          shape_pose;
	};
	
	/** \brief Get the list of namespaces */
	std::vector<std::string> getNamespaces(void) const;
	
	/** \brief Get the list of objects */
	const NamespaceObjects& getObjects(const std::string &ns) const;
	
	/** \brief Get the list of objects */
	NamespaceObjects& getObjects(const std::string &ns);
	
	/** \brief Check if a particular namespace exists */
	bool haveNamespace(const std::string &ns) const;
	
	/** \brief Add a static object to the namespace. The user releases ownership of the object. */
	void addObject(const std::string &ns, shapes::StaticShape *shape);
	
	/** \brief Add an object to the namespace. The user releases ownership of the object. */
	void addObject(const std::string &ns, shapes::Shape *shape, const btTransform &pose);
	
	/** \brief Remove object. Object equality is verified by comparing pointers. Ownership of the object is renounced upon. Returns true on success. */
	bool removeObject(const std::string &ns, const shapes::Shape *shape);
	
	/** \brief Remove object. Object equality is verified by comparing pointers. Ownership of the object is renounced upon. Returns true on success. */
	bool removeObject(const std::string &ns, const shapes::StaticShape *shape);
	
	/** \brief Clear the objects in a specific namespace. Memory is freed. */
	void clearObjects(const std::string &ns);
	
	/** \brief Clear all objects. Memory is freed. */
	void clearObjects(void);
	
    private:
	
	std::map<std::string, NamespaceObjects> objects_;
    };
    
}

#endif

