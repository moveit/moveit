/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage nor the names of its
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

/* Author: Ioan Sucan, Acorn Pooley, Sachin Chitta */

#ifndef MOVEIT_COLLISION_DETECTION_WORLD_
#define MOVEIT_COLLISION_DETECTION_WORLD_

#include <moveit/macros/class_forward.h>

#include <string>
#include <vector>
#include <set>
#include <map>
#include <memory>
#include <boost/function.hpp>
#include <Eigen/Geometry>
#include <eigen_stl_containers/eigen_stl_vector_container.h>

namespace shapes
{
MOVEIT_CLASS_FORWARD(Shape);
}

namespace collision_detection
{
MOVEIT_CLASS_FORWARD(World);

/** \brief Maintain a representation of the environment */
class World
{
public:
  /** \brief Constructor */
  World();

  /** \brief A copy constructor.
   * \e other should not be changed while the copy constructor is running
   * This does copy on write and should be quick. */
  World(const World& other);

  ~World();

  /**********************************************************************/
  /* Collision Bodies                                                   */
  /**********************************************************************/

  MOVEIT_CLASS_FORWARD(Object);

  /** \brief A representation of an object */
  struct Object
  {
    Object(const std::string& object_id) : id_(object_id)
    {
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /** \brief The object_id for this object */
    std::string id_;

    /** \brief All the shapes making up this object.
     *
     * The pose of each Shape is stored in the corresponding element of the shape_poses_ array.
     *
     * @note Although the code generally supports having multiple
     * shapes per object, there are many cases where it is better to
     * have only a single shape per object.  For instance
     * planning_scene::PlanningScene::getFrameTransform() will
     * return the pose of an Object.  As defined here, the pose of a
     * multi-shaped object is ambiguous, so getFrameTransform() just
     * returns the pose of the first Shape in the object. */
    std::vector<shapes::ShapeConstPtr> shapes_;

    /** \brief The poses of the corresponding entries in shapes_.
     *
     * @copydetails shapes_ */
    EigenSTL::vector_Affine3d shape_poses_;

    /** \brief Transforms to named frames on the object. Transforms are applied to the link.
     *  Use these to define points of interest on the object to plan with
     *  (e.g. screwdriver_tip, kettle_spout, mug_base).
     * */
    std::map<std::string, Eigen::Affine3d> named_frame_poses_;
  };

  /** \brief Get the list of Object ids */
  std::vector<std::string> getObjectIds() const;

  /** \brief Get a particular object */
  ObjectConstPtr getObject(const std::string& object_id) const;

  /** iterator over the objects in the world. */
  typedef std::map<std::string, ObjectPtr>::const_iterator const_iterator;
  /** iterator pointing to first change */
  const_iterator begin() const
  {
    return objects_.begin();
  }
  /** iterator pointing to end of changes */
  const_iterator end() const
  {
    return objects_.end();
  }
  /** number of changes stored */
  std::size_t size() const
  {
    return objects_.size();
  }
  /** find changes for a named object */
  const_iterator find(const std::string& object_id) const
  {
    return objects_.find(object_id);
  }

  /** \brief Check if a particular object exists in the collision world*/
  bool hasObject(const std::string& object_id) const;

  /** \brief Check if a frame or object with name object_id exists in the collision world*/
  bool knowsTransform(const std::string& object_id) const;

  /** \brief Get the transform to a frame or object with name object_id*/
  const Eigen::Affine3d& getTransform(const std::string& object_id) const;

  /** \brief Get the object id that owns the named frame with name object_id*/
  std::string getObjectOwningFrame(const std::string& frame_name) const;

  /** \brief Add shapes to an object in the map.
   * This function makes repeated calls to addToObjectInternal() to add the
   * shapes one by one.
   *  \note This function does NOT call the addToObject() variant that takes
   * a single shape and a single pose as input. */
  void addToObject(const std::string& id, const std::vector<shapes::ShapeConstPtr>& shapes,
                   const EigenSTL::vector_Affine3d& poses);

  /** \brief Add a shape to an object.
   * If the object already exists, this call will add the shape to the object
   * at the specified pose. Otherwise, the object is created and the
   * specified shape is added. This calls addToObjectInternal(). */
  void addToObject(const std::string& object_id, const shapes::ShapeConstPtr& shape, const Eigen::Affine3d& pose);

  /** \brief Update the pose of a shape in an object. Shape equality is
   * verified by comparing pointers. Returns true on success. */
  bool moveShapeInObject(const std::string& object_id, const shapes::ShapeConstPtr& shape, const Eigen::Affine3d& pose);

  /** \brief Move all shapes in an object according to the given transform specified in world frame */
  bool moveObject(const std::string& object_id, const Eigen::Affine3d& transform);

  /** \brief Replaces all shapes in an existing object.
   * If the object already exists, this call will assign the new shapes to the object
   * at the specified pose. Otherwise, the object is created and the
   * specified shape is added. This calls addToObjectInternal(). */
  bool replaceShapesInObject(const std::string& object_id, const std::vector<shapes::ShapeConstPtr>& shapes,
                             const EigenSTL::vector_Affine3d& poses);

  /** \brief Remove shape from object.
   * Shape equality is verified by comparing pointers. Ownership of the
   * object is renounced (i.e. object is deleted if no external references
   * exist) if this was the last shape in the object.
   * Returns true on success and false if the object did not exist or did not
   * contain the shape. */
  bool removeShapeFromObject(const std::string& object_id, const shapes::ShapeConstPtr& shape);

  /** \brief Remove a particular object.
   * If there are no external pointers to the corresponding instance of
   * Object, the memory is freed.
   * Returns true on success and false if no such object was found. */
  bool removeObject(const std::string& object_id);

  /** \brief Set named frames on an object. */
  bool setNamedFramesOfObject(const std::string& object_id, const std::map<std::string, Eigen::Affine3d>& named_frame_poses);

  /** \brief Clear all objects.
   * If there are no other pointers to corresponding instances of Objects,
   * the memory is freed. */
  void clearObjects();

  enum ActionBits
  {
    UNINITIALIZED = 0,
    CREATE = 1,        /** object was created */
    DESTROY = 2,       /** object was destroyed */
    MOVE_SHAPE = 4,    /** one or more shapes in object were moved */
    ADD_SHAPE = 8,     /** shape(s) were added to object */
    REMOVE_SHAPE = 16, /** shape(s) were removed from object */
  };

  /** \brief Represents an action that occurred on an object in the world.
   * Several bits may be set indicating several things happened to the object.
   * If the DESTROY bit is set, other bits will not be set. */
  class Action
  {
  public:
    Action() : action_(UNINITIALIZED)
    {
    }
    Action(int v) : action_(v)
    {
    }
    operator ActionBits() const
    {
      return ActionBits(action_);
    }

  private:
    int action_;
  };

private:
  class Observer;

public:
  class ObserverHandle
  {
  public:
    ObserverHandle() : observer_(NULL)
    {
    }

  private:
    ObserverHandle(const Observer* o) : observer_(o)
    {
    }
    const Observer* observer_;
    friend class World;
  };

  typedef boost::function<void(const ObjectConstPtr&, Action)> ObserverCallbackFn;

  /** \brief register a callback function for notification of changes.
   * \e callback will be called right after any change occurs to any Object.
   * \e observer is the object which is requesting the changes.  It is only
   * used for identifying the callback in removeObserver(). */
  ObserverHandle addObserver(const ObserverCallbackFn& callback);

  /** \brief remove a notifier callback */
  void removeObserver(const ObserverHandle observer_handle);

  /** send notification of change to all objects to a particular observer.
   * Used which switching from one world to another. */
  void notifyObserverAllObjects(const ObserverHandle observer_handle, Action action) const;

private:
  /** notify all observers of a change */
  void notify(const ObjectConstPtr&, Action);

  /** send notification of change to all objects. */
  void notifyAll(Action action);

  /** \brief Make sure that the object named \e object_id is known only to this
   * instance of the World. If the object is known outside of it, a
   * clone is made so that it can be safely modified later on. */
  void ensureUnique(ObjectPtr& obj);

  /* Add a shape with no checking */
  virtual void addToObjectInternal(const ObjectPtr& obj, const shapes::ShapeConstPtr& shape,
                                   const Eigen::Affine3d& pose);

  /** The objects maintained in the world */
  std::map<std::string, ObjectPtr> objects_;

  /* observers to call when something changes */
  class Observer
  {
  public:
    Observer(const ObserverCallbackFn& callback) : callback_(callback)
    {
    }
    ObserverCallbackFn callback_;
  };
  std::vector<Observer*> observers_;
};
}

#endif
