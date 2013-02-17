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

/* Author: Ioan Sucan, Sachin Chitta, Acorn Pooley */

#ifndef MOVEIT_COLLISION_DETECTION_WORLD_
#define MOVEIT_COLLISION_DETECTION_WORLD_

#include <string>
#include <vector>
#include <map>
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>
#include <Eigen/Geometry>
#include <eigen_stl_containers/eigen_stl_vector_container.h>
#include <geometric_shapes/shapes.h>

namespace collision_detection
{

  /** \brief Maintain a representation of the environment */
  class World
  {
  public:
    
    /** \brief Constructor */
    World();
    
    /** \brief A copy constructor.
     * \e other should not be changed while the copy constructor is running */
    World(const World &other);
    
    /**********************************************************************/
    /* Collision Bodies                                                   */
    /**********************************************************************/
    
    /** \brief A representation of an object */
    struct Object
    {
      Object(const std::string &id) : id_(id) {}

      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      /** \brief The id for this object */
      std::string                         id_;
      
      /** \brief An array of shapes */
      std::vector<shapes::ShapeConstPtr> shapes_;
      
      /** \brief An array of shape poses */
      EigenSTL::vector_Affine3d           shape_poses_;
    };
    
    typedef boost::shared_ptr<Object> ObjectPtr;
    typedef boost::shared_ptr<Object> ObjectConstPtr;
    
    /** \brief Get the list of Object ids */
    std::vector<std::string> getObjectIds() const;
    
    const std::map<std::string, ObjectConstPtr>& getObjects() const
    {
      return  objects_;
    }

    /** \brief Get the number of objects in this collision world */
    std::size_t getObjectsCount() const
    {
      return objects_.size();
    }
    
    /** \brief Get a particular object */
    const ObjectConstPtr& getObject(const std::string &id) const;
    
    /** \brief Check if a particular object exists in the collision world*/
    bool hasObject(const std::string &id) const;
    
    /** \brief Add shapes to an object in the map.
     * This function makes repeated calls to addToObjectInternal() to add the
     * shapes one by one. 
     *  \note This function does NOT call the addToObject() variant that takes
     * a single shape and a single pose as input. */
    void addToObject(const std::string &id,
                             const std::vector<shapes::ShapeConstPtr> &shapes,
                             const EigenSTL::vector_Affine3d &poses);
    
    /** \brief Add a shape to an object.
     * If the object already exists, this call will add the shape to the object
     * at the specified pose. Otherwise, the object is created and the
     * specified shape is added. This calls addToObjectInternal(). */
    void addToObject(const std::string &id,
                             const shapes::ShapeConstPtr &shape,
                             const Eigen::Affine3d &pose);
    
    /** \brief Update the pose of a shape in an object. Shape equality is
     * verified by comparing pointers. Returns true on success. */
    bool moveShapeInObject(const std::string &id,
                                   const shapes::ShapeConstPtr &shape,
                                   const Eigen::Affine3d &pose);
    
    /** \brief Remove shape from object.
     * Shape equality is verified by comparing pointers. Ownership of the
     * object is renounced (i.e. object is deleted if no external references
     * exist) if this was the last shape in the object. 
     * Returns true on success and false if the object did not exist or did not
     * contain the shape. */
    bool removeShapeFromObject(const std::string &id,
                               const shapes::ShapeConstPtr &shape);
    
    /** \brief Remove a particular object.
     * If there are no external pointers to the corresponding instance of
     * Object, the memory is freed. */
    void removeObject(const std::string &id);
    
    /** \brief Clear all objects.
     * If there are no other pointers to corresponding instances of Objects,
     * the memory is freed. */
    void clearObjects();

    enum ActionBits
    {
      CREATE = 1,         /** object was created */
      DESTROY = 2,        /** object was destroyed */
      MOVE_SHAPE = 4,     /** one or more shapes in object were moved */
      ADD_SHAPE = 8,      /** shape(s) were added to object */
      REMOVE_SHAPE = 16,  /** shape(s) were removed from object */
    };
    class Action
    {
    public:
      Action() : action_(0) {}
      Action(int v) : action_(v) {}
      Action(const Action& v) : action_(v.action_) {}
      Action& operator=(int v) { action_ = v; return *this; }
      operator ActionBits() const { return ActionBits(action_); }
    private:
      int action_;
    };
  
    /** \brief register a callback function for notification of changes.
     * \e callback will be called right after any change occurs to any Object.
     * \e observer is the object which is requesting the changes.  It is only
     * used for identifying the callback in removeObserver(). */
    template<class ClientType>
    void addObserver(ClientType* observer,
                     void (*callback)(ClientType*, const ObjectConstPtr&, Action));

    /** \brief remove a notifier callback */
    template<class ClientType>
    void removeObserver(const ClientType* observer);

    /** send notification of change to all objects to a particular observer.
     * Used which switching from one world to another. */
    template<class ClientType>
    void notifyObserverAllObjects(const ClientType* observer, Action action);

  private:
    
    /** notify all observers of a change */
    void notify(const ObjectConstPtr&, Action);

    /** send notification of change to all objects. */
    void notifyAll(Action action);


    /** \brief Make sure that the object named \e id is known only to this
     * instance of the World. If the object is known outside of it, a
     * clone is made so that it can be safely modified later on. */
    void ensureUnique(ObjectPtr &obj);

    /* Add a shape with no checking */
    virtual void addToObjectInternal(const ObjectPtr &obj,
                                     const shapes::ShapeConstPtr &shape,
                                     const Eigen::Affine3d &pose);

    void removeObserverInternal(const void* observer);
    void notifyObserverAllObjectsInternal(const void* observer, Action action);


    /** The objects maintained in the world */
    std::map<std::string, ObjectPtr> objects_;

    /* observers to call when something changes */
    struct Observer
    {
      Observer(void* observer, void (*callback)(void*, const ObjectConstPtr&, Action)) :
        observer_(observer),
        callback_(callback)
      {}
      void* observer_;
      void (*callback_)(void*, const ObjectConstPtr&, Action);
    };
    std::vector<Observer> observers_;
  };
  
  typedef boost::shared_ptr<World> WorldPtr;
  typedef boost::shared_ptr<const World> WorldConstPtr;


  template<class ClientType> inline
  void collision_detection::World::addObserver(ClientType* observer,
                              void (*callback)(ClientType*, const ObjectConstPtr&, Action))
  {
    Observer o(observer, (void (*)(void*, const ObjectConstPtr&, Action))(callback));
    observers_.push_back(o);
  }

  template<class ClientType> inline
  void collision_detection::World::removeObserver(const ClientType* observer)
  {
    removeObserverInternal(observer);
  }

  template<class ClientType> inline
  void collision_detection::World::notifyObserverAllObjects(const ClientType* observer, Action action)
  {
    notifyObserverAllObjectsInternal(observer, action);
  }
}


#endif
